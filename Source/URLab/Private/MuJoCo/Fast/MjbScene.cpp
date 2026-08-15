// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. This plugin incorporates
// third-party software: MuJoCo (Apache 2.0). See ThirdPartyNotices.txt.

#include "MuJoCo/Fast/MjbScene.h"

#include "Components/StaticMeshComponent.h"
#include "Components/SceneComponent.h"
#include "ProceduralMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "MeshDescription.h"
#include "StaticMeshAttributes.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Engine/Texture2D.h"
#include "TextureResource.h"
#include "UObject/ConstructorHelpers.h"
#include "Engine/World.h"

#include "MuJoCo/Spec/MjAssetResolve.h"
#include "MuJoCo/Elements/MjCamera.h"
#include "MuJoCo/Capture/MjCameraTypes.h"
#include "MuJoCo/Utils/URLabAxisConv.h"
#include "Utils/URLabLogging.h"
#include "Bridge/MsgpackHelpers.h"
#include "Dom/JsonObject.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "Misc/FileHelper.h"
#include "Misc/Base64.h"

#include "zmq.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
// The engine primitives are 100 cm across / tall, so component scale 1 is a
// 50 cm half-extent. MJCF sizes are metres: scale = size(m) * 100 / 50.
constexpr double kSizeToScale = 2.0;
constexpr double kInfinitePlaneHalfM = 25.0; // size==0 plane -> 25 m half-extent

UStaticMesh* LoadBasic(const TCHAR* Path)
{
	return LoadObject<UStaticMesh>(nullptr, Path);
}

// Drives the transform-bus receive loop on a worker thread.
class FMjbBusRunnable : public FRunnable
{
public:
	explicit FMjbBusRunnable(AMjbScene* InScene) : Scene(InScene) {}
	virtual uint32 Run() override
	{
		if (Scene)
		{
			Scene->RunBusLoop();
		}
		return 0;
	}
	virtual void Stop() override
	{
		if (Scene)
		{
			Scene->SignalBusStop();
		}
	}

private:
	AMjbScene* Scene = nullptr;
};
} // namespace

AMjbScene::AMjbScene()
{
	PrimaryActorTick.bCanEverTick = true;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
}

void AMjbScene::BeginPlay()
{
	Super::BeginPlay();

	// The raw mjModel/mjData pointers were shallow-copied from the editor actor on
	// the PIE duplication and are stale -- clear them WITHOUT freeing (the editor
	// actor still owns its own). The transient index maps do not duplicate either.
	Model = nullptr;
	Data = nullptr;
	BodyActors.Reset();
	GeomComps.Reset();
	CameraComps.Reset();

	const bool bHaveModel = !MjbFilePath.IsEmpty() || MjbBytes.Num() > 0;
	UE_LOG(LogURLab, Log,
		TEXT("[MjbScene] BeginPlay (game=%d): file='%s' bytes=%d bus='%s' cameras=%d"),
		GetWorld() && GetWorld()->IsGameWorld(), *MjbFilePath, MjbBytes.Num(),
		*BusEndpoint, bEnableCameraStreaming);

	if (!bHaveModel)
	{
		UE_LOG(LogURLab, Error,
			TEXT("[MjbScene] BeginPlay: no MJB path or bytes -- nothing to stream (discovery/duplication issue?)"));
		return;
	}

	// Reuse the editor-world preview that duplicated into PIE: reload only the
	// (small) model for the index space and re-wire GeomComps to the already-built
	// components, instead of destroying every actor and rebuilding all the meshes
	// (the expensive part). Falls back to a full build when there is no preview to
	// reuse (e.g. spawned fresh in -game).
	int32 Geoms = ReindexFromLevel();
	if (Geoms > 0)
	{
		// Dynamic material instances made in the editor world do not survive the
		// PIE duplication, and the texture cache is transient -- re-apply materials
		// onto the reused components. Cheap next to a mesh rebuild.
		for (int32 G = 0; G < GeomComps.Num(); ++G)
		{
			if (GeomComps[G])
			{
				ApplyGeomMaterial(GeomComps[G], G);
			}
		}
		UE_LOG(LogURLab, Log, TEXT("[MjbScene] BeginPlay reused preview (%d geoms) -- no mesh rebuild"), Geoms);
	}
	else
	{
		TArray<AActor*> Inherited;
		GetAttachedActors(Inherited, /*bResetArray=*/true, /*bRecursivelyIncludeAttachedActors=*/true);
		for (AActor* A : Inherited)
		{
			if (A)
			{
				A->Destroy();
			}
		}
		BodyActors.Reset();
		GeomComps.Reset();
		CameraComps.Reset();
		Geoms = LoadAndBuild();
		if (Geoms < 0)
		{
			UE_LOG(LogURLab, Error, TEXT("[MjbScene] BeginPlay build FAILED (no usable MJB)"));
			return;
		}
	}

	// The camera components exist already -- built dormant in the preview (reused via
	// reindex) or by the fallback LoadAndBuild. Turn capture + streaming on for this
	// play session, so the render server goes live without rebuilding them.
	StartCameraStreaming();

	if (!BusEndpoint.IsEmpty())
	{
		StartBus();
	}
}

void AMjbScene::EndPlay(const EEndPlayReason::Type Reason)
{
	Teardown();
	Super::EndPlay(Reason);
}

void AMjbScene::BeginDestroy()
{
	// The editor can destroy/GC this actor without EndPlay; stop the worker
	// thread before its members are torn down to avoid a use-after-free.
	StopBus();
	Super::BeginDestroy();
}

void AMjbScene::Launch()
{
	LoadAndBuild();
	if (!BusEndpoint.IsEmpty())
	{
		StartBus();
	}
}

int32 AMjbScene::BuildStaticPreview()
{
	// Editor-world preview: build geometry at the rest pose and stop. No bus, no
	// tick -- the scene must never animate outside a play session. When the user
	// presses Play, the PIE duplicate of this actor connects the bus and streams.
	return LoadAndBuild();
}

int32 AMjbScene::ReindexFromLevel()
{
	// Need the model for sizing (nbody/ngeom) and the index space the stream uses.
	if (!LoadModelOnly())
	{
		return -1;
	}
	const int32 NBody = static_cast<int32>(Model->nbody);
	const int32 NGeom = static_cast<int32>(Model->ngeom);

	BodyActors.Reset();
	BodyActors.SetNum(NBody);
	GeomComps.Reset();
	GeomComps.SetNum(NGeom);
	CameraComps.Reset();
	CameraComps.SetNum(static_cast<int32>(Model->ncam));

	auto TagIndex = [](const FName& Tag, const TCHAR* Prefix) -> int32 {
		const FString S = Tag.ToString();
		const int32 PrefixLen = FCString::Strlen(Prefix);
		return S.StartsWith(Prefix) ? FCString::Atoi(*S.Mid(PrefixLen)) : -1;
	};

	// Recursive: the tagged geom components live on per-geom child actors, which
	// are grandchildren of this scene (scene -> body actor -> geom actor).
	TArray<AActor*> AttachedActors;
	GetAttachedActors(AttachedActors, /*bResetArray=*/true, /*bRecursivelyIncludeAttachedActors=*/true);
	int32 Found = 0;
	for (AActor* A : AttachedActors)
	{
		if (!A)
		{
			continue;
		}
		for (const FName& Tag : A->Tags)
		{
			const int32 BodyId = TagIndex(Tag, TEXT("MjbBody="));
			if (BodyActors.IsValidIndex(BodyId))
			{
				BodyActors[BodyId] = A;
			}
		}
		TArray<UPrimitiveComponent*> Comps;
		A->GetComponents<UPrimitiveComponent>(Comps);
		for (UPrimitiveComponent* C : Comps)
		{
			for (const FName& Tag : C->ComponentTags)
			{
				const int32 GeomId = TagIndex(Tag, TEXT("MjbGeom="));
				if (GeomComps.IsValidIndex(GeomId))
				{
					GeomComps[GeomId] = C;
					++Found;
				}
			}
		}
		// Cameras are UMjCamera scene components (not primitives); re-wire them too.
		TArray<UMjCamera*> Cams;
		A->GetComponents<UMjCamera>(Cams);
		for (UMjCamera* Cam : Cams)
		{
			for (const FName& Tag : Cam->ComponentTags)
			{
				const int32 CamId = TagIndex(Tag, TEXT("MjbCam="));
				if (CameraComps.IsValidIndex(CamId))
				{
					CameraComps[CamId] = Cam;
				}
			}
		}
	}
	UE_LOG(LogURLab, Log, TEXT("[MjbScene] re-indexed %d geoms across %d attached actors from the level"),
		Found, AttachedActors.Num());
	return Found > 0 ? Found : -1;
}

bool AMjbScene::FetchModelFromOwner(const FString& ControlEndpoint,
	TArray<uint8>& OutMjb, FString& OutBusEndpoint, FString& OutError)
{
	OutMjb.Reset();
	OutBusEndpoint.Empty();
	OutError.Empty();

	void* Ctx = zmq_ctx_new();
	void* Req = zmq_socket(Ctx, ZMQ_REQ);
	int Timeout = 5000;
	zmq_setsockopt(Req, ZMQ_RCVTIMEO, &Timeout, sizeof(Timeout));
	zmq_setsockopt(Req, ZMQ_SNDTIMEO, &Timeout, sizeof(Timeout));
	int Linger = 0;
	zmq_setsockopt(Req, ZMQ_LINGER, &Linger, sizeof(Linger));

	bool bOk = false;
	do
	{
		if (zmq_connect(Req, TCHAR_TO_UTF8(*ControlEndpoint)) != 0)
		{
			OutError = FString::Printf(TEXT("connect failed: %s"), *ControlEndpoint);
			break;
		}

		// Request: {op:"fastpath_hello"}. Both a Python owner and a UE live/direct
		// owner answer this with their MJB bytes and geoms-bus endpoint.
		TSharedPtr<FJsonObject> ReqObj = MakeShared<FJsonObject>();
		ReqObj->SetStringField(TEXT("op"), TEXT("fastpath_hello"));
		TArray<uint8> ReqBuf;
		FURLabMsgpackUtil::PackJsonObject(ReqObj, ReqBuf);
		if (zmq_send(Req, ReqBuf.GetData(), ReqBuf.Num(), 0) < 0)
		{
			OutError = TEXT("send failed");
			break;
		}

		zmq_msg_t Msg;
		zmq_msg_init(&Msg);
		if (zmq_msg_recv(&Msg, Req, 0) < 0)
		{
			zmq_msg_close(&Msg);
			OutError = TEXT("no reply (owner not answering fastpath_hello within timeout)");
			break;
		}
		TSharedPtr<FJsonObject> Reply;
		const bool bUnpacked = FURLabMsgpackUtil::UnpackToJsonObject(
			static_cast<const uint8*>(zmq_msg_data(&Msg)), static_cast<int32>(zmq_msg_size(&Msg)), Reply);
		zmq_msg_close(&Msg);
		if (!bUnpacked || !Reply.IsValid())
		{
			OutError = TEXT("reply was not msgpack");
			break;
		}

		// MJB bytes are msgpack bin, which unpacks to a base64 string under the
		// `__b64__`-suffixed key. Tolerate a plain base64 `mjb` string too.
		FString B64;
		if (!Reply->TryGetStringField(TEXT("mjb__b64__"), B64) || B64.IsEmpty())
		{
			Reply->TryGetStringField(TEXT("mjb"), B64);
		}
		if (B64.IsEmpty() || !FBase64::Decode(B64, OutMjb) || OutMjb.Num() == 0)
		{
			FString RemoteErr;
			Reply->TryGetStringField(TEXT("error"), RemoteErr);
			OutError = RemoteErr.IsEmpty() ? TEXT("reply carried no mjb") : RemoteErr;
			break;
		}
		Reply->TryGetStringField(TEXT("bus"), OutBusEndpoint);
		bOk = true;
	} while (false);

	if (Req)
	{
		zmq_close(Req);
	}
	if (Ctx)
	{
		zmq_ctx_term(Ctx);
	}
	return bOk;
}

bool AMjbScene::LoadModelOnly()
{
	if (Model)
	{
		return true; // already loaded
	}
	// Prefer an in-memory MJB (received over the wire) over a file path, so a
	// renderer never needs a shared file. Fall back to reading the file into a
	// buffer; either way we load from the buffer with mj_loadModelBuffer.
	TArray<uint8> LocalBytes;
	const TArray<uint8>* Bytes = nullptr;
	if (MjbBytes.Num() > 0)
	{
		Bytes = &MjbBytes;
	}
	else if (!MjbFilePath.IsEmpty() && FFileHelper::LoadFileToArray(LocalBytes, *MjbFilePath))
	{
		Bytes = &LocalBytes;
	}
	if (!Bytes || Bytes->Num() == 0)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjbScene] no MJB to load (bytes empty, file '%s' unreadable)"),
			*MjbFilePath);
		return false;
	}

	Model = mj_loadModelBuffer(Bytes->GetData(), Bytes->Num());
	if (!Model)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjbScene] mj_loadModelBuffer failed (%d bytes; version-mismatched MJB?)"),
			Bytes->Num());
		return false;
	}
	Data = mj_makeData(Model);
	if (!Data)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjbScene] mj_makeData failed"));
		Teardown();
		return false;
	}
	// One-shot forward for the rest pose (not stepping; the stream overrides it).
	mj_forward(Model, Data);

	Master = MjLoadMasterMaterial();
	if (!Master)
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjbScene] master material not found; geoms will be default-lit"));
	}
	return true;
}

int32 AMjbScene::LoadAndBuild()
{
	Teardown();
	if (!LoadModelOnly())
	{
		return -1;
	}

	BuildBodies();
	BuildGeoms();
	BuildCameras();
	ApplyFromData();

	UE_LOG(LogURLab, Log,
		TEXT("[MjbScene] built from %s: nbody=%d ngeom=%d (%d geom comps built, %d body actors)"),
		*MjbFilePath, (int)Model->nbody, (int)Model->ngeom, NumBuiltGeoms(), BodyActors.Num());
	return Model->ngeom;
}

int32 AMjbScene::NumBuiltGeoms() const
{
	int32 N = 0;
	for (const TObjectPtr<UPrimitiveComponent>& C : GeomComps)
	{
		if (C)
		{
			++N;
		}
	}
	return N;
}

void AMjbScene::BuildBodies()
{
	const int32 NBody = static_cast<int32>(Model->nbody);
	BodyActors.SetNum(NBody);
	// In the editor world the body actors are persistent (saveable, re-indexable
	// -- see ReindexFromLevel); in a play world they are transient scratch actors.
	const bool bEditorPreview = GetWorld() && !GetWorld()->IsGameWorld();
	for (int32 B = 0; B < NBody; ++B)
	{
		// One lightweight actor per body so the renderer culls per body. World
		// body (0) holds static geoms (floor); include it.
		FActorSpawnParameters Params;
		Params.Owner = this;
		if (!bEditorPreview)
		{
			Params.ObjectFlags |= RF_Transient;
		}
		AActor* Body = GetWorld()->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, Params);
		if (!Body)
		{
			continue;
		}
		USceneComponent* Root = NewObject<USceneComponent>(Body, *FString::Printf(TEXT("MjbBody_%d"), B));
		Body->SetRootComponent(Root);
		Root->RegisterComponent();
		Body->AttachToActor(this, FAttachmentTransformRules::KeepRelativeTransform);
		// Stable re-index key: the MuJoCo body id, plus the body name for a
		// readable outliner label. Lets a saved scene rebuild its body->actor map.
		Body->Tags.Add(FName(*FString::Printf(TEXT("MjbBody=%d"), B)));
		const char* Name = mj_id2name(Model, mjOBJ_BODY, B);
		if (bEditorPreview && Name && *Name)
		{
			Body->Tags.Add(FName(*FString::Printf(TEXT("MjbBodyName=%s"), ANSI_TO_TCHAR(Name))));
#if WITH_EDITOR
			Body->SetActorLabel(FString::Printf(TEXT("Mjb_%s"), ANSI_TO_TCHAR(Name)));
#endif
		}
		BodyActors[B] = Body;
	}
}

void AMjbScene::BuildGeoms()
{
	const int32 NGeom = static_cast<int32>(Model->ngeom);
	GeomComps.SetNum(NGeom);
	for (int32 G = 0; G < NGeom; ++G)
	{
		GeomComps[G] = BuildGeom(G);
		if (GeomComps[G])
		{
			// Stable re-index key for the geom -> component map.
			GeomComps[G]->ComponentTags.Add(FName(*FString::Printf(TEXT("MjbGeom=%d"), G)));
		}
	}
}

void AMjbScene::BuildCameras()
{
	if (!bEnableCameraStreaming || !Model || Model->ncam == 0)
	{
		return;
	}
	// Builds the camera components in BOTH the editor preview and a play session, so
	// the scene has one representation. The components are built DORMANT here (no
	// render target, no ZMQ bind); StartCameraStreaming() turns capture + streaming
	// on for a play session only, which is what keeps the editor preview cheap and
	// avoids the PIE duplicate double-binding the ports.
	const int32 NCam = static_cast<int32>(Model->ncam);
	CameraComps.SetNum(NCam);
	for (int32 C = 0; C < NCam; ++C)
	{
		// Host the camera under its MuJoCo body actor when it has one, else the
		// scene actor. Its world pose is driven explicitly each frame regardless.
		const int32 BodyId = Model->cam_bodyid[C];
		AActor* Host = (BodyActors.IsValidIndex(BodyId) && BodyActors[BodyId]) ? BodyActors[BodyId].Get() : this;

		UMjCamera* Cam = NewObject<UMjCamera>(Host);
		if (!Cam)
		{
			continue;
		}
		Cam->CaptureMode = EMjCameraMode::Real;
		// Resolution: MuJoCo leaves an unspecified camera at 1x1, so treat <=1 as
		// "use a sane default". Then optionally cap the height (each camera is a
		// full scene capture) keeping aspect.
		int32 W = Model->cam_resolution ? static_cast<int32>(Model->cam_resolution[2 * C]) : 0;
		int32 H = Model->cam_resolution ? static_cast<int32>(Model->cam_resolution[2 * C + 1]) : 0;
		if (W <= 1 || H <= 1)
		{
			W = 640;
			H = 480;
		}
		if (CameraMaxHeight > 0 && H > CameraMaxHeight)
		{
			W = FMath::Max(1, FMath::RoundToInt(W * (static_cast<double>(CameraMaxHeight) / H)));
			H = CameraMaxHeight;
		}
		TArray<int32> Res;
		Res.Add(W);
		Res.Add(H);
		Cam->SetResolution(Res);
		if (Model->cam_fovy[C] > 0.0)
		{
			Cam->SetFovy(Model->cam_fovy[C]);
		}

		// Transport: one ZMQ PUB per camera (port base + id); no manager here, so
		// the authored endpoint is used directly. Optional SHM ring alongside.
		Cam->bEnableZmqBroadcast = true;
		Cam->ZmqEndpoint = FString::Printf(TEXT("tcp://0.0.0.0:%d"), CameraStreamBasePort + C);
		Cam->bEnableShmBroadcast = bEnableCameraShm;
		Cam->SetStreamPortIndex(C);

		Cam->SetupAttachment(Host->GetRootComponent());
		Cam->ComponentTags.Add(FName(*FString::Printf(TEXT("MjbCam=%d"), C)));
		Cam->RegisterComponent();
		CameraComps[C] = Cam;
	}
	UE_LOG(LogURLab, Log, TEXT("[MjbScene] built %d camera component(s) (dormant)"), NCam);
}

void AMjbScene::StartCameraStreaming()
{
	// Turn dormant cameras into a live render server: set up the render target, bind
	// the per-camera ZMQ port, and capture every frame. Network config is re-applied
	// here so it is correct whether the cameras were just built or reused from the
	// duplicated preview. Play session only.
	for (int32 C = 0; C < CameraComps.Num(); ++C)
	{
		UMjCamera* Cam = CameraComps[C];
		if (!Cam)
		{
			continue;
		}
		Cam->bEnableZmqBroadcast = true;
		Cam->ZmqEndpoint = FString::Printf(TEXT("tcp://0.0.0.0:%d"), CameraStreamBasePort + C);
		Cam->bEnableShmBroadcast = bEnableCameraShm;
		Cam->SetStreamPortIndex(C);
		// Render every frame: with no AAMjManager the state-change capture gate never
		// advances, so it would render once then stall without this.
		Cam->SetCaptureRate(/*bOnStateChange=*/false, /*MaxFps=*/30.0f);
		Cam->SetStreamingEnabled(true);
	}
	if (CameraComps.Num() > 0)
	{
		UE_LOG(LogURLab, Log, TEXT("[MjbScene] camera server: %d camera(s) streaming from port %d"),
			CameraComps.Num(), CameraStreamBasePort);
	}
}

void AMjbScene::ApplyCameraPoses(const double* Cxpos, const double* Cxquat)
{
	for (int32 C = 0; C < CameraComps.Num(); ++C)
	{
		UMjCamera* Cam = CameraComps[C];
		if (!Cam)
		{
			continue;
		}
		FVector Loc;
		FQuat Rot;
		if (Cxpos && Cxquat)
		{
			// Streamed camera world transforms (wxyz), same convention as geoms.
			Loc = URLabAxisConv::MjPositionToUe(Cxpos + 3 * C);
			Rot = URLabAxisConv::MjQuatToUe(Cxquat + 4 * C);
		}
		else if (Data)
		{
			// Rest pose from this process's mjData (geom_xmat-style 3x3).
			double Q[4];
			mju_mat2Quat(Q, Data->cam_xmat + 9 * C);
			Loc = URLabAxisConv::MjPositionToUe(Data->cam_xpos + 3 * C);
			Rot = URLabAxisConv::MjQuatToUe(Q);
		}
		else
		{
			continue;
		}
		Cam->SetWorldLocationAndRotation(Loc, Rot);
	}
}

UPrimitiveComponent* AMjbScene::BuildGeom(int32 G)
{
	const int32 Type = Model->geom_type[G];
	const int32 BodyId = Model->geom_bodyid[G];
	const int32 MatId = Model->geom_matid[G];
	const double* Size = Model->geom_size + 3 * G;

	// Geom-group visibility: hide collision/other groups the mask excludes
	// (default shows 0-2). Matches MuJoCo's group-toggled visualization.
	const int32 Group = Model->geom_group[G];
	if (Group < 0 || Group > 30 || !(VisibleGroupMask & (1 << Group)))
	{
		return nullptr;
	}

	// A fully transparent geom is MJCF's "do not draw" (collision/inertial
	// proxies routinely carry rgba="0 0 0 0"). Honour it.
	const float* Rgba = (MatId >= 0) ? (Model->mat_rgba + 4 * MatId) : (Model->geom_rgba + 4 * G);
	if (Rgba[3] <= 0.0f)
	{
		return nullptr;
	}
	if (BodyId < 0 || BodyId >= BodyActors.Num() || !BodyActors[BodyId])
	{
		return nullptr;
	}

	// One actor per geom, attached under its body actor: every geom becomes a
	// named, individually selectable entry in the outliner (bare components on the
	// body actor are not surfaced there), and the per-geom transform stream drives
	// its mesh component directly.
	const bool bEditorPreview = GetWorld() && !GetWorld()->IsGameWorld();
	FActorSpawnParameters GeomParams;
	GeomParams.Owner = this;
	if (!bEditorPreview)
	{
		GeomParams.ObjectFlags |= RF_Transient;
	}
	AActor* GeomActor = GetWorld()->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, GeomParams);
	if (!GeomActor)
	{
		return nullptr;
	}
	USceneComponent* GeomRoot = NewObject<USceneComponent>(GeomActor, TEXT("GeomRoot"));
	GeomActor->SetRootComponent(GeomRoot);
	GeomRoot->RegisterComponent();
	GeomActor->AttachToActor(BodyActors[BodyId], FAttachmentTransformRules::KeepRelativeTransform);
	GeomActor->Tags.Add(FName(*FString::Printf(TEXT("MjbGeom=%d"), G)));
	{
		// Label by the geom's MJCF name, else its mesh name + id, else the id.
		const char* GeomName = mj_id2name(Model, mjOBJ_GEOM, G);
		FString Label;
		if (GeomName && *GeomName)
		{
			Label = ANSI_TO_TCHAR(GeomName);
		}
		else if (Model->geom_dataid[G] >= 0)
		{
			const char* MeshName = mj_id2name(Model, mjOBJ_MESH, Model->geom_dataid[G]);
			Label = FString::Printf(TEXT("%s_g%d"), MeshName ? ANSI_TO_TCHAR(MeshName) : TEXT("mesh"), G);
		}
		else
		{
			Label = FString::Printf(TEXT("geom_%d"), G);
		}
		GeomActor->Tags.Add(FName(*FString::Printf(TEXT("MjbGeomName=%s"), *Label)));
#if WITH_EDITOR
		if (bEditorPreview)
		{
			GeomActor->SetActorLabel(Label);
		}
#endif
	}
	// The geom actor is the parent for this geom's mesh component(s).
	AActor* Body = GeomActor;

	// Primitive selection + scale from MuJoCo size semantics. Mesh geoms are a
	// follow-on (ProceduralMeshComponent); skipped here with a note.
	const TCHAR* MeshPath = nullptr;
	FVector Scale(1, 1, 1);
	switch (Type)
	{
		case mjGEOM_PLANE:
		{
			MeshPath = TEXT("/Engine/BasicShapes/Plane.Plane");
			const double Hx = Size[0] > 0 ? Size[0] : kInfinitePlaneHalfM;
			const double Hy = Size[1] > 0 ? Size[1] : kInfinitePlaneHalfM;
			Scale = FVector(Hx * kSizeToScale, Hy * kSizeToScale, 1.0);
			break;
		}
		case mjGEOM_SPHERE:
			MeshPath = TEXT("/Engine/BasicShapes/Sphere.Sphere");
			Scale = FVector(Size[0], Size[0], Size[0]) * kSizeToScale;
			break;
		case mjGEOM_ELLIPSOID:
			MeshPath = TEXT("/Engine/BasicShapes/Sphere.Sphere");
			Scale = FVector(Size[0], Size[1], Size[2]) * kSizeToScale;
			break;
		case mjGEOM_CYLINDER:
			MeshPath = TEXT("/Engine/BasicShapes/Cylinder.Cylinder");
			Scale = FVector(Size[0], Size[0], Size[1]) * kSizeToScale;
			break;
		case mjGEOM_CAPSULE:
			// Cylinder shaft; the two rounded end caps are added as sphere child
			// components below (same as the authoring path). size[0]=radius,
			// size[1]=half-length of the cylinder part.
			MeshPath = TEXT("/Engine/BasicShapes/Cylinder.Cylinder");
			Scale = FVector(Size[0], Size[0], Size[1]) * kSizeToScale;
			break;
		case mjGEOM_BOX:
			MeshPath = TEXT("/Engine/BasicShapes/Cube.Cube");
			Scale = FVector(Size[0], Size[1], Size[2]) * kSizeToScale;
			break;
		case mjGEOM_MESH:
		{
#if WITH_EDITOR
			// Editor render server: a shared UStaticMesh (built once per mesh id)
			// referenced by pointer, so the PIE-world duplication stays cheap. Mesh
			// verts are already in UE units, so the component needs no extra scale.
			UStaticMesh* Mesh = GetOrBuildStaticMesh(Model->geom_dataid[G]);
			if (!Mesh)
			{
				return nullptr;
			}
			UStaticMeshComponent* Comp = NewObject<UStaticMeshComponent>(Body);
			Comp->SetStaticMesh(Mesh);
			Comp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
			Comp->RegisterComponent();
			Comp->AttachToComponent(Body->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
			ApplyGeomMaterial(Comp, G);
			return Comp;
#else
			// Packaged game: BuildFromMeshDescriptions is editor-only, so build a
			// ProceduralMeshComponent that generates its render data at runtime.
			UProceduralMeshComponent* Pmc = BuildMesh(G, Body);
			if (Pmc)
			{
				ApplyGeomMaterial(Pmc, G);
			}
			return Pmc;
#endif
		}
		default:
			return nullptr;
	}

	UStaticMesh* Mesh = LoadBasic(MeshPath);
	if (!Mesh)
	{
		return nullptr;
	}
	UStaticMeshComponent* Comp = NewObject<UStaticMeshComponent>(Body);
	Comp->SetStaticMesh(Mesh);
	Comp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Comp->SetRelativeScale3D(Scale);
	Comp->RegisterComponent();
	Comp->AttachToComponent(Body->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
	ApplyGeomMaterial(Comp, G);

	// Rounded capsule caps: a sphere at each end of the cylinder shaft, as child
	// components so they follow the shaft's streamed world transform. The base
	// cylinder/sphere meshes are 100 units, so the shaft's local half-height is 50
	// units; a cap sits there. The shaft's own non-uniform scale (r,r,halflen) is
	// cancelled on the cap's Z (1,1,r/halflen) so each cap stays a sphere of
	// radius r. Same construction as UMjGeom's VisualizerCap parts.
	if (Type == mjGEOM_CAPSULE)
	{
		if (UStaticMesh* SphereMesh = LoadBasic(TEXT("/Engine/BasicShapes/Sphere.Sphere")))
		{
			const double R = Size[0];
			const double HalfLen = Size[1] > KINDA_SMALL_NUMBER ? Size[1] : R;
			const FVector CapScale(1.0f, 1.0f, static_cast<float>(R / HalfLen));
			const double CapZ[2] = {50.0, -50.0};
			for (int32 S = 0; S < 2; ++S)
			{
				UStaticMeshComponent* Cap = NewObject<UStaticMeshComponent>(Body);
				Cap->SetStaticMesh(SphereMesh);
				Cap->SetCollisionEnabled(ECollisionEnabled::NoCollision);
				Cap->RegisterComponent();
				Cap->AttachToComponent(Comp, FAttachmentTransformRules::KeepRelativeTransform);
				Cap->SetRelativeLocation(FVector(0.0, 0.0, CapZ[S]));
				Cap->SetRelativeScale3D(CapScale);
				ApplyGeomMaterial(Cap, G);
			}
		}
	}
	return Comp;
}

void AMjbScene::BuildMeshArrays(int32 MeshId, TArray<FVector>& Verts, TArray<FVector>& Normals,
	TArray<FVector2D>& UVs, TArray<int32>& Tris)
{
	Verts.Reset();
	Normals.Reset();
	UVs.Reset();
	Tris.Reset();
	if (!Model || MeshId < 0 || MeshId >= static_cast<int32>(Model->nmesh))
	{
		return;
	}
	const int32 FaceAdr = Model->mesh_faceadr[MeshId];
	const int32 FaceNum = Model->mesh_facenum[MeshId];
	const bool bHasUV = Model->mesh_texcoordadr[MeshId] >= 0;
	// Face indices are LOCAL to each mesh (0-based); add the per-mesh base
	// addresses to reach this mesh's slice of the shared vert/normal/uv pools.
	const int32 VertAdr = Model->mesh_vertadr[MeshId];
	const int32 NormalAdr = Model->mesh_normaladr[MeshId];
	const int32 TexAdr = bHasUV ? Model->mesh_texcoordadr[MeshId] : 0;

	// Expand per face-corner (each corner its own vertex + normal + texcoord). The
	// static-mesh build later welds coincident positions while keeping the crease
	// normals; the procedural path uses the expansion directly.
	Verts.Reserve(FaceNum * 3);
	Normals.Reserve(FaceNum * 3);
	UVs.Reserve(FaceNum * 3);
	Tris.Reserve(FaceNum * 3);

	// Single-sided, one triangle per face. MuJoCo winds faces CCW-from-outside in
	// its right-handed frame; MjPositionToUe negates Y, a reflection that flips
	// the winding sense, so MuJoCo's own order (0,1,2) is the front-facing (outward)
	// order in Unreal. Keep the outward normal as-is. (An earlier reversed order
	// culled the visible faces -- the "see-through" holes -- and duplicating faces
	// to hide that introduced coplanar shadow acne / dark self-shadowing; a single
	// correctly-wound face is both hole-free and correctly lit.)
	// MuJoCo stores ONE averaged normal per vertex (mjCMesh::MakeNormal), so every
	// hard edge shades soft. Recompute per-corner normals with MuJoCo's own crease
	// threshold (acos(0.8)) -- the same split clean_meshes.py does on the import
	// path -- so box/extrusion edges stay sharp while cylinders stay round.
	// Faces meeting at a vertex are clustered greedily: a face joins the first group
	// whose running-mean normal it agrees with (dot >= 0.8), else it starts a group;
	// a corner's normal is its group's averaged normal.
	constexpr double kCreaseDot = 0.8;
	const int32 VertNum = static_cast<int32>(Model->mesh_vertnum[MeshId]);
	TArray<FVector> FaceGeoN;
	FaceGeoN.SetNumUninitialized(FaceNum);
	for (int32 F = 0; F < FaceNum; ++F)
	{
		const int32* FV = Model->mesh_face + 3 * (FaceAdr + F);
		const FVector P0 = URLabAxisConv::MjPositionToUe(Model->mesh_vert + 3 * (FV[0] + VertAdr));
		const FVector P1 = URLabAxisConv::MjPositionToUe(Model->mesh_vert + 3 * (FV[1] + VertAdr));
		const FVector P2 = URLabAxisConv::MjPositionToUe(Model->mesh_vert + 3 * (FV[2] + VertAdr));
		FVector Gn = FVector::CrossProduct(P1 - P0, P2 - P0).GetSafeNormal();
		// Align outward using MuJoCo's per-vertex normal (sign only); fall back to it
		// for a degenerate (zero-area) face.
		const int32* FN = Model->mesh_facenormal + 3 * (FaceAdr + F);
		const int32 Ni0 = FN[0] + NormalAdr;
		const double Nm[3] = {Model->mesh_normal[3 * Ni0], Model->mesh_normal[3 * Ni0 + 1],
			Model->mesh_normal[3 * Ni0 + 2]};
		const FVector Ref = URLabAxisConv::MjDirectionToUe(Nm).GetSafeNormal();
		if (Gn.IsNearlyZero())
		{
			Gn = Ref;
		}
		else if (FVector::DotProduct(Gn, Ref) < 0.0)
		{
			Gn = -Gn;
		}
		FaceGeoN[F] = Gn;
	}
	// Incident faces per local vertex.
	TArray<TArray<int32, TInlineAllocator<8>>> Incident;
	Incident.SetNum(FMath::Max(VertNum, 0));
	for (int32 F = 0; F < FaceNum; ++F)
	{
		const int32* FV = Model->mesh_face + 3 * (FaceAdr + F);
		for (int32 K = 0; K < 3; ++K)
		{
			if (FV[K] >= 0 && FV[K] < VertNum)
			{
				Incident[FV[K]].Add(F);
			}
		}
	}
	// Per-corner crease-averaged normal, indexed [3*F + K].
	TArray<FVector> CornerN;
	CornerN.SetNumUninitialized(FaceNum * 3);
	for (int32 V = 0; V < VertNum; ++V)
	{
		const TArray<int32, TInlineAllocator<8>>& Faces = Incident[V];
		if (Faces.Num() == 0)
		{
			continue;
		}
		TArray<FVector, TInlineAllocator<8>> GroupSum; // running summed normal per group
		TArray<int32, TInlineAllocator<16>> FaceGroup; // group index, parallel to Faces
		FaceGroup.SetNumUninitialized(Faces.Num());
		for (int32 i = 0; i < Faces.Num(); ++i)
		{
			const FVector Fn = FaceGeoN[Faces[i]];
			int32 GroupIdx = INDEX_NONE;
			for (int32 g = 0; g < GroupSum.Num(); ++g)
			{
				if (FVector::DotProduct(Fn, GroupSum[g].GetSafeNormal()) >= kCreaseDot)
				{
					GroupSum[g] += Fn;
					GroupIdx = g;
					break;
				}
			}
			FaceGroup[i] = (GroupIdx != INDEX_NONE) ? GroupIdx : GroupSum.Add(Fn);
		}
		for (int32 i = 0; i < Faces.Num(); ++i)
		{
			const int32 F = Faces[i];
			const FVector Gn = GroupSum[FaceGroup[i]].GetSafeNormal();
			const int32* FV = Model->mesh_face + 3 * (FaceAdr + F);
			for (int32 K = 0; K < 3; ++K)
			{
				if (FV[K] == V)
				{
					CornerN[3 * F + K] = Gn;
				}
			}
		}
	}

	const int32 Order[3] = {0, 1, 2};
	for (int32 F = 0; F < FaceNum; ++F)
	{
		const int32* FV = Model->mesh_face + 3 * (FaceAdr + F);
		const int32* FT = bHasUV ? Model->mesh_facetexcoord + 3 * (FaceAdr + F) : nullptr;
		for (int32 C = 0; C < 3; ++C)
		{
			const int32 K = Order[C];
			const int32 Vi = FV[K] + VertAdr;
			Verts.Add(URLabAxisConv::MjPositionToUe(Model->mesh_vert + 3 * Vi));
			Normals.Add(CornerN[3 * F + K].GetSafeNormal());
			if (FT)
			{
				const int32 Ti = FT[K] + TexAdr;
				// No V flip: MuJoCo stores tex_data bottom-row-first (OpenGL), and
				// GetOrBuildTexture uploads it row-0-first, so the texture is already
				// oriented to sample the raw MuJoCo texcoord directly. Flipping V here
				// (1 - v) double-flips and samples the wrong band of the atlas.
				UVs.Add(FVector2D(Model->mesh_texcoord[2 * Ti], Model->mesh_texcoord[2 * Ti + 1]));
			}
			else
			{
				UVs.Add(FVector2D::ZeroVector);
			}
			Tris.Add(Verts.Num() - 1);
		}
	}
}

#if WITH_EDITOR
UStaticMesh* AMjbScene::GetOrBuildStaticMesh(int32 MeshId)
{
	if (const TObjectPtr<UStaticMesh>* Found = StaticMeshCache.Find(MeshId))
	{
		return *Found;
	}
	TArray<FVector> Verts;
	TArray<FVector> Normals;
	TArray<FVector2D> UVs;
	TArray<int32> Tris;
	BuildMeshArrays(MeshId, Verts, Normals, UVs, Tris);
	if (Verts.Num() < 3 || Tris.Num() < 3)
	{
		return nullptr;
	}

	// UStaticMesh via a MeshDescription: the render build welds coincident positions
	// while splitting by our crease normals, so many geoms share one pointer-
	// referenced asset and the PIE-world duplication copies pointers, not verts.
	FMeshDescription MeshDesc;
	FStaticMeshAttributes Attrs(MeshDesc);
	Attrs.Register();
	Attrs.GetVertexInstanceUVs().SetNumChannels(1);
	TVertexAttributesRef<FVector3f> Positions = Attrs.GetVertexPositions();
	TVertexInstanceAttributesRef<FVector3f> InstNormals = Attrs.GetVertexInstanceNormals();
	TVertexInstanceAttributesRef<FVector2f> InstUVs = Attrs.GetVertexInstanceUVs();

	const int32 NumVerts = Verts.Num();
	MeshDesc.ReserveNewVertices(NumVerts);
	TArray<FVertexID> VertIDs;
	VertIDs.SetNumUninitialized(NumVerts);
	for (int32 v = 0; v < NumVerts; ++v)
	{
		VertIDs[v] = MeshDesc.CreateVertex();
		Positions[VertIDs[v]] = FVector3f(Verts[v]);
	}
	const FPolygonGroupID PolyGroup = MeshDesc.CreatePolygonGroup();
	MeshDesc.ReserveNewVertexInstances(Tris.Num());
	MeshDesc.ReserveNewPolygons(Tris.Num() / 3);
	for (int32 t = 0; t + 2 < Tris.Num(); t += 3)
	{
		FVertexInstanceID Inst[3];
		for (int32 K = 0; K < 3; ++K)
		{
			const int32 Vi = Tris[t + K];
			Inst[K] = MeshDesc.CreateVertexInstance(VertIDs[Vi]);
			InstNormals[Inst[K]] = FVector3f(Normals[Vi].GetSafeNormal());
			InstUVs.Set(Inst[K], 0, FVector2f(UVs[Vi]));
		}
		MeshDesc.CreatePolygon(PolyGroup, TArray<FVertexInstanceID>{Inst[0], Inst[1], Inst[2]});
	}

	UStaticMesh* Mesh = NewObject<UStaticMesh>(this, NAME_None, RF_Transient);
	Mesh->GetStaticMaterials().Add(FStaticMaterial());
	// No mesh distance field: these are puppet-render meshes (no Lumen GI/DFAO), and
	// the distance-field scene update ensure-spams on their transforms, stalling
	// ~2.5s per ensure (FDistanceFieldSceneData::UpdateDistanceFieldObjectBuffers).
	Mesh->bGenerateMeshDistanceField = false;
	UStaticMesh::FBuildMeshDescriptionsParams Params;
	Params.bBuildSimpleCollision = false;
	Params.bFastBuild = true;
	Mesh->NeverStream = true;
	Mesh->BuildFromMeshDescriptions({&MeshDesc}, Params);
	StaticMeshCache.Add(MeshId, Mesh);
	return Mesh;
}
#endif // WITH_EDITOR

UProceduralMeshComponent* AMjbScene::BuildMesh(int32 G, AActor* Body)
{
	TArray<FVector> Verts;
	TArray<FVector> Normals;
	TArray<FVector2D> UVs;
	TArray<int32> Tris;
	BuildMeshArrays(Model->geom_dataid[G], Verts, Normals, UVs, Tris);
	if (Verts.Num() < 3)
	{
		return nullptr;
	}

	// Per-face tangents from the UV gradient (the packaged-game path builds render
	// data at runtime, so it supplies them); assigned to all three corners.
	TArray<FProcMeshTangent> Tangents;
	Tangents.SetNum(Verts.Num());
	for (int32 t = 0; t + 2 < Tris.Num(); t += 3)
	{
		const int32 I0 = Tris[t], I1 = Tris[t + 1], I2 = Tris[t + 2];
		const FVector E1 = Verts[I1] - Verts[I0];
		const FVector E2 = Verts[I2] - Verts[I0];
		const FVector2D D1 = UVs[I1] - UVs[I0];
		const FVector2D D2 = UVs[I2] - UVs[I0];
		const double Det = D1.X * D2.Y - D2.X * D1.Y;
		FVector Tan = FMath::Abs(Det) > SMALL_NUMBER ? ((E1 * D2.Y - E2 * D1.Y) / Det) : E1;
		Tan = Tan.GetSafeNormal();
		if (Tan.IsNearlyZero())
		{
			Tan = FVector::ForwardVector;
		}
		Tangents[I0] = Tangents[I1] = Tangents[I2] = FProcMeshTangent(Tan, /*bFlipTangentY=*/false);
	}

	UProceduralMeshComponent* Pmc = NewObject<UProceduralMeshComponent>(Body);
	Pmc->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Pmc->RegisterComponent();
	Pmc->AttachToComponent(Body->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
	Pmc->CreateMeshSection(0, Verts, Tris, Normals, UVs, TArray<FColor>(), Tangents, /*bCreateCollision=*/false);
	return Pmc;
}

UTexture2D* AMjbScene::GetOrBuildTexture(int32 TexId, bool bSRGB)
{
	if (!Model || TexId < 0 || TexId >= static_cast<int32>(Model->ntex))
	{
		return nullptr;
	}
	if (const TObjectPtr<UTexture2D>* Found = TextureCache.Find(TexId))
	{
		return *Found;
	}
	const int32 W = Model->tex_width[TexId];
	const int32 H = Model->tex_height[TexId];
	const int32 NC = Model->tex_nchannel[TexId];
	if (W <= 0 || H <= 0 || NC < 1)
	{
		return nullptr;
	}
	const uint8* Src = Model->tex_data + Model->tex_adr[TexId];

	UTexture2D* Tex = UTexture2D::CreateTransient(W, H, PF_B8G8R8A8);
	if (!Tex)
	{
		return nullptr;
	}
	Tex->SRGB = bSRGB;
	FTexturePlatformData* PD = Tex->GetPlatformData();
	uint8* Dst = static_cast<uint8*>(PD->Mips[0].BulkData.Lock(LOCK_READ_WRITE));
	const int32 Pixels = W * H;
	for (int32 i = 0; i < Pixels; ++i)
	{
		uint8 R, Gc, B, A;
		if (NC >= 3)
		{
			R = Src[i * NC + 0];
			Gc = Src[i * NC + 1];
			B = Src[i * NC + 2];
			A = (NC >= 4) ? Src[i * NC + 3] : 255;
		}
		else
		{
			R = Gc = B = Src[i * NC]; // grayscale replicated
			A = 255;
		}
		Dst[i * 4 + 0] = B; // BGRA8
		Dst[i * 4 + 1] = Gc;
		Dst[i * 4 + 2] = R;
		Dst[i * 4 + 3] = A;
	}
	PD->Mips[0].BulkData.Unlock();
	Tex->UpdateResource();

	TextureCache.Add(TexId, Tex);
	return Tex;
}

void AMjbScene::ApplyGeomMaterial(UPrimitiveComponent* Comp, int32 G)
{
	if (!Master || !Comp)
	{
		return;
	}
	const int32 MatId = Model->geom_matid[G];
	const float* Rgba = (MatId >= 0) ? (Model->mat_rgba + 4 * MatId) : (Model->geom_rgba + 4 * G);

	UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(Master, Comp);
	if (!Mid)
	{
		return;
	}
	Comp->SetMaterial(0, Mid);
	Mid->SetVectorParameterValue(TEXT("BaseColor"), FLinearColor(Rgba[0], Rgba[1], Rgba[2], Rgba[3]));
	// Neutralise every texture slot the master declares first, so any role the
	// material does not fill samples a neutral (not the master's editor default).
	MjBindNeutralMaterialTextures(*Mid);

	// Bind the real MJB textures for the roles this material fills. mat_texid is
	// (nmat x mjNTEXROLE), role order matching EMjMaterialRole after the unused
	// USER slot (offset +1). Colour roles sample sRGB; data roles linear.
	if (MatId >= 0)
	{
		for (int32 R = 0; R < static_cast<int32>(EMjMaterialRole::Count); ++R)
		{
			const int32 TexId = Model->mat_texid[MatId * mjNTEXROLE + R + 1];
			if (TexId < 0)
			{
				continue;
			}
			const EMjMaterialRole MatRole = static_cast<EMjMaterialRole>(R);
			const bool bSRGB = (MatRole == EMjMaterialRole::Rgb || MatRole == EMjMaterialRole::Rgba
				|| MatRole == EMjMaterialRole::Emissive);
			if (UTexture2D* Tex = GetOrBuildTexture(TexId, bSRGB))
			{
				Mid->SetTextureParameterValue(MjMaterialRoleParameter(MatRole), Tex);
			}
		}
		Mid->SetScalarParameterValue(TEXT("TexRepeatU"), Model->mat_texrepeat[MatId * 2 + 0]);
		Mid->SetScalarParameterValue(TEXT("TexRepeatV"), Model->mat_texrepeat[MatId * 2 + 1]);
	}
	// PBR terms. MuJoCo stores metallic/roughness as -1 when "not specified", so
	// pushing the raw field makes a mirror-smooth, aliased surface. Map exactly
	// as the authoring path does (MjMetallicFor / MjRoughnessFor): metallic -1 ->
	// 0, roughness -1 -> 1 - shininess. This is what a material's look depends on.
	if (MatId >= 0)
	{
		const float RawMetal = Model->mat_metallic[MatId];
		const float RawRough = Model->mat_roughness[MatId];
		const float Metallic = FMath::Clamp(RawMetal >= 0.f ? RawMetal : 0.f, 0.f, 1.f);
		const float Roughness = FMath::Clamp(RawRough >= 0.f ? RawRough : 1.f - Model->mat_shininess[MatId], 0.f, 1.f);
		Mid->SetScalarParameterValue(TEXT("Metallic"), Metallic);
		Mid->SetScalarParameterValue(TEXT("Roughness"), Roughness);
		Mid->SetScalarParameterValue(TEXT("Specular"), FMath::Clamp(Model->mat_specular[MatId], 0.f, 1.f));
		Mid->SetScalarParameterValue(TEXT("Reflectance"), FMath::Clamp(Model->mat_reflectance[MatId], 0.f, 1.f));
		Mid->SetScalarParameterValue(TEXT("Emission"), FMath::Max(Model->mat_emission[MatId], 0.f));
	}
	else
	{
		// No material: MuJoCo draws a matte, non-metallic surface. Keep it matte
		// so bare meshes never come out shiny.
		Mid->SetScalarParameterValue(TEXT("Metallic"), 0.0f);
		Mid->SetScalarParameterValue(TEXT("Roughness"), 0.8f);
		Mid->SetScalarParameterValue(TEXT("Specular"), 0.5f);
		Mid->SetScalarParameterValue(TEXT("Reflectance"), 0.0f);
		Mid->SetScalarParameterValue(TEXT("Emission"), 0.0f);
	}
}

void AMjbScene::SendPerturbation(int32 BodyId, const FVector& ForceUE, const FVector& TorqueUE)
{
	if (OwnerControlEndpoint.IsEmpty() || BodyId < 0)
	{
		return;
	}
	double ForceMj[3];
	double TorqueMj[3];
	URLabAxisConv::UeDirectionToMj(ForceUE, ForceMj);
	URLabAxisConv::UeDirectionToMj(TorqueUE, TorqueMj);

	TSharedPtr<FJsonObject> Obj = MakeShared<FJsonObject>();
	Obj->SetStringField(TEXT("op"), TEXT("fastpath_perturb"));
	Obj->SetNumberField(TEXT("body"), BodyId);
	TArray<TSharedPtr<FJsonValue>> F, T;
	for (int32 i = 0; i < 3; ++i)
	{
		F.Add(MakeShared<FJsonValueNumber>(ForceMj[i]));
		T.Add(MakeShared<FJsonValueNumber>(TorqueMj[i]));
	}
	Obj->SetArrayField(TEXT("force"), F);
	Obj->SetArrayField(TEXT("torque"), T);
	TArray<uint8> Buf;
	FURLabMsgpackUtil::PackJsonObject(Obj, Buf);

	// Short-lived REQ; fire the request and read the ack so the REP stays in sync.
	void* Ctx = zmq_ctx_new();
	void* Req = zmq_socket(Ctx, ZMQ_REQ);
	int Timeout = 500;
	zmq_setsockopt(Req, ZMQ_RCVTIMEO, &Timeout, sizeof(Timeout));
	zmq_setsockopt(Req, ZMQ_SNDTIMEO, &Timeout, sizeof(Timeout));
	int Linger = 0;
	zmq_setsockopt(Req, ZMQ_LINGER, &Linger, sizeof(Linger));
	if (zmq_connect(Req, TCHAR_TO_UTF8(*OwnerControlEndpoint)) == 0)
	{
		zmq_send(Req, Buf.GetData(), Buf.Num(), 0);
		zmq_msg_t Ack;
		zmq_msg_init(&Ack);
		zmq_msg_recv(&Ack, Req, 0); // best-effort ack
		zmq_msg_close(&Ack);
	}
	zmq_close(Req);
	zmq_ctx_term(Ctx);
}

void AMjbScene::ApplyGeomTransforms(const double* Xpos, const double* Xquat)
{
	if (!Xpos || !Xquat)
	{
		return;
	}
	for (int32 G = 0; G < GeomComps.Num(); ++G)
	{
		UPrimitiveComponent* Comp = GeomComps[G];
		if (!Comp)
		{
			continue;
		}
		const FVector Loc = URLabAxisConv::MjPositionToUe(Xpos + 3 * G);
		const FQuat Rot = URLabAxisConv::MjQuatToUe(Xquat + 4 * G);
		Comp->SetWorldLocationAndRotation(Loc, Rot);
	}
}

void AMjbScene::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	// Streaming is a play-session behaviour only. In the editor world this actor
	// is a static, persistent preview and must never animate from the network or
	// the dev sweep -- authoring the level shouldn't mutate scene actors. (The
	// editor preview also never connects the bus, so this is belt-and-suspenders.)
	const UWorld* W = GetWorld();
	if (!W || !W->IsGameWorld())
	{
		return;
	}

	// A streamed frame takes priority: copy the newest raw payload out under the
	// lock, then decode + apply here on the game thread (UE components and all
	// UObject/TArray work must stay on the game thread).
	{
		TArray<uint8> Local;
		{
			FScopeLock Lock(&FrameMutex);
			if (bRxPending && RxBuf && RxSize > 0)
			{
				Local.SetNumUninitialized(RxSize);
				FMemory::Memcpy(Local.GetData(), RxBuf, RxSize);
				bRxPending = false;
			}
		}
		if (Local.Num() > 0)
		{
			TSharedPtr<FJsonObject> Obj;
			if (FURLabMsgpackUtil::UnpackToJsonObject(Local.GetData(), Local.Num(), Obj) && Obj.IsValid())
			{
				const int32 NGeom = Model ? static_cast<int32>(Model->ngeom) : 0;
				TArray<double> Xp;
				TArray<double> Xq;
				const TArray<TSharedPtr<FJsonValue>>* A = nullptr;
				if (Obj->TryGetArrayField(TEXT("xpos"), A) && A)
				{
					Xp.Reserve(A->Num());
					for (const TSharedPtr<FJsonValue>& V : *A)
					{
						Xp.Add(V.IsValid() ? V->AsNumber() : 0.0);
					}
				}
				if (Obj->TryGetArrayField(TEXT("xquat"), A) && A)
				{
					Xq.Reserve(A->Num());
					for (const TSharedPtr<FJsonValue>& V : *A)
					{
						Xq.Add(V.IsValid() ? V->AsNumber() : 0.0);
					}
				}
				if (NGeom > 0 && Xp.Num() == 3 * NGeom && Xq.Num() == 4 * NGeom)
				{
					ApplyGeomTransforms(Xp.GetData(), Xq.GetData());
				}

				// Optional camera world transforms, so streamed cameras track
				// moving bodies. Absent from owners that don't send them; cameras
				// then stay at their rest pose.
				if (CameraComps.Num() > 0)
				{
					const int32 NCam = CameraComps.Num();
					TArray<double> Cp, Cq;
					if (Obj->TryGetArrayField(TEXT("cxpos"), A) && A)
					{
						Cp.Reserve(A->Num());
						for (const TSharedPtr<FJsonValue>& V : *A)
						{
							Cp.Add(V.IsValid() ? V->AsNumber() : 0.0);
						}
					}
					if (Obj->TryGetArrayField(TEXT("cxquat"), A) && A)
					{
						Cq.Reserve(A->Num());
						for (const TSharedPtr<FJsonValue>& V : *A)
						{
							Cq.Add(V.IsValid() ? V->AsNumber() : 0.0);
						}
					}
					if (Cp.Num() == 3 * NCam && Cq.Num() == 4 * NCam)
					{
						ApplyCameraPoses(Cp.GetData(), Cq.GetData());
					}
				}
			}
			return;
		}
	}

	if (!bTestSweep || !Model || !Data || ZmqSub)
	{
		return;
	}
	// Owner-less dev sweep: animate joints locally and forward, so the builder
	// is exercised without an external owner. The real path applies a streamed
	// transform set and never touches Data.
	SweepTime += DeltaSeconds;
	for (int32 J = 0; J < Model->njnt; ++J)
	{
		const int32 Adr = Model->jnt_qposadr[J];
		switch (Model->jnt_type[J])
		{
			case mjJNT_HINGE:
			case mjJNT_SLIDE:
				Data->qpos[Adr] = Model->qpos0[Adr] + 0.4 * FMath::Sin(SweepTime * 1.5 + J);
				break;
			case mjJNT_FREE:
			{
				// Bob + spin the base so a single-free-body model still moves.
				Data->qpos[Adr + 2] = Model->qpos0[Adr + 2] + 0.25 * FMath::Sin(SweepTime * 1.2);
				const double Half = 0.5 * SweepTime;
				Data->qpos[Adr + 3] = FMath::Cos(Half);
				Data->qpos[Adr + 4] = 0.0;
				Data->qpos[Adr + 5] = 0.0;
				Data->qpos[Adr + 6] = FMath::Sin(Half);
				break;
			}
			default:
				break;
		}
	}
	mj_forward(Model, Data);
	ApplyFromData();
}

void AMjbScene::ApplyFromData()
{
	if (!Model || !Data)
	{
		return;
	}
	for (int32 G = 0; G < GeomComps.Num(); ++G)
	{
		UPrimitiveComponent* Comp = GeomComps[G];
		if (!Comp)
		{
			continue;
		}
		// mjData stores geom orientation as a 3x3 (geom_xmat); convert to wxyz.
		double Quat[4];
		mju_mat2Quat(Quat, Data->geom_xmat + 9 * G);
		const FVector Loc = URLabAxisConv::MjPositionToUe(Data->geom_xpos + 3 * G);
		const FQuat Rot = URLabAxisConv::MjQuatToUe(Quat);
		Comp->SetWorldLocationAndRotation(Loc, Rot);
	}
	ApplyCameraPoses(nullptr, nullptr); // rest pose from mjData
}

void AMjbScene::StartBus()
{
	if (BusEndpoint.IsEmpty() || ZmqSub)
	{
		return;
	}
	ZmqCtx = zmq_ctx_new();
	ZmqSub = zmq_socket(ZmqCtx, ZMQ_SUB);
	int Timeout = 200;
	zmq_setsockopt(ZmqSub, ZMQ_RCVTIMEO, &Timeout, sizeof(Timeout));
	int Linger = 0;
	zmq_setsockopt(ZmqSub, ZMQ_LINGER, &Linger, sizeof(Linger));
	int Hwm = 8;
	zmq_setsockopt(ZmqSub, ZMQ_RCVHWM, &Hwm, sizeof(Hwm));
	if (zmq_connect(ZmqSub, TCHAR_TO_UTF8(*BusEndpoint)) != 0)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjbScene] transform bus connect failed: %s"), *BusEndpoint);
		zmq_close(ZmqSub);
		ZmqSub = nullptr;
		zmq_ctx_term(ZmqCtx);
		ZmqCtx = nullptr;
		return;
	}
	zmq_setsockopt(ZmqSub, ZMQ_SUBSCRIBE, "geoms", 5);
	RxCap = 8 * 1024 * 1024; // ample for per-geom transform frames
	RxBuf = static_cast<uint8*>(FMemory::Malloc(RxCap));
	RxSize = 0;
	bRxPending = false;
	bBusStop = false;
	BusRunnable = new FMjbBusRunnable(this);
	BusThread = FRunnableThread::Create(BusRunnable, TEXT("MjbBusSub"));
	UE_LOG(LogURLab, Log, TEXT("[MjbScene] subscribing to transform bus %s"), *BusEndpoint);
}

void AMjbScene::StopBus()
{
	if (!ZmqSub && !ZmqCtx)
	{
		return;
	}
	bBusStop = true;
	if (BusThread)
	{
		BusThread->WaitForCompletion();
		delete BusThread;
		BusThread = nullptr;
	}
	delete BusRunnable;
	BusRunnable = nullptr;
	if (ZmqSub)
	{
		zmq_close(ZmqSub);
		ZmqSub = nullptr;
	}
	if (ZmqCtx)
	{
		zmq_ctx_term(ZmqCtx);
		ZmqCtx = nullptr;
	}
	if (RxBuf)
	{
		FMemory::Free(RxBuf);
		RxBuf = nullptr;
		RxCap = 0;
		RxSize = 0;
		bRxPending = false;
	}
}

void AMjbScene::RunBusLoop()
{
	// Worker thread does NO UE allocation and NO msgpack decode: it copies the
	// newest raw payload into the preallocated RxBuf under the lock and flags it.
	// The game thread (Tick) decodes + applies. This keeps all UObject / TArray /
	// FJsonObject work on the game thread.
	while (!bBusStop.load(std::memory_order_acquire))
	{
		bool bGot = false;
		while (true)
		{
			zmq_msg_t Topic;
			zmq_msg_init(&Topic);
			// Block (bounded by RCVTIMEO) on the first read of a batch, then
			// drain non-blocking to the newest.
			if (zmq_msg_recv(&Topic, ZmqSub, bGot ? ZMQ_DONTWAIT : 0) < 0)
			{
				zmq_msg_close(&Topic);
				break; // timeout / drained
			}
			int More = 0;
			size_t Ms = sizeof(More);
			zmq_getsockopt(ZmqSub, ZMQ_RCVMORE, &More, &Ms);
			zmq_msg_close(&Topic);
			if (!More)
			{
				continue;
			}
			zmq_msg_t Msg;
			zmq_msg_init(&Msg);
			if (zmq_msg_recv(&Msg, ZmqSub, 0) < 0)
			{
				zmq_msg_close(&Msg);
				break;
			}
			const int32 Sz = static_cast<int32>(zmq_msg_size(&Msg));
			if (Sz > 0 && Sz <= RxCap && RxBuf)
			{
				FScopeLock Lock(&FrameMutex);
				FMemory::Memcpy(RxBuf, zmq_msg_data(&Msg), Sz);
				RxSize = Sz;
				bRxPending = true;
			}
			zmq_msg_close(&Msg);
			bGot = true;
			bEverReceived.store(true, std::memory_order_release);
		}
	}
}

void AMjbScene::Teardown()
{
	StopBus();
	// Destroy body actors AND their per-geom child actors (Destroy does not cascade
	// to attached actors, so gather the whole attached tree first).
	TArray<AActor*> Attached;
	GetAttachedActors(Attached, /*bResetArray=*/true, /*bRecursivelyIncludeAttachedActors=*/true);
	for (AActor* A : Attached)
	{
		if (A)
		{
			A->Destroy();
		}
	}
	BodyActors.Reset();
	GeomComps.Reset();
	TextureCache.Reset();
	StaticMeshCache.Reset();
	if (Data)
	{
		mj_deleteData(Data);
		Data = nullptr;
	}
	if (Model)
	{
		mj_deleteModel(Model);
		Model = nullptr;
	}
}
