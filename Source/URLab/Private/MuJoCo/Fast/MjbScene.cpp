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
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/SceneComponent.h"
#include "ProceduralMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"

#include "Materials/MaterialInstanceDynamic.h"

#include "MuJoCo/Fast/MjbAssetBaker.h"
#include "MuJoCo/Fast/MjbTransportBus.h"
#include "MuJoCo/Entity/MjAppearance.h"
#include "MuJoCo/Entity/MjGeomAppearance.h"
#include "MuJoCo/Entity/MjBakedAssetResolver.h"
#include "MuJoCo/Elements/MjCamera.h"
#include "MuJoCo/Capture/MjCameraTypes.h"
#include "MuJoCo/Utils/URLabAxisConv.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Entity/MjAppearanceStore.h"
#include "Kismet/GameplayStatics.h"
#include "Camera/CameraActor.h"
#include "GameFramework/PlayerController.h"
#include "Engine/DirectionalLight.h"
#include "Engine/SkyLight.h"
#include "Engine/Engine.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/SkyLightComponent.h"
#include "Utils/URLabLogging.h"
#include "Bridge/MsgpackHelpers.h"
#include "Dom/JsonObject.h"
#include "Misc/FileHelper.h"
#include "Misc/Base64.h"
#include "Misc/SecureHash.h"

#include "zmq.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
// Re-index tag channel: a body/geom/instance/camera carries a "<prefix><id>" name
// tag so a saved level can map its components back to MuJoCo ids. One maker + one
// parser keep the writer and reader in lockstep -- no hardcoded prefix lengths.
constexpr const TCHAR* kTagBody = TEXT("MjbBody=");
constexpr const TCHAR* kTagGeom = TEXT("MjbGeom=");
constexpr const TCHAR* kTagIsm = TEXT("MjbIsm=");
constexpr const TCHAR* kTagCam = TEXT("MjbCam=");

FName MjbIdTag(const TCHAR* Prefix, int32 Id)
{
	return FName(*FString::Printf(TEXT("%s%d"), Prefix, Id));
}

int32 MjbParseIdTag(const FString& Tag, const TCHAR* Prefix)
{
	return Tag.StartsWith(Prefix) ? FCString::Atoi(*Tag.Mid(FCString::Strlen(Prefix))) : -1;
}

// Convert a MuJoCo 3x3 orientation (row-major geom_xmat/cam_xmat) to a UE quat,
// via a wxyz quaternion. Shared by every apply/build path that reads mjData mats.
FQuat MjMat3ToUeQuat(const double* Mat3)
{
	double Quat[4];
	mju_mat2Quat(Quat, Mat3);
	return URLabAxisConv::MjQuatToUe(Quat);
}

// Fast-path render components never contribute to distance-field lighting or AO.
// The shared engine primitive meshes (the Plane especially) carry a mesh distance
// field, and a flat/non-uniformly-scaled primitive yields a degenerate DF matrix
// that spams "InverseFast: NIL/non-invertible matrix" ensures every frame in -game.
// Dropping the component from the DF scene removes the cost and the noise.
void DisableDistanceFields(UPrimitiveComponent* Comp)
{
	if (Comp)
	{
		Comp->SetAffectDistanceFieldLighting(false);
	}
}

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
				AssetBaker->ApplyGeomMaterial(GeomComps[G], G);
			}
		}
		// Same for the instanced statics on the world body (their MID is re-applied
		// via the rep-geom stored in the MjbIsm tag).
		if (BodyActors.IsValidIndex(0) && BodyActors[0])
		{
			TArray<UInstancedStaticMeshComponent*> Isms;
			BodyActors[0]->GetComponents<UInstancedStaticMeshComponent>(Isms);
			for (UInstancedStaticMeshComponent* Ism : Isms)
			{
				for (const FName& Tag : Ism->ComponentTags)
				{
					const int32 RepG = MjbParseIdTag(Tag.ToString(), kTagIsm);
					if (Model && RepG >= 0 && RepG < static_cast<int32>(Model->ngeom))
					{
						AssetBaker->ApplyGeomMaterial(Ism, RepG);
					}
				}
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

	// Direct: step this scene's own model through the shared engine and render the
	// stepped state. Puppet (default): mirror an owner's transform stream.
	if (RunMode == EMjbRunMode::Direct)
	{
		Direct.Begin(*this);
	}
	else if (!BusEndpoint.IsEmpty())
	{
		// A puppet render slave mirrors an owner's stream but is still a render
		// server: stand up the manager (hence bridge + RPC) so an owner can push
		// live scene swaps (fastpath_load) to it over the wire.
		EnsureManager();
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
	// Editor-world preview actors never receive EndPlay (so Teardown never runs),
	// which is the only other place Model/Data are freed -- free them here so the
	// preview does not leak its mjModel/mjData. On a play-session actor Teardown
	// already ran and nulled them, so this is a no-op there; the PIE-duplicated
	// copy nulled its shallow-copied pointers in BeginPlay, so it is safe too.
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
			const int32 BodyId = MjbParseIdTag(Tag.ToString(), kTagBody);
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
				const int32 GeomId = MjbParseIdTag(Tag.ToString(), kTagGeom);
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
				const int32 CamId = MjbParseIdTag(Tag.ToString(), kTagCam);
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

	// Content id for the cached, persistent asset folder: a hash of the MJB bytes.
	// The compiled model IS the content, so an identical model hits the same cache.
	FString ContentHash;
	{
		uint8 Digest[20];
		FSHA1::HashBuffer(Bytes->GetData(), Bytes->Num(), Digest);
		ContentHash = BytesToHex(Digest, 20).Left(16);
	}

	// Prime the asset baker for this model (loads the shared master material).
	if (!AssetBaker)
	{
		AssetBaker = NewObject<UMjbAssetBaker>(this);
	}
	AssetBaker->Init(Model, ContentHash, bForceRebuildAssets);
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
		Body->Tags.Add(MjbIdTag(kTagBody, B));
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

	// Repeated static world-body geoms become instanced components; the rest are
	// built individually below. GeomComps stays null for instanced geoms -- they are
	// static, so the transform stream simply never touches them.
	TSet<int32> Instanced;
#if WITH_EDITOR
	BuildInstancedStatics(Instanced);
#endif

	for (int32 G = 0; G < NGeom; ++G)
	{
		if (Instanced.Contains(G))
		{
			continue;
		}
		GeomComps[G] = BuildGeom(G);
		if (GeomComps[G])
		{
			// Stable re-index key for the geom -> component map.
			GeomComps[G]->ComponentTags.Add(MjbIdTag(kTagGeom, G));
		}
	}
}

#if WITH_EDITOR
void AMjbScene::BuildInstancedStatics(TSet<int32>& OutHandled)
{
	if (!Model || !Data)
	{
		return;
	}
	// Only the static world body (id 0): its geoms never move, so instance transforms
	// are set once here from the rest pose.
	const int32 WorldBody = 0;
	if (!BodyActors.IsValidIndex(WorldBody) || !BodyActors[WorldBody])
	{
		return;
	}
	AActor* Host = BodyActors[WorldBody].Get();

	// Group visible world-body mesh geoms by (mesh id, material id) -- a group only
	// shares one instanced component if it shares both the mesh and the material.
	TMap<TPair<int32, int32>, TArray<int32>> Groups;
	for (int32 G = 0; G < static_cast<int32>(Model->ngeom); ++G)
	{
		if (Model->geom_bodyid[G] != WorldBody || Model->geom_type[G] != mjGEOM_MESH)
		{
			continue;
		}
		if (!IsGeomVisible(G))
		{
			continue;
		}
		const int32 MatId = Model->geom_matid[G];
		if (GeomRgba(G)[3] <= 0.0f)
		{
			continue;
		}
		const int32 MeshId = Model->geom_dataid[G];
		if (MeshId < 0)
		{
			continue;
		}
		Groups.FindOrAdd(TPair<int32, int32>(MeshId, MatId)).Add(G);
	}

	int32 NumGroups = 0;
	for (const TPair<TPair<int32, int32>, TArray<int32>>& KV : Groups)
	{
		const TArray<int32>& GeomIds = KV.Value;
		if (GeomIds.Num() < 2)
		{
			continue; // instancing only pays for a repeated mesh
		}
		UStaticMesh* Mesh = AssetBaker->GetOrBuildStaticMesh(KV.Key.Key);
		if (!Mesh)
		{
			continue;
		}
		UInstancedStaticMeshComponent* Ism = NewObject<UInstancedStaticMeshComponent>(Host);
		Ism->SetStaticMesh(Mesh);
		Ism->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		DisableDistanceFields(Ism);
		Ism->RegisterComponent();
		Ism->AttachToComponent(Host->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
		AssetBaker->ApplyGeomMaterial(Ism, GeomIds[0]); // the group shares one material
		// Rep-geom in the tag so a reused (PIE-duplicated) scene can re-apply the MID.
		Ism->ComponentTags.Add(MjbIdTag(kTagIsm, GeomIds[0]));
		for (int32 G : GeomIds)
		{
			const FVector Loc = URLabAxisConv::MjPositionToUe(Data->geom_xpos + 3 * G) + SceneOrigin;
			const FQuat Rot = MjMat3ToUeQuat(Data->geom_xmat + 9 * G);
			Ism->AddInstance(FTransform(Rot, Loc), /*bWorldSpace=*/true);
			OutHandled.Add(G);
		}
		++NumGroups;
	}
	if (NumGroups > 0)
	{
		UE_LOG(LogURLab, Log, TEXT("[MjbScene] instanced %d static geoms into %d ISM group(s)"),
			OutHandled.Num(), NumGroups);
	}
}
#endif // WITH_EDITOR

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
		Cam->ComponentTags.Add(MjbIdTag(kTagCam, C));
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
			Loc = URLabAxisConv::MjPositionToUe(Cxpos + 3 * C) + SceneOrigin;
			Rot = URLabAxisConv::MjQuatToUe(Cxquat + 4 * C);
		}
		else if (Data)
		{
			// Rest pose from this process's mjData (geom_xmat-style 3x3).
			Loc = URLabAxisConv::MjPositionToUe(Data->cam_xpos + 3 * C) + SceneOrigin;
			Rot = MjMat3ToUeQuat(Data->cam_xmat + 9 * C);
		}
		else
		{
			continue;
		}
		Cam->SetWorldLocationAndRotation(Loc, Rot);
	}
}

void AMjbScene::ApplyUserCamera(const double* Pos, const double* Fwd, const double* Up)
{
	UWorld* World = GetWorld();
	APlayerController* PC = World ? World->GetFirstPlayerController() : nullptr;
	if (!PC)
	{
		return;
	}
	// Lock onto a plain view camera once and keep driving THAT one. The player's
	// default view target can change across a live scene swap, so re-reading it
	// every frame silently drops the copycat; the cached actor (not attached to this
	// scene) survives the geometry rebuild. Re-acquire only if it was destroyed.
	ACameraActor* Cam = UserCam.Get();
	if (!Cam)
	{
		Cam = Cast<ACameraActor>(PC->GetViewTarget());
		if (!Cam)
		{
			return;
		}
		UserCam = Cam;
	}
	if (PC->GetViewTarget() != Cam)
	{
		PC->SetViewTargetWithBlend(Cam);
	}

	const FVector FwdUe = URLabAxisConv::MjDirectionToUe(Fwd).GetSafeNormal();
	const FVector UpUe = URLabAxisConv::MjDirectionToUe(Up).GetSafeNormal();
	if (FwdUe.IsNearlyZero())
	{
		return; // a degenerate frame would spin the view; keep the last good pose
	}
	// MuJoCo camera looks along its view direction with +Up; a UE camera looks down
	// +X with +Z up. MakeFromXZ builds that basis directly from the two vectors.
	const FVector LocUe = URLabAxisConv::MjPositionToUe(Pos) + SceneOrigin;
	const FQuat Rot = FRotationMatrix::MakeFromXZ(FwdUe, UpUe).ToQuat();
	Cam->SetActorLocationAndRotation(LocUe, Rot);
}

bool AMjbScene::IsGeomVisible(int32 G) const
{
	const int32 Group = Model->geom_group[G];
	return Group >= 0 && Group <= 30 && (VisibleGroupMask & (1 << Group)) != 0;
}

const float* AMjbScene::GeomRgba(int32 G) const
{
	const int32 MatId = Model->geom_matid[G];
	return (MatId >= 0) ? (Model->mat_rgba + 4 * MatId) : (Model->geom_rgba + 4 * G);
}

UPrimitiveComponent* AMjbScene::BuildGeom(int32 G)
{
	const int32 BodyId = Model->geom_bodyid[G];

	// Geom-group visibility: hide collision/other groups the mask excludes
	// (default shows 0-2). Matches MuJoCo's group-toggled visualization.
	if (!IsGeomVisible(G))
	{
		return nullptr;
	}

	// A fully transparent geom is MJCF's "do not draw" (collision/inertial
	// proxies routinely carry rgba="0 0 0 0"). Honour it.
	if (GeomRgba(G)[3] <= 0.0f)
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
	GeomActor->Tags.Add(MjbIdTag(kTagGeom, G));
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
	// The geom actor is the parent for this geom's mesh component(s). The primitive
	// (and mesh-vs-proc-mesh) build lives in the baked resolver, so the wire path and
	// a future imported path share one component-creation contract.
	FMjBakedAssetResolver Resolver(Model, AssetBaker);
	return Resolver.MakeGeomComponent(G, GeomActor);
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

int32 AMjbScene::NumGeomsNamed(FName GeomName) const
{
	if (!Model)
	{
		return 0;
	}
	const FString Want = GeomName.ToString();
	int32 Count = 0;
	for (int32 G = 0; G < GeomComps.Num(); ++G)
	{
		if (!GeomComps[G])
		{
			continue;
		}
		const char* Nm = mj_id2name(Model, mjOBJ_GEOM, G);
		if (Nm && *Nm && Want == ANSI_TO_TCHAR(Nm))
		{
			++Count;
		}
	}
	return Count;
}

int32 AMjbScene::ApplyAppearanceOverride(FName GeomName, const FMjGeomAppearance* Override,
	TFunctionRef<UTexture*(FName)> ResolveTexture)
{
	if (!Model)
	{
		return 0;
	}
	const FString Want = GeomName.ToString();
	int32 Applied = 0;
	for (int32 G = 0; G < GeomComps.Num(); ++G)
	{
		UPrimitiveComponent* Comp = GeomComps[G];
		if (!Comp)
		{
			continue;
		}
		const char* Nm = mj_id2name(Model, mjOBJ_GEOM, G);
		if (!Nm || !*Nm || Want != ANSI_TO_TCHAR(Nm))
		{
			continue;
		}

		if (Override != nullptr)
		{
			if (UMaterialInstanceDynamic* Mid = Cast<UMaterialInstanceDynamic>(Comp->GetMaterial(0)))
			{
				MjAppearance::Apply(Mid, *Override, ResolveTexture);
				++Applied;
			}
		}
		else if (AssetBaker)
		{
			// Restore the baked appearance by rebuilding this geom's MID from the model.
			AssetBaker->ApplyGeomMaterial(Comp, G);
			++Applied;
		}
	}
	return Applied;
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
		const FVector Loc = URLabAxisConv::MjPositionToUe(Xpos + 3 * G) + SceneOrigin;
		const FQuat Rot = URLabAxisConv::MjQuatToUe(Xquat + 4 * G);
		Comp->SetWorldLocationAndRotation(Loc, Rot);
	}
}

void AMjbScene::ApplyBodyTransforms(const double* Bxpos, const double* Bxquat)
{
	if (!Bxpos || !Bxquat || !Model)
	{
		return;
	}
	const int32 NBody = static_cast<int32>(Model->nbody);
	for (int32 G = 0; G < GeomComps.Num(); ++G)
	{
		UPrimitiveComponent* Comp = GeomComps[G];
		if (!Comp)
		{
			continue;
		}
		const int32 B = Model->geom_bodyid[G];
		if (B < 0 || B >= NBody)
		{
			continue;
		}
		// geom world = body world  ∘  geom-in-body offset (from the model). The
		// offset is constant, so the wire only carries the body transforms.
		double Rotated[3];
		double WorldPos[3];
		double WorldQuat[4];
		mju_rotVecQuat(Rotated, Model->geom_pos + 3 * G, Bxquat + 4 * B);
		WorldPos[0] = Bxpos[3 * B + 0] + Rotated[0];
		WorldPos[1] = Bxpos[3 * B + 1] + Rotated[1];
		WorldPos[2] = Bxpos[3 * B + 2] + Rotated[2];
		mju_mulQuat(WorldQuat, Bxquat + 4 * B, Model->geom_quat + 4 * G);
		const FVector Loc = URLabAxisConv::MjPositionToUe(WorldPos) + SceneOrigin;
		const FQuat Rot = URLabAxisConv::MjQuatToUe(WorldQuat);
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

	// Direct mode renders the engine's stepped state, not a streamed frame.
	if (RunMode == EMjbRunMode::Direct)
	{
		Direct.ApplyFromSnapshot(*this);
		return;
	}

	// A streamed frame takes priority: pull the newest raw payload from the bus,
	// then decode + apply here on the game thread (UE components and all
	// UObject/TArray work must stay on the game thread).
	{
		TArray<uint8> Local;
		if (TransportBus)
		{
			TransportBus->TakeLatestFrame(Local);
		}
		if (Local.Num() > 0)
		{
			TSharedPtr<FJsonObject> Obj;
			if (FURLabMsgpackUtil::UnpackToJsonObject(Local.GetData(), Local.Num(), Obj) && Obj.IsValid())
			{
				const int32 NGeom = Model ? static_cast<int32>(Model->ngeom) : 0;
				const int32 NBody = Model ? static_cast<int32>(Model->nbody) : 0;
				const TArray<TSharedPtr<FJsonValue>>* A = nullptr;
				auto ReadArr = [&](const TCHAR* Key, TArray<double>& Out) -> bool {
					if (Obj->TryGetArrayField(Key, A) && A)
					{
						Out.Reserve(A->Num());
						for (const TSharedPtr<FJsonValue>& V : *A)
						{
							Out.Add(V.IsValid() ? V->AsNumber() : 0.0);
						}
						return true;
					}
					return false;
				};

				// Prefer the per-body stream (fewer transforms, covers mocap); fall
				// back to a per-geom stream from a legacy owner.
				TArray<double> Bp;
				TArray<double> Bq;
				ReadArr(TEXT("bxpos"), Bp);
				ReadArr(TEXT("bxquat"), Bq);
				if (NBody > 0 && Bp.Num() == 3 * NBody && Bq.Num() == 4 * NBody)
				{
					ApplyBodyTransforms(Bp.GetData(), Bq.GetData());
				}
				else
				{
					TArray<double> Xp;
					TArray<double> Xq;
					ReadArr(TEXT("xpos"), Xp);
					ReadArr(TEXT("xquat"), Xq);
					if (NGeom > 0 && Xp.Num() == 3 * NGeom && Xq.Num() == 4 * NGeom)
					{
						ApplyGeomTransforms(Xp.GetData(), Xq.GetData());
					}
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

				// Optional free/user camera ("copycat"): the owner mirrors its
				// MuJoCo-viewer camera as eye position + forward + up, and this
				// render slave points its game viewport at the same view.
				{
					TArray<double> Ucp, Ucf, Ucu;
					if (ReadArr(TEXT("ucpos"), Ucp) && ReadArr(TEXT("ucfwd"), Ucf) &&
						ReadArr(TEXT("ucup"), Ucu) &&
						Ucp.Num() == 3 && Ucf.Num() == 3 && Ucu.Num() == 3)
					{
						ApplyUserCamera(Ucp.GetData(), Ucf.GetData(), Ucu.GetData());
					}
				}
			}
			return;
		}
	}

	if (!bTestSweep || !Model || !Data || (TransportBus && TransportBus->IsConnected()))
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
		// mjData stores geom orientation as a 3x3 (geom_xmat); convert to a UE quat.
		const FVector Loc = URLabAxisConv::MjPositionToUe(Data->geom_xpos + 3 * G) + SceneOrigin;
		const FQuat Rot = MjMat3ToUeQuat(Data->geom_xmat + 9 * G);
		Comp->SetWorldLocationAndRotation(Loc, Rot);
	}
	ApplyCameraPoses(nullptr, nullptr); // rest pose from mjData
}

AAMjManager* AMjbScene::EnsureManager()
{
	if (AAMjManager* Cached = Direct.Manager.Get())
	{
		return Cached;
	}
	// Find a manager already in the level (placed, or spawned by something else)
	// via a level scan rather than the Instance singleton, so a placed manager
	// that has not begun play yet is still found and we do not double-spawn.
	AAMjManager* Mgr = nullptr;
	{
		TArray<AActor*> Found;
		UGameplayStatics::GetAllActorsOfClass(GetWorld(), AAMjManager::StaticClass(), Found);
		if (Found.Num() > 0)
		{
			Mgr = Cast<AAMjManager>(Found[0]);
		}
	}
	if (!Mgr)
	{
		FActorSpawnParameters Params;
		Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		Mgr = GetWorld()->SpawnActor<AAMjManager>(AAMjManager::StaticClass(), Params);
		UE_LOG(LogURLab, Log, TEXT("[MjbScene] spawned a manager (bridge/RPC + stepping context)"));
	}
	Direct.Manager = Mgr;
	return Mgr;
}

void AMjbScene::InstallIntoEngine()
{
	// Timer target for the deferred Direct install; the logic lives in FMjbDirectMode.
	Direct.InstallIntoEngine(*this);
}

void AMjbScene::ReloadFromBytes(const TArray<uint8>& NewMjb)
{
	if (NewMjb.Num() == 0)
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjbScene] ReloadFromBytes: empty MJB, ignoring"));
		return;
	}

	// Retire the current model, shadow articulation and geometry, but KEEP the
	// manager + engine so the swap reuses the same physics + RPC context.
	AAMjManager* Mgr = Direct.Manager.Get();
	Direct.RetireForReload();

	// Destroy the current geometry tree (body actors + their per-geom child actors;
	// Destroy does not cascade to attached actors, so gather the whole tree first).
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
	CameraComps.Reset();
	if (AssetBaker)
	{
		AssetBaker->Reset();
	}
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

	// Rebuild from the new bytes. In Direct mode reinstall into the (retained)
	// engine; in Puppet mode reconnect the transform bus, which LoadAndBuild's
	// Teardown tore down (mismatched in-flight frames are skipped by the nbody guard).
	MjbBytes = NewMjb;
	MjbFilePath.Empty(); // bytes take precedence on the next build
	const int32 Geoms = LoadAndBuild();
	if (Geoms < 0)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjbScene] ReloadFromBytes: build failed for the new MJB"));
		return;
	}
	StartCameraStreaming();
	// LoadAndBuild -> Teardown -> StopBus dropped the subscription; bring it back so
	// the puppet keeps mirroring the owner after the swap.
	if (!BusEndpoint.IsEmpty())
	{
		StartBus();
	}
	if (RunMode == EMjbRunMode::Direct && Mgr)
	{
		// The manager has long since begun play, so install immediately (the timer
		// poll in Direct.Begin is only for the first-frame race at level start).
		Direct.Manager = Mgr;
		InstallIntoEngine();
	}

	// The swap built fresh MIDs, so any visual-DR overrides the client set on the
	// retired scene are gone; re-drive them onto the new components by name.
	if (Mgr)
	{
		Mgr->GetAppearanceStore()->ReapplyAll();
	}
	UE_LOG(LogURLab, Log, TEXT("[MjbScene] ReloadFromBytes: swapped model -- %d geoms built"), Geoms);
}

// Bring the render slave up at high quality with the noisy, temporally-accumulated
// post effects turned down: full scalability groups (Lumen GI / reflections / shadows
// at Epic so the final-gather grain converges), film grain and motion blur off. This
// is what makes a movable-light scene read clean instead of grainy. Overridable at
// runtime from the console (set URLAB_NO_RENDER_QUALITY=1 to skip).
static void ApplyRenderSlaveQuality()
{
	if (!GEngine || FParse::Param(FCommandLine::Get(), TEXT("URLabFastNoQuality")))
	{
		return;
	}
	static const TCHAR* const Cmds[] = {
		TEXT("sg.ViewDistanceQuality 4"), TEXT("sg.AntiAliasingQuality 4"),
		TEXT("sg.ShadowQuality 4"), TEXT("sg.GlobalIlluminationQuality 4"),
		TEXT("sg.ReflectionQuality 4"), TEXT("sg.PostProcessQuality 4"),
		TEXT("sg.TextureQuality 4"), TEXT("sg.EffectsQuality 4"),
		TEXT("sg.FoliageQuality 4"), TEXT("sg.ShadingQuality 4"),
		TEXT("r.FilmGrain 0"), TEXT("r.MotionBlurQuality 0"),
		TEXT("r.DefaultFeature.MotionBlur 0"),
	};
	for (const TCHAR* Cmd : Cmds)
	{
		GEngine->Exec(nullptr, Cmd);
	}
}

AMjbScene* AMjbScene::SpawnRenderSlave(UWorld* World, const TArray<uint8>& MjbBytes,
	const FString& MjbFilePath, const FString& BusEndpoint, const FVector& Origin,
	bool bDirect, bool bBaseLevel, bool bCameras)
{
	if (!World)
	{
		return nullptr;
	}
	ApplyRenderSlaveQuality();
	// Deferred spawn so the fields are set BEFORE BeginPlay runs; BeginPlay then
	// owns the whole build (geometry + camera streaming + Direct/bus connect).
	AMjbScene* Scene = World->SpawnActorDeferred<AMjbScene>(AMjbScene::StaticClass(), FTransform::Identity);
	if (!Scene)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjbScene] SpawnRenderSlave: failed to spawn AMjbScene"));
		return nullptr;
	}
	Scene->RunMode = bDirect ? EMjbRunMode::Direct : EMjbRunMode::Puppet;
	// Local dev sweep only when there is neither an owner bus nor Direct stepping.
	Scene->bTestSweep = BusEndpoint.IsEmpty() && !bDirect;
	Scene->MjbBytes = MjbBytes;
	Scene->MjbFilePath = MjbFilePath;
	Scene->BusEndpoint = BusEndpoint;
	Scene->bEnableCameraStreaming = bCameras;
	Scene->SceneOrigin = Origin;
	UGameplayStatics::FinishSpawningActor(Scene, FTransform::Identity);

	// A bare boot map has no lighting, so give the scene its own movable rig unless
	// it was dropped into a curated base level that brings its own.
	if (!bBaseLevel)
	{
		const FTransform SunXf(FRotator(-46.0, -60.0, 0.0), FVector::ZeroVector);
		if (ADirectionalLight* Sun =
				World->SpawnActor<ADirectionalLight>(ADirectionalLight::StaticClass(), SunXf))
		{
			if (ULightComponent* L = Sun->GetLightComponent())
			{
				L->SetMobility(EComponentMobility::Movable);
			}
		}
		const FTransform FillXf(FRotator(-18.0, 120.0, 0.0), FVector::ZeroVector);
		if (ADirectionalLight* Fill =
				World->SpawnActor<ADirectionalLight>(ADirectionalLight::StaticClass(), FillXf))
		{
			if (ULightComponent* L = Fill->GetLightComponent())
			{
				L->SetMobility(EComponentMobility::Movable);
				L->SetIntensity(0.4f * L->Intensity);
				L->SetLightColor(FLinearColor(0.7f, 0.75f, 0.9f));
				L->SetCastShadows(false);
			}
		}
		if (ASkyLight* Sky = World->SpawnActor<ASkyLight>(ASkyLight::StaticClass()))
		{
			if (USkyLightComponent* SkyComp = Sky->GetLightComponent())
			{
				SkyComp->SetMobility(EComponentMobility::Movable);
			}
		}
	}

	// Framing camera at the scene origin (the copycat retargets it once an owner
	// streams its free camera).
	if (APlayerController* PC = World->GetFirstPlayerController())
	{
		const FTransform View(FRotator(-18.0, 0.0, 0.0), FVector(-450.0, 0.0, 190.0) + Origin);
		if (ACameraActor* Cam = World->SpawnActor<ACameraActor>(ACameraActor::StaticClass(), View))
		{
			PC->SetViewTargetWithBlend(Cam);
		}
	}

	UE_LOG(LogURLab, Log,
		TEXT("[MjbScene] SpawnRenderSlave: mode=%s bus=%s baseLevel=%d cameras=%d bytes=%d origin=(%s)"),
		bDirect ? TEXT("direct") : TEXT("puppet"),
		BusEndpoint.IsEmpty() ? TEXT("(none)") : *BusEndpoint, bBaseLevel ? 1 : 0,
		bCameras ? 1 : 0, MjbBytes.Num(), *Origin.ToString());
	return Scene;
}

void AMjbScene::StartBus()
{
	if (BusEndpoint.IsEmpty())
	{
		return;
	}
	if (!TransportBus)
	{
		TransportBus = NewObject<UMjbTransportBus>(this);
	}
	TransportBus->Start(BusEndpoint);
}

void AMjbScene::StopBus()
{
	if (TransportBus)
	{
		TransportBus->Stop();
	}
}

bool AMjbScene::HasReceivedFrame() const
{
	return TransportBus ? TransportBus->HasEverReceived() : false;
}

void AMjbScene::Teardown()
{
	StopBus();
	// Direct mode aliased our raw model+data into the shared engine. Stop-join the
	// physics worker and unalias (and clear the deferred-install timer) BEFORE the
	// deletes below, so the worker is never mid-step against memory we are about to
	// free.
	Direct.Teardown(*this);
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
	if (AssetBaker)
	{
		AssetBaker->Reset();
	}
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
