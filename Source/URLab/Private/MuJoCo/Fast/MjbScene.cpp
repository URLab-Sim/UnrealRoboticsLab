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
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Engine/Texture2D.h"
#include "TextureResource.h"
#include "UObject/ConstructorHelpers.h"
#include "Engine/World.h"

#include "MuJoCo/Spec/MjAssetResolve.h"
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

	// Streaming only ever happens in a running world (PIE / -game). When this
	// actor is the PIE duplicate of the editor-world preview, its raw mjModel /
	// mjData pointers and its (transient, non-duplicated) child-actor references
	// were shallow-copied and are stale -- clear them WITHOUT freeing (the editor
	// actor still owns its own), so LoadAndBuild rebuilds a clean scene in this
	// world instead of tearing down the editor's model.
	Model = nullptr;
	Data = nullptr;
	BodyActors.Reset();
	GeomComps.Reset();

	if (!MjbFilePath.IsEmpty())
	{
		LoadAndBuild();
	}
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

int32 AMjbScene::LoadAndBuild()
{
	Teardown();

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
		return -1;
	}

	Model = mj_loadModelBuffer(Bytes->GetData(), Bytes->Num());
	if (!Model)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjbScene] mj_loadModelBuffer failed (%d bytes; version-mismatched MJB?)"),
			Bytes->Num());
		return -1;
	}
	Data = mj_makeData(Model);
	if (!Data)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjbScene] mj_makeData failed"));
		Teardown();
		return -1;
	}
	// One-shot forward for the rest pose (not stepping; the stream overrides it).
	mj_forward(Model, Data);

	Master = MjLoadMasterMaterial();
	if (!Master)
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjbScene] master material not found; geoms will be default-lit"));
	}

	BuildBodies();
	BuildGeoms();
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
	for (int32 B = 0; B < NBody; ++B)
	{
		// One lightweight actor per body so the renderer culls per body. World
		// body (0) holds static geoms (floor); include it.
		FActorSpawnParameters Params;
		Params.Owner = this;
		Params.ObjectFlags |= RF_Transient;
		AActor* Body = GetWorld()->SpawnActor<AActor>(AActor::StaticClass(), FTransform::Identity, Params);
		if (!Body)
		{
			continue;
		}
		USceneComponent* Root = NewObject<USceneComponent>(Body, *FString::Printf(TEXT("MjbBody_%d"), B));
		Body->SetRootComponent(Root);
		Root->RegisterComponent();
		Body->AttachToActor(this, FAttachmentTransformRules::KeepRelativeTransform);
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
	AActor* Body = BodyActors[BodyId];

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
			UProceduralMeshComponent* Pmc = BuildMesh(G, Body);
			if (Pmc)
			{
				ApplyGeomMaterial(Pmc, G);
			}
			return Pmc;
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

UProceduralMeshComponent* AMjbScene::BuildMesh(int32 G, AActor* Body)
{
	const int32 MeshId = Model->geom_dataid[G];
	if (MeshId < 0 || MeshId >= static_cast<int32>(Model->nmesh))
	{
		return nullptr;
	}
	const int32 FaceAdr = Model->mesh_faceadr[MeshId];
	const int32 FaceNum = Model->mesh_facenum[MeshId];
	const bool bHasUV = Model->mesh_texcoordadr[MeshId] >= 0;
	// Face indices are LOCAL to each mesh (0-based); add the per-mesh base
	// addresses to reach this mesh's slice of the shared vert/normal/uv pools.
	const int32 VertAdr = Model->mesh_vertadr[MeshId];
	const int32 NormalAdr = Model->mesh_normaladr[MeshId];
	const int32 TexAdr = bHasUV ? Model->mesh_texcoordadr[MeshId] : 0;

	// Expand per face-corner so MuJoCo's split vertex/normal/texcoord pools (the
	// hard-edge rule) are preserved: each corner gets its own vertex carrying the
	// face's own normal + texcoord index.
	TArray<FVector> Verts;
	TArray<FVector> Normals;
	TArray<FVector2D> UVs;
	TArray<int32> Tris;
	TArray<FProcMeshTangent> Tangents;
	Verts.Reserve(FaceNum * 3);
	Normals.Reserve(FaceNum * 3);
	UVs.Reserve(FaceNum * 3);
	Tris.Reserve(FaceNum * 3);
	Tangents.Reserve(FaceNum * 3);

	// Single-sided, one triangle per face. MuJoCo winds faces CCW-from-outside in
	// its right-handed frame; MjPositionToUe negates Y, a reflection that flips
	// the winding sense, so MuJoCo's own order (0,1,2) is the front-facing (outward)
	// order in Unreal. Keep the outward normal as-is. (An earlier reversed order
	// culled the visible faces -- the "see-through" holes -- and duplicating faces
	// to hide that introduced coplanar shadow acne / dark self-shadowing; a single
	// correctly-wound face is both hole-free and correctly lit.)
	const int32 Order[3] = {0, 1, 2};
	for (int32 F = 0; F < FaceNum; ++F)
	{
		const int32* FV = Model->mesh_face + 3 * (FaceAdr + F);
		const int32* FN = Model->mesh_facenormal + 3 * (FaceAdr + F);
		const int32* FT = bHasUV ? Model->mesh_facetexcoord + 3 * (FaceAdr + F) : nullptr;
		const int32 Base = Verts.Num();
		for (int32 C = 0; C < 3; ++C)
		{
			const int32 K = Order[C];
			const int32 Vi = FV[K] + VertAdr;
			const int32 Ni = FN[K] + NormalAdr;
			Verts.Add(URLabAxisConv::MjPositionToUe(Model->mesh_vert + 3 * Vi));
			const double N[3] = {Model->mesh_normal[3 * Ni], Model->mesh_normal[3 * Ni + 1],
				Model->mesh_normal[3 * Ni + 2]};
			Normals.Add(URLabAxisConv::MjDirectionToUe(N).GetSafeNormal());
			if (FT)
			{
				const int32 Ti = FT[K] + TexAdr;
				UVs.Add(FVector2D(Model->mesh_texcoord[2 * Ti], 1.0f - Model->mesh_texcoord[2 * Ti + 1]));
			}
			else
			{
				UVs.Add(FVector2D::ZeroVector);
			}
			Tris.Add(Verts.Num() - 1);
		}

		// Per-face tangent from the UV gradient, so normal maps orient correctly.
		// Assigned to all three corners; degenerate UVs fall back to an edge dir.
		const FVector E1 = Verts[Base + 1] - Verts[Base];
		const FVector E2 = Verts[Base + 2] - Verts[Base];
		const FVector2D D1 = UVs[Base + 1] - UVs[Base];
		const FVector2D D2 = UVs[Base + 2] - UVs[Base];
		const double Det = D1.X * D2.Y - D2.X * D1.Y;
		FVector T = FMath::Abs(Det) > SMALL_NUMBER ? ((E1 * D2.Y - E2 * D1.Y) / Det) : E1;
		T = T.GetSafeNormal();
		if (T.IsNearlyZero())
		{
			T = FVector::ForwardVector;
		}
		for (int32 C = 0; C < 3; ++C)
		{
			Tangents.Add(FProcMeshTangent(T, /*bFlipTangentY=*/false));
		}
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

	UMaterialInstanceDynamic* Mid = Comp->CreateDynamicMaterialInstance(0, Master);
	if (!Mid)
	{
		return;
	}
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
			const EMjMaterialRole Role = static_cast<EMjMaterialRole>(R);
			const bool bSRGB = (Role == EMjMaterialRole::Rgb || Role == EMjMaterialRole::Rgba
				|| Role == EMjMaterialRole::Emissive);
			if (UTexture2D* Tex = GetOrBuildTexture(TexId, bSRGB))
			{
				Mid->SetTextureParameterValue(MjMaterialRoleParameter(Role), Tex);
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
	for (TObjectPtr<AActor>& B : BodyActors)
	{
		if (B)
		{
			B->Destroy();
		}
	}
	BodyActors.Reset();
	GeomComps.Reset();
	TextureCache.Reset();
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
