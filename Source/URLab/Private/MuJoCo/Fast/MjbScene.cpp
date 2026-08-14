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
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "UObject/ConstructorHelpers.h"
#include "Engine/World.h"

#include "MuJoCo/Spec/MjAssetResolve.h"
#include "MuJoCo/Utils/URLabAxisConv.h"
#include "Utils/URLabLogging.h"

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
} // namespace

AMjbScene::AMjbScene()
{
	PrimaryActorTick.bCanEverTick = true;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
}

void AMjbScene::BeginPlay()
{
	Super::BeginPlay();
	if (!MjbFilePath.IsEmpty())
	{
		LoadAndBuild();
	}
}

void AMjbScene::EndPlay(const EEndPlayReason::Type Reason)
{
	Teardown();
	Super::EndPlay(Reason);
}

int32 AMjbScene::LoadAndBuild()
{
	Teardown();

	char Err[1024] = {0};
	Model = mj_loadModel(TCHAR_TO_UTF8(*MjbFilePath), nullptr);
	if (!Model)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjbScene] mj_loadModel failed for %s (version-mismatched MJB?)"),
			*MjbFilePath);
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
			// Approximated as a cylinder for now (flat ends); rounded caps are a
			// follow-on. Matches the URDF exporter's capsule handling.
			MeshPath = TEXT("/Engine/BasicShapes/Cylinder.Cylinder");
			Scale = FVector(Size[0], Size[0], Size[1]) * kSizeToScale;
			break;
		case mjGEOM_BOX:
			MeshPath = TEXT("/Engine/BasicShapes/Cube.Cube");
			Scale = FVector(Size[0], Size[1], Size[2]) * kSizeToScale;
			break;
		case mjGEOM_MESH:
			// Follow-on: build a ProceduralMeshComponent from mesh_vert/face.
			UE_LOG(LogURLab, Verbose, TEXT("[MjbScene] geom %d is a mesh; skipped in this pass"), G);
			return nullptr;
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
	return Comp;
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
	// Full PBR params come from the material when the geom has one; sensible
	// neutral defaults otherwise. Texture roles are the next pass.
	if (MatId >= 0)
	{
		Mid->SetScalarParameterValue(TEXT("Metallic"), Model->mat_metallic[MatId]);
		Mid->SetScalarParameterValue(TEXT("Roughness"), Model->mat_roughness[MatId]);
		Mid->SetScalarParameterValue(TEXT("Specular"), FMath::Clamp(Model->mat_specular[MatId], 0.f, 1.f));
		Mid->SetScalarParameterValue(TEXT("Reflectance"), FMath::Clamp(Model->mat_reflectance[MatId], 0.f, 1.f));
		Mid->SetScalarParameterValue(TEXT("Emission"), FMath::Max(Model->mat_emission[MatId], 0.f));
	}
	else
	{
		Mid->SetScalarParameterValue(TEXT("Metallic"), 0.0f);
		Mid->SetScalarParameterValue(TEXT("Roughness"), 0.6f);
		Mid->SetScalarParameterValue(TEXT("Specular"), 0.5f);
	}
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
	if (!bTestSweep || !Model || !Data)
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

void AMjbScene::Teardown()
{
	for (TObjectPtr<AActor>& B : BodyActors)
	{
		if (B)
		{
			B->Destroy();
		}
	}
	BodyActors.Reset();
	GeomComps.Reset();
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
