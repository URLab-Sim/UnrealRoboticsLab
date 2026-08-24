// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Entity/MjBakedAssetResolver.h"

#include "Components/StaticMeshComponent.h"
#include "ProceduralMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"

#include "MuJoCo/Fast/MjRendererAssetBaker.h"
#include "MuJoCo/Utils/MjRenderComponentUtils.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
// The engine primitives are 100 cm across / tall, so component scale 1 is a
// 50 cm half-extent. MJCF sizes are metres: scale = size(m) * 100 / 50.
constexpr double kSizeToScale = 2.0;
constexpr double kInfinitePlaneHalfM = 25.0; // size==0 plane -> 25 m half-extent

constexpr const TCHAR* kBasicPlane = TEXT("/Engine/BasicShapes/Plane.Plane");
constexpr const TCHAR* kBasicSphere = TEXT("/Engine/BasicShapes/Sphere.Sphere");
constexpr const TCHAR* kBasicCylinder = TEXT("/Engine/BasicShapes/Cylinder.Cylinder");
constexpr const TCHAR* kBasicCube = TEXT("/Engine/BasicShapes/Cube.Cube");

UStaticMesh* LoadBasic(const TCHAR* Path)
{
	return LoadObject<UStaticMesh>(nullptr, Path);
}
} // namespace

FMjBakedAssetResolver::FMjBakedAssetResolver(mjModel_* InModel, UMjRendererAssetBaker* InBaker)
	: Model(InModel)
	, Baker(InBaker)
{
}

UPrimitiveComponent* FMjBakedAssetResolver::MakeGeomComponent(int32 G, AActor* Body) const
{
	if (!Model || !Baker || !Body)
	{
		return nullptr;
	}

	const int32 Type = Model->geom_type[G];
	const double* Size = Model->geom_size + 3 * G;

	// Primitive selection + scale from MuJoCo size semantics. A mesh geom is a
	// shared UStaticMesh (editor) or a runtime ProceduralMeshComponent (packaged).
	const TCHAR* MeshPath = nullptr;
	FVector Scale(1, 1, 1);
	switch (Type)
	{
		case mjGEOM_PLANE:
		{
			MeshPath = kBasicPlane;
			const double Hx = Size[0] > 0 ? Size[0] : kInfinitePlaneHalfM;
			const double Hy = Size[1] > 0 ? Size[1] : kInfinitePlaneHalfM;
			Scale = FVector(Hx * kSizeToScale, Hy * kSizeToScale, 1.0);
			break;
		}
		case mjGEOM_SPHERE:
			MeshPath = kBasicSphere;
			Scale = FVector(Size[0], Size[0], Size[0]) * kSizeToScale;
			break;
		case mjGEOM_ELLIPSOID:
			MeshPath = kBasicSphere;
			Scale = FVector(Size[0], Size[1], Size[2]) * kSizeToScale;
			break;
		case mjGEOM_CYLINDER:
			MeshPath = kBasicCylinder;
			Scale = FVector(Size[0], Size[0], Size[1]) * kSizeToScale;
			break;
		case mjGEOM_CAPSULE:
			// Cylinder shaft; the two rounded end caps are added as sphere child
			// components below. size[0]=radius, size[1]=half-length of the cylinder.
			MeshPath = kBasicCylinder;
			Scale = FVector(Size[0], Size[0], Size[1]) * kSizeToScale;
			break;
		case mjGEOM_BOX:
			MeshPath = kBasicCube;
			Scale = FVector(Size[0], Size[1], Size[2]) * kSizeToScale;
			break;
		case mjGEOM_MESH:
		{
#if WITH_EDITOR
			// Editor render server: a shared UStaticMesh (built once per mesh id)
			// referenced by pointer, so the PIE-world duplication stays cheap. Mesh
			// verts are already in UE units, so the component needs no extra scale.
			UStaticMesh* Mesh = Baker->GetOrBuildStaticMesh(Model->geom_dataid[G]);
			if (!Mesh)
			{
				return nullptr;
			}
			UStaticMeshComponent* Comp = NewObject<UStaticMeshComponent>(Body);
			Comp->SetStaticMesh(Mesh);
			Comp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
			DisableDistanceFields(Comp);
			Comp->RegisterComponent();
			Comp->AttachToComponent(Body->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
			Baker->ApplyGeomMaterial(Comp, G);
			return Comp;
#else
			// Packaged game: BuildFromMeshDescriptions is editor-only, so build a
			// ProceduralMeshComponent that generates its render data at runtime.
			UProceduralMeshComponent* Pmc = Baker->BuildMesh(G, Body);
			if (Pmc)
			{
				Baker->ApplyGeomMaterial(Pmc, G);
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
	DisableDistanceFields(Comp);
	Comp->RegisterComponent();
	Comp->AttachToComponent(Body->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
	Baker->ApplyGeomMaterial(Comp, G);

	// Rounded capsule caps: a sphere at each end of the cylinder shaft, as child
	// components so they follow the shaft's streamed world transform. The base
	// cylinder/sphere meshes are 100 units, so the shaft's local half-height is 50
	// units; a cap sits there. The shaft's own non-uniform scale (r,r,halflen) is
	// cancelled on the cap's Z (1,1,r/halflen) so each cap stays a sphere of radius r.
	if (Type == mjGEOM_CAPSULE)
	{
		if (UStaticMesh* SphereMesh = LoadBasic(kBasicSphere))
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
				DisableDistanceFields(Cap);
				Cap->RegisterComponent();
				Cap->AttachToComponent(Comp, FAttachmentTransformRules::KeepRelativeTransform);
				Cap->SetRelativeLocation(FVector(0.0, 0.0, CapZ[S]));
				Cap->SetRelativeScale3D(CapScale);
				Baker->ApplyGeomMaterial(Cap, G);
			}
		}
	}
	return Comp;
}
