// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Entity/MjImportedAssetResolver.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"

#include "MuJoCo/Spec/MjAssetResolve.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
// The engine primitives are 100 cm across / tall, so component scale 1 is a
// 50 cm half-extent. MJCF sizes are metres: scale = size(m) * 100 / 50.
constexpr double kSizeToScale = 2.0;
constexpr double kBaseHalf = 50.0;
constexpr double kInfinitePlaneHalfM = 25.0;

constexpr const TCHAR* kBasicPlane = TEXT("/Engine/BasicShapes/Plane.Plane");
constexpr const TCHAR* kBasicSphere = TEXT("/Engine/BasicShapes/Sphere.Sphere");
constexpr const TCHAR* kBasicCylinder = TEXT("/Engine/BasicShapes/Cylinder.Cylinder");
constexpr const TCHAR* kBasicCube = TEXT("/Engine/BasicShapes/Cube.Cube");

// The engine primitive a non-mesh geom previews as, plus the sphere a capsule
// caps with, keyed by mjtGeom. Null where a type carries no engine primitive.
struct FPrimitiveShape
{
	const TCHAR* MeshPath = nullptr;
	const TCHAR* CapMeshPath = nullptr;
};

FPrimitiveShape ShapeFor(int32 Type)
{
	switch (Type)
	{
		case mjGEOM_PLANE:
			return {kBasicPlane, nullptr};
		case mjGEOM_SPHERE:
		case mjGEOM_ELLIPSOID:
			return {kBasicSphere, nullptr};
		case mjGEOM_CYLINDER:
			return {kBasicCylinder, nullptr};
		case mjGEOM_CAPSULE:
			return {kBasicCylinder, kBasicSphere};
		case mjGEOM_BOX:
			return {kBasicCube, nullptr};
		default:
			return {nullptr, nullptr};
	}
}

// The scale a scaled engine primitive needs for a geom's MuJoCo size.
FVector PrimitiveScale(int32 Type, const double* Size)
{
	switch (Type)
	{
		case mjGEOM_PLANE:
		{
			const double Hx = Size[0] > 0 ? Size[0] : kInfinitePlaneHalfM;
			const double Hy = Size[1] > 0 ? Size[1] : kInfinitePlaneHalfM;
			return FVector(Hx * kSizeToScale, Hy * kSizeToScale, 1.0);
		}
		case mjGEOM_SPHERE:
			return FVector(Size[0], Size[0], Size[0]) * kSizeToScale;
		case mjGEOM_ELLIPSOID:
			return FVector(Size[0], Size[1], Size[2]) * kSizeToScale;
		case mjGEOM_CYLINDER:
		case mjGEOM_CAPSULE:
			return FVector(Size[0], Size[0], Size[1]) * kSizeToScale;
		case mjGEOM_BOX:
			return FVector(Size[0], Size[1], Size[2]) * kSizeToScale;
		default:
			return FVector::OneVector;
	}
}

// The MakeVisualizerPart pattern from UMjGeom: a query-only static mesh part
// carrying the named engine primitive (null path leaves the asset unassigned).
UStaticMeshComponent* MakePart(AActor* Body, const TCHAR* MeshPath)
{
	UStaticMeshComponent* Part = NewObject<UStaticMeshComponent>(Body);
	if (!Part)
	{
		return nullptr;
	}
	Part->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	if (MeshPath)
	{
		if (UStaticMesh* Asset = LoadObject<UStaticMesh>(nullptr, MeshPath))
		{
			Part->SetStaticMesh(Asset);
		}
	}
	Part->RegisterComponent();
	Part->AttachToComponent(Body->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
	return Part;
}
} // namespace

FMjImportedAssetResolver::FMjImportedAssetResolver(mjModel_* InModel, const FSpecRef& InSpec, UMjbAssetBaker* InBaker)
	: Model(InModel)
	, Spec(InSpec)
	, BakedFallback(InModel, InBaker)
{
}

void FMjImportedAssetResolver::ApplyImportedMaterial(UPrimitiveComponent* Comp, int32 G) const
{
	UMaterialInterface* Master = MjLoadMasterMaterial();
	if (!Master || !Comp)
	{
		return;
	}

	const int32 MatId = Model->geom_matid[G];
	FString MaterialName;
	if (MatId >= 0)
	{
		const char* Name = mj_id2name(Model, mjOBJ_MATERIAL, MatId);
		if (Name)
		{
			MaterialName = ANSI_TO_TCHAR(Name);
		}
	}
	FMjMaterialValues Values;
	MjResolveMaterial(Spec, MaterialName, Values);

	// The rgba a geom draws with: its material's when it has one, else its own --
	// the same convention the compiled model resolves geom_matid by.
	const float* Rgba = (MatId >= 0) ? (Model->mat_rgba + 4 * MatId) : (Model->geom_rgba + 4 * G);
	const FLinearColor BaseColor(Rgba[0], Rgba[1], Rgba[2], Rgba[3]);

	// The planar extent texuniform tiles by (a size-0 plane is drawn as a finite
	// quad, so report its half-extent rather than zero).
	const bool bPlane = (Model->geom_type[G] == mjGEOM_PLANE);
	const double Sx = Model->geom_size[3 * G + 0] > 0.0 ? Model->geom_size[3 * G + 0]
		: (bPlane ? kInfinitePlaneHalfM : 0.0);
	const double Sy = Model->geom_size[3 * G + 1] > 0.0 ? Model->geom_size[3 * G + 1]
		: (bPlane ? kInfinitePlaneHalfM : 0.0);

	UMaterialInstanceDynamic* Mid = UMaterialInstanceDynamic::Create(Master, Comp);
	if (!Mid)
	{
		return;
	}
	Comp->SetMaterial(0, Mid);
	MjApplyMaterialParameters(*Mid, Values, BaseColor, Spec, FVector2D(Sx, Sy));
}

UPrimitiveComponent* FMjImportedAssetResolver::MakeGeomComponent(int32 G, AActor* Body) const
{
	if (!Model || !Body)
	{
		return nullptr;
	}
	const int32 Type = Model->geom_type[G];

	if (Type == mjGEOM_MESH)
	{
		// The `<mesh>` element the geom names, resolved to the imported asset. An
		// inline mesh the importer never produced has no element to find; the baked
		// resolver draws it from the MJB instead.
		const char* MeshName = mj_id2name(Model, mjOBJ_MESH, Model->geom_dataid[G]);
		const FMjResolvedMesh Resolved = MeshName
			? MjResolveMesh(Spec, ANSI_TO_TCHAR(MeshName))
			: FMjResolvedMesh();
		if (!Resolved.Asset)
		{
			return BakedFallback.MakeGeomComponent(G, Body);
		}
		UStaticMeshComponent* Comp = MakePart(Body, nullptr);
		if (!Comp)
		{
			return nullptr;
		}
		Comp->SetStaticMesh(Resolved.Asset);
		// `<mesh scale>` only: the import already put the asset in the level's units.
		Comp->SetRelativeScale3D(Resolved.Scale);
		ApplyImportedMaterial(Comp, G);
		return Comp;
	}

	const FPrimitiveShape Shape = ShapeFor(Type);
	if (!Shape.MeshPath)
	{
		return nullptr;
	}
	UStaticMeshComponent* Comp = MakePart(Body, Shape.MeshPath);
	if (!Comp)
	{
		return nullptr;
	}
	Comp->SetRelativeScale3D(PrimitiveScale(Type, Model->geom_size + 3 * G));
	ApplyImportedMaterial(Comp, G);

	// Rounded capsule caps: a sphere at each end of the cylinder shaft. The base
	// meshes are 100 units, so the shaft's local half-height is 50; the shaft's own
	// non-uniform scale (r,r,halflen) is cancelled on the cap's Z so each stays a
	// sphere of radius r.
	if (Shape.CapMeshPath)
	{
		const double* Size = Model->geom_size + 3 * G;
		const double R = Size[0];
		const double HalfLen = Size[1] > KINDA_SMALL_NUMBER ? Size[1] : R;
		const FVector CapScale(1.0f, 1.0f, static_cast<float>(R / HalfLen));
		const double CapZ[2] = {kBaseHalf, -kBaseHalf};
		for (int32 S = 0; S < 2; ++S)
		{
			UStaticMeshComponent* Cap = NewObject<UStaticMeshComponent>(Body);
			if (!Cap)
			{
				continue;
			}
			Cap->SetCollisionEnabled(ECollisionEnabled::NoCollision);
			if (UStaticMesh* SphereMesh = LoadObject<UStaticMesh>(nullptr, Shape.CapMeshPath))
			{
				Cap->SetStaticMesh(SphereMesh);
			}
			Cap->RegisterComponent();
			Cap->AttachToComponent(Comp, FAttachmentTransformRules::KeepRelativeTransform);
			Cap->SetRelativeLocation(FVector(0.0, 0.0, CapZ[S]));
			Cap->SetRelativeScale3D(CapScale);
			ApplyImportedMaterial(Cap, G);
		}
	}
	return Comp;
}
