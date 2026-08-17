// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Entity/MjImportedAssetResolver.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"

#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Spec/MjAssetResolve.h"
#include "MuJoCo/Spec/MjSpecRef.h"

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

// MuJoCo's default geom colour. An rgba equal to it, componentwise and exactly,
// is treated as unauthored -- the same test the engine and the editor preview use.
const FLinearColor kMagicDefaultRgba(0.5f, 0.5f, 0.5f, 1.0f);

bool IsMagicDefaultRgba(const FLinearColor& Color)
{
	return Color.R == kMagicDefaultRgba.R && Color.G == kMagicDefaultRgba.G && Color.B == kMagicDefaultRgba.B
		&& Color.A == kMagicDefaultRgba.A;
}

// A query-only static mesh part carrying the named engine primitive (null path
// leaves the asset unassigned), attached under Body's root.
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

FMjImportedAssetResolver::FMjImportedAssetResolver(mjModel_* InModel, UMjRendererAssetBaker* InBaker,
	TMap<int32, TWeakObjectPtr<UMjGeom>> InGeomIndex)
	: Model(InModel)
	, GeomIndex(MoveTemp(InGeomIndex))
	, BakedFallback(InModel, InBaker)
{
}

UMjGeom* FMjImportedAssetResolver::OriginFor(int32 GeomId) const
{
	if (const TWeakObjectPtr<UMjGeom>* Found = GeomIndex.Find(GeomId))
	{
		return Found->Get();
	}
	return nullptr;
}

void FMjImportedAssetResolver::RecordMeshFrameInverse(int32 G) const
{
	if (!Model)
	{
		return;
	}
	const int32 MeshId = Model->geom_dataid[G];
	if (MeshId < 0)
	{
		return;
	}
	const double* MeshPos = Model->mesh_pos + 3 * MeshId;
	const double* MeshQuat = Model->mesh_quat + 4 * MeshId;

	// Inverse of the mesh's baked pose (mesh_pos, mesh_quat), in MuJoCo's frame:
	// quat conjugate, and the position rotated back and negated.
	FMjMeshFrameInverse Inv;
	Inv.Quat[0] = MeshQuat[0];
	Inv.Quat[1] = -MeshQuat[1];
	Inv.Quat[2] = -MeshQuat[2];
	Inv.Quat[3] = -MeshQuat[3];
	double Rotated[3];
	mju_rotVecQuat(Rotated, MeshPos, Inv.Quat);
	Inv.Pos[0] = -Rotated[0];
	Inv.Pos[1] = -Rotated[1];
	Inv.Pos[2] = -Rotated[2];
	MeshFrameInverse.Add(G, Inv);
}

void FMjImportedAssetResolver::ApplyImportedMaterial(UPrimitiveComponent* Comp, UMjGeom* Geom, int32 G) const
{
	UMaterialInterface* Master = MjLoadMasterMaterial();
	if (!Master || !Comp || !Geom)
	{
		return;
	}

	// The material the element names, resolved through its own default-class chain
	// and its own spec -- the prefix-free authored name, not the model's prefixed one.
	const FSpecRef Spec = FSpecRef::OverOwner(Geom);
	const FString MaterialName = Geom->EffectiveMaterialName();
	const FLinearColor BaseColor = Geom->GetEffectiveColor();

	// Parity with the editor preview (UMjGeom::ApplySpecMaterial): an imported mesh
	// arrives with materials of its own, so overwriting them with a spec colour the
	// model never asked for is a downgrade. Dress a mesh only where the spec asks --
	// a named material, or an rgba authored away from the magic default; a converted
	// prop keeps its own StaticMesh's materials this way. A primitive has nothing of
	// its own to keep and is dressed either way.
	const bool bAssetBacked = (Model->geom_type[G] == mjGEOM_MESH);
	const bool bSpecAsksForColour = !MaterialName.IsEmpty() || !IsMagicDefaultRgba(BaseColor);
	if (bAssetBacked && !bSpecAsksForColour)
	{
		return;
	}

	FMjMaterialValues Values;
	MjResolveMaterial(Spec, MaterialName, Values);

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

	// No authoring element bound to this id (scene-root geometry, an inline mesh):
	// the model is all there is, so the baked resolver draws it.
	UMjGeom* Geom = OriginFor(G);
	if (!Geom)
	{
		return BakedFallback.MakeGeomComponent(G, Body);
	}

	const int32 Type = Model->geom_type[G];

	if (Type == mjGEOM_MESH)
	{
		// The `<mesh>` element the geom names, resolved to the imported asset by the
		// element's own local name. An inline mesh the importer never produced has no
		// element to find; the baked resolver draws it from the model instead.
		const FMjResolvedMesh Resolved = MjResolveMesh(FSpecRef::OverOwner(Geom), Geom->EffectiveMeshName());
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
		// The materials the source carried, where it carried its own (a converted
		// prop records its actor mesh component's materials, overrides included).
		// Applied before the spec pass so a spec that asks for no colour keeps them.
		for (int32 Slot = 0; Slot < Resolved.Materials.Num(); ++Slot)
		{
			if (Resolved.Materials[Slot])
			{
				Comp->SetMaterial(Slot, Resolved.Materials[Slot]);
			}
		}
		// The raw asset keeps its own origin; undo the compile-time mesh recentre so
		// it draws where the compiled geom (and the collider) is.
		RecordMeshFrameInverse(G);
		ApplyImportedMaterial(Comp, Geom, G);
		return Comp;
	}

	const FPrimitiveShape Shape = ShapeFor(Type);
	if (!Shape.MeshPath)
	{
		return BakedFallback.MakeGeomComponent(G, Body);
	}
	UStaticMeshComponent* Comp = MakePart(Body, Shape.MeshPath);
	if (!Comp)
	{
		return nullptr;
	}
	Comp->SetRelativeScale3D(PrimitiveScale(Type, Model->geom_size + 3 * G));
	ApplyImportedMaterial(Comp, Geom, G);

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
			ApplyImportedMaterial(Cap, Geom, G);
		}
	}
	return Comp;
}
