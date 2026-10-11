// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "Render/RenderPrimitiveBuilder.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"

#include "Render/RenderScene.h"
#include "Render/RenderComponentUtils.h"

namespace
{
constexpr double kSizeCmToBasicShapeScale = 0.02;
constexpr double kInfinitePlaneHalfCm = 2500.0;

constexpr const TCHAR* kBasicPlane = TEXT("/Engine/BasicShapes/Plane.Plane");
constexpr const TCHAR* kBasicSphere = TEXT("/Engine/BasicShapes/Sphere.Sphere");
constexpr const TCHAR* kBasicCylinder = TEXT("/Engine/BasicShapes/Cylinder.Cylinder");
constexpr const TCHAR* kBasicCube = TEXT("/Engine/BasicShapes/Cube.Cube");

bool HasFiniteSizes(const FRenderGeomDesc& Geom)
{
	return FMath::IsFinite(Geom.SizeCm.X) && FMath::IsFinite(Geom.SizeCm.Y)
		&& FMath::IsFinite(Geom.SizeCm.Z);
}

UStaticMesh* LoadBasic(const TCHAR* Path)
{
	return LoadObject<UStaticMesh>(nullptr, Path);
}

UStaticMeshComponent* MakePrimitivePart(AActor* Body, UStaticMesh* Mesh, const FVector& Scale)
{
	if (!Body || !Body->GetRootComponent() || !Mesh)
	{
		return nullptr;
	}
	UStaticMeshComponent* Component = NewObject<UStaticMeshComponent>(Body);
	if (!Component)
	{
		return nullptr;
	}
	Component->SetStaticMesh(Mesh);
	Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Component->SetRelativeScale3D(Scale);
	DisableDistanceFields(Component);
	Component->RegisterComponent();
	Component->AttachToComponent(Body->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
	return Component;
}
} // namespace

UPrimitiveComponent* BuildNeutralPrimitive(const FRenderGeomDesc& Geom, AActor* Body,
	TFunctionRef<void(UPrimitiveComponent*)> ApplyMaterial)
{
	if (!Body || !HasFiniteSizes(Geom))
	{
		return nullptr;
	}

	const double Radius = Geom.SizeCm.X;
	const double HalfLength = Geom.SizeCm.Y;
	const TCHAR* MeshPath = nullptr;
	FVector Scale = FVector::OneVector;
	bool bCapsuleWithShaft = false;

	switch (Geom.Kind)
	{
		case ERenderGeomKind::Plane:
			if (Geom.SizeCm.X < 0.0 || Geom.SizeCm.Y < 0.0)
			{
				return nullptr;
			}
			MeshPath = kBasicPlane;
			Scale = FVector((Geom.SizeCm.X > 0.0 ? Geom.SizeCm.X : kInfinitePlaneHalfCm)
					* kSizeCmToBasicShapeScale,
				(Geom.SizeCm.Y > 0.0 ? Geom.SizeCm.Y : kInfinitePlaneHalfCm)
					* kSizeCmToBasicShapeScale, 1.0);
			break;
		case ERenderGeomKind::Sphere:
			if (Radius <= 0.0) return nullptr;
			MeshPath = kBasicSphere;
			Scale = FVector(Radius, Radius, Radius) * kSizeCmToBasicShapeScale;
			break;
		case ERenderGeomKind::Ellipsoid:
			if (Geom.SizeCm.X <= 0.0 || Geom.SizeCm.Y <= 0.0 || Geom.SizeCm.Z <= 0.0) return nullptr;
			MeshPath = kBasicSphere;
			Scale = Geom.SizeCm * kSizeCmToBasicShapeScale;
			break;
		case ERenderGeomKind::Cylinder:
			if (Radius <= 0.0 || HalfLength <= 0.0) return nullptr;
			MeshPath = kBasicCylinder;
			Scale = FVector(Radius, Radius, HalfLength) * kSizeCmToBasicShapeScale;
			break;
		case ERenderGeomKind::Capsule:
			if (Radius <= 0.0 || HalfLength < 0.0) return nullptr;
			// A zero shaft is exactly a sphere. Do not attach caps under a
			// zero-scaled cylinder: inherited zero scale collapses them too.
			if (HalfLength == 0.0)
			{
				MeshPath = kBasicSphere;
				Scale = FVector(Radius, Radius, Radius) * kSizeCmToBasicShapeScale;
			}
			else
			{
				MeshPath = kBasicCylinder;
				Scale = FVector(Radius, Radius, HalfLength) * kSizeCmToBasicShapeScale;
				bCapsuleWithShaft = true;
			}
			break;
		case ERenderGeomKind::Box:
			if (Geom.SizeCm.X <= 0.0 || Geom.SizeCm.Y <= 0.0 || Geom.SizeCm.Z <= 0.0) return nullptr;
			MeshPath = kBasicCube;
			Scale = Geom.SizeCm * kSizeCmToBasicShapeScale;
			break;
		default:
			return nullptr;
	}

	UStaticMeshComponent* Component = MakePrimitivePart(Body, LoadBasic(MeshPath), Scale);
	if (!Component)
	{
		return nullptr;
	}
	ApplyMaterial(Component);

	if (bCapsuleWithShaft)
	{
		UStaticMesh* SphereMesh = LoadBasic(kBasicSphere);
		if (!SphereMesh)
		{
			return Component;
		}
		const FVector CapScale(1.0, 1.0, Radius / HalfLength);
		for (const double CapZ : {50.0, -50.0})
		{
			UStaticMeshComponent* Cap = NewObject<UStaticMeshComponent>(Body);
			if (!Cap)
			{
				continue;
			}
			Cap->SetStaticMesh(SphereMesh);
			Cap->SetCollisionEnabled(ECollisionEnabled::NoCollision);
			Cap->SetRelativeScale3D(CapScale);
			DisableDistanceFields(Cap);
			Cap->RegisterComponent();
			Cap->AttachToComponent(Component, FAttachmentTransformRules::KeepRelativeTransform);
			Cap->SetRelativeLocation(FVector(0.0, 0.0, CapZ));
			ApplyMaterial(Cap);
		}
	}
	return Component;
}
