// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

// -----------------------------------------------------------------------------
// Pooled instanced-mesh overlay renderer (source-of-truth 8.5). Replaces the old
// DrawDebug* suite: every overlay primitive is one instance in a pooled
// UInstancedStaticMeshComponent, so the overlay is O(distinct-styles) draw calls
// and scales to hundreds of contacts. Primitives compose from four engine
// BasicShapes meshes (Sphere/Cube/Cylinder/Cone) -- arrows are a shaft cylinder +
// cone head, lines/wire-boxes are thin cylinders -- so no overlay mesh needs
// authoring. Pools are cleared + refilled each frame (one-frame lifetime, matching
// the old DrawDebug persist=false path) with high-water-mark component reuse.
//
// CONTENT-ASSET GAP (flagged): true per-instance colour + translucency + a
// jump-flood selection outline all need an authored master material that reads
// PerInstanceCustomData and is set Translucent / custom-depth -- a .uasset that
// cannot be created headlessly. Until it exists this renderer falls back to:
//   * per-instance colour via a per-(mesh,colour,blend) POOL SPLIT, each pool's
//     UMaterialInstanceDynamic carrying the colour (correct colour today);
//   * 4-float PerInstanceCustomData (RGBA) written on every instance regardless,
//     so swapping in the authored material is plug-and-play (collapse the colour
//     key, keep the custom data);
//   * BasicShapeMaterial as the opaque fallback; a translucent engine material is
//     used when found, else translucent overlays render opaque (alpha in custom
//     data only);
//   * custom-depth marking on the selection pool (the outline post-process
//     material itself is the one remaining authored-asset task).
// -----------------------------------------------------------------------------

#include "MuJoCo/Entity/MjOverlayRenderer.h"

#include "Components/InstancedStaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "CoreGlobals.h"

#include "MuJoCo/Core/MjRenderSnapshot.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "MuJoCo/Utils/URLabAxisConv.h"
#include "Utils/URLabLogging.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
// A flag set in a runtime-sized array; absent/short arrays read as off.
bool FlagSet(const TArray<uint8>& Flags, int32 Index)
{
	return Flags.IsValidIndex(Index) && Flags[Index] != 0;
}

// A group's visibility. An unsized mask (nothing authored yet) shows every group,
// so a freshly bound renderer draws rather than hiding everything by default.
bool GroupVisible(const TArray<uint8>& Mask, int32 Group)
{
	if (Mask.Num() == 0)
	{
		return true;
	}
	return Mask.IsValidIndex(Group) && Mask[Group] != 0;
}

// Column c (0..2) of a row-major 3x3 world rotation, i.e. that frame axis in
// world coordinates: element (r, c) is Mat[r * 3 + c].
void MatColumn(const mjtNum* Mat, int32 c, mjtNum Out[3])
{
	Out[0] = Mat[0 * 3 + c];
	Out[1] = Mat[1 * 3 + c];
	Out[2] = Mat[2 * 3 + c];
}

FLinearColor Lin(const FColor& C)
{
	return FLinearColor(C);
}

// A MuJoCo float[4] rgba (0..1) as an FColor. Falls back to the given colour when
// the model slot is unset (all-zero), so an unstyled model still renders visibly.
FColor RgbaToColor(const float* Rgba, const FColor& Fallback)
{
	if (!Rgba || (Rgba[0] <= 0.f && Rgba[1] <= 0.f && Rgba[2] <= 0.f && Rgba[3] <= 0.f))
	{
		return Fallback;
	}
	return FColor(static_cast<uint8>(FMath::Clamp(Rgba[0], 0.f, 1.f) * 255.f),
		static_cast<uint8>(FMath::Clamp(Rgba[1], 0.f, 1.f) * 255.f),
		static_cast<uint8>(FMath::Clamp(Rgba[2], 0.f, 1.f) * 255.f),
		static_cast<uint8>(FMath::Clamp(Rgba[3], 0.f, 1.f) * 255.f));
}

// Rotation whose local +Z maps to the given (assumed non-zero) world direction.
FQuat ZToDir(const FVector& Dir)
{
	return FRotationMatrix::MakeFromZ(Dir).ToQuat();
}
} // namespace

UMjOverlayRenderer::UMjOverlayRenderer()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
}

void UMjOverlayRenderer::SetModel(mjModel_* InModel)
{
	Model = InModel;
}

// -----------------------------------------------------------------------------
// Resources + pool management
// -----------------------------------------------------------------------------

void UMjOverlayRenderer::EnsureResources()
{
	if (bResourcesReady)
	{
		return;
	}
	bResourcesReady = true;

	Meshes.SetNum(static_cast<int32>(EMjMesh::Count));
	auto Load = [](const TCHAR* Path) -> UStaticMesh* {
		return LoadObject<UStaticMesh>(nullptr, Path);
	};
	Meshes[static_cast<int32>(EMjMesh::Sphere)] = Load(TEXT("/Engine/BasicShapes/Sphere.Sphere"));
	Meshes[static_cast<int32>(EMjMesh::Cube)] = Load(TEXT("/Engine/BasicShapes/Cube.Cube"));
	Meshes[static_cast<int32>(EMjMesh::Cylinder)] = Load(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	Meshes[static_cast<int32>(EMjMesh::Cone)] = Load(TEXT("/Engine/BasicShapes/Cone.Cone"));

	for (int32 i = 0; i < static_cast<int32>(EMjMesh::Count); ++i)
	{
		if (Meshes[i])
		{
			const FBox B = Meshes[i]->GetBoundingBox();
			MeshExtent[i] = B.GetExtent();
			MeshPivotZ[i] = static_cast<float>(B.GetCenter().Z);
		}
		else
		{
			UE_LOG(LogURLab, Warning,
				TEXT("Overlay renderer: BasicShapes mesh %d failed to load -- some overlays degrade."), i);
		}
	}

	// Fallback material: BasicShapeMaterial (opaque). Probe a colour vector param the
	// same way the tendon overlay does.
	OpaqueBaseMaterial = LoadObject<UMaterialInterface>(
		nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	if (OpaqueBaseMaterial)
	{
		TArray<FMaterialParameterInfo> VecInfos;
		TArray<FGuid> Guids;
		OpaqueBaseMaterial->GetAllVectorParameterInfo(VecInfos, Guids);
		static const FName Preferred[] = {
			TEXT("Color"), TEXT("BaseColor"), TEXT("Tint"), TEXT("TintColor"), TEXT("DiffuseColor")};
		for (const FName& Pref : Preferred)
		{
			for (const FMaterialParameterInfo& Info : VecInfos)
			{
				if (Info.Name == Pref)
				{
					ColorParamName = Pref;
					break;
				}
			}
			if (!ColorParamName.IsNone())
				break;
		}
		if (ColorParamName.IsNone() && VecInfos.Num() > 0)
		{
			ColorParamName = VecInfos[0].Name;
		}
	}
	else
	{
		UE_LOG(LogURLab, Warning,
			TEXT("Overlay renderer: BasicShapeMaterial failed to load -- overlays render untinted."));
	}

	// A translucent engine material is not guaranteed to ship; if one is found we use
	// it for translucent overlays, else they fall back to the opaque material.
	// (An authored master material is the proper fix -- see the file header note.)
	TranslucentBaseMaterial = nullptr;
}

UInstancedStaticMeshComponent* UMjOverlayRenderer::GetPool(FPoolLayer& Layer, EMjMesh Mesh,
	const FColor& Color, bool bTranslucent, bool bCustomDepth)
{
	UStaticMesh* SM = Meshes[static_cast<int32>(Mesh)];
	if (!SM)
	{
		return nullptr;
	}

	const uint64 Key = (static_cast<uint64>(Mesh) << 34) | (static_cast<uint64>(bCustomDepth ? 1 : 0) << 33)
					   | (static_cast<uint64>(bTranslucent ? 1 : 0) << 32)
					   | static_cast<uint64>(Color.ToPackedARGB());

	if (TObjectPtr<UInstancedStaticMeshComponent>* Found = Layer.Pools.Find(Key))
	{
		return Found->Get();
	}

	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return nullptr;
	}
	UInstancedStaticMeshComponent* ISM =
		NewObject<UInstancedStaticMeshComponent>(Owner, NAME_None, RF_Transient);
	if (!ISM)
	{
		return nullptr;
	}
	ISM->SetMobility(EComponentMobility::Movable);
	ISM->SetStaticMesh(SM);
	ISM->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	ISM->SetCastShadow(false); // decor casts no shadows (matches mjCAT_DECOR)
	ISM->SetCanEverAffectNavigation(false);
	ISM->SetNumCustomDataFloats(4); // RGBA, for a future PerInstanceCustomData material
	ISM->RegisterComponent();
	ISM->AttachToComponent(this, FAttachmentTransformRules::KeepWorldTransform);

	UMaterialInterface* Base = (bTranslucent && TranslucentBaseMaterial)
								   ? TranslucentBaseMaterial.Get()
								   : OpaqueBaseMaterial.Get();
	if (Base)
	{
		UMaterialInstanceDynamic* MID = ISM->CreateDynamicMaterialInstance(0, Base);
		if (MID && !ColorParamName.IsNone())
		{
			MID->SetVectorParameterValue(ColorParamName, Lin(Color));
		}
	}

	if (bCustomDepth)
	{
		// Selection highlight: mark for custom-depth so an outline post-process can
		// pick it out. The outline material itself is the flagged authored asset.
		ISM->SetRenderCustomDepth(true);
		ISM->SetCustomDepthStencilValue(1);
	}

	Layer.Pools.Add(Key, ISM);
	return ISM;
}

void UMjOverlayRenderer::ClearLayer(FPoolLayer& Layer)
{
	for (TPair<uint64, TObjectPtr<UInstancedStaticMeshComponent>>& Pair : Layer.Pools)
	{
		if (Pair.Value)
		{
			Pair.Value->ClearInstances();
		}
	}
}

void UMjOverlayRenderer::AddPrim(FPoolLayer& Layer, EMjMesh Mesh, const FTransform& WorldXform,
	const FColor& Color, bool bTranslucent, bool bCustomDepth)
{
	UInstancedStaticMeshComponent* ISM = GetPool(Layer, Mesh, Color, bTranslucent, bCustomDepth);
	if (!ISM)
	{
		return;
	}
	const int32 Idx = ISM->AddInstance(WorldXform, /*bWorldSpace=*/true);
	if (Idx < 0)
	{
		return;
	}
	const FLinearColor L = Lin(Color);
	ISM->SetCustomDataValue(Idx, 0, L.R, false);
	ISM->SetCustomDataValue(Idx, 1, L.G, false);
	ISM->SetCustomDataValue(Idx, 2, L.B, false);
	ISM->SetCustomDataValue(Idx, 3, L.A, true);
}

// -----------------------------------------------------------------------------
// Composed primitives
// -----------------------------------------------------------------------------

void UMjOverlayRenderer::EmitSphere(FPoolLayer& Layer, const FVector& Center, float RadiusCm,
	const FColor& Color, bool bTranslucent, bool bCustomDepth)
{
	const float Ext = MeshExtent[static_cast<int32>(EMjMesh::Sphere)].X;
	if (Ext <= KINDA_SMALL_NUMBER)
	{
		return;
	}
	const float S = RadiusCm / Ext;
	AddPrim(Layer, EMjMesh::Sphere, FTransform(FQuat::Identity, Center, FVector(S)), Color,
		bTranslucent, bCustomDepth);
}

void UMjOverlayRenderer::EmitEllipsoid(FPoolLayer& Layer, const FVector& Center, const FVector& RadiiCm,
	const FQuat& Rot, const FColor& Color, bool bTranslucent)
{
	const float Ext = MeshExtent[static_cast<int32>(EMjMesh::Sphere)].X;
	if (Ext <= KINDA_SMALL_NUMBER)
	{
		return;
	}
	const FVector S(RadiiCm.X / Ext, RadiiCm.Y / Ext, RadiiCm.Z / Ext);
	AddPrim(Layer, EMjMesh::Sphere, FTransform(Rot, Center, S), Color, bTranslucent);
}

void UMjOverlayRenderer::EmitBox(FPoolLayer& Layer, const FVector& Center, const FVector& HalfExtentCm,
	const FQuat& Rot, const FColor& Color, bool bTranslucent)
{
	const FVector E = MeshExtent[static_cast<int32>(EMjMesh::Cube)];
	if (E.X <= KINDA_SMALL_NUMBER || E.Y <= KINDA_SMALL_NUMBER || E.Z <= KINDA_SMALL_NUMBER)
	{
		return;
	}
	const FVector S(HalfExtentCm.X / E.X, HalfExtentCm.Y / E.Y, HalfExtentCm.Z / E.Z);
	AddPrim(Layer, EMjMesh::Cube, FTransform(Rot, Center, S), Color, bTranslucent);
}

void UMjOverlayRenderer::EmitCylinder(FPoolLayer& Layer, const FVector& A, const FVector& B,
	float RadiusCm, const FColor& Color, bool bTranslucent)
{
	FVector Dir = B - A;
	const float Len = Dir.Size();
	if (Len <= KINDA_SMALL_NUMBER)
	{
		return;
	}
	Dir /= Len;
	const float HalfZ = MeshExtent[static_cast<int32>(EMjMesh::Cylinder)].Z;
	const float RExt = MeshExtent[static_cast<int32>(EMjMesh::Cylinder)].X;
	if (HalfZ <= KINDA_SMALL_NUMBER || RExt <= KINDA_SMALL_NUMBER)
	{
		return;
	}
	const float SZ = Len / (2.0f * HalfZ);
	const float SXY = RadiusCm / RExt;
	AddPrim(Layer, EMjMesh::Cylinder, FTransform(ZToDir(Dir), (A + B) * 0.5, FVector(SXY, SXY, SZ)),
		Color, bTranslucent);
}

void UMjOverlayRenderer::EmitLine(FPoolLayer& Layer, const FVector& A, const FVector& B,
	float ThicknessCm, const FColor& Color)
{
	EmitCylinder(Layer, A, B, FMath::Max(ThicknessCm * 0.5f, 0.15f), Color, /*bTranslucent=*/false);
}

void UMjOverlayRenderer::EmitArrow(FPoolLayer& Layer, const FVector& A, const FVector& B,
	float ShaftRadiusCm, float HeadRadiusCm, const FColor& Color, bool bTranslucent)
{
	FVector Dir = B - A;
	const float Len = Dir.Size();
	if (Len <= KINDA_SMALL_NUMBER)
	{
		return;
	}
	Dir /= Len;
	const float HeadLen = FMath::Clamp(HeadRadiusCm * 2.0f, 2.0f, Len * 0.9f);
	const FVector ShaftEnd = B - Dir * HeadLen;
	EmitCylinder(Layer, A, ShaftEnd, ShaftRadiusCm, Color, bTranslucent);

	// Cone head: place its base at ShaftEnd, apex at B.
	const int32 ci = static_cast<int32>(EMjMesh::Cone);
	if (!Meshes[ci])
	{
		// No cone mesh: thicken the tip so the arrow still reads.
		EmitCylinder(Layer, ShaftEnd, B, HeadRadiusCm, Color, bTranslucent);
		return;
	}
	const float ConeHalfZ = MeshExtent[ci].Z;
	const float ConeR = MeshExtent[ci].X;
	const float FullH = 2.0f * ConeHalfZ;
	if (FullH <= KINDA_SMALL_NUMBER || ConeR <= KINDA_SMALL_NUMBER)
	{
		return;
	}
	const float SZ = HeadLen / FullH;
	const float SXY = HeadRadiusCm / ConeR;
	const FQuat Rot = ZToDir(Dir);
	const float LocalBaseZ = (MeshPivotZ[ci] - ConeHalfZ) * SZ;
	const FVector T = ShaftEnd - Rot.RotateVector(FVector(0, 0, LocalBaseZ));
	AddPrim(Layer, EMjMesh::Cone, FTransform(Rot, T, FVector(SXY, SXY, SZ)), Color, bTranslucent);
}

void UMjOverlayRenderer::EmitWireBox(FPoolLayer& Layer, const FVector& Center,
	const FVector& HalfExtentCm, const FQuat& Rot, float ThicknessCm, const FColor& Color)
{
	FVector C[8];
	int32 n = 0;
	for (int32 sx = -1; sx <= 1; sx += 2)
		for (int32 sy = -1; sy <= 1; sy += 2)
			for (int32 sz = -1; sz <= 1; sz += 2)
			{
				C[n++] = Center + Rot.RotateVector(FVector(sx * HalfExtentCm.X, sy * HalfExtentCm.Y, sz * HalfExtentCm.Z));
			}
	// Corner index = ((sx+1)/2)*4 + ((sy+1)/2)*2 + ((sz+1)/2). 12 edges of the box.
	static const int32 Edges[12][2] = {
		{0, 1}, {0, 2}, {0, 4}, {1, 3}, {1, 5}, {2, 3},
		{2, 6}, {3, 7}, {4, 5}, {4, 6}, {5, 7}, {6, 7}};
	for (const auto& E : Edges)
	{
		EmitLine(Layer, C[E[0]], C[E[1]], ThicknessCm, Color);
	}
}

void UMjOverlayRenderer::EmitTorqueGlyph(FPoolLayer& Layer, const FVector& Center,
	const FVector& AxisUE, float RadiusCm, float TubeRadiusCm, const FColor& Color, bool bTranslucent)
{
	// Curl-about-axis torque glyph: an arced arrow encircling the torque axis by the
	// right-hand rule. The ring is a fan of short cylinder segments; the arc is left
	// slightly open and closed by a cone-headed arrow tangent to the circle so the
	// rotation sense reads. Built only from the existing cylinder/cone primitives.
	const FVector Axis = AxisUE.GetSafeNormal();
	if (Axis.IsNearlyZero() || RadiusCm <= KINDA_SMALL_NUMBER)
	{
		return;
	}
	// Orthonormal basis in the plane perpendicular to the axis.
	FVector U, V;
	Axis.FindBestAxisVectors(U, V);

	constexpr int32 NumSeg = 24;           // full-circle resolution
	constexpr float ArcFrac = 0.85f;       // leave a gap so the head is visible
	const float ArcAngle = 2.0f * PI * ArcFrac;
	const int32 ArcSeg = FMath::Max(2, static_cast<int32>(NumSeg * ArcFrac));
	const float dT = ArcAngle / static_cast<float>(ArcSeg);

	auto RingPoint = [&](float t) -> FVector {
		return Center + RadiusCm * (FMath::Cos(t) * U + FMath::Sin(t) * V);
	};

	FVector Prev = RingPoint(0.0f);
	for (int32 i = 1; i <= ArcSeg; ++i)
	{
		const float t = dT * static_cast<float>(i);
		const FVector Cur = RingPoint(t);
		if (i == ArcSeg)
		{
			// Final segment as a cone-headed arrow, tangent to the circle, so the
			// glyph shows the rotation direction (right-hand rule about the axis).
			EmitArrow(Layer, Prev, Cur, TubeRadiusCm, TubeRadiusCm * 2.5f, Color, bTranslucent);
		}
		else
		{
			EmitCylinder(Layer, Prev, Cur, TubeRadiusCm, Color, bTranslucent);
		}
		Prev = Cur;
	}
}

// -----------------------------------------------------------------------------
// Frame dispatch
// -----------------------------------------------------------------------------

void UMjOverlayRenderer::DrawOverlays(const FMjRenderSnapshot& Snap)
{
	if (!Model || !GetWorld())
	{
		return;
	}
	EnsureResources();
	ClearLayer(MainLayer);
	LastDrawFrame = GFrameCounter;

	if (FlagSet(Flags.VisFlags, mjVIS_CONVEXHULL))
	{
		DrawCollision(Snap);
	}
	// mjVIS_JOINT deliberately EXCLUDED -- joints render natively (DrawJoints dropped).
	if (bDrawSites)
	{
		DrawSites(Snap);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_COM))
	{
		DrawCom(Snap);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_INERTIA))
	{
		DrawInertia(Snap);
	}
	const bool bContactPoints = FlagSet(Flags.VisFlags, mjVIS_CONTACTPOINT);
	const bool bContactForces = FlagSet(Flags.VisFlags, mjVIS_CONTACTFORCE);
	const bool bContactSplit = FlagSet(Flags.VisFlags, mjVIS_CONTACTSPLIT);
	if (bContactPoints || bContactForces)
	{
		DrawContacts(Snap, bContactPoints, bContactForces, bContactSplit);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_PERTFORCE) || FlagSet(Flags.VisFlags, mjVIS_PERTOBJ))
	{
		DrawPerturb(Snap);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_CAMERA))
	{
		DrawCameras(Snap);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_LIGHT))
	{
		DrawLights(Snap);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_ACTUATOR))
	{
		DrawActuators(Snap);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_TENDON))
	{
		DrawTendons(Snap);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_RANGEFINDER))
	{
		DrawRangefinders(Snap);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_CONSTRAINT))
	{
		DrawConstraints(Snap);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_STATIC))
	{
		DrawStaticBodies(Snap);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_AUTOCONNECT))
	{
		DrawAutoConnect(Snap);
	}
	// Net-new-vs-upstream 6-DOF wrench glyph. Opt-in (off by default) so it never
	// clutters the scene unless the owner explicitly enables it -- gated on its own
	// flag rather than an mjVIS_* bit, which upstream never draws.
	if (bDrawWrench)
	{
		DrawWrenchGlyphs(Snap);
	}
}

void UMjOverlayRenderer::TickComponent(float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// The owner only calls DrawOverlays while some vis flag is on. Once they all go
	// off, DrawOverlays stops running and the main pools would keep their last
	// instances -- clear them a frame after drawing stops, matching DrawDebug expiry.
	if (GFrameCounter > LastDrawFrame + 1)
	{
		ClearLayer(MainLayer);
	}
}

// -----------------------------------------------------------------------------
// Auxiliary layers (perturbation drag spring, legacy contact debug)
// -----------------------------------------------------------------------------

void UMjOverlayRenderer::DrawDragSpring(const FVector& GrabUE, const FVector& TargetUE,
	bool bTranslate, bool bRotate, const FVector& RotTangentEndUE)
{
	EnsureResources();
	ClearLayer(DragLayer);
	EmitSphere(DragLayer, GrabUE, 3.0f, FColor::Red, /*bTranslucent=*/false);
	if (bTranslate)
	{
		EmitArrow(DragLayer, GrabUE, TargetUE, 1.5f, 4.0f, FColor::Green, false);
	}
	if (bRotate)
	{
		EmitArrow(DragLayer, GrabUE, RotTangentEndUE, 1.5f, 4.0f, FColor::Yellow, false);
	}
}

void UMjOverlayRenderer::ClearDragSpring()
{
	ClearLayer(DragLayer);
}

void UMjOverlayRenderer::BeginDebugContacts()
{
	EnsureResources();
	ClearLayer(DebugContactLayer);
}

void UMjOverlayRenderer::AddDebugContactPoint(const FVector& PointUE, float RadiusCm, const FColor& Color)
{
	EmitSphere(DebugContactLayer, PointUE, RadiusCm, Color, /*bTranslucent=*/false);
}

void UMjOverlayRenderer::AddDebugContactArrow(const FVector& FromUE, const FVector& ToUE, const FColor& Color)
{
	EmitArrow(DebugContactLayer, FromUE, ToUE, 1.0f, 3.0f, Color, /*bTranslucent=*/false);
}

// -----------------------------------------------------------------------------
// Per-overlay builders (identical maths to the old DrawDebug path)
// -----------------------------------------------------------------------------

void UMjOverlayRenderer::DrawCollision(const FMjRenderSnapshot& Snap)
{
	const mjModel* M = Model;
	// Collision hulls: solid, low-alpha, magenta family -- matching the old
	// per-type colour remap in MjUtils::DrawDebugGeom. NB: like the old path this
	// draws in raw world space WITHOUT SceneOrigin (preserved for parity).
	for (int32 G = 0; G < static_cast<int32>(M->ngeom); ++G)
	{
		if (!GroupVisible(Flags.GeomGroup, M->geom_group[G]))
		{
			continue;
		}
		const int32 Group = M->geom_group[G];
		const int32 Contype = M->geom_contype ? M->geom_contype[G] : 0;
		const int32 Conaff = M->geom_conaffinity ? M->geom_conaffinity[G] : 0;
		if (!(Group == 3 || (Contype != 0 && Conaff != 0)))
		{
			continue;
		}
		if (!Snap.GeomXPos.IsValidIndex(G * 3 + 2) || !Snap.GeomXMat.IsValidIndex(G * 9 + 8))
		{
			continue;
		}

		const FVector Pos = URLabAxisConv::MjPositionToUe(&Snap.GeomXPos[G * 3]);
		mjtNum Quat[4];
		mju_mat2Quat(Quat, &Snap.GeomXMat[G * 9]);
		const FQuat Rot = URLabAxisConv::MjQuatToUe(Quat);
		const mjtNum* Sz = &M->geom_size[G * 3];
		const uint8 A = 90; // low alpha "hull" look

		switch (M->geom_type[G])
		{
			case mjGEOM_BOX:
				EmitBox(MainLayer, Pos, FVector(Sz[0] * 100.0, Sz[1] * 100.0, Sz[2] * 100.0), Rot,
					FColor(0, 255, 255, A), true);
				break;
			case mjGEOM_SPHERE:
				EmitSphere(MainLayer, Pos, Sz[0] * 100.0f, FColor(0, 255, 0, A), true);
				break;
			case mjGEOM_ELLIPSOID:
				EmitEllipsoid(MainLayer, Pos, FVector(Sz[0] * 100.0, Sz[1] * 100.0, Sz[2] * 100.0), Rot,
					FColor(0, 255, 0, A), true);
				break;
			case mjGEOM_CAPSULE:
			{
				// Cylinder body + two end spheres, along the geom's local +Z.
				const float R = Sz[0] * 100.0f;
				const float HalfH = Sz[1] * 100.0f;
				const FVector Axis = Rot.GetAxisZ();
				const FVector P0 = Pos - Axis * HalfH;
				const FVector P1 = Pos + Axis * HalfH;
				EmitCylinder(MainLayer, P0, P1, R, FColor(255, 255, 0, A), true);
				EmitSphere(MainLayer, P0, R, FColor(255, 255, 0, A), true);
				EmitSphere(MainLayer, P1, R, FColor(255, 255, 0, A), true);
				break;
			}
			case mjGEOM_CYLINDER:
			{
				const float R = Sz[0] * 100.0f;
				const float HalfH = Sz[1] * 100.0f;
				const FVector Axis = Rot.GetAxisZ();
				EmitCylinder(MainLayer, Pos - Axis * HalfH, Pos + Axis * HalfH, R, FColor(255, 165, 0, A), true);
				break;
			}
			case mjGEOM_MESH:
			{
				// The old path drew the convex hull as lines; approximate with a
				// bounding sphere of the collision radius (flagged degradation).
				const float R = M->geom_rbound ? static_cast<float>(M->geom_rbound[G]) * 100.0f : Sz[0] * 100.0f;
				if (R > KINDA_SMALL_NUMBER)
				{
					EmitSphere(MainLayer, Pos, R, FColor(255, 0, 255, A), true);
				}
				break;
			}
			default:
				break; // plane / hfield / other: not drawn (as before)
		}
	}
}

void UMjOverlayRenderer::DrawSites(const FMjRenderSnapshot& Snap)
{
	const mjModel* M = Model;
	for (int32 S = 0; S < static_cast<int32>(M->nsite); ++S)
	{
		if (!GroupVisible(Flags.SiteGroup, M->site_group[S]))
		{
			continue;
		}
		if (!Snap.SiteXPos.IsValidIndex(S * 3 + 2))
		{
			continue;
		}
		const FVector Pos = URLabAxisConv::MjPositionToUe(&Snap.SiteXPos[S * 3]) + SceneOrigin;
		const float* Rgba = &M->site_rgba[S * 4];
		const FColor Color(static_cast<uint8>(Rgba[0] * 255.0), static_cast<uint8>(Rgba[1] * 255.0),
			static_cast<uint8>(Rgba[2] * 255.0), 200);

		const float Radius = FMath::Max(static_cast<float>(M->site_size[S * 3]) * 100.0f, 0.5f);
		const float CrossSize = FMath::Max(Radius * 2.0f, 2.0f);
		EmitSphere(MainLayer, Pos, 1.0f, Color, false);
		EmitLine(MainLayer, Pos - FVector(CrossSize, 0, 0), Pos + FVector(CrossSize, 0, 0), 1.0f, Color);
		EmitLine(MainLayer, Pos - FVector(0, CrossSize, 0), Pos + FVector(0, CrossSize, 0), 1.0f, Color);
		EmitLine(MainLayer, Pos - FVector(0, 0, CrossSize), Pos + FVector(0, 0, CrossSize), 1.0f, Color);
	}
}

void UMjOverlayRenderer::DrawCom(const FMjRenderSnapshot& Snap)
{
	const mjModel* M = Model;
	// Body 0 is the world; skip it, matching simulate's per-body subtree markers.
	for (int32 B = 1; B < static_cast<int32>(M->nbody); ++B)
	{
		if (!Snap.SubtreeCom.IsValidIndex(B * 3 + 2))
		{
			continue;
		}
		const FVector Pos = URLabAxisConv::MjPositionToUe(&Snap.SubtreeCom[B * 3]) + SceneOrigin;
		EmitSphere(MainLayer, Pos, 3.0f, FColor(255, 128, 255), false);
	}
}

void UMjOverlayRenderer::DrawInertia(const FMjRenderSnapshot& Snap)
{
	const mjModel* M = Model;
	for (int32 B = 1; B < static_cast<int32>(M->nbody); ++B)
	{
		const mjtNum Mass = M->body_mass[B];
		if (Mass <= 0.0)
		{
			continue;
		}
		if (!Snap.XiPos.IsValidIndex(B * 3 + 2) || !Snap.XiMat.IsValidIndex(B * 9 + 8))
		{
			continue;
		}

		// Equivalent inertia box: solve the uniform-box moments for the full edge
		// lengths, then halve for the draw extent. Same box simulate renders.
		const mjtNum* I = &M->body_inertia[B * 3];
		const mjtNum T = 6.0 * (I[0] + I[1] + I[2]) / Mass;
		const mjtNum Ex = FMath::Sqrt(FMath::Max(0.0, T - 12.0 * I[0] / Mass));
		const mjtNum Ey = FMath::Sqrt(FMath::Max(0.0, T - 12.0 * I[1] / Mass));
		const mjtNum Ez = FMath::Sqrt(FMath::Max(0.0, T - 12.0 * I[2] / Mass));

		const FVector Pos = URLabAxisConv::MjPositionToUe(&Snap.XiPos[B * 3]) + SceneOrigin;
		mjtNum Quat[4];
		mju_mat2Quat(Quat, &Snap.XiMat[B * 9]);
		const FQuat Rot = URLabAxisConv::MjQuatToUe(Quat);

		// Metres -> cm half-extent: full edge * 100 / 2 = * 50.
		const FVector Extent(Ex * 50.0, Ey * 50.0, Ez * 50.0);
		EmitBox(MainLayer, Pos, Extent, Rot, FColor(120, 180, 255, 153), true); // alpha ~0.6
	}
}

void UMjOverlayRenderer::DrawContacts(const FMjRenderSnapshot& Snap, bool bPoints, bool bForces, bool bSplit)
{
	for (const FMjContactViz& C : Snap.Contacts)
	{
		const FVector Pos = URLabAxisConv::MjPositionToUe(C.Pos) + SceneOrigin;
		if (bPoints)
		{
			EmitSphere(MainLayer, Pos, 2.0f, FColor::Cyan, false);
		}
		if (!bForces)
		{
			continue;
		}

		auto DrawWrench = [&](const mjtNum InForce[3], const FColor& Colour, float HeadCm) {
			// Rotate a contact-frame force into world: the frame's rows are its
			// world-space axes, so world_f = sum_i frame_row_i * force_i.
			mjtNum FW[3] = {0, 0, 0};
			for (int32 k = 0; k < 3; ++k)
			{
				FW[k] = C.Frame[0 * 3 + k] * InForce[0] + C.Frame[1 * 3 + k] * InForce[1]
						+ C.Frame[2 * 3 + k] * InForce[2];
			}
			const FVector Dir = URLabAxisConv::MjDirectionToUe(FW);
			const float LenCm = FMath::Clamp(static_cast<float>(Dir.Size()) * 2.0f, 0.0f, 200.0f);
			if (LenCm > 0.5f)
			{
				const FVector End = Pos + Dir.GetSafeNormal() * LenCm;
				EmitArrow(MainLayer, Pos, End, 1.0f, HeadCm, Colour, false);
			}
		};

		if (bSplit)
		{
			// Contact frame row 0 is the normal, rows 1-2 the tangents; the force
			// vector's first component is normal, the other two are friction.
			const mjtNum Normal[3] = {C.Force[0], 0, 0};
			const mjtNum Tangent[3] = {0, C.Force[1], C.Force[2]};
			DrawWrench(Normal, FColor::Red, 4.0f);
			DrawWrench(Tangent, FColor(64, 160, 255), 3.0f);
		}
		else
		{
			DrawWrench(C.Force, FColor::Red, 3.0f);
		}
	}
}

void UMjOverlayRenderer::DrawPerturb(const FMjRenderSnapshot& Snap)
{
	if (PerturbBodyId < 0 || !Snap.XPos.IsValidIndex(PerturbBodyId * 3 + 2))
	{
		return;
	}
	const FVector BodyPos = URLabAxisConv::MjPositionToUe(&Snap.XPos[PerturbBodyId * 3]) + SceneOrigin;

	if (FlagSet(Flags.VisFlags, mjVIS_PERTOBJ))
	{
		// Selection marker: custom-depth enabled so an outline post-process can pick
		// it out (outline material is the flagged authored asset).
		EmitSphere(MainLayer, BodyPos, 6.0f, FColor::Yellow, false, /*bCustomDepth=*/true);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_PERTFORCE))
	{
		const FVector Dir = URLabAxisConv::MjDirectionToUe(PerturbForce);
		const float LenCm = FMath::Clamp(static_cast<float>(Dir.Size()) * 2.0f, 0.0f, 200.0f);
		if (LenCm > 0.5f)
		{
			const FVector End = BodyPos + Dir.GetSafeNormal() * LenCm;
			EmitArrow(MainLayer, BodyPos, End, 1.5f, 5.0f, FColor::Orange, false);
		}
	}
}

void UMjOverlayRenderer::DrawCameras(const FMjRenderSnapshot& Snap)
{
	const mjModel* M = Model;
	const mjtNum Depth = 0.15; // metres, purely a visual reach
	for (int32 C = 0; C < static_cast<int32>(M->ncam); ++C)
	{
		if (!Snap.CamXPos.IsValidIndex(C * 3 + 2) || !Snap.CamXMat.IsValidIndex(C * 9 + 8))
		{
			continue;
		}
		const mjtNum* P = &Snap.CamXPos[C * 3];
		const mjtNum* R = &Snap.CamXMat[C * 9];
		mjtNum X[3], Y[3], Z[3];
		MatColumn(R, 0, X);
		MatColumn(R, 1, Y);
		MatColumn(R, 2, Z);

		double Aspect = 1.0;
		if (M->cam_resolution && M->cam_resolution[C * 2 + 1] > 0)
		{
			Aspect = static_cast<double>(M->cam_resolution[C * 2 + 0]) / static_cast<double>(M->cam_resolution[C * 2 + 1]);
		}
		const double Fovy = M->cam_fovy ? FMath::DegreesToRadians(M->cam_fovy[C]) : FMath::DegreesToRadians(45.0);
		const double HalfH = Depth * FMath::Tan(Fovy * 0.5);
		const double HalfW = HalfH * Aspect;

		auto ToUe = [&](const mjtNum W[3]) { return URLabAxisConv::MjPositionToUe(W) + SceneOrigin; };
		const FVector Apex = ToUe(P);

		FVector Corners[4];
		const double Signs[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
		for (int32 i = 0; i < 4; ++i)
		{
			mjtNum WPt[3];
			for (int32 k = 0; k < 3; ++k)
			{
				WPt[k] = P[k] - Z[k] * Depth + X[k] * (Signs[i][0] * HalfW) + Y[k] * (Signs[i][1] * HalfH);
			}
			Corners[i] = ToUe(WPt);
		}

		const FColor Colour(80, 200, 255);
		EmitSphere(MainLayer, Apex, 2.0f, Colour, false);
		for (int32 i = 0; i < 4; ++i)
		{
			EmitLine(MainLayer, Apex, Corners[i], 1.0f, Colour);
			EmitLine(MainLayer, Corners[i], Corners[(i + 1) % 4], 1.0f, Colour);
		}
	}
}

void UMjOverlayRenderer::DrawLights(const FMjRenderSnapshot& Snap)
{
	const mjModel* M = Model;
	for (int32 L = 0; L < static_cast<int32>(M->nlight); ++L)
	{
		const int32 Bid = M->light_bodyid ? M->light_bodyid[L] : -1;
		if (Bid < 0 || !Snap.XPos.IsValidIndex(Bid * 3 + 2) || !Snap.XQuat.IsValidIndex(Bid * 4 + 3))
		{
			continue;
		}
		const mjtNum* Bp = &Snap.XPos[Bid * 3];
		const mjtNum* Bq = &Snap.XQuat[Bid * 4];

		mjtNum LocalPos[3] = {0, 0, 0};
		mjtNum LocalDir[3] = {0, 0, -1};
		if (M->light_pos)
		{
			for (int32 k = 0; k < 3; ++k)
				LocalPos[k] = M->light_pos[L * 3 + k];
		}
		if (M->light_dir)
		{
			for (int32 k = 0; k < 3; ++k)
				LocalDir[k] = M->light_dir[L * 3 + k];
		}

		mjtNum RotPos[3], WorldPos[3], WorldDir[3];
		mju_rotVecQuat(RotPos, LocalPos, Bq);
		for (int32 k = 0; k < 3; ++k)
			WorldPos[k] = Bp[k] + RotPos[k];
		mju_rotVecQuat(WorldDir, LocalDir, Bq);

		const FVector Pos = URLabAxisConv::MjPositionToUe(WorldPos) + SceneOrigin;
		const FVector Dir = URLabAxisConv::MjDirectionToUe(WorldDir).GetSafeNormal();

		const FColor Colour(255, 240, 120);
		EmitSphere(MainLayer, Pos, 2.0f, Colour, false);
		if (!Dir.IsNearlyZero())
		{
			EmitArrow(MainLayer, Pos, Pos + Dir * 20.0f, 1.0f, 3.0f, Colour, false);
		}
	}
}

void UMjOverlayRenderer::DrawActuators(const FMjRenderSnapshot& Snap)
{
	const mjModel* M = Model;
	for (int32 A = 0; A < static_cast<int32>(M->nu); ++A)
	{
		if (!GroupVisible(Flags.ActuatorGroup, M->actuator_group ? M->actuator_group[A] : 0))
		{
			continue;
		}

		FVector Anchor;
		FVector Axis;
		const int32 Trn = M->actuator_trntype ? M->actuator_trntype[A] : -1;
		if (Trn == mjTRN_JOINT || Trn == mjTRN_JOINTINPARENT)
		{
			const int32 Jid = M->actuator_trnid[A * 2];
			if (Jid < 0 || !Snap.JntXAnchor.IsValidIndex(Jid * 3 + 2) || !Snap.JntXAxis.IsValidIndex(Jid * 3 + 2))
				continue;
			Anchor = URLabAxisConv::MjPositionToUe(&Snap.JntXAnchor[Jid * 3]) + SceneOrigin;
			Axis = URLabAxisConv::MjDirectionToUe(&Snap.JntXAxis[Jid * 3]).GetSafeNormal();
		}
		else if (Trn == mjTRN_SITE)
		{
			const int32 Sid = M->actuator_trnid[A * 2];
			if (Sid < 0 || !Snap.SiteXPos.IsValidIndex(Sid * 3 + 2) || !Snap.SiteXMat.IsValidIndex(Sid * 9 + 8))
				continue;
			Anchor = URLabAxisConv::MjPositionToUe(&Snap.SiteXPos[Sid * 3]) + SceneOrigin;
			mjtNum ZAxis[3];
			MatColumn(&Snap.SiteXMat[Sid * 9], 2, ZAxis);
			Axis = URLabAxisConv::MjDirectionToUe(ZAxis).GetSafeNormal();
		}
		else
		{
			continue;
		}

		if (Axis.IsNearlyZero())
			Axis = FVector::UpVector;

		double Norm = 0.5;
		const double Ctrl = Snap.Ctrl.IsValidIndex(A) ? Snap.Ctrl[A] : 0.0;
		if (M->actuator_ctrllimited && M->actuator_ctrllimited[A] && M->actuator_ctrlrange)
		{
			const double Lo = M->actuator_ctrlrange[A * 2 + 0];
			const double Hi = M->actuator_ctrlrange[A * 2 + 1];
			if (Hi > Lo)
				Norm = FMath::Clamp((Ctrl - Lo) / (Hi - Lo), 0.0, 1.0);
		}
		else if (M->actuator_forcerange && Snap.ActuatorForce.IsValidIndex(A))
		{
			const double Lo = M->actuator_forcerange[A * 2 + 0];
			const double Hi = M->actuator_forcerange[A * 2 + 1];
			if (Hi > Lo)
				Norm = FMath::Clamp((Snap.ActuatorForce[A] - Lo) / (Hi - Lo), 0.0, 1.0);
		}

		const float LenCm = 3.0f + 12.0f * static_cast<float>(Norm);
		const FColor Colour = FLinearColor::LerpUsingHSV(
			FLinearColor(0.1f, 0.3f, 1.0f), FLinearColor(1.0f, 0.15f, 0.05f), static_cast<float>(Norm))
								  .ToFColor(false);
		EmitLine(MainLayer, Anchor, Anchor + Axis * LenCm, 3.0f, Colour);
		EmitSphere(MainLayer, Anchor, 2.0f, Colour, false);
	}
}

void UMjOverlayRenderer::DrawTendons(const FMjRenderSnapshot& Snap)
{
	if (Snap.TenWrapAdr.Num() == 0 || Snap.WrapXPos.Num() == 0)
	{
		return;
	}
	const mjModel* M = Model;
	const FColor Colour(255, 160, 64);
	for (int32 T = 0; T < static_cast<int32>(M->ntendon); ++T)
	{
		if (!GroupVisible(Flags.TendonGroup, M->tendon_group ? M->tendon_group[T] : 0))
		{
			continue;
		}
		if (!Snap.TenWrapAdr.IsValidIndex(T) || !Snap.TenWrapNum.IsValidIndex(T))
		{
			continue;
		}
		const int32 Adr = Snap.TenWrapAdr[T];
		const int32 Num = Snap.TenWrapNum[T];
		for (int32 j = Adr; j + 1 < Adr + Num; ++j)
		{
			if (Snap.WrapObj.IsValidIndex(j) && Snap.WrapObj[j] == -2)
				continue;
			if (Snap.WrapObj.IsValidIndex(j + 1) && Snap.WrapObj[j + 1] == -2)
				continue;
			if (!Snap.WrapXPos.IsValidIndex((j + 1) * 3 + 2))
				continue;
			const FVector A = URLabAxisConv::MjPositionToUe(&Snap.WrapXPos[j * 3]) + SceneOrigin;
			const FVector B = URLabAxisConv::MjPositionToUe(&Snap.WrapXPos[(j + 1) * 3]) + SceneOrigin;
			EmitLine(MainLayer, A, B, 2.0f, Colour);
		}
	}
}

void UMjOverlayRenderer::DrawRangefinders(const FMjRenderSnapshot& Snap)
{
	const mjModel* M = Model;
	for (int32 S = 0; S < static_cast<int32>(M->nsensor); ++S)
	{
		if (M->sensor_type[S] != mjSENS_RANGEFINDER || M->sensor_objtype[S] != mjOBJ_SITE)
		{
			continue;
		}
		const int32 Sid = M->sensor_objid[S];
		if (Sid < 0 || !Snap.SiteXPos.IsValidIndex(Sid * 3 + 2) || !Snap.SiteXMat.IsValidIndex(Sid * 9 + 8))
		{
			continue;
		}
		const int32 Adr = M->sensor_adr[S];
		const double Dist = Snap.SensorData.IsValidIndex(Adr) ? Snap.SensorData[Adr] : -1.0;

		const mjtNum* Op = &Snap.SiteXPos[Sid * 3];
		mjtNum Zaxis[3];
		MatColumn(&Snap.SiteXMat[Sid * 9], 2, Zaxis);
		const FVector Origin = URLabAxisConv::MjPositionToUe(Op) + SceneOrigin;

		if (Dist >= 0.0)
		{
			mjtNum Hit[3];
			for (int32 k = 0; k < 3; ++k)
				Hit[k] = Op[k] + Zaxis[k] * Dist;
			const FVector End = URLabAxisConv::MjPositionToUe(Hit) + SceneOrigin;
			EmitLine(MainLayer, Origin, End, 1.0f, FColor(64, 255, 64));
			EmitSphere(MainLayer, End, 2.0f, FColor(64, 255, 64), false);
		}
		else
		{
			const FVector Dir = URLabAxisConv::MjDirectionToUe(Zaxis).GetSafeNormal();
			EmitLine(MainLayer, Origin, Origin + Dir * 10.0f, 1.0f, FColor(64, 128, 64));
		}
	}
}

void UMjOverlayRenderer::DrawConstraints(const FMjRenderSnapshot& Snap)
{
	const mjModel* M = Model;
	const FColor Colour(255, 64, 255);
	for (int32 E = 0; E < static_cast<int32>(M->neq); ++E)
	{
		const int32 Type = M->eq_type[E];
		if (Type != mjEQ_CONNECT && Type != mjEQ_WELD)
		{
			continue;
		}
		if (M->eq_objtype && M->eq_objtype[E] != mjOBJ_BODY)
		{
			continue;
		}
		const int32 B1 = M->eq_obj1id[E];
		const int32 B2 = M->eq_obj2id[E];

		auto BodyPoint = [&](int32 B, FVector& Out) -> bool {
			if (B <= 0)
			{
				Out = SceneOrigin;
				return true;
			}
			if (!Snap.XPos.IsValidIndex(B * 3 + 2))
				return false;
			Out = URLabAxisConv::MjPositionToUe(&Snap.XPos[B * 3]) + SceneOrigin;
			return true;
		};

		FVector P1, P2;
		if (!BodyPoint(B1, P1) || !BodyPoint(B2, P2))
			continue;
		EmitLine(MainLayer, P1, P2, 1.5f, Colour);
		EmitSphere(MainLayer, P1, 3.0f, Colour, false);
		EmitSphere(MainLayer, P2, 3.0f, Colour, false);
	}
}

void UMjOverlayRenderer::DrawStaticBodies(const FMjRenderSnapshot& Snap)
{
	const mjModel* M = Model;
	for (int32 B = 1; B < static_cast<int32>(M->nbody); ++B)
	{
		if (M->body_dofnum && M->body_dofnum[B] != 0)
		{
			continue;
		}
		if (!Snap.XPos.IsValidIndex(B * 3 + 2))
		{
			continue;
		}
		const FVector Pos = URLabAxisConv::MjPositionToUe(&Snap.XPos[B * 3]) + SceneOrigin;
		EmitBox(MainLayer, Pos, FVector(2.0f), FQuat::Identity, FColor(160, 160, 160), false);
	}
}

void UMjOverlayRenderer::DrawAutoConnect(const FMjRenderSnapshot& Snap)
{
	const mjModel* M = Model;
	const FColor Colour(120, 120, 255);
	for (int32 B = 1; B < static_cast<int32>(M->nbody); ++B)
	{
		const int32 Parent = M->body_parentid ? M->body_parentid[B] : -1;
		if (Parent < 0 || !Snap.XPos.IsValidIndex(B * 3 + 2) || !Snap.XPos.IsValidIndex(Parent * 3 + 2))
		{
			continue;
		}
		const FVector Child = URLabAxisConv::MjPositionToUe(&Snap.XPos[B * 3]) + SceneOrigin;
		const FVector Par = URLabAxisConv::MjPositionToUe(&Snap.XPos[Parent * 3]) + SceneOrigin;
		EmitLine(MainLayer, Par, Child, 1.0f, Colour);
	}
}

void UMjOverlayRenderer::DrawWrenchGlyphs(const FMjRenderSnapshot& Snap)
{
	// The one net-new-vs-upstream overlay (source-of-truth 8.5): a true 6-DOF wrench
	// glyph -- a force arrow PLUS a curl/torus-about-axis torque glyph -- for the
	// contact torque (confrc[3:6]) and applied torque (xfrc[3:6]) that MuJoCo computes
	// but never draws. Force scaling / colours follow the model's own vis.* so an XML
	// restyle carries; the torque half is scaled by vis.map.torque and coloured
	// vis.rgba.contacttorque, exactly the unused upstream slot.
	const mjModel* M = Model;

	const float MapForce = (M->vis.map.force > 0.f) ? M->vis.map.force : 0.005f;
	const float MapTorque = (M->vis.map.torque > 0.f) ? M->vis.map.torque : 0.1f;

	const FColor ContactForceColor = RgbaToColor(M->vis.rgba.contactforce, FColor(179, 230, 230, 255));
	const FColor ContactTorqueColor = RgbaToColor(M->vis.rgba.contacttorque, FColor(230, 179, 230, 255));
	const FColor AppliedForceColor = RgbaToColor(M->vis.rgba.force, FColor(255, 128, 128, 255));
	// No dedicated "applied torque" slot upstream; reuse the contact-torque colour so
	// every torque curl reads the same, per the plan.
	const FColor AppliedTorqueColor = ContactTorqueColor;

	// Force arrow: |force| (world) mapped to cm, clamped to the same visible band the
	// existing contact/perturb arrows use. Torque ring radius: |torque|*vis.map.torque
	// (metres) -> cm, similarly clamped.
	auto ForceLenCm = [&](const FVector& Fw) -> float {
		return FMath::Clamp(static_cast<float>(Fw.Size()) * (MapForce / 0.005f) * 2.0f, 0.0f, 200.0f);
	};
	auto TorqueRadiusCm = [&](const FVector& Tw) -> float {
		return FMath::Clamp(static_cast<float>(Tw.Size()) * MapTorque * 100.0f, 0.0f, 120.0f);
	};

	auto EmitWrench = [&](const FVector& OriginUE, const FVector& ForceUE, const FVector& TorqueUE,
						  const FColor& FColour, const FColor& TColour) {
		const float FLen = ForceLenCm(ForceUE);
		if (FLen > 0.5f)
		{
			const FVector End = OriginUE + ForceUE.GetSafeNormal() * FLen;
			EmitArrow(MainLayer, OriginUE, End, 1.0f, 3.0f, FColour, false);
		}
		const float TRad = TorqueRadiusCm(TorqueUE);
		if (TRad > 1.0f)
		{
			const float Tube = FMath::Max(TRad * 0.08f, 0.5f);
			EmitTorqueGlyph(MainLayer, OriginUE, TorqueUE, TRad, Tube, TColour, false);
		}
	};

	// --- Contact wrenches: force[6] is (force xyz, torque xyz) in the contact frame.
	for (const FMjContactViz& C : Snap.Contacts)
	{
		const FVector Pos = URLabAxisConv::MjPositionToUe(C.Pos) + SceneOrigin;
		// contact frame -> world: the frame's rows are its world-space axes, so
		// world_v = sum_i frame_row_i * v_i (same rotation the force overlay uses).
		mjtNum Fw[3] = {0, 0, 0};
		mjtNum Tw[3] = {0, 0, 0};
		for (int32 k = 0; k < 3; ++k)
		{
			Fw[k] = C.Frame[0 * 3 + k] * C.Force[0] + C.Frame[1 * 3 + k] * C.Force[1]
					+ C.Frame[2 * 3 + k] * C.Force[2];
			Tw[k] = C.Frame[0 * 3 + k] * C.Force[3] + C.Frame[1 * 3 + k] * C.Force[4]
					+ C.Frame[2 * 3 + k] * C.Force[5];
		}
		EmitWrench(Pos, URLabAxisConv::MjDirectionToUe(Fw), URLabAxisConv::MjDirectionToUe(Tw),
			ContactForceColor, ContactTorqueColor);
	}

	// --- Applied wrenches: xfrc_applied is 6*nbody (force xyz, torque xyz), world.
	for (int32 B = 1; B < static_cast<int32>(M->nbody); ++B)
	{
		if (!Snap.XfrcApplied.IsValidIndex(B * 6 + 5) || !Snap.XPos.IsValidIndex(B * 3 + 2))
		{
			continue;
		}
		const mjtNum* W = &Snap.XfrcApplied[B * 6];
		mjtNum Mag2 = 0.0;
		for (int32 k = 0; k < 6; ++k)
		{
			Mag2 += W[k] * W[k];
		}
		if (Mag2 <= KINDA_SMALL_NUMBER)
		{
			continue;
		}
		const FVector Pos = URLabAxisConv::MjPositionToUe(&Snap.XPos[B * 3]) + SceneOrigin;
		const mjtNum Fw[3] = {W[0], W[1], W[2]};
		const mjtNum Tw[3] = {W[3], W[4], W[5]};
		EmitWrench(Pos, URLabAxisConv::MjDirectionToUe(Fw), URLabAxisConv::MjDirectionToUe(Tw),
			AppliedForceColor, AppliedTorqueColor);
	}
}
