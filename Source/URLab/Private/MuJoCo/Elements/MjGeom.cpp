// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Elements/MjGeom.h"

#include "Chaos/TriangleMeshImplicitObject.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "Misc/ScopedSlowTask.h"
#include "PhysicsEngine/BodySetup.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/MjRenderSnapshot.h"
#include "MuJoCo/Spec/MjEffective.h"
#include "MuJoCo/Spec/MjAssetResolve.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "Utils/IO.h"
#include "Utils/MeshUtils.h"
#include "Utils/URLabLogging.h"

#if WITH_EDITOR
#include "AssetToolsModule.h"
#include "AutomatedAssetImportData.h"
#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "IAssetTools.h"
#include "Kismet2/BlueprintEditorUtils.h"
#endif

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{

// The engine's basic shapes are 100 cm across and 100 cm tall, so a component
// scale of 1 is a 50 cm half-extent in every direction. MJCF sizes are metres.
constexpr double kCmPerM = 100.0;
constexpr double kBaseHalf = 50.0;
constexpr double kSizeToScale = kCmPerM / kBaseHalf;
constexpr double kMinScaleZ = 0.001; // guard for the cap counter-scale divide

/** Which scale axes a shape lets the gizmo move independently. */
enum class EScaleLock : uint8
{
	/** All three, as a box wants. */
	Free,
	/** X and Y move together, so a round cross-section stays round. */
	RadialXY,
	/** One number for all three, so a sphere stays a sphere. */
	Uniform,
	/** X and Y are free and Z is pinned to 1: a plane has no thickness. */
	FlatXY,
};

/** One component of the relative scale, and the `size` slot that decides it. */
struct FSizeAxis
{
	/** 0, 1 or 2: X, Y or Z of the component's relative scale. */
	uint8 ScaleAxis = 0;

	/** Index into the element's MJCF `size` array. */
	uint8 SizeSlot = 0;
};

/**
 * Everything that used to be the difference between one geom subclass and the next.
 *
 * A row per shape, keyed by the `type` attribute. Six of the nine shapes have a
 * preview and the other three are here to say they have none, which is the
 * honest answer for a height field, a mesh and an SDF: each carries its picture
 * as a child component rather than as a scaled engine primitive.
 *
 * `Axes` is the size mapping, declared rather than computed. Both directions are
 * derived from it -- `size` to scale for the preview, scale to `size` for the
 * write-back -- so the round trip is consistent by construction and cannot drift
 * the way a forward function and a hand-written inverse would. The schema cannot
 * supply this: it says only `size : double[1..3] (writing=custom)`.
 */
struct FGeomShape
{
	/** Engine primitive the preview is built from; null when there is no preview. */
	const TCHAR* MeshPath = nullptr;

	/** Sphere for the capsule's two end caps; null for every other shape. */
	const TCHAR* CapMeshPath = nullptr;

	/** The size mapping, one row per axis the shape's `size` decides. */
	FSizeAxis Axes[3] = {};

	/** How many of `Axes` are live. Zero means the shape has no scale mapping. */
	uint8 AxisNum = 0;

	EScaleLock Lock = EScaleLock::Free;

	/**
	 * Metres a mapped `size` of zero previews at, or 0 when zero is simply zero.
	 *
	 * Only a plane needs it: MJCF spells "infinite in this direction" as a zero
	 * half-extent, and Unreal has to draw something finite. MuJoCo's own
	 * visualiser substitutes `vis.map.zfar * stat.extent`, neither of which
	 * exists before a compile, so the preview uses a fixed extent instead.
	 */
	double InfiniteExtent = 0.0;

	/**
	 * True when the scale gizmo must not author `size` back onto the spec.
	 *
	 * A plane's `size` is (half-x, half-y, grid-spacing): two of the three are
	 * dimensions and the third is not, and either dimension may be the zero
	 * that means infinite. There is no scale a drag could write that preserves
	 * all of that, so a plane previews from its size and never authors one.
	 */
	bool bSizeIsReadOnly = false;
};

const TCHAR* const kCubeMesh = TEXT("/Engine/BasicShapes/Cube.Cube");
const TCHAR* const kSphereMesh = TEXT("/Engine/BasicShapes/Sphere.Sphere");
const TCHAR* const kCylinderMesh = TEXT("/Engine/BasicShapes/Cylinder.Cylinder");
const TCHAR* const kPlaneMesh = TEXT("/Engine/BasicShapes/Plane.Plane");

/** Half-extent, in metres, an infinite plane previews at. */
constexpr double kInfinitePlaneHalfExtent = 10.0;

constexpr FGeomShape GGeomShapes[] = {
	/* plane     */ {kPlaneMesh, nullptr, {{0, 0}, {1, 1}}, 2, EScaleLock::FlatXY, kInfinitePlaneHalfExtent, true},
	/* hfield    */ {},
	/* sphere    */ {kSphereMesh, nullptr, {{0, 0}}, 1, EScaleLock::Uniform},
	/* capsule   */ {kCylinderMesh, kSphereMesh, {{0, 0}, {2, 1}}, 2, EScaleLock::RadialXY},
	/* ellipsoid */ {kSphereMesh, nullptr, {{0, 0}, {1, 1}, {2, 2}}, 3, EScaleLock::Free},
	/* cylinder  */ {kCylinderMesh, nullptr, {{0, 0}, {2, 1}}, 2, EScaleLock::RadialXY},
	/* box       */ {kCubeMesh, nullptr, {{0, 0}, {1, 1}, {2, 2}}, 3, EScaleLock::Free},
	/* mesh      */ {},
	/* sdf       */ {},
};

static_assert(static_cast<int32>(EMjGeomType::sdf) + 1 == static_cast<int32>(UE_ARRAY_COUNT(GGeomShapes)),
	"GGeomShapes is indexed by EMjGeomType and must have a row for every value, in declaration order");

/**
 * The size slots a shape maps must be 0..AxisNum-1 with no gaps.
 *
 * The write-back authors the whole array for the type, so a gap would leave a
 * slot MuJoCo reads holding a zero nobody wrote.
 */
constexpr bool SizeSlotsAreContiguous()
{
	for (const FGeomShape& Shape : GGeomShapes)
	{
		uint32 Seen = 0;
		for (uint8 Index = 0; Index < Shape.AxisNum; ++Index)
		{
			Seen |= 1u << Shape.Axes[Index].SizeSlot;
		}
		if (Seen != (1u << Shape.AxisNum) - 1u)
		{
			return false;
		}
	}
	return true;
}

static_assert(SizeSlotsAreContiguous(), "a shape's size slots must be 0..AxisNum-1");

const FGeomShape& ShapeFor(EMjGeomType Type)
{
	static const FGeomShape NoPreview;
	const int32 Index = static_cast<int32>(Type);
	if (Index < 0 || Index >= static_cast<int32>(UE_ARRAY_COUNT(GGeomShapes)))
	{
		return NoPreview;
	}
	return GGeomShapes[Index];
}

/**
 * Force the scale onto what the shape can actually represent.
 *
 * X is the master, which is the rule the user sees: drag a sphere's Y handle and
 * all three snap together on the spot. Applied in both directions, so the axes
 * the table leaves unmapped are filled here rather than restated per shape, and
 * a component the shape cannot express can never reach the spec.
 */
void ApplyLock(EScaleLock Lock, FVector& Scale)
{
	switch (Lock)
	{
		case EScaleLock::Uniform:
			Scale.Y = Scale.Z = Scale.X;
			break;
		case EScaleLock::RadialXY:
			Scale.Y = Scale.X;
			break;
		case EScaleLock::FlatXY:
			Scale.Z = 1.0;
			break;
		case EScaleLock::Free:
			break;
	}
}

// A size too short for its type is unresolvable, and so is a non-positive one.
// Both come back as a zero scale, which the caller reads as "leave it alone" --
// except where the shape declares that a zero size means infinite, which is the
// plane's spelling and previews at the extent the row names.
FVector ScaleFromSize(const FGeomShape& Shape, const TArray<double>& Size)
{
	if (Shape.AxisNum == 0)
	{
		return FVector::ZeroVector;
	}
	FVector Scale = FVector::ZeroVector;
	for (uint8 Index = 0; Index < Shape.AxisNum; ++Index)
	{
		const FSizeAxis& Axis = Shape.Axes[Index];
		if (Size.Num() <= static_cast<int32>(Axis.SizeSlot))
		{
			return FVector::ZeroVector;
		}
		const double Extent = Size[Axis.SizeSlot];
		Scale[Axis.ScaleAxis] =
			(Extent == 0.0 ? Shape.InfiniteExtent : Extent) * kSizeToScale;
	}
	ApplyLock(Shape.Lock, Scale);
	return Scale;
}

/** The inverse of ScaleFromSize, off the same rows. Authors the whole array. */
TArray<double> SizeFromScale(const FGeomShape& Shape, FVector Scale)
{
	ApplyLock(Shape.Lock, Scale);
	TArray<double> Size;
	Size.SetNumZeroed(Shape.AxisNum);
	for (uint8 Index = 0; Index < Shape.AxisNum; ++Index)
	{
		const FSizeAxis& Axis = Shape.Axes[Index];
		Size[Axis.SizeSlot] = Scale[Axis.ScaleAxis] / kSizeToScale;
	}
	return Size;
}

/** A geom's `type` and `size` as the compiler will see them. */
struct FGeomShapeState
{
	EMjGeomType Type = EMjGeomType::sphere;
	TArray<double> Size;
};

/**
 * `type` and `size` with the default-class chain resolved.
 *
 * A geom that says only `class="collision"` is the common case in a menagerie
 * model, and previewing it from its own storage alone would draw a sphere of no
 * size. The chain walk runs only for the parts the element did not author.
 */
FGeomShapeState EffectiveShapeOf(const UMjGeomBase& Geom)
{
	FGeomShapeState Out;
	bool bTypeSet = Geom.Type.IsSet();
	bool bSizeSet = Geom.Size.IsSet();
	if (bTypeSet)
	{
		Out.Type = Geom.Type.GetValue();
	}
	if (bSizeSet)
	{
		Out.Size = Geom.Size.GetValue();
	}

#if URLAB_MJ_GEN
	if (!bTypeSet || !bSizeSet)
	{
		urlab::spec::WithEffectiveDoc(Geom, [&](auto& Effective) {
			Effective.ForEachLayer(Geom, [&](const auto& Layer) {
				if (!bTypeSet && Layer.Type.IsSet())
				{
					Out.Type = Layer.Type.GetValue();
					bTypeSet = true;
				}
				if (!bSizeSet && Layer.Size.IsSet())
				{
					Out.Size = Layer.Size.GetValue();
					bSizeSet = true;
				}
				return bTypeSet && bSizeSet;
			});
		});
	}
#endif

	return Out;
}

/**
 * The engine driving `Node`, and the compiled id `Node` bound to.
 *
 * Says nothing about whether the id is in range: the snapshot arrays the callers
 * index are sized per family and carry their own bound, so they check there.
 */
bool ResolveBoundGeom(const UMjNodeComponent* Node, UMjPhysicsEngine*& OutEngine, int32& OutId)
{
	if (Node == nullptr)
	{
		return false;
	}
	const TOptional<int32>& Id = Node->GetBoundId();
	if (!Id.IsSet() || Id.GetValue() < 0)
	{
		return false;
	}
	UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(Node);
	if (Engine == nullptr)
	{
		return false;
	}
	OutEngine = Engine;
	OutId = Id.GetValue();
	return true;
}

#if WITH_EDITOR

/** The element's authored MJCF name, falling back to the component's own. */
FString ElementName(const UMjNodeComponent& Node)
{
	if (Node.MjName.IsSet() && !Node.MjName.GetValue().IsEmpty())
	{
		return Node.MjName.GetValue();
	}
	return Node.GetName();
}

#endif // WITH_EDITOR

} // namespace

// --- Preview ---------------------------------------------------------------- //

void UMjGeom::OnRegister()
{
	// The base drives pose and scale from the spec and takes the write-back
	// baseline with it; the preview parts are built after, because a part made
	// before this geom registered has nothing to attach to.
	Super::OnRegister();

	RebuildVisualizer();
}

void UMjGeom::RefreshPresentation()
{
	// The pose and size come from the base; the meshes and their colour are the
	// half that only exists here, and both halves can move when a class or a
	// material somewhere else in the spec does.
	Super::RefreshPresentation();
	RebuildVisualizer();
}

bool UMjGeom::HasScaleMapping() const
{
	const FGeomShape& Shape = ShapeFor(EffectiveShapeOf(*this).Type);
	return Shape.AxisNum > 0 && !Shape.bSizeIsReadOnly;
}

bool UMjGeom::TryPreviewScaleFromSpec(FVector& OutScale) const
{
	const FGeomShapeState State = EffectiveShapeOf(*this);
	const FGeomShape& Shape = ShapeFor(State.Type);
	if (Shape.AxisNum == 0)
	{
		return false;
	}

	// An unresolvable size -- too short for the type, or non-positive -- leaves
	// the scale alone rather than collapsing the geom to nothing.
	const FVector Scale = ScaleFromSize(Shape, State.Size);
	if (Scale.GetMin() <= 0.0)
	{
		return false;
	}
	OutScale = Scale;
	return true;
}

void UMjGeom::ConstrainPreviewScale()
{
	ApplyAxisLock();
	UpdateCapTransforms();
}

void UMjGeom::WriteBackScale(const FVector& Scale)
{
	const FGeomShape& Shape = ShapeFor(EffectiveShapeOf(*this).Type);
	if (Shape.AxisNum == 0 || Shape.bSizeIsReadOnly)
	{
		return;
	}
	Size = SizeFromScale(Shape, Scale);
}

UStaticMeshComponent* UMjGeom::MakeVisualizerPart(FName PartName, const TCHAR* MeshPath)
{
	// A unique name rather than the plain one: a rebuild leaves the previous part
	// destroyed but not yet collected, and it still holds the name it was made with.
	UStaticMeshComponent* Part = NewObject<UStaticMeshComponent>(
		this, MakeUniqueObjectName(this, UStaticMeshComponent::StaticClass(), PartName));
	if (Part == nullptr)
	{
		return nullptr;
	}

	Part->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
	Part->SetCollisionResponseToAllChannels(ECR_Overlap);

	// Null for a mesh geom, whose asset is not a constant the shape table can
	// hold; the caller assigns that one. Asking the loader for a null path
	// only earns a "failed to find object StaticMesh None" in the log.
	if (MeshPath != nullptr)
	{
		if (UStaticMesh* Asset = LoadObject<UStaticMesh>(nullptr, MeshPath))
		{
			Part->SetStaticMesh(Asset);
		}
	}
	return Part;
}

void UMjGeom::DestroyVisualizer()
{
	auto Drop = [](TObjectPtr<UStaticMeshComponent>& Part) {
		if (IsValid(Part))
		{
			Part->DestroyComponent();
		}
		Part = nullptr;
	};

	Drop(VisualizerMesh);
	Drop(VisualizerCapTop);
	Drop(VisualizerCapBottom);
	VisualizerScale = FVector::OneVector;
}

FString UMjGeom::EffectiveMeshName() const
{
	FString Name = Mesh.Get(FString());
#if URLAB_MJ_GEN
	if (Mesh.IsSet())
	{
		return Name;
	}
	// `mesh` is as defaultable as `material` is, and a menagerie model that
	// puts `class="visual"` on the geom and everything else on the class is the
	// ordinary case rather than the exception.
	urlab::spec::WithEffectiveDoc(*this, [&](auto& Effective) {
		Effective.ForEachLayer(static_cast<const UMjGeomBase&>(*this), [&](const auto& Layer) {
			if (!Layer.Mesh.IsSet())
			{
				return false;
			}
			Name = Layer.Mesh.GetValue();
			return true;
		});
	});
#endif
	return Name;
}

void UMjGeom::RebuildVisualizer()
{
	// A `<default>` partial is an inheritance template, not a geom: MuJoCo never
	// places it and never draws it, and its `size` is whatever the class chose to
	// declare -- humanoid's `<default class="body">` declares a capsule and no
	// size at all, which previews at the engine primitive's own metre scale and
	// puts a person-sized capsule at the origin that belongs to nothing. Nor can
	// it be hidden: it is bound to no compiled geom, so every toggle that works
	// off the compiled model passes it by.
	if (IsClassPartial())
	{
		DestroyVisualizer();
		return;
	}

	const EMjGeomType EffectiveType = EffectiveShapeOf(*this).Type;
	if (!BuiltType.IsSet() || BuiltType.GetValue() != EffectiveType)
	{
		DestroyVisualizer();
		BuiltType = EffectiveType;
	}

	// A mesh geom's picture is not an engine primitive scaled by `size`; it is
	// the asset the import pass made from the file its `<mesh>` element names.
	// The shape table cannot hold it, because it is per element and only known
	// once the spec has been read.
	const FMjResolvedMesh SpecMesh = EffectiveType == EMjGeomType::mesh
		? MjResolveMesh(FSpecRef::OverOwner(this), EffectiveMeshName())
		: FMjResolvedMesh();
	VisualizerScale = SpecMesh.Asset != nullptr ? SpecMesh.Scale : FVector::OneVector;

	const FGeomShape& Shape = ShapeFor(EffectiveType);
	if (Shape.MeshPath == nullptr && SpecMesh.Asset == nullptr)
	{
		return;
	}

	if (!IsValid(VisualizerMesh))
	{
		VisualizerMesh = MakeVisualizerPart(TEXT("VisualizerMesh"), Shape.MeshPath);
	}
	if (SpecMesh.Asset != nullptr && IsValid(VisualizerMesh))
	{
		VisualizerMesh->SetStaticMesh(SpecMesh.Asset);
	}
	if (Shape.CapMeshPath != nullptr)
	{
		if (!IsValid(VisualizerCapTop))
		{
			VisualizerCapTop = MakeVisualizerPart(TEXT("VisualizerCapTop"), Shape.CapMeshPath);
		}
		if (!IsValid(VisualizerCapBottom))
		{
			VisualizerCapBottom = MakeVisualizerPart(TEXT("VisualizerCapBottom"), Shape.CapMeshPath);
		}
	}

	// Parts made before this geom itself registered have nothing to attach to yet;
	// they get picked up the next time through, from OnRegister.
	auto AttachIfNeeded = [this](UStaticMeshComponent* Part) {
		if (IsRegistered() && IsValid(Part) && !Part->IsRegistered())
		{
			Part->SetupAttachment(this);
			Part->RegisterComponent();
		}
	};
	AttachIfNeeded(VisualizerMesh);
	AttachIfNeeded(VisualizerCapTop);
	AttachIfNeeded(VisualizerCapBottom);

	ApplyOverrideMaterial(OverrideMaterial);
	ApplySpecMaterial();
	UpdateCapTransforms();
}

void UMjGeom::ApplySpecMaterial()
{
	// An explicit override is the user overruling the spec; leave it be.
	if (IsValid(OverrideMaterial))
	{
		return;
	}
	UMaterialInterface* Base = MjLoadMasterMaterial();
	if (!IsValid(Base))
	{
		return;
	}

	const FSpecRef Spec = FSpecRef::OverOwner(this);
	const FString MaterialName = EffectiveMaterialName();
	FMjMaterialValues Values;
	MjResolveMaterial(Spec, MaterialName, Values);

	// The two axes MuJoCo's `texuniform` scales by. The third is not used: the
	// mapping is planar in the geom's own frame, so a box's depth never enters.
	const FGeomShapeState Shape = EffectiveShapeOf(*this);
	const double Infinite = ShapeFor(Shape.Type).InfiniteExtent;
	auto Extent = [&](int32 Slot) {
		const double Value = Shape.Size.IsValidIndex(Slot) ? Shape.Size[Slot] : 0.0;
		return Value == 0.0 ? Infinite : Value;
	};
	const FVector2D GeomSize(Extent(0), Extent(1));

	const FLinearColor Color = GetEffectiveColor();

	// A fully transparent colour is how MJCF says "do not draw this": menagerie
	// models routinely give a collision or inertial proxy `rgba="0 0 0 0"` and
	// expect it to disappear. The preview is opaque, so an alpha of zero would
	// otherwise come out solid black -- the loudest possible way to draw
	// something meant to be invisible.
	const bool bDrawIt = Color.A > 0.0f;

	auto Dress = [&](UStaticMeshComponent* Part) {
		if (!IsValid(Part))
		{
			return;
		}
		Part->SetVisibility(bDrawIt);
		if (UMaterialInstanceDynamic* Instance = Part->CreateDynamicMaterialInstance(0, Base))
		{
			MjApplyMaterialParameters(*Instance, Values, Color, Spec, GeomSize);
		}
	};

	// An imported asset arrives with materials of its own, sometimes genuine
	// ones an artist authored, and replacing those with a flat MuJoCo colour the
	// spec never asked for would be a downgrade. So an asset-backed preview
	// is dressed only when the spec actually names a material; an engine
	// primitive, which has nothing to lose, is dressed either way.
	const bool bAssetBacked = Shape.Type == EMjGeomType::mesh;
	if (!bAssetBacked || !MaterialName.IsEmpty())
	{
		Dress(VisualizerMesh);
		Dress(VisualizerCapTop);
		Dress(VisualizerCapBottom);
	}

	// The same rule for a static mesh the user hung under the geom themselves,
	// which is the other way a mesh geom acquires a picture.
	if (MaterialName.IsEmpty())
	{
		return;
	}
	TArray<USceneComponent*> Children;
	GetChildrenComponents(/*bIncludeAllDescendants=*/true, Children);
	for (USceneComponent* Child : Children)
	{
		UStaticMeshComponent* ChildMesh = Cast<UStaticMeshComponent>(Child);
		if (ChildMesh != nullptr && ChildMesh != VisualizerMesh && ChildMesh != VisualizerCapTop &&
			ChildMesh != VisualizerCapBottom)
		{
			Dress(ChildMesh);
		}
	}
}

void UMjGeom::UpdateCapTransforms()
{
	if (IsValid(VisualizerMesh))
	{
		VisualizerMesh->SetRelativeLocation(FVector::ZeroVector);
		// One for a primitive, whose size rides on the geom component's own
		// scale; the mesh element's `scale` and the metre-to-centimetre factor
		// for an imported asset, which the geom's scale says nothing about.
		VisualizerMesh->SetRelativeScale3D(VisualizerScale);
	}

	if (!IsValid(VisualizerCapTop) && !IsValid(VisualizerCapBottom))
	{
		return;
	}

	const FVector ParentScale = GetRelativeScale3D();
	const double SafeZ = FMath::Max(FMath::Abs(ParentScale.Z), kMinScaleZ);
	const double CapZ = (ParentScale.Z >= 0.0 ? 1.0 : -1.0) * (ParentScale.X / SafeZ);
	const FVector CapScale(1.0, 1.0, CapZ);

	if (IsValid(VisualizerCapTop))
	{
		VisualizerCapTop->SetRelativeLocation(FVector(0.0, 0.0, kBaseHalf));
		VisualizerCapTop->SetRelativeScale3D(CapScale);
	}
	if (IsValid(VisualizerCapBottom))
	{
		VisualizerCapBottom->SetRelativeLocation(FVector(0.0, 0.0, -kBaseHalf));
		VisualizerCapBottom->SetRelativeScale3D(CapScale);
	}
}

void UMjGeom::SyncEditorScaleFromSize()
{
	FVector NewScale;
	if (!TryPreviewScaleFromSpec(NewScale))
	{
		return;
	}

	SetRelativeScale3D(NewScale);
	UpdateCapTransforms();
}

void UMjGeom::ApplyAxisLock()
{
	const FVector Scale = GetRelativeScale3D();
	FVector Locked = Scale;
	ApplyLock(ShapeFor(EffectiveShapeOf(*this).Type).Lock, Locked);
	if (!Locked.Equals(Scale))
	{
		SetRelativeScale3D(Locked);
	}
}

namespace
{

/**
 * The magic default both MuJoCo renderers test a colour against.
 *
 * It is `<geom rgba>`'s schema default, and it is also the value the Filament
 * renderer compares a MATERIAL's rgba with -- which is not that material's own
 * default (1 1 1 1). The asymmetry is deliberate upstream and copied here.
 */
const FLinearColor MjMagicDefaultRgba(0.5f, 0.5f, 0.5f, 1.0f);

/**
 * Whether a colour is that default, componentwise and exactly.
 *
 * Exactly, because the engine's test is exact (`rgba[0] != 0.5f || ...`) and an
 * authored 0.5 grey is meant to behave like an unauthored one. Not
 * `FLinearColor::Equals`, whose comparison is a strict `<` against the
 * tolerance -- so a tolerance of zero calls every colour different from itself.
 */
bool IsMagicDefaultRgba(const FLinearColor& Color)
{
	return Color.R == MjMagicDefaultRgba.R && Color.G == MjMagicDefaultRgba.G && Color.B == MjMagicDefaultRgba.B &&
		Color.A == MjMagicDefaultRgba.A;
}

} // namespace

FString UMjGeom::EffectiveMaterialName() const
{
	FString Name = Material.Get(FString());
#if URLAB_MJ_GEN
	if (Material.IsSet())
	{
		return Name;
	}
	// A menagerie model almost never puts `material` on the geom: MuJoCo's
	// humanoid puts it on a default class, which is why reading the geom's own
	// storage found nothing to resolve.
	urlab::spec::WithEffectiveDoc(*this, [&](auto& Effective) {
		Effective.ForEachLayer(static_cast<const UMjGeomBase&>(*this), [&](const auto& Layer) {
			if (!Layer.Material.IsSet())
			{
				return false;
			}
			Name = Layer.Material.GetValue();
			return true;
		});
	});
#endif
	return Name;
}

FLinearColor UMjGeom::GetEffectiveColor() const
{
	FLinearColor GeomRgba = Rgba.Get(MjMagicDefaultRgba);

#if URLAB_MJ_GEN
	if (!Rgba.IsSet())
	{
		urlab::spec::WithEffectiveDoc(*this, [&](auto& Effective) {
			Effective.ForEachLayer(static_cast<const UMjGeomBase&>(*this), [&](const auto& Layer) {
				if (!Layer.Rgba.IsSet())
				{
					return false;
				}
				GeomRgba = Layer.Rgba.GetValue();
				return true;
			});
		});
	}

	// Filament's rule (model_renderables.cc GetDefaultMaterial), not classic's:
	// start from the geom's rgba, and let the material's take it back when that
	// is anything other than the magic default. Classic decides the other way
	// round -- it starts from the material and lets a non-default geom rgba win
	// -- so the two disagree when both are explicitly non-default, and this
	// plugin follows Filament because Unreal is PBR and classic is not.
	FMjMaterialValues Values;
	if (MjResolveMaterial(FSpecRef::OverOwner(this), EffectiveMaterialName(), Values) &&
		!IsMagicDefaultRgba(Values.Rgba))
	{
		return Values.Rgba;
	}
#endif

	return GeomRgba;
}

void UMjGeom::ApplyOverrideMaterial(UMaterialInterface* InMaterial)
{
	if (!IsValid(InMaterial))
	{
		return;
	}

	auto Apply = [InMaterial](UStaticMeshComponent* Part) {
		if (IsValid(Part))
		{
			Part->SetMaterial(0, InMaterial);
		}
	};
	Apply(VisualizerMesh);
	Apply(VisualizerCapTop);
	Apply(VisualizerCapBottom);
}

void UMjGeom::SetGeomVisibility(bool bNewVisibility)
{
	TSet<UStaticMeshComponent*> Meshes;
	Meshes.Add(VisualizerMesh.Get());
	Meshes.Add(VisualizerCapTop.Get());
	Meshes.Add(VisualizerCapBottom.Get());

	// The child walk is what covers a mesh geom, whose picture is an imported
	// static mesh hanging under it rather than one of the parts above.
	TArray<USceneComponent*> Children;
	GetChildrenComponents(true, Children);
	for (USceneComponent* Child : Children)
	{
		if (UStaticMeshComponent* ChildMesh = Cast<UStaticMeshComponent>(Child))
		{
			Meshes.Add(ChildMesh);
		}
	}

	for (UStaticMeshComponent* Part : Meshes)
	{
		if (!IsValid(Part))
		{
			continue;
		}
		Part->SetVisibility(bNewVisibility, false);
		Part->bHiddenInGame = !bNewVisibility;

#if WITH_EDITOR
		Part->Modify();
		Part->MarkRenderStateDirty();
		Part->RecreateRenderState_Concurrent();
#endif
	}
}

#if WITH_EDITOR
void UMjGeom::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	// The base drives the preview transform and the write-back from the schema's
	// own field ids, `type` and `size` included. What is left here is the
	// picture: a different `type` is a different engine primitive.
	Super::PostEditChangeProperty(PropertyChangedEvent);
	RebuildVisualizer();
}
#endif

// --- Compiled state --------------------------------------------------------- //

void UMjGeom::UpdateGlobalTransform()
{
	UMjPhysicsEngine* Engine = nullptr;
	int32 Id = 0;
	if (!ResolveBoundGeom(this, Engine, Id))
	{
		return;
	}

	Engine->WithRenderState([this, Id](const FMjRenderSnapshot& Snap) {
		const int32 PosIdx = Id * 3;
		const int32 MatIdx = Id * 9;
		if (Snap.GeomXPos.Num() <= PosIdx + 2 || Snap.GeomXMat.Num() <= MatIdx + 8)
		{
			return;
		}

		const FVector WorldPos = URLabAxisConv::MjPositionToUe(&Snap.GeomXPos[PosIdx]);
		mjtNum Quat[4];
		mju_mat2Quat(Quat, const_cast<mjtNum*>(&Snap.GeomXMat[MatIdx]));
		const FQuat WorldRot = URLabAxisConv::MjQuatToUe(Quat);

		// A zero or non-finite snapshot row would write a degenerate transform that
		// NaN-floods the renderer (NIL LocalToWorld in the distance-field pass).
		// Skip the frame and name the offender once.
		if (WorldPos.ContainsNaN() || WorldRot.ContainsNaN() || WorldRot.SizeSquared() < KINDA_SMALL_NUMBER)
		{
			if (!bWarnedDegenerateTransform)
			{
				UE_LOG(LogURLabBind, Warning,
					TEXT("MjGeom::UpdateGlobalTransform - geom '%s' (id=%d) got a degenerate snapshot "
						 "transform (pos=%s quat=[%f %f %f %f]); skipping."),
					*GetName(), Id, *WorldPos.ToString(), Quat[0], Quat[1], Quat[2], Quat[3]);
				bWarnedDegenerateTransform = true;
			}
			return;
		}

		SetWorldLocation(WorldPos);
		SetWorldRotation(WorldRot);
	});
}

FVector UMjGeom::GetWorldLocation() const
{
	UMjPhysicsEngine* Engine = nullptr;
	int32 Id = 0;
	if (!ResolveBoundGeom(this, Engine, Id))
	{
		return GetComponentLocation();
	}

	FVector Out = GetComponentLocation();
	Engine->WithRenderState([Id, &Out](const FMjRenderSnapshot& Snap) {
		const int32 PosIdx = Id * 3;
		if (Snap.GeomXPos.Num() > PosIdx + 2)
		{
			Out = URLabAxisConv::MjPositionToUe(&Snap.GeomXPos[PosIdx]);
		}
	});
	return Out;
}

// --- Mesh decomposition ----------------------------------------------------- //

#if WITH_EDITOR

void UMjGeom::DecomposeMesh()
{
	if (EffectiveShapeOf(*this).Type != EMjGeomType::mesh)
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjGeom] DecomposeMesh: '%s' is not a mesh geom."), *GetName());
		return;
	}

	// The attachment hierarchy answers for a placed instance. A Blueprint template
	// is not attached to anything, so its children are only reachable through the
	// construction script's node tree.
	UStaticMeshComponent* SMC = nullptr;

	TArray<USceneComponent*> Children;
	GetChildrenComponents(true, Children);
	for (USceneComponent* Child : Children)
	{
		if (UStaticMeshComponent* Found = Cast<UStaticMeshComponent>(Child))
		{
			SMC = Found;
			break;
		}
	}

	UBlueprint* BP = nullptr;
	for (UObject* Outer = GetOuter(); Outer != nullptr; Outer = Outer->GetOuter())
	{
		if (UBlueprint* Found = Cast<UBlueprint>(Outer))
		{
			BP = Found;
			break;
		}
		if (UBlueprintGeneratedClass* BPGC = Cast<UBlueprintGeneratedClass>(Outer))
		{
			BP = Cast<UBlueprint>(BPGC->ClassGeneratedBy);
			break;
		}
	}

	USCS_Node* MyNode = nullptr;
	if (BP != nullptr && BP->SimpleConstructionScript != nullptr)
	{
		for (USCS_Node* Node : BP->SimpleConstructionScript->GetAllNodes())
		{
			if (Node->ComponentTemplate == this)
			{
				MyNode = Node;
				break;
			}
		}
	}

	if (SMC == nullptr && MyNode != nullptr)
	{
		for (USCS_Node* ChildNode : MyNode->ChildNodes)
		{
			if (UStaticMeshComponent* Found = Cast<UStaticMeshComponent>(ChildNode->ComponentTemplate))
			{
				SMC = Found;
				break;
			}
		}
	}

	if (SMC == nullptr || SMC->GetStaticMesh() == nullptr)
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjGeom] DecomposeMesh: '%s' has no StaticMesh child."), *GetName());
		return;
	}

	UStaticMesh* LocalMesh = SMC->GetStaticMesh();
	UBodySetup* BodySetup = LocalMesh->GetBodySetup();
	if (BodySetup == nullptr || BodySetup->TriMeshGeometries.Num() == 0)
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjGeom] DecomposeMesh: '%s' has no collision geometry."), *GetName());
		return;
	}

	RemoveDecomposition();

	FScopedSlowTask SlowTask(2.f, NSLOCTEXT("URLab", "DecomposingMesh", "Running CoACD mesh decomposition..."));
	SlowTask.MakeDialog(/*bShowCancelButton=*/false);
	SlowTask.EnterProgressFrame(1.f, NSLOCTEXT("URLab", "DecompStep1", "Decomposing mesh with CoACD..."));

	const FString AssetName = LocalMesh->GetName();
	const FString OwnerDir = GetOwner() != nullptr ? GetOwner()->GetClass()->GetName() : TEXT("Shared");
	const FString FullFilePath = FPaths::ConvertRelativePathToFull(
		FString::Printf(TEXT("%s/URLab/ConvertedMeshes/%s/Complex_%s.obj"),
			*FPaths::ProjectSavedDir(), *OwnerDir, *AssetName));

	auto& TriGeom = BodySetup->TriMeshGeometries[0];
	auto& Vertices = TriGeom.GetReference()->Particles().X();

	int32 MeshCount = 0;
	IO::DeleteMeshCache(FullFilePath, true);

	if (TriGeom.GetReference()->Elements().RequiresLargeIndices())
	{
		const auto& Indices = TriGeom.GetReference()->Elements().GetLargeIndexBuffer();
		MeshCount = MeshUtils::SaveMesh(FullFilePath, Vertices, Indices, true, CoACDThreshold);
		IO::SaveMeshHash(FullFilePath,
			IO::ComputeMeshHash(Vertices, Indices) + FString::Printf(TEXT("_complex_%.4f"), CoACDThreshold));
	}
	else
	{
		const auto& Indices = TriGeom.GetReference()->Elements().GetSmallIndexBuffer();
		MeshCount = MeshUtils::SaveMesh(FullFilePath, Vertices, Indices, true, CoACDThreshold);
		IO::SaveMeshHash(FullFilePath,
			IO::ComputeMeshHash(Vertices, Indices) + FString::Printf(TEXT("_complex_%.4f"), CoACDThreshold));
	}

	if (MeshCount == 0)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjGeom] DecomposeMesh: CoACD produced 0 hulls for '%s'."), *GetName());
		return;
	}

	SlowTask.EnterProgressFrame(1.f,
		FText::Format(NSLOCTEXT("URLab", "DecompStep2", "Creating {0} hull components..."), FText::AsNumber(MeshCount)));

	// Hulls are siblings of this geom, so in a Blueprint they go under the same SCS
	// node this one hangs from, and on a placed actor under the same component.
	USCS_Node* ParentNode = nullptr;
	if (MyNode != nullptr && BP != nullptr)
	{
		for (USCS_Node* Node : BP->SimpleConstructionScript->GetAllNodes())
		{
			if (Node->ChildNodes.Contains(MyNode))
			{
				ParentNode = Node;
				break;
			}
		}
	}

	USceneComponent* ParentComp = GetAttachParent();
	const bool bIsScsContext = MyNode != nullptr && ParentNode != nullptr;
	if (!bIsScsContext && ParentComp == nullptr)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjGeom] DecomposeMesh: '%s' has no parent to attach hulls to."), *GetName());
		return;
	}

	if (BP != nullptr)
	{
		BP->Modify();
	}

	const FString SourceName = ElementName(*this);

	for (int32 i = 0; i < MeshCount; ++i)
	{
		const FString HullNameStr = FString::Printf(TEXT("%s_hull_%d"), *GetName(), i);

		UMjGeom* Hull = nullptr;
		if (bIsScsContext)
		{
			USCS_Node* HullNode = BP->SimpleConstructionScript->CreateNode(UMjGeom::StaticClass(), *HullNameStr);
			Hull = Cast<UMjGeom>(HullNode->ComponentTemplate);
			ParentNode->AddChildNode(HullNode);
		}
		else
		{
			Hull = NewObject<UMjGeom>(GetOwner(), UMjGeom::StaticClass(),
				MakeUniqueObjectName(GetOwner(), UMjGeom::StaticClass(), *HullNameStr));
			Hull->CreationMethod = EComponentCreationMethod::Instance;
			GetOwner()->AddInstanceComponent(Hull);
			Hull->RegisterComponent();
			Hull->AttachToComponent(ParentComp, FAttachmentTransformRules::KeepRelativeTransform);
		}

		if (Hull == nullptr)
		{
			continue;
		}

		Hull->EnsureSerial();
		Hull->bIsDecomposedHull = true;
		Hull->SetType(EMjGeomType::mesh);
		Hull->MeshName = FString::Printf(TEXT("%s_%d"), *AssetName, i);
		Hull->MjName = FString::Printf(TEXT("%s_hull_%d"), *SourceName, i);

		// Contact behaviour comes across attribute by attribute, presence included:
		// a hull that inherits friction from a default class must keep inheriting it.
		Hull->Dclass = Dclass;
		Hull->Contype = Contype;
		Hull->Conaffinity = Conaffinity;
		Hull->Condim = Condim;
		Hull->Priority = Priority;
		Hull->Friction = Friction;
		Hull->Mass = Mass;
		Hull->Density = Density;
		Hull->Solref = Solref;
		Hull->Solimp = Solimp;
		Hull->Margin = Margin;
		Hull->Gap = Gap;
		Hull->Rgba = Rgba;

		// Group 3 is the collision group, which is what a hull is.
		Hull->SetGroup(3);

		const FString SubObjFullPath = FPaths::ConvertRelativePathToFull(
			FString::Printf(TEXT("%s/URLab/ConvertedMeshes/%s/Complex_%s_sub_%d.obj"),
				*FPaths::ProjectSavedDir(), *OwnerDir, *AssetName, i));
		if (!FPaths::FileExists(SubObjFullPath))
		{
			continue;
		}

		const FString ArticName = GetOwner() != nullptr ? GetOwner()->GetName() : TEXT("Unknown");
		UAutomatedAssetImportData* ImportData = NewObject<UAutomatedAssetImportData>();
		ImportData->Filenames.Add(SubObjFullPath);
		ImportData->DestinationPath = FString::Printf(TEXT("/Game/URLab/DecomposedMeshes/%s"), *ArticName);
		ImportData->bReplaceExisting = true;
		ImportData->bSkipReadOnly = true;

		IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools").Get();
		UStaticMesh* ImportedMesh = nullptr;
		for (UObject* Asset : AssetTools.ImportAssetsAutomated(ImportData))
		{
			if (UStaticMesh* SM = Cast<UStaticMesh>(Asset))
			{
				ImportedMesh = SM;
				break;
			}
		}
		if (ImportedMesh == nullptr)
		{
			continue;
		}

		const FString SMCName = FString::Printf(TEXT("%s_vis"), *HullNameStr);
		USCS_Node* HullNode = nullptr;
		if (bIsScsContext)
		{
			for (USCS_Node* Node : BP->SimpleConstructionScript->GetAllNodes())
			{
				if (Node->ComponentTemplate == Hull)
				{
					HullNode = Node;
					break;
				}
			}
		}

		if (HullNode != nullptr)
		{
			USCS_Node* SMCNode =
				BP->SimpleConstructionScript->CreateNode(UStaticMeshComponent::StaticClass(), *SMCName);
			if (UStaticMeshComponent* VisMesh = Cast<UStaticMeshComponent>(SMCNode->ComponentTemplate))
			{
				VisMesh->SetStaticMesh(ImportedMesh);
				// The OBJ is in metres and the level is in centimetres.
				VisMesh->SetRelativeScale3D(FVector(kCmPerM));
			}
			HullNode->AddChildNode(SMCNode);
		}
		else
		{
			UStaticMeshComponent* VisMesh = NewObject<UStaticMeshComponent>(GetOwner(), *SMCName);
			VisMesh->SetStaticMesh(ImportedMesh);
			VisMesh->SetRelativeScale3D(FVector(kCmPerM));
			VisMesh->CreationMethod = EComponentCreationMethod::Instance;
			GetOwner()->AddInstanceComponent(VisMesh);
			VisMesh->RegisterComponent();
			VisMesh->AttachToComponent(Hull, FAttachmentTransformRules::KeepRelativeTransform);
		}
	}

	if (BP != nullptr)
	{
		FBlueprintEditorUtils::MarkBlueprintAsStructurallyModified(BP);
	}

	bDisabledByDecomposition = true;
	UE_LOG(LogURLab, Log, TEXT("[MjGeom] Decomposed '%s' into %d hull sub-geoms."), *GetName(), MeshCount);
}

void UMjGeom::RemoveDecomposition()
{
	const FString HullPrefix = ElementName(*this) + TEXT("_hull_");
	int32 Removed = 0;

	UBlueprint* BP = nullptr;
	for (UObject* Outer = GetOuter(); Outer != nullptr; Outer = Outer->GetOuter())
	{
		if (UBlueprint* Found = Cast<UBlueprint>(Outer))
		{
			BP = Found;
			break;
		}
		if (UBlueprintGeneratedClass* BPGC = Cast<UBlueprintGeneratedClass>(Outer))
		{
			BP = Cast<UBlueprint>(BPGC->ClassGeneratedBy);
			break;
		}
	}

	auto IsMyHull = [this, &HullPrefix](const UMjGeom* Geom) {
		return Geom != nullptr && Geom != this && Geom->bIsDecomposedHull && Geom->MjName.IsSet()
			&& Geom->MjName.GetValue().StartsWith(HullPrefix);
	};

	if (BP != nullptr && BP->SimpleConstructionScript != nullptr)
	{
		BP->Modify();

		TArray<USCS_Node*> NodesToRemove;
		for (USCS_Node* Node : BP->SimpleConstructionScript->GetAllNodes())
		{
			if (IsMyHull(Cast<UMjGeom>(Node->ComponentTemplate)))
			{
				NodesToRemove.Add(Node);
			}
		}

		for (USCS_Node* Node : NodesToRemove)
		{
			BP->SimpleConstructionScript->RemoveNode(Node);
			++Removed;
		}

		if (Removed > 0)
		{
			FBlueprintEditorUtils::MarkBlueprintAsStructurallyModified(BP);
		}
	}
	else if (GetOwner() != nullptr)
	{
		TArray<UMjGeom*> AllGeoms;
		GetOwner()->GetComponents<UMjGeom>(AllGeoms);
		for (UMjGeom* Geom : AllGeoms)
		{
			if (IsMyHull(Geom))
			{
				Geom->DestroyComponent();
				++Removed;
			}
		}
	}

	bDisabledByDecomposition = false;
	UE_LOG(LogURLab, Log, TEXT("[MjGeom] Removed %d hull sub-geoms for '%s'."), Removed, *GetName());
}

#else // WITH_EDITOR

void UMjGeom::DecomposeMesh()
{
}

void UMjGeom::RemoveDecomposition()
{
}

#endif // WITH_EDITOR
