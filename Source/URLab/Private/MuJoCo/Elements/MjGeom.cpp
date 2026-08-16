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
#include "MuJoCo/Entity/MjAppearance.h"
#include "MuJoCo/Entity/MjGeomAppearance.h"
#include "MuJoCo/Spec/MjEffective.h"
#include "MuJoCo/Spec/MjAssetResolve.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjScalePolicy.h"
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

/**
 * The engine primitive a shape previews as, keyed by the `type` attribute.
 *
 * Six of the nine shapes have a preview and the other three are here to say
 * they have none, which is the honest answer for a height field, a mesh and an
 * SDF: each carries its picture as a child component rather than as a scaled
 * engine primitive.
 *
 * The other half of what a shape is -- how its `size` and a component's scale
 * correspond, and which axes a drag may move independently -- is not a geom
 * question, because `<site>` asks it too and answers it the same way. That half
 * lives in `MjScalePolicy.h` and is reached here through `MjSizeShapeFor`.
 */
struct FGeomShape
{
	/** Engine primitive the preview is built from; null when there is no preview. */
	const TCHAR* MeshPath = nullptr;

	/** Sphere for the capsule's two end caps; null for every other shape. */
	const TCHAR* CapMeshPath = nullptr;
};

// constexpr (not just const): these initialize the constexpr GGeomShapes[]
// below, and Clang (unlike MSVC) rejects a non-constexpr pointer in a constant
// expression. See "Linux portability" notes.
constexpr const TCHAR* kCubeMesh = TEXT("/Engine/BasicShapes/Cube.Cube");
constexpr const TCHAR* kSphereMesh = TEXT("/Engine/BasicShapes/Sphere.Sphere");
constexpr const TCHAR* kCylinderMesh = TEXT("/Engine/BasicShapes/Cylinder.Cylinder");
constexpr const TCHAR* kPlaneMesh = TEXT("/Engine/BasicShapes/Plane.Plane");

constexpr FGeomShape GGeomShapes[] = {
	/* plane     */ {kPlaneMesh, nullptr},
	/* hfield    */
	{},
	/* sphere    */
	{kSphereMesh, nullptr},
	/* capsule   */
	{kCylinderMesh, kSphereMesh},
	/* ellipsoid */
	{kSphereMesh, nullptr},
	/* cylinder  */
	{kCylinderMesh, nullptr},
	/* box       */
	{kCubeMesh, nullptr},
	/* mesh      */
	{},
	/* sdf       */
	{},
};

static_assert(static_cast<int32>(EMjGeomType::sdf) + 1 == static_cast<int32>(UE_ARRAY_COUNT(GGeomShapes)),
	"GGeomShapes is indexed by EMjGeomType and must have a row for every value, in declaration order");

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

/** The `<hfield>` this geom names, resolved the way `EffectiveMeshName` resolves `mesh`. */
FString EffectiveHfieldName(const UMjGeom& Geom)
{
	FString Name = Geom.Hfield.Get(FString());
#if URLAB_MJ_GEN
	if (Geom.Hfield.IsSet())
	{
		return Name;
	}
	urlab::spec::WithEffectiveDoc(Geom, [&](auto& Effective) {
		Effective.ForEachLayer(static_cast<const UMjGeomBase&>(Geom), [&](const auto& Layer) {
			if (!Layer.Hfield.IsSet())
			{
				return false;
			}
			Name = Layer.Hfield.GetValue();
			return true;
		});
	});
#endif
	return Name;
}

/**
 * The `<default>` class this geom's effective `type` came from.
 *
 * Empty when the geom authored the type itself, which is also what the schema
 * layer yields -- and the schema's own `type` is `sphere`, which always draws,
 * so it never reaches the caller below.
 */
FString ClassSupplyingType(const UMjGeom& Geom)
{
#if URLAB_MJ_GEN
	TArray<urlab::spec::FMjEffectiveLayer> Layers;
	urlab::spec::MjEffectiveLayersOf(Geom, Layers);
	for (const urlab::spec::FMjEffectiveLayer& Layer : Layers)
	{
		const UMjGeomBase* const Partial = Cast<UMjGeomBase>(Layer.Node);
		if (Partial != nullptr && Partial->Type.IsSet())
		{
			return Layer.ClassName;
		}
	}
#endif
	return FString();
}

/**
 * Say so when a geom's effective shape leaves it with no picture and no geom in
 * the compiled model, and withdraw it when that stops being true.
 *
 * A menagerie model puts `type="mesh"` on a `<default>` class and the mesh name
 * on each geom, so a geom added by hand under a body carrying that class inherits
 * the type, names no mesh, and is silent twice over: it draws nothing, and the
 * next compile fails somewhere with no element to blame. The user did nothing
 * but add a geom.
 *
 * Keyed on the NAME being absent, never on the asset failing to load. A named
 * mesh whose asset did not resolve is the import pass's report to make -- it
 * knows which file was missing -- and repeating it here would accuse every
 * import whose meshes are prepared in a later step.
 */
void NoteUndrawableShape(UMjGeom& Geom, EMjGeomType EffectiveType)
{
	const TCHAR* Attribute = nullptr;
	FString Named;
	if (EffectiveType == EMjGeomType::mesh)
	{
		Attribute = TEXT("mesh");
		Named = Geom.EffectiveMeshName();
	}
	else if (EffectiveType == EMjGeomType::hfield)
	{
		Attribute = TEXT("hfield");
		Named = EffectiveHfieldName(Geom);
	}
	else
	{
		// An `sdf` geom takes its shape from a `<plugin>` child rather than from a
		// named asset, so there is no empty name to key on and one that compiles
		// perfectly well would be accused of the opposite.
		Geom.ClearPreviewProblem(EMjPreviewProblem::UndrawableShape);
		return;
	}

	if (!Named.IsEmpty())
	{
		Geom.ClearPreviewProblem(EMjPreviewProblem::UndrawableShape);
		return;
	}

	const FString Source = ClassSupplyingType(Geom);
	const FString Where = Source.IsEmpty()
							? FString::Printf(TEXT("type=\"%s\" is authored here"), Attribute)
							: FString::Printf(TEXT("type=\"%s\" comes from default class '%s'"), Attribute, *Source);
	Geom.NotePreviewProblem(EMjPreviewProblem::UndrawableShape,
		FString::Printf(TEXT("%s and no <%s> is named: this geom will not draw, and the model will not compile"),
			*Where, Attribute));
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
	return Color.R == MjMagicDefaultRgba.R && Color.G == MjMagicDefaultRgba.G && Color.B == MjMagicDefaultRgba.B && Color.A == MjMagicDefaultRgba.A;
}

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

void UMjGeom::ConstrainPreviewScale()
{
	// The lock itself is the base's, off the shared shape table, because a
	// site's sphere locks exactly as a geom's does. What is a geom's own is the
	// capsule: its caps are separate components carrying a counter-scale, and
	// they have to follow the scale the lock just settled on.
	Super::ConstrainPreviewScale();
	UpdateCapTransforms();
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

EMjGeomType UMjGeom::EffectiveType() const
{
	return EffectiveShapeOf(*this).Type;
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
		// No engine primitive and no asset: nothing is drawn, and the user is told
		// rather than left with a geom that is simply not there. Deliberately not
		// a stand-in sphere -- a sphere the model does not contain is a worse lie
		// than an absence, because it looks like it will be simulated.
		NoteUndrawableShape(*this, EffectiveType);
		return;
	}
	ClearPreviewProblem(EMjPreviewProblem::UndrawableShape);

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
	const double Infinite = urlab::spec::MjSizeShapeFor(Shape.Type).InfiniteExtent;
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

	// An imported asset arrives with materials of its own, and replacing those
	// with a colour the spec never asked for would be a downgrade. So an
	// asset-backed preview is dressed only where the spec does ask: a named
	// material, or an rgba authored away from the magic default. An engine
	// primitive has nothing to lose and is dressed either way.
	const bool bAssetBacked = Shape.Type == EMjGeomType::mesh;
	const bool bSpecAsksForColour = !MaterialName.IsEmpty() || !IsMagicDefaultRgba(Color);
	if (!bAssetBacked || bSpecAsksForColour)
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
		if (ChildMesh != nullptr && ChildMesh != VisualizerMesh && ChildMesh != VisualizerCapTop && ChildMesh != VisualizerCapBottom)
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

namespace
{

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
	if (MjResolveMaterial(FSpecRef::OverOwner(this), EffectiveMaterialName(), Values) && !IsMagicDefaultRgba(Values.Rgba))
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

int32 UMjGeom::ApplyAppearanceOverride(const FMjGeomAppearance* Override,
	TFunctionRef<UTexture*(FName)> ResolveTexture)
{
	// No override re-runs the spec pass, which rebuilds each part's dynamic
	// instance at the authored appearance -- the clean way to undo a variant.
	if (Override == nullptr)
	{
		ApplySpecMaterial();
	}

	int32 Applied = 0;
	auto Drive = [&](UStaticMeshComponent* Part) {
		if (!IsValid(Part))
		{
			return;
		}
		if (UMaterialInstanceDynamic* Mid = Cast<UMaterialInstanceDynamic>(Part->GetMaterial(0)))
		{
			if (Override != nullptr)
			{
				MjAppearance::Apply(Mid, *Override, ResolveTexture);
			}
			++Applied;
		}
	};

	Drive(VisualizerMesh);
	Drive(VisualizerCapTop);
	Drive(VisualizerCapBottom);

	// A mesh geom's picture is a static mesh hung under it rather than one of the
	// preview parts, so its instance is reached through the child walk.
	TArray<USceneComponent*> Children;
	GetChildrenComponents(/*bIncludeAllDescendants=*/true, Children);
	for (USceneComponent* Child : Children)
	{
		UStaticMeshComponent* ChildMesh = Cast<UStaticMeshComponent>(Child);
		if (ChildMesh != nullptr && ChildMesh != VisualizerMesh && ChildMesh != VisualizerCapTop
			&& ChildMesh != VisualizerCapBottom)
		{
			Drive(ChildMesh);
		}
	}
	return Applied;
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

namespace
{
/**
 * The geom attributes the picture is derived from.
 *
 * Rebuilding a visualiser resolves the shape, the mesh asset, the material and
 * the colour through the default-class chain, which is several whole-spec reads;
 * a geom carries thirty-odd attributes and most of them are physics. Editing
 * `contype` or `group` used to cost what editing `size` costs, which is what
 * made a geom's details panel the slowest one in the editor.
 *
 * `class` is in the list because it is the chain itself: changing it changes
 * every inherited value at once. Nothing here is a guess about the schema -- the
 * names are this element's own members, and an attribute that starts feeding the
 * picture has to be added here in the same change that makes it do so.
 */
bool AffectsGeomPicture(FName PropertyName)
{
	static const TSet<FName> Visual = {
		FName(TEXT("Dclass")),
		FName(TEXT("Type")),
		FName(TEXT("Size")),
		FName(TEXT("Fromto")),
		FName(TEXT("Pos")),
		FName(TEXT("Quat")),
		FName(TEXT("Mesh")),
		FName(TEXT("Hfield")),
		FName(TEXT("Fitscale")),
		FName(TEXT("Material")),
		FName(TEXT("Rgba")),
		FName(TEXT("OverrideMaterial")),
	};
	return Visual.Contains(PropertyName);
}
} // namespace

void UMjGeom::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	// One index for the whole edit, the base's half included. The base resolves
	// the pose and the scale through the class chain and the rebuild below
	// resolves the shape, the mesh, the material and the colour through the same
	// chain; each of those indexed the spec on its own, so editing one geom
	// attribute cost the model several times over.
#if URLAB_MJ_GEN
	urlab::spec::FMjEffectiveScope Edit(*this);
#endif

	// The base drives the preview transform and the write-back from the schema's
	// own field ids, `type` and `size` included. What is left here is the
	// picture: a different `type` is a different engine primitive.
	Super::PostEditChangeProperty(PropertyChangedEvent);

	// A property this hook cannot name is rebuilt for: an undo, a paste, or a
	// bulk edit arrives with no property at all, and the picture may be anything.
	const FName Changed = PropertyChangedEvent.GetPropertyName();
	const FName Member =
		PropertyChangedEvent.MemberProperty != nullptr ? PropertyChangedEvent.MemberProperty->GetFName() : NAME_None;
	if (!Changed.IsNone() && !AffectsGeomPicture(Changed) && !AffectsGeomPicture(Member))
	{
		return;
	}

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
	TArray<FDecomposedHull> Ignored;
	DecomposeMeshInto(Ignored);
}

bool UMjGeom::DecomposeMeshInto(TArray<FDecomposedHull>& OutHulls)
{
	OutHulls.Reset();

	if (EffectiveShapeOf(*this).Type != EMjGeomType::mesh)
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjGeom] DecomposeMesh: '%s' is not a mesh geom."), *GetName());
		return false;
	}

	// The geom's own `<mesh>` element, the same source its picture is drawn from.
	// Not a child UStaticMeshComponent: the visualiser's parts are output, and a
	// Blueprint template has no attached children to find one under anyway.
	UStaticMesh* LocalMesh = MjResolveMesh(FSpecRef::OverOwner(this), EffectiveMeshName()).Asset;
	if (LocalMesh == nullptr)
	{
		UE_LOG(LogURLab, Warning,
			TEXT("[MjGeom] DecomposeMesh: '%s' names mesh '%s', which resolves to no asset."),
			*GetName(), *EffectiveMeshName());
		return false;
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

	UBodySetup* BodySetup = LocalMesh->GetBodySetup();
	if (BodySetup == nullptr || BodySetup->TriMeshGeometries.Num() == 0)
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjGeom] DecomposeMesh: '%s' has no collision geometry."), *GetName());
		return false;
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
		MeshCount = MeshUtils::SaveMesh(FullFilePath, Vertices, Indices, true, CoACDThreshold, bCoACDExtrude, CoACDExtrudeMargin);
		IO::SaveMeshHash(FullFilePath,
			IO::ComputeMeshHash(Vertices, Indices)
				+ FString::Printf(TEXT("_complex_%.4f_%d_%.4f"), CoACDThreshold, bCoACDExtrude ? 1 : 0,
					CoACDExtrudeMargin));
	}
	else
	{
		const auto& Indices = TriGeom.GetReference()->Elements().GetSmallIndexBuffer();
		MeshCount = MeshUtils::SaveMesh(FullFilePath, Vertices, Indices, true, CoACDThreshold, bCoACDExtrude, CoACDExtrudeMargin);
		IO::SaveMeshHash(FullFilePath,
			IO::ComputeMeshHash(Vertices, Indices)
				+ FString::Printf(TEXT("_complex_%.4f_%d_%.4f"), CoACDThreshold, bCoACDExtrude ? 1 : 0,
					CoACDExtrudeMargin));
	}

	if (MeshCount == 0)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjGeom] DecomposeMesh: CoACD produced 0 hulls for '%s'."), *GetName());
		return false;
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
		return false;
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

		// Reported before the visualiser is built: what the caller needs is the
		// geom and the file, and neither depends on the preview succeeding.
		FDecomposedHull& Reported = OutHulls.AddDefaulted_GetRef();
		Reported.MeshName = Hull->MeshName;
		Reported.ObjPath = SubObjFullPath;
		Reported.Geom = Hull;

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

	return !OutHulls.IsEmpty();
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

bool UMjGeom::DecomposeMeshInto(TArray<FDecomposedHull>& OutHulls)
{
	OutHulls.Reset();
	return false;
}

void UMjGeom::RemoveDecomposition()
{
}

#endif // WITH_EDITOR
