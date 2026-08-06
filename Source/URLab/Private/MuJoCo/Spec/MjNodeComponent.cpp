// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjNodeComponent.h"

#include "Engine/Blueprint.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformAtomics.h"
#include "MuJoCo/Spec/MjSpecProfile.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjEffective.h"
#include "MuJoCo/Spec/MjElementIdentity.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"
#include "MuJoCo/Utils/URLabAxisConv.h"
#include "UObject/UObjectHash.h"
#include "Utils/URLabLogging.h"

#if URLAB_PROTOSPEC
THIRD_PARTY_INCLUDES_START
#include "protospec/model_core.h"
THIRD_PARTY_INCLUDES_END
#endif

namespace
{
/**
 * Identity counter. Serials only have to be unique and monotonic within the
 * process: they key the compile-time auto-namer and the adapters' spec-order
 * tie-break, both of which live entirely inside one session's compile.
 */
std::atomic<uint64> GMjSerialCounter{0};

/** How close two transforms must be before a gizmo drag counts as a no-op. */
constexpr double MjPreviewEpsilon = UE_KINDA_SMALL_NUMBER;
}  // namespace

UMjNodeComponent::UMjNodeComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
	bWantsOnUpdateTransform = false;
}

void UMjNodeComponent::MintSerial()
{
	Serial = ++GMjSerialCounter;
}

void UMjNodeComponent::EnsureSerial()
{
	if (Serial == 0)
	{
		MintSerial();
	}
	else
	{
		// Keep the counter ahead of every serial the process has seen, so a tree
		// loaded from disk cannot collide with one minted afterwards.
		uint64 Seen = GMjSerialCounter.load(std::memory_order_relaxed);
		while (Seen < Serial && !GMjSerialCounter.compare_exchange_weak(Seen, Serial))
		{
		}
	}
}

void UMjNodeComponent::PostInitProperties()
{
	Super::PostInitProperties();
	// Class default objects and archetypes are not elements; only real instances
	// and templates carry identity.
	if (!HasAnyFlags(RF_ClassDefaultObject))
	{
		EnsureSerial();
	}
}

void UMjNodeComponent::PostLoad()
{
	Super::PostLoad();
	EnsureSerial();
	// Saved levels and SCS templates both arrive here. Templates never register,
	// so this is the only moment a template can acquire the write-back baseline,
	// and a template does receive gizmo moves.
	SyncPreviewFromSpec();
}

void UMjNodeComponent::OnRegister()
{
	Super::OnRegister();
	SyncPreviewFromSpec();
}

void UMjNodeComponent::OnComponentCreated()
{
	Super::OnComponentCreated();
	EnsureSerial();
}

void UMjNodeComponent::PostDuplicate(bool bDuplicateForPIE)
{
	Super::PostDuplicate(bDuplicateForPIE);
	// A copy is a new element, so it gets a new identity -- except when the copy
	// IS the original as far as the user is concerned. PIE duplicates the editor
	// world wholesale; re-minting there would sever the correspondence between an
	// editor element and its PIE instance that state migration and the golden
	// serial maps rely on.
	if (!bDuplicateForPIE)
	{
		MintSerial();
	}
	else
	{
		EnsureSerial();
	}
}

// --- Runtime binding ------------------------------------------------------- //

void UMjNodeComponent::BindTo(int32 Id)
{
	if (Id >= 0)
	{
		BoundId = Id;
	}
	else
	{
		BoundId.Reset();
	}
}

void UMjNodeComponent::Unbind()
{
	BoundId.Reset();
}

// --- Reference dropdowns --------------------------------------------------- //

TArray<FString> UMjNodeComponent::RefNameOptions(int32 FieldId) const
{
#if URLAB_MJ_GEN
	return urlab::spec::gen::RefNameOptions(this, FieldId);
#else
	(void)FieldId;
	return TArray<FString>();
#endif
}

// --- Pose preview and write-back ------------------------------------------- //
//
// The element's authored `pos` and `quat` are reached through the schema's own
// reflection tables (field id by MJCF attribute name) and the profile's field
// accessors, so no per-element code exists here and none is generated for it.
// What varies is the storage, and that is the Shape policy's business.

#if URLAB_MJ_GEN

namespace
{
namespace psm = ps::mjcf;
namespace pssdk = ps::sdk;

struct FMjPoseFieldIds
{
	int Pos = -1;
	int Quat = -1;

	bool IsValid() const { return Pos >= 0 || Quat >= 0; }
};

FMjPoseFieldIds PoseFieldIdsOf(psm::ElementType Type)
{
	FMjPoseFieldIds Out;
	Out.Pos = ps::sdk::internal::FieldIdByName(Type, "pos");
	Out.Quat = ps::sdk::internal::FieldIdByName(Type, "quat");
	return Out;
}

/** The element's authored MuJoCo-space pose, in MJCF component order. */
bool ReadMjPose(UMjNodeComponent& Node, double OutPos[3], double OutQuat[4], bool& bPosSet, bool& bQuatSet)
{
	using P = urlab::spec::FMjInstanceProfile;
	bPosSet = false;
	bQuatSet = false;
	bool bAny = false;
	urlab::spec::gen::DispatchByType(Node, [&](auto& Element) {
		using E = std::decay_t<decltype(Element)>;
		const FMjPoseFieldIds Ids = PoseFieldIdsOf(urlab::spec::gen::TMjElementType<E>::Value);
		if (!Ids.IsValid())
		{
			return;
		}
		bAny = true;
		if (Ids.Pos >= 0)
		{
			bPosSet = urlab::spec::ReadFixed<P, 3>(Element, Ids.Pos, OutPos);
		}
		if (Ids.Quat >= 0)
		{
			bQuatSet = urlab::spec::ReadFixed<P, 4>(Element, Ids.Quat, OutQuat);
		}
	});
	return bAny;
}

/**
 * Fill whichever of `pos` and `quat` the element did not author from its
 * default-class chain.
 *
 * Reached only when the authored read left something unset, because resolving a
 * chain means indexing the whole spec and the answer is the same either way
 * when the element authored the value itself.
 */
void FillMjPoseFromClasses(UMjNodeComponent& Node, double OutPos[3], double OutQuat[4], bool& bPosSet, bool& bQuatSet)
{
	urlab::spec::WithEffectiveDoc(Node, [&](auto& Effective) {
		using P = typename std::decay_t<decltype(Effective)>::ProfileType;
		urlab::spec::gen::DispatchByType(Node, [&](auto& Element) {
			using E = std::decay_t<decltype(Element)>;
			const FMjPoseFieldIds Ids = PoseFieldIdsOf(urlab::spec::gen::TMjElementType<E>::Value);
			if (!bPosSet && Ids.Pos >= 0)
			{
				Effective.ForEachLayer(Element, [&](const auto& Layer) {
					return (bPosSet = urlab::spec::ReadFixed<P, 3>(Layer, Ids.Pos, OutPos));
				});
			}
			if (!bQuatSet && Ids.Quat >= 0)
			{
				Effective.ForEachLayer(Element, [&](const auto& Layer) {
					return (bQuatSet = urlab::spec::ReadFixed<P, 4>(Layer, Ids.Quat, OutQuat));
				});
			}
		});
	});
}

/** Author `pos`, `quat`, or both. An id below zero means the element has none. */
void WriteMjPose(UMjNodeComponent& Node, const double* InPos, const double* InQuat)
{
	using P = urlab::spec::FMjInstanceProfile;
	urlab::spec::gen::DispatchByType(Node, [&](auto& Element) {
		using E = std::decay_t<decltype(Element)>;
		const FMjPoseFieldIds Ids = PoseFieldIdsOf(urlab::spec::gen::TMjElementType<E>::Value);
		if (InPos != nullptr && Ids.Pos >= 0)
		{
			const double Values[3] = {InPos[0], InPos[1], InPos[2]};
			pssdk::internal::SetFixedField<P, 3>(Element, Ids.Pos, Values);
		}
		if (InQuat != nullptr && Ids.Quat >= 0)
		{
			const double Values[4] = {InQuat[0], InQuat[1], InQuat[2], InQuat[3]};
			pssdk::internal::SetFixedField<P, 4>(Element, Ids.Quat, Values);
		}
	});
}

/** True when this element's schema declares an orientation attribute. */
bool HasQuatAttribute(UMjNodeComponent& Node)
{
	bool bHas = false;
	urlab::spec::gen::DispatchByType(Node, [&](auto& Element) {
		using E = std::decay_t<decltype(Element)>;
		bHas = PoseFieldIdsOf(urlab::spec::gen::TMjElementType<E>::Value).Quat >= 0;
	});
	return bHas;
}

/**
 * Every component built from `Template`, without a global object scan.
 *
 * `UObject::GetArchetypeInstances` answers this by walking every object in the
 * process, which is affordable once and ruinous per gizmo delta. A construction
 * script's templates instantiate as components of actors of the Blueprint's
 * generated class, and the object hash answers that directly.
 *
 * Collected first and visited afterwards, never visited during. The traversal
 * runs inside the object hash's own iteration, and creating a UObject while that
 * is open is fatal, not slow -- "Trying to modify UObject map (FindOrAdd) that
 * is currently being iterated". What a visitor does here reaches
 * `RefreshPresentation`, and a geom's is in the business of creating objects:
 * preview mesh components and a dynamic material instance per part. So the walk
 * gathers and ends, and the work begins after it has closed.
 *
 * Weakly held across that gap. The two phases are no longer one atomic step, and
 * the work is free to destroy components -- rebuilding a visualiser destroys the
 * previous one -- so an entry may be gone by the time its turn comes.
 *
 * Nothing to do for a component that already belongs to an actor: it is an
 * instance, not a template, which is also what keeps the level-editor drag off
 * this path entirely.
 */
void ForEachInstanceOfTemplate(UMjNodeComponent& Template, TFunctionRef<void(UMjNodeComponent&)> Visit)
{
#if WITH_EDITOR
	if (Template.GetOwner() != nullptr)
	{
		return;
	}
	const FSpecRef Doc = FSpecRef::OverOwner(&Template);
	UBlueprint* Blueprint = Doc.GetGraph() == EMjSpecGraph::Scs ? Doc.GetBlueprint() : nullptr;
	UClass* Generated = Blueprint != nullptr ? Blueprint->GeneratedClass : nullptr;
	if (Generated == nullptr)
	{
		return;
	}

	TArray<TWeakObjectPtr<UMjNodeComponent>> Instances;
	ForEachObjectOfClass(Generated, [&Template, &Instances](UObject* Object) {
		AActor* Actor = Cast<AActor>(Object);
		if (Actor == nullptr)
		{
			return;
		}
		TArray<UMjNodeComponent*> Nodes;
		Actor->GetComponents(Nodes);
		for (UMjNodeComponent* Node : Nodes)
		{
			if (Node != nullptr && Node->GetArchetype() == &Template)
			{
				Instances.Add(Node);
			}
		}
	});

	for (const TWeakObjectPtr<UMjNodeComponent>& Instance : Instances)
	{
		if (UMjNodeComponent* Node = Instance.Get())
		{
			Visit(*Node);
		}
	}
#endif
}

/** True when `FieldId` on `Type` is one of the attributes the preview draws. */
bool IsSpatialField(psm::ElementType Type, FName PropertyName)
{
	static const FName PosName(TEXT("Pos"));
	static const FName QuatName(TEXT("Quat"));
	static const FName SizeName(TEXT("Size"));
	static const FName TypeName(TEXT("Type"));
	if (PropertyName != PosName && PropertyName != QuatName && PropertyName != SizeName && PropertyName != TypeName)
	{
		return false;
	}

	// The member spelling is the schema's, PascalCased, so the schema's own
	// field-id lookup is what decides whether this element actually has it --
	// no hand-kept list of which classes carry which attribute.
	const char* const Attribute = PropertyName == PosName	 ? "pos"
								  : PropertyName == QuatName ? "quat"
								  : PropertyName == SizeName ? "size"
															 : "type";
	return ps::sdk::internal::FieldIdByName(Type, Attribute) >= 0;
}
}  // namespace

bool UMjNodeComponent::HasPoseAttributes() const
{
	bool bHas = false;
	urlab::spec::gen::DispatchByType(const_cast<UMjNodeComponent&>(*this), [&](auto& Element) {
		using P = urlab::spec::FMjInstanceProfile;
		using E = std::decay_t<decltype(Element)>;
		bHas = PoseFieldIdsOf(urlab::spec::gen::TMjElementType<E>::Value).IsValid();
	});
	return bHas;
}

// --- What an element is in the spec ------------------------------------ //

namespace
{
/** True when `Node` is an element of exactly `Wanted`. */
bool IsElementOfType(const UMjNodeComponent& Node, psm::ElementType Wanted)
{
	psm::ElementType Type;
	return urlab::spec::MjElementTypeOfNode(Node, Type) && Type == Wanted;
}

/** Offer `Node` and each of its ancestors to `Predicate` until one accepts. */
template <class Adapter, class Pred>
bool AnyAncestorMatches(const UMjNodeComponent& Node, Pred&& Predicate)
{
	const UMjNodeComponent* Cursor = &Node;
	// Neither graph can represent a cycle, but a bound is what keeps a tree
	// corrupted by something else from hanging the editor here.
	for (int32 Depth = 0; Cursor != nullptr && Depth < 512; ++Depth)
	{
		if (Predicate(*Cursor))
		{
			return true;
		}
		Cursor = Adapter::ParentOf(*Cursor);
	}
	return false;
}

/**
 * The same walk, over whichever object graph holds `Node`'s spec.
 *
 * A component with an owner is in a spawned actor and its parent link is the
 * attachment; a Blueprint template has neither, and its tree is only reachable
 * through the construction script the ambient scope names.
 */
template <class Pred>
bool AnyAncestorInSpec(const UMjNodeComponent& Node, Pred&& Predicate)
{
	if (Node.GetOwner() != nullptr)
	{
		return AnyAncestorMatches<urlab::spec::FMjInstanceAdapter>(Node, Predicate);
	}
#if WITH_EDITOR
	const FSpecRef Doc = FSpecRef::OverOwner(&Node);
	if (Doc.GetGraph() == EMjSpecGraph::Scs)
	{
		if (UBlueprint* Blueprint = Doc.GetBlueprint())
		{
			urlab::spec::FMjScsScope Scope(*Blueprint);
			return AnyAncestorMatches<urlab::spec::FMjScsAdapter>(Node, Predicate);
		}
	}
#endif
	return false;
}
}  // namespace

bool UMjNodeComponent::IsClassPartial() const
{
	return AnyAncestorInSpec(
		*this, [](const UMjNodeComponent& Node) { return IsElementOfType(Node, psm::ElementType::Default); });
}

bool UMjNodeComponent::IsSharedPresentationInput() const
{
	return AnyAncestorInSpec(*this, [](const UMjNodeComponent& Node) {
		return IsElementOfType(Node, psm::ElementType::Default) || IsElementOfType(Node, psm::ElementType::Asset);
	});
}

// --- Presentation ----------------------------------------------------------- //

void UMjNodeComponent::RefreshPresentation()
{
	SyncPreviewFromSpec();
}

namespace
{
template <class Adapter>
void RefreshSubtree(UMjNodeComponent& Node)
{
	Node.RefreshPresentation();
	for (const urlab::spec::FMjOrderedChild& Child : Adapter::OrderedChildren(Node))
	{
		if (Child.Node != nullptr)
		{
			RefreshSubtree<Adapter>(*Child.Node);
		}
	}
}
}  // namespace

void UMjNodeComponent::RefreshSpecPresentation()
{
	const FSpecRef Doc = FSpecRef::OverOwner(this);
	UMjNodeComponent* Root = Doc.GetRoot();
	if (Root == nullptr)
	{
		return;
	}

	// One context for the whole walk. Each node asks the class chain several
	// questions and a geom asks more, and every one of those used to index the
	// spec from scratch -- so refreshing N elements cost N whole-spec walks per
	// question rather than one.
	urlab::spec::FMjEffectiveScope Effective(*Root);

#if WITH_EDITOR
	if (Doc.GetGraph() == EMjSpecGraph::Scs)
	{
		if (UBlueprint* Blueprint = Doc.GetBlueprint())
		{
			urlab::spec::FMjScsScope Scope(*Blueprint);
			RefreshSubtree<urlab::spec::FMjScsAdapter>(*Root);
		}
		return;
	}
#endif

	RefreshSubtree<urlab::spec::FMjInstanceAdapter>(*Root);
}

bool UMjNodeComponent::ComputePreviewTransform(FTransform& Out)
{
	// A class default object is not an element and has no spec to read.
	if (HasAnyFlags(RF_ClassDefaultObject))
	{
		return false;
	}

	double MjPos[3] = {0.0, 0.0, 0.0};
	double MjQuat[4] = {1.0, 0.0, 0.0, 0.0};
	bool bPosSet = false;
	bool bQuatSet = false;
	if (!ReadMjPose(*this, MjPos, MjQuat, bPosSet, bQuatSet))
	{
		return false;
	}
	if (!bPosSet || !bQuatSet)
	{
		FillMjPoseFromClasses(*this, MjPos, MjQuat, bPosSet, bQuatSet);
	}

	// A pose no layer authored previews at the schema default (origin, identity),
	// which is what MuJoCo compiles it to. Reading it back out of the preview is
	// what the write-back's change detector then suppresses.
	FVector Scale = GetRelativeScale3D();
	TryPreviewScaleFromSpec(Scale);
	Out = FTransform(URLabAxisConv::MjQuatToUe(MjQuat), URLabAxisConv::MjPositionToUe(MjPos), Scale);
	return true;
}

void UMjNodeComponent::SyncPreviewFromSpec()
{
	FTransform Preview;
	if (!ComputePreviewTransform(Preview))
	{
		return;
	}

	FVector Scale = GetRelativeScale3D();
	const bool bHasScale = TryPreviewScaleFromSpec(Scale);

	LastPreviewTransform = Preview;
	SetRelativeLocationAndRotation(Preview.GetLocation(), Preview.GetRotation());
	if (bHasScale)
	{
		SetRelativeScale3D(Preview.GetScale3D());
	}
}

void UMjNodeComponent::WriteBackTransformIfChanged()
{
	if (!HasPoseAttributes() || HasAnyFlags(RF_ClassDefaultObject))
	{
		return;
	}

	// No cached baseline is not the same as no baseline. The cache is a cache OF
	// the spec, so when it is cold the spec itself answers the question,
	// and the comparison below is the same comparison it always was.
	//
	// Syncing and returning here instead -- which is what this did -- discards
	// the move that just happened and puts the component back where the spec
	// says it is. That is the transform widget moving and the geom staying put,
	// and it is this guard failing in the direction that loses the user's input
	// rather than the direction that protects the spec. The protection is
	// unaffected: a descendant that received the hook without moving still
	// matches the spec and still authors nothing.
	if (!LastPreviewTransform.IsSet())
	{
		FTransform FromSpec;
		if (!ComputePreviewTransform(FromSpec))
		{
			return;
		}
		LastPreviewTransform = FromSpec;
	}

	// Moving the actor delivers this hook to every descendant, and not one of
	// their relative transforms changed. The baseline answers that on its own,
	// and it has to: both the scale snap below and the scale mapping resolve the
	// default-class chain, which indexes the entire spec once per element
	// that asks. Answering N of those per mouse-move is the whole cost.
	//
	// The snap is skipped along with the rest, and is a no-op in this case by
	// construction: the baseline's scale came from the spec through the
	// shape's own size mapping, so it is already a scale the shape can hold.
	if (LastPreviewTransform->Equals(
			FTransform(GetRelativeRotation().Quaternion(), GetRelativeLocation(), GetRelativeScale3D()),
			MjPreviewEpsilon))
	{
		return;
	}

	// Past the early-out, so the descendants of a moving actor never pay for it:
	// everything below resolves the class chain more than once, and each instance
	// below opens its own, an instance being a separate spec with its own root.
	urlab::spec::FMjEffectiveScope Effective(*this);

	// Snap first, so the comparison and the spec both see a scale the shape
	// can actually hold.
	ConstrainPreviewScale();

	// The pose this element was at before the drag. It is the baseline the change
	// detector uses below, and it is also how the propagation at the end tells an
	// instance that was following this element from one that had moved itself.
	const FTransform Baseline = LastPreviewTransform.GetValue();
	const FTransform Current(GetRelativeRotation().Quaternion(), GetRelativeLocation(), GetRelativeScale3D());
	const bool bPosMoved = !LastPreviewTransform->GetLocation().Equals(Current.GetLocation(), MjPreviewEpsilon);
	const bool bRotMoved = HasQuatAttribute(*this) &&
						   !LastPreviewTransform->GetRotation().Equals(Current.GetRotation(), MjPreviewEpsilon);
	const bool bScaleMoved =
		HasScaleMapping() && !LastPreviewTransform->GetScale3D().Equals(Current.GetScale3D(), MjPreviewEpsilon);

	if (!bPosMoved && !bRotMoved && !bScaleMoved)
	{
		return;
	}

	double MjPos[3] = {0.0, 0.0, 0.0};
	double MjQuat[4] = {1.0, 0.0, 0.0, 0.0};
	URLabAxisConv::UePositionToMj(Current.GetLocation(), MjPos);
	URLabAxisConv::UeQuatToMj(Current.GetRotation(), MjQuat);

	// Join the open undo transaction before authoring anything.
	//
	// The gizmo opens one and records the component's own transform, but the
	// MJCF attributes below are separate properties and are not in that record
	// unless the object asks to be. Without this an undo restores the transform,
	// leaves `pos` at its new value, and the next sync puts that value straight
	// back on the component -- so the undo appears to do nothing at all.
	//
	// A no-op outside a transaction, which is what the paths that author without
	// a user edit behind them want.
	Modify();

	// Each attribute is authored only where its own component moved. Writing the
	// unmoved half as well would author a value the element was inheriting.
	WriteMjPose(*this, bPosMoved ? MjPos : nullptr, bRotMoved ? MjQuat : nullptr);
	if (bScaleMoved)
	{
		WriteBackScale(Current.GetScale3D());
	}
	LastPreviewTransform = Current;

	// The Blueprint editor's viewport does not drag the component the user can
	// see. It applies the delta to the construction-script TEMPLATE and leaves
	// the preview actor's own components holding the pose they had, then carries
	// `RelativeLocation` across for the ones that had not overridden it.
	//
	// Nothing carries the authored attribute, and the gap does not stay
	// cosmetic. Re-running the construction scripts -- which that same drag does,
	// on every delta -- reads every attribute an instance holds differently from
	// its template as an instance OVERRIDE, caches it, and puts it back on the
	// rebuilt component. So the moment the template's `pos` moves, the instance's
	// unchanged `pos` becomes an override of it, and the preview is pinned to the
	// pre-drag pose for the rest of the session while the widget goes on moving.
	// That is the transform that cannot be edited.
	//
	// Attribute by attribute, and only onto an instance that still holds this
	// element's own pre-drag value: one that had been moved on its own really has
	// overridden the template and must keep what it authored.
	ForEachInstanceOfTemplate(*this, [&](UMjNodeComponent& Instance) {
		const bool bTakePos =
			bPosMoved && Instance.GetRelativeLocation().Equals(Baseline.GetLocation(), MjPreviewEpsilon);
		const bool bTakeRot = bRotMoved &&
			Instance.GetRelativeRotation().Quaternion().Equals(Baseline.GetRotation(), MjPreviewEpsilon);
		const bool bTakeScale =
			bScaleMoved && Instance.GetRelativeScale3D().Equals(Baseline.GetScale3D(), MjPreviewEpsilon);
		if (!bTakePos && !bTakeRot && !bTakeScale)
		{
			return;
		}
		// The instance is a different object from the template being dragged, so
		// it carries its own undo record or none.
		Instance.Modify();
		WriteMjPose(Instance, bTakePos ? MjPos : nullptr, bTakeRot ? MjQuat : nullptr);
		if (bTakeScale)
		{
			Instance.WriteBackScale(Current.GetScale3D());
		}
		Instance.SyncPreviewFromSpec();
	});
}

#else  // !URLAB_MJ_GEN

bool UMjNodeComponent::HasPoseAttributes() const
{
	return false;
}

void UMjNodeComponent::SyncPreviewFromSpec()
{
}

void UMjNodeComponent::WriteBackTransformIfChanged()
{
}

bool UMjNodeComponent::IsClassPartial() const
{
	return false;
}

bool UMjNodeComponent::IsSharedPresentationInput() const
{
	return false;
}

void UMjNodeComponent::RefreshPresentation()
{
}

void UMjNodeComponent::RefreshSpecPresentation()
{
}

#endif  // URLAB_MJ_GEN

#if WITH_EDITOR

void UMjNodeComponent::PostEditComponentMove(bool bFinished)
{
	Super::PostEditComponentMove(bFinished);
	// Deliberately not gated on bFinished: an SCS template only ever receives
	// false, and the change detector makes the per-delta invocations cheap.
	WriteBackTransformIfChanged();
}

void UMjNodeComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	const FName Changed = PropertyChangedEvent.GetPropertyName();
	const FName Member = PropertyChangedEvent.MemberProperty != nullptr
							 ? PropertyChangedEvent.MemberProperty->GetFName()
							 : NAME_None;

	if (Changed == USceneComponent::GetRelativeLocationPropertyName() ||
		Changed == USceneComponent::GetRelativeRotationPropertyName() ||
		Changed == USceneComponent::GetRelativeScale3DPropertyName())
	{
		WriteBackTransformIfChanged();
		return;
	}

#if URLAB_MJ_GEN
	// The other direction: typing into a spatial attribute has to move the
	// viewport. Which names those are comes from the schema's field-id lookup,
	// not from a list kept by hand here.
	psm::ElementType Type;
	if (urlab::spec::MjElementTypeOfNode(*this, Type) && (IsSpatialField(Type, Changed) || IsSpatialField(Type, Member)))
	{
		SyncPreviewFromSpec();
	}

	// Everything above acts on the element that was edited, which is the whole
	// answer for content. A shared node is an input to other elements' pictures,
	// and none of them re-reads on its own.
	if (IsSharedPresentationInput())
	{
		RefreshSpecPresentation();

		// A Blueprint template's own spec is the template graph, which draws
		// nothing. What the user is looking at is the preview actor -- and every
		// placed actor of that class -- so the instances built from this template
		// have to re-read their own specs too.
		// Each instance is a spec of its own, so `RefreshSpecPresentation` opens
		// its own context per instance rather than one covering the loop.
		ForEachInstanceOfTemplate(*this, [](UMjNodeComponent& Instance) {
			Instance.RefreshSpecPresentation();
		});
	}
#endif
}

void UMjNodeComponent::PostEditUndo()
{
	Super::PostEditUndo();
	// Undo restores properties without firing a change hook, so the baseline is
	// stale and the preview may disagree with the spec. Re-derive both.
	SyncPreviewFromSpec();
}

#endif  // WITH_EDITOR
