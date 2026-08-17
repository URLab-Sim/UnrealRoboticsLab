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
#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjScalePolicy.h"
#include "MuJoCo/Spec/MjSpecProfile.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjNodeRefOptions.h"
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

#if WITH_EDITOR
#include "Logging/MessageLog.h"
#endif

namespace
{
/**
 * Identity counter. Serials only have to be unique and monotonic within the
 * process: they are the adapters' spec-order tie-break for siblings whose
 * authored `SiblingIndex` collides, which lives entirely inside one session.
 *
 * Nothing that leaves the process is derived from one. The generated names an
 * unnamed element compiles under used to be, and are not: they count their
 * family in document order instead, so they repeat across runs.
 */
std::atomic<uint64> GMjSerialCounter{0};

/** How close two transforms must be before a gizmo drag counts as a no-op. */
constexpr double MjPreviewEpsilon = UE_KINDA_SMALL_NUMBER;
} // namespace

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
	SyncPreviewUnderOneScope();
}

void UMjNodeComponent::OnRegister()
{
	Super::OnRegister();
	SyncPreviewUnderOneScope();
#if WITH_EDITOR
	// Both ways of adding an element arrive here: the components panel of a
	// placed actor registers what it creates, and a Blueprint's construction
	// script registers the preview actor's copy of every template. The rule is
	// the schema's and there is one of it, so there is one place that applies it.
	CheckPlacementLegality();
#endif
}

void UMjNodeComponent::SyncPreviewUnderOneScope()
{
#if URLAB_MJ_GEN
	// One index for the whole sync. Syncing a single element resolves its pose,
	// its scale and -- for a geom -- its mesh and its material through the same
	// default-class chain, and each of those questions used to index the entire
	// spec on its own. Registering a model is N of those, which is the shape the
	// import path already avoids by holding one scope open across its walk.
	//
	// Only where the sync is the whole of the work: inside a pass that already
	// holds a scope open, this joins it and costs nothing.
	urlab::spec::FMjEffectiveScope Effective(*this);
#endif
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

// --- Preview diagnostics --------------------------------------------------- //

namespace
{
/**
 * Elements already told about, by identity and rule rather than by object.
 *
 * The same reasoning as `GReportedIllegalPlacement`: a component is not the
 * same object from one Blueprint reconstruct to the next, so a message keyed on
 * the object is said again on every recompile, while `Serial` survives a
 * reconstruct and a genuinely new element mints a fresh one. The rule is part of
 * the key because an element can break two of them, and clearing one must not
 * silence the other.
 */
TSet<TPair<uint64, uint8>> GReportedPreviewProblems;
} // namespace

void UMjNodeComponent::NotePreviewProblem(EMjPreviewProblem Problem, const FString& Message)
{
	// A class default object is not an element: it has no spec, and nothing the
	// user can see is derived from its picture.
	if (HasAnyFlags(RF_ClassDefaultObject))
	{
		return;
	}

	// The keys and the rows are one table in two arrays, and a mismatch between
	// them could only come from something writing the rows directly. Rebuilding
	// is the recovery, because the rows are re-derived anyway.
	if (PreviewProblemKeys.Num() != PreviewProblems.Num())
	{
		PreviewProblemKeys.Reset();
		PreviewProblems.Reset();
	}

	const int32 Existing = PreviewProblemKeys.Find(Problem);
	if (Existing != INDEX_NONE)
	{
		PreviewProblems[Existing] = Message;
	}
	else
	{
		PreviewProblemKeys.Add(Problem);
		PreviewProblems.Add(Message);
	}

	// The row above is the persistent surface and is rewritten every time. The
	// two logs are a transition: said when the element starts breaking the rule,
	// not once per registration for the rest of the session.
	const TPair<uint64, uint8> Key(Serial, static_cast<uint8>(Problem));
	if (GReportedPreviewProblems.Contains(Key))
	{
		return;
	}
	GReportedPreviewProblems.Add(Key);

	const FString Line = FString::Printf(TEXT("%s: %s"), *MjName.Get(GetName()), *Message);
	UE_LOG(LogURLab, Warning, TEXT("%s"), *Line);
#if WITH_EDITOR
	FMessageLog(TEXT("URLab")).Warning(FText::FromString(Line));
#endif
}

void UMjNodeComponent::ClearPreviewProblem(EMjPreviewProblem Problem)
{
	const int32 Existing = PreviewProblemKeys.Find(Problem);
	if (Existing != INDEX_NONE && PreviewProblems.IsValidIndex(Existing))
	{
		PreviewProblemKeys.RemoveAt(Existing);
		PreviewProblems.RemoveAt(Existing);
	}

	// Breaking the same rule a second time is a new mistake, and is said again.
	GReportedPreviewProblems.Remove(TPair<uint64, uint8>(Serial, static_cast<uint8>(Problem)));
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
	return urlab::spec::MjNodeRefNameOptions(*this, FieldId);
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
 *
 * `Doc` is the template's own spec, resolved by the caller: reaching a spec's
 * root is a walk of its own, and the callers here have already done it.
 */
void ForEachInstanceOfTemplate(
	const FSpecRef& Doc, UMjNodeComponent& Template, TFunctionRef<void(UMjNodeComponent&)> Visit)
{
#if WITH_EDITOR
	if (Template.GetOwner() != nullptr)
	{
		return;
	}
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

// --- References ------------------------------------------------------------ //
//
// Every cross-reference in a MuJoCo model is a NAME with a typed target, never a
// pointer: an actuator names the joint it drives, a geom names its material. The
// schema says which attributes those are and what each may point at, so
// everything below is reflection-driven and no element type is named by hand.

using FProfile = urlab::spec::FMjInstanceProfile;

/**
 * Invoke `On` for every authored reference of one element.
 *
 * Two kinds. The typed ones the profile's own scan hands out with their target
 * sets. The dynamic ones -- MuJoCo's frame sensors, whose `objname` means
 * whatever the sibling `objtype` says -- are plain strings the schema marks with
 * the sibling that types them, so the descriptor is what resolves them and they
 * are reached with no per-element code either.
 *
 * `On` is called as `On(FieldId, FieldName, Slot, Targets)`. The slot is a live,
 * move-only proxy over the stored name: writing to it edits the reference in
 * place, which is what makes the rename fixup below a rewrite rather than a
 * report.
 */
template <class E, class OnRef>
void ForEachReference(E& Element, OnRef&& On)
{
	FProfile::Ref::ScanTyped(Element, On);

	const psm::ElementType Type = urlab::spec::gen::TMjElementType<std::decay_t<E>>::Value;
	const psm::reflect::ElementDescriptor& Descriptor = psm::reflect::Describe(Type);
	for (int FieldId = 0; FieldId < static_cast<int>(Descriptor.field_count); ++FieldId)
	{
		const psm::reflect::FieldDescriptor& Field = Descriptor.fields[FieldId];
		if (Field.target_from.empty())
		{
			continue;
		}
		const int Sibling = ps::sdk::internal::FieldIdByName(Type, Field.target_from);
		if (Sibling < 0)
		{
			continue;
		}
		auto Keyword = FProfile::Ref::DynSlot(Element, Sibling);
		auto Name = FProfile::Ref::DynSlot(Element, FieldId);
		if (!Keyword.IsSet() || !Name.IsSet())
		{
			continue;
		}
		// An unset or unrecognised keyword types nothing, and a reference whose
		// target type is unknown is opaque rather than wrong: left alone.
		const std::vector<psm::ElementType> Targets =
			ps::sdk::detail::DynRefTargetTypes(FProfile::Str::ToUtf8(Keyword.Get()));
		if (Targets.empty())
		{
			continue;
		}
		On(FieldId, Field.name.data(), MoveTemp(Name), Targets);
	}
}

/** Every element of the tree rooted at `Node`, in spec order. */
template <class Adapter>
void WalkSpecTree(UMjNodeComponent& Node, TFunctionRef<void(UMjNodeComponent&)> Visit)
{
	Visit(Node);
	for (const urlab::spec::FMjOrderedChild& Child : Adapter::OrderedChildren(Node))
	{
		if (Child.Node != nullptr)
		{
			WalkSpecTree<Adapter>(*Child.Node, Visit);
		}
	}
}

#if WITH_EDITOR

/** The same walk, over whichever object graph holds `Spec`. */
void ForEachElementOfSpec(const FSpecRef& Spec, TFunctionRef<void(UMjNodeComponent&)> Visit)
{
	UMjNodeComponent* Root = Spec.GetRoot();
	if (Root == nullptr)
	{
		return;
	}
	if (Spec.GetGraph() == EMjSpecGraph::Scs)
	{
		if (UBlueprint* Blueprint = Spec.GetBlueprint())
		{
			urlab::spec::FMjScsScope Scope(*Blueprint);
			WalkSpecTree<urlab::spec::FMjScsAdapter>(*Root, Visit);
		}
		return;
	}
	WalkSpecTree<urlab::spec::FMjInstanceAdapter>(*Root, Visit);
}

/** True when `Type`'s attribute called `PropertyName` is a reference. */
bool IsReferenceField(psm::ElementType Type, FName PropertyName)
{
	if (PropertyName.IsNone())
	{
		return false;
	}
	// The member spelling is the schema's field name, PascalCased, and the
	// PascalCasing is the only difference -- so the schema's own descriptor
	// answers this rather than a list kept by hand here.
	const FString Wanted = PropertyName.ToString();
	const psm::reflect::ElementDescriptor& Descriptor = psm::reflect::Describe(Type);
	for (std::size_t FieldId = 0; FieldId < Descriptor.field_count; ++FieldId)
	{
		const psm::reflect::FieldDescriptor& Field = Descriptor.fields[FieldId];
		if (Field.kind != psm::reflect::FieldKind::Ref && Field.target_from.empty())
		{
			continue;
		}
		if (Wanted.Equals(FProfile::Str::FromUtf8(Field.name), ESearchCase::IgnoreCase))
		{
			return true;
		}
	}
	return false;
}

/**
 * Point every reference to `OldName` at `NewName` instead.
 *
 * Only references whose target set admits `TargetType` are rewritten: MuJoCo
 * names elements per category, so a body and a geom may both be called `torso`
 * and only one of them is what an actuator's `joint` attribute could mean.
 */
int32 RepointReferrers(
	const FSpecRef& Spec, psm::ElementType TargetType, const FString& OldName, const FString& NewName)
{
	int32 Updated = 0;
	ForEachElementOfSpec(Spec, [&](UMjNodeComponent& Node) {
		urlab::spec::gen::DispatchByType(Node, [&](auto& Element) {
			ForEachReference(Element,
				[&](int, const char*, auto&& Slot, const std::vector<psm::ElementType>& Targets) {
					if (!ps::sdk::detail::Contains(Targets, TargetType))
					{
						return;
					}
					if (!Slot.Get().Equals(FStringView(OldName), ESearchCase::CaseSensitive))
					{
						return;
					}
					// Its own undo record: the panel's transaction covers the
					// element the user typed into, not the ones being corrected.
					Node.Modify();
					Slot.Set(FStringView(NewName));
					++Updated;
				});
		});
	});
	return Updated;
}

#endif // WITH_EDITOR

/** True when the reference `Name` with target set `Targets` resolves in `Declared`. */
bool ReferenceResolves(const TMap<FString, TArray<int32>>& Declared, const FString& Name,
	const std::vector<psm::ElementType>& Targets)
{
	// MuJoCo's root default class is called `main` whether or not the model
	// spells it, so a `class="main"` on a model that declared no such block is
	// naming something real.
	if (Name == TEXT("main") && ps::sdk::detail::Contains(Targets, psm::ElementType::Default))
	{
		return true;
	}
	const TArray<int32>* Types = Declared.Find(Name);
	if (Types == nullptr)
	{
		return false;
	}
	for (psm::ElementType Target : Targets)
	{
		if (Types->Contains(static_cast<int32>(Target)))
		{
			return true;
		}
	}
	return false;
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
	const char* const Attribute = PropertyName == PosName  ? "pos"
								: PropertyName == QuatName ? "quat"
								: PropertyName == SizeName ? "size"
														   : "type";
	return ps::sdk::internal::FieldIdByName(Type, Attribute) >= 0;
}
} // namespace

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

// --- Scale, and what it is allowed to mean --------------------------------- //
//
// One implementation for every element, keyed by what the schema says the
// element carries rather than by which class it is. That is what puts `<site>`
// under the same per-type lock as `<geom>` without a second copy of the rule,
// and what makes an element MuJoCo adds tomorrow behave the day it is
// generated instead of the day someone remembers it.

namespace
{
/**
 * Why a drag on this shape is not an edit of its `size`, in the terms the user
 * can act on: which element DOES own the size they were trying to change.
 *
 * Every shape here is one MuJoCo itself sizes from somewhere other than the
 * element's own `size`, so there is always somewhere to send them.
 */
FString ScaleRefusalMessage(EMjGeomType Type)
{
	switch (Type)
	{
		case EMjGeomType::mesh:
			return TEXT("the scale handle does not author a mesh shape's size: it is the <mesh> asset's own, "
						"so scale the <mesh> element rather than this one");
		case EMjGeomType::hfield:
			return TEXT("the scale handle does not author a height field's size: it is the <hfield> asset's own, "
						"so scale the <hfield> element rather than this one");
		case EMjGeomType::sdf:
			return TEXT("the scale handle does not author an sdf shape's size: it comes from the geom's plugin");
		case EMjGeomType::plane:
			return TEXT("the scale handle does not author a plane's size: two of its three values are half-extents "
						"and the third is a grid spacing, and a zero half-extent means infinite");
		default:
			return TEXT("the scale handle does not author this shape's size");
	}
}
} // namespace

bool UMjNodeComponent::TryPreviewScaleFromSpec(FVector& OutScale) const
{
	EMjGeomType Type = EMjGeomType::sphere;
	TArray<double> Size;
	if (!urlab::spec::MjEffectiveShapeOf(*this, Type, Size))
	{
		return false;
	}
	const urlab::spec::FMjSizeShape& Shape = urlab::spec::MjSizeShapeFor(Type);
	if (Shape.AxisNum == 0)
	{
		return false;
	}

	// An unresolvable size -- too short for the type, or non-positive -- leaves
	// the scale alone rather than collapsing the element to nothing.
	const FVector Scale = urlab::spec::MjScaleFromSize(Shape, Size);
	if (Scale.GetMin() <= 0.0)
	{
		return false;
	}
	OutScale = Scale;
	return true;
}

bool UMjNodeComponent::HasScaleMapping() const
{
	EMjGeomType Type = EMjGeomType::sphere;
	TArray<double> Size;
	if (!urlab::spec::MjEffectiveShapeOf(*this, Type, Size))
	{
		return false;
	}
	const urlab::spec::FMjSizeShape& Shape = urlab::spec::MjSizeShapeFor(Type);
	return Shape.AxisNum > 0 && !Shape.bSizeIsReadOnly;
}

void UMjNodeComponent::ConstrainPreviewScale()
{
	const FVector Scale = GetRelativeScale3D();
	FVector Locked = Scale;

	EMjGeomType Type = EMjGeomType::sphere;
	TArray<double> Size;
	const bool bShaped = urlab::spec::MjEffectiveShapeOf(*this, Type, Size);
	const urlab::spec::FMjSizeShape& Shape = urlab::spec::MjSizeShapeFor(Type);
	const bool bScaleIsTheSize = bShaped && Shape.AxisNum > 0 && !Shape.bSizeIsReadOnly;

	if (bScaleIsTheSize)
	{
		// From the baseline where there is one, so the lock resolves toward the
		// handle that moved rather than toward a fixed axis. Without a baseline
		// there is no gesture to read and the fixed rule is all there is.
		if (LastPreviewTransform.IsSet())
		{
			urlab::spec::MjApplyScaleLockFrom(Shape.Lock, LastPreviewTransform->GetScale3D(), Locked);
		}
		else
		{
			urlab::spec::MjApplyScaleLock(Shape.Lock, Locked);
		}
	}
	else
	{
		// The refusal. Nothing about this element's `size` -- it has none, or has
		// one no scale expresses -- so a scaled component is a picture that
		// disagrees with what will be simulated, and the honest answer is to put
		// it back rather than to keep a distortion nobody can act on.
		//
		// A shape whose `size` is not a scale is the same refusal and used to be
		// silent: a mesh geom's handle moved, the write-back had nothing to author
		// and authored nothing, and the component kept a stretch that would never
		// be simulated. It snaps back to what the spec implies -- which for a mesh
		// is one, because a mesh geom's own scale says nothing about its picture --
		// exactly as a body's does.
		if (!TryPreviewScaleFromSpec(Locked))
		{
			Locked = FVector::OneVector;
		}
		if (bShaped)
		{
			// Only for a shape, because an element with no `size` at all has no
			// alternative to point the user at: the message below names the element
			// that DOES own the size, and there is one.
			NotePreviewProblem(EMjPreviewProblem::ScaleNotEditable, ScaleRefusalMessage(Type));
		}
	}

	if (!Locked.Equals(Scale))
	{
		SetRelativeScale3D(Locked);
	}
}

bool UMjNodeComponent::WriteBackScale(const FVector& Scale)
{
	EMjGeomType Type = EMjGeomType::sphere;
	TArray<double> Size;
	if (!urlab::spec::MjEffectiveShapeOf(*this, Type, Size))
	{
		return false;
	}
	const urlab::spec::FMjSizeShape& Shape = urlab::spec::MjSizeShapeFor(Type);
	if (Shape.AxisNum == 0 || Shape.bSizeIsReadOnly)
	{
		return false;
	}

	// Exactly the slots this type reads, which is the arity MuJoCo's own writer
	// trims a `size` to. A drag can therefore never author an over-long one.
	const TArray<double> Authored = urlab::spec::MjSizeFromScale(Shape, Scale);

	// And never a size of zero or less. MuJoCo's own `checksize` refuses one --
	// "size 0 must be positive" -- so authoring it turns a working model into one
	// that will not compile, from a gesture that looks like an ordinary drag.
	//
	// It is not a rare gesture either: the level viewport's scale grid steps in
	// 0.25 and most of a robot is smaller than that, so the first drag on a
	// centimetre-scale geom lands exactly on zero. The grid is the editor's and
	// stays as the user configured it; what changes is that the spec no longer
	// takes the collapse.
	for (const double Value : Authored)
	{
		if (Value <= 0.0)
		{
			NotePreviewProblem(EMjPreviewProblem::NonPositiveSize,
				FString::Printf(
					TEXT("a scale of %s would author a size of zero or less, so the size is unchanged: the "
						 "viewport's scale grid steps in 0.25 by default, which is larger than most of a robot"),
					*Scale.ToString()));
			return false;
		}
	}
	ClearPreviewProblem(EMjPreviewProblem::NonPositiveSize);

	using P = urlab::spec::FMjInstanceProfile;
	urlab::spec::gen::DispatchByType(*this, [&Authored](auto& Element) {
		using E = std::decay_t<decltype(Element)>;
		const int FieldId =
			pssdk::internal::FieldIdByName(urlab::spec::gen::TMjElementType<E>::Value, "size");
		if (FieldId < 0)
		{
			return;
		}
		double Slots[3] = {0.0, 0.0, 0.0};
		const int32 Num = FMath::Min(Authored.Num(), 3);
		for (int32 Index = 0; Index < Num; ++Index)
		{
			Slots[Index] = Authored[Index];
		}
		pssdk::internal::SetSeqField<P>(Element, FieldId, Slots, static_cast<std::size_t>(Num));
	});
	return true;
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
 *
 * Asked once per element by the callers below, so the scope it opens is opened
 * once per element too. That is affordable because a scope nested inside one
 * over the same Blueprint adopts the outer scope's maps and builds none: a pass
 * that already holds one open pays for one, not for one per element.
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
} // namespace

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

/**
 * Re-derive the picture of every element of an already-resolved spec.
 *
 * Taking the resolved spec rather than a node in it is what lets a fan-out over
 * several specs resolve each of them once: reaching a spec's root is itself a
 * walk, and doing it per pass instead of per query is the difference between
 * one and several.
 */
void RefreshSpecPresentationOf(const FSpecRef& Doc)
{
	UMjNodeComponent* Root = Doc.GetRoot();
	if (Root == nullptr)
	{
		return;
	}

	// One index for the whole walk. Each node asks the class chain several
	// questions and a geom asks more, and every one of those used to index the
	// spec from scratch -- so refreshing N elements cost N whole-spec walks per
	// question rather than one. The template graph comes with the scope, because
	// a spec held as Blueprint templates cannot be walked without it.
	urlab::spec::FMjEffectiveScope Effective(Doc);

#if WITH_EDITOR
	if (Doc.GetGraph() == EMjSpecGraph::Scs)
	{
		RefreshSubtree<urlab::spec::FMjScsAdapter>(*Root);
		return;
	}
#endif

	RefreshSubtree<urlab::spec::FMjInstanceAdapter>(*Root);
}
} // namespace

void UMjNodeComponent::RefreshSpecPresentation()
{
	RefreshSpecPresentationOf(FSpecRef::OverOwner(this));
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
	//
	// And a scale no layer authored previews at the shape's own default, which
	// for every element that has no resolvable size is one. NOT the component's
	// current scale: this transform is the write-back's baseline whenever the
	// cache is cold, and seeding it from the very component the user is dragging
	// folds the drag into the baseline -- "nothing moved", no snap, and a sphere
	// that previews as an ellipsoid while compiling as MuJoCo's default sphere.
	FVector Scale = FVector::OneVector;
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
	if (HasAnyFlags(RF_ClassDefaultObject))
	{
		return;
	}

	// The scale answer does not depend on the element having a pose, and tying
	// the two together is how a whole model came to be drawn at a scale MuJoCo
	// never sees. A `<mujoco>` root authors neither `pos` nor `quat`, so it left
	// here at once -- and it is the component a placed actor's own transform
	// lands on. Scale the actor and every geom under it is drawn stretched while
	// the simulation goes on using the `size` the gesture never touched, which is
	// the picture and the model saying different things.
	//
	// Every element the schema gives a `size` also gives a transform, so a
	// pose-less element is an unsized one and the scale it can hold is one. That
	// makes the guard exact rather than a heuristic, and it is a guard rather
	// than an unconditional call because resolving a shape indexes the spec.
	if (!HasPoseAttributes())
	{
		if (!GetRelativeScale3D().Equals(FVector::OneVector, MjPreviewEpsilon))
		{
			urlab::spec::FMjEffectiveScope Effective(*this);
			ConstrainPreviewScale();
		}
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

	// The scale is settled BEFORE anything is compared, so the comparison and
	// the spec both see a scale the element can actually hold. The lock used to
	// run after the early-out below, which meant it never ran at all in the one
	// case it exists for: a freshly added geom whose baseline was cold and whose
	// size was unset folded the dragged scale into its own baseline, "nothing
	// moved", and the non-uniform scale stood.
	//
	// Guarded on the scale differing, and that guard is the cost, not the
	// correctness: moving an actor delivers this hook to every descendant, not
	// one of whose relative transforms changed, and resolving a shape indexes
	// the entire spec once per element that asks. A descendant that did not move
	// matches its baseline on all three components, and its baseline scale came
	// from the spec through the shape's own mapping, so the snap would be a
	// no-op there by construction.
	const FVector CurrentScale = GetRelativeScale3D();
	TUniquePtr<urlab::spec::FMjEffectiveScope> Effective;
	if (!LastPreviewTransform->GetScale3D().Equals(CurrentScale, MjPreviewEpsilon))
	{
		Effective = MakeUnique<urlab::spec::FMjEffectiveScope>(*this);
		ConstrainPreviewScale();
	}

	if (LastPreviewTransform->Equals(
			FTransform(GetRelativeRotation().Quaternion(), GetRelativeLocation(), GetRelativeScale3D()),
			MjPreviewEpsilon))
	{
		return;
	}

	// Past the early-out, so the descendants of a moving actor never pay for it:
	// everything below resolves the class chain more than once, and each instance
	// below opens its own, an instance being a separate spec with its own root.
	if (!Effective.IsValid())
	{
		Effective = MakeUnique<urlab::spec::FMjEffectiveScope>(*this);
	}

	// The pose this element was at before the drag. It is the baseline the change
	// detector uses below, and it is also how the propagation at the end tells an
	// instance that was following this element from one that had moved itself.
	const FTransform Baseline = LastPreviewTransform.GetValue();
	const FTransform Current(GetRelativeRotation().Quaternion(), GetRelativeLocation(), GetRelativeScale3D());
	const bool bPosMoved = !LastPreviewTransform->GetLocation().Equals(Current.GetLocation(), MjPreviewEpsilon);
	const bool bRotMoved = HasQuatAttribute(*this) && !LastPreviewTransform->GetRotation().Equals(Current.GetRotation(), MjPreviewEpsilon);
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
	const bool bScaleAuthored = bScaleMoved && WriteBackScale(Current.GetScale3D());

	// A refused scale is a refusal in the viewport too. The spec still says what
	// it said, so a component left holding the scale that was refused is a
	// picture of a model that does not exist -- and for the collapse this refusal
	// exists for, that picture is a geom that has vanished. It goes back to the
	// pose it was dragged to and the scale the spec still holds.
	FTransform Settled = Current;
	if (bScaleMoved && !bScaleAuthored)
	{
		Settled.SetScale3D(Baseline.GetScale3D());
		SetRelativeScale3D(Settled.GetScale3D());
	}
	LastPreviewTransform = Settled;

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
	ForEachInstanceOfTemplate(FSpecRef::OverOwner(this), *this, [&](UMjNodeComponent& Instance) {
		const bool bTakePos =
			bPosMoved && Instance.GetRelativeLocation().Equals(Baseline.GetLocation(), MjPreviewEpsilon);
		const bool bTakeRot = bRotMoved && Instance.GetRelativeRotation().Quaternion().Equals(Baseline.GetRotation(), MjPreviewEpsilon);
		const bool bTakeScale =
			bScaleAuthored && Instance.GetRelativeScale3D().Equals(Baseline.GetScale3D(), MjPreviewEpsilon);
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

// --- Reference diagnostics -------------------------------------------------- //

namespace urlab::spec
{

template <class Adapter>
void MjNoteDanglingReferences(UMjNodeComponent& Root)
{
	// The names the model declares, and the element types declaring each. Both
	// halves matter: MuJoCo names elements per category, so `torso` naming a
	// body says nothing about whether an actuator's `joint="torso"` resolves.
	TMap<FString, TArray<int32>> Declared;
	WalkSpecTree<Adapter>(Root, [&Declared](UMjNodeComponent& Node) {
		psm::ElementType Type;
		if (!MjElementTypeOfNode(Node, Type))
		{
			return;
		}
		// Not `MjName`: an asset that omits `name` is still declared, under the
		// stem of the file it names, and MuJoCo resolves references to it by
		// that. Menagerie writes assets that way as a matter of course -- ten of
		// the eleven meshes in the Trossen arm -- so reading only the authored
		// name reports every one of them as dangling on a model that is
		// perfectly correct. The rule comes from the asset pass rather than
		// being restated, so the two cannot disagree about what a thing is
		// called.
		const FString DeclaredName = MjAssetElementName(Node);
		if (DeclaredName.IsEmpty())
		{
			return;
		}
		Declared.FindOrAdd(DeclaredName).AddUnique(static_cast<int32>(Type));
	});

	// Gathered rather than emitted as they are found: the pass runs on every
	// rename and every reference edit, and a message log constructed per pass
	// would touch the listing even on the ordinary run where nothing dangles.
	TArray<FString> Reported;

	WalkSpecTree<Adapter>(Root, [&Declared, &Reported](UMjNodeComponent& Node) {
		Node.DanglingReferences.Reset();
		gen::DispatchByType(Node, [&Declared, &Node, &Reported](auto& Element) {
			ForEachReference(Element, [&Declared, &Node, &Reported](int, const char* FieldName, auto&& Slot,
										  const std::vector<psm::ElementType>& Targets) {
				const FString Name(Slot.Get());
				if (Name.IsEmpty() || ReferenceResolves(Declared, Name, Targets))
				{
					return;
				}
				const FString Message =
					FString::Printf(TEXT("%hs=\"%s\" names no element this model declares"), FieldName, *Name);
				Node.DanglingReferences.Add(Message);

				// Named by the model's own name for the referrer where it has
				// one: the component name is an editor fact, and the name the
				// user typed the reference against is the MJCF one.
				Reported.Add(FString::Printf(
					TEXT("%s: %s"), *Node.MjName.Get(Node.GetName()), *Message));
			});
		});
	});

	// Two audiences, the same as every other diagnostic that cannot fail a
	// build: the run's log, and the editor's message log -- because a component
	// row is only seen by someone who has already selected the component, and
	// the whole difficulty with a dangling name is not knowing which one to look
	// at.
	for (const FString& Line : Reported)
	{
		UE_LOG(LogURLab, Warning, TEXT("%s"), *Line);
	}
#if WITH_EDITOR
	if (Reported.Num() > 0)
	{
		FMessageLog MessageLog(TEXT("URLab"));
		for (const FString& Line : Reported)
		{
			MessageLog.Warning(FText::FromString(Line));
		}
	}
#endif
}

template void MjNoteDanglingReferences<FMjInstanceAdapter>(UMjNodeComponent&);
#if WITH_EDITOR
template void MjNoteDanglingReferences<FMjScsAdapter>(UMjNodeComponent&);
#endif

} // namespace urlab::spec

#else // !URLAB_MJ_GEN

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

// The scale policy is a question about the schema, and without the generated
// profile there is no schema to ask: every element answers "no size" and the
// scale handle is left alone, which is what this build did before there was a
// policy at all.

bool UMjNodeComponent::TryPreviewScaleFromSpec(FVector& OutScale) const
{
	return false;
}

bool UMjNodeComponent::HasScaleMapping() const
{
	return false;
}

void UMjNodeComponent::ConstrainPreviewScale()
{
}

bool UMjNodeComponent::WriteBackScale(const FVector& Scale)
{
	return false;
}

#endif // URLAB_MJ_GEN

#if WITH_EDITOR

namespace
{
/** The member the MJCF `name` attribute is stored in. */
const FName GMjNamePropertyName(TEXT("MjName"));

#if URLAB_MJ_GEN
/** Re-derive the reference diagnostics of the whole spec `Spec` names. */
void RefreshSpecReferenceDiagnostics(const FSpecRef& Spec)
{
	UMjNodeComponent* Root = Spec.GetRoot();
	if (Root == nullptr)
	{
		return;
	}
	if (Spec.GetGraph() == EMjSpecGraph::Scs)
	{
		if (UBlueprint* Blueprint = Spec.GetBlueprint())
		{
			urlab::spec::FMjScsScope Scope(*Blueprint);
			urlab::spec::MjNoteDanglingReferences<urlab::spec::FMjScsAdapter>(*Root);
		}
		return;
	}
	urlab::spec::MjNoteDanglingReferences<urlab::spec::FMjInstanceAdapter>(*Root);
}

/**
 * Elements already reported, by identity rather than by object.
 *
 * A component is not the same object from one Blueprint reconstruct to the
 * next, and a rule broken once should be said once -- not on every recompile for
 * the rest of the session. `Serial` survives a reconstruct and a genuinely new
 * element mints a fresh one, so it separates "this again" from "another one".
 *
 * Cleared for an element whose placement becomes legal, because putting the same
 * element somewhere illegal a second time is a new mistake.
 */
TSet<uint64> GReportedIllegalPlacement;
#endif // URLAB_MJ_GEN
} // namespace

void UMjNodeComponent::CheckPlacementLegality()
{
#if URLAB_MJ_GEN
	PlacementProblems.Reset();

	// A class default object is not placed anywhere, and a template is not
	// registered at all; both reach here only through some other path.
	if (HasAnyFlags(RF_ClassDefaultObject))
	{
		return;
	}

	const UMjNodeComponent* const Parent = Cast<UMjNodeComponent>(GetAttachParent());
	if (Parent == nullptr)
	{
		// An organisational folder or the actor root: not an element, so the
		// schema has nothing to say about the pair. The import pass owns where
		// those go.
		return;
	}

	psm::ElementType ChildType{};
	psm::ElementType ParentType{};
	if (!urlab::spec::MjElementTypeOfNode(*this, ChildType)
		|| !urlab::spec::MjElementTypeOfNode(*Parent, ParentType))
	{
		return;
	}

	if (urlab::spec::gen::SlotFor(ParentType, ChildType) >= 0)
	{
		GReportedIllegalPlacement.Remove(Serial);
		return;
	}

	const FString Message = FString::Printf(
		TEXT("<%s> is not a legal child of <%s>: it will be dropped when the model compiles"),
		urlab::spec::gen::TagForElement(ChildType), urlab::spec::gen::TagForElement(ParentType));
	PlacementProblems.Add(Message);

	// The row above is the persistent surface and is rewritten every time. The
	// two logs are a transition: said when the element becomes illegally placed,
	// not once per registration for the rest of the session.
	if (GReportedIllegalPlacement.Contains(Serial))
	{
		return;
	}
	GReportedIllegalPlacement.Add(Serial);

	const FString Line = FString::Printf(TEXT("%s: %s"), *MjName.Get(GetName()), *Message);
	UE_LOG(LogURLab, Warning, TEXT("%s (parent '%s')"), *Line, *Parent->MjName.Get(Parent->GetName()));
	FMessageLog(TEXT("URLab")).Warning(FText::FromString(Line));
#endif // URLAB_MJ_GEN
}

void UMjNodeComponent::PostEditComponentMove(bool bFinished)
{
	Super::PostEditComponentMove(bFinished);
	// Deliberately not gated on bFinished: an SCS template only ever receives
	// false, and the change detector makes the per-delta invocations cheap.
	WriteBackTransformIfChanged();
}

void UMjNodeComponent::PreEditChange(FProperty* PropertyAboutToChange)
{
	Super::PreEditChange(PropertyAboutToChange);
	if (PropertyAboutToChange != nullptr && PropertyAboutToChange->GetFName() == GMjNamePropertyName)
	{
		NameBeforeEdit = MjName;
	}

	// The value the instances following this template are still holding. It only
	// exists now: see the member's comment.
	PropertyBeforeEdit = nullptr;
	PropertyTextBeforeEdit.Reset();
	const UClass* const Owner = PropertyAboutToChange != nullptr ? PropertyAboutToChange->GetOwnerClass() : nullptr;
	if (Owner != nullptr && GetOwner() == nullptr && !HasAnyFlags(RF_ClassDefaultObject) && IsA(Owner))
	{
		PropertyBeforeEdit = PropertyAboutToChange;
		PropertyAboutToChange->ExportTextItem_Direct(PropertyTextBeforeEdit,
			PropertyAboutToChange->ContainerPtrToValuePtr<void>(this), nullptr, this, PPF_None);
	}
}

void UMjNodeComponent::CarryEditToInstances(const FSpecRef& Doc, const FPropertyChangedEvent& Event)
{
#if URLAB_MJ_GEN
	FProperty* const Property = PropertyBeforeEdit;
	const FString Before = PropertyTextBeforeEdit;
	PropertyBeforeEdit = nullptr;
	PropertyTextBeforeEdit.Reset();

	if (Property == nullptr || GetOwner() != nullptr)
	{
		return;
	}

	// The snapshot has to belong to the edit that just landed. `PreEditChange`
	// and `PostEditChangeProperty` are not always a matched pair -- a cancelled
	// edit fires only the first -- so a stale snapshot must not be spent on the
	// next property that happens along.
	const FName Name = Property->GetFName();
	if (Event.Property != Property && Event.MemberProperty != Property && Event.GetPropertyName() != Name && Event.GetMemberPropertyName() != Name)
	{
		return;
	}

	FString After;
	Property->ExportTextItem_Direct(After, Property->ContainerPtrToValuePtr<void>(this), nullptr, this, PPF_None);
	if (After == Before)
	{
		return;
	}

	ForEachInstanceOfTemplate(Doc, *this, [Property, &Before, &After](UMjNodeComponent& Instance) {
		if (!Instance.IsA(Property->GetOwnerClass()))
		{
			return;
		}
		void* const Slot = Property->ContainerPtrToValuePtr<void>(&Instance);
		FString Held;
		Property->ExportTextItem_Direct(Held, Slot, nullptr, &Instance, PPF_None);
		if (Held != Before)
		{
			// This instance authored its own value: the template is no longer
			// what decides it, and taking that back would discard the user's edit.
			return;
		}

		Instance.Modify();
		Property->ImportText_Direct(*After, Slot, &Instance, PPF_None);

		FPropertyChangedEvent Carried(Property, EPropertyChangeType::ValueSet);
		Instance.PostEditChangeProperty(Carried);
	});
#else
	// Without the generated profile there is no spec to walk and no template
	// graph to find instances in.
	(void)Doc;
	(void)Event;
#endif // URLAB_MJ_GEN
}

void UMjNodeComponent::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	const FName Changed = PropertyChangedEvent.GetPropertyName();
	const FName Member = PropertyChangedEvent.MemberProperty != nullptr
						   ? PropertyChangedEvent.MemberProperty->GetFName()
						   : NAME_None;

	// The member, not the innermost name. Typing into one field of the transform
	// sends the float inside the vector -- `X` -- and comparing that against
	// `RelativeScale3D` never matched, so the panel moved the component and the
	// write-back never ran: no authoring, and for a geom no scale lock either, so
	// a sphere took an X-only stretch and kept it. A whole-vector set reports the
	// vector in both fields, so reading the member covers the gizmo path too.
	const FName Transform = (Member != NAME_None) ? Member : Changed;

	if (Transform == USceneComponent::GetRelativeLocationPropertyName() || Transform == USceneComponent::GetRelativeRotationPropertyName() || Transform == USceneComponent::GetRelativeScale3DPropertyName())
	{
		// The transform carries itself: the write-back authors the spec and
		// hands the authored value to the instances that were following it. The
		// snapshot is spent here so it cannot be applied to the next edit.
		PropertyBeforeEdit = nullptr;
		PropertyTextBeforeEdit.Reset();
		WriteBackTransformIfChanged();
		return;
	}

#if URLAB_MJ_GEN
	psm::ElementType Type;
	const bool bIsElement = urlab::spec::MjElementTypeOfNode(*this, Type);

	// Resolved once for the whole hook. Reaching the spec's root is a walk of the
	// object graph, and every branch below wants the same answer.
	const FSpecRef Doc = FSpecRef::OverOwner(this);

	// A rename is not a local edit. Every reference in the model is a name, so
	// the elements pointing here are pointing at a name that no longer exists,
	// and MuJoCo would report that from the compiler with nothing to blame it
	// on. They are corrected in the same transaction, so one undo puts both
	// sides back.
	if (bIsElement && (Changed == GMjNamePropertyName || Member == GMjNamePropertyName))
	{
		const FString OldName = NameBeforeEdit.IsSet() ? NameBeforeEdit.GetValue() : FString();
		const FString NewName = MjName.IsSet() ? MjName.GetValue() : FString();
		// An element that gains its first name had nothing pointing at it, and
		// one that loses its name leaves references that are now genuinely
		// dangling -- clearing them would discard what the user authored.
		if (!OldName.IsEmpty() && !NewName.IsEmpty() && OldName != NewName)
		{
			RepointReferrers(Doc, Type, OldName, NewName);
		}
		NameBeforeEdit.Reset();
		RefreshSpecReferenceDiagnostics(Doc);
	}
	else if (bIsElement && (IsReferenceField(Type, Changed) || IsReferenceField(Type, Member)))
	{
		// Typing a name into a reference is the other way to make one dangle,
		// and the user finds out here rather than at play.
		RefreshSpecReferenceDiagnostics(Doc);
	}

	// The picture, under one index for the whole of it. Everything below asks the
	// same spec the same structural questions -- what class chain an element
	// resolves through, which node holds which template -- and each of those used
	// to index the spec on its own. It opens after the branches above because
	// those can rewrite names, and the index is keyed by them.
	urlab::spec::FMjEffectiveScope Presentation(Doc);

	// Typing into a spatial attribute has to move the viewport. Which names those
	// are comes from the schema's field-id lookup, not from a list kept by hand
	// here.
	if (bIsElement && (IsSpatialField(Type, Changed) || IsSpatialField(Type, Member)))
	{
		SyncPreviewFromSpec();
	}

	// The element that was edited is the whole answer for content. A shared node
	// is an input to other elements' pictures, and none of them re-reads on its
	// own.
	if (IsSharedPresentationInput())
	{
		// The edited spec is walked once, from the root already resolved above.
		RefreshSpecPresentationOf(Doc);

		// A Blueprint template's own spec is the template graph, which draws
		// nothing. What the user is looking at is the preview actor -- and every
		// placed actor of that class -- so the instances built from this template
		// have to re-read their own specs too.
		//
		// Each instance is a spec of its own, with its own root and its own class
		// chain, so it gets one walk and one index -- one, not one per element
		// and not one per question, which is what resolving the instance's root
		// per query used to cost.
		ForEachInstanceOfTemplate(Doc, *this, [](UMjNodeComponent& Instance) {
			RefreshSpecPresentationOf(FSpecRef::OverOwner(&Instance));
		});
	}

	// Last, because it re-enters this hook on each instance and everything above
	// is about the template's own spec.
	CarryEditToInstances(Doc, PropertyChangedEvent);
#endif
}

void UMjNodeComponent::PostEditUndo()
{
	Super::PostEditUndo();
	// Undo restores properties without firing a change hook, so the baseline is
	// stale and the preview may disagree with the spec. Re-derive both.
	SyncPreviewUnderOneScope();
}

#endif // WITH_EDITOR
