// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSourceBundle.h"
#include "UObject/ObjectSaveContext.h"

#include "Engine/Blueprint.h"
#include "Engine/InheritableComponentHandler.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformAtomics.h"
#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjNodeRefOptions.h"
#include "MuJoCo/Spec/MjNodeDiagnostics.h"
#include "MuJoCo/Spec/MjNodeScale.h"
#include "MuJoCo/Spec/MjNodePresentation.h"
#include "MuJoCo/Spec/MjNodeEditorCarry.h"
#include "MuJoCo/Spec/MjEffective.h"
#include "MuJoCo/Spec/MjElementIdentity.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"
#include "MjReservedNames.h"
#include "MuJoCo/Gen/MjReflect.gen.h"
#include "MuJoCo/Gen/MjVisit.gen.h"
#include "UObject/UObjectHash.h"
#include "Utils/URLabLogging.h"

#if URLAB_PROTOSPEC
THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
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
 * unnamed element compiles under are not derived from one either: they count
 * their family in document order instead, so they repeat across runs.
 */
std::atomic<uint64> GMjSerialCounter{0};
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

void UMjNodeComponent::PreSave(FObjectPreSaveContext SaveContext)
{
#if WITH_EDITOR
	FString ResourceError;
	if (!MjPrepareAssetResource(*this, ResourceError))
	{
		if (SaveContext.IsCooking())
		{
			UE_LOG(LogURLab, Error, TEXT("MuJoCo asset preparation failed for '%s' during cook: %s"),
				*GetPathName(), *ResourceError);
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("MuJoCo asset preparation failed for '%s': %s"),
				*GetPathName(), *ResourceError);
		}
	}
#endif
#if WITH_EDITOR && URLAB_MJ_GEN
	const FSpecRef Spec = FSpecRef::OverOwner(this);
	FString ReferenceError;
	if (!urlab::spec::MjSynchronizeReferenceBindings(Spec, ReferenceError, this))
	{
		UE_LOG(LogURLab, Error, TEXT("MuJoCo reference identity validation failed for '%s': %s"),
			*GetPathName(), *ReferenceError);
	}
#endif
	Super::PreSave(SaveContext);
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
	// default-class chain; indexing the entire spec per question would cost N
	// of those for a model of N elements, the shape the import path already
	// avoids by holding one scope open across its walk.
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

void UMjNodeComponent::NotePreviewProblem(EMjPreviewProblem Problem, const FString& Message)
{
	urlab::spec::MjNodeNotePreviewProblem(*this, Problem, Message);
}

void UMjNodeComponent::ClearPreviewProblem(EMjPreviewProblem Problem)
{
	urlab::spec::MjNodeClearPreviewProblem(*this, Problem);
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
namespace gen = urlab::spec::gen;

// --- References ------------------------------------------------------------ //
//
// Every cross-reference in a MuJoCo model is a NAME with a typed target, never a
// pointer: an actuator names the joint it drives, a geom names its material. The
// schema says which attributes those are and what each may point at, so
// everything below is reflection-driven and no element type is named by hand.

bool TargetsContain(const std::vector<psm::ElementType>& Targets, psm::ElementType Want)
{
	for (const psm::ElementType Type : Targets) if (Type == Want) return true;
	return false;
}

std::vector<psm::ElementType> DynamicRefTargets(FStringView Keyword)
{
	const FTCHARToUTF8 Utf8(Keyword.GetData(), Keyword.Len());
	int32 ObjectType = mju_str2Type(Utf8.Get());
	// Both body object spellings refer to the same authored body element.
	if (ObjectType == mjOBJ_XBODY) ObjectType = mjOBJ_BODY;
	if (ObjectType == mjOBJ_UNKNOWN) return {};
	std::vector<psm::ElementType> Out;
	gen::ForEachElementType([&](psm::ElementType Type) {
		if (urlab::spec::MjBindableObjType(Type) == ObjectType) Out.push_back(Type);
	});
	return Out;
}

template <class E, class OnRef>
struct FReferenceVisitor
{
	E& Element;
	OnRef& On;
	const gen::FMjElementThunks& Fields;
	psm::ElementType Type;

	template <class U>
	void field(int Id, const char* Name, U& Value)
	{
		using V = std::decay_t<U>;
		if constexpr (urlab::ref_view_detail::TIsRefView<V>::value)
		{
			if (Value.IsSet()) On(Id, Name, Value, gen::RefTargetsAt(Type, Id));
		}
		else if constexpr (std::is_same_v<V, FString> || std::is_same_v<V, TOptional<FString>>)
		{
			if (Id < 0 || Id >= Fields.FieldCount) return;
			const gen::FMjFieldSpec& Field = Fields.Fields[Id];
			if (Field.TargetFrom == nullptr || Field.TargetFrom[0] == 0) return;
			int32 Sibling = -1;
			for (int32 Candidate = 0; Candidate < Fields.FieldCount; ++Candidate)
				if (FStringView(Fields.Fields[Candidate].XmlName).Equals(Field.TargetFrom,
					ESearchCase::CaseSensitive)) { Sibling = Candidate; break; }
			FString Keyword;
			if (Sibling < 0 || !Fields.Format(&Element, Sibling, Keyword)) return;
			const std::vector<psm::ElementType> Targets = DynamicRefTargets(Keyword);
			if (Targets.empty()) return;
			urlab::RefView<UMjNodeComponent, U> Ref{Value};
			if (Ref.IsSet()) On(Id, Name, Ref, Targets);
		}
	}
	template <class C> void child(int, const char*, C&) {}
	template <class C> void union_child(int, const char*, C&) {}
};

/**
 * Invoke `On` for every authored reference of one element.
 *
 * Two kinds. The generated Visit hands typed references out with their target
 * sets. The dynamic ones -- MuJoCo's frame sensors, whose `objname` means
 * whatever the sibling `objtype` says -- are plain strings the schema marks with
 * the sibling that types them, so the descriptor is what resolves them and they
 * are reached with no per-element code either.
 *
 * `On` is called as `On(FieldId, FieldName, RefView, Targets)`. The view aliases
 * the stored name for the duration of the callback, so rename edits it in place.
 */
template <class E, class OnRef>
void ForEachReference(E& Element, OnRef&& On)
{
	const psm::ElementType Type = gen::TMjElementType<std::decay_t<E>>::Value;
	FReferenceVisitor<E, std::remove_reference_t<OnRef>> Visitor{
		Element, On, gen::Thunks(Type), Type};
	gen::Visit(Element, Visitor);
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
	const gen::FMjElementThunks& Descriptor = gen::Thunks(Type);
	for (int32 FieldId = 0; FieldId < Descriptor.FieldCount; ++FieldId)
	{
		const gen::FMjFieldSpec& Field = Descriptor.Fields[FieldId];
		if (Field.Kind != gen::EMjFieldKind::Ref && (Field.TargetFrom == nullptr || Field.TargetFrom[0] == 0))
		{
			continue;
		}
		if (Wanted.Equals(Field.MemberName, ESearchCase::IgnoreCase))
		{
			return true;
		}
	}
	return false;
}

/**
 * Get the template a rename fixup is allowed to write.
 *
 * A derived Blueprint's effective tree includes ancestor SCS templates, but
 * writing one of those directly would change the parent Blueprint. Ask UE for
 * the child's normal inheritable-component override, creating it only for an
 * element that actually needs a reference edit.
 */
UMjNodeComponent* EditableReferrer(const FSpecRef& Spec, UMjNodeComponent& Node)
{
	if (Spec.GetGraph() != EMjSpecGraph::Scs || Spec.GetBlueprint() == nullptr)
	{
		return &Node;
	}
	UBlueprint* const Blueprint = Spec.GetBlueprint();
	urlab::spec::FMjScsScope Scope(*Blueprint);
	USCS_Node* const ScsNode = Scope.FindNode(Node);
	if (ScsNode == nullptr || ScsNode->GetSCS() == &Scope.GetScs())
	{
		return &Node;
	}

	UInheritableComponentHandler* const Handler = Blueprint->GetInheritableComponentHandler(true);
	if (Handler == nullptr)
	{
		return nullptr;
	}
	const FComponentKey Key(ScsNode);
	UActorComponent* Template = Handler->GetOverridenComponentTemplate(Key);
	if (Template == nullptr)
	{
		Template = Handler->CreateOverridenComponentTemplate(Key);
		if (Template != nullptr)
		{
			Scope.InvalidateNodeMap();
		}
	}
	return Cast<UMjNodeComponent>(Template);
}

/**
 * Point every reference to `OldName` at `NewName` instead.
 *
 * Only references whose target set admits `TargetType` are rewritten: MuJoCo
 * names elements per category, so a body and a geom may both be called `torso`
 * and only one of them is what an actuator's `joint` attribute could mean.
 */
int32 RepointReferrers(
	const FSpecRef& Spec, UMjNodeComponent& RenamedTarget, psm::ElementType TargetType,
	const FString& OldName, const FString& NewName)
{
	int32 Updated = 0;
	FComponentKey TargetKey;
	UBlueprint* Blueprint = Spec.GetBlueprint();
	UMjNodeComponent* KeyTemplate = &RenamedTarget;
	if (Spec.GetGraph() == EMjSpecGraph::Instance)
	{
		AActor* const Owner = Spec.GetActor();
		Blueprint = Owner != nullptr ? Cast<UBlueprint>(Owner->GetClass()->ClassGeneratedBy) : nullptr;
		KeyTemplate = Cast<UMjNodeComponent>(RenamedTarget.GetArchetype());
	}
	if (Blueprint != nullptr && KeyTemplate != nullptr)
	{
		urlab::spec::FMjScsScope Scope(*Blueprint);
		if (USCS_Node* const TargetNode = Scope.FindNode(*KeyTemplate)) TargetKey = FComponentKey(TargetNode);
	}
	bool bOldNameIsAmbiguous = false;
	ForEachElementOfSpec(Spec, [&](UMjNodeComponent& Candidate) {
		psm::ElementType CandidateType;
		if (&Candidate != &RenamedTarget && Candidate.MjName.IsSet() && Candidate.MjName.GetValue() == OldName &&
			urlab::spec::MjElementTypeOfNode(Candidate, CandidateType) && CandidateType == TargetType)
		{
			bOldNameIsAmbiguous = true;
		}
	});
	ForEachElementOfSpec(Spec, [&](UMjNodeComponent& EffectiveNode) {
	TSet<int32> MatchingFields;
	urlab::spec::gen::DispatchByType(EffectiveNode, [&](auto& Element) {
			ForEachReference(Element,
				[&](int FieldId, const char* FieldName, auto&& Slot, const std::vector<psm::ElementType>& Targets) {
					if (!TargetsContain(Targets, TargetType))
					{
						return;
					}
					const FName Field(UTF8_TO_TCHAR(FieldName));
					if (EffectiveNode.UnresolvedReferenceFields.Contains(Field)) return;
					if (const FComponentKey* BoundKey = EffectiveNode.ReferenceTargets.Find(Field))
					{
						if (!TargetKey.IsValid() || !BoundKey->Match(TargetKey)) return;
					}
					else if (bOldNameIsAmbiguous ||
						!Slot.Get().Equals(FStringView(OldName), ESearchCase::CaseSensitive))
					{
						return;
					}
					MatchingFields.Add(FieldId);
				});
		});
	if (MatchingFields.IsEmpty()) return;

	UMjNodeComponent* const WritableNode = EditableReferrer(Spec, EffectiveNode);
	if (WritableNode == nullptr)
	{
		UE_LOG(LogURLab, Error, TEXT("Could not create a Blueprint override while renaming a MuJoCo reference"));
		return;
	}
	urlab::spec::gen::DispatchByType(*WritableNode, [&](auto& WritableElement) {
		ForEachReference(WritableElement,
			[&](int FieldId, const char* FieldName, auto&& Slot, const std::vector<psm::ElementType>& Targets) {
				const FName Field(UTF8_TO_TCHAR(FieldName));
				const FComponentKey* const BoundKey = WritableNode->ReferenceTargets.Find(Field);
				const bool bBoundToRenamedTarget = TargetKey.IsValid() && BoundKey != nullptr && BoundKey->Match(TargetKey);
				if (!MatchingFields.Contains(FieldId) || !TargetsContain(Targets, TargetType) ||
					(!bBoundToRenamedTarget && !Slot.Get().Equals(FStringView(OldName), ESearchCase::CaseSensitive))) return;
				// Its own undo record: the panel's transaction covers the element
				// the user typed into, not the inherited referrer being corrected.
				WritableNode->Modify();
				Slot.Set(FStringView(NewName));
				if (TargetKey.IsValid()) WritableNode->ReferenceTargets.Add(Field, TargetKey);
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
	if (Name == TEXT("main") && TargetsContain(Targets, psm::ElementType::Default))
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
	return urlab::spec::MjSchemaFieldOf(Type, Attribute) != nullptr;
}
} // namespace

namespace urlab::spec
{
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
void MjNodeForEachInstanceOfTemplate(
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
} // namespace urlab::spec

bool UMjNodeComponent::HasPoseAttributes() const
{
	return urlab::spec::MjNodeHasPoseAttributes(*this);
}

// --- Scale, and what it is allowed to mean --------------------------------- //
//
// One implementation for every element, keyed by what the schema says the
// element carries rather than by which class it is. That is what puts `<site>`
// under the same per-type lock as `<geom>` without a second copy of the rule,
// and what makes an element MuJoCo adds tomorrow behave the day it is
// generated instead of the day someone remembers it.

bool UMjNodeComponent::TryPreviewScaleFromSpec(FVector& OutScale) const
{
	return urlab::spec::MjNodeTryPreviewScaleFromSpec(*this, OutScale);
}

bool UMjNodeComponent::HasScaleMapping() const
{
	return urlab::spec::MjNodeHasScaleMapping(*this);
}

void UMjNodeComponent::ConstrainPreviewScale()
{
	urlab::spec::MjNodeConstrainPreviewScale(*this);
}

bool UMjNodeComponent::WriteBackScale(const FVector& Scale)
{
	return urlab::spec::MjNodeWriteBackScale(*this, Scale);
}

// --- What an element is in the spec ------------------------------------ //

bool UMjNodeComponent::IsClassPartial() const
{
	return urlab::spec::MjNodeIsClassPartial(*this);
}

bool UMjNodeComponent::IsSharedPresentationInput() const
{
	return urlab::spec::MjNodeIsSharedPresentationInput(*this);
}

// --- Presentation ----------------------------------------------------------- //

void UMjNodeComponent::RefreshPresentation()
{
	urlab::spec::MjNodeRefreshPresentation(*this);
}

void UMjNodeComponent::RefreshSpecPresentation()
{
	urlab::spec::MjNodeRefreshSpecPresentation(*this);
}

bool UMjNodeComponent::ComputePreviewTransform(FTransform& Out)
{
	return urlab::spec::MjNodeComputePreviewTransform(*this, Out);
}

void UMjNodeComponent::SyncPreviewFromSpec()
{
	urlab::spec::MjNodeSyncPreviewFromSpec(*this);
}

void UMjNodeComponent::WriteBackTransformIfChanged()
{
	urlab::spec::MjNodeWriteBackTransformIfChanged(*this);
}

// --- Reference diagnostics -------------------------------------------------- //

namespace urlab::spec
{

#if WITH_EDITOR
namespace
{
UBlueprint* BlueprintForSpec(const FSpecRef& Spec)
{
	if (UBlueprint* Blueprint = Spec.GetBlueprint()) return Blueprint;
	if (AActor* Actor = Spec.GetActor())
	{
		if (UClass* Class = Actor->GetClass()) return Cast<UBlueprint>(Class->ClassGeneratedBy);
	}
	return nullptr;
}

bool ScsKeyForNode(const FSpecRef& Spec, const UMjNodeComponent& Node, FComponentKey& OutKey)
{
	UBlueprint* const Blueprint = BlueprintForSpec(Spec);
	if (Blueprint == nullptr) return false;
	FMjScsScope Scope(*Blueprint);
	const UMjNodeComponent* KeyTemplate = &Node;
	if (Spec.GetGraph() == EMjSpecGraph::Instance)
		KeyTemplate = Cast<UMjNodeComponent>(Node.GetArchetype());
	if (KeyTemplate == nullptr) return false;
	USCS_Node* const SCSNode = Scope.FindNode(*const_cast<UMjNodeComponent*>(KeyTemplate));
	if (SCSNode == nullptr) return false;
	OutKey = FComponentKey(SCSNode);
	return OutKey.IsValid();
}

bool IsWritableInBlueprint(const FSpecRef& Spec, UMjNodeComponent& Node)
{
	UBlueprint* const Blueprint = Spec.GetBlueprint();
	if (Spec.GetGraph() != EMjSpecGraph::Scs || Blueprint == nullptr) return false;
	FMjScsScope Scope(*Blueprint);
	USCS_Node* const SCSNode = Scope.FindNode(Node);
	if (SCSNode == nullptr) return false;
	if (SCSNode->GetSCS() == &Scope.GetScs()) return true;
	UInheritableComponentHandler* const Handler = Blueprint->GetInheritableComponentHandler(false);
	return Handler != nullptr && Handler->GetOverridenComponentTemplate(FComponentKey(SCSNode)) == &Node;
}

bool ResolveUniqueReferenceKey(const FSpecRef& Spec, const FString& Name,
	const std::vector<psm::ElementType>& Targets, FComponentKey& OutKey)
{
	int32 MatchCount = 0;
	ForEachElementOfSpec(Spec, [&](UMjNodeComponent& Candidate) {
		psm::ElementType CandidateType;
		if (!Candidate.MjName.IsSet() || Candidate.MjName->IsEmpty() || Candidate.MjName.GetValue() != Name ||
			!urlab::spec::MjElementTypeOfNode(Candidate, CandidateType) || !TargetsContain(Targets, CandidateType))
		{
			return;
		}
		FComponentKey CandidateKey;
		if (!ScsKeyForNode(Spec, Candidate, CandidateKey)) return;
		if (MatchCount > 0 && OutKey.Match(CandidateKey)) return;
		OutKey = CandidateKey;
		++MatchCount;
	});
	return MatchCount == 1;
}

UMjNodeComponent* EffectiveTargetForKey(const FSpecRef& Spec, const FComponentKey& Key)
{
	UBlueprint* const Blueprint = BlueprintForSpec(Spec);
	if (Blueprint == nullptr) return nullptr;
	USCS_Node* const TargetNode = Key.FindSCSNode();
	if (TargetNode == nullptr) return nullptr;
	FMjScsScope Scope(*Blueprint);
	if (Spec.GetGraph() == EMjSpecGraph::Scs) return Scope.TemplateFor(*TargetNode);
	AActor* const Actor = Spec.GetActor();
	if (Actor == nullptr) return nullptr;
	TArray<UMjNodeComponent*> Components;
	Actor->GetComponents(Components);
	UMjNodeComponent* Found = nullptr;
	for (UMjNodeComponent* Candidate : Components)
	{
		FComponentKey CandidateKey;
		if (Candidate != nullptr && ScsKeyForNode(Spec, *Candidate, CandidateKey) && CandidateKey.Match(Key))
		{
			if (Found != nullptr) return nullptr;
			Found = Candidate;
		}
	}
	return Found;
}

FName ReferenceFieldKey(const char* FieldName)
{
	return FName(UTF8_TO_TCHAR(FieldName));
}
} // namespace

void MjRebindEditedReference(UMjNodeComponent& Node, const FSpecRef& Spec, FName Field)
{
#if WITH_EDITORONLY_DATA
	Node.ReferenceTargets.Remove(Field);
	Node.UnresolvedReferenceFields.Remove(Field);
	FComponentKey ResolvedKey;
	bool bMatchedField = false;
	urlab::spec::gen::DispatchByType(Node, [&](auto& Element) {
		ForEachReference(Element, [&](int, const char* FieldName, auto&& Slot,
			const std::vector<psm::ElementType>& Targets) {
			if (ReferenceFieldKey(FieldName) != Field) return;
			bMatchedField = true;
			const FString Value(Slot.Get());
			if (!Value.IsEmpty() && ResolveUniqueReferenceKey(Spec, Value, Targets, ResolvedKey))
			{
				Node.Modify();
				Node.ReferenceTargets.Add(Field, ResolvedKey);
			}
		});
	});
	(void)bMatchedField;
#else
	(void)Node;
	(void)Spec;
	(void)Field;
#endif
}

bool MjSynchronizeReferenceBindings(const FSpecRef& Spec, FString& OutError, UMjNodeComponent* OnlyNode)
{
#if WITH_EDITORONLY_DATA
	if (Spec.GetRoot() == nullptr) return true;
	bool bValid = true;
	auto SyncNode = [&](UMjNodeComponent& EffectiveNode) {
		struct FPendingReferenceWrite
		{
			FName Field;
			FString TargetName;
			FComponentKey TargetKey;
			psm::ElementType TargetType;
			std::vector<psm::ElementType> Targets;
		};
		TArray<FPendingReferenceWrite> PendingWrites;
		urlab::spec::gen::DispatchByType(EffectiveNode, [&](auto& Element) {
			ForEachReference(Element, [&](int, const char* FieldName, auto&& Slot,
				const std::vector<psm::ElementType>& Targets) {
				const FName Field = ReferenceFieldKey(FieldName);
				if (EffectiveNode.UnresolvedReferenceFields.Contains(Field))
				{
					OutError = FString::Printf(TEXT("reference field '%s' on '%s' has an unresolved component identity"),
						*Field.ToString(), *EffectiveNode.GetPathName());
					bValid = false;
					return;
				}

				FComponentKey* Binding = EffectiveNode.ReferenceTargets.Find(Field);
				if (Binding == nullptr)
				{
					if (Spec.GetGraph() == EMjSpecGraph::Scs && IsWritableInBlueprint(Spec, EffectiveNode))
					{
						FComponentKey ResolvedKey;
						const FString Value(Slot.Get());
					if (!Value.IsEmpty() && ResolveUniqueReferenceKey(Spec, Value, Targets, ResolvedKey))
						{
							EffectiveNode.Modify();
							EffectiveNode.ReferenceTargets.Add(Field, ResolvedKey);
						}
					}
					return; // unbound raw text, including macro names, remains user-authored.
				}

				UMjNodeComponent* Target = EffectiveTargetForKey(Spec, *Binding);
				psm::ElementType TargetType;
				if (Target == nullptr || !urlab::spec::MjElementTypeOfNode(*Target, TargetType) ||
					!TargetsContain(Targets, TargetType) || !Target->MjName.IsSet() || Target->MjName->IsEmpty())
				{
					EffectiveNode.Modify();
					EffectiveNode.ReferenceTargets.Remove(Field);
					EffectiveNode.UnresolvedReferenceFields.Add(Field);
					OutError = FString::Printf(TEXT("reference field '%s' on '%s' targets a deleted or incompatible component"),
						*Field.ToString(), *EffectiveNode.GetPathName());
					bValid = false;
					return;
				}

				if (!Slot.Get().Equals(FStringView(Target->MjName.GetValue()), ESearchCase::CaseSensitive))
				{
					FPendingReferenceWrite& Pending = PendingWrites.AddDefaulted_GetRef();
					Pending.Field = Field;
					Pending.TargetName = Target->MjName.GetValue();
					Pending.TargetKey = *Binding;
					Pending.TargetType = TargetType;
					Pending.Targets = Targets;
				}
			});
		});

		// Keep this generated dispatch separate from the read dispatch above.
		// Nesting the two generic dispatchers instantiates every element-type pair.
		for (const FPendingReferenceWrite& Pending : PendingWrites)
		{
			UMjNodeComponent* const Writable = EditableReferrer(Spec, EffectiveNode);
			if (Writable == nullptr)
			{
				OutError = FString::Printf(TEXT("could not create a Blueprint override for reference field '%s'"),
					*Pending.Field.ToString());
				bValid = false;
				continue;
			}
			urlab::spec::gen::DispatchByType(*Writable, [&](auto& WritableElement) {
				ForEachReference(WritableElement, [&](int, const char* WritableFieldName, auto&& WritableSlot,
					const std::vector<psm::ElementType>& WritableTargets) {
					if (ReferenceFieldKey(WritableFieldName) == Pending.Field &&
						TargetsContain(WritableTargets, Pending.TargetType))
					{
						Writable->Modify();
						WritableSlot.Set(FStringView(Pending.TargetName));
						Writable->ReferenceTargets.Add(Pending.Field, Pending.TargetKey);
						Writable->UnresolvedReferenceFields.Remove(Pending.Field);
					}
				});
			});
		}
	};

	if (OnlyNode != nullptr)
	{
		SyncNode(*OnlyNode);
	}
	else
	{
		ForEachElementOfSpec(Spec, SyncNode);
	}
	return bValid;
#else
	(void)Spec;
	(void)OnlyNode;
	OutError.Reset();
	return true;
#endif
}
#endif // WITH_EDITOR

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
#if WITH_EDITORONLY_DATA
				const FName FieldKey(UTF8_TO_TCHAR(FieldName));
				if (Node.UnresolvedReferenceFields.Contains(FieldKey))
				{
					const FString Message = FString::Printf(
						TEXT("%hs=\"%s\" has an unresolved component identity; select a target again"), FieldName, *FString(Slot.Get()));
					Node.DanglingReferences.Add(Message);
					Reported.Add(FString::Printf(TEXT("%s: %s"), *Node.MjName.Get(Node.GetName()), *Message));
					return;
				}
#endif
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
#endif // URLAB_MJ_GEN
} // namespace

void UMjNodeComponent::CheckPlacementLegality()
{
	urlab::spec::MjNodeCheckPlacementLegality(*this);
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
	urlab::spec::MjNodeCarryEditToInstances(*this, Doc, Event);
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
			RepointReferrers(Doc, *this, Type, OldName, NewName);
		}
		NameBeforeEdit.Reset();
		RefreshSpecReferenceDiagnostics(Doc);
	}
	else if (bIsElement && (IsReferenceField(Type, Changed) || IsReferenceField(Type, Member)))
	{
		const FName ReferenceField = IsReferenceField(Type, Member) ? Member : Changed;
		urlab::spec::MjRebindEditedReference(*this, Doc, ReferenceField);
		// Typing a name into a reference is the other way to make one dangle,
		// and the user finds out here rather than at play.
		RefreshSpecReferenceDiagnostics(Doc);
	}

	// The picture, under one index for the whole of it. Everything below asks the
	// same spec the same structural questions -- what class chain an element
	// resolves through, which node holds which template -- and indexing the spec
	// per question would repeat that walk for each one. It opens after the
	// branches above because those can rewrite names, and the index is keyed by
	// them.
	urlab::spec::FMjEffectiveScope Presentation(Doc);

	// Typing into a spatial attribute has to move the viewport. Which names those
	// are comes from the schema's field-id lookup, not from a list kept by hand
	// here.
	const FName HalfExtentsName = GET_MEMBER_NAME_CHECKED(UMjNodeComponent, MjSpatialHalfExtentsCm);
	if (bIsElement && (IsSpatialField(Type, Changed) || IsSpatialField(Type, Member)
		|| Changed == HalfExtentsName || Member == HalfExtentsName))
	{
		SyncPreviewFromSpec();
	}

	// The element that was edited is the whole answer for content. A shared node
	// is an input to other elements' pictures, and none of them re-reads on its
	// own.
	if (IsSharedPresentationInput())
	{
		// The edited spec is walked once, from the root already resolved above.
		urlab::spec::MjNodeRefreshSpecPresentationForSpec(Doc);

		// A Blueprint template's own spec is the template graph, which draws
		// nothing. What the user is looking at is the preview actor -- and every
		// placed actor of that class -- so the instances built from this template
		// have to re-read their own specs too.
		//
		// Each instance is a spec of its own, with its own root and its own class
		// chain, so it gets one walk and one index -- one, not one per element
		// and not one per question, avoiding the cost of resolving the instance's
		// root on every query.
		urlab::spec::MjNodeForEachInstanceOfTemplate(Doc, *this, [](UMjNodeComponent& Instance) {
			urlab::spec::MjNodeRefreshSpecPresentationForSpec(FSpecRef::OverOwner(&Instance));
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
#if URLAB_MJ_GEN
	if (IsSharedPresentationInput())
	{
		const FSpecRef Doc = FSpecRef::OverOwner(this);
		RefreshSpecPresentation();
		urlab::spec::MjNodeForEachInstanceOfTemplate(Doc, *this, [](UMjNodeComponent& Instance) {
			Instance.RefreshSpecPresentation();
		});
	}
	else
#endif
	{
		SyncPreviewUnderOneScope();
	}
}

#endif // WITH_EDITOR
