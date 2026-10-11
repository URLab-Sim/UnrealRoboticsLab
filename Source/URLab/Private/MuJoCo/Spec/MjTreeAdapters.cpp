// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjTreeAdapters.h"

#if URLAB_MJ_GEN

#include <atomic>

#include "Algo/StableSort.h"
#include "Engine/Blueprint.h"
#include "Engine/BlueprintGeneratedClass.h"

#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Gen/MjDispatch.gen.h"
#include "MuJoCo/Gen/MjElements.gen.h"

namespace urlab::spec
{

namespace
{
/**
 * Give the unstamped children of one already-ordered sibling set a stamp.
 *
 * Only the unstamped ones are written. Restating the whole group densely would
 * express the same order, and would also dirty the package of every element the
 * user did not touch; the stamped siblings are the imported model, and it is not
 * this walk's business to rewrite them.
 *
 * The value handed out is one past the highest stamp seen so far in the slot, so
 * an element that sorted last keeps sorting last once it is stamped.
 */
void StampUnorderedChildren(const TArray<FMjOrderedChild>& Children)
{
	int32 Slot = MIN_int32;
	int32 Next = 0;
	for (const FMjOrderedChild& Child : Children)
	{
		if (Child.Node == nullptr)
		{
			continue;
		}
		if (Child.Slot != Slot)
		{
			Slot = Child.Slot;
			Next = 0;
		}
		if (Child.Node->SiblingIndex == INDEX_NONE)
		{
			Child.Node->Modify();
			Child.Node->SiblingIndex = Next++;
		}
		else
		{
			Next = FMath::Max(Next, Child.Node->SiblingIndex + 1);
		}
	}
}
} // namespace

TArray<FMjOrderedChild> MjOrderedChildrenOf(const FSpecRef& Spec, UMjNodeComponent& Parent)
{
#if WITH_EDITOR
	if (Spec.GetGraph() == EMjSpecGraph::Scs && Spec.GetBlueprint() != nullptr)
	{
		FMjScsScope Scope(*Spec.GetBlueprint());
		TArray<FMjOrderedChild> Children = FMjScsAdapter::OrderedChildren(Parent);
		StampUnorderedChildren(Children);
		return Children;
	}
#endif
	TArray<FMjOrderedChild> Children = FMjInstanceAdapter::OrderedChildren(Parent);
	StampUnorderedChildren(Children);
	return Children;
}

namespace
{
/**
 * Whether a node is one of the actuator shorthands.
 *
 * Every actuator element type a `<default>` admits folds onto the class's single
 * `mjsActuator` template, so they are all leaves of `UMjActuator`. Derived from
 * that child-slot set rather than a hand list, so a new actuator type is covered
 * the moment it is generated.
 */
bool IsActuatorShorthand(const UMjNodeComponent* Node)
{
	if (Node == nullptr)
	{
		return false;
	}
	static const TSet<psm::ElementType> Kinds = [] {
		TSet<psm::ElementType> Out;
		gen::ChildSlots(static_cast<const UMjActuator*>(nullptr),
			[&Out](int32, auto Tag) {
				using Leaf = typename decltype(Tag)::type;
				Out.Add(gen::TMjElementType<Leaf>::Value);
			});
		return Out;
	}();
	psm::ElementType Type;
	return gen::ElementTypeOfNode(*Node, Type) && Kinds.Contains(Type);
}
} // namespace

void MjSortSpecOrder(TArray<FMjOrderedChild>& Children)
{
	Algo::StableSort(Children, [](const FMjOrderedChild& A, const FMjOrderedChild& B) {
		// A child with no node cannot be ordered against one that has one, so it
		// goes last rather than being dereferenced. Nulls order equal to each
		// other, which keeps this a strict weak ordering.
		if (A.Node == nullptr || B.Node == nullptr)
		{
			return A.Node != nullptr && B.Node == nullptr;
		}
		if (A.Slot != B.Slot)
		{
			// Actuator shorthands are the one family whose storage slot lies: a
			// class folds motor/velocity/position onto a single actuator template
			// where the document-last one wins, so their per-type slots must not
			// decide the order. Sort them by source instead, which is the order
			// MuJoCo's own reader applies them to that template.
			if (IsActuatorShorthand(A.Node) && IsActuatorShorthand(B.Node)
				&& A.Node->SourceLine > 0 && B.Node->SourceLine > 0
				&& A.Node->SourceLine != B.Node->SourceLine)
			{
				return A.Node->SourceLine < B.Node->SourceLine;
			}
			return A.Slot < B.Slot;
		}
		// An unstamped element is one the user just added, and the only honest
		// place for it is the end of its group: MJCF declaration order is the
		// qpos layout, and a component whose SiblingIndex is still the class
		// default has expressed no position at all.
		const bool bAStamped = A.Node->SiblingIndex != INDEX_NONE;
		const bool bBStamped = B.Node->SiblingIndex != INDEX_NONE;
		if (bAStamped != bBStamped)
		{
			return bAStamped;
		}
		if (bAStamped && A.Node->SiblingIndex != B.Node->SiblingIndex)
		{
			return A.Node->SiblingIndex < B.Node->SiblingIndex;
		}
		return A.Node->Serial < B.Node->Serial;
	});
}

// --- Live attachment hierarchy --------------------------------------------- //

TArray<UMjNodeComponent*> FMjInstanceAdapter::RawChildren(const UMjNodeComponent& Parent)
{
	TArray<UMjNodeComponent*> Out;
	for (USceneComponent* Child : Parent.GetAttachChildren())
	{
		if (UMjNodeComponent* Node = Cast<UMjNodeComponent>(Child))
		{
			Out.Add(Node);
		}
	}
	return Out;
}

UMjNodeComponent* FMjInstanceAdapter::ParentOf(const UMjNodeComponent& Child)
{
	return Cast<UMjNodeComponent>(Child.GetAttachParent());
}

void FMjInstanceAdapter::Attach(UMjNodeComponent& Parent, UMjNodeComponent& Child)
{
	// KeepRelative: the spec's pose is authored in the element's own frame,
	// which is exactly the relative transform. Reparenting is a pure tree
	// operation and must not move the element.
	Child.AttachToComponent(&Parent, FAttachmentTransformRules::KeepRelativeTransform);
}

void FMjInstanceAdapter::Detach(UMjNodeComponent& Child)
{
	Child.DetachFromComponent(FDetachmentTransformRules::KeepRelativeTransform);
}

// --- Blueprint construction-script graph ----------------------------------- //

#if WITH_EDITOR

namespace
{
thread_local FMjScsScope* GCurrentScsScope = nullptr;
std::atomic<int64> GNodeMapBuilds{0};
} // namespace

FMjScsScope::FMjScsScope(UBlueprint& Blueprint)
	: Owner(&Blueprint)
	, Scs(Blueprint.SimpleConstructionScript)
	, Previous(GCurrentScsScope)
{
	checkf(Scs != nullptr, TEXT("FMjScsScope needs a Blueprint with a construction script"));

	// An enclosing scope over the same Blueprint has already answered every
	// question this one can be asked, so its maps are adopted rather than a
	// second set built. Without this a nested scope starts empty and rebuilds the
	// whole-Blueprint map on its first lookup, which is what made a per-element
	// query cost the size of the model: the ancestor test opens a scope per call.
	for (FMjScsScope* Outer = Previous; Outer != nullptr; Outer = Outer->Previous)
	{
		if (Outer->Owner == Owner)
		{
			Maps = Outer->Maps;
			break;
		}
	}
	GCurrentScsScope = this;
}

FMjScsScope::~FMjScsScope()
{
	GCurrentScsScope = Previous;
}

FMjScsScope* FMjScsScope::Current()
{
	return GCurrentScsScope;
}

void FMjScsScope::InvalidateNodeMap()
{
	Maps->NodeMap.Reset();
	Maps->TemplateMap.Reset();
	Maps->ParentMap.Reset();
	Maps->ChildMap.Reset();
	Maps->bNodeMapValid = false;
}

void FMjScsScope::NoteNodeCreated(const UMjNodeComponent& Template, USCS_Node& Node)
{
	if (Maps->bNodeMapValid)
	{
		Maps->NodeMap.Add(&Template, &Node);
		Maps->TemplateMap.Add(&Node, const_cast<UMjNodeComponent*>(&Template));
	}
}

void FMjScsScope::NoteChildAttached(USCS_Node& Parent, USCS_Node& Child)
{
	if (Maps->bNodeMapValid)
	{
		Maps->ParentMap.Add(&Child, &Parent);
		Maps->ChildMap.FindOrAdd(&Parent).AddUnique(&Child);
	}
}

void FMjScsScope::NoteChildDetached(USCS_Node& Child)
{
	if (Maps->bNodeMapValid)
	{
		if (USCS_Node** Parent = Maps->ParentMap.Find(&Child))
		{
			Maps->ChildMap.FindOrAdd(*Parent).Remove(&Child);
		}
		Maps->ParentMap.Remove(&Child);
	}
}

int64 FMjScsScope::NodeMapBuilds()
{
	return GNodeMapBuilds.load(std::memory_order_relaxed);
}

void FMjScsScope::EnsureNodeMap() const
{
	if (Maps != this)
	{
		Maps->EnsureNodeMap();
		return;
	}
	if (bNodeMapValid)
	{
		return;
	}
	GNodeMapBuilds.fetch_add(1, std::memory_order_relaxed);
	NodeMap.Reset();
	TemplateMap.Reset();
	ParentMap.Reset();
	ChildMap.Reset();
	// One pass builds both. A USCS_Node records its children and not its parent,
	// so the parent of an element is otherwise a scan of every node in the
	// Blueprint -- and that question is asked once per ancestor test, of which
	// there are several per geom.
	TArray<UBlueprintGeneratedClass*> Classes;
	for (UClass* Class = Owner->GeneratedClass; Class != nullptr; Class = Class->GetSuperClass())
	{
		if (UBlueprintGeneratedClass* Generated = Cast<UBlueprintGeneratedClass>(Class))
		{
			Classes.Insert(Generated, 0);
		}
	}
	for (UBlueprintGeneratedClass* Class : Classes)
	{
		USimpleConstructionScript* ClassScs = Class->SimpleConstructionScript;
		if (ClassScs == nullptr)
		{
			continue;
		}
		for (USCS_Node* Node : ClassScs->GetAllNodes())
		{
			if (Node == nullptr) continue;
			if (UMjNodeComponent* Template = Cast<UMjNodeComponent>(Node->GetActualComponentTemplate(
				Cast<UBlueprintGeneratedClass>(Owner->GeneratedClass))))
			{
				NodeMap.Add(Template, Node);
				TemplateMap.Add(Node, Template);
			}
			for (USCS_Node* Child : Node->GetChildNodes())
			{
				if (Child != nullptr)
				{
					ParentMap.Add(Child, Node);
					ChildMap.FindOrAdd(Node).AddUnique(Child);
				}
			}
		}
	}
	// A child Blueprint stores additions to an inherited component as local
	// SCS roots whose ParentComponentOrVariableName points at the ancestor.
	for (UBlueprintGeneratedClass* Class : Classes)
	{
		if (Class->SimpleConstructionScript == nullptr) continue;
		for (USCS_Node* Node : Class->SimpleConstructionScript->GetRootNodes())
		{
			if (Node == nullptr || Node->ParentComponentOrVariableName.IsNone() ||
				Node->bIsParentComponentNative) continue;
			for (const auto& Pair : TemplateMap)
			{
				USCS_Node* Parent = const_cast<USCS_Node*>(Pair.Key);
				if (Parent->GetVariableName() == Node->ParentComponentOrVariableName &&
					Parent->GetSCS()->GetOwnerClass()->GetFName() == Node->ParentComponentOwnerClassName)
				{
					ParentMap.Add(Node, Parent);
					ChildMap.FindOrAdd(Parent).AddUnique(Node);
					break;
				}
			}
		}
	}
	bNodeMapValid = true;
}

USCS_Node* FMjScsScope::ParentNodeOf(const UMjNodeComponent& Child) const
{
	EnsureNodeMap();
	const USCS_Node* ChildNode = nullptr;
	if (USCS_Node* const* Found = Maps->NodeMap.Find(&Child))
	{
		ChildNode = *Found;
	}
	if (ChildNode == nullptr)
	{
		return nullptr;
	}
	USCS_Node* const* Parent = Maps->ParentMap.Find(ChildNode);
	return Parent != nullptr ? *Parent : nullptr;
}

USCS_Node* FMjScsScope::FindNode(const UMjNodeComponent& Template) const
{
	EnsureNodeMap();
	USCS_Node* const* Found = Maps->NodeMap.Find(&Template);
	return Found != nullptr ? *Found : nullptr;
}

UMjNodeComponent* FMjScsScope::TemplateFor(const USCS_Node& Node) const
{
	EnsureNodeMap();
	UMjNodeComponent* const* Found = Maps->TemplateMap.Find(&Node);
	return Found != nullptr ? *Found : nullptr;
}

TArray<UMjNodeComponent*> FMjScsScope::ChildrenOf(const UMjNodeComponent& Parent) const
{
	TArray<UMjNodeComponent*> Out;
	USCS_Node* ParentNode = FindNode(Parent);
	if (ParentNode == nullptr) return Out;
	if (const TArray<USCS_Node*>* Children = Maps->ChildMap.Find(ParentNode))
	{
		for (USCS_Node* Child : *Children)
		{
			if (UMjNodeComponent* Template = TemplateFor(*Child)) Out.Add(Template);
		}
	}
	return Out;
}

USCS_Node* FMjScsScope::NodeFor(UMjNodeComponent& Template)
{
	if (USCS_Node* Existing = FindNode(Template))
	{
		return Existing;
	}
	// The SDK constructs elements through Ident::New, which cannot know about a
	// template graph. Adopting the loose component here is what makes the
	// construct-and-link contract total on this profile rather than only on the
	// path the reader's factory takes.
	USCS_Node* Node = Scs->CreateNodeAndRenameComponent(&Template);
	if (Node != nullptr)
	{
		NoteNodeCreated(Template, *Node);
	}
	return Node;
}

TArray<UMjNodeComponent*> FMjScsAdapter::RawChildren(const UMjNodeComponent& Parent)
{
	FMjScsScope* Scope = FMjScsScope::Current();
	return Scope != nullptr ? Scope->ChildrenOf(Parent) : TArray<UMjNodeComponent*>();
}

UMjNodeComponent* FMjScsAdapter::ParentOf(const UMjNodeComponent& Child)
{
	FMjScsScope* Scope = FMjScsScope::Current();
	if (Scope == nullptr)
	{
		return nullptr;
	}
	USCS_Node* ParentNode = Scope->ParentNodeOf(Child);
	if (ParentNode == nullptr)
	{
		return nullptr;
	}
	// Null rather than a climb when the parent is not an element: a spec root
	// under a plain DefaultSceneRoot has no element parent, and answering with
	// its grandparent would invent an ancestry the spec does not have.
	return Scope->TemplateFor(*ParentNode);
}

void FMjScsAdapter::Attach(UMjNodeComponent& Parent, UMjNodeComponent& Child)
{
	FMjScsScope* Scope = FMjScsScope::Current();
	if (Scope == nullptr)
	{
		return;
	}
	USCS_Node* ParentNode = Scope->NodeFor(Parent);
	USCS_Node* ChildNode = Scope->NodeFor(Child);
	if (ParentNode == nullptr || ChildNode == nullptr || ParentNode == ChildNode)
	{
		return;
	}
	// An inherited child belongs to its declaring Blueprint. It cannot be
	// reparented by editing an SCS in a derived Blueprint.
	if (ChildNode->GetSCS() != &Scope->GetScs())
	{
		return;
	}
	if (ParentNode->GetSCS() != &Scope->GetScs())
	{
		// UE represents a child of an inherited component as a local SCS root
		// with an external parent key. Never edit the ancestor's ChildNodes.
		if (!Scope->GetScs().GetRootNodes().Contains(ChildNode))
		{
			Scope->GetScs().AddNode(ChildNode);
		}
		ChildNode->SetParent(ParentNode);
		Scope->NoteChildAttached(*ParentNode, *ChildNode);
		return;
	}
	if (!ParentNode->GetChildNodes().Contains(ChildNode))
	{
		ParentNode->AddChildNode(ChildNode);
		Scope->NoteChildAttached(*ParentNode, *ChildNode);
	}
}

void FMjScsAdapter::Detach(UMjNodeComponent& Child)
{
	FMjScsScope* Scope = FMjScsScope::Current();
	if (Scope == nullptr)
	{
		return;
	}
	USCS_Node* ChildNode = Scope->FindNode(Child);
	if (ChildNode == nullptr)
	{
		return;
	}
	if (USCS_Node* ParentNode = Scope->ParentNodeOf(Child))
	{
		if (ChildNode->GetSCS() != &Scope->GetScs()) return;
		if (ParentNode->GetSCS() != &Scope->GetScs())
		{
			// It is already a root of this Blueprint's SCS. RemoveNode would
			// remove it from AllNodes as well, losing the local authored node.
			// Clear only the external-parent key and leave it as a local root.
			ChildNode->Modify();
			ChildNode->bIsParentComponentNative = false;
			ChildNode->ParentComponentOrVariableName = NAME_None;
			ChildNode->ParentComponentOwnerClassName = NAME_None;
			Scope->NoteChildDetached(*ChildNode);
			return;
		}
		ParentNode->RemoveChildNode(ChildNode, /*bRemoveFromAllNodes=*/false);
		Scope->NoteChildDetached(*ChildNode);
	}
}

#endif // WITH_EDITOR

} // namespace urlab::spec

#endif // URLAB_MJ_GEN
