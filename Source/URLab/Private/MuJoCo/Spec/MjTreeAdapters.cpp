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

#include "MuJoCo/Spec/MjSpecRef.h"

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
	Maps->ParentMap.Reset();
	Maps->bNodeMapValid = false;
}

void FMjScsScope::NoteNodeCreated(const UMjNodeComponent& Template, USCS_Node& Node)
{
	if (Maps->bNodeMapValid)
	{
		Maps->NodeMap.Add(&Template, &Node);
	}
}

void FMjScsScope::NoteChildAttached(USCS_Node& Parent, USCS_Node& Child)
{
	if (Maps->bNodeMapValid)
	{
		Maps->ParentMap.Add(&Child, &Parent);
	}
}

void FMjScsScope::NoteChildDetached(USCS_Node& Child)
{
	if (Maps->bNodeMapValid)
	{
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
	ParentMap.Reset();
	// One pass builds both. A USCS_Node records its children and not its parent,
	// so the parent of an element is otherwise a scan of every node in the
	// Blueprint -- and that question is asked once per ancestor test, of which
	// there are several per geom.
	for (USCS_Node* Node : Scs->GetAllNodes())
	{
		if (Node == nullptr)
		{
			continue;
		}
		if (UMjNodeComponent* Template = Cast<UMjNodeComponent>(Node->ComponentTemplate))
		{
			NodeMap.Add(Template, Node);
		}
		for (USCS_Node* Child : Node->GetChildNodes())
		{
			if (Child != nullptr)
			{
				ParentMap.Add(Child, Node);
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
	TArray<UMjNodeComponent*> Out;
	FMjScsScope* Scope = FMjScsScope::Current();
	if (Scope == nullptr)
	{
		return Out;
	}
	const USCS_Node* Node = Scope->FindNode(Parent);
	if (Node == nullptr)
	{
		return Out;
	}
	for (USCS_Node* Child : Node->GetChildNodes())
	{
		if (Child == nullptr)
		{
			continue;
		}
		if (UMjNodeComponent* Template = Cast<UMjNodeComponent>(Child->ComponentTemplate))
		{
			Out.Add(Template);
		}
	}
	return Out;
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
	return Cast<UMjNodeComponent>(ParentNode->ComponentTemplate);
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
		ParentNode->RemoveChildNode(ChildNode, /*bRemoveFromAllNodes=*/false);
		Scope->NoteChildDetached(*ChildNode);
	}
}

#endif // WITH_EDITOR

} // namespace urlab::spec

#endif // URLAB_MJ_GEN
