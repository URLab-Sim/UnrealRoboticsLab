// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjTreeAdapters.h"

#if URLAB_MJ_GEN

#include "Algo/StableSort.h"
#include "Engine/Blueprint.h"

#include "MuJoCo/Spec/MjSpecRef.h"

namespace urlab::spec
{

TArray<FMjOrderedChild> MjOrderedChildrenOf(const FSpecRef& Spec, UMjNodeComponent& Parent)
{
#if WITH_EDITOR
	if (Spec.GetGraph() == EMjSpecGraph::Scs && Spec.GetBlueprint() != nullptr)
	{
		FMjScsScope Scope(*Spec.GetBlueprint());
		return FMjScsAdapter::OrderedChildren(Parent);
	}
#endif
	return FMjInstanceAdapter::OrderedChildren(Parent);
}

void MjSortSpecOrder(TArray<FMjOrderedChild>& Children)
{
	Algo::StableSort(Children, [](const FMjOrderedChild& A, const FMjOrderedChild& B) {
		if (A.Slot != B.Slot)
		{
			return A.Slot < B.Slot;
		}
		if (A.Node->SiblingIndex != B.Node->SiblingIndex)
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
}

FMjScsScope::FMjScsScope(UBlueprint& Blueprint)
	: Owner(&Blueprint)
	, Scs(Blueprint.SimpleConstructionScript)
	, Previous(GCurrentScsScope)
{
	checkf(Scs != nullptr, TEXT("FMjScsScope needs a Blueprint with a construction script"));
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
	NodeMap.Reset();
	ParentMap.Reset();
	bNodeMapValid = false;
}

void FMjScsScope::EnsureNodeMap() const
{
	if (bNodeMapValid)
	{
		return;
	}
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
	if (USCS_Node* const* Found = NodeMap.Find(&Child))
	{
		ChildNode = *Found;
	}
	if (ChildNode == nullptr)
	{
		return nullptr;
	}
	USCS_Node* const* Parent = ParentMap.Find(ChildNode);
	return Parent != nullptr ? *Parent : nullptr;
}

USCS_Node* FMjScsScope::FindNode(const UMjNodeComponent& Template) const
{
	EnsureNodeMap();
	USCS_Node* const* Found = NodeMap.Find(&Template);
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
		NodeMap.Add(&Template, Node);
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
		Scope->InvalidateNodeMap();
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
		Scope->InvalidateNodeMap();
	}
}

#endif  // WITH_EDITOR

}  // namespace urlab::spec

#endif  // URLAB_MJ_GEN
