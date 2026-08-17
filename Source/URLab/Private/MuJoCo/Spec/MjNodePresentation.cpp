// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjNodePresentation.h"

#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjEffective.h"
#include "MuJoCo/Spec/MjElementIdentity.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"
#include "Engine/Blueprint.h"

#if URLAB_MJ_GEN
#include <vector>
THIRD_PARTY_INCLUDES_START
#include "protospec/core.h"
#include "protospec/detail.h"
#include "protospec/model_core.h"
#include "protospec/profile.h"
THIRD_PARTY_INCLUDES_END
#include "MuJoCo/Gen/MjDispatch.gen.h"
#endif

#if URLAB_MJ_GEN

namespace
{
namespace psm = ps::mjcf;

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

namespace urlab::spec
{
bool MjNodeIsClassPartial(const UMjNodeComponent& Node)
{
	return AnyAncestorInSpec(
		Node, [](const UMjNodeComponent& Node) { return IsElementOfType(Node, psm::ElementType::Default); });
}

bool MjNodeIsSharedPresentationInput(const UMjNodeComponent& Node)
{
	return AnyAncestorInSpec(Node, [](const UMjNodeComponent& Node) {
		return IsElementOfType(Node, psm::ElementType::Default) || IsElementOfType(Node, psm::ElementType::Asset);
	});
}

void MjNodeRefreshPresentation(UMjNodeComponent& Node)
{
	Node.SyncPreviewFromSpec();
}

void MjNodeRefreshSpecPresentationForSpec(const FSpecRef& Doc)
{
	RefreshSpecPresentationOf(Doc);
}

void MjNodeRefreshSpecPresentation(UMjNodeComponent& Node)
{
	MjNodeRefreshSpecPresentationForSpec(FSpecRef::OverOwner(&Node));
}
} // namespace urlab::spec

#endif // URLAB_MJ_GEN
