// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Reading an attribute the way the compiler will see it.
//
// An authored value and an effective value are different questions. The writer
// asks the first: it emits exactly what the element authored, which is what the
// round-trip fixpoint rests on. The editor preview asks the second: a geom whose
// `pos` comes from its default class must draw where MuJoCo will put it, not at
// the origin. Write-back stays authored-only, and that asymmetry is the point --
// previewing an inherited value must not author it.
//
// Resolution itself is ProtoSpec's (`ps::sdk::EffectiveContext`); what is here
// is the two things the SDK cannot supply: which of Unreal's two object graphs
// holds the spec, and a layer walk that hands back the storage rather than a
// merged copy, because a merged copy of a UObject element is an object
// allocation and the preview runs on every registration.
//
// The context holds pointers into the spec, so it is built per batch and
// discarded. Never cache one across an edit.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjSpecProfile.h"
#include "MuJoCo/Spec/MjSpecRef.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Spec/MjTreeAdapters.h"
#include "MuJoCo/Gen/Elements/MjModel.gen.h"

THIRD_PARTY_INCLUDES_START
#include "protospec/classes.h"
THIRD_PARTY_INCLUDES_END

namespace urlab::spec
{

/**
 * The layers that decide one element's attributes, in priority order.
 *
 * Built once per spec and reused for every query against it.
 */
template <class P>
class TMjEffective
{
public:
	/** The profile this context resolves against, for the generic-lambda caller. */
	using ProfileType = P;

	explicit TMjEffective(const typename P::Doc::doc_type& Root) : Context(Root) {}

	/**
	 * Offer `Function` each layer governing `Element` until one accepts.
	 *
	 * The element itself comes first, then its default-class chain from nearest
	 * to furthest. `Function` returns true to stop, which is how a caller says
	 * "this layer authored the field I asked for". Element families the schema
	 * declares no `<default>` partial for see only the element.
	 */
	template <class T, class Fn>
	void ForEachLayer(const T& Element, Fn&& Function) const
	{
		if (Function(Element))
		{
			return;
		}
		if constexpr (pssdk::has_default_family_v<P, T>)
		{
			for (const pssdk::detail::DefaultOf<P>* Class : Context.ChainFor(Element))
			{
				if (const T* Partial = pssdk::detail::FamilyPartial<P, T>(*Class))
				{
					if (Function(*Partial))
					{
						return;
					}
				}
			}
		}
	}

private:
	pssdk::EffectiveContext<P> Context;
};

/**
 * Run `Function` with a TMjEffective over the spec `Node` belongs to.
 *
 * `Function` is a generic lambda: it is called once, with the profile the node's
 * object graph decides. False when the spec root cannot be reached at all --
 * a detached node, or one whose tree has not been built yet -- in which case the
 * caller keeps whatever the authored-only read gave it.
 */
template <class Fn>
bool WithEffectiveDoc(const UMjNodeComponent& Node, Fn&& Function)
{
	const FSpecRef Doc = FSpecRef::OverOwner(&Node);
	const UMjModel* Root = Cast<UMjModel>(Doc.GetRoot());
	if (Root == nullptr)
	{
		return false;
	}

#if WITH_EDITOR
	if (Doc.GetGraph() == EMjSpecGraph::Scs)
	{
		UBlueprint* Blueprint = Doc.GetBlueprint();
		if (Blueprint == nullptr)
		{
			return false;
		}
		FMjScsScope Scope(*Blueprint);
		TMjEffective<FMjScsProfile> Effective(*Root);
		Function(Effective);
		return true;
	}
#endif

	TMjEffective<FMjInstanceProfile> Effective(*Root);
	Function(Effective);
	return true;
}

}  // namespace urlab::spec

#endif  // URLAB_MJ_GEN
