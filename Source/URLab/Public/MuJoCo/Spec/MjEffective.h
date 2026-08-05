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
 * One context, held open across a batch of queries.
 *
 * `TMjEffective` indexes every element and every default class in the spec to
 * answer a single query, which the class comment above says is built once per
 * spec and reused. Nothing enforced that: each `WithEffectiveDoc` built its own,
 * so a pass over N elements paid N whole-spec walks to answer N questions.
 *
 * Open one of these around such a pass and every `WithEffectiveDoc` inside it
 * joins the context instead of building one. Outside a scope the behaviour is
 * unchanged, so a lone query still costs exactly what it did.
 *
 * The scope may not span the creation or destruction of a spec node: the context
 * holds a parent map and a default index built from the tree as it was when the
 * scope opened. It is keyed on the root, so a nested scope over a different spec
 * correctly builds its own.
 */
class FMjEffectiveScope
{
public:
	explicit FMjEffectiveScope(const UMjNodeComponent& Node)
		: Root(Cast<UMjModel>(FSpecRef::OverOwner(&Node).GetRoot()))
		, Previous(Current)
	{
		if (Root != nullptr)
		{
			Current = this;
		}
	}

	~FMjEffectiveScope()
	{
		Current = Previous;
	}

	FMjEffectiveScope(const FMjEffectiveScope&) = delete;
	FMjEffectiveScope& operator=(const FMjEffectiveScope&) = delete;

	/** The open scope over `InRoot`, or null when there is none. */
	static FMjEffectiveScope* Find(const UMjModel* InRoot)
	{
		for (FMjEffectiveScope* Scope = Current; Scope != nullptr; Scope = Scope->Previous)
		{
			if (Scope->Root == InRoot)
			{
				return Scope;
			}
		}
		return nullptr;
	}

	template <class P>
	TMjEffective<P>& Get(const UMjModel& InRoot)
	{
		TUniquePtr<TMjEffective<P>>& Slot = Storage<P>();
		if (!Slot.IsValid())
		{
			Slot = MakeUnique<TMjEffective<P>>(InRoot);
		}
		return *Slot;
	}

	/**
	 * A by-name element lookup, scanned once per pass instead of once per query.
	 *
	 * Resolving one geom's picture asks for a mesh, a material and a texture per
	 * layer, and each of those walked every component in the spec. Over a refresh
	 * of N elements that is quadratic, and on a model with a few hundred geoms it
	 * is the largest single cost in the editor.
	 *
	 * `Kind` separates the namespaces MuJoCo itself separates -- a mesh and a
	 * material may share a name. `Enumerate` is called at most once per kind and
	 * yields every (name, element) pair for it.
	 */
	template <class EnumerateFn>
	UActorComponent* FindNamed(int32 Kind, const FString& Name, EnumerateFn&& Enumerate)
	{
		TMap<FString, TWeakObjectPtr<UActorComponent>>& Index = Named.FindOrAdd(Kind);
		if (Index.Num() == 0)
		{
			Enumerate([&Index](const FString& ElementName, UActorComponent* Element) {
				if (!ElementName.IsEmpty() && Element != nullptr)
				{
					Index.FindOrAdd(ElementName) = Element;
				}
			});
			// An empty spec would rescan every query. Cheap, and it keeps the
			// map honest about "not built yet" versus "nothing to find".
			Index.FindOrAdd(FString()) = nullptr;
		}
		const TWeakObjectPtr<UActorComponent>* Found = Index.Find(Name);
		return Found != nullptr ? Found->Get() : nullptr;
	}

private:
	template <class P>
	TUniquePtr<TMjEffective<P>>& Storage();

	const UMjModel* Root = nullptr;
	FMjEffectiveScope* Previous = nullptr;

#if WITH_EDITOR
	TUniquePtr<TMjEffective<FMjScsProfile>> Scs;
#endif
	TUniquePtr<TMjEffective<FMjInstanceProfile>> Instance;

	/** Per kind, the name index built on first use. */
	TMap<int32, TMap<FString, TWeakObjectPtr<UActorComponent>>> Named;

	static URLAB_API FMjEffectiveScope* Current;
};

#if WITH_EDITOR
template <>
inline TUniquePtr<TMjEffective<FMjScsProfile>>& FMjEffectiveScope::Storage<FMjScsProfile>()
{
	return Scs;
}
#endif

template <>
inline TUniquePtr<TMjEffective<FMjInstanceProfile>>& FMjEffectiveScope::Storage<FMjInstanceProfile>()
{
	return Instance;
}

/**
 * Run `Function` with a TMjEffective over the spec `Node` belongs to.
 *
 * `Function` is a generic lambda: it is called once, with the profile the node's
 * object graph decides. False when the spec root cannot be reached at all --
 * a detached node, or one whose tree has not been built yet -- in which case the
 * caller keeps whatever the authored-only read gave it.
 *
 * Joins an open `FMjEffectiveScope` when there is one, and builds its own when
 * there is not.
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

	if (FMjEffectiveScope* Scope = FMjEffectiveScope::Find(Root))
	{
#if WITH_EDITOR
		if (Doc.GetGraph() == EMjSpecGraph::Scs)
		{
			UBlueprint* Blueprint = Doc.GetBlueprint();
			if (Blueprint == nullptr)
			{
				return false;
			}
			// Only if one is not already open over this Blueprint. A nested
			// scope shadows the outer one and starts with an empty node map, so
			// opening one per query rebuilds that map per query -- which is the
			// cost this scope exists to remove.
			if (FMjScsScope* Open = FMjScsScope::Current();
				Open != nullptr && &Open->GetBlueprint() == Blueprint)
			{
				Function(Scope->Get<FMjScsProfile>(*Root));
				return true;
			}
			FMjScsScope ScsScope(*Blueprint);
			Function(Scope->Get<FMjScsProfile>(*Root));
			return true;
		}
#endif
		Function(Scope->Get<FMjInstanceProfile>(*Root));
		return true;
	}

#if WITH_EDITOR
	if (Doc.GetGraph() == EMjSpecGraph::Scs)
	{
		UBlueprint* Blueprint = Doc.GetBlueprint();
		if (Blueprint == nullptr)
		{
			return false;
		}
		if (FMjScsScope* Open = FMjScsScope::Current();
			Open != nullptr && &Open->GetBlueprint() == Blueprint)
		{
			TMjEffective<FMjScsProfile> Effective(*Root);
			Function(Effective);
			return true;
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
