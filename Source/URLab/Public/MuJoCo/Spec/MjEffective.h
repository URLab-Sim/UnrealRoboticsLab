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
 * Whole-spec effective-value contexts built, process-wide, and the ticker the
 * constructor below rings.
 *
 * Building one indexes every element and every default class in the spec, so a
 * pass that builds one per query costs the square of the spec -- which is
 * exactly the shape `FMjEffectiveScope` exists to remove. Monotonic since
 * process start; the import benchmark takes a delta around a parse and asserts
 * the count does not grow with the model.
 */
URLAB_API int64 MjEffectiveContextBuilds();
URLAB_API void MjNoteEffectiveContextBuilt();

/** One layer that decides an element's attributes, and where it came from. */
struct FMjEffectiveLayer
{
	/**
	 * The storage to read the attribute out of.
	 *
	 * A node of the document -- the element itself or a `<default>` partial --
	 * except for the schema layer, which is a shared prototype owned by the
	 * generated profile and never part of any document.
	 */
	const UMjNodeComponent* Node = nullptr;

	/** The `<default>` class this layer is, or empty when it is not one. */
	FString ClassName;

	/** True for the schema layer: MuJoCo's own value, which no document authored. */
	bool bSchema = false;
};

/**
 * The layers deciding `Node`'s attributes, in the order the compiler resolves.
 *
 * The element itself, then its `<default>` class chain from nearest to
 * furthest, then the schema's own values -- `ps::sdk::EffectiveField`'s layer
 * order, walked once. A caller reads whichever attribute it is after out of the
 * first layer that has it.
 *
 * Exported because it is a whole-spec question with a per-element answer, and
 * the panel that asks it lives in the editor module while the layer order is a
 * fact about the schema. Joins an open `FMjEffectiveScope`, so a pass over many
 * elements batches into one index.
 *
 * The pointers are into the spec: valid until the next tree mutation.
 */
URLAB_API void MjEffectiveLayersOf(const UMjNodeComponent& Node, TArray<FMjEffectiveLayer>& Out);

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

	explicit TMjEffective(const typename P::Doc::doc_type& Root)
		: Context(Root)
	{
		MjNoteEffectiveContextBuilt();
	}

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

	/**
	 * The same walk, ending where the compiler's own merge ends.
	 *
	 * `ps::sdk::EffectiveField` (classes.h) resolves a field through element,
	 * class chain and then `P::Defaults<T>()`, and that last layer is the one a
	 * class chain never supplies: MuJoCo's own value for an attribute no
	 * `<default>` in the document mentions, which is most attributes of most
	 * elements. A caller that has to answer "what will the compiler use" rather
	 * than "what did some class say" wants this walk.
	 *
	 * It is the SAME walk with one more layer, not a second pass beside it, so
	 * the two cannot come to different answers about the class chain.
	 *
	 * `SchemaDefaultsOf` is how a caller tells that last layer apart: the layer
	 * it is handed is that exact object, so an identity test says "this came from
	 * the schema, not from a class the user can go and edit".
	 */
	template <class T, class Fn>
	void ForEachEffectiveLayer(const T& Element, Fn&& Function) const
	{
		bool bAccepted = false;
		ForEachLayer(Element, [&bAccepted, &Function](const auto& Layer) {
			bAccepted = Function(Layer);
			return bAccepted;
		});
		if (!bAccepted)
		{
			Function(SchemaDefaultsOf(Element));
		}
	}

	/** The schema's own values for `Element`'s family, as one shared layer. */
	template <class T>
	static const T& SchemaDefaultsOf(const T&)
	{
		return P::template Defaults<T>();
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
 * correctly builds its own -- and one over a spec an enclosing scope already
 * covers shares that scope's contexts rather than building a second set.
 *
 * A spec held as Blueprint templates needs the construction-script graph open to
 * be walked at all, so the scope opens that too. One scope is then the whole of
 * what a pass over a spec has to hold: the class index, the name index, and the
 * template graph.
 */
class FMjEffectiveScope
{
public:
	explicit FMjEffectiveScope(const UMjNodeComponent& Node)
		: FMjEffectiveScope(FSpecRef::OverOwner(&Node))
	{
	}

	/**
	 * The same, over an already-resolved spec.
	 *
	 * Resolving one means walking to the spec's root, which a caller that is
	 * about to walk the whole spec has already done. Taking the answer avoids
	 * doing it twice per pass.
	 */
	explicit FMjEffectiveScope(const FSpecRef& Doc)
		: Previous(Current)
	{
		Root = Cast<UMjModel>(Doc.GetRoot());
		if (Root == nullptr)
		{
			return;
		}
#if WITH_EDITOR
		if (Doc.GetGraph() == EMjSpecGraph::Scs)
		{
			if (UBlueprint* Blueprint = Doc.GetBlueprint())
			{
				ScsScope = MakeUnique<FMjScsScope>(*Blueprint);
			}
		}
#endif
		if (FMjEffectiveScope* Outer = Find(Root))
		{
			Shared = Outer->Shared;
		}
		Current = this;
	}

	~FMjEffectiveScope()
	{
		// Unlinked rather than popped. Scopes are ordinarily stack objects and
		// close in the order they opened, and for those the two are the same
		// thing; a scope held as a member for the length of a frame -- which the
		// editor's element drawing does, so that one index serves a whole sweep
		// -- is not, and popping there would put a dead pointer back on the
		// chain for the next lookup to walk.
		if (Current == this)
		{
			Current = Previous;
			return;
		}
		for (FMjEffectiveScope* Scope = Current; Scope != nullptr; Scope = Scope->Previous)
		{
			if (Scope->Previous == this)
			{
				Scope->Previous = Previous;
				return;
			}
		}
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
		TUniquePtr<TMjEffective<P>>& Slot = Shared->Storage<P>();
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
		TMap<FString, TWeakObjectPtr<UActorComponent>>& Index = Shared->Named.FindOrAdd(Kind);
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

	/**
	 * Whichever scope owns the indexes this one reads.
	 *
	 * `this` for the outermost scope over a spec, and the outermost one for every
	 * scope nested inside it. Nesting is ordinary -- a whole-spec pass opens one
	 * and the single-element work inside it opens its own -- and an inner scope
	 * that built its own indexes would pay the whole-spec walk the outer one
	 * exists to have paid once.
	 */
	FMjEffectiveScope* Shared = this;

#if WITH_EDITOR
	/** The template graph, when the spec is held as Blueprint templates. */
	TUniquePtr<FMjScsScope> ScsScope;

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

#if WITH_EDITOR
	if (Doc.GetGraph() == EMjSpecGraph::Scs)
	{
		UBlueprint* Blueprint = Doc.GetBlueprint();
		if (Blueprint == nullptr)
		{
			return false;
		}
		// A spec held as Blueprint templates cannot be walked without the
		// template graph open, and this opens one per query. That is affordable
		// only because a scope nested inside one over the same Blueprint adopts
		// its maps and builds none, so an enclosing pass still pays for one.
		FMjScsScope ScsScope(*Blueprint);
		if (FMjEffectiveScope* Scope = FMjEffectiveScope::Find(Root))
		{
			Function(Scope->Get<FMjScsProfile>(*Root));
			return true;
		}
		TMjEffective<FMjScsProfile> Effective(*Root);
		Function(Effective);
		return true;
	}
#endif

	if (FMjEffectiveScope* Scope = FMjEffectiveScope::Find(Root))
	{
		Function(Scope->Get<FMjInstanceProfile>(*Root));
		return true;
	}

	TMjEffective<FMjInstanceProfile> Effective(*Root);
	Function(Effective);
	return true;
}

} // namespace urlab::spec

#endif // URLAB_MJ_GEN
