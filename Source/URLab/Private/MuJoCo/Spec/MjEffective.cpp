// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjEffective.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Gen/Elements/Defaults/MjDefault.gen.h"
#include "MuJoCo/Gen/MjDispatch.gen.h"
#include "MuJoCo/Spec/MjNodeComponent.h"

#include <atomic>
#include <type_traits>

namespace urlab::spec
{
namespace
{
std::atomic<int64> GContextBuilds{0};

/**
 * The class a `<default>` partial belongs to, as MuJoCo names it.
 *
 * Empty when the layer is not a partial at all, which is how the element's own
 * storage and the schema prototype are told from a class.
 */
template <class P>
FString ClassNameOf(const UMjNodeComponent& Layer)
{
	UMjNodeComponent* const Parent = P::Tree::ParentOf(Layer);
	if (Parent == nullptr || Cast<UMjDefault>(Parent) == nullptr)
	{
		return FString();
	}
	// MuJoCo's own name for the root `<default>`, which authors no class name
	// because everything inherits from it.
	return Parent->MjName.Get(TEXT("main"));
}
}  // namespace

// Editor and game thread both re-derive presentation, never at the same time and
// never across a suspension point, so the open scope is a plain pointer rather
// than thread-local storage. A scope that outlived its stack would be the bug
// either way.
FMjEffectiveScope* FMjEffectiveScope::Current = nullptr;

int64 MjEffectiveContextBuilds()
{
	return GContextBuilds.load(std::memory_order_relaxed);
}

void MjNoteEffectiveContextBuilt()
{
	GContextBuilds.fetch_add(1, std::memory_order_relaxed);
}

void MjEffectiveLayersOf(const UMjNodeComponent& Node, TArray<FMjEffectiveLayer>& Out)
{
	Out.Reset();
	if (WithEffectiveDoc(Node, [&Out, &Node](auto& Effective) {
		using P = typename std::decay_t<decltype(Effective)>::ProfileType;
		gen::DispatchByType(const_cast<UMjNodeComponent&>(Node), [&Out, &Effective](auto& Element) {
			// The schema layer is a shared prototype rather than a node of the
			// document: it is recognised by being that object, and never asked
			// which class it belongs to, because it belongs to none.
			const UMjNodeComponent* const Schema =
				&static_cast<const UMjNodeComponent&>(Effective.SchemaDefaultsOf(Element));

			Effective.ForEachEffectiveLayer(Element, [&Out, Schema](const auto& Layer) {
				const UMjNodeComponent& LayerNode = static_cast<const UMjNodeComponent&>(Layer);
				FMjEffectiveLayer& Row = Out.AddDefaulted_GetRef();
				Row.Node = &LayerNode;
				Row.bSchema = &LayerNode == Schema;
				Row.ClassName = Row.bSchema ? FString() : ClassNameOf<P>(LayerNode);
				// Never stops: the caller is collecting the layers, not asking
				// one of them a question.
				return false;
			});
		});
	}))
	{
		return;
	}

	// No `<mujoco>` above this element: a component dropped onto an ordinary
	// actor, or one whose tree has not been built yet. The default-class chain
	// genuinely cannot be resolved without a document -- there is no document to
	// declare a class in -- but the SCHEMA layer never needed one. MuJoCo's own
	// value for `condim` is 3 whatever file it is read from, and dropping that
	// layer with the rest left every attribute of such an element reading "(no
	// default)": a panel that answered "nothing decides this" about attributes
	// MuJoCo decides.
	gen::DispatchByType(const_cast<UMjNodeComponent&>(Node), [&Out](auto& Element) {
		using E = std::decay_t<decltype(Element)>;

		FMjEffectiveLayer& Own = Out.AddDefaulted_GetRef();
		Own.Node = &static_cast<const UMjNodeComponent&>(Element);

		FMjEffectiveLayer& Schema = Out.AddDefaulted_GetRef();
		Schema.Node = &static_cast<const UMjNodeComponent&>(FMjInstanceProfile::Defaults<E>());
		Schema.bSchema = true;
	});
}

}  // namespace urlab::spec

#endif  // URLAB_MJ_GEN
