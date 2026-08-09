// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// The state a spec-write walk carries from one node to the next.
//
// A component tree says what the document contains; it does not say what an
// element is created ON. That is the walk's answer, and it is different for
// every node: which body owns it, which frame encloses it, which default class
// it resolves through, and which already-existing struct an embedded element
// writes onto. Keeping it in one object is what lets the generated dispatch and
// the hand-written hooks be given the same thing.
//
// Private because nothing outside the build has any use for it: the handles in
// here point into a spec that is still being assembled.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Spec/MjSpecRef.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

class UMjNodeComponent;

namespace urlab::spec
{

struct FMjSpecWriteContext
{
	/** The component tree being read. Never mutated. */
	const FSpecRef* Source = nullptr;

	/** The spec being built. */
	mjSpec* Spec = nullptr;

	/** The body an element is created on; the world body at the top level. */
	mjsBody* Body = nullptr;

	/** The enclosing `<frame>`, or null when the node is not inside one. */
	mjsFrame* Frame = nullptr;

	/** The resolved default class, passed to every mjs_add* that takes one. */
	const mjsDefault* Class = nullptr;

	/** Its name, so a diagnostic can say which class failed to resolve. */
	FString ClassName = TEXT("main");

	/**
	 * The struct an embedded child writes onto.
	 *
	 * `<lengthrange>` writes onto its parent compiler's LRopt and a class
	 * partial onto the owning default's member struct, so an embedded element
	 * needs the struct its PARENT owns rather than one of its own.
	 */
	void* Parent = nullptr;

	/**
	 * The component that owns the current one.
	 *
	 * A few hooks need the siblings rather than the node: a skin's bones are
	 * one row each across parallel vectors with no per-bone handle to append
	 * to, so writing one means writing the run. Rebuilding from the parent's
	 * children keeps that stateless, where an accumulator would leak from one
	 * skin into the next.
	 */
	const UMjNodeComponent* ParentNode = nullptr;

	/**
	 * The default class this node is a partial OF, when it is inside one.
	 *
	 * A `<geom>` under a `<default>` is the same element as one under a body,
	 * and is not a geom: it is the class's geom template. So the walk carries
	 * whether it is inside a class, and a creating element inside one writes
	 * onto the class's member struct instead of adding to the spec.
	 */
	mjsDefault* Partial = nullptr;

	/** The current node's own struct, once created. Hooks reach it typed. */
	void* Struct = nullptr;

	/** The current node's element handle, once created. */
	mjsElement* Element = nullptr;

	/**
	 * Set by a hook that has already carried this node's children.
	 *
	 * A macro element's subtree is the template its expansion is generated
	 * from, not content of its own: the bridge serializes the whole subtree into
	 * the wrapper it parses, and the expansion is what lands in the spec.
	 * Walking those children as well would create the template a second time,
	 * as real elements beside the expansion.
	 */
	bool bChildrenConsumed = false;

	/** Per-component element identity, filled as the walk creates. */
	TMap<TObjectPtr<const UMjNodeComponent>, mjsElement*>* ElementFor = nullptr;

	/** Where diagnostics go. A failed node records one and skips its subtree. */
	TArray<FMjSpecDiagnostic>* Diagnostics = nullptr;

	/**
	 * Set once anything has recorded an error.
	 *
	 * A pointer rather than a flag because the context is COPIED down the walk:
	 * every node gets its own body, frame and class, and a hook that failed
	 * three levels down still has to stop the build. The one thing every copy
	 * shares is where the failure is recorded.
	 */
	bool* Failed = nullptr;

	/**
	 * Record a failure against a node, naming where it was authored.
	 *
	 * Always returns false, so a hook can `return Ctx.Error(...)` and read as
	 * what it is. Nothing here throws: a spec write runs over user-authored
	 * content inside an editor, and one bad element must not take the rest of
	 * the document with it.
	 */
	bool Error(const UMjNodeComponent& Node, const FString& Message);

	/** As above, for something recoverable. Returns true. */
	bool Warn(const UMjNodeComponent& Node, const FString& Message);
};

}  // namespace urlab::spec

#endif  // URLAB_MJ_GEN
