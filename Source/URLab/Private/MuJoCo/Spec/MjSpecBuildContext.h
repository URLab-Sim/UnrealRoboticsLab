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

/**
 * The child specs a document's `<model>` assets parsed into.
 *
 * MuJoCo's reader hands each one to the spec with `mjs_addSpec` and finds it
 * again by the child's own model name; this keeps them beside the walk instead,
 * because `mjs_attach` takes the child's element directly and registration buys
 * nothing a map does not. Lookup is first-registered-wins, which is what
 * `mjCModel::FindSpec` does with a repeated name.
 *
 * Ownership is the point. A child nothing attaches is still released, and one
 * that is attached survives this release on the target's own reference count:
 * attach appends the child spec to the target and takes a reference of its own
 * (`user_objects.cc:1685`), so the walk's reference is the only one left over.
 */
class FMjModelAssets
{
public:
	FMjModelAssets() = default;
	FMjModelAssets(const FMjModelAssets&) = delete;
	FMjModelAssets& operator=(const FMjModelAssets&) = delete;
	~FMjModelAssets();

	/** Take ownership of `Child`, findable under `Name` if nothing else is. */
	void Add(const FString& Name, mjSpec* Child);

	/** The spec registered under `Name`, or null. */
	mjSpec* Find(const FString& Name) const;

private:
	TMap<FString, mjSpec*> ByName;
	TArray<mjSpec*> Owned;
};

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
	 * The component the walk is on.
	 *
	 * Carried rather than recovered: a diagnostic wants to say which element it
	 * is about, and the alternative -- looking the element up by spec id once
	 * the failure is known -- asks for an id that a failed write never made. It
	 * is set before the hooks run, so every hook has it whether or not the
	 * thing it is reporting is the node it was handed: a tuple entry, a skin
	 * bone and a macro's asset are all reported against themselves, and this
	 * says what they were being written under.
	 */
	const UMjNodeComponent* Walked = nullptr;

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
	 * Set once the spec can no longer be built on at all.
	 *
	 * A failed `mjs_attach` is not a failed element: the duplicate is already
	 * inserted and the counts have already moved, so every later `mj_compile`
	 * fails with the same error and there is no unwind. The walk stops outright
	 * rather than recording more diagnostics against a spec that can only
	 * produce more of the same.
	 */
	bool* Aborted = nullptr;

	/** The `<model>` assets registered so far, for `<attach model=>` to find. */
	FMjModelAssets* Models = nullptr;

	/**
	 * Record a failure against a node, naming it and where it was authored.
	 *
	 * The source file and line are the reader's answer and are empty for
	 * anything authored in the editor, which is where most of this content now
	 * comes from; so the diagnostic leads with the component's own name and the
	 * MJCF name it compiles under, which are what a user can select and what a
	 * bridge client sees.
	 *
	 * Always returns false, so a hook can `return Ctx.Error(...)` and read as
	 * what it is. Nothing here throws: a spec write runs over user-authored
	 * content inside an editor, and one bad element must not take the rest of
	 * the document with it.
	 */
	bool Error(const UMjNodeComponent& Node, const FString& Message);

	/** As above, and the walk stops here. Returns false. */
	bool Abort(const UMjNodeComponent& Node, const FString& Message);

	/** As above, for something recoverable. Returns true. */
	bool Warn(const UMjNodeComponent& Node, const FString& Message);
};

}  // namespace urlab::spec

#endif  // URLAB_MJ_GEN
