// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// The hand-written remainder of the spec write.
//
// A generated write exists exactly when an attribute lands in a field the shape
// rules can carry. Everything else is here, and each entry earns its place by
// an absence the emitter re-checks on every run: mjsEquality has no `anchor`,
// mjsActuator has no `kp`, a keyword set is a bitmask rather than a list. The
// generator fails rather than emit a write it cannot justify, so this file
// cannot quietly grow to cover what the generator should have.
//
// Registration is by the overlay's own handler name, and the generated
// `HooksFor` is what names them, so the two tables cannot drift: a handler the
// schema asks for and nobody registers fails the completeness check.
//
// Hooks report; they never throw. A spec write runs over content a user
// authored in an editor, and a document with one bad element still has to
// produce the rest.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN

#include "MjSpecBuildContext.h"

class UMjNodeComponent;

namespace urlab::spec
{

/**
 * One hook. Returns false after recording a diagnostic.
 *
 * `Created` is the element handle when the generated dispatch made one, and
 * null when creating it is the hook's own job. Its typed half is `Ctx.Struct`:
 * an mjsElement cannot be cast back to the struct that owns it, so both are
 * carried and a hook takes whichever it needs.
 */
using FMjSpecWriteHook = bool (*)(FMjSpecWriteContext& Ctx,
	const UMjNodeComponent& Node, mjsElement* Created);

/**
 * The two phases a handler name can implement.
 *
 * Two rather than one because the reader's ordering needs it: an actuator
 * shorthand is created, then has its shared attributes written by generated
 * code, then has its transmission elected, and only then does `mjs_setTo*` run
 * over the result. A single entry point could not sit on both sides of the
 * generated write.
 */
struct FMjSpecWriteHookRow
{
	const TCHAR* Name = nullptr;

	/** Create the element. Null when the generated dispatch already did. */
	FMjSpecWriteHook Create = nullptr;

	/** Write what the generated field application left alone. */
	FMjSpecWriteHook Apply = nullptr;
};

/** The row registered under `Name`, or null. */
const FMjSpecWriteHookRow* FindSpecWriteHook(const TCHAR* Name);

/**
 * Every hook the schema asks for is registered.
 *
 * The same discipline as ProtoSpec's own resolver registry: a handler named by
 * the overlay and missing here is a silently dropped write, so it is asserted
 * rather than discovered. Reached through the generated `HooksFor`, which is
 * the overlay table restated in C++.
 */
bool SpecWriteRegistryComplete(TArray<FString>* OutMissing = nullptr);

} // namespace urlab::spec

#endif // URLAB_MJ_GEN
