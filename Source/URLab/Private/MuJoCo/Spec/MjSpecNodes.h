// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Every component of one spec, flattened, with the ones that must not be named.
//
// Two callers need the same walk for the same reason: they put a name on an
// element that authored none, and the set of elements they may not touch is
// decidable only from position rather than from the element's own type. Sharing
// the walk is what keeps the two from disagreeing about which those are.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN

struct FSpecRef;

class UMjNodeComponent;

namespace urlab::spec
{

/** A spec's components, and the ones that must not be given a name. */
struct FMjSpecNodes
{
	TArray<UMjNodeComponent*> Nodes;

	/**
	 * Elements the namer must leave alone.
	 *
	 * Two kinds, for two different reasons. `<worldbody>` is a `body` that sits
	 * directly under the root and MuJoCo rejects every attribute on it, name
	 * included. Everything under `<default>` is a class partial rather than an
	 * element, and the schema's reduced rows do not carry `name` at all. Both
	 * are decidable only from position, which is why this is collected during
	 * the walk rather than derived from the element's own type.
	 */
	TSet<const UMjNodeComponent*> Unnamable;
};

/** Flatten `Spec`, through whichever graph holds it. */
FMjSpecNodes MjSpecNodesOf(const FSpecRef& Spec);

} // namespace urlab::spec

#endif // URLAB_MJ_GEN
