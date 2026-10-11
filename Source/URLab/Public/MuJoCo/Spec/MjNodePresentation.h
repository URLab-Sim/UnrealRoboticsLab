// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include "CoreMinimal.h"

class UMjNodeComponent;
struct FSpecRef;

namespace urlab::spec
{
/**
 * True when the node is a `<default>` class, or sits inside one.
 *
 * A class partial is an inheritance template, not scene content: MuJoCo never
 * places it, compiles it or draws it, so anything previewing it is previewing a
 * value that was never meant to stand on its own.
 */
URLAB_API bool MjNodeIsClassPartial(const UMjNodeComponent& Node);

/**
 * True when other elements resolve their own presentation through this node: a
 * `<default>` class it is under, or an `<asset>` resource it is part of.
 */
URLAB_API bool MjNodeIsSharedPresentationInput(const UMjNodeComponent& Node);

/** Re-derive everything about the node's own picture from the spec. */
URLAB_API void MjNodeRefreshPresentation(UMjNodeComponent& Node);

/** Re-derive the picture of every element of the spec the node belongs to. */
URLAB_API void MjNodeRefreshSpecPresentation(UMjNodeComponent& Node);

/**
 * The same, over an already-resolved spec, so a fan-out across several specs
 * reaches each root once rather than resolving it per query.
 */
URLAB_API void MjNodeRefreshSpecPresentationForSpec(const FSpecRef& Doc);
} // namespace urlab::spec
