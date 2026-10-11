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

namespace urlab::spec
{
/**
 * The relative scale the node's effective size implies, if any.
 *
 * False for a node with no size, and for one whose size cannot be resolved --
 * too short for its type, or non-positive -- which leaves the component where it
 * is rather than collapsing it to nothing.
 */
URLAB_API bool MjNodeTryPreviewScaleFromSpec(const UMjNodeComponent& Node, FVector& OutScale);

/** True when a scale drag is an edit of the node's size. */
URLAB_API bool MjNodeHasScaleMapping(const UMjNodeComponent& Node);

/**
 * Snap the component's scale onto what the node can represent, and refuse a drag
 * the node's size cannot express by putting the component back.
 */
URLAB_API void MjNodeConstrainPreviewScale(UMjNodeComponent& Node);

/**
 * Author the node's size from a relative scale, under its own arity. False when
 * nothing was authored -- the size is not a scale, or the scale asks for a size
 * of zero or less.
 */
URLAB_API bool MjNodeWriteBackScale(UMjNodeComponent& Node, const FVector& Scale);
} // namespace urlab::spec
