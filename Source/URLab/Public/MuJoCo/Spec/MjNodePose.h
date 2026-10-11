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
/** True when the node's schema declares `pos` or an orientation attribute. */
URLAB_API bool MjNodeHasPoseAttributes(const UMjNodeComponent& Node);

/**
 * The relative transform the spec says the node sits at, without applying it.
 *
 * The effective pose (authored, else resolved through the default-class chain)
 * and the effective size, so the write-back can recover a cold baseline without
 * moving the component it is about to read. False when the node has no pose.
 */
URLAB_API bool MjNodeComputePreviewTransform(UMjNodeComponent& Node, FTransform& Out);

/**
 * Drive the node's relative transform from its effective pose and size.
 *
 * Uses `SetRelativeLocationAndRotation`, which fires no editor hooks, so this
 * can never re-enter the write-back; it also establishes the write-back's
 * `LastPreviewTransform` baseline.
 */
URLAB_API void MjNodeSyncPreviewFromSpec(UMjNodeComponent& Node);

/**
 * Author the node's `pos`, orientation and size from its relative transform,
 * but only the parts that differ from what the preview last applied.
 *
 * The change guard against `LastPreviewTransform` is load-bearing: a body drag
 * delivers the move hook to descendants that did not move, and an unconditional
 * write would author pose presence on every one of them.
 */
URLAB_API void MjNodeWriteBackTransformIfChanged(UMjNodeComponent& Node);
} // namespace urlab::spec
