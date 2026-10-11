// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Normalize imported fromto into canonical pose and UE half-extents once.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN

class UMjNodeComponent;

namespace urlab::spec
{

/** Capture effective source values, normalize templates and elements, then
 * discard fromto. Returns false if a consumed source is invalid.
 */
template <class Adapter>
bool NormalizeImportedFromtoTree(UMjNodeComponent& Root);

/** Return a complete native size from the effective normalized UE half-extents.
 * An empty OutSize means an untyped default template should omit raw size;
 * concrete consumers get their own typed size on export. A later explicit local
 * size takes precedence and returns false.
 */
bool MjSpatialSizeOverride(const UMjNodeComponent& Node, bool bInDefault, TArray<double>& OutSize);

} // namespace urlab::spec

#endif // URLAB_MJ_GEN
