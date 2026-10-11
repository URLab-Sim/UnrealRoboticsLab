// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include "CoreMinimal.h"

struct FMjGeomAppearance;

/**
 * Applies a visual domain-randomization override onto a geom's live material.
 *
 * The override is sparse -- only the fields it sets are written -- so it re-drives
 * an existing UMaterialInstanceDynamic (off the one M_MuJoCo_Master master) rather
 * than swapping the material: the base pass ran first, and this layers the variant's
 * scalar/vector terms and texture bindings over whatever that left. The parameter
 * names and the per-role texture slots are the master's contract, shared with the
 * base pass through MjMaterialRoleParameter so a randomized geom shades identically.
 */
namespace MjAppearance
{
	/**
	 * Write the set fields of `Override` onto `Mid`.
	 *
	 * `Mid` must be a UMaterialInstanceDynamic off M_MuJoCo_Master; a null MID is a
	 * no-op. Each binding in `Override.TextureBindings` is resolved to a UTexture by
	 * `ResolveTexture`, keyed by the bound FName; a key the resolver returns null for
	 * leaves that slot untouched, keeping the base pass's texture.
	 */
	URLAB_API void Apply(class UMaterialInstanceDynamic* Mid, const FMjGeomAppearance& Override,
		TFunctionRef<class UTexture*(FName Key)> ResolveTexture);
}
