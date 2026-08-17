// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include "CoreMinimal.h"

#if WITH_EDITOR

class UMjNodeComponent;
struct FSpecRef;
struct FPropertyChangedEvent;

namespace urlab::spec
{
/**
 * Judge the node's parentage against the schema's child slots, filling
 * `PlacementProblems` and saying so once per element that becomes illegally
 * placed. Called from `OnRegister`, the one moment both ways of adding a
 * component reach.
 */
URLAB_API void MjNodeCheckPlacementLegality(UMjNodeComponent& Node);

/**
 * Carry a template's edited attribute onto the instances still following it, so
 * a construction-script rebuild does not read the unchanged instance value as an
 * override that pins the preview to the pre-edit state.
 */
URLAB_API void MjNodeCarryEditToInstances(
	UMjNodeComponent& Node, const FSpecRef& Doc, const FPropertyChangedEvent& Event);
} // namespace urlab::spec

#endif // WITH_EDITOR
