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
 * The reference-name dropdown options a node offers for one of its ref fields,
 * or an empty list when the node is not an element or the field has no targets.
 *
 * Kept out of any generated element header: it resolves the field's target
 * element types through the dispatch tables, which include every element header
 * and so cannot themselves be included by one.
 */
URLAB_API TArray<FString> MjNodeRefNameOptions(const UMjNodeComponent& Node, int32 FieldId);
}
