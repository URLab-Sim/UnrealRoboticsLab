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
enum class EMjPreviewProblem : uint8;

namespace urlab::spec
{
/**
 * Record `Message` as the node's row for `Problem`, replacing any earlier one,
 * and say it once in the message log and the run log.
 *
 * Keyed on the node's `Serial` rather than on the object, so a Blueprint
 * reconstruct does not repeat the logs and a genuinely new element is a new
 * mistake. Operates on the preview-problem arrays the node still owns.
 */
URLAB_API void MjNodeNotePreviewProblem(UMjNodeComponent& Node, EMjPreviewProblem Problem, const FString& Message);

/** Drop the node's row for `Problem`, and let it be said again if it returns. */
URLAB_API void MjNodeClearPreviewProblem(UMjNodeComponent& Node, EMjPreviewProblem Problem);
} // namespace urlab::spec
