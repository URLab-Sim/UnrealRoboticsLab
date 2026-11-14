// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include "CoreMinimal.h"
#include "mujoco/mujoco.h"

/**
 * Applies URLab's default scene options onto an mjSpec before compilation.
 */
URLAB_API void MjApplySceneOptionDefaults(mjSpec* Spec);

/**
 * Copies mjOption fields onto a compiled mjModel->opt.
 */
URLAB_API void MjApplyOptionToModel(const mjOption& Source, mjModel* Model);
