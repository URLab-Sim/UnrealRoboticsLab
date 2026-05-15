// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.

#pragma once

#include "CoreMinimal.h"
#include "MjCameraTypes.generated.h"

/**
 * @enum EMjCameraMode
 * @brief What a UMjCamera captures. Read at SetStreamingEnabled(true) time —
 *        to change mode on a running camera, toggle streaming off then on.
 *
 * Lives in its own header so UMjDebugVisualizer can reference the enum without
 * pulling in the full UMjCamera include chain (SceneCaptureComponent2D, etc).
 */
UENUM(BlueprintType)
enum class EMjCameraMode : uint8
{
    Real                 UMETA(DisplayName = "Photoreal RGB"),
    Depth                UMETA(DisplayName = "Depth"),
    SemanticSegmentation UMETA(DisplayName = "Semantic Segmentation"),
    InstanceSegmentation UMETA(DisplayName = "Instance Segmentation"),
};

/**
 * @enum EMjCameraTrackingMode
 * @brief MJCF ``<camera mode="...">`` tracking mode. Mirrors MuJoCo's
 *        mjtCamLight subset that applies to cameras. Distinct from
 *        ``EMjCameraMode`` above (which is URLab's render-mode selector,
 *        not a MuJoCo concept).
 *
 *  - Fixed:          camera attached to its body with a fixed transform.
 *  - Track:          camera follows the body's world position, keeps own rotation.
 *  - TrackCom:       like Track but follows the subtree centre of mass.
 *  - TargetBody:     camera stays at its own position; rotates to look at target body.
 *  - TargetBodyCom:  like TargetBody but aims at the target's subtree CoM.
 */
UENUM(BlueprintType)
enum class EMjCameraTrackingMode : uint8
{
    Fixed         UMETA(DisplayName = "Fixed"),
    Track         UMETA(DisplayName = "Track"),
    TrackCom      UMETA(DisplayName = "Track (Centre of Mass)"),
    TargetBody    UMETA(DisplayName = "Target Body"),
    TargetBodyCom UMETA(DisplayName = "Target Body (Centre of Mass)"),
};

/**
 * @enum EMjCameraProjection
 * @brief MJCF ``<camera projection="...">``. Mirrors MuJoCo's mjtProjection.
 *
 *  - Perspective:  standard pinhole camera (default).
 *  - Orthographic: parallel projection, no perspective foreshortening.
 */
UENUM(BlueprintType)
enum class EMjCameraProjection : uint8
{
    Perspective  UMETA(DisplayName = "Perspective"),
    Orthographic UMETA(DisplayName = "Orthographic"),
};
