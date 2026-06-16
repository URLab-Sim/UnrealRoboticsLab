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
#include "MuJoCo/Generated/MjArticulationEnums.h"
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
	Real UMETA(DisplayName = "Photoreal RGB"),
	Depth UMETA(DisplayName = "Depth"),
	SemanticSegmentation UMETA(DisplayName = "Semantic Segmentation"),
	InstanceSegmentation UMETA(DisplayName = "Instance Segmentation"),
};

// EMjCameraTrackingMode, EMjCameraProjection moved to
// MuJoCo/Generated/MjArticulationEnums.h.

/** Magic identifying a camera frame-metadata header. Stored as a uint32 that
 *  reads as the ASCII bytes "UCM1" in a little-endian hexdump, so the C++ and
 *  Python sides agree by inspection. */
inline constexpr uint32 URLAB_CAMERA_META_MAGIC = 0x314D4355u; // 'UCM1'
inline constexpr uint32 URLAB_CAMERA_META_VERSION = 1u;

/**
 * @struct FMjCameraFrameMeta
 * @brief Per-frame metadata prepended to streamed camera pixels on BOTH the
 *        ZMQ and SHM transports.
 *
 * The streaming channels (PUB socket / SHM ring) are how the client gets
 * camera frames in every step mode — frames are no longer bundled into the
 * step RPC reply. This header is what lets the client associate a streamed
 * frame with the step that produced it (FrameId) so a "fresh" query can wait
 * for FrameId >= the step's post-state id.
 *
 * Fixed 32-byte POD, little-endian, no padding (verified by static_assert).
 * The Python consumer parses the identical layout: "<IIQdII".
 *
 * Layout (offsets in bytes):
 *   0  uint32 magic
 *   4  uint32 version
 *   8  uint64 frame_id
 *   16 double sim_time
 *   24 uint32 width
 *   28 uint32 height
 */
struct FMjCameraFrameMeta
{
	uint32 Magic = URLAB_CAMERA_META_MAGIC;
	uint32 Version = URLAB_CAMERA_META_VERSION;
	uint64 FrameId = 0;  // post-step render-snapshot id this frame shows
	double SimTime = 0.0;
	uint32 Width = 0;
	uint32 Height = 0;
};
static_assert(sizeof(FMjCameraFrameMeta) == 32,
	"FMjCameraFrameMeta must be exactly 32 bytes for cross-language ABI");
