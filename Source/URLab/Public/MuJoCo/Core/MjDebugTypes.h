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
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#pragma once

#include "CoreMinimal.h"

/**
 * @struct FMuJoCoDebugData
 * @brief Thread-safe buffer for debug visualization data.
 */
struct FMuJoCoDebugData
{
    TArray<FVector> ContactPoints;
    TArray<FVector> ContactNormals;
    TArray<float> ContactForces;

    /** Per-body awake state snapshot (`mjData.body_awake`), size nbody. 0 = asleep. */
    TArray<int32> BodyAwake;

    /**
     * Halton colour seed per body. `island_dofadr[island]` when in an active
     * constraint island, else `tree_dofadr[tree]` (with `mj_sleepCycle` when
     * asleep). -1 means "don't colour" (e.g. worldbody). Size nbody.
     */
    TArray<int32> BodyIslandSeed;

    /**
     * Flat array of 3D points mirroring MuJoCo's `mjData.wrap_xpos` (stride 3).
     * MuJoCo's own renderer reads point A at offset `3*j` and point B at
     * `3*j + 3`, where j iterates wrap-point index in `[ten_wrapadr[t],
     * ten_wrapadr[t] + ten_wrapnum[t] - 1)`. We convert to UE coords at
     * capture time. Logical size = 2 * nwrap (same as `nwrap x 6` layout).
     */
    TArray<FVector> WrapPointsFlat;

    /** Mirror of `mjData.wrap_obj` (nwrap * 2 ints). -2 = pulley, skip when drawing. */
    TArray<int32> WrapObj;

    /** Per-tendon: `ten_wrapadr` / `ten_wrapnum`, length, and limit range. Sizes = ntendon. */
    TArray<int32> TendonWrapAdr;
    TArray<int32> TendonWrapNum;
    TArray<float> TendonLength;
    TArray<uint8> TendonLimited;
    TArray<float> TendonRangeLo;
    TArray<float> TendonRangeHi;

    /**
     * Per-tendon muscle activation in [0, 1] if any muscle actuator drives this
     * tendon, else -1. Resolved by scanning actuators with trntype==TENDON.
     * Size = ntendon.
     */
    TArray<float> TendonActivation;

    /** Geom world positions (UE coords), size ngeom. Used to centre wrap-arc interpolation. */
    TArray<FVector> GeomXPos;
};
