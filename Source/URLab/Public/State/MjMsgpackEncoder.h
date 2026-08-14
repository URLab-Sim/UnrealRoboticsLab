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
#include "Dom/JsonObject.h"
#include "State/MjObservationLevel.h"

struct FMjStateSnapshot;

/**
 * The msgpack encoder: IR -> FJsonObject (packed to bytes via FURLabMsgpackUtil).
 * One writer serves both consumers -- the streamed state/full snapshot packs the
 * object to bytes, and RPC step replies embed the same arts/scene objects in the
 * reply tree.
 *
 * Canonical schema. The full snapshot is
 *   { op:"state_full", time, step, sim_time, wall_time, arts, scene }.
 * Per articulation, by observation level:
 *   minimal  -> { qpos, qvel }
 *   standard -> +{ ctrl, act, sensors:{name:[...]} }
 *   full     -> +{ bodies:{name:{xpos,xquat}}, actuator_force }
 *   twist    -> { twist:{linear,angular}, actions }  (whenever a twist controller
 *               is attached; independent of level)
 * The scene block is { name:{ xpos, xquat, [qpos, qvel] } } for free-based props.
 */
class URLAB_API FMjMsgpackEncoder
{
public:
	static TSharedPtr<FJsonObject> EncodeSnapshot(const FMjStateSnapshot& S,
		EObservationLevel Level);
	static TSharedPtr<FJsonObject> EncodeArts(const FMjStateSnapshot& S,
		EObservationLevel Level);
	static TSharedPtr<FJsonObject> EncodeScene(const FMjStateSnapshot& S);
	static TArray<uint8> EncodeSnapshotBytes(const FMjStateSnapshot& S,
		EObservationLevel Level);
};
