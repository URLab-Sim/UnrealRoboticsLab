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
#include "UObject/Interface.h"
#include "MjStateProducer.generated.h"

struct FMjArticulationState;
struct FMjStateSnapshot;

UINTERFACE(MinimalAPI)
class UMjStateProducer : public UInterface
{
	GENERATED_BODY()
};

/**
 * The seam any UObject implements to contribute typed values to the per-step
 * state IR. It is transport-agnostic: producers fill the IR and never know which
 * encoders (msgpack, ...) read it afterwards.
 *
 * Both methods run on the physics thread inside the engine's CallbackMutex, the
 * same contract UMjComponent::DescribeState has today. Implementers must not
 * allocate UObjects, must not touch game-thread-only state, and must stay cheap;
 * they run once per physics step. Values are exposed through atomics or a lock
 * (see UMjUserChannelComponent's mailbox), not by reading arbitrary game state.
 *
 * Scope is resolved by the collector at cache-rebuild time on the game thread: a
 * producer owned by (or attached under) an AMjArticulation contributes through
 * DescribeState; any other producer contributes through DescribeSceneState. A
 * given producer only ever has one of the two called per step.
 */
class URLAB_API IMjStateProducer
{
	GENERATED_BODY()

public:
	/** Art-scoped: called when the producer is cached under an articulation. */
	virtual void DescribeState(FMjArticulationState& Out) const {}

	/** Scene-scoped: called for producers not owned by any articulation. */
	virtual void DescribeSceneState(FMjStateSnapshot& Out) const {}
};
