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
#include "UObject/Interface.h"
#include "MjSimClock.generated.h"

UINTERFACE(MinimalAPI)
class UMjSimClock : public UInterface
{
	GENERATED_BODY()
};

/**
 * @interface IMjSimClock
 * @brief Source of the currently-applied post-step render state: the frame id and
 *        mujoco.time that the Unreal scene actors currently reflect.
 *
 * Whatever drives the render owns this — `AAMjManager` on the manager paths, the
 * manager-less `AMjRenderer` on the Mirror render-server path. `UMjCamera` reads its
 * capture-stamp id/time and its delay-reveal "now" through this interface instead of
 * reaching for `AAMjManager::GetManager()`, so the capture/delay pipeline is
 * mode-agnostic (Mirror included) and decoupled from the manager singleton.
 */
class IMjSimClock
{
	GENERATED_BODY()

public:
	/** Post-step render-snapshot id the scene actors currently reflect. */
	virtual uint64 GetAppliedFrameId() const = 0;

	/** mujoco.time (seconds) of that snapshot. */
	virtual double GetAppliedSimTime() const = 0;
};
