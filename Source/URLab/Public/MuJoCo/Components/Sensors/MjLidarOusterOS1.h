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
#include "MuJoCo/Components/Sensors/MjLidarSensor.h"
#include "MjLidarOusterOS1.generated.h"

/**
 * @class UMjLidarOusterOS1
 * @brief Ouster OS1-64-style revolving lidar: 1024 x 64 beams, 360-degree
 * azimuth, 45-degree vertical FOV (+22.5 to -22.5), 10 Hz, 120 m range.
 *
 * Concrete lidar devices stay intentionally tiny: the constructor only sets
 * resolution, FOV, and rates on the UMjLidarSensor base, per the roadmap's
 * "drop-in sensor device library" direction.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent, DisplayName = "Mj Lidar Ouster OS1-64"))
class URLAB_API UMjLidarOusterOS1 : public UMjLidarSensor
{
	GENERATED_BODY()

public:
	UMjLidarOusterOS1();
};
