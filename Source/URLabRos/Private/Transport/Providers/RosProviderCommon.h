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
#include "State/MjStateTypes.h"

// Small helpers shared by the built-in output providers. Header-only inlines; no
// ROS dependency, so they compile in every configuration.
namespace MjRosProvider
{
/** Find a sensor on an articulation by canonical name; null if absent. */
inline const FMjSensorState* FindSensor(const FMjArticulationState& Art, FName Name)
{
	for (const FMjSensorState& Sensor : Art.Sensors)
	{
		if (Sensor.Name == Name)
		{
			return &Sensor;
		}
	}
	return nullptr;
}

/** Copy up to three doubles out of a value array into a fixed vec3, zero-padded. */
inline void FirstThree(const TArray<double>& Values, double Out[3])
{
	Out[0] = Values.Num() > 0 ? Values[0] : 0.0;
	Out[1] = Values.Num() > 1 ? Values[1] : 0.0;
	Out[2] = Values.Num() > 2 ? Values[2] : 0.0;
}
} // namespace MjRosProvider
