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

#include "Transport/RosSensorRouting.h"

ERosSensorRoute RouteForSemantic(EMjSensorSemantic Semantic)
{
	// Total switch, no default: a new EMjSensorSemantic value breaks the build here
	// until it is routed, keeping ROS sensor coverage total by construction.
	switch (Semantic)
	{
		case EMjSensorSemantic::Gyro:
		case EMjSensorSemantic::Accel:
			return ERosSensorRoute::Imu;

		case EMjSensorSemantic::Force:
		case EMjSensorSemantic::Torque:
			return ERosSensorRoute::Wrench;

		case EMjSensorSemantic::Rangefinder:
			return ERosSensorRoute::Range;

		case EMjSensorSemantic::Magnetometer:
			return ERosSensorRoute::MagneticField;

		case EMjSensorSemantic::Velocity:
			return ERosSensorRoute::Twist;

		case EMjSensorSemantic::Generic:
		case EMjSensorSemantic::Touch:
		case EMjSensorSemantic::JointPos:
		case EMjSensorSemantic::JointVel:
		case EMjSensorSemantic::ActuatorPos:
		case EMjSensorSemantic::ActuatorVel:
		case EMjSensorSemantic::ActuatorFrc:
		case EMjSensorSemantic::FramePos:
		case EMjSensorSemantic::FrameQuat:
		case EMjSensorSemantic::FrameAxis:
		case EMjSensorSemantic::FrameLinVel:
		case EMjSensorSemantic::FrameAngVel:
		case EMjSensorSemantic::FrameLinAcc:
		case EMjSensorSemantic::FrameAngAcc:
		case EMjSensorSemantic::SubtreeCom:
		case EMjSensorSemantic::SubtreeLinVel:
		case EMjSensorSemantic::SubtreeAngMom:
		case EMjSensorSemantic::Clock:
			return ERosSensorRoute::MultiArray;
	}

	// Unreachable: the switch above is total. Present only so a corrupt cast does
	// not fall through to undefined behaviour.
	return ERosSensorRoute::MultiArray;
}

namespace MjRosSensorRouting
{
FString TopicFor(const FString& ArtSegment, const FString& SensorName, ERosSensorRoute Route)
{
	switch (Route)
	{
		case ERosSensorRoute::Imu:
			return FString::Printf(TEXT("/%s/imu"), *ArtSegment);
		case ERosSensorRoute::Wrench:
			return FString::Printf(TEXT("/%s/%s/wrench"), *ArtSegment, *SensorName);
		case ERosSensorRoute::Range:
			return FString::Printf(TEXT("/%s/%s/range"), *ArtSegment, *SensorName);
		case ERosSensorRoute::MagneticField:
			return FString::Printf(TEXT("/%s/%s/magnetic_field"), *ArtSegment, *SensorName);
		case ERosSensorRoute::Twist:
			return FString::Printf(TEXT("/%s/%s/velocity"), *ArtSegment, *SensorName);
		case ERosSensorRoute::MultiArray:
		default:
			return FString::Printf(TEXT("/%s/sensors/%s"), *ArtSegment, *SensorName);
	}
}

void GatherWrenchPairs(const FMjArticulationState& Art, TArray<FMjWrenchPair>& OutPairs)
{
	OutPairs.Reset();

	TArray<FName> Forces;
	TArray<FName> Torques;
	for (const FMjSensorState& Sensor : Art.Sensors)
	{
		if (RouteForSemantic(Sensor.Semantic) != ERosSensorRoute::Wrench)
		{
			continue;
		}
		if (Sensor.Semantic == EMjSensorSemantic::Force)
		{
			Forces.Add(Sensor.Name);
		}
		else if (Sensor.Semantic == EMjSensorSemantic::Torque)
		{
			Torques.Add(Sensor.Name);
		}
	}

	const int32 N = FMath::Max(Forces.Num(), Torques.Num());
	OutPairs.Reserve(N);
	for (int32 i = 0; i < N; ++i)
	{
		FMjWrenchPair Pair;
		Pair.ForceSensor = Forces.IsValidIndex(i) ? Forces[i] : NAME_None;
		Pair.TorqueSensor = Torques.IsValidIndex(i) ? Torques[i] : NAME_None;
		Pair.TopicName = Pair.ForceSensor.IsNone() ? Pair.TorqueSensor : Pair.ForceSensor;
		OutPairs.Add(Pair);
	}
}
} // namespace MjRosSensorRouting
