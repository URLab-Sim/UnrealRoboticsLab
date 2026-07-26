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
#include "State/MjStateTypes.h" // EMjSensorSemantic, FMjArticulationState

/**
 * The ROS message family a MuJoCo sensor semantic maps to. The mapping is total
 * over EMjSensorSemantic (see RouteForSemantic), so every sensor in a model
 * reaches ROS: a typed message where one exists, else the Float64MultiArray
 * fallback. Nothing is silently dropped.
 */
enum class ERosSensorRoute : uint8
{
	/** Gyro / Accel: pooled into the articulation's sensor_msgs/Imu. Owned by the
	 *  Imu provider, so the generic sensor provider skips this route. */
	Imu,
	/** Force / Torque: paired into a geometry_msgs/WrenchStamped. */
	Wrench,
	/** Rangefinder: sensor_msgs/Range. */
	Range,
	/** Magnetometer: sensor_msgs/MagneticField. */
	MagneticField,
	/** Velocimeter: geometry_msgs/TwistStamped (linear only). */
	Twist,
	/** Everything with no standard typed message (touch, subtree, frame,
	 *  actuator, joint, generic, ...): std_msgs/Float64MultiArray on
	 *  /<art>/sensors/<name>, so coverage is total by construction. */
	MultiArray
};

/**
 * The total sensor-semantic -> ROS route table. Implemented as a switch with no
 * default, so adding an EMjSensorSemantic value fails to compile here until it is
 * routed. That is what keeps ROS sensor coverage total by construction.
 */
URLABROS_API ERosSensorRoute RouteForSemantic(EMjSensorSemantic Semantic);

/** One Force+Torque pairing on an articulation. A pair with both names set is a
 *  fully populated wrench; a half-pair (extra force or torque) leaves the missing
 *  side None and publishes that half with the other zero. TopicName is the sensor
 *  the topic is named after (the force sensor, or the torque sensor if unpaired). */
struct FMjWrenchPair
{
	FName ForceSensor;
	FName TorqueSensor;
	FName TopicName;
};

namespace MjRosSensorRouting
{
	/** The topic a sensor publishes on for its route. The MultiArray fallback uses
	 *  the self-describing /<art>/sensors/<name>; typed routes use
	 *  /<art>/<name>/<suffix>. Pure, exposed for tests. */
	URLABROS_API FString TopicFor(const FString& ArtSegment, const FString& SensorName,
		ERosSensorRoute Route);

	/** Pair an articulation's Force and Torque sensors into wrench rows by array
	 *  order: the i-th force with the i-th torque. One force + one torque yields a
	 *  single fully-populated pair. Pure, exposed for tests. */
	URLABROS_API void GatherWrenchPairs(const FMjArticulationState& Art,
		TArray<FMjWrenchPair>& OutPairs);
}
