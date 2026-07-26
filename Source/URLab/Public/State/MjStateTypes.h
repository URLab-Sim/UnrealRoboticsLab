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
#include "MuJoCo/Components/Joints/MjJoint.h" // EMjJointType

/**
 * Semantically-typed intermediate representation of one physics step. Producers
 * declare into it once (UMjComponent::DescribeState); encoders read it. There is
 * one IR and, downstream, two encoders: msgpack (state/full + step replies) and,
 * later, ROS. Semantics and grouping are first-class because typed ROS messages
 * need them; msgpack ignores the extra typing harmlessly.
 *
 * Names are FName so the per-step copies the collector makes are index-cheap and
 * never allocate. Numeric slots are double so no precision is lost relative to
 * mjData's mjtNum.
 */

/**
 * Coarse grouping derived from EMjSensorType. It is what lets a ROS publisher
 * pair gyro+accel into one Imu and route pose-like sensors to tf2. msgpack does
 * not read it.
 */
enum class EMjSensorSemantic : uint8
{
	Generic,
	Gyro,
	Accel,
	Velocity,
	Force,
	Torque,
	Touch,
	Rangefinder,
	Magnetometer,
	JointPos,
	JointVel,
	ActuatorPos,
	ActuatorVel,
	ActuatorFrc,
	FramePos,
	FrameQuat,
	FrameAxis,
	FrameLinVel,
	FrameAngVel,
	FrameLinAcc,
	FrameAngAcc,
	SubtreeCom,
	SubtreeLinVel,
	SubtreeAngMom,
	Clock
};

/**
 * Sim + wall clock, stored as ROS builtin_interfaces/Time sec/nsec pairs so the
 * projection to the wire matches AppendClockFields exactly. int32 sec + int32
 * nsec both fit a double's 2^53 mantissa; wall seconds since epoch need int64.
 */
struct FMjClock
{
	int32 SimSec = 0;
	int32 SimNsec = 0;
	int64 WallSec = 0;
	int64 WallNsec = 0;
};

/** One joint's position/velocity slices. Slot widths follow the joint type. */
struct FMjJointState
{
	FName Name;
	EMjJointType Type = EMjJointType::Hinge;
	TArray<double> QPos;
	TArray<double> QVel;

	void Reset()
	{
		Name = FName();
		QPos.Reset();
		QVel.Reset();
	}
};

/** One actuator's control setpoint, activation state, and applied force. */
struct FMjActuatorState
{
	FName Name;
	double Ctrl = 0.0;
	double Act = 0.0;
	double Force = 0.0;
};

/** One sensor's reading, in the same transformed units GetReading() returns. */
struct FMjSensorState
{
	FName Name;
	EMjSensorSemantic Semantic = EMjSensorSemantic::Generic;
	TArray<double> Values;
	FName FrameId;

	void Reset()
	{
		Name = FName();
		Semantic = EMjSensorSemantic::Generic;
		Values.Reset();
		FrameId = FName();
	}
};

/** One body's world pose. The body name is its own tf2 frame id. */
struct FMjBodyState
{
	FName Name;
	double Xpos[3] = {0.0, 0.0, 0.0};
	double Xquat[4] = {1.0, 0.0, 0.0, 0.0};
};

/** A twist command (linear + angular) plus the active-action bitmask. */
struct FMjTwistState
{
	double Linear[3] = {0.0, 0.0, 0.0};
	double Angular[3] = {0.0, 0.0, 0.0};
	int32 Actions = 0;
};

/** All per-step state for one articulation, grouped by element kind. */
struct FMjArticulationState
{
	FName Name; // canonical art segment
	TArray<FMjJointState> Joints;
	TArray<FMjActuatorState> Actuators;
	TArray<FMjSensorState> Sensors;
	TArray<FMjBodyState> Bodies;
	TOptional<FMjTwistState> Twist;

	void Reset()
	{
		Name = FName();
		Joints.Reset();
		Actuators.Reset();
		Sensors.Reset();
		Bodies.Reset();
		Twist.Reset();
	}
};

/**
 * A non-articulation dynamic body (prop, free-jointed scene object). These are
 * raw MjIds with no owning component, so the collector fills them directly from
 * mjData rather than through a DescribeState producer.
 */
struct FMjEntityState
{
	FName Name;
	double Xpos[3] = {0.0, 0.0, 0.0};
	double Xquat[4] = {1.0, 0.0, 0.0, 0.0};
	bool bFreeBase = false;
	TArray<double> QPos;
	TArray<double> QVel;
};

/**
 * The whole per-step snapshot. Built full every step; the msgpack encoder omits
 * blocks per the requested observation level. StructureVersion bumps whenever
 * the producer set changes so consumers can cache derived state (key tables, ROS
 * publisher handles) and invalidate only on a registry change.
 */
struct FMjStateSnapshot
{
	double Time = 0.0;
	int64 Step = 0;
	FMjClock Clock;
	uint32 StructureVersion = 0;
	TArray<FMjArticulationState> Articulations;
	TArray<FMjEntityState> Entities;

	/** Clears the payload while keeping the top-level array capacity so the
	 *  steady-state per-step build does not reallocate the outer arrays. */
	void Reset()
	{
		Time = 0.0;
		Step = 0;
		Clock = FMjClock();
		StructureVersion = 0;
		Articulations.Reset();
		Entities.Reset();
	}
};
