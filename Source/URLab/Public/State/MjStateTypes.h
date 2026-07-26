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
	/** The joint's qpos0 (reference) slice. Filled for 1-DOF joints (hinge/slide)
	 *  so the ROS /joint_states shift can emit `qpos - qpos0`, matching the URDF
	 *  zero pose (URDF q=0 == MuJoCo qpos0). Empty means no shift. */
	TArray<double> RefPos;

	void Reset()
	{
		Name = FName();
		QPos.Reset();
		QVel.Reset();
		RefPos.Reset();
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

/** One sensor's reading, as raw MuJoCo SI values (MuJoCo frame, double
 *  precision) copied straight from d->sensordata. The MuJoCo -> UE coordinate/
 *  unit transform lives on the display-facing UMjSensor::GetReading() accessor,
 *  not here; encoders own their own target conventions. */
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

/** Kind tag for one user channel value. Closed set; encoders switch on it. */
enum class EMjUserChannelKind : uint8
{
	Bool,      // Values[0] != 0.0
	Int,       // Values[0], integral
	Scalar,    // Values[0]
	Vec3,      // Values[0..2]
	Quat,      // Values[0..3], wxyz (MuJoCo order, matching xquat)
	Transform, // Values[0..2] pos, Values[3..6] quat wxyz
	Array,     // Values[0..N-1], free length
	String,    // Text
	Struct     // Packed: a msgpack-map blob converted at publish time
};

/**
 * One user-declared payload value. Scoped by which container holds it
 * (FMjArticulationState = art scope, FMjStateSnapshot = scene scope). At most one
 * of Values / Text / Packed is populated per kind; the empty fields cost two idle
 * TArrays, negligible at the unit-to-tens channel counts this targets.
 *
 * Spatial kinds (Vec3/Quat/Transform) carry raw MuJoCo SI values (metres, wxyz),
 * converted from UE space in the publish path so the IR stays transport-neutral
 * and matches every other pose in the snapshot.
 */
struct FMjUserChannel
{
	FName Name;                                       // sanitized, unique within its scope
	EMjUserChannelKind Kind = EMjUserChannelKind::Scalar;
	TArray<double> Values;                            // numeric kinds
	FString Text;                                     // String kind
	TArray<uint8> Packed;                             // Struct kind: msgpack map bytes
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
	TArray<FMjUserChannel> UserChannels; // art-scoped user payloads

	void Reset()
	{
		Name = FName();
		Joints.Reset();
		Actuators.Reset();
		Sensors.Reset();
		Bodies.Reset();
		Twist.Reset();
		UserChannels.Reset();
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
	TArray<FMjUserChannel> UserChannels; // scene-scoped user payloads

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
		UserChannels.Reset();
	}
};
