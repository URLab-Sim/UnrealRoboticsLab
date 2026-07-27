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

// ============================================================================
// MjJointStateTests.cpp
//
// Correctness tests for UURLabRosPublishTransport::FillJointState, the pure
// IR -> sensor_msgs/JointState transform. FillJointState compiles in every
// configuration (it has no rcl dependency), so these tests need no ROS fence and
// run on every build. They cover:
//  - free/ball joints are excluded so names stay aligned with positions /
//    velocities (the free-base misalignment + truncation bug),
//  - a fixed-base arm (all 1-DOF joints) is passed through unchanged,
//  - the qpos - qpos0 shift is preserved,
//  - effort is filled from the force of the actuator that drives each joint (keyed
//    by target joint, not by the actuator's own name), zero for undriven joints,
//    and empty when the art has no actuators.
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Transport/RosPublishTransport.h"
#include "State/MjStateTypes.h"

namespace
{
FMjJointState MakeJoint(const TCHAR* Name, EMjJointType Type,
	TArray<double> QPos, TArray<double> QVel, TArray<double> RefPos = {})
{
	FMjJointState J;
	J.Name = FName(Name);
	J.Type = Type;
	J.QPos = MoveTemp(QPos);
	J.QVel = MoveTemp(QVel);
	J.RefPos = MoveTemp(RefPos);
	return J;
}

// Name and TargetJoint are deliberately distinct: an actuator's own name need not
// match the joint it drives, and effort must map by the target joint, not the name.
FMjActuatorState MakeActuator(const TCHAR* Name, const TCHAR* TargetJoint, double Force)
{
	FMjActuatorState A;
	A.Name = FName(Name);
	A.TargetJoint = FName(TargetJoint);
	A.Force = Force;
	return A;
}
}  // namespace

// ---------------------------------------------------------------------------
// 1. Free base + 1-DOF joints: the free root is dropped, and every remaining
//    entry's name lines up with its own position / velocity / effort (no shift,
//    no truncation). The free root is placed first, the layout a legged robot
//    uses, so the old whole-slice append would have shifted every hinge.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjJointStateFreeBaseAlignment,
	"URLab.JointState.FreeBaseAlignment",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjJointStateFreeBaseAlignment::RunTest(const FString& Parameters)
{
	FMjArticulationState Art;
	Art.Name = FName(TEXT("go2"));

	Art.Joints.Add(MakeJoint(TEXT("root"), EMjJointType::Free,
		{0.0, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
	Art.Joints.Add(MakeJoint(TEXT("fl_hip"), EMjJointType::Hinge,
		{0.10}, {1.10}, {0.04}));
	Art.Joints.Add(MakeJoint(TEXT("fl_knee"), EMjJointType::Hinge,
		{0.20}, {1.20}));
	Art.Joints.Add(MakeJoint(TEXT("fl_slide"), EMjJointType::Slide,
		{0.30}, {1.30}, {0.05}));

	// fl_hip and fl_knee are driven 1:1 by differently-named actuators; fl_slide has
	// no actuator.
	Art.Actuators.Add(MakeActuator(TEXT("act_fl_hip"), TEXT("fl_hip"), 5.0));
	Art.Actuators.Add(MakeActuator(TEXT("act_fl_knee"), TEXT("fl_knee"), -3.0));

	TArray<FString> Names;
	TArray<double> Positions;
	TArray<double> Velocities;
	TArray<double> Efforts;
	UURLabRosPublishTransport::FillJointState(Art, Names, Positions, Velocities, Efforts);

	// The free root is excluded; only the three 1-DOF joints remain, and every
	// parallel array is the same length (no truncation).
	TestEqual(TEXT("entry count == 1-DOF joint count"), Names.Num(), 3);
	TestEqual(TEXT("positions align with names"), Positions.Num(), Names.Num());
	TestEqual(TEXT("velocities align with names"), Velocities.Num(), Names.Num());
	TestEqual(TEXT("efforts align with names"), Efforts.Num(), Names.Num());

	TestEqual(TEXT("name[0]"), Names[0], FString(TEXT("fl_hip")));
	TestEqual(TEXT("name[1]"), Names[1], FString(TEXT("fl_knee")));
	TestEqual(TEXT("name[2]"), Names[2], FString(TEXT("fl_slide")));

	// name[i] carries fl_hip's own values, proving the free root did not shift it.
	TestEqual(TEXT("fl_hip position is qpos - qpos0"), Positions[0], 0.06, 1e-9);
	TestEqual(TEXT("fl_knee position unshifted (no RefPos)"), Positions[1], 0.20, 1e-9);
	TestEqual(TEXT("fl_slide position is qpos - qpos0"), Positions[2], 0.25, 1e-9);
	TestEqual(TEXT("fl_hip velocity"), Velocities[0], 1.10, 1e-9);
	TestEqual(TEXT("fl_knee velocity"), Velocities[1], 1.20, 1e-9);
	TestEqual(TEXT("fl_slide velocity"), Velocities[2], 1.30, 1e-9);

	// Effort follows the actuator that drives the joint (by target joint, not by
	// the actuator's own name); undriven joints report 0.
	TestEqual(TEXT("fl_hip effort from actuator force"), Efforts[0], 5.0, 1e-9);
	TestEqual(TEXT("fl_knee effort from actuator force"), Efforts[1], -3.0, 1e-9);
	TestEqual(TEXT("fl_slide effort zero (no actuator)"), Efforts[2], 0.0, 1e-9);

	return true;
}

// ---------------------------------------------------------------------------
// 2. Fixed-base arm (all 1-DOF joints, no free/ball): every joint is kept and
//    passed through unchanged, so the fix does not disturb the common case.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjJointStateFixedBaseUnchanged,
	"URLab.JointState.FixedBaseUnchanged",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjJointStateFixedBaseUnchanged::RunTest(const FString& Parameters)
{
	FMjArticulationState Art;
	Art.Name = FName(TEXT("arm"));
	Art.Joints.Add(MakeJoint(TEXT("j0"), EMjJointType::Hinge, {0.1}, {0.01}));
	Art.Joints.Add(MakeJoint(TEXT("j1"), EMjJointType::Hinge, {0.2}, {0.02}));
	Art.Joints.Add(MakeJoint(TEXT("j2"), EMjJointType::Slide, {0.3}, {0.03}));
	Art.Actuators.Add(MakeActuator(TEXT("motor_j1"), TEXT("j1"), 7.5));

	TArray<FString> Names;
	TArray<double> Positions;
	TArray<double> Velocities;
	TArray<double> Efforts;
	UURLabRosPublishTransport::FillJointState(Art, Names, Positions, Velocities, Efforts);

	TestEqual(TEXT("all joints kept"), Names.Num(), 3);
	TestEqual(TEXT("positions align"), Positions.Num(), 3);
	TestEqual(TEXT("velocities align"), Velocities.Num(), 3);
	TestEqual(TEXT("efforts align"), Efforts.Num(), 3);

	TestEqual(TEXT("j0 position"), Positions[0], 0.1, 1e-9);
	TestEqual(TEXT("j2 velocity"), Velocities[2], 0.03, 1e-9);
	TestEqual(TEXT("j0 effort zero (undriven)"), Efforts[0], 0.0, 1e-9);
	TestEqual(TEXT("j1 effort from actuator"), Efforts[1], 7.5, 1e-9);
	TestEqual(TEXT("j2 effort zero (undriven)"), Efforts[2], 0.0, 1e-9);

	return true;
}

// ---------------------------------------------------------------------------
// 3. No actuators: effort is left empty (distinct from an all-zero array) so a
//    consumer can tell "no effort data" from "zero force".
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjJointStateEffortEmptyWithoutActuators,
	"URLab.JointState.EffortEmptyWithoutActuators",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjJointStateEffortEmptyWithoutActuators::RunTest(const FString& Parameters)
{
	FMjArticulationState Art;
	Art.Name = FName(TEXT("passive"));
	Art.Joints.Add(MakeJoint(TEXT("j0"), EMjJointType::Hinge, {0.1}, {0.0}));
	Art.Joints.Add(MakeJoint(TEXT("j1"), EMjJointType::Hinge, {0.2}, {0.0}));

	TArray<FString> Names;
	TArray<double> Positions;
	TArray<double> Velocities;
	TArray<double> Efforts;
	UURLabRosPublishTransport::FillJointState(Art, Names, Positions, Velocities, Efforts);

	TestEqual(TEXT("both joints present"), Names.Num(), 2);
	TestEqual(TEXT("effort empty when no actuators drive the art"), Efforts.Num(), 0);

	return true;
}
