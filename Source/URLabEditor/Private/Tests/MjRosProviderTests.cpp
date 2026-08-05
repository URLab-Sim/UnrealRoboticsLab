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
// MjRosProviderTests.cpp
//
// Tests for the URLabRos output-provider library that need no rcl runtime, so
// they run in every configuration (ROS on or off): the total sensor routing
// table, the sensor topic naming, the Force+Torque wrench pairing, and the
// self-registering provider registry listing the built-ins. These prove the
// provider library is populated and total even when ROS is compiled out, which is
// the point of keeping the routing / registry ROS-agnostic.
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Transport/RosSensorRouting.h"
#include "Transport/RosOutputProvider.h"
#include "Transport/RosStateEstimation.h"
#include "State/MjStateTypes.h"

// ---------------------------------------------------------------------------
// 1. RouteForSemantic is total: every EMjSensorSemantic value maps, the typed
//    routes match the spec, and everything else lands on the MultiArray fallback.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosRouteTableTotal,
	"URLab.Ros.RouteTableTotal",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosRouteTableTotal::RunTest(const FString& Parameters)
{
	// Typed routes.
	TestEqual(TEXT("Gyro -> Imu"), RouteForSemantic(EMjSensorSemantic::Gyro), ERosSensorRoute::Imu);
	TestEqual(TEXT("Accel -> Imu"), RouteForSemantic(EMjSensorSemantic::Accel), ERosSensorRoute::Imu);
	TestEqual(TEXT("Force -> Wrench"), RouteForSemantic(EMjSensorSemantic::Force), ERosSensorRoute::Wrench);
	TestEqual(TEXT("Torque -> Wrench"), RouteForSemantic(EMjSensorSemantic::Torque), ERosSensorRoute::Wrench);
	TestEqual(TEXT("Rangefinder -> Range"), RouteForSemantic(EMjSensorSemantic::Rangefinder), ERosSensorRoute::Range);
	TestEqual(TEXT("Magnetometer -> MagneticField"),
		RouteForSemantic(EMjSensorSemantic::Magnetometer), ERosSensorRoute::MagneticField);
	TestEqual(TEXT("Velocity -> Twist"), RouteForSemantic(EMjSensorSemantic::Velocity), ERosSensorRoute::Twist);

	// A spread of the untyped semantics all fall back to MultiArray.
	TestEqual(TEXT("Generic -> MultiArray"), RouteForSemantic(EMjSensorSemantic::Generic), ERosSensorRoute::MultiArray);
	TestEqual(TEXT("Touch -> MultiArray"), RouteForSemantic(EMjSensorSemantic::Touch), ERosSensorRoute::MultiArray);
	TestEqual(TEXT("JointPos -> MultiArray"), RouteForSemantic(EMjSensorSemantic::JointPos), ERosSensorRoute::MultiArray);
	TestEqual(TEXT("FramePos -> MultiArray"), RouteForSemantic(EMjSensorSemantic::FramePos), ERosSensorRoute::MultiArray);
	TestEqual(TEXT("SubtreeCom -> MultiArray"), RouteForSemantic(EMjSensorSemantic::SubtreeCom), ERosSensorRoute::MultiArray);

	// Totality: every value from Generic..Clock maps to a defined route. The
	// compiler enforces this (the switch has no default); the loop documents it and
	// guards against a value being dropped from the mapping.
	for (uint8 V = 0; V <= static_cast<uint8>(EMjSensorSemantic::Clock); ++V)
	{
		const ERosSensorRoute Route = RouteForSemantic(static_cast<EMjSensorSemantic>(V));
		const bool bValid = Route == ERosSensorRoute::Imu || Route == ERosSensorRoute::Wrench
			|| Route == ERosSensorRoute::Range || Route == ERosSensorRoute::MagneticField
			|| Route == ERosSensorRoute::Twist || Route == ERosSensorRoute::MultiArray;
		TestTrue(*FString::Printf(TEXT("semantic %u maps to a route"), V), bValid);
	}
	return true;
}

// ---------------------------------------------------------------------------
// 2. Sensor topic naming: the MultiArray fallback is the self-describing
//    /<art>/sensors/<name>; typed routes are /<art>/<name>/<suffix>.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosSensorTopics,
	"URLab.Ros.SensorTopics",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosSensorTopics::RunTest(const FString& Parameters)
{
	using namespace MjRosSensorRouting;
	TestEqual(TEXT("unmapped sensor -> /go2/sensors/<name>"),
		TopicFor(TEXT("go2"), TEXT("belly_touch"), ERosSensorRoute::MultiArray),
		FString(TEXT("/go2/sensors/belly_touch")));
	TestEqual(TEXT("wrench topic"),
		TopicFor(TEXT("go2"), TEXT("ankle_ft"), ERosSensorRoute::Wrench),
		FString(TEXT("/go2/ankle_ft/wrench")));
	TestEqual(TEXT("range topic"),
		TopicFor(TEXT("go2"), TEXT("front_range"), ERosSensorRoute::Range),
		FString(TEXT("/go2/front_range/range")));
	TestEqual(TEXT("magnetic field topic"),
		TopicFor(TEXT("go2"), TEXT("mag0"), ERosSensorRoute::MagneticField),
		FString(TEXT("/go2/mag0/magnetic_field")));
	TestEqual(TEXT("velocimeter topic"),
		TopicFor(TEXT("go2"), TEXT("base_vel"), ERosSensorRoute::Twist),
		FString(TEXT("/go2/base_vel/velocity")));
	return true;
}

// ---------------------------------------------------------------------------
// 3. Force+Torque pairing: one force + one torque sensor on an art pair into a
//    single wrench row, named after the force sensor. Extra sensors that are not
//    Force/Torque are ignored by the pairing.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosWrenchPairing,
	"URLab.Ros.WrenchPairing",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosWrenchPairing::RunTest(const FString& Parameters)
{
	FMjArticulationState Art;
	Art.Name = FName(TEXT("go2"));

	FMjSensorState Force;
	Force.Name = FName(TEXT("ankle_force"));
	Force.Semantic = EMjSensorSemantic::Force;
	Force.Values = {1.0, 2.0, 3.0};
	Art.Sensors.Add(Force);

	FMjSensorState Torque;
	Torque.Name = FName(TEXT("ankle_torque"));
	Torque.Semantic = EMjSensorSemantic::Torque;
	Torque.Values = {4.0, 5.0, 6.0};
	Art.Sensors.Add(Torque);

	// A touch sensor should not affect wrench pairing.
	FMjSensorState Touch;
	Touch.Name = FName(TEXT("foot_touch"));
	Touch.Semantic = EMjSensorSemantic::Touch;
	Touch.Values = {0.0};
	Art.Sensors.Add(Touch);

	TArray<FMjWrenchPair> Pairs;
	MjRosSensorRouting::GatherWrenchPairs(Art, Pairs);

	TestEqual(TEXT("one force + one torque -> one wrench pair"), Pairs.Num(), 1);
	if (Pairs.Num() == 1)
	{
		TestEqual(TEXT("pair force sensor"), Pairs[0].ForceSensor, FName(TEXT("ankle_force")));
		TestEqual(TEXT("pair torque sensor"), Pairs[0].TorqueSensor, FName(TEXT("ankle_torque")));
		TestEqual(TEXT("pair named after the force sensor"), Pairs[0].TopicName, FName(TEXT("ankle_force")));
	}

	// An unpaired torque (no force) still produces a wrench row, named after itself.
	FMjArticulationState TorqueOnly;
	TorqueOnly.Name = FName(TEXT("arm"));
	FMjSensorState LoneTorque;
	LoneTorque.Name = FName(TEXT("wrist_torque"));
	LoneTorque.Semantic = EMjSensorSemantic::Torque;
	LoneTorque.Values = {0.1, 0.2, 0.3};
	TorqueOnly.Sensors.Add(LoneTorque);

	TArray<FMjWrenchPair> LonePairs;
	MjRosSensorRouting::GatherWrenchPairs(TorqueOnly, LonePairs);
	TestEqual(TEXT("lone torque -> one wrench row"), LonePairs.Num(), 1);
	if (LonePairs.Num() == 1)
	{
		TestTrue(TEXT("lone torque has no force sensor"), LonePairs[0].ForceSensor.IsNone());
		TestEqual(TEXT("lone torque names the row"), LonePairs[0].TopicName, FName(TEXT("wrist_torque")));
	}
	return true;
}

// ---------------------------------------------------------------------------
// 4. The provider registry is populated with the built-in outputs at module
//    load, in every configuration (this test is not ROS-fenced).
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosProviderRegistryBuiltins,
	"URLab.Ros.ProviderRegistryBuiltins",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosProviderRegistryBuiltins::RunTest(const FString& Parameters)
{
	const TArray<FName> Names = FMjRosOutputRegistry::Get().GetRegisteredNames();

	const TCHAR* Expected[] = {
		TEXT("joint_state"), TEXT("imu"), TEXT("cmd_twist"), TEXT("sensors"),
		TEXT("tf"), TEXT("clock"), TEXT("robot_description"),
		TEXT("odometry"), TEXT("pose"), TEXT("camera_info"), TEXT("rep105_frames")
	};
	for (const TCHAR* Name : Expected)
	{
		TestTrue(*FString::Printf(TEXT("registry lists built-in '%s'"), Name),
			Names.Contains(FName(Name)));
	}

	// Instantiating the registry yields one live provider per registered entry.
	TArray<TUniquePtr<IMjRosOutputProvider>> Providers;
	FMjRosOutputRegistry::Get().InstantiateAll(Providers);
	TestEqual(TEXT("instantiated provider count matches registry"),
		Providers.Num(), FMjRosOutputRegistry::Get().Num());
	TestTrue(TEXT("at least the built-ins are present"),
		Providers.Num() >= UE_ARRAY_COUNT(Expected));
	return true;
}

// ---------------------------------------------------------------------------
// 5. Free-base odometry math: the MuJoCo free-joint convention is asymmetric
//    (qvel = [WORLD linear, BODY angular]). ComputeFreeBaseState must rotate the
//    linear half into the base frame and pass the angular half through, and map
//    the pose (wxyz -> xyzw) and base-body index correctly.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosFreeBaseTwist,
	"URLab.Ros.FreeBaseTwist",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosFreeBaseTwist::RunTest(const FString& Parameters)
{
	using namespace MjRosStateEstimation;

	// Base rotated +90 degrees about world Z: quaternion wxyz = (cos45, 0, 0, sin45).
	const double S = FMath::Sqrt(0.5);

	FMjArticulationState Art;
	Art.Name = FName(TEXT("go2"));

	// A base body whose world pose matches the free joint qpos, so the base-body
	// index resolves to it.
	FMjBodyState Base;
	Base.Name = FName(TEXT("trunk"));
	Base.Xpos[0] = 1.0; Base.Xpos[1] = 2.0; Base.Xpos[2] = 3.0;
	Base.Xquat[0] = S; Base.Xquat[1] = 0.0; Base.Xquat[2] = 0.0; Base.Xquat[3] = S;
	Art.Bodies.Add(Base);

	FMjJointState Free;
	Free.Name = FName(TEXT("root"));
	Free.Type = EMjJointType::Free;
	Free.QPos = {1.0, 2.0, 3.0, S, 0.0, 0.0, S};       // pos + wxyz
	// qvel: WORLD linear (1,0,0), BODY angular (0.1, 0.2, 0.3).
	Free.QVel = {1.0, 0.0, 0.0, 0.1, 0.2, 0.3};
	Art.Joints.Add(Free);

	FMjFreeBaseState State;
	const bool bOk = ComputeFreeBaseState(Art, State);
	TestTrue(TEXT("free-base state derived"), bOk);
	TestTrue(TEXT("state valid"), State.bValid);

	// Pose position straight through; orientation reordered wxyz -> xyzw.
	TestEqual(TEXT("pos x"), State.Position[0], 1.0);
	TestEqual(TEXT("pos z"), State.Position[2], 3.0);
	TestEqual(TEXT("quat x (from wxyz.x)"), State.OrientationXyzw[0], 0.0);
	TestEqual(TEXT("quat z (from wxyz.z)"), State.OrientationXyzw[2], S);
	TestEqual(TEXT("quat w (from wxyz.w)"), State.OrientationXyzw[3], S);

	// A world +X velocity, viewed from a base yawed +90 about Z, reads as base -Y.
	TestEqual(TEXT("linear rotated world->body x"), State.LinearBody[0], 0.0, 1e-9);
	TestEqual(TEXT("linear rotated world->body y"), State.LinearBody[1], -1.0, 1e-9);
	TestEqual(TEXT("linear rotated world->body z"), State.LinearBody[2], 0.0, 1e-9);

	// Angular half is already body-frame: passed through unchanged.
	TestEqual(TEXT("angular x passthrough"), State.AngularBody[0], 0.1);
	TestEqual(TEXT("angular y passthrough"), State.AngularBody[1], 0.2);
	TestEqual(TEXT("angular z passthrough"), State.AngularBody[2], 0.3);

	// The base body matched by world position.
	TestEqual(TEXT("base body index"), State.BaseBodyIndex, 0);

	// A fixed-base art (no free joint) yields no odometry.
	FMjArticulationState Fixed;
	Fixed.Name = FName(TEXT("arm"));
	FMjJointState Hinge;
	Hinge.Name = FName(TEXT("j0"));
	Hinge.Type = EMjJointType::Hinge;
	Hinge.QPos = {0.0};
	Hinge.QVel = {0.0};
	Fixed.Joints.Add(Hinge);
	FMjFreeBaseState None;
	TestFalse(TEXT("fixed-base art has no free-base state"), ComputeFreeBaseState(Fixed, None));

	return true;
}

// ---------------------------------------------------------------------------
// 6. CameraInfo intrinsics: the pinhole K derived from a vertical FOV matches
//    fy = (H/2)/tan(fovy/2), fx = fy, principal point at the image centre.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosCameraInfoIntrinsics,
	"URLab.Ros.CameraInfoIntrinsics",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosCameraInfoIntrinsics::RunTest(const FString& Parameters)
{
	using namespace MjRosStateEstimation;

	// fovy = 90 deg, 640x480: fy = 240/tan(45) = 240, fx = fy, cx = 320, cy = 240.
	double K[9];
	PinholeKFromFovy(90.0, 640, 480, K);
	TestEqual(TEXT("fx"), K[0], 240.0, 1e-6);
	TestEqual(TEXT("cx"), K[2], 320.0, 1e-6);
	TestEqual(TEXT("fy"), K[4], 240.0, 1e-6);
	TestEqual(TEXT("cy"), K[5], 240.0, 1e-6);
	TestEqual(TEXT("K[8] == 1"), K[8], 1.0);
	TestEqual(TEXT("K skew zero"), K[1], 0.0);

	// A non-square image keeps fx == fy (square pixels); the horizontal FOV differs
	// from the vertical because the width differs.
	double K2[9];
	PinholeKFromFovy(60.0, 800, 600, K2);
	const double ExpectedFy = (600.0 * 0.5) / FMath::Tan(FMath::DegreesToRadians(60.0) * 0.5);
	TestEqual(TEXT("fy from 60deg over 600px"), K2[4], ExpectedFy, 1e-6);
	TestEqual(TEXT("fx == fy square pixels"), K2[0], K2[4], 1e-9);
	TestEqual(TEXT("cx at width centre"), K2[2], 400.0, 1e-6);

	// A zero / unset fovy falls back to MuJoCo's 45-degree default rather than
	// collapsing the focal length.
	double K3[9];
	PinholeKFromFovy(0.0, 640, 480, K3);
	const double ExpectedFy45 = (480.0 * 0.5) / FMath::Tan(FMath::DegreesToRadians(45.0) * 0.5);
	TestEqual(TEXT("fovy 0 falls back to 45 deg"), K3[4], ExpectedFy45, 1e-6);

	return true;
}
