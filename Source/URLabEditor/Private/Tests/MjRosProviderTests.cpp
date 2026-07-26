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
		TEXT("tf"), TEXT("clock"), TEXT("robot_description")
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
