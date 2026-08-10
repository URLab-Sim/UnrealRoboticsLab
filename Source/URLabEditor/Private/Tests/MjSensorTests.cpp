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

// Sensors authored into a spec, compiled, and read back.
//
// A sensor is a child of the `<sensor>` section and its kind is its element
// class, so authoring one is two Add calls: the section, then the leaf. The
// reads all go through UMjSensorRuntime, which takes the element and resolves
// the model and the address itself.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjTestHelpers.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Elements/MjJointRuntime.h"
#include "MuJoCo/Elements/MjSensorRuntime.h"
#include "MuJoCo/Gen/Elements/Sensors/MjJointpos.gen.h"
#include "MuJoCo/Gen/Elements/Sensors/MjJointvel.gen.h"
#include "MuJoCo/Gen/Elements/MjModel.gen.h"
#include "MuJoCo/Gen/Elements/Sensors/MjSensor.gen.h"
#include "Engine/World.h"
#include "mujoco/mujoco.h"

namespace MjSensorTests
{
/** The `<sensor>` section of the session's articulation, created on demand. */
UMjSensor* SensorSection(FMjUESession& Sess)
{
	return Sess.Add<UMjSensor>(Sess.Robot->Spec);
}
}  // namespace MjSensorTests

// ============================================================================
// URLab.Sensor.JointPosSensor_Binds
//   Verify that a <jointpos> targeting a hinge joint receives a valid sensor ID
//   after compilation, and that nsensor == 1.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSensorJointPosSensorBinds,
	"URLab.Sensor.JointPosSensor_Binds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjSensorJointPosSensorBinds::RunTest(const FString& Parameters)
{
	UMjJointpos* Sensor = nullptr;

	FMjUESession S;
	if (!S.Init([&Sensor](FMjUESession& Sess) {
			Sess.Joint->SetType(EMjJointType::hinge);

			Sensor = Sess.Add<UMjJointpos>(MjSensorTests::SensorSection(Sess), TEXT("TestSensor"));
			Sensor->Joint = TEXT("TestJoint");
		}))
	{
		AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	TestNotNull(TEXT("Sensor should not be null after Init"), Sensor);
	if (Sensor)
	{
		TestTrue(TEXT("Sensor should be bound after the compile"), Sensor->GetBoundId().IsSet());
		TestTrue(TEXT("Sensor's bound id should be >= 0"), Sensor->GetBoundId().Get(-1) >= 0);
	}

	TestEqual(TEXT("Manager->PhysicsEngine->m_model->nsensor should be 1"),
		(int)S.Manager->PhysicsEngine->m_model->nsensor, 1);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Sensor.JointPosSensor_GetReading
//   Set the hinge joint position to 1.5 rad, run mj_forward, and confirm
//   that GetReading() returns a value approximately equal to 1.5.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSensorJointPosSensorGetReading,
	"URLab.Sensor.JointPosSensor_GetReading",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjSensorJointPosSensorGetReading::RunTest(const FString& Parameters)
{
	UMjJointpos* Sensor = nullptr;

	FMjUESession S;
	if (!S.Init([&Sensor](FMjUESession& Sess) {
			Sess.Joint->SetType(EMjJointType::hinge);

			Sensor = Sess.Add<UMjJointpos>(MjSensorTests::SensorSection(Sess), TEXT("TestSensor"));
			Sensor->Joint = TEXT("TestJoint");
		}))
	{
		AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	TestNotNull(TEXT("Sensor should not be null"), Sensor);
	if (!Sensor)
	{
		S.Cleanup();
		return false;
	}

	// Drive the joint to 1.5 rad and propagate through kinematics
	UMjJointRuntime::SetPosition(S.Joint, 1.5f);
	S.Manager->PhysicsEngine->ForwardSync();

	TArray<float> Reading = UMjSensorRuntime::GetReading(Sensor);
	TestTrue(TEXT("GetReading() should return at least one element"), Reading.Num() > 0);

	if (Reading.Num() > 0)
	{
		TestTrue(TEXT("GetReading()[0] should be approximately 1.5"),
			FMath::Abs(Reading[0] - 1.5f) < 0.01f);
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Sensor.JointPosSensor_DimMatchesModel
//   Verify that the dimension reported for the sensor matches the value stored
//   in sensor_dim[] of the compiled mjModel (should be 1 for jointpos).
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSensorJointPosSensorDimMatchesModel,
	"URLab.Sensor.JointPosSensor_DimMatchesModel",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjSensorJointPosSensorDimMatchesModel::RunTest(const FString& Parameters)
{
	UMjJointpos* Sensor = nullptr;

	FMjUESession S;
	if (!S.Init([&Sensor](FMjUESession& Sess) {
			Sess.Joint->SetType(EMjJointType::hinge);

			Sensor = Sess.Add<UMjJointpos>(MjSensorTests::SensorSection(Sess), TEXT("TestSensor"));
			Sensor->Joint = TEXT("TestJoint");
		}))
	{
		AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	TestNotNull(TEXT("Sensor should not be null"), Sensor);
	// The id is read from the model by name so the comparison has an independent
	// left-hand side: the library resolving the same width off the element's own
	// bound id is what is under test.
	const int32 ModelId = S.MjId(mjOBJ_SENSOR, TEXT("TestSensor"));
	if (Sensor && ModelId >= 0)
	{
		const int32 ModelDim = S.Manager->PhysicsEngine->m_model->sensor_dim[ModelId];
		TestEqual(TEXT("GetDimension should match the model's sensor_dim for this sensor"),
			UMjSensorRuntime::GetDimension(Sensor), ModelDim);
		TestEqual(TEXT("JointPos sensor dimension should be 1"),
			UMjSensorRuntime::GetDimension(Sensor), 1);
	}
	else
	{
		AddError(TEXT("the sensor did not reach the compiled model"));
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Sensor.MultipleSensors_AllBind
//   Two hinge joints, each with its own <jointpos>.  Both sensor IDs must be
//   >= 0 after compilation, and nsensor must equal 2.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSensorMultipleSensorsAllBind,
	"URLab.Sensor.MultipleSensors_AllBind",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjSensorMultipleSensorsAllBind::RunTest(const FString& Parameters)
{
	UMjJointpos* Sensor1 = nullptr;
	UMjJointpos* Sensor2 = nullptr;

	FMjUESession S;
	if (!S.Init([&Sensor1, &Sensor2](FMjUESession& Sess) {
			// Configure the default joint as a hinge
			Sess.Joint->SetType(EMjJointType::hinge);

			// Second body + joint hierarchy
			UMjBody* Body2 = Sess.Add<UMjBody>(Sess.Body, TEXT("Body2"));

			UMjGeom* Geom2 = Sess.Add<UMjGeom>(Body2, TEXT("Geom2"));
			Geom2->SetSize({0.1});

			UMjJoint* Joint2 = Sess.Add<UMjJoint>(Body2, TEXT("TestJoint2"));
			Joint2->SetType(EMjJointType::hinge);

			// One <sensor> section holds both leaves, as MJCF has it.
			UMjSensor* Section = MjSensorTests::SensorSection(Sess);

			Sensor1 = Sess.Add<UMjJointpos>(Section, TEXT("TestSensor1"));
			Sensor1->Joint = TEXT("TestJoint");

			Sensor2 = Sess.Add<UMjJointpos>(Section, TEXT("TestSensor2"));
			Sensor2->Joint = TEXT("TestJoint2");
		}))
	{
		AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	TestNotNull(TEXT("Sensor1 should not be null"), Sensor1);
	TestNotNull(TEXT("Sensor2 should not be null"), Sensor2);

	if (Sensor1)
	{
		TestTrue(TEXT("Sensor1's bound id should be >= 0"), Sensor1->GetBoundId().Get(-1) >= 0);
	}
	if (Sensor2)
	{
		TestTrue(TEXT("Sensor2's bound id should be >= 0"), Sensor2->GetBoundId().Get(-1) >= 0);
	}

	TestEqual(TEXT("Manager->PhysicsEngine->m_model->nsensor should be 2"),
		(int)S.Manager->PhysicsEngine->m_model->nsensor, 2);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Sensor.JointVelSensor_Binds
//   Verify that a <jointvel> targeting a hinge joint receives a valid sensor ID
//   (>= 0) after compilation.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSensorJointVelSensorBinds,
	"URLab.Sensor.JointVelSensor_Binds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjSensorJointVelSensorBinds::RunTest(const FString& Parameters)
{
	UMjJointvel* Sensor = nullptr;

	FMjUESession S;
	if (!S.Init([&Sensor](FMjUESession& Sess) {
			Sess.Joint->SetType(EMjJointType::hinge);

			Sensor = Sess.Add<UMjJointvel>(MjSensorTests::SensorSection(Sess), TEXT("TestVelSensor"));
			Sensor->Joint = TEXT("TestJoint");
		}))
	{
		AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	TestNotNull(TEXT("Sensor should not be null after Init"), Sensor);
	if (Sensor)
	{
		TestTrue(TEXT("Sensor's bound id should be >= 0"), Sensor->GetBoundId().Get(-1) >= 0);
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Sensor.SensorViewModelPointer
//   The sensor has to have bound against the model the manager compiled, and
//   nothing but the id says so now: the id it carries must be the id its
//   compiled name resolves to in that model.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSensorSensorViewModelPointer,
	"URLab.Sensor.SensorViewModelPointer",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjSensorSensorViewModelPointer::RunTest(const FString& Parameters)
{
	UMjJointpos* Sensor = nullptr;

	FMjUESession S;
	if (!S.Init([&Sensor](FMjUESession& Sess) {
			Sess.Joint->SetType(EMjJointType::hinge);

			Sensor = Sess.Add<UMjJointpos>(MjSensorTests::SensorSection(Sess), TEXT("TestSensor"));
			Sensor->Joint = TEXT("TestJoint");
		}))
	{
		AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	TestNotNull(TEXT("Sensor should not be null"), Sensor);
	if (Sensor)
	{
		const int32 FromModel = S.MjId(mjOBJ_SENSOR, TEXT("TestSensor"));
		TestTrue(TEXT("the sensor survived the compile"), FromModel >= 0);
		TestTrue(TEXT("the sensor is bound"), Sensor->GetBoundId().IsSet());
		TestEqual(TEXT("Sensor's bound id is the id its name resolves to in the manager's model"),
			Sensor->GetBoundId().Get(-1), FromModel);
	}

	S.Cleanup();
	return true;
}
