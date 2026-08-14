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

// Actuators authored into a spec, compiled, and driven.
//
// A <motor> is a child of the <actuator> section and its kind is its element
// class, so authoring one is two Add calls. Driving it goes through
// UMjActuatorRuntime, which stages onto the articulation's slots rather than
// onto the element.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjTestHelpers.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Elements/MjActuatorRuntime.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Gen/Elements/Actuators/MjActuator.gen.h"
#include "MuJoCo/Gen/Elements/MjModel.gen.h"
#include "MuJoCo/Gen/Elements/Actuators/MjMotor.gen.h"
#include "Engine/World.h"
#include "mujoco/mujoco.h"

namespace MjActuatorTests
{
/** The `<actuator>` section of the session's articulation. */
UMjActuator* ActuatorSection(FMjUESession& Sess)
{
	return Sess.Add<UMjActuator>(Sess.Robot->Spec);
}
} // namespace MjActuatorTests

// ============================================================================
// URLab.Actuator.MotorActuator_Binds
//   Verify that a <motor> targeting a hinge joint receives a valid actuator ID
//   after compilation, and that nu == 1.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjActuatorMotorActuatorBinds,
	"URLab.Actuator.MotorActuator_Binds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjActuatorMotorActuatorBinds::RunTest(const FString& Parameters)
{
	UMjMotor* Actuator = nullptr;

	FMjUESession S;
	if (!S.Init([&Actuator](FMjUESession& Sess) {
			Sess.Joint->SetType(EMjJointType::hinge);

			Actuator = Sess.Add<UMjMotor>(MjActuatorTests::ActuatorSection(Sess), TEXT("TestActuator"));
			Actuator->SetJoint(TEXT("TestJoint"));
		}))
	{
		AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	TestNotNull(TEXT("Actuator should not be null after Init"), Actuator);
	if (Actuator)
	{
		TestTrue(TEXT("Actuator should be bound after the compile"), Actuator->GetBoundId().IsSet());
		TestTrue(TEXT("Actuator's bound id should be >= 0"), Actuator->GetBoundId().Get(-1) >= 0);
	}

	TestEqual(TEXT("Manager->PhysicsEngine->m_model->nu should be 1"),
		(int)S.Manager->PhysicsEngine->m_model->nu, 1);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Actuator.MotorActuator_SetControl
//   Call SetControl(5.0f) on a bound motor actuator and verify that
//   GetControl() returns approximately 5.0.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjActuatorMotorActuatorSetControl,
	"URLab.Actuator.MotorActuator_SetControl",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjActuatorMotorActuatorSetControl::RunTest(const FString& Parameters)
{
	UMjMotor* Actuator = nullptr;

	FMjUESession S;
	if (!S.Init([&Actuator](FMjUESession& Sess) {
			Sess.Joint->SetType(EMjJointType::hinge);

			Actuator = Sess.Add<UMjMotor>(MjActuatorTests::ActuatorSection(Sess), TEXT("TestActuator"));
			Actuator->SetJoint(TEXT("TestJoint"));

			// ControlSource 0 is the network slot, anything else the UI one.
			// SetControl writes the UI slot, so the source has to select it for
			// GetControl to read back what was staged.
			Sess.Robot->ControlSource = 1;
		}))
	{
		AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	TestNotNull(TEXT("Actuator should not be null"), Actuator);
	if (!Actuator)
	{
		S.Cleanup();
		return false;
	}

	UMjActuatorRuntime::SetControl(Actuator, 5.0f);

	TestTrue(TEXT("GetControl() should return approximately 5.0 after SetControl(5.0f)"),
		FMath::Abs(UMjActuatorRuntime::GetControl(Actuator) - 5.0f) < 0.001f);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Actuator.NuCountMatchesActuators
//   A single <motor> should result in nu == 1 on the compiled model.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjActuatorNuCountMatchesActuators,
	"URLab.Actuator.NuCountMatchesActuators",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjActuatorNuCountMatchesActuators::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init([](FMjUESession& Sess) {
			Sess.Joint->SetType(EMjJointType::hinge);

			UMjMotor* Actuator =
				Sess.Add<UMjMotor>(MjActuatorTests::ActuatorSection(Sess), TEXT("TestActuator"));
			Actuator->SetJoint(TEXT("TestJoint"));
		}))
	{
		AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	TestEqual(TEXT("Manager->PhysicsEngine->m_model->nu should be 1 for one motor actuator"),
		(int)S.Manager->PhysicsEngine->m_model->nu, 1);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Actuator.MultipleActuators_AllBind
//   Two hinge joints, each with its own <motor>.  Both actuator IDs must be
//   >= 0 after compilation and nu must equal 2.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjActuatorMultipleActuatorsAllBind,
	"URLab.Actuator.MultipleActuators_AllBind",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjActuatorMultipleActuatorsAllBind::RunTest(const FString& Parameters)
{
	UMjMotor* Act1 = nullptr;
	UMjMotor* Act2 = nullptr;

	FMjUESession S;
	if (!S.Init([&Act1, &Act2](FMjUESession& Sess) {
			// Configure the default joint as a hinge
			Sess.Joint->SetType(EMjJointType::hinge);

			// Second body + joint hierarchy
			UMjBody* Body2 = Sess.Add<UMjBody>(Sess.Body, TEXT("Body2"));

			UMjGeom* Geom2 = Sess.Add<UMjGeom>(Body2, TEXT("Geom2"));
			Geom2->SetSize({0.1});

			UMjJoint* Joint2 = Sess.Add<UMjJoint>(Body2, TEXT("TestJoint2"));
			Joint2->SetType(EMjJointType::hinge);

			// One <actuator> section holds both leaves, as MJCF has it.
			UMjActuator* Section = MjActuatorTests::ActuatorSection(Sess);

			Act1 = Sess.Add<UMjMotor>(Section, TEXT("TestActuator1"));
			Act1->SetJoint(TEXT("TestJoint"));

			Act2 = Sess.Add<UMjMotor>(Section, TEXT("TestActuator2"));
			Act2->SetJoint(TEXT("TestJoint2"));
		}))
	{
		AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	TestNotNull(TEXT("Act1 should not be null"), Act1);
	TestNotNull(TEXT("Act2 should not be null"), Act2);

	if (Act1)
	{
		TestTrue(TEXT("Act1's bound id should be >= 0"), Act1->GetBoundId().Get(-1) >= 0);
	}
	if (Act2)
	{
		TestTrue(TEXT("Act2's bound id should be >= 0"), Act2->GetBoundId().Get(-1) >= 0);
	}

	TestEqual(TEXT("Manager->PhysicsEngine->m_model->nu should be 2"),
		(int)S.Manager->PhysicsEngine->m_model->nu, 2);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Actuator.MotorActuator_GetForce_NocrashAfterStep
//   Verify that calling GetForce() after stepping does not crash and returns
//   a finite floating-point value.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjActuatorMotorActuatorGetForceNoCrashAfterStep,
	"URLab.Actuator.MotorActuator_GetForce_NocrashAfterStep",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjActuatorMotorActuatorGetForceNoCrashAfterStep::RunTest(const FString& Parameters)
{
	UMjMotor* Actuator = nullptr;

	FMjUESession S;
	if (!S.Init([&Actuator](FMjUESession& Sess) {
			Sess.Joint->SetType(EMjJointType::hinge);

			Actuator = Sess.Add<UMjMotor>(MjActuatorTests::ActuatorSection(Sess), TEXT("TestActuator"));
			Actuator->SetJoint(TEXT("TestJoint"));
		}))
	{
		AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	TestNotNull(TEXT("Actuator should not be null"), Actuator);
	if (!Actuator)
	{
		S.Cleanup();
		return false;
	}

	S.Step(5);

	const float Force = UMjActuatorRuntime::GetForce(Actuator);
	TestTrue(TEXT("GetForce() should return a finite value after 5 steps"),
		FMath::IsFinite(Force));

	S.Cleanup();
	return true;
}
