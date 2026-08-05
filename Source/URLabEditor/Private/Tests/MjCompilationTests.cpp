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

// What an authored spec compiles to.
//
// Every case here writes attributes onto spec elements and then reads the
// compiled model back, because that pair is the whole contract: the spec is
// authoring data in MuJoCo's own units and frame, and the model is what MuJoCo
// made of it. Nothing in between is asserted on -- there is no second artifact
// to assert on.
//
// Angles are the one place that reads oddly. MJCF's default is degrees, and
// this spec authors no <compiler>, so an authored joint range is degrees
// while the compiled jnt_range is always radians.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjTestHelpers.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Gen/Elements/Actuators/MjActuator.gen.h"
#include "MuJoCo/Gen/Elements/Tendons/MjFixed.gen.h"
#include "MuJoCo/Gen/Elements/Tendons/MjFixedJoint.gen.h"
#include "MuJoCo/Gen/Elements/Sensors/MjJointpos.gen.h"
#include "MuJoCo/Gen/Elements/Keyframes/MjKey.gen.h"
#include "MuJoCo/Gen/Elements/Keyframes/MjKeyframe.gen.h"
#include "MuJoCo/Gen/Elements/MjModel.gen.h"
#include "MuJoCo/Gen/Elements/Actuators/MjMotor.gen.h"
#include "MuJoCo/Gen/Elements/Sensors/MjSensor.gen.h"
#include "MuJoCo/Gen/Elements/Geometry/MjSite.gen.h"
#include "MuJoCo/Gen/Elements/Tendons/MjTendon.gen.h"
#include "Engine/World.h"
#include "mujoco/mujoco.h"

// ============================================================================
// URLab.Compile.MinimalBody
//   Default FMjUESession (no callback).
//   Asserts that Manager is initialized and that the geom and joint elements
//   received compiled ids.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileMinimalBody,
	"URLab.Compile.MinimalBody",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileMinimalBody::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	TestTrue(TEXT("Manager should be initialized"), S.Manager->IsInitialized());
	TestTrue(TEXT("Geom's bound id should be >= 0"), S.Geom->GetBoundId().Get(-1) >= 0);
	TestTrue(TEXT("Joint's bound id should be >= 0"), S.Joint->GetBoundId().Get(-1) >= 0);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Compile.JointTypeHinge
//   ConfigCallback sets the joint type to hinge.
//   Asserts that the compiled model records mjJNT_HINGE for the joint.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileJointTypeHinge,
	"URLab.Compile.JointTypeHinge",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileJointTypeHinge::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init([](FMjUESession& Sess) {
			Sess.Joint->SetType(EMjJointType::hinge);
		}))
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	int JntId = S.MjId(mjOBJ_JOINT, TEXT("TestJoint"));
	if (TestTrue(TEXT("joint in compiled model"), JntId >= 0) && S.Manager->PhysicsEngine->m_model)
	{
		TestEqual(TEXT("jnt_type == HINGE"),
			(int)S.Manager->PhysicsEngine->m_model->jnt_type[JntId],
			(int)mjJNT_HINGE);
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Compile.JointTypeSlide
//   ConfigCallback sets the joint type to slide.
//   Asserts that the compiled model records mjJNT_SLIDE for the joint.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileJointTypeSlide,
	"URLab.Compile.JointTypeSlide",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileJointTypeSlide::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init([](FMjUESession& Sess) {
			Sess.Joint->SetType(EMjJointType::slide);
		}))
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	int JntId = S.MjId(mjOBJ_JOINT, TEXT("TestJoint"));
	if (TestTrue(TEXT("joint in compiled model"), JntId >= 0) && S.Manager->PhysicsEngine->m_model)
	{
		TestEqual(TEXT("jnt_type == SLIDE"),
			(int)S.Manager->PhysicsEngine->m_model->jnt_type[JntId],
			(int)mjJNT_SLIDE);
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Compile.JointTypeBall
//   ConfigCallback sets the joint type to ball.
//   Asserts that the compiled model records mjJNT_BALL for the joint.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileJointTypeBall,
	"URLab.Compile.JointTypeBall",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileJointTypeBall::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init([](FMjUESession& Sess) {
			Sess.Joint->SetType(EMjJointType::ball);
		}))
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	int JntId = S.MjId(mjOBJ_JOINT, TEXT("TestJoint"));
	if (TestTrue(TEXT("joint in compiled model"), JntId >= 0) && S.Manager->PhysicsEngine->m_model)
	{
		TestEqual(TEXT("jnt_type == BALL"),
			(int)S.Manager->PhysicsEngine->m_model->jnt_type[JntId],
			(int)mjJNT_BALL);
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Compile.JointTypeFree
//   ConfigCallback sets the joint type to free.
//   Asserts that the compiled model records mjJNT_FREE for the joint.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileJointTypeFree,
	"URLab.Compile.JointTypeFree",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileJointTypeFree::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init([](FMjUESession& Sess) {
			Sess.Joint->SetType(EMjJointType::free);
		}))
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	int JntId = S.MjId(mjOBJ_JOINT, TEXT("TestJoint"));
	if (TestTrue(TEXT("joint in compiled model"), JntId >= 0) && S.Manager->PhysicsEngine->m_model)
	{
		TestEqual(TEXT("jnt_type == FREE"),
			(int)S.Manager->PhysicsEngine->m_model->jnt_type[JntId],
			(int)mjJNT_FREE);
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Compile.JointAxisExported
//   ConfigCallback sets a hinge joint with Axis = (1, 0, 0).
//   Asserts that the compiled model records the same axis. The spec holds
//   MuJoCo's own frame, so an authored axis reaches jnt_axis unchanged; the
//   handedness fix belongs to the editor preview transform, not to attribute
//   values on their way to the compiler.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileJointAxisExported,
	"URLab.Compile.JointAxisExported",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileJointAxisExported::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init([](FMjUESession& Sess) {
			Sess.Joint->SetType(EMjJointType::hinge);
			Sess.Joint->SetAxis(FMjDirection3(1.0, 0.0, 0.0));
		}))
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	int JntId = S.MjId(mjOBJ_JOINT, TEXT("TestJoint"));
	if (TestTrue(TEXT("joint in compiled model"), JntId >= 0) && S.Manager->PhysicsEngine->m_model)
	{
		const mjtNum* Axis = &S.Manager->PhysicsEngine->m_model->jnt_axis[JntId * 3];
		TestTrue(TEXT("jnt_axis[0] ~= 1.0"), FMath::Abs((float)Axis[0] - 1.0f) < 1e-4f);
		TestTrue(TEXT("jnt_axis[1] ~= 0.0"), FMath::Abs((float)Axis[1]) < 1e-4f);
		TestTrue(TEXT("jnt_axis[2] ~= 0.0"), FMath::Abs((float)Axis[2]) < 1e-4f);
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Compile.GeomSizeExported
//   ConfigCallback sets geom Size to (0.3, 0.3, 0.3) in MuJoCo metres.
//   Asserts that geom_size[id*3+0] in the compiled model is approximately 0.3.
//
//   NOTE: UMjGeom's Size is already in MuJoCo metres — no scale conversion is
//   applied between the authored attribute and the emitted MJCF.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileGeomSizeExported,
	"URLab.Compile.GeomSizeExported",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileGeomSizeExported::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init([](FMjUESession& Sess) {
			Sess.Geom->SetSize({0.3, 0.3, 0.3});
		}))
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	int GeomId = S.MjId(mjOBJ_GEOM, TEXT("TestGeom"));
	if (TestTrue(TEXT("geom in compiled model"), GeomId >= 0) && S.Manager->PhysicsEngine->m_model)
	{
		TestTrue(TEXT("geom_size[0] ~= 0.3"),
			FMath::Abs((float)S.Manager->PhysicsEngine->m_model->geom_size[GeomId * 3 + 0] - 0.3f) < 1e-4f);
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Compile.GeomFrictionExported
//   ConfigCallback sets geom Friction to {0.7, 0.01, 0.001}.
//   Asserts that geom_friction[id*3+0] in the compiled model is approximately 0.7.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileGeomFrictionExported,
	"URLab.Compile.GeomFrictionExported",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileGeomFrictionExported::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init([](FMjUESession& Sess) {
			Sess.Geom->SetFriction({0.7, 0.01, 0.001});
		}))
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	int GeomId = S.MjId(mjOBJ_GEOM, TEXT("TestGeom"));
	if (TestTrue(TEXT("geom in compiled model"), GeomId >= 0) && S.Manager->PhysicsEngine->m_model)
	{
		TestTrue(TEXT("geom_friction[0] ~= 0.7"),
			FMath::Abs((float)S.Manager->PhysicsEngine->m_model->geom_friction[GeomId * 3 + 0] - 0.7f) < 1e-4f);
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Compile.JointDampingExported
//   ConfigCallback sets a hinge joint with Damping = 2.5.
//   Asserts that dof_damping at the joint's dof address is approximately 2.5.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileJointDampingExported,
	"URLab.Compile.JointDampingExported",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileJointDampingExported::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init([](FMjUESession& Sess) {
			Sess.Joint->SetType(EMjJointType::hinge);
			Sess.Joint->SetDamping({2.5});
		}))
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	int JntId = S.MjId(mjOBJ_JOINT, TEXT("TestJoint"));
	if (TestTrue(TEXT("joint in compiled model"), JntId >= 0) && S.Manager->PhysicsEngine->m_model)
	{
		int DofAdr = S.Manager->PhysicsEngine->m_model->jnt_dofadr[JntId];
		TestTrue(TEXT("dof_damping ~= 2.5"),
			FMath::Abs((float)S.Manager->PhysicsEngine->m_model->dof_damping[DofAdr] - 2.5f) < 1e-4f);
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Compile.JointLimitsExported
//   ConfigCallback sets a hinge joint with range [-1.5, 1.5] rad and limited.
//   Asserts that jnt_limited is set and jnt_range matches the specified values.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileJointLimitsExported,
	"URLab.Compile.JointLimitsExported",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileJointLimitsExported::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init([](FMjUESession& Sess) {
			Sess.Joint->SetType(EMjJointType::hinge);
			// The authored range is in the spec's angle unit, which with no
			// <compiler> element is MJCF's default of degrees. Express ±1.5 rad
			// as degrees: 1.5 * 180/π ≈ 85.943669.
			const double Rad15Deg = 1.5 * 180.0 / UE_DOUBLE_PI;
			Sess.Joint->SetRange(FVector2D(-Rad15Deg, Rad15Deg));
			Sess.Joint->SetLimited(EMjTriState::true_);
		}))
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	int JntId = S.MjId(mjOBJ_JOINT, TEXT("TestJoint"));
	if (TestTrue(TEXT("joint in compiled model"), JntId >= 0) && S.Manager->PhysicsEngine->m_model)
	{
		TestTrue(TEXT("jnt_limited should be set"),
			S.Manager->PhysicsEngine->m_model->jnt_limited[JntId] != 0);
		// jnt_range is always radians on the compiled model.
		TestTrue(TEXT("jnt_range[0] ~= -1.5"),
			FMath::Abs((float)S.Manager->PhysicsEngine->m_model->jnt_range[JntId * 2 + 0] - (-1.5f)) < 1e-4f);
		TestTrue(TEXT("jnt_range[1] ~= 1.5"),
			FMath::Abs((float)S.Manager->PhysicsEngine->m_model->jnt_range[JntId * 2 + 1] - 1.5f) < 1e-4f);
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Compile.ManagerModelValid
//   Default FMjUESession.
//   Asserts that m_model and m_data are non-null after compilation.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileManagerModelValid,
	"URLab.Compile.ManagerModelValid",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileManagerModelValid::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	TestNotNull(TEXT("Manager->PhysicsEngine->m_model should not be null"), S.Manager->PhysicsEngine->m_model);
	TestNotNull(TEXT("Manager->PhysicsEngine->m_data should not be null"), S.Manager->PhysicsEngine->m_data);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Compile.ManagerIsRunning
//   Default FMjUESession.
//   Asserts that Manager reports both IsInitialized() and IsRunning().
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileManagerIsRunning,
	"URLab.Compile.ManagerIsRunning",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileManagerIsRunning::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	TestTrue(TEXT("Manager should be initialized"), S.Manager->IsInitialized());
	// Sim starts paused by design — IsRunning() requires unpaused.
	// IsInitialized() is the correct check for post-compile readiness.
	TestTrue(TEXT("Manager should be initialized (physics)"), S.Manager->PhysicsEngine->IsInitialized());

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Compile.BodyCountMatchesComponents
//   Default FMjUESession (1 WorldBody + 1 user Body).
//   Asserts that nbody >= 2 (MuJoCo always includes a world body at index 0,
//   plus at least the one RootBody created by FMjUESession::Init).
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileBodyCountMatchesComponents,
	"URLab.Compile.BodyCountMatchesComponents",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileBodyCountMatchesComponents::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	TestTrue(TEXT("nbody should be >= 2 (world body + RootBody)"),
		S.Manager->PhysicsEngine->m_model != nullptr && S.Manager->PhysicsEngine->m_model->nbody >= 2);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Compile.GeomViewIdBound
//   Default FMjUESession.
//   Asserts that the geom element carries a compiled id after compilation.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileGeomViewIdBound,
	"URLab.Compile.GeomViewIdBound",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileGeomViewIdBound::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	TestTrue(TEXT("Geom's bound id should be >= 0"), S.Geom->GetBoundId().Get(-1) >= 0);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Compile.JointViewIdBound
//   Default FMjUESession.
//   Asserts that the joint element carries a compiled id after compilation.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileJointViewIdBound,
	"URLab.Compile.JointViewIdBound",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileJointViewIdBound::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	TestTrue(TEXT("Joint's bound id should be >= 0"), S.Joint->GetBoundId().Get(-1) >= 0);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Compile.SensorBinds
//   ConfigCallback adds a <jointpos> targeting "TestJoint" under the <sensor>
//   section, with the joint type set to hinge.
//   Asserts that the sensor bound and that nsensor == 1.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileSensorBinds,
	"URLab.Compile.SensorBinds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileSensorBinds::RunTest(const FString& Parameters)
{
	UMjJointpos* Sensor = nullptr;

	FMjUESession S;
	if (!S.Init([&Sensor](FMjUESession& Sess) {
			Sess.Joint->SetType(EMjJointType::hinge);

			UMjSensor* Section = Sess.Add<UMjSensor>(Sess.Robot->Spec);
			Sensor = Sess.Add<UMjJointpos>(Section, TEXT("TestSensor"));
			Sensor->Joint = TEXT("TestJoint");
		}))
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	if (TestNotNull(TEXT("Sensor pointer should be non-null"), Sensor))
	{
		TestTrue(TEXT("Sensor's bound id should be >= 0"), Sensor->GetBoundId().Get(-1) >= 0);
	}

	if (TestNotNull(TEXT("m_model should be non-null"), S.Manager->PhysicsEngine->m_model))
	{
		TestEqual(TEXT("nsensor should be 1"), (int)S.Manager->PhysicsEngine->m_model->nsensor, 1);
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Compile.ActuatorBinds
//   ConfigCallback adds a <motor> targeting "TestJoint" under the <actuator>
//   section, with the joint type set to hinge.
//   Asserts that the actuator bound and that nu == 1.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileActuatorBinds,
	"URLab.Compile.ActuatorBinds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileActuatorBinds::RunTest(const FString& Parameters)
{
	UMjMotor* Act = nullptr;

	FMjUESession S;
	if (!S.Init([&Act](FMjUESession& Sess) {
			Sess.Joint->SetType(EMjJointType::hinge);

			UMjActuator* Section = Sess.Add<UMjActuator>(Sess.Robot->Spec);
			Act = Sess.Add<UMjMotor>(Section, TEXT("TestActuator"));
			Act->SetJoint(TEXT("TestJoint"));
		}))
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	if (TestNotNull(TEXT("Actuator pointer should be non-null"), Act))
	{
		TestTrue(TEXT("Actuator's bound id should be >= 0"), Act->GetBoundId().Get(-1) >= 0);
	}

	if (TestNotNull(TEXT("m_model should be non-null"), S.Manager->PhysicsEngine->m_model))
	{
		TestEqual(TEXT("nu should be 1"), (int)S.Manager->PhysicsEngine->m_model->nu, 1);
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Compile.PIERestart
//   Runs two complete FMjUESession init/cleanup cycles back-to-back.
//   Both must succeed and Manager->IsInitialized() must be true in both,
//   verifying that the plugin can be re-initialised without leaving stale
//   state (simulates a PIE stop + restart).
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompilePIERestart,
	"URLab.Compile.PIERestart",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompilePIERestart::RunTest(const FString& Parameters)
{
	// --- First session ---
	{
		FMjUESession S;
		if (!S.Init())
		{
			AddError(FString::Printf(TEXT("First Init failed: %s"), *S.LastError));
			S.Cleanup();
			return false;
		}
		TestTrue(TEXT("First session: Manager should be initialized"), S.Manager->IsInitialized());
		S.Cleanup();
	}

	// --- Second session ---
	{
		FMjUESession S;
		if (!S.Init())
		{
			AddError(FString::Printf(TEXT("Second Init failed: %s"), *S.LastError));
			S.Cleanup();
			return false;
		}
		TestTrue(TEXT("Second session: Manager should be initialized"), S.Manager->IsInitialized());
		S.Cleanup();
	}

	return true;
}

// ============================================================================
// URLab.Compile.SiteBinds
//   ConfigCallback adds a <site> to the body.
//   Asserts that the site bound, that its id is the one its compiled name
//   resolves to, and that nsite >= 1.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileSiteBinds,
	"URLab.Compile.SiteBinds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileSiteBinds::RunTest(const FString& Parameters)
{
	UMjSite* Site = nullptr;

	FMjUESession S;
	if (!S.Init([&Site](FMjUESession& Sess) {
			Site = Sess.Add<UMjSite>(Sess.Body, TEXT("TestSite"));
		}))
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	if (TestNotNull(TEXT("Site pointer should be non-null"), Site))
	{
		TestTrue(TEXT("Site should be bound"), Site->GetBoundId().IsSet());
		TestTrue(TEXT("Site's bound id should be >= 0"), Site->GetBoundId().Get(-1) >= 0);
		TestEqual(TEXT("Site's bound id is the id its name resolves to"),
			Site->GetBoundId().Get(-1), S.MjId(mjOBJ_SITE, TEXT("TestSite")));
	}

	if (TestNotNull(TEXT("m_model should be non-null"), S.Manager->PhysicsEngine->m_model))
	{
		TestTrue(TEXT("nsite should be >= 1"), S.Manager->PhysicsEngine->m_model->nsite >= 1);
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Compile.TendonExportTo
//   Creates a fixed tendon with two joint wraps, stiffness and damping.
//   Verifies that the tendon compiles, binds, and its properties are in the
//   model.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileTendonExportTo,
	"URLab.Compile.TendonExportTo",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileTendonExportTo::RunTest(const FString& Parameters)
{
	UMjFixed* Tendon = nullptr;

	FMjUESession S;
	if (!S.Init([&Tendon](FMjUESession& Sess) {
			Sess.Joint->SetType(EMjJointType::hinge);

			// A second body + joint for the tendon to couple against.
			UMjBody* Body2 = Sess.Add<UMjBody>(Sess.Body, TEXT("Body2"));

			UMjGeom* Geom2 = Sess.Add<UMjGeom>(Body2, TEXT("Geom2"));
			Geom2->SetSize({0.1, 0.1, 0.1});

			UMjJoint* Joint2 = Sess.Add<UMjJoint>(Body2, TEXT("Joint2"));
			Joint2->SetType(EMjJointType::hinge);

			// <tendon><fixed> with two <joint> wraps.
			UMjTendon* Section = Sess.Add<UMjTendon>(Sess.Robot->Spec);
			Tendon = Sess.Add<UMjFixed>(Section, TEXT("TestTendon"));
			Tendon->SetStiffness({5.0});
			Tendon->SetDamping({0.1});

			UMjFixedJoint* W1 = Sess.Add<UMjFixedJoint>(Tendon);
			W1->Joint = TEXT("TestJoint");
			W1->SetCoef(1.0);

			UMjFixedJoint* W2 = Sess.Add<UMjFixedJoint>(Tendon);
			W2->Joint = TEXT("Joint2");
			W2->SetCoef(-1.0);
		}))
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	if (TestNotNull(TEXT("Tendon pointer"), Tendon) && TestNotNull(TEXT("m_model"), S.Manager->PhysicsEngine->m_model))
	{
		TestTrue(TEXT("ntendon >= 1"), S.Manager->PhysicsEngine->m_model->ntendon >= 1);
		TestTrue(TEXT("Tendon bound"), Tendon->GetBoundId().Get(-1) >= 0);

		int tid = S.MjId(mjOBJ_TENDON, TEXT("TestTendon"));
		if (tid >= 0)
		{
			TestTrue(TEXT("stiffness ~= 5.0"),
				FMath::Abs((float)S.Manager->PhysicsEngine->m_model->tendon_stiffness[tid] - 5.0f) < 1e-4f);
			TestTrue(TEXT("damping ~= 0.1"),
				FMath::Abs((float)S.Manager->PhysicsEngine->m_model->tendon_damping[tid] - 0.1f) < 1e-4f);
		}
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Compile.DefaultTendonSiteCamera
//   A <default> class carrying tendon, site and camera settings has to reach
//   the elements that name it. There is no delegation step to check any more --
//   the class is an element with children and MuJoCo resolves the inheritance --
//   so the claim is stated end to end: read the spec, emit it, compile it,
//   and read the three families' values back out of the model.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileDefaultTendonSiteCamera,
	"URLab.Compile.DefaultTendonSiteCamera",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileDefaultTendonSiteCamera::RunTest(const FString& Parameters)
{
	static const TCHAR* Xml = TEXT(R"(
        <mujoco>
          <default>
            <default class="kit">
              <tendon stiffness="7.5" damping="0.3"/>
              <site type="box" group="3" size="0.01 0.01 0.01"/>
              <camera fovy="90" resolution="1280 720"/>
            </default>
          </default>
          <worldbody>
            <body name="b1">
              <geom type="capsule" size="0.05 0.5" mass="1"/>
              <joint name="j1" type="hinge"/>
              <site name="s1" class="kit"/>
              <camera name="c1" class="kit"/>
              <body name="b2" pos="0 0 -1">
                <geom type="capsule" size="0.05 0.5" mass="1"/>
                <joint name="j2" type="hinge"/>
              </body>
            </body>
          </worldbody>
          <tendon>
            <fixed name="t1" class="kit">
              <joint joint="j1" coef="1"/>
              <joint joint="j2" coef="-1"/>
            </fixed>
          </tendon>
        </mujoco>
    )");

	FMjXmlImportSession S;
	if (!S.Init(Xml))
	{
		AddError(S.LastError);
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}

	const mjModel* M = S.Model();
	if (!TestNotNull(TEXT("compiled model"), (void*)M))
	{
		S.Cleanup();
		return false;
	}

	if (TestEqual(TEXT("one tendon"), (int)M->ntendon, 1))
	{
		TestTrue(TEXT("tendon stiffness ~= 7.5"),
			FMath::Abs((float)M->tendon_stiffness[0] - 7.5f) < 1e-4f);
		TestTrue(TEXT("tendon damping ~= 0.3"),
			FMath::Abs((float)M->tendon_damping[0] - 0.3f) < 1e-4f);
	}

	if (TestEqual(TEXT("one site"), (int)M->nsite, 1))
	{
		TestEqual(TEXT("site type == box"), (int)M->site_type[0], (int)mjGEOM_BOX);
		TestEqual(TEXT("site group == 3"), (int)M->site_group[0], 3);
	}

	if (TestEqual(TEXT("one camera"), (int)M->ncam, 1))
	{
		TestTrue(TEXT("camera fovy ~= 90.0"), FMath::Abs((float)M->cam_fovy[0] - 90.0f) < 1e-4f);
		TestEqual(TEXT("camera resolution width == 1280"), (int)M->cam_resolution[0], 1280);
		TestEqual(TEXT("camera resolution height == 720"), (int)M->cam_resolution[1], 720);
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Compile.EqualityExportTo
//   A weld equality between two bodies, compiled twice: once by stock MuJoCo
//   and once through the spec. Both must produce the same constraint, and
//   the torquescale must land in eq_data slot 10 -- an older codegen wrote slot
//   7, and only a value read out of the compiled model can tell the difference.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileEqualityExportTo,
	"URLab.Compile.EqualityExportTo",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileEqualityExportTo::RunTest(const FString& Parameters)
{
	static const TCHAR* Xml = TEXT(
		"<mujoco>"
		"  <worldbody>"
		"    <body name='b1'>"
		"      <geom size='0.1'/>"
		"      <joint name='j1' type='hinge'/>"
		"      <body name='b2'>"
		"        <geom size='0.1'/>"
		"        <joint name='j2' type='hinge'/>"
		"      </body>"
		"    </body>"
		"  </worldbody>"
		"  <equality>"
		"    <weld body1='b1' body2='b2' active='true' torquescale='2'/>"
		"  </equality>"
		"</mujoco>");

	FMjTestSession Ref;
	if (!Ref.CompileXml(Xml))
	{
		AddError(FString::Printf(TEXT("Compile failed: %s"), *Ref.LastError));
		return false;
	}
	TestTrue(TEXT("neq >= 1"), Ref.m->neq >= 1);

	FMjXmlImportSession S;
	if (!S.Init(Xml))
	{
		AddError(S.LastError);
		Ref.Cleanup();
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		Ref.Cleanup();
		return false;
	}

	const mjModel* M = S.Model();
	if (!TestEqual(TEXT("the spec produced the same constraint count"), (int)M->neq, (int)Ref.m->neq)
		|| M->neq < 1)
	{
		S.Cleanup();
		Ref.Cleanup();
		return false;
	}

	TestEqual(TEXT("type == weld"), (int)M->eq_type[0], (int)mjEQ_WELD);
	TestEqual(TEXT("type matches the native compile"), (int)M->eq_type[0], (int)Ref.m->eq_type[0]);
	TestEqual(TEXT("active == 1"), (int)M->eq_active0[0], 1);
	TestEqual(TEXT("active matches the native compile"),
		(int)M->eq_active0[0], (int)Ref.m->eq_active0[0]);

	// eq_data layout for weld: data[0..2] anchor, data[3..9] relpose,
	// data[10] torquescale.
	TestTrue(TEXT("eq_data[10] ~= 2.0 (torquescale)"),
		FMath::Abs((float)M->eq_data[10] - 2.0f) < 1e-4f);
	TestTrue(TEXT("eq_data[10] matches the native compile"),
		FMath::Abs((float)(M->eq_data[10] - Ref.m->eq_data[10])) < 1e-4f);

	S.Cleanup();
	Ref.Cleanup();
	return true;
}

// ============================================================================
// URLab.Compile.KeyframeExportTo
//   Creates a keyframe with qpos and time values.
//   Verifies that the keyframe compiles and nkey >= 1, time is correct.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileKeyframeExportTo,
	"URLab.Compile.KeyframeExportTo",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileKeyframeExportTo::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init([](FMjUESession& Sess) {
			Sess.Joint->SetType(EMjJointType::hinge);

			UMjKeyframe* Section = Sess.Add<UMjKeyframe>(Sess.Robot->Spec);
			UMjKey* KF = Sess.Add<UMjKey>(Section, TEXT("TestKeyframe"));
			KF->SetTime(1.5);
			KF->SetQpos({0.5});
		}))
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	if (TestNotNull(TEXT("m_model"), S.Manager->PhysicsEngine->m_model))
	{
		TestTrue(TEXT("nkey >= 1"), S.Manager->PhysicsEngine->m_model->nkey >= 1);

		if (S.Manager->PhysicsEngine->m_model->nkey >= 1)
		{
			TestTrue(TEXT("key_time[0] ~= 1.5"),
				FMath::Abs((float)S.Manager->PhysicsEngine->m_model->key_time[0] - 1.5f) < 1e-4f);
		}
	}

	S.Cleanup();
	return true;
}
