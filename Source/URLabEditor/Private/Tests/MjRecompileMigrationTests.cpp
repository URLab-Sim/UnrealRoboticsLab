// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Simulation state across a structural edit.
//
// A recompile is a new mjModel, and a new mjModel is new addresses: the joint
// that gained a sibling is no longer at the qpos slot it was at. So the state
// cannot follow the address, and the interesting question is whether it follows
// the element instead.
//
// `InstallCompiledSpec` stashes qpos, qvel, ctrl, act, mocap and time out of the
// old `mjData` before the old model is freed, and writes them back into the new
// one by element identity once it exists (`MjPhysicsEngine.cpp`, `StashState` /
// `RestoreState`). An element whose width does not match between the two models
// is left at whatever `mj_resetData` already put there -- the new model's own
// qpos0 and zero velocity -- because there is no meaning in copying a hinge's
// one number into a ball joint's four.
//
// Two families of case, and the file is the acceptance instrument for both.
// First, identity: delete a joint, and the joint that inherits its qpos slot
// must hold ITS OWN value, not the deleted one's -- the case an address-keyed
// migration gets wrong and is seen to get wrong. Second, completeness: a
// running scene at step 100, with every carried quantity nonzero, must come
// through an edit untouched, and an edit that changes one joint's shape must
// disturb that joint and nothing else.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#if WITH_EDITOR

#include "MjTestHelpers.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Gen/Elements/Actuators/MjActuator.gen.h"
#include "MuJoCo/Gen/Elements/Actuators/MjActuatorGeneral.gen.h"
#include "MuJoCo/Gen/Elements/Actuators/MjMotor.gen.h"
#include "MuJoCo/Gen/Elements/Joints/MjJoint.gen.h"
#include "MuJoCo/Gen/MjEnums.gen.h"
#include "MuJoCo/Spec/MjFrameTypes.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"

namespace MjRecompileMigrationTests
{
// --- Direct mjData access, by element name --------------------------------- //
//
// The staged control slots (`UMjActuatorRuntime`) resolve into `d->ctrl` on the
// physics worker's own thread; these tests drive `mj_step` directly through
// `FMjUESession::Step`, so they read and write `mjData` directly too.

/** The value in `Data` at the qpos address of the joint named `Name`. */
bool QPosOf(const FMjUESession& Session, const TCHAR* Name, double& Out)
{
	const mjModel* M = Session.Model();
	const mjData* D = Session.Data();
	const int Id = Session.MjId(mjOBJ_JOINT, Name);
	if (M == nullptr || D == nullptr || Id < 0)
	{
		return false;
	}
	Out = D->qpos[M->jnt_qposadr[Id]];
	return true;
}

/** Write `Value` at the qpos address of the joint named `Name`. */
bool SetQPosOf(const FMjUESession& Session, const TCHAR* Name, double Value)
{
	const mjModel* M = Session.Model();
	mjData* D = Session.Data();
	const int Id = Session.MjId(mjOBJ_JOINT, Name);
	if (M == nullptr || D == nullptr || Id < 0)
	{
		return false;
	}
	D->qpos[M->jnt_qposadr[Id]] = Value;
	return true;
}

/** The value in `Data` at the qvel address of the joint named `Name`. */
bool QVelOf(const FMjUESession& Session, const TCHAR* Name, double& Out)
{
	const mjModel* M = Session.Model();
	const mjData* D = Session.Data();
	const int Id = Session.MjId(mjOBJ_JOINT, Name);
	if (M == nullptr || D == nullptr || Id < 0)
	{
		return false;
	}
	Out = D->qvel[M->jnt_dofadr[Id]];
	return true;
}

/** Write `Value` at the qvel address of the joint named `Name`. */
bool SetQVelOf(const FMjUESession& Session, const TCHAR* Name, double Value)
{
	const mjModel* M = Session.Model();
	mjData* D = Session.Data();
	const int Id = Session.MjId(mjOBJ_JOINT, Name);
	if (M == nullptr || D == nullptr || Id < 0)
	{
		return false;
	}
	D->qvel[M->jnt_dofadr[Id]] = Value;
	return true;
}

/** The control value staged for the actuator named `Name`. */
bool CtrlOf(const FMjUESession& Session, const TCHAR* Name, double& Out)
{
	const mjData* D = Session.Data();
	const int Id = Session.MjId(mjOBJ_ACTUATOR, Name);
	if (D == nullptr || Id < 0)
	{
		return false;
	}
	Out = D->ctrl[Id];
	return true;
}

/** Stage `Value` as the control of the actuator named `Name`. */
bool SetCtrlOf(const FMjUESession& Session, const TCHAR* Name, double Value)
{
	mjData* D = Session.Data();
	const int Id = Session.MjId(mjOBJ_ACTUATOR, Name);
	if (D == nullptr || Id < 0)
	{
		return false;
	}
	D->ctrl[Id] = Value;
	return true;
}

/** The first (and here, only) activation slot of an actuator that carries one. */
bool ActOf(const FMjUESession& Session, const TCHAR* Name, double& Out)
{
	const mjModel* M = Session.Model();
	const mjData* D = Session.Data();
	const int Id = Session.MjId(mjOBJ_ACTUATOR, Name);
	if (M == nullptr || D == nullptr || Id < 0 || M->actuator_actnum[Id] < 1)
	{
		return false;
	}
	Out = D->act[M->actuator_actadr[Id]];
	return true;
}

/** Write `Value` into the first activation slot of the actuator named `Name`. */
bool SetActOf(const FMjUESession& Session, const TCHAR* Name, double Value)
{
	const mjModel* M = Session.Model();
	mjData* D = Session.Data();
	const int Id = Session.MjId(mjOBJ_ACTUATOR, Name);
	if (M == nullptr || D == nullptr || Id < 0 || M->actuator_actnum[Id] < 1)
	{
		return false;
	}
	D->act[M->actuator_actadr[Id]] = Value;
	return true;
}

/** The mocap pose of the body named `Name`. */
bool MocapOf(const FMjUESession& Session, const TCHAR* Name, double OutPos[3], double OutQuat[4])
{
	const mjModel* M = Session.Model();
	const mjData* D = Session.Data();
	const int BodyId = Session.MjId(mjOBJ_BODY, Name);
	if (M == nullptr || D == nullptr || BodyId < 0 || M->body_mocapid[BodyId] < 0)
	{
		return false;
	}
	const int MocapId = M->body_mocapid[BodyId];
	FMemory::Memcpy(OutPos, D->mocap_pos + 3 * MocapId, 3 * sizeof(double));
	FMemory::Memcpy(OutQuat, D->mocap_quat + 4 * MocapId, 4 * sizeof(double));
	return true;
}

/** Write the mocap pose of the body named `Name`. */
bool SetMocapOf(const FMjUESession& Session, const TCHAR* Name, const double Pos[3], const double Quat[4])
{
	const mjModel* M = Session.Model();
	mjData* D = Session.Data();
	const int BodyId = Session.MjId(mjOBJ_BODY, Name);
	if (M == nullptr || D == nullptr || BodyId < 0 || M->body_mocapid[BodyId] < 0)
	{
		return false;
	}
	const int MocapId = M->body_mocapid[BodyId];
	FMemory::Memcpy(D->mocap_pos + 3 * MocapId, Pos, 3 * sizeof(double));
	FMemory::Memcpy(D->mocap_quat + 4 * MocapId, Quat, 4 * sizeof(double));
	return true;
}

/** A body with one hinge, both named, hung under the session's world body. */
UMjBody* AddHingeBody(FMjUESession& Session, const TCHAR* BodyName, const TCHAR* JointName)
{
	UMjBody* NewBody = Session.Add<UMjBody>(Session.WorldBody, BodyName);
	if (NewBody == nullptr)
	{
		return nullptr;
	}
	// A body with no geom has no mass, and MuJoCo will not compile a moving
	// body without inertia, so every body authored here carries one.
	if (UMjGeom* NewGeom = Session.Add<UMjGeom>(NewBody))
	{
		NewGeom->SetSize({0.1});
	}
	Session.Add<UMjJoint>(NewBody, JointName);
	return NewBody;
}

// --- The carried-state fixture --------------------------------------------- //

/**
 * Everything an in-session edit has to carry, in one articulation.
 *
 * The session's own `RootBody` / `TestJoint`, driven by a motor so a control
 * value is carried; `KeptJoint`, never edited; `ActJoint`, driven by a
 * `<general>` with `dyntype="integrator"` so an activation state is carried;
 * `ChangedJoint`, which only the joint-type case touches; and `MocapBody`, a
 * mocap body with no dofs at all. Every body gets its own offset so their geoms
 * do not touch: a contact would make the recorded state a function of the
 * solver rather than of the edit under test.
 */
struct FFixture
{
	UMjMotor* TestMotor = nullptr;
	UMjJoint* KeptJoint = nullptr;
	UMjJoint* ActJoint = nullptr;
	UMjActuatorGeneral* ActGeneral = nullptr;
	UMjJoint* ChangedJoint = nullptr;
};

/** A body with a geom, offset so nothing in the fixture touches anything else. */
UMjBody* AddOffsetBody(FMjUESession& S, const TCHAR* Name, const FMjPosition3& Pos, double Size)
{
	UMjBody* const NewBody = S.Add<UMjBody>(S.WorldBody, Name);
	if (NewBody == nullptr)
	{
		return nullptr;
	}
	NewBody->SetPos(Pos);
	if (UMjGeom* const Geom = S.Add<UMjGeom>(NewBody))
	{
		Geom->SetSize({Size});
	}
	return NewBody;
}

void BuildFixture(FMjUESession& S, FFixture& Fx)
{
	UMjActuator* const ActuatorSection = S.Add<UMjActuator>(S.Robot->Spec);
	if (ActuatorSection == nullptr)
	{
		return;
	}

	Fx.TestMotor = S.Add<UMjMotor>(ActuatorSection, TEXT("TestMotor"));
	if (Fx.TestMotor != nullptr)
	{
		Fx.TestMotor->SetJoint(TEXT("TestJoint"));
	}

	if (UMjBody* const KeptBody = AddOffsetBody(S, TEXT("KeptBody"), FMjPosition3(0.5, 0.0, 0.0), 0.1))
	{
		Fx.KeptJoint = S.Add<UMjJoint>(KeptBody, TEXT("KeptJoint"));
	}

	if (UMjBody* const ActBody = AddOffsetBody(S, TEXT("ActBody"), FMjPosition3(1.0, 0.0, 0.0), 0.1))
	{
		Fx.ActJoint = S.Add<UMjJoint>(ActBody, TEXT("ActJoint"));
	}
	Fx.ActGeneral = S.Add<UMjActuatorGeneral>(ActuatorSection, TEXT("ActGeneral"));
	if (Fx.ActGeneral != nullptr)
	{
		Fx.ActGeneral->SetJoint(TEXT("ActJoint"));
		// An integrator activation is what makes `act` a quantity worth carrying:
		// a plain motor has none, so nothing would be asserted about it.
		Fx.ActGeneral->SetDyntype(EMjDynType::integrator);
	}

	if (UMjBody* const ChangedBody =
			AddOffsetBody(S, TEXT("ChangedBody"), FMjPosition3(1.5, 0.0, 0.0), 0.1))
	{
		Fx.ChangedJoint = S.Add<UMjJoint>(ChangedBody, TEXT("ChangedJoint"));
		if (Fx.ChangedJoint != nullptr)
		{
			Fx.ChangedJoint->SetType(EMjJointType::hinge);
		}
	}

	if (UMjBody* const MocapBody =
			AddOffsetBody(S, TEXT("MocapBody"), FMjPosition3(0.0, 0.0, 1.0), 0.05))
	{
		MocapBody->SetMocap(true);
	}
}

/** Every number the recompile is supposed to carry forward, in one snapshot. */
struct FSnapshot
{
	double Time = 0.0;
	double TestJointQPos = 0.0;
	double TestJointQVel = 0.0;
	double KeptJointQPos = 0.0;
	double KeptJointQVel = 0.0;
	double ActJointQPos = 0.0;
	double ActJointQVel = 0.0;
	double ChangedJointQPos = 0.0;
	double ChangedJointQVel = 0.0;
	double TestMotorCtrl = 0.0;
	double ActGeneralCtrl = 0.0;
	double ActGeneralAct = 0.0;
	double MocapPos[3] = {0.0, 0.0, 0.0};
	double MocapQuat[4] = {1.0, 0.0, 0.0, 0.0};
};

FSnapshot Capture(const FMjUESession& S)
{
	FSnapshot Out;
	Out.Time = S.Data() != nullptr ? S.Data()->time : 0.0;
	QPosOf(S, TEXT("TestJoint"), Out.TestJointQPos);
	QVelOf(S, TEXT("TestJoint"), Out.TestJointQVel);
	QPosOf(S, TEXT("KeptJoint"), Out.KeptJointQPos);
	QVelOf(S, TEXT("KeptJoint"), Out.KeptJointQVel);
	QPosOf(S, TEXT("ActJoint"), Out.ActJointQPos);
	QVelOf(S, TEXT("ActJoint"), Out.ActJointQVel);
	QPosOf(S, TEXT("ChangedJoint"), Out.ChangedJointQPos);
	QVelOf(S, TEXT("ChangedJoint"), Out.ChangedJointQVel);
	CtrlOf(S, TEXT("TestMotor"), Out.TestMotorCtrl);
	CtrlOf(S, TEXT("ActGeneral"), Out.ActGeneralCtrl);
	ActOf(S, TEXT("ActGeneral"), Out.ActGeneralAct);
	MocapOf(S, TEXT("MocapBody"), Out.MocapPos, Out.MocapQuat);
	return Out;
}

/** Distinct, nonzero values, so a reset to defaults could not pass by accident. */
bool Seed(FAutomationTestBase& Test, const FMjUESession& S)
{
	const bool bStaged = SetQPosOf(S, TEXT("TestJoint"), 0.11) && SetQVelOf(S, TEXT("TestJoint"), 0.05)
		&& SetQPosOf(S, TEXT("KeptJoint"), 0.22) && SetQVelOf(S, TEXT("KeptJoint"), 0.06)
		&& SetQPosOf(S, TEXT("ActJoint"), 0.33) && SetQVelOf(S, TEXT("ActJoint"), 0.07)
		&& SetQPosOf(S, TEXT("ChangedJoint"), 0.44) && SetQVelOf(S, TEXT("ChangedJoint"), 0.08)
		&& SetCtrlOf(S, TEXT("TestMotor"), 0.3) && SetCtrlOf(S, TEXT("ActGeneral"), 0.4)
		&& SetActOf(S, TEXT("ActGeneral"), 0.15);

	const double Pos[3] = {1.0, 2.0, 3.0};
	const double Quat[4] = {0.5, 0.5, 0.5, 0.5};  // unit: 0.5^2 * 4 == 1.
	return Test.TestTrue(TEXT("every carried quantity was staged"),
		bStaged && SetMocapOf(S, TEXT("MocapBody"), Pos, Quat));
}

/** Every carried quantity but `ChangedJoint`'s: the edits never touch these. */
void ExpectCarried(FAutomationTestBase& Test, const FSnapshot& Before, const FSnapshot& After)
{
	constexpr double Eps = UE_DOUBLE_KINDA_SMALL_NUMBER;
	Test.TestEqual(TEXT("time continues across the recompile"), After.Time, Before.Time, Eps);
	Test.TestEqual(TEXT("TestJoint qpos carried"), After.TestJointQPos, Before.TestJointQPos, Eps);
	Test.TestEqual(TEXT("TestJoint qvel carried"), After.TestJointQVel, Before.TestJointQVel, Eps);
	Test.TestEqual(TEXT("KeptJoint qpos carried"), After.KeptJointQPos, Before.KeptJointQPos, Eps);
	Test.TestEqual(TEXT("KeptJoint qvel carried"), After.KeptJointQVel, Before.KeptJointQVel, Eps);
	Test.TestEqual(TEXT("ActJoint qpos carried"), After.ActJointQPos, Before.ActJointQPos, Eps);
	Test.TestEqual(TEXT("ActJoint qvel carried"), After.ActJointQVel, Before.ActJointQVel, Eps);
	Test.TestEqual(TEXT("TestMotor ctrl carried"), After.TestMotorCtrl, Before.TestMotorCtrl, Eps);
	Test.TestEqual(TEXT("ActGeneral ctrl carried"), After.ActGeneralCtrl, Before.ActGeneralCtrl, Eps);
	Test.TestEqual(TEXT("ActGeneral act carried"), After.ActGeneralAct, Before.ActGeneralAct, Eps);
	for (int32 Index = 0; Index < 3; ++Index)
	{
		Test.TestEqual(*FString::Printf(TEXT("mocap pos[%d] carried"), Index), After.MocapPos[Index],
			Before.MocapPos[Index], Eps);
	}
	for (int32 Index = 0; Index < 4; ++Index)
	{
		Test.TestEqual(*FString::Printf(TEXT("mocap quat[%d] carried"), Index), After.MocapQuat[Index],
			Before.MocapQuat[Index], Eps);
	}
}

/** A session at step 100 with every carried quantity nonzero, ready to edit. */
bool RunToStep100(FAutomationTestBase& Test, FMjUESession& S, FFixture& Fx, FSnapshot& OutBefore)
{
	if (!S.Init([&Fx](FMjUESession& Session) { BuildFixture(Session, Fx); }))
	{
		Test.AddError(S.LastError);
		return false;
	}
	if (!Seed(Test, S))
	{
		return false;
	}
	S.Step(100);
	OutBefore = Capture(S);

	// The state has to be worth carrying, or every assertion below passes on
	// zeroes that a total reset would also produce.
	return Test.TestTrue(TEXT("the stepped state is nonzero"),
		FMath::Abs(OutBefore.TestJointQPos) > 0.01 && FMath::Abs(OutBefore.ChangedJointQPos) > 0.01
			&& FMath::Abs(OutBefore.ActGeneralAct) > UE_DOUBLE_KINDA_SMALL_NUMBER
			&& FMath::Abs(OutBefore.MocapPos[0]) > UE_DOUBLE_KINDA_SMALL_NUMBER);
}
}  // namespace MjRecompileMigrationTests

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRecompileMigratesStateByElement,
	"URLab.Doc.RecompileMigratesStateByElement",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRecompileMigratesStateByElement::RunTest(const FString& Parameters)
{
	using namespace MjRecompileMigrationTests;

	// Three hinges beyond the session's own, so there is a qpos slot for a
	// deleted joint to vacate and a survivor to inherit it.
	UMjBody* Doomed = nullptr;
	FMjUESession Session;
	if (!Session.Init([&](FMjUESession& S) {
			Doomed = AddHingeBody(S, TEXT("DoomedBody"), TEXT("DoomedJoint"));
			AddHingeBody(S, TEXT("KeptBody"), TEXT("KeptJoint"));
		}))
	{
		AddError(Session.LastError);
		return false;
	}
	if (!TestNotNull(TEXT("the body to delete"), Doomed))
	{
		return false;
	}

	// Distinct values, so an assertion cannot pass on the wrong one.
	constexpr double TestJointPose = 0.11;
	constexpr double DoomedPose = 0.22;
	constexpr double KeptPose = 0.33;
	if (!TestTrue(TEXT("staged the session joint"), SetQPosOf(Session, TEXT("TestJoint"), TestJointPose))
		|| !TestTrue(TEXT("staged the doomed joint"), SetQPosOf(Session, TEXT("DoomedJoint"), DoomedPose))
		|| !TestTrue(TEXT("staged the kept joint"), SetQPosOf(Session, TEXT("KeptJoint"), KeptPose)))
	{
		return false;
	}

	// The address the deleted joint is about to vacate, remembered before it
	// stops existing: the whole point is what ends up here afterwards.
	const int DoomedId = Session.MjId(mjOBJ_JOINT, TEXT("DoomedJoint"));
	if (!TestTrue(TEXT("the doomed joint compiled"), DoomedId >= 0))
	{
		return false;
	}
	const int VacatedQPosAdr = Session.Model()->jnt_qposadr[DoomedId];
	Session.Data()->time = 1.5;

	// The structural edit: one body leaves, one arrives.
	if (!TestTrue(TEXT("removed the doomed body"),
			urlab::spec::FMjInstanceAdapter::Remove(*Session.WorldBody, Doomed)))
	{
		return false;
	}
	if (!TestNotNull(TEXT("the added body"),
			AddHingeBody(Session, TEXT("AddedBody"), TEXT("AddedJoint"))))
	{
		return false;
	}

	if (!Session.Recompile())
	{
		AddError(FString::Printf(TEXT("recompile failed: %s"), *Session.LastError));
		return false;
	}

	// The deleted element is gone from the model, not merely unbound.
	TestEqual(TEXT("the deleted joint is not in the recompiled model"),
		Session.MjId(mjOBJ_JOINT, TEXT("DoomedJoint")), -1);

	double Value = 0.0;
	if (TestTrue(TEXT("the session joint survived"), QPosOf(Session, TEXT("TestJoint"), Value)))
	{
		TestEqual(TEXT("and kept its pose"), Value, TestJointPose, UE_DOUBLE_KINDA_SMALL_NUMBER);
	}
	if (TestTrue(TEXT("the kept joint survived"), QPosOf(Session, TEXT("KeptJoint"), Value)))
	{
		TestEqual(TEXT("and kept its own pose"), Value, KeptPose, UE_DOUBLE_KINDA_SMALL_NUMBER);
		// The assertion the serial keying exists for. Migrating by address would
		// put the deleted joint's 0.22 here.
		TestNotEqual(TEXT("not the deleted joint's"), Value, DoomedPose);
	}
	if (TestTrue(TEXT("the added joint compiled"), QPosOf(Session, TEXT("AddedJoint"), Value)))
	{
		TestEqual(TEXT("and starts at the model's own default"), Value, 0.0,
			UE_DOUBLE_KINDA_SMALL_NUMBER);
	}

	// The slot the deleted joint vacated now belongs to whichever joint the
	// compiler put there, and it must hold that joint's state rather than the
	// bytes that were at the address before.
	const mjModel* M = Session.Model();
	if (TestTrue(TEXT("the vacated address is still in range"), VacatedQPosAdr < M->nq))
	{
		TestNotEqual(TEXT("the vacated qpos slot does not hold the deleted pose"),
			Session.Data()->qpos[VacatedQPosAdr], DoomedPose);
	}

	TestEqual(TEXT("time continues across the edit"), Session.Data()->time, 1.5,
		UE_DOUBLE_KINDA_SMALL_NUMBER);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRecompileStartsAFreshSceneAtDefaults,
	"URLab.Doc.RecompileStartsAFreshSceneAtDefaults",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRecompileStartsAFreshSceneAtDefaults::RunTest(const FString& Parameters)
{
	using namespace MjRecompileMigrationTests;

	// Migration must not turn the first compile into a special case: with no
	// previous model there is no state, and the scene starts where the model
	// says it does.
	FMjUESession Session;
	if (!Session.Init())
	{
		AddError(Session.LastError);
		return false;
	}

	double Value = 0.0;
	if (TestTrue(TEXT("the joint compiled"), QPosOf(Session, TEXT("TestJoint"), Value)))
	{
		TestEqual(TEXT("a first compile starts at qpos0"), Value, 0.0,
			UE_DOUBLE_KINDA_SMALL_NUMBER);
	}
	TestEqual(TEXT("and at time zero"), Session.Data()->time, 0.0,
		UE_DOUBLE_KINDA_SMALL_NUMBER);
	return true;
}

// ============================================================================
// URLab.Doc.RecompileCarriesStateThroughANoOpEdit
//   A recompile with no spec edit at all must not move a single number: same
//   elements, same widths, so every element's state is written straight back to
//   its own unchanged address.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRecompileCarriesStateThroughANoOpEdit,
	"URLab.Doc.RecompileCarriesStateThroughANoOpEdit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRecompileCarriesStateThroughANoOpEdit::RunTest(const FString& Parameters)
{
	using namespace MjRecompileMigrationTests;

	FFixture Fx;
	FMjUESession S;
	FSnapshot Before;
	if (!RunToStep100(*this, S, Fx, Before))
	{
		return false;
	}

	if (!S.Recompile())
	{
		AddError(FString::Printf(TEXT("recompile failed: %s"), *S.LastError));
		return false;
	}

	const FSnapshot After = Capture(S);
	ExpectCarried(*this, Before, After);
	TestEqual(TEXT("ChangedJoint qpos carried (this edit does not touch it)"),
		After.ChangedJointQPos, Before.ChangedJointQPos, UE_DOUBLE_KINDA_SMALL_NUMBER);
	TestEqual(TEXT("ChangedJoint qvel carried (this edit does not touch it)"),
		After.ChangedJointQVel, Before.ChangedJointQVel, UE_DOUBLE_KINDA_SMALL_NUMBER);
	return true;
}

// ============================================================================
// URLab.Doc.RecompileCarriesStateThroughAValueEdit
//   A geom's size is part of no element's dof or activation count, so editing
//   one and recompiling carries every joint, actuator and mocap value forward
//   exactly -- and the edit is asserted to have reached the model, so the test
//   cannot pass by having changed nothing.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRecompileCarriesStateThroughAValueEdit,
	"URLab.Doc.RecompileCarriesStateThroughAValueEdit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRecompileCarriesStateThroughAValueEdit::RunTest(const FString& Parameters)
{
	using namespace MjRecompileMigrationTests;

	FFixture Fx;
	FMjUESession S;
	FSnapshot Before;
	if (!RunToStep100(*this, S, Fx, Before))
	{
		return false;
	}

	S.Geom->SetSize({0.25});

	if (!S.Recompile())
	{
		AddError(FString::Printf(TEXT("recompile failed: %s"), *S.LastError));
		return false;
	}

	const int32 GeomId = S.MjId(mjOBJ_GEOM, TEXT("TestGeom"));
	if (TestTrue(TEXT("TestGeom compiled"), GeomId >= 0))
	{
		TestEqual(TEXT("the geom size edit reached the model"), S.Model()->geom_size[3 * GeomId], 0.25,
			UE_DOUBLE_KINDA_SMALL_NUMBER);
	}

	const FSnapshot After = Capture(S);
	ExpectCarried(*this, Before, After);
	TestEqual(TEXT("ChangedJoint qpos carried (this edit does not touch it)"),
		After.ChangedJointQPos, Before.ChangedJointQPos, UE_DOUBLE_KINDA_SMALL_NUMBER);
	TestEqual(TEXT("ChangedJoint qvel carried (this edit does not touch it)"),
		After.ChangedJointQVel, Before.ChangedJointQVel, UE_DOUBLE_KINDA_SMALL_NUMBER);
	return true;
}

// ============================================================================
// URLab.Doc.RecompileJointTypeChangeResetsOnlyThatJoint
//   Flipping one hinge to a ball takes its qpos width from one to four, so the
//   migration cannot carry its old number into the new slots and leaves that
//   joint at the new model's own qpos0 (a unit quaternion, not zero) and zero
//   velocity. Exactly that joint: every other joint, both actuators and the
//   mocap body come through untouched. This is the width guard
//   (`MjPhysicsEngine.cpp`, RestoreState) seen from the outside.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRecompileJointTypeChangeResetsOnlyThatJoint,
	"URLab.Doc.RecompileJointTypeChangeResetsOnlyThatJoint",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRecompileJointTypeChangeResetsOnlyThatJoint::RunTest(const FString& Parameters)
{
	using namespace MjRecompileMigrationTests;

	FFixture Fx;
	FMjUESession S;
	FSnapshot Before;
	if (!RunToStep100(*this, S, Fx, Before))
	{
		return false;
	}
	if (!TestNotNull(TEXT("the joint to change"), Fx.ChangedJoint))
	{
		return false;
	}

	Fx.ChangedJoint->SetType(EMjJointType::ball);

	if (!S.Recompile())
	{
		AddError(FString::Printf(TEXT("recompile failed: %s"), *S.LastError));
		return false;
	}

	const FSnapshot After = Capture(S);
	ExpectCarried(*this, Before, After);

	const mjModel* const M = S.Model();
	const mjData* const D = S.Data();
	const int32 ChangedId = S.MjId(mjOBJ_JOINT, TEXT("ChangedJoint"));
	if (!TestTrue(TEXT("ChangedJoint compiled"), ChangedId >= 0))
	{
		return false;
	}

	// The edit reached the model it claims to have edited.
	TestEqual(TEXT("ChangedJoint is now a ball joint"), static_cast<int32>(M->jnt_type[ChangedId]),
		static_cast<int32>(mjJNT_BALL));

	const int32 QAdr = M->jnt_qposadr[ChangedId];
	const int32 QWidth = (ChangedId + 1 < M->njnt) ? (M->jnt_qposadr[ChangedId + 1] - QAdr) : (M->nq - QAdr);
	TestEqual(TEXT("a ball joint owns four qpos entries"), QWidth, 4);
	for (int32 Index = 0; Index < QWidth; ++Index)
	{
		TestEqual(*FString::Printf(TEXT("ChangedJoint qpos[%d] reset to qpos0"), Index),
			D->qpos[QAdr + Index], M->qpos0[QAdr + Index], UE_DOUBLE_KINDA_SMALL_NUMBER);
	}

	const int32 VAdr = M->jnt_dofadr[ChangedId];
	const int32 VWidth = (ChangedId + 1 < M->njnt) ? (M->jnt_dofadr[ChangedId + 1] - VAdr) : (M->nv - VAdr);
	TestEqual(TEXT("a ball joint owns three dofs"), VWidth, 3);
	for (int32 Index = 0; Index < VWidth; ++Index)
	{
		TestEqual(*FString::Printf(TEXT("ChangedJoint qvel[%d] reset to zero"), Index),
			D->qvel[VAdr + Index], 0.0, UE_DOUBLE_KINDA_SMALL_NUMBER);
	}

	return true;
}

#endif  // URLAB_MJ_GEN

#endif  // WITH_EDITOR
