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
// The load-bearing case is the one an address-keyed migration would get wrong
// and be seen to get right: delete a joint, and the joint that inherits its
// qpos slot must hold ITS OWN value, not the deleted one's. Everything else
// here -- a survivor keeping its pose, a new element starting at qpos0 -- would
// pass under either rule.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#if WITH_EDITOR

#include "MjTestHelpers.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Spec/MjTreeAdapters.h"

namespace MjRecompileMigrationTests
{
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
		NewGeom->SetSize({0.1, 0.1, 0.1});
	}
	Session.Add<UMjJoint>(NewBody, JointName);
	return NewBody;
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

#endif  // URLAB_MJ_GEN

#endif  // WITH_EDITOR
