// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// The PD control law itself: torque = Kp * (target - qpos) - Kv * qvel, clamped.
//
// The law runs every physics step and is what a policy trained motor+PD lands
// on, so a sign error or a lost gain is a robot that falls over rather than a
// failed assertion. The config round-trip next door tests the JSON, not this.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#if WITH_EDITOR

#include "MjTestHelpers.h"

#include "MuJoCo/Controllers/MjPDController.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Elements/MjActuatorRuntime.h"
#include "MuJoCo/Gen/Elements/Actuators/MjActuator.gen.h"
#include "MuJoCo/Gen/Elements/Actuators/MjMotor.gen.h"
#include "MuJoCo/Gen/Elements/Joints/MjJoint.gen.h"
#include "MuJoCo/Spec/MjNodeComponent.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

namespace
{

/** The actuator-id map `Bind` takes, built from the articulation's actuators. */
TMap<int32, UMjNodeComponent*> ActuatorIdMap(const AMjArticulation& Robot)
{
	TMap<int32, UMjNodeComponent*> Out;
	for (UMjNodeComponent* A : Robot.GetActuators())
	{
		const int32 Id = A ? A->GetBoundId().Get(-1) : -1;
		if (Id >= 0)
		{
			Out.Add(Id, A);
		}
	}
	return Out;
}

/**
 * A hinge with a range, driven by a `<motor>`: the actuator kind
 * `UMjPDController` is written for, and a limited joint so the target clamp
 * has something to clamp against.
 */
void ConfigureMotorArm(FMjUESession& Sess)
{
	Sess.Joint->SetType(EMjJointType::hinge);
	Sess.Joint->SetRange(FVector2D(-1.0, 1.0));
	UMjActuator* Section = Sess.Add<UMjActuator>(Sess.Robot->Spec);
	if (Section == nullptr)
	{
		return;
	}
	if (UMjMotor* A = Sess.Add<UMjMotor>(Section, TEXT("TestActuator")))
	{
		A->SetJoint(Sess.Joint->MjName.GetValue());
	}
}

} // namespace

// ---------------------------------------------------------------------------
// The law, against hand-computed torque.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjPDControllerLaw,
	"URLab.Controllers.PDLaw",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjPDControllerLaw::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init(&ConfigureMotorArm))
	{
		AddInfo(FString::Printf(TEXT("session did not stand up: %s"), *S.LastError));
		return true;
	}

	mjModel* m = S.Manager->PhysicsEngine ? S.Manager->PhysicsEngine->GetModel() : nullptr;
	mjData* d = S.Manager->PhysicsEngine ? S.Manager->PhysicsEngine->GetData() : nullptr;
	if (m == nullptr || d == nullptr || m->nu == 0)
	{
		AddError(TEXT("test session compiled no actuated model"));
		S.Cleanup();
		return false;
	}

	UMjPDController* Pd = NewObject<UMjPDController>(S.Robot);
	Pd->RegisterComponent();
	Pd->DefaultKp = 20.0f;
	Pd->DefaultKv = 3.0f;
	Pd->DefaultTorqueLimit = 1000.0f; // out of the way until the clamp case
	Pd->Bind(m, d, ActuatorIdMap(*S.Robot));
	TestTrue(TEXT("controller bound"), Pd->IsBound());

	const TArray<int32> Owned = S.Robot->GetOwnedActuatorIds();
	if (Owned.Num() == 0)
	{
		AddError(TEXT("articulation owns no actuator slots"));
		S.Cleanup();
		return false;
	}
	const int32 ActId = Owned[0];
	const int32 JntId = m->actuator_trnid[ActId * 2];
	const int32 QposAddr = m->jnt_qposadr[JntId];
	const int32 QvelAddr = m->jnt_dofadr[JntId];

	// A target the joint range cannot clip, so this case tests the law alone.
	const double Target = m->jnt_limited[JntId]
		? 0.5 * (m->jnt_range[JntId * 2] + m->jnt_range[JntId * 2 + 1])
		: 0.25;
	const double Pos = Target - 0.1;
	const double Vel = 0.2;

	d->qpos[QposAddr] = Pos;
	d->qvel[QvelAddr] = Vel;
	S.Robot->StageNetworkControl(ActId, Target);
	S.Robot->ControlSource = 0;

	Pd->ComputeAndApply(m, d, /*Source=*/0);

	const double Expected = 20.0 * (Target - Pos) - 3.0 * Vel;
	TestEqual(TEXT("torque is Kp*(target-pos) - Kv*vel"),
		(double)d->ctrl[ActId], Expected, 1e-9);

	// Zero error and zero velocity is zero torque: catches a stray bias term.
	d->qpos[QposAddr] = Target;
	d->qvel[QvelAddr] = 0.0;
	Pd->ComputeAndApply(m, d, /*Source=*/0);
	TestEqual(TEXT("no error and no motion is no torque"), (double)d->ctrl[ActId], 0.0, 1e-9);

	// Velocity alone damps against the direction of travel.
	d->qvel[QvelAddr] = 1.0;
	Pd->ComputeAndApply(m, d, /*Source=*/0);
	TestEqual(TEXT("damping opposes velocity"), (double)d->ctrl[ActId], -3.0, 1e-9);

	// The clamp is symmetric and binds. Bound fresh rather than by moving the
	// default under a controller that already seeded its per-joint arrays.
	UMjPDController* Clamped = NewObject<UMjPDController>(S.Robot);
	Clamped->RegisterComponent();
	Clamped->DefaultKp = 20.0f;
	Clamped->DefaultKv = 3.0f;
	Clamped->DefaultTorqueLimit = 0.5f;
	Clamped->Bind(m, d, ActuatorIdMap(*S.Robot));

	d->qpos[QposAddr] = Target - 1.0;
	d->qvel[QvelAddr] = 0.0;
	Clamped->ComputeAndApply(m, d, /*Source=*/0);
	TestEqual(TEXT("torque clamps to +limit"), (double)d->ctrl[ActId], 0.5, 1e-9);

	d->qpos[QposAddr] = Target + 1.0;
	Clamped->ComputeAndApply(m, d, /*Source=*/0);
	TestEqual(TEXT("torque clamps to -limit"), (double)d->ctrl[ActId], -0.5, 1e-9);

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// A target outside the joint range is clamped to it, not fed through.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjPDControllerTargetClamp,
	"URLab.Controllers.PDTargetClamp",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjPDControllerTargetClamp::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init(&ConfigureMotorArm))
	{
		AddInfo(FString::Printf(TEXT("session did not stand up: %s"), *S.LastError));
		return true;
	}

	mjModel* m = S.Manager->PhysicsEngine ? S.Manager->PhysicsEngine->GetModel() : nullptr;
	mjData* d = S.Manager->PhysicsEngine ? S.Manager->PhysicsEngine->GetData() : nullptr;
	const TArray<int32> Owned = S.Robot ? S.Robot->GetOwnedActuatorIds() : TArray<int32>();
	if (m == nullptr || d == nullptr || Owned.Num() == 0)
	{
		AddError(TEXT("test session compiled no actuated model"));
		S.Cleanup();
		return false;
	}

	const int32 ActId = Owned[0];
	const int32 JntId = m->actuator_trnid[ActId * 2];
	if (!m->jnt_limited[JntId])
	{
		AddInfo(TEXT("test model's joint is unlimited; nothing to clamp against"));
		S.Cleanup();
		return true;
	}

	UMjPDController* Pd = NewObject<UMjPDController>(S.Robot);
	Pd->RegisterComponent();
	Pd->DefaultKp = 1.0f;
	Pd->DefaultKv = 0.0f;
	Pd->DefaultTorqueLimit = 1e6f;
	Pd->Bind(m, d, ActuatorIdMap(*S.Robot));

	const double Hi = m->jnt_range[JntId * 2 + 1];
	d->qpos[m->jnt_qposadr[JntId]] = 0.0;
	d->qvel[m->jnt_dofadr[JntId]] = 0.0;
	S.Robot->ControlSource = 0;
	S.Robot->StageNetworkControl(ActId, Hi + 100.0);

	Pd->ComputeAndApply(m, d, /*Source=*/0);

	TestEqual(TEXT("target clamped to the joint's upper limit"),
		(double)d->ctrl[ActId], Hi, 1e-9);

	S.Cleanup();
	return true;
}

#endif // WITH_EDITOR
