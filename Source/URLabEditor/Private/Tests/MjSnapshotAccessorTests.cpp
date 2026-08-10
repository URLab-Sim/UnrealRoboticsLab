// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0.

// The Blueprint-facing accessors and the per-step snapshot they read.
//
// They used to index live mjData, so a gameplay script asking two questions
// while the physics thread was mid-step could be answered out of two different
// steps, or out of memory being written as it was read. They now read the same
// published snapshot the visual update and the networking path already read,
// which is one coherent physics frame per step.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjTestHelpers.h"

#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/MjRenderSnapshot.h"
#include "MuJoCo/Elements/MjActuatorRuntime.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Elements/MjJointRuntime.h"
#include "MuJoCo/Elements/MjSensorRuntime.h"
#include "MuJoCo/Gen/Elements/Keyframes/MjKey.gen.h"
#include "MuJoCo/Gen/Elements/Keyframes/MjKeyframe.gen.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Utils/URLabAxisConv.h"

namespace
{
/** A model with one of everything the accessor surfaces can be asked about. */
const TCHAR* kAccessorMjcf =
	TEXT("<mujoco model=\"accessors\">\n")
	TEXT("  <worldbody>\n")
	TEXT("    <body name=\"link\" pos=\"0 0 0.5\">\n")
	TEXT("      <joint name=\"hinge\" type=\"hinge\" axis=\"0 1 0\"/>\n")
	TEXT("      <geom name=\"shaft\" type=\"capsule\" fromto=\"0 0 0 0.4 0 0\" size=\"0.05\"/>\n")
	TEXT("    </body>\n")
	TEXT("  </worldbody>\n")
	TEXT("  <tendon><fixed name=\"cable\"><joint joint=\"hinge\" coef=\"1\"/></fixed></tendon>\n")
	TEXT("  <actuator><general name=\"drive\" joint=\"hinge\" gear=\"5\" dyntype=\"integrator\"/></actuator>\n")
	TEXT("  <sensor><jointpos name=\"encoder\" joint=\"hinge\"/></sensor>\n")
	TEXT("</mujoco>\n");

/** The spec element `Actor` carries under the MJCF name `MjName`. */
UMjNodeComponent* FindElement(const AActor* Actor, const TCHAR* MjName)
{
	if (Actor == nullptr)
	{
		return nullptr;
	}
	for (UActorComponent* Component : Actor->GetComponents())
	{
		UMjNodeComponent* Node = Cast<UMjNodeComponent>(Component);
		if (Node != nullptr && Node->MjName.IsSet() && Node->MjName.GetValue() == MjName)
		{
			return Node;
		}
	}
	return nullptr;
}

/** The compiled id of one of the imported articulation's elements. */
int32 MjIdOf(const FMjXmlImportSession& S, mjtObj Type, const TCHAR* Name)
{
	mjModel* const M = S.Model();
	if (M == nullptr || S.Robot == nullptr)
	{
		return -1;
	}
	const FString Full = S.Robot->GetCompiledPrefix() + Name;
	return mj_name2id(M, Type, TCHAR_TO_UTF8(*Full));
}
}  // namespace

// ============================================================================
// URLab.Runtime.AccessorsReadThePublishedSnapshot
//   The snapshot carries everything the accessors report, sized with the model
//   it was compiled for, and what each accessor answers is what the snapshot
//   holds -- which after a completed step is the stepped state.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjAccessorsReadThePublishedSnapshot,
	"URLab.Runtime.AccessorsReadThePublishedSnapshot",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjAccessorsReadThePublishedSnapshot::RunTest(const FString& Parameters)
{
	FMjXmlImportSession S;
	if (!S.Init(kAccessorMjcf) || !S.Compile())
	{
		AddError(FString::Printf(TEXT("session setup failed: %s"), *S.LastError));
		return false;
	}

	UMjPhysicsEngine* Engine = S.Manager->PhysicsEngine;
	const mjModel* M = S.Model();
	const mjData* D = S.Data();
	if (Engine == nullptr || M == nullptr || D == nullptr)
	{
		AddError(TEXT("no compiled model"));
		return false;
	}

	// A few steps under gravity so nothing being compared is still at zero.
	Engine->StepSync(50);

	const int32 JointId = MjIdOf(S, mjOBJ_JOINT, TEXT("hinge"));
	const int32 ActuatorId = MjIdOf(S, mjOBJ_ACTUATOR, TEXT("drive"));
	const int32 TendonId = MjIdOf(S, mjOBJ_TENDON, TEXT("cable"));
	const int32 SensorId = MjIdOf(S, mjOBJ_SENSOR, TEXT("encoder"));
	const int32 BodyId = MjIdOf(S, mjOBJ_BODY, TEXT("link"));
	if (JointId < 0 || ActuatorId < 0 || TendonId < 0 || SensorId < 0 || BodyId < 0)
	{
		AddError(TEXT("the compiled model is missing one of the fixture's elements"));
		return false;
	}

	const int32 QposAdr = M->jnt_qposadr[JointId];
	const int32 DofAdr = M->jnt_dofadr[JointId];
	const int32 SensorAdr = M->sensor_adr[SensorId];

	// The worker is not running, so live mjData is the reference the snapshot
	// is checked against; there is nothing here for a step to move underneath.
	Engine->WithRenderState([&](const FMjRenderSnapshot& Snap) {
		TestEqual(TEXT("joint anchors are sized with the model"),
			Snap.JntXAnchor.Num(), static_cast<int32>(M->njnt) * 3);
		TestEqual(TEXT("joint axes are sized with the model"),
			Snap.JntXAxis.Num(), static_cast<int32>(M->njnt) * 3);
		TestEqual(TEXT("controls are sized with the model"),
			Snap.Ctrl.Num(), static_cast<int32>(M->nu));
		TestEqual(TEXT("actuator lengths are sized with the model"),
			Snap.ActuatorLength.Num(), static_cast<int32>(M->nu));
		TestEqual(TEXT("actuator velocities are sized with the model"),
			Snap.ActuatorVelocity.Num(), static_cast<int32>(M->nu));
		TestEqual(TEXT("activations are sized with the model"),
			Snap.Act.Num(), static_cast<int32>(M->na));
		TestEqual(TEXT("tendon lengths are sized with the model"),
			Snap.TenLength.Num(), static_cast<int32>(M->ntendon));
		TestEqual(TEXT("tendon velocities are sized with the model"),
			Snap.TenVelocity.Num(), static_cast<int32>(M->ntendon));

		TestEqual(TEXT("snapshot qpos is the stepped qpos"),
			Snap.QPos[QposAdr], D->qpos[QposAdr]);
		TestEqual(TEXT("snapshot tendon length is the stepped tendon length"),
			Snap.TenLength[TendonId], D->ten_length[TendonId]);
		TestEqual(TEXT("snapshot actuator length is the stepped actuator length"),
			Snap.ActuatorLength[ActuatorId], D->actuator_length[ActuatorId]);
		TestEqual(TEXT("snapshot joint anchor is the stepped joint anchor"),
			Snap.JntXAnchor[JointId * 3 + 2], D->xanchor[JointId * 3 + 2]);
	});

	// The model must actually have moved, or every equality above would hold
	// against a snapshot that was never refreshed.
	TestNotEqual(TEXT("the fixture moved under gravity"), D->qpos[QposAdr], 0.0);

	UMjNodeComponent* Joint = FindElement(S.Robot, TEXT("hinge"));
	UMjNodeComponent* Actuator = FindElement(S.Robot, TEXT("drive"));
	UMjNodeComponent* Tendon = FindElement(S.Robot, TEXT("cable"));
	UMjNodeComponent* Sensor = FindElement(S.Robot, TEXT("encoder"));
	UMjBody* Body = Cast<UMjBody>(FindElement(S.Robot, TEXT("link")));
	if (Joint == nullptr || Actuator == nullptr || Tendon == nullptr
		|| Sensor == nullptr || Body == nullptr)
	{
		AddError(TEXT("the imported articulation is missing one of the fixture's elements"));
		return false;
	}

	TestEqual(TEXT("joint position"), UMjJointRuntime::GetPosition(Joint),
		static_cast<float>(D->qpos[QposAdr]));
	TestEqual(TEXT("joint velocity"), UMjJointRuntime::GetVelocity(Joint),
		static_cast<float>(D->qvel[DofAdr]));
	TestEqual(TEXT("joint acceleration"), UMjJointRuntime::GetAcceleration(Joint),
		static_cast<float>(D->qacc[DofAdr]));
	TestEqual(TEXT("joint world anchor"), UMjJointRuntime::GetWorldAnchor(Joint),
		URLabAxisConv::MjPositionToUe(&D->xanchor[JointId * 3]));

	TestEqual(TEXT("tendon length"), UMjTendonRuntime::GetLength(Tendon),
		static_cast<float>(D->ten_length[TendonId]));
	TestEqual(TEXT("tendon velocity"), UMjTendonRuntime::GetVelocity(Tendon),
		static_cast<float>(D->ten_velocity[TendonId]));

	TestEqual(TEXT("actuator applied control"), UMjActuatorRuntime::GetAppliedControl(Actuator),
		static_cast<float>(D->ctrl[ActuatorId]));
	TestEqual(TEXT("actuator force"), UMjActuatorRuntime::GetForce(Actuator),
		static_cast<float>(D->actuator_force[ActuatorId]));
	TestEqual(TEXT("actuator length"), UMjActuatorRuntime::GetLength(Actuator),
		static_cast<float>(D->actuator_length[ActuatorId]));
	TestEqual(TEXT("actuator velocity"), UMjActuatorRuntime::GetVelocity(Actuator),
		static_cast<float>(D->actuator_velocity[ActuatorId]));
	if (M->actuator_actadr[ActuatorId] >= 0)
	{
		TestEqual(TEXT("actuator activation"), UMjActuatorRuntime::GetActivation(Actuator),
			static_cast<float>(D->act[M->actuator_actadr[ActuatorId]]));
	}

	TestEqual(TEXT("sensor reading"), UMjSensorRuntime::GetScalarReading(Sensor),
		static_cast<float>(D->sensordata[SensorAdr]));

	TestEqual(TEXT("body world position"), Body->GetWorldPosition(),
		URLabAxisConv::MjPositionToUe(&D->xpos[BodyId * 3]));

	return true;
}

// ============================================================================
// URLab.Runtime.AccessorsUnderStepping
//   The reason for the snapshot: the game thread reading while the physics
//   thread steps. Every read has to come back a whole value from one frame,
//   for thousands of reads across thousands of steps.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjAccessorsUnderStepping,
	"URLab.Runtime.AccessorsUnderStepping",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjAccessorsUnderStepping::RunTest(const FString& Parameters)
{
	FMjXmlImportSession S;
	if (!S.Init(kAccessorMjcf) || !S.Compile())
	{
		AddError(FString::Printf(TEXT("session setup failed: %s"), *S.LastError));
		return false;
	}

	UMjPhysicsEngine* Engine = S.Manager->PhysicsEngine;
	UMjNodeComponent* Joint = FindElement(S.Robot, TEXT("hinge"));
	UMjNodeComponent* Actuator = FindElement(S.Robot, TEXT("drive"));
	UMjNodeComponent* Tendon = FindElement(S.Robot, TEXT("cable"));
	UMjNodeComponent* Sensor = FindElement(S.Robot, TEXT("encoder"));
	UMjBody* Body = Cast<UMjBody>(FindElement(S.Robot, TEXT("link")));
	if (Engine == nullptr || Joint == nullptr || Actuator == nullptr
		|| Tendon == nullptr || Sensor == nullptr || Body == nullptr)
	{
		AddError(TEXT("the imported articulation is missing one of the fixture's elements"));
		return false;
	}

	Engine->SetPaused(false);
	Engine->SetStepMode(EStepMode::Live);
	Engine->RunMujocoAsync();

	int32 Finite = 0;
	const int32 Reads = 20000;
	for (int32 I = 0; I < Reads; ++I)
	{
		const float Sum = UMjJointRuntime::GetPosition(Joint)
			+ UMjJointRuntime::GetVelocity(Joint)
			+ UMjJointRuntime::GetAcceleration(Joint)
			+ UMjTendonRuntime::GetLength(Tendon)
			+ UMjActuatorRuntime::GetForce(Actuator)
			+ UMjSensorRuntime::GetScalarReading(Sensor)
			+ static_cast<float>(Body->GetWorldPosition().Z)
			+ static_cast<float>(UMjJointRuntime::GetWorldAxis(Joint).X);
		if (FMath::IsFinite(Sum))
		{
			++Finite;
		}
	}

	Engine->bShouldStopTask = true;
	if (Engine->StepRequestEvent != nullptr)
	{
		Engine->StepRequestEvent->Trigger();
	}
	if (Engine->AsyncPhysicsFuture.IsValid())
	{
		Engine->AsyncPhysicsFuture.Wait();
	}

	TestEqual(TEXT("every read came back a finite value"), Finite, Reads);
	TestTrue(TEXT("the worker stepped while the reads ran"), Engine->GetRenderFrameId() > 1);

	return true;
}

// ============================================================================
// URLab.Runtime.ResetToKeyframeIsVisibleToReaders
//   Resetting to a keyframe is a state change a caller expects to observe on
//   the very next read. The accessors answer from the published snapshot, so
//   the reset has to publish; before it did, a read after the reset still
//   returned the pose the reset replaced.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjResetToKeyframeIsVisibleToReaders,
	"URLab.Runtime.ResetToKeyframeIsVisibleToReaders",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjResetToKeyframeIsVisibleToReaders::RunTest(const FString& Parameters)
{
	const double KeyframeAngle = 0.7;
	const double PreResetAngle = 0.2;

	FMjUESession S;
	if (!S.Init([KeyframeAngle](FMjUESession& Sess) {
			Sess.Joint->SetType(EMjJointType::hinge);
			UMjKeyframe* Section = Sess.Add<UMjKeyframe>(Sess.Robot->Spec);
			UMjKey* Key = Sess.Add<UMjKey>(Section, TEXT("home"));
			if (Key != nullptr)
			{
				Key->SetQpos({KeyframeAngle});
			}
		}))
	{
		AddError(FString::Printf(TEXT("session setup failed: %s"), *S.LastError));
		return false;
	}

	UMjPhysicsEngine* Engine = S.Manager->PhysicsEngine;
	const mjModel* M = S.Model();
	const mjData* D = S.Data();
	const int32 JointId = S.MjId(mjOBJ_JOINT, TEXT("TestJoint"));
	if (Engine == nullptr || M == nullptr || D == nullptr || JointId < 0)
	{
		AddError(TEXT("the compiled model is missing the fixture's joint"));
		return false;
	}
	if (M->nkey < 1)
	{
		AddError(TEXT("the fixture's keyframe did not compile"));
		return false;
	}

	const int32 QposAdr = M->jnt_qposadr[JointId];

	// A published pose for the reset to be stale against: this write goes
	// through the engine's synchronous edit, which publishes, so the accessor
	// genuinely reads it before the reset runs.
	Engine->ApplyJointPosition(JointId, PreResetAngle);
	TestEqual(TEXT("the joint reads the pre-reset pose"),
		UMjJointRuntime::GetPosition(S.Joint), static_cast<float>(PreResetAngle));

	if (!S.Robot->ResetToKeyframe(TEXT("home")))
	{
		AddError(TEXT("ResetToKeyframe did not find the fixture's keyframe"));
		return false;
	}

	// Separating the write from the publish: if only the first of these fails
	// the keyframe was never applied, and if only the second does the reset
	// applied it without publishing it.
	TestEqual(TEXT("the reset reached mjData"), D->qpos[QposAdr], KeyframeAngle);
	TestEqual(TEXT("a read after the reset is the keyframe pose, not the pre-reset one"),
		UMjJointRuntime::GetPosition(S.Joint), static_cast<float>(KeyframeAngle));
	TestEqual(TEXT("the articulation's own joint query agrees"),
		S.Robot->GetJointAngle(TEXT("TestJoint")), static_cast<float>(KeyframeAngle));

	S.Cleanup();
	return true;
}
