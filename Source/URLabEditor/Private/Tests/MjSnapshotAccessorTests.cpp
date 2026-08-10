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
#include "Components/LineBatchComponent.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjTestHelpers.h"

#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/MjRenderSnapshot.h"
#include "MuJoCo/Elements/MjActuatorRuntime.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Elements/MjJointRuntime.h"
#include "MuJoCo/Elements/MjSensorRuntime.h"
#include "MuJoCo/Gen/Elements/Geometry/MjSite.gen.h"
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
//   thread steps. Two things have to hold. Every frame a reader observes is
//   one whole physics frame -- a jointpos sensor reads exactly the qpos it
//   sensed, which is only true if both came out of the same step. And a reader
//   that keeps reading keeps getting new frames: in live mode the worker
//   publishes what a consumer asked for, and reading is the asking, so an
//   accessor is not pinned to whichever frame the visual update last wanted.
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
	const mjModel* M = S.Model();
	const mjData* D = S.Data();
	UMjNodeComponent* Joint = FindElement(S.Robot, TEXT("hinge"));
	UMjNodeComponent* Actuator = FindElement(S.Robot, TEXT("drive"));
	UMjNodeComponent* Tendon = FindElement(S.Robot, TEXT("cable"));
	UMjNodeComponent* Sensor = FindElement(S.Robot, TEXT("encoder"));
	UMjBody* Body = Cast<UMjBody>(FindElement(S.Robot, TEXT("link")));
	if (Engine == nullptr || M == nullptr || D == nullptr || Joint == nullptr
		|| Actuator == nullptr || Tendon == nullptr || Sensor == nullptr || Body == nullptr)
	{
		AddError(TEXT("the imported articulation is missing one of the fixture's elements"));
		return false;
	}

	const int32 JointId = MjIdOf(S, mjOBJ_JOINT, TEXT("hinge"));
	const int32 SensorId = MjIdOf(S, mjOBJ_SENSOR, TEXT("encoder"));
	const int32 BodyId = MjIdOf(S, mjOBJ_BODY, TEXT("link"));
	if (JointId < 0 || SensorId < 0 || BodyId < 0)
	{
		AddError(TEXT("the compiled model is missing one of the fixture's elements"));
		return false;
	}
	const int32 QposAdr = M->jnt_qposadr[JointId];
	const int32 DofAdr = M->jnt_dofadr[JointId];
	const int32 SensorAdr = M->sensor_adr[SensorId];

	Engine->SetPaused(false);
	Engine->SetStepMode(EStepMode::Live);
	Engine->RunMujocoAsync();

	// Enough frames that no publish cadence but a per-step one can produce them,
	// with a wall-clock budget so a worker that never steps fails rather than
	// hangs. At the fixture's timestep the target is a few tens of milliseconds.
	const int32 WantedFrames = 16;
	const double Budget = 5.0;
	const double Started = FPlatformTime::Seconds();

	int32 Visits = 0;
	int32 CoherentVisits = 0;
	int32 DistinctFrames = 0;
	int32 DistinctAccessorValues = 0;
	uint64 LastFrameId = 0;
	float LastAngle = 0.0f;
	bool bHaveAngle = false;

	while (DistinctFrames < WantedFrames && (FPlatformTime::Seconds() - Started) < Budget)
	{
		Engine->WithRenderState([&](const FMjRenderSnapshot& Snap) {
			if (!Snap.QPos.IsValidIndex(QposAdr) || !Snap.SensorData.IsValidIndex(SensorAdr))
			{
				return;
			}
			++Visits;
			// A jointpos sensor is a copy of the qpos it sensed, so these two
			// arrays disagree only if the frame was assembled out of two steps.
			if (Snap.QPos[QposAdr] == Snap.SensorData[SensorAdr])
			{
				++CoherentVisits;
			}
			if (Snap.FrameId != LastFrameId)
			{
				LastFrameId = Snap.FrameId;
				++DistinctFrames;
			}
		});

		// The accessor surface, hammered through the same stepping. Its value
		// tracking the model is the consumer-facing half of the cadence: pinned
		// to one publish it would answer the same number every time.
		const float Angle = UMjJointRuntime::GetPosition(Joint);
		if (!bHaveAngle || Angle != LastAngle)
		{
			LastAngle = Angle;
			bHaveAngle = true;
			++DistinctAccessorValues;
		}
		UMjJointRuntime::GetVelocity(Joint);
		UMjJointRuntime::GetAcceleration(Joint);
		UMjTendonRuntime::GetLength(Tendon);
		UMjActuatorRuntime::GetForce(Actuator);
		UMjSensorRuntime::GetScalarReading(Sensor);
		Body->GetWorldPosition();

		// The reads take the same lock the publish does. Yielding between
		// bursts keeps this loop from being the reason no new frame arrives.
		FPlatformProcess::YieldThread();
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

	AddInfo(FString::Printf(TEXT("visits=%d frames=%d accessor values=%d"),
		Visits, DistinctFrames, DistinctAccessorValues));

	TestTrue(TEXT("every read landed on a published frame"), Visits > 0);
	TestEqual(TEXT("every frame read was one whole physics frame"), CoherentVisits, Visits);
	TestEqual(TEXT("reading kept the frames coming while the worker stepped"),
		DistinctFrames, WantedFrames);
	TestTrue(TEXT("the accessor tracked the stepping model"), DistinctAccessorValues > 1);

	// The worker is joined, so one more step is the only thing that moves the
	// model and the accessors answer from what it published.
	Engine->StepSync(1);
	TestEqual(TEXT("the joint accessor is the stepped joint position"),
		UMjJointRuntime::GetPosition(Joint), static_cast<float>(D->qpos[QposAdr]));
	TestEqual(TEXT("the joint accessor is the stepped joint velocity"),
		UMjJointRuntime::GetVelocity(Joint), static_cast<float>(D->qvel[DofAdr]));
	TestEqual(TEXT("the sensor accessor is the stepped sensor reading"),
		UMjSensorRuntime::GetScalarReading(Sensor), static_cast<float>(D->sensordata[SensorAdr]));
	TestEqual(TEXT("the body accessor is the stepped body position"),
		Body->GetWorldPosition(), URLabAxisConv::MjPositionToUe(&D->xpos[BodyId * 3]));

	return true;
}

// ============================================================================
// URLab.Runtime.DebugDrawReadsThePublishedSnapshot
//   The viewport overlays are consumers like any other. Drawn from live mjData
//   they show a frame the meshes beside them are not showing, and they read it
//   while the physics thread writes it. Drawn from the snapshot they show the
//   frame everything else in that UE frame shows.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDebugDrawReadsThePublishedSnapshot,
	"URLab.Runtime.DebugDrawReadsThePublishedSnapshot",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjDebugDrawReadsThePublishedSnapshot::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init([](FMjUESession& Sess) { Sess.Add<UMjSite>(Sess.Body, TEXT("TestSite")); }))
	{
		AddError(FString::Printf(TEXT("session setup failed: %s"), *S.LastError));
		return false;
	}

	UMjPhysicsEngine* Engine = S.Manager->PhysicsEngine;
	mjData* D = S.Data();
	const int32 SiteId = S.MjId(mjOBJ_SITE, TEXT("TestSite"));
	ULineBatchComponent* Batcher = S.World != nullptr
		? S.World->GetLineBatcher(UWorld::ELineBatcherType::World)
		: nullptr;
	if (Engine == nullptr || D == nullptr || SiteId < 0 || Batcher == nullptr)
	{
		AddError(TEXT("the compiled model is missing the fixture's site, or the world has no line batcher"));
		return false;
	}

	Engine->PushRenderState();

	FVector Published = FVector::ZeroVector;
	Engine->WithRenderState([&Published, SiteId](const FMjRenderSnapshot& Snap) {
		if (Snap.SiteXPos.IsValidIndex(SiteId * 3 + 2))
		{
			Published = URLabAxisConv::MjPositionToUe(&Snap.SiteXPos[SiteId * 3]);
		}
	});

	// Live mjData moves and nothing publishes it, which is exactly the state a
	// mid-step read would find.
	D->site_xpos[SiteId * 3 + 0] += 5.0;
	const FVector Live = URLabAxisConv::MjPositionToUe(&D->site_xpos[SiteId * 3]);

	Batcher->BatchedLines.Reset();
	S.Robot->DrawDebugSites();

	TestEqual(TEXT("the site draws its cross"), Batcher->BatchedLines.Num(), 3);
	if (Batcher->BatchedLines.Num() == 3)
	{
		const FVector Centre = (Batcher->BatchedLines[0].Start + Batcher->BatchedLines[0].End) * 0.5;
		TestEqual(TEXT("the cross is where the published frame puts the site"), Centre, Published);
		TestFalse(TEXT("the cross is not where unpublished mjData puts it"), Centre.Equals(Live, 1.0f));
	}

	// Publishing the moved state moves the draw with it, so the overlay is
	// following the snapshot rather than ignoring simulation state.
	Engine->PushRenderState();
	Batcher->BatchedLines.Reset();
	S.Robot->DrawDebugSites();

	if (Batcher->BatchedLines.Num() == 3)
	{
		const FVector Centre = (Batcher->BatchedLines[0].Start + Batcher->BatchedLines[0].End) * 0.5;
		TestEqual(TEXT("publishing the moved state moves the cross"), Centre, Live);
	}

	S.Cleanup();
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
