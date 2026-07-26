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
// MjStateCollectorTests.cpp
//
// Unit tests for the state-serialization IR (Phase 0a):
//  - FMjCanonicalName sanitize / part-segment stripping
//  - FMjStateCollector produces the IR fields today's paths carried
//  - Sensor values are emitted transformed (wire == GetReading())
//  - FMjMsgpackEncoder canonical schema + observation-level filter
//  - StructureVersion bumps on a producer-cache rebuild, not a plain collect
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "MjTestHelpers.h"
#include "State/MjCanonicalName.h"
#include "State/MjStateCollector.h"
#include "State/MjMsgpackEncoder.h"
#include "State/MjStateTypes.h"
#include "Bridge/RpcDispatcher.h"
#include "Bridge/BridgeServer.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "MuJoCo/Components/Sensors/MjSensor.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"

using EObs = FURLabRpcDispatcher::EObservationLevel;

// ---------------------------------------------------------------------------
// 1. FMjCanonicalName::Sanitize + PartSegment
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStateCanonicalName,
	"URLab.State.CanonicalName",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStateCanonicalName::RunTest(const FString& Parameters)
{
	TestEqual(TEXT("passthrough legal"),
		FMjCanonicalName::Sanitize(TEXT("go2_imu_gyro")), FString(TEXT("go2_imu_gyro")));
	TestEqual(TEXT("illegal chars -> _"),
		FMjCanonicalName::Sanitize(TEXT("arm/link-1.x")), FString(TEXT("arm_link_1_x")));
	TestEqual(TEXT("leading digit gets _ prefix"),
		FMjCanonicalName::Sanitize(TEXT("3dof")), FString(TEXT("_3dof")));
	TestEqual(TEXT("empty stays empty"),
		FMjCanonicalName::Sanitize(TEXT("")), FString(TEXT("")));

	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}
	AMjArticulation* Art = S.Manager->GetAllArticulations()[0];
	const FString ArtName = Art->GetName();

	// PartSegment strips exactly one "<ArtName>_" prefix, then sanitizes.
	TestEqual(TEXT("PartSegment strips art prefix"),
		FMjCanonicalName::PartSegment(Art, ArtName + TEXT("_shoulder")),
		FName(TEXT("shoulder")));
	// No prefix -> passthrough (sanitized).
	TestEqual(TEXT("PartSegment no-prefix passthrough"),
		FMjCanonicalName::PartSegment(Art, TEXT("free_body")),
		FName(TEXT("free_body")));
	// ArtSegment mirrors the actor name (already legal in tests).
	TestEqual(TEXT("ArtSegment == sanitized actor name"),
		FMjCanonicalName::ArtSegment(Art), FName(*ArtName));

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 2. Canonical msgpack schema: EncodeSnapshot top-level keys + level filter.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStateSchema,
	"URLab.State.Schema",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStateSchema::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}
	mjModel* m = S.Manager->PhysicsEngine->GetModel();
	mjData* d = S.Manager->PhysicsEngine->GetData();
	if (!m || !d)
	{
		AddError(TEXT("Model/data missing"));
		S.Cleanup();
		return false;
	}

	FMjStateCollector& C = S.Manager->GetStateCollector();
	C.Init(S.Manager);
	C.RebuildProducerCacheGameThread();
	const FMjStateSnapshot& Snap = C.Collect(m, d, 7);

	TSharedPtr<FJsonObject> Full = FMjMsgpackEncoder::EncodeSnapshot(Snap, EObs::Full);
	TestTrue(TEXT("snapshot has time"), Full->HasField(TEXT("time")));
	TestTrue(TEXT("snapshot has step"), Full->HasField(TEXT("step")));
	TestTrue(TEXT("snapshot has sim_time"), Full->HasField(TEXT("sim_time")));
	TestTrue(TEXT("snapshot has wall_time"), Full->HasField(TEXT("wall_time")));
	TestTrue(TEXT("snapshot has arts"), Full->HasField(TEXT("arts")));
	TestTrue(TEXT("snapshot has scene"), Full->HasField(TEXT("scene")));

	FString Op;
	Full->TryGetStringField(TEXT("op"), Op);
	TestEqual(TEXT("op == state_full"), Op, FString(TEXT("state_full")));
	double Step = 0.0;
	Full->TryGetNumberField(TEXT("step"), Step);
	TestEqual(TEXT("step echoes collected index"), (int64)Step, (int64)7);

	// Minimal level: per-art block carries qpos/qvel only.
	TSharedPtr<FJsonObject> MinArts = FMjMsgpackEncoder::EncodeArts(Snap, EObs::Minimal);
	if (MinArts->Values.Num() > 0)
	{
		const TSharedPtr<FJsonObject>* ArtObj = nullptr;
		MinArts->Values.CreateConstIterator()->Value->TryGetObject(ArtObj);
		if (ArtObj && ArtObj->IsValid())
		{
			TestTrue(TEXT("Minimal has qpos"), (*ArtObj)->HasField(TEXT("qpos")));
			TestTrue(TEXT("Minimal has qvel"), (*ArtObj)->HasField(TEXT("qvel")));
			TestFalse(TEXT("Minimal lacks ctrl"), (*ArtObj)->HasField(TEXT("ctrl")));
			TestFalse(TEXT("Minimal lacks sensors"), (*ArtObj)->HasField(TEXT("sensors")));
			TestFalse(TEXT("Minimal lacks bodies"), (*ArtObj)->HasField(TEXT("bodies")));
		}
	}
	else
	{
		AddError(TEXT("expected at least one articulation in the arts block"));
	}

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 3. Joint slot widths: hinge is 1/1; a free base is 7/6.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStateJointWidths,
	"URLab.State.JointWidths",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStateJointWidths::RunTest(const FString& Parameters)
{
	// Default rig has a single hinge joint -> per-art qpos/qvel width 1/1.
	{
		FMjUESession S;
		if (!S.Init([](FMjUESession& Sess) {
				Sess.Joint->Type = EMjJointType::Hinge;
				Sess.Joint->bOverride_Type = true;
			}))
		{
			AddError(S.LastError);
			return false;
		}
		mjModel* m = S.Manager->PhysicsEngine->GetModel();
		mjData* d = S.Manager->PhysicsEngine->GetData();
		FMjStateCollector& C = S.Manager->GetStateCollector();
		C.Init(S.Manager);
		C.RebuildProducerCacheGameThread();
		const FMjStateSnapshot& Snap = C.Collect(m, d, 0);
		if (Snap.Articulations.Num() > 0 && Snap.Articulations[0].Joints.Num() > 0)
		{
			const FMjJointState& J = Snap.Articulations[0].Joints[0];
			TestEqual(TEXT("hinge type"), (int)J.Type, (int)EMjJointType::Hinge);
			TestEqual(TEXT("hinge qpos width 1"), J.QPos.Num(), 1);
			TestEqual(TEXT("hinge qvel width 1"), J.QVel.Num(), 1);
		}
		else
		{
			AddError(TEXT("expected a hinge joint in the IR"));
		}
		S.Cleanup();
	}

	// Free base -> 7/6.
	{
		FMjUESession S;
		if (!S.Init([](FMjUESession& Sess) {
				Sess.Joint->Type = EMjJointType::Free;
				Sess.Joint->bOverride_Type = true;
			}))
		{
			AddInfo(FString::Printf(TEXT("Skipping free-joint width: %s"), *S.LastError));
			return true;
		}
		mjModel* m = S.Manager->PhysicsEngine->GetModel();
		mjData* d = S.Manager->PhysicsEngine->GetData();
		FMjStateCollector& C = S.Manager->GetStateCollector();
		C.Init(S.Manager);
		C.RebuildProducerCacheGameThread();
		const FMjStateSnapshot& Snap = C.Collect(m, d, 0);
		if (Snap.Articulations.Num() > 0 && Snap.Articulations[0].Joints.Num() > 0)
		{
			const FMjJointState& J = Snap.Articulations[0].Joints[0];
			TestEqual(TEXT("free type"), (int)J.Type, (int)EMjJointType::Free);
			TestEqual(TEXT("free qpos width 7"), J.QPos.Num(), 7);
			TestEqual(TEXT("free qvel width 6"), J.QVel.Num(), 6);
		}
		else
		{
			AddError(TEXT("expected a free joint in the IR"));
		}
		S.Cleanup();
	}

	return true;
}

// ---------------------------------------------------------------------------
// 4. Actuator ctrl / act / force equal the raw mjData values.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStateActuatorParity,
	"URLab.State.ActuatorParity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStateActuatorParity::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init([](FMjUESession& Sess) {
			Sess.Joint->Type = EMjJointType::Slide;
			Sess.Joint->bOverride_Type = true;
			UMjActuator* A = NewObject<UMjActuator>(Sess.Robot, TEXT("TestActuator"));
			A->Type = EMjActuatorType::Motor;
			A->TargetName = Sess.Joint->GetName();
			A->RegisterComponent();
			A->AttachToComponent(Sess.Robot->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
		}))
	{
		AddInfo(FString::Printf(TEXT("Skipping ActuatorParity: %s"), *S.LastError));
		return true;
	}

	mjModel* m = S.Manager->PhysicsEngine->GetModel();
	mjData* d = S.Manager->PhysicsEngine->GetData();
	if (!m || !d || m->nu == 0)
	{
		AddInfo(TEXT("Skipping ActuatorParity: no actuators compiled"));
		S.Cleanup();
		return true;
	}

	AMjArticulation* Art = S.Manager->GetAllArticulations()[0];
	TArray<UMjActuator*> Acts = Art->GetActuators();
	if (Acts.Num() == 0 || !Acts[0] || Acts[0]->GetMjID() < 0)
	{
		AddInfo(TEXT("Skipping ActuatorParity: actuator did not bind"));
		S.Cleanup();
		return true;
	}
	const int32 Aid = Acts[0]->GetMjID();

	// Drive a known ctrl into d and recompute derived quantities.
	d->ctrl[Aid] = 0.55;
	mj_forward(m, d);

	FMjStateCollector& C = S.Manager->GetStateCollector();
	C.Init(S.Manager);
	C.RebuildProducerCacheGameThread();
	const FMjStateSnapshot& Snap = C.Collect(m, d, 0);

	bool bFound = false;
	for (const FMjArticulationState& AS : Snap.Articulations)
	{
		for (const FMjActuatorState& Act : AS.Actuators)
		{
			bFound = true;
			TestEqual(TEXT("ctrl matches d->ctrl"), Act.Ctrl, (double)d->ctrl[Aid], 1e-9);
			TestEqual(TEXT("force matches d->actuator_force"),
				Act.Force, (double)d->actuator_force[Aid], 1e-9);
			const int ActAddr = (m->actuator_actadr && m->actuator_actadr[Aid] >= 0)
									? m->actuator_actadr[Aid] : -1;
			const double ExpectedAct = (ActAddr >= 0 && ActAddr < m->na) ? d->act[ActAddr] : 0.0;
			TestEqual(TEXT("act matches d->act (0 when stateless)"), Act.Act, ExpectedAct, 1e-9);
		}
	}
	TestTrue(TEXT("actuator present in IR"), bFound);

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 5. Sensor values are emitted transformed: IR Values == GetReading().
//    (Section-7 bug: the transform used to be applied only in GetReading().)
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStateSensorTransformParity,
	"URLab.State.SensorTransformParity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStateSensorTransformParity::RunTest(const FString& Parameters)
{
	// A framequat sensor exercises the quaternion reorder path in
	// TransformSensorReading, so raw slots differ from the transformed reading.
	const FString Xml = TEXT(
		"<mujoco>"
		"  <worldbody>"
		"    <body name=\"b1\" pos=\"0 0 1\">"
		"      <freejoint/>"
		"      <geom type=\"box\" size=\"0.1 0.1 0.1\"/>"
		"      <site name=\"s1\"/>"
		"    </body>"
		"  </worldbody>"
		"  <sensor>"
		"    <framequat name=\"fq\" objtype=\"site\" objname=\"s1\"/>"
		"  </sensor>"
		"</mujoco>");

	FMjXmlImportSession S;
	if (!S.Init(Xml) || !S.Compile())
	{
		AddInfo(FString::Printf(TEXT("Skipping SensorTransformParity: %s"), *S.LastError));
		return true;
	}

	mjModel* m = S.Model();
	mjData* d = S.Data();
	if (!m || !d || !S.Robot)
	{
		AddInfo(TEXT("Skipping SensorTransformParity: no model/robot"));
		S.Cleanup();
		return true;
	}
	mj_forward(m, d);

	// Rotate the body so the quaternion is non-trivial, then recompute.
	if (m->nq >= 7)
	{
		d->qpos[3] = 0.7071; // w
		d->qpos[4] = 0.7071; // x
		d->qpos[5] = 0.0;
		d->qpos[6] = 0.0;
		mj_forward(m, d);
	}

	UMjSensor* Sensor = nullptr;
	TArray<UMjSensor*> Sensors;
	S.Robot->GetComponents<UMjSensor>(Sensors);
	for (UMjSensor* Sen : Sensors)
	{
		if (Sen && !Sen->bIsDefault && Sen->GetMjID() >= 0)
		{
			Sensor = Sen;
			break;
		}
	}
	if (!Sensor)
	{
		AddInfo(TEXT("Skipping SensorTransformParity: no bound sensor"));
		S.Cleanup();
		return true;
	}

	const TArray<float> Reading = Sensor->GetReading();

	FMjStateCollector& C = S.Manager->GetStateCollector();
	C.Init(S.Manager);
	C.RebuildProducerCacheGameThread();
	const FMjStateSnapshot& Snap = C.Collect(m, d, 0);

	const TArray<double>* IRValues = nullptr;
	for (const FMjArticulationState& AS : Snap.Articulations)
	{
		for (const FMjSensorState& Sen : AS.Sensors)
		{
			IRValues = &Sen.Values;
			break;
		}
		if (IRValues)
			break;
	}

	if (!IRValues)
	{
		AddError(TEXT("sensor missing from the IR"));
		S.Cleanup();
		return false;
	}

	TestEqual(TEXT("IR sensor dim == GetReading dim"), IRValues->Num(), Reading.Num());
	if (IRValues->Num() == Reading.Num())
	{
		for (int32 i = 0; i < Reading.Num(); ++i)
			TestEqual(*FString::Printf(TEXT("value[%d] transformed"), i),
				(*IRValues)[i], (double)Reading[i], 1e-5);
	}

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 6. StructureVersion bumps on a producer-cache rebuild, not a plain collect.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStateStructureVersion,
	"URLab.State.StructureVersion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStateStructureVersion::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}
	mjModel* m = S.Manager->PhysicsEngine->GetModel();
	mjData* d = S.Manager->PhysicsEngine->GetData();

	FMjStateCollector& C = S.Manager->GetStateCollector();
	C.Init(S.Manager);
	C.RebuildProducerCacheGameThread();
	const uint32 V0 = C.GetStructureVersion();

	// A plain collect does not change the version.
	C.Collect(m, d, 0);
	TestEqual(TEXT("collect leaves version unchanged"), C.GetStructureVersion(), V0);

	// A rebuild (registry change) bumps it.
	C.RebuildProducerCacheGameThread();
	TestTrue(TEXT("rebuild bumps version"), C.GetStructureVersion() > V0);

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 7. A step reply's `arts` block matches EncodeArts of a fresh Collect.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStateStepReplyArts,
	"URLab.State.StepReplyArts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStateStepReplyArts::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}
	FURLabRpcDispatcher* Disp = S.Manager->GetStepDispatcher();
	if (!Disp)
	{
		AddError(TEXT("Manager has no StepDispatcher"));
		S.Cleanup();
		return false;
	}
	Disp->SetActiveSessionIdForTest(TEXT("test-session"));
	Disp->SetActiveStepMode(EStepMode::Live);

	FMjStateCollector& C = S.Manager->GetStateCollector();
	C.Init(S.Manager);
	C.RebuildProducerCacheGameThread();

	TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
	Req->SetStringField(TEXT("op"), TEXT("step"));
	Req->SetStringField(TEXT("session_id"), TEXT("test-session"));
	TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);

	FString Op;
	Reply->TryGetStringField(TEXT("op"), Op);
	TestEqual(TEXT("op == step_ok"), Op, FString(TEXT("step_ok")));

	const TSharedPtr<FJsonObject>* ReplyArts = nullptr;
	TestTrue(TEXT("reply carries arts"), Reply->TryGetObjectField(TEXT("arts"), ReplyArts));
	TestTrue(TEXT("reply carries scene"), Reply->HasField(TEXT("scene")));

	// The reply's arts share the same articulation key(s) as a direct encode.
	mjModel* m = S.Manager->PhysicsEngine->GetModel();
	mjData* d = S.Manager->PhysicsEngine->GetData();
	const FMjStateSnapshot& Snap = C.Collect(m, d, 0);
	TSharedPtr<FJsonObject> DirectArts = FMjMsgpackEncoder::EncodeArts(Snap, EObs::Standard);
	if (ReplyArts && ReplyArts->IsValid())
	{
		TestEqual(TEXT("same art count as a fresh encode"),
			(*ReplyArts)->Values.Num(), DirectArts->Values.Num());
		for (const auto& Pair : DirectArts->Values)
			TestTrue(*FString::Printf(TEXT("reply arts has key %s"), *Pair.Key),
				(*ReplyArts)->HasField(Pair.Key));
	}

	S.Cleanup();
	return true;
}
