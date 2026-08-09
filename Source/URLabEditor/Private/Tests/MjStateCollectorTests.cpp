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

// The state-serialization IR, from the spec to the wire.
//
//  - FMjCanonicalName sanitize / part-segment stripping
//  - FMjStateCollector produces the IR fields the runtime paths carry
//  - Sensor values are emitted raw (IR == d->sensordata; GetReading == transform(IR))
//  - FMjMsgpackEncoder canonical schema + observation-level filter
//  - StructureVersion bumps on a producer-cache rebuild, not a plain collect
//
// Element state is produced by the collector rather than by the elements, so
// what these assert against is the collector's dispatch on element family and
// the compiled model it reads through -- not a per-class override.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "MjTestHelpers.h"
#include "State/MjCanonicalName.h"
#include "State/MjStateCollector.h"
#include "State/MjMsgpackEncoder.h"
#include "State/MjStateTypes.h"
#include "State/MjObservationLevel.h"
#include "Bridge/RpcDispatcher.h"
#include "Bridge/BridgeServer.h"
#include "Transport/SnapshotPublisher.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Elements/MjActuatorRuntime.h"
#include "MuJoCo/Elements/MjSensorRuntime.h"
#include "MuJoCo/Gen/Elements/Actuators/MjActuator.gen.h"
#include "MuJoCo/Gen/Elements/MjModel.gen.h"
#include "MuJoCo/Gen/Elements/Actuators/MjMotor.gen.h"
#include "Transport/RosPublishTransport.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"

using EObs = EObservationLevel;

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
		if (!S.Init([](FMjUESession& Sess) { Sess.Joint->SetType(EMjJointType::hinge); }))
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
			TestEqual(TEXT("hinge type"), (int)J.Type, (int)EMjJointType::hinge);
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
		if (!S.Init([](FMjUESession& Sess) { Sess.Joint->SetType(EMjJointType::free); }))
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
			TestEqual(TEXT("free type"), (int)J.Type, (int)EMjJointType::free);
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
			Sess.Joint->SetType(EMjJointType::slide);
			// <motor> is a child of the spec's <actuator> section, so the
			// section is authored first and the motor into it.
			UMjActuator* Section = Sess.Add<UMjActuator>(Sess.Robot->Spec);
			if (Section == nullptr)
				return;
			if (UMjMotor* Motor = Sess.Add<UMjMotor>(Section, TEXT("TestActuator")))
				Motor->SetJoint(Sess.Joint->MjName.GetValue());
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
	TArray<UMjNodeComponent*> Acts = Art->GetActuators();
	const int32 Aid = S.MjId(mjOBJ_ACTUATOR, TEXT("TestActuator"));
	if (Acts.Num() == 0 || !Acts[0] || Aid < 0)
	{
		AddInfo(TEXT("Skipping ActuatorParity: actuator did not bind"));
		S.Cleanup();
		return true;
	}
	TestEqual(TEXT("the indexed actuator holds the compiled id its name resolves to"),
		Acts[0]->GetBoundId().Get(-1), Aid);

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
			// The transmission target is read out of the compiled model, so it
			// names the joint the actuator actually drives whether the spec
			// spelled it out or inherited it from a default class.
			TestEqual(TEXT("target joint is the compiled transmission target"),
				Act.TargetJoint, FName(TEXT("TestJoint")));
			TestEqual(TEXT("ctrl matches d->ctrl"), Act.Ctrl, (double)d->ctrl[Aid], 1e-9);
			TestEqual(TEXT("force matches d->actuator_force"),
				Act.Force, (double)d->actuator_force[Aid], 1e-9);
			const int ActAddr = (m->actuator_actadr && m->actuator_actadr[Aid] >= 0)
								  ? m->actuator_actadr[Aid]
								  : -1;
			const double ExpectedAct = (ActAddr >= 0 && ActAddr < m->na) ? d->act[ActAddr] : 0.0;
			TestEqual(TEXT("act matches d->act (0 when stateless)"), Act.Act, ExpectedAct, 1e-9);
		}
	}
	TestTrue(TEXT("actuator present in IR"), bFound);

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 5. Sensor values are emitted raw: IR Values == d->sensordata (MuJoCo SI),
//    while GetReading() applies the MuJoCo -> UE transform on top. For a
//    framequat the quaternion reorder makes the two provably differ, proving the
//    transform no longer contaminates the IR.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStateSensorRawParity,
	"URLab.State.SensorRawParity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStateSensorRawParity::RunTest(const FString& Parameters)
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
		S.Manager->PhysicsEngine->ForwardSync();
	}

	UMjNodeComponent* Sensor = S.Robot->GetSensor(TEXT("fq"));
	if (!Sensor || !Sensor->GetBoundId().IsSet())
	{
		AddInfo(TEXT("Skipping SensorRawParity: no bound sensor"));
		S.Cleanup();
		return true;
	}
	const int32 SensorId = Sensor->GetBoundId().GetValue();

	const TArray<float> Reading = UMjSensorRuntime::GetReading(Sensor);

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

	// The IR carries the raw MuJoCo sensordata slice verbatim (double precision).
	const int SensorAdr = m->sensor_adr[SensorId];
	const int SensorDim = m->sensor_dim[SensorId];
	TestEqual(TEXT("IR sensor dim == model sensor_dim"), IRValues->Num(), SensorDim);
	if (IRValues->Num() == SensorDim)
	{
		for (int32 i = 0; i < SensorDim; ++i)
			TestEqual(*FString::Printf(TEXT("IR value[%d] == raw d->sensordata"), i),
				(*IRValues)[i], d->sensordata[SensorAdr + i], 1e-12);
	}

	// GetReading() applies the MuJoCo -> UE transform on top of the raw IR. For a
	// framequat (wxyz -> UE xyzw with handedness flip) the two must differ, which
	// proves the IR is genuinely raw and the transform lives only on the getter.
	TestEqual(TEXT("GetReading dim == IR dim"), Reading.Num(), IRValues->Num());
	if (Reading.Num() == 4 && IRValues->Num() == 4)
	{
		const double mj_w = (*IRValues)[0], mj_x = (*IRValues)[1],
					 mj_y = (*IRValues)[2], mj_z = (*IRValues)[3];
		TestEqual(TEXT("GetReading[0] == -mjX"), (double)Reading[0], -mj_x, 1e-5);
		TestEqual(TEXT("GetReading[1] == mjY"), (double)Reading[1], mj_y, 1e-5);
		TestEqual(TEXT("GetReading[2] == -mjZ"), (double)Reading[2], -mj_z, 1e-5);
		TestEqual(TEXT("GetReading[3] == mjW"), (double)Reading[3], mj_w, 1e-5);
		// The reorder must actually move data: IR[0] is mjW, GetReading[0] is -mjX.
		TestTrue(TEXT("IR differs from GetReading (transform is real)"),
			FMath::Abs((*IRValues)[0] - (double)Reading[0]) > 1e-6);
	}

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 5b. Gyro + accel raw values reach the IR unchanged (MuJoCo SI, no Y-negation),
//     and FillImu emits them verbatim into the Imu components. MuJoCo's gyro/
//     accel convention already equals ROS's, so a correct IMU needs the raw IR
//     and no flip anywhere on the ROS path.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStateImuRawToFillImu,
	"URLab.State.ImuRawToFillImu",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStateImuRawToFillImu::RunTest(const FString& Parameters)
{
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
		"    <gyro name=\"g1\" site=\"s1\"/>"
		"    <accelerometer name=\"a1\" site=\"s1\"/>"
		"  </sensor>"
		"</mujoco>");

	FMjXmlImportSession S;
	if (!S.Init(Xml) || !S.Compile())
	{
		AddInfo(FString::Printf(TEXT("Skipping ImuRawToFillImu: %s"), *S.LastError));
		return true;
	}

	mjModel* m = S.Model();
	mjData* d = S.Data();
	if (!m || !d || !S.Robot)
	{
		AddInfo(TEXT("Skipping ImuRawToFillImu: no model/robot"));
		S.Cleanup();
		return true;
	}

	// Give the base a nonzero body angular velocity so the gyro reads a distinct,
	// asymmetric value per axis (proving no accidental Y flip), then recompute so
	// the vel/acc sensor stages populate d->sensordata.
	if (m->nv >= 6)
	{
		d->qvel[3] = 0.3; // body wx
		d->qvel[4] = 0.5; // body wy
		d->qvel[5] = 0.7; // body wz
	}
	mj_forward(m, d);

	// Raw slices are read via each sensor's bound id: compiled sensor names carry
	// an articulation prefix, so a bare mj_name2id lookup would miss them.
	auto RawSliceFor = [&](const TCHAR* MjName) {
		TArray<double> Out;
		UMjNodeComponent* Sen = S.Robot->GetSensor(MjName);
		if (Sen == nullptr || !Sen->GetBoundId().IsSet())
			return Out;
		const int Id = Sen->GetBoundId().GetValue();
		const int Adr = m->sensor_adr[Id];
		const int Dim = m->sensor_dim[Id];
		for (int i = 0; i < Dim; ++i)
			Out.Add(d->sensordata[Adr + i]);
		return Out;
	};
	const TArray<double> RawGyro = RawSliceFor(TEXT("g1"));
	const TArray<double> RawAccel = RawSliceFor(TEXT("a1"));
	if (RawGyro.Num() != 3 || RawAccel.Num() != 3)
	{
		AddInfo(TEXT("Skipping ImuRawToFillImu: gyro/accel did not bind"));
		S.Cleanup();
		return true;
	}

	FMjStateCollector& C = S.Manager->GetStateCollector();
	C.Init(S.Manager);
	C.RebuildProducerCacheGameThread();
	const FMjStateSnapshot& Snap = C.Collect(m, d, 0);

	// The IR must carry each sensor's raw slice verbatim under the right semantic.
	const FMjSensorState* IRGyro = nullptr;
	const FMjSensorState* IRAccel = nullptr;
	for (const FMjArticulationState& AS : Snap.Articulations)
	{
		for (const FMjSensorState& Sen : AS.Sensors)
		{
			if (Sen.Semantic == EMjSensorSemantic::Gyro)
				IRGyro = &Sen;
			else if (Sen.Semantic == EMjSensorSemantic::Accel)
				IRAccel = &Sen;
		}
	}

	if (!IRGyro || !IRAccel)
	{
		AddError(TEXT("gyro/accel missing from the IR"));
		S.Cleanup();
		return false;
	}

	TestEqual(TEXT("IR gyro dim == 3"), IRGyro->Values.Num(), 3);
	TestEqual(TEXT("IR accel dim == 3"), IRAccel->Values.Num(), 3);
	if (IRGyro->Values.Num() == 3 && IRAccel->Values.Num() == 3)
	{
		for (int32 i = 0; i < 3; ++i)
		{
			TestEqual(*FString::Printf(TEXT("IR gyro[%d] == raw"), i),
				IRGyro->Values[i], RawGyro[i], 1e-12);
			TestEqual(*FString::Printf(TEXT("IR accel[%d] == raw"), i),
				IRAccel->Values[i], RawAccel[i], 1e-12);
		}
	}

	// FillImu (the ROS Imu producer, pure and compiled in every config) emits the
	// IR values with no coordinate flip: the Imu carries raw MuJoCo == ROS SI.
	const FMjArticulationState& Art = Snap.Articulations[0];
	double Ang[3] = {0, 0, 0};
	double Acc[3] = {0, 0, 0};
	bool bHasAng = false;
	bool bHasAcc = false;
	const bool bHas = UURLabRosPublishTransport::FillImu(Art, Ang, bHasAng, Acc, bHasAcc);
	TestTrue(TEXT("FillImu reports an Imu"), bHas);
	TestTrue(TEXT("FillImu has angular velocity"), bHasAng);
	TestTrue(TEXT("FillImu has linear acceleration"), bHasAcc);
	for (int32 i = 0; i < 3; ++i)
	{
		TestEqual(*FString::Printf(TEXT("FillImu angular[%d] unflipped"), i),
			Ang[i], RawGyro[i], 1e-12);
		TestEqual(*FString::Printf(TEXT("FillImu linear[%d] unflipped"), i),
			Acc[i], RawAccel[i], 1e-12);
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

// ---------------------------------------------------------------------------
// 8. Byte fan-out delivers snapshots to registered publishers, and the
//    bPublishersPaused gate suppresses delivery.
// ---------------------------------------------------------------------------
namespace
{
struct FFakeSnapshotPublisher : public IMjSnapshotPublisher
{
	int32 Count = 0;
	int32 LastBytes = 0;
	virtual void PublishSnapshot(const TArray<uint8>& Bytes) override
	{
		++Count;
		LastBytes = Bytes.Num();
	}
};
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStateByteFanOutPause,
	"URLab.State.ByteFanOutPause",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStateByteFanOutPause::RunTest(const FString& Parameters)
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

	FFakeSnapshotPublisher Fake;
	S.Manager->RegisterSnapshotPublisher(&Fake, S.Manager);

	// Live (unpaused): the fan-out encodes and delivers bytes to the publisher.
	S.Manager->bPublishersPaused.store(false);
	S.Manager->FanOutStateSnapshot(m, d);
	TestTrue(TEXT("publisher receives bytes when live"), Fake.Count >= 1);
	TestTrue(TEXT("delivered payload is non-empty"), Fake.LastBytes > 0);

	// Paused (Direct/Puppet): the byte fan-out is suppressed.
	const int32 Before = Fake.Count;
	S.Manager->bPublishersPaused.store(true);
	S.Manager->FanOutStateSnapshot(m, d);
	TestEqual(TEXT("no byte delivery while paused"), Fake.Count, Before);

	S.Manager->bPublishersPaused.store(false);
	S.Manager->UnregisterSnapshotPublisher(&Fake);
	S.Cleanup();
	return true;
}
