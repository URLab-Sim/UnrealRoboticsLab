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
// MjRosLinkTests.cpp
//
// Link + behaviour tests for the in-process ROS 2 output:
//  - FURLabRosContext init / shutdown round-trip and idempotent double-init
//  - UURLabRosPublishTransport creates JointState publishers and publishes a
//    collected IR against real rcl without error
//  - FillJointState maps the IR to the JointState parallel arrays correctly
//
// The whole file is compiled only when ROS 2 is linked (URLAB_WITH_ROS2). The
// context-dependent tests early-return true with a log when no live ROS context
// is available, so CI stays green on machines without a running DDS. Byte-level
// rosidl fill correctness over the wire is covered by the standalone harness in
// ros/urlab_ros_ws.
// ============================================================================

#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Transport/RosContext.h"
#include "Transport/RosPublishTransport.h"
#include "Transport/RosRpcTransport.h"
#include "Transport/RosOutputProvider.h"
#include "Transport/SnapshotPublisher.h"
#include "State/MjStateTypes.h"
#include "State/MjCanonicalName.h"
#include "MjTestHelpers.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "UserChannels/MjUserChannelComponent.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/ControlOwnership.h"
#include "Bridge/RpcDispatcher.h"
#include "Dom/JsonValue.h"
#include "Dom/JsonObject.h"
#include <atomic>

namespace
{
// A minimal collected IR: one articulation with two hinge joints and a free base,
// so the DOF flattening (1/1 hinge, 7/6 free) is exercised without a live model.
FMjStateSnapshot MakeSnapshot()
{
	FMjStateSnapshot Snap;
	Snap.StructureVersion = 1;
	Snap.Clock.SimSec = 3;
	Snap.Clock.SimNsec = 500000000;

	FMjArticulationState Art;
	Art.Name = FName(TEXT("go2"));

	FMjJointState Hip;
	Hip.Name = FName(TEXT("fl_hip"));
	Hip.Type = EMjJointType::Hinge;
	Hip.QPos = {0.10};
	Hip.QVel = {1.10};
	Art.Joints.Add(Hip);

	FMjJointState Knee;
	Knee.Name = FName(TEXT("fl_knee"));
	Knee.Type = EMjJointType::Hinge;
	Knee.QPos = {0.20};
	Knee.QVel = {1.20};
	Art.Joints.Add(Knee);

	FMjJointState Root;
	Root.Name = FName(TEXT("root"));
	Root.Type = EMjJointType::Free;
	Root.QPos = {0.0, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0};
	Root.QVel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	Art.Joints.Add(Root);

	Snap.Articulations.Add(Art);
	return Snap;
}
}  // namespace

// ---------------------------------------------------------------------------
// 1. Context round-trip + idempotent double-init
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosContextRoundTrip,
	"URLab.Ros.ContextRoundTrip",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosContextRoundTrip::RunTest(const FString& Parameters)
{
	FURLabRosContext& Ctx = FURLabRosContext::Get();
	Ctx.Initialize();
	if (!Ctx.IsAvailable())
	{
		UE_LOG(LogTemp, Display,
			TEXT("URLab.Ros.ContextRoundTrip: no live ROS context; skipping."));
		return true;
	}

	UrlabRclContext* const Handle = Ctx.GetHandle();
	TestNotNull(TEXT("context handle after init"), Handle);

	// Double-init is a no-op: the same handle, no second context.
	Ctx.Initialize();
	TestTrue(TEXT("double-init keeps the same handle"), Ctx.GetHandle() == Handle);

	// Teardown then bring it back, leaving the process context available for the
	// rest of the session.
	Ctx.Shutdown();
	TestFalse(TEXT("unavailable after shutdown"), Ctx.IsAvailable());
	Ctx.Initialize();
	TestTrue(TEXT("available again after re-init"), Ctx.IsAvailable());

	return true;
}

// ---------------------------------------------------------------------------
// 2. Publisher create + publish a collected IR against real rcl
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosPublishJointState,
	"URLab.Ros.PublishJointState",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosPublishJointState::RunTest(const FString& Parameters)
{
	FURLabRosContext::Get().Initialize();
	if (!FURLabRosContext::Get().IsAvailable())
	{
		UE_LOG(LogTemp, Display,
			TEXT("URLab.Ros.PublishJointState: no live ROS context; skipping."));
		return true;
	}

	UURLabRosPublishTransport* Transport = NewObject<UURLabRosPublishTransport>();
	TestNotNull(TEXT("transport object"), Transport);
	TestTrue(TEXT("transport init"), Transport->TransportInit());

	const FMjStateSnapshot Snap = MakeSnapshot();
	// First call builds the publisher set; second reuses it (StructureVersion
	// unchanged). Neither should crash or tear the context down.
	Transport->PublishState(Snap);
	Transport->PublishState(Snap);
	TestTrue(TEXT("context still available after publish"),
		FURLabRosContext::Get().IsAvailable());

	Transport->TransportShutdown();
	return true;
}

// ---------------------------------------------------------------------------
// 3. FillJointState: names = part segments, arrays sized to joint DOF counts
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosFillJointState,
	"URLab.Ros.FillJointState",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosFillJointState::RunTest(const FString& Parameters)
{
	// Pure IR -> arrays; no live ROS context needed.
	const FMjStateSnapshot Snap = MakeSnapshot();
	const FMjArticulationState& Art = Snap.Articulations[0];

	TArray<FString> Names;
	TArray<double> Positions;
	TArray<double> Velocities;
	TArray<double> Efforts;
	UURLabRosPublishTransport::FillJointState(Art, Names, Positions, Velocities, Efforts);

	// Only 1-DoF joints are emitted; the free "root" joint is excluded so
	// name[i] aligns 1:1 with position[i] / velocity[i].
	TestEqual(TEXT("name count == 1-DoF joint count"), Names.Num(), 2);
	TestEqual(TEXT("name[0]"), Names[0], FString(TEXT("fl_hip")));
	TestEqual(TEXT("name[1]"), Names[1], FString(TEXT("fl_knee")));

	TestEqual(TEXT("positions aligned to names"), Positions.Num(), 2);
	TestEqual(TEXT("velocities aligned to names"), Velocities.Num(), 2);
	TestEqual(TEXT("position[0] is hip qpos"), Positions[0], 0.10);
	TestEqual(TEXT("velocity[1] is knee qvel"), Velocities[1], 1.20);

	return true;
}

// ---------------------------------------------------------------------------
// 4. FillImu: paired gyro+accel -> both fields; unpaired gyro -> angular only
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosFillImu,
	"URLab.Ros.FillImu",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosFillImu::RunTest(const FString& Parameters)
{
	// Pure IR -> Imu components; no live ROS context needed.

	// Paired gyro + accel on one art.
	{
		FMjArticulationState Art;
		Art.Name = FName(TEXT("go2"));
		FMjSensorState Gyro;
		Gyro.Semantic = EMjSensorSemantic::Gyro;
		Gyro.Values = {0.1, 0.2, 0.3};
		Art.Sensors.Add(Gyro);
		FMjSensorState Accel;
		Accel.Semantic = EMjSensorSemantic::Accel;
		Accel.Values = {1.0, 2.0, 3.0};
		Art.Sensors.Add(Accel);

		double Ang[3] = {0, 0, 0};
		double Acc[3] = {0, 0, 0};
		bool bHasAng = false;
		bool bHasAcc = false;
		const bool bHas = UURLabRosPublishTransport::FillImu(Art, Ang, bHasAng, Acc, bHasAcc);
		TestTrue(TEXT("paired imu present"), bHas);
		TestTrue(TEXT("paired has angular velocity"), bHasAng);
		TestTrue(TEXT("paired has linear acceleration"), bHasAcc);
		TestEqual(TEXT("gyro x"), Ang[0], 0.1);
		TestEqual(TEXT("gyro z"), Ang[2], 0.3);
		TestEqual(TEXT("accel z"), Acc[2], 3.0);
	}

	// Unpaired gyro: angular velocity only.
	{
		FMjArticulationState Art;
		Art.Name = FName(TEXT("go2"));
		FMjSensorState Gyro;
		Gyro.Semantic = EMjSensorSemantic::Gyro;
		Gyro.Values = {0.5, 0.6, 0.7};
		Art.Sensors.Add(Gyro);

		double Ang[3] = {0, 0, 0};
		double Acc[3] = {0, 0, 0};
		bool bHasAng = false;
		bool bHasAcc = false;
		const bool bHas = UURLabRosPublishTransport::FillImu(Art, Ang, bHasAng, Acc, bHasAcc);
		TestTrue(TEXT("gyro-only imu present"), bHas);
		TestTrue(TEXT("gyro-only has angular velocity"), bHasAng);
		TestFalse(TEXT("gyro-only has no linear acceleration"), bHasAcc);
		TestEqual(TEXT("gyro-only y"), Ang[1], 0.6);
	}

	// No imu sensors: nothing to publish.
	{
		FMjArticulationState Art;
		Art.Name = FName(TEXT("go2"));
		double Ang[3] = {0, 0, 0};
		double Acc[3] = {0, 0, 0};
		bool bHasAng = false;
		bool bHasAcc = false;
		const bool bHas = UURLabRosPublishTransport::FillImu(Art, Ang, bHasAng, Acc, bHasAcc);
		TestFalse(TEXT("no imu when art has no gyro/accel"), bHas);
	}

	return true;
}

// ---------------------------------------------------------------------------
// 5. FillClock: sec/nsec split matches AppendClockFields for the same sim time
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosFillClock,
	"URLab.Ros.FillClock",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosFillClock::RunTest(const FString& Parameters)
{
	// AppendClockFields (RpcDispatcher.cpp) splits a sim-time double this way; the
	// IR collector fills FMjClock identically and FillClock recombines to ns, so a
	// round-trip through FillClock must reproduce the same sec/nsec pair.
	const double SimTimeSec = 3.5;
	const int32 ExpectedSec = static_cast<int32>(SimTimeSec);
	const int32 ExpectedNsec = static_cast<int32>((SimTimeSec - ExpectedSec) * 1.0e9);

	FMjClock Clock;
	Clock.SimSec = ExpectedSec;
	Clock.SimNsec = ExpectedNsec;

	const int64 Ns = UURLabRosPublishTransport::FillClock(Clock);
	TestEqual(TEXT("clock sec matches AppendClockFields"),
		static_cast<int32>(Ns / 1000000000LL), ExpectedSec);
	TestEqual(TEXT("clock nsec matches AppendClockFields"),
		static_cast<int32>(Ns % 1000000000LL), ExpectedNsec);
	return true;
}

// ---------------------------------------------------------------------------
// 6. Publisher-set rebuild on StructureVersion change
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosPublisherRebuild,
	"URLab.Ros.PublisherRebuild",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosPublisherRebuild::RunTest(const FString& Parameters)
{
	FURLabRosContext::Get().Initialize();
	if (!FURLabRosContext::Get().IsAvailable())
	{
		UE_LOG(LogTemp, Display,
			TEXT("URLab.Ros.PublisherRebuild: no live ROS context; skipping."));
		return true;
	}

	UURLabRosPublishTransport* Transport = NewObject<UURLabRosPublishTransport>();
	TestTrue(TEXT("transport init"), Transport->TransportInit());

	// One art, version 1: one per-art publisher entry.
	const FMjStateSnapshot Snap1 = MakeSnapshot();
	Transport->PublishState(Snap1);
	TestEqual(TEXT("one art publisher after first publish"),
		Transport->GetArtPublisherCountForTest(), 1);
	TestEqual(TEXT("cached structure version tracks the snapshot"),
		Transport->GetCachedStructureVersionForTest(), 1u);

	// A same-version re-publish must NOT rebuild.
	Transport->PublishState(Snap1);
	TestEqual(TEXT("no rebuild on unchanged structure version"),
		Transport->GetArtPublisherCountForTest(), 1);

	// Add a second art and bump the version: the set rebuilds to match.
	FMjStateSnapshot Snap2 = MakeSnapshot();
	FMjArticulationState Art2;
	Art2.Name = FName(TEXT("arm"));
	FMjJointState J;
	J.Name = FName(TEXT("j0"));
	J.Type = EMjJointType::Hinge;
	J.QPos = {0.0};
	J.QVel = {0.0};
	Art2.Joints.Add(J);
	Snap2.Articulations.Add(Art2);
	Snap2.StructureVersion = 2;

	Transport->PublishState(Snap2);
	TestEqual(TEXT("two art publishers after registry grows"),
		Transport->GetArtPublisherCountForTest(), 2);
	TestEqual(TEXT("cached structure version follows the bump"),
		Transport->GetCachedStructureVersionForTest(), 2u);

	Transport->TransportShutdown();
	return true;
}

// ---------------------------------------------------------------------------
// 7. Direct-mode fan-out: ROS publishes every step (distinct consumer) while the
//    byte publishers stay paused (3.6 rule). A fake IMjSnapshotPublisher stands
//    in for the ZMQ / SHM byte streams.
// ---------------------------------------------------------------------------
namespace
{
struct FCountingSnapshotPublisher : public IMjSnapshotPublisher
{
	std::atomic<int32> Count{0};
	virtual void PublishSnapshot(const TArray<uint8>& /*Bytes*/) override
	{
		++Count;
	}
};
}  // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosDirectModeFanOut,
	"URLab.Ros.DirectModeFanOut",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosDirectModeFanOut::RunTest(const FString& Parameters)
{
	FURLabRosContext::Get().Initialize();
	if (!FURLabRosContext::Get().IsAvailable())
	{
		UE_LOG(LogTemp, Display,
			TEXT("URLab.Ros.DirectModeFanOut: no live ROS context; skipping."));
		return true;
	}

	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}

	// Register a ROS publish transport into the manager's fan-out set, exactly as
	// EnsureExternalTransportsBound does at runtime.
	UURLabRosPublishTransport* Ros = NewObject<UURLabRosPublishTransport>(S.Manager);
	TestTrue(TEXT("ros publish transport init"), Ros->TransportInit());
	S.Manager->ManagerOwnedPublishTransports.Add(Ros);

	// A fake byte publisher standing in for ZMQ / SHM.
	FCountingSnapshotPublisher Fake;
	S.Manager->RegisterSnapshotPublisher(&Fake, S.Manager);

	FURLabRpcDispatcher* Disp = S.Manager->GetStepDispatcher();
	if (!Disp)
	{
		AddError(TEXT("Manager has no StepDispatcher"));
		Ros->TransportShutdown();
		S.Manager->UnregisterSnapshotPublisher(&Fake);
		S.Cleanup();
		return false;
	}

	// Direct mode pauses the byte fan-out (bPublishersPaused = true).
	Disp->SetActiveStepMode(EStepMode::Direct);
	TestTrue(TEXT("direct mode pauses byte publishers"),
		S.Manager->bPublishersPaused.load());

	mjModel* m = S.Manager->PhysicsEngine->m_model;
	mjData* d = S.Manager->PhysicsEngine->m_data;
	const int64 Before = Ros->GetPublishStateCountForTest();

	S.Manager->FanOutStateSnapshot(m, d);

	TestEqual(TEXT("ROS published once despite pause (distinct consumer)"),
		Ros->GetPublishStateCountForTest() - Before, static_cast<int64>(1));
	TestEqual(TEXT("byte publisher received nothing while paused"),
		Fake.Count.load(), 0);

	// Restore Live so downstream tests see a clean cadence, then tear down.
	Disp->SetActiveStepMode(EStepMode::Live);
	Ros->TransportShutdown();
	S.Manager->UnregisterSnapshotPublisher(&Fake);
	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 8. ROS control ownership across surfaces: a ROS-sourced claim blocks an
//    RPC-session control write (and vice versa); a TTL frees the ROS claim.
//    Drives FMjControlOwnership + Dispatch, so no live ROS runtime is needed.
// ---------------------------------------------------------------------------
namespace
{
FString RosReplyField(const TSharedPtr<FJsonObject>& Reply, const TCHAR* Field)
{
	FString Out;
	if (Reply.IsValid())
		Reply->TryGetStringField(Field, Out);
	return Out;
}
}  // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosControlOwnershipAcrossSurfaces,
	"URLab.Ros.ControlOwnershipAcrossSurfaces",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosControlOwnershipAcrossSurfaces::RunTest(const FString& Parameters)
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

	AMjArticulation* Art = S.Manager->GetAllArticulations()[0];
	const FString ArtName = Art->GetName();
	const FName Key(*ArtName);
	const FString RosSrc = UURLabRosRpcTransport::RosControlSourceId();
	const FString RpcSrc = TEXT("rpc-session-src");

	auto Claim = [Disp, &ArtName](const FString& Source) {
		TSharedPtr<FJsonObject> R = MakeShared<FJsonObject>();
		R->SetStringField(TEXT("op"), TEXT("claim_control"));
		R->SetStringField(TEXT("session_id"), TEXT("test-session"));
		R->SetStringField(TEXT("source"), Source);
		R->SetStringField(TEXT("articulation"), ArtName);
		return Disp->Dispatch(R);
	};
	auto Release = [Disp, &ArtName](const FString& Source) {
		TSharedPtr<FJsonObject> R = MakeShared<FJsonObject>();
		R->SetStringField(TEXT("op"), TEXT("release_control"));
		R->SetStringField(TEXT("session_id"), TEXT("test-session"));
		R->SetStringField(TEXT("source"), Source);
		R->SetStringField(TEXT("articulation"), ArtName);
		return Disp->Dispatch(R);
	};
	auto SetTwist = [Disp, &ArtName](const FString& Source) {
		TSharedPtr<FJsonObject> R = MakeShared<FJsonObject>();
		R->SetStringField(TEXT("op"), TEXT("set_twist"));
		R->SetStringField(TEXT("session_id"), TEXT("test-session"));
		R->SetStringField(TEXT("source"), Source);
		R->SetStringField(TEXT("articulation"), ArtName);
		TArray<TSharedPtr<FJsonValue>> Lin;
		Lin.Add(MakeShared<FJsonValueNumber>(1.0));
		Lin.Add(MakeShared<FJsonValueNumber>(0.0));
		R->SetArrayField(TEXT("linear"), Lin);
		return Disp->Dispatch(R);
	};

	// ROS claims; an RPC-session control write is then rejected, naming ROS owner.
	TestEqual(TEXT("ROS claim ok"), RosReplyField(Claim(RosSrc), TEXT("op")),
		FString(TEXT("claim_control_ok")));
	{
		TSharedPtr<FJsonObject> R = SetTwist(RpcSrc);
		TestEqual(TEXT("RPC write rejected while ROS owns"),
			RosReplyField(R, TEXT("code")), FString(TEXT("not_control_owner")));
		TestEqual(TEXT("rejection names the ROS owner"),
			RosReplyField(R, TEXT("owner")), RosSrc);
	}

	// Hand the art to the RPC session; a ROS write is now the one rejected.
	TestEqual(TEXT("ROS release ok"), RosReplyField(Release(RosSrc), TEXT("op")),
		FString(TEXT("release_control_ok")));
	TestEqual(TEXT("RPC claim ok"), RosReplyField(Claim(RpcSrc), TEXT("op")),
		FString(TEXT("claim_control_ok")));
	{
		TSharedPtr<FJsonObject> R = SetTwist(RosSrc);
		TestEqual(TEXT("ROS write rejected while RPC owns"),
			RosReplyField(R, TEXT("code")), FString(TEXT("not_control_owner")));
		TestEqual(TEXT("rejection names the RPC owner"),
			RosReplyField(R, TEXT("owner")), RpcSrc);
	}

	// TTL frees a dropped ROS owner: past the TTL, another source can claim.
	FMjControlOwnership& Own = Disp->GetControlOwnership();
	Own.Reset();
	Own.SetClockOverrideForTest(0.0);
	FString Cur;
	TestTrue(TEXT("ROS claims with a TTL"),
		Own.Claim(Key, RosSrc, 5.0, false, Cur) == FMjControlOwnership::EClaimResult::Ok);
	Own.SetClockOverrideForTest(10.0);
	TestTrue(TEXT("TTL freed the ROS claim; RPC can claim"),
		Own.Claim(Key, RpcSrc, 0.0, false, Cur) == FMjControlOwnership::EClaimResult::Ok);
	Own.SetClockOverrideForTest(-1.0);

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 9. Mode gating: a marshalled ROS ctrl write is dropped outside Live mode and
//    applies in Live mode. Ownership is granted first so mode is the only gate;
//    no live ROS runtime is needed (HandleRosCtrl touches no rcl).
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosCtrlModeGating,
	"URLab.Ros.CtrlModeGating",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosCtrlModeGating::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init([](FMjUESession& Sess) {
			Sess.Joint->Type = EMjJointType::Slide;
			UMjActuator* A = NewObject<UMjActuator>(Sess.Robot, TEXT("TestActuator"));
			A->Type = EMjActuatorType::Position;
			A->TargetName = Sess.Joint->GetName();
			A->RegisterComponent();
			A->AttachToComponent(Sess.Robot->GetRootComponent(),
				FAttachmentTransformRules::KeepRelativeTransform);
		}))
	{
		AddInfo(FString::Printf(TEXT("Skipping CtrlModeGating: %s"), *S.LastError));
		return true;
	}

	mjModel* m = S.Manager->PhysicsEngine->GetModel();
	mjData* d = S.Manager->PhysicsEngine->GetData();
	if (!m || !d || m->nu == 0)
	{
		AddInfo(TEXT("Skipping CtrlModeGating: no actuators in compiled model"));
		S.Cleanup();
		return true;
	}

	FURLabRpcDispatcher* Disp = S.Manager->GetStepDispatcher();
	AMjArticulation* Art = S.Manager->GetAllArticulations()[0];
	const FString ArtName = Art->GetName();
	const FName Key(*ArtName);
	const FString RosSrc = UURLabRosRpcTransport::RosControlSourceId();

	UURLabRosRpcTransport* Ros = NewObject<UURLabRosRpcTransport>(S.Manager);
	Ros->SetOwningBridge(S.Manager->BridgeServer);

	// Grant ROS ownership so ownership never blocks; the mode is the only gate.
	FString Cur;
	Disp->GetControlOwnership().Claim(Key, RosSrc, 0.0, false, Cur);

	// Direct mode: the write is dropped, so d->ctrl stays at its initial value.
	Disp->SetActiveStepMode(EStepMode::Direct);
	Ros->ApplyRosCtrlForTest(ArtName, {0.5});
	Art->ApplyControls(/*bSkipController=*/true);
	TestEqual(TEXT("direct-mode ROS ctrl dropped"), (double)d->ctrl[0], 0.0, 1e-6);

	// Live mode: the same write reaches the actuator staging and lands.
	Disp->SetActiveStepMode(EStepMode::Live);
	Ros->ApplyRosCtrlForTest(ArtName, {0.5});
	Art->ApplyControls(/*bSkipController=*/true);
	TestEqual(TEXT("live-mode ROS ctrl applies"), (double)d->ctrl[0], 0.5, 1e-6);

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 10. Availability-gated wire test: publish a Float64MultiArray on
//     /<art>/cmd_ctrl via rcl and assert the staged ctrl value lands. Requires a
//     live ROS context (skips otherwise) and a compiled actuator.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosCtrlWire,
	"URLab.Ros.CtrlWire",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosCtrlWire::RunTest(const FString& Parameters)
{
	FURLabRosContext::Get().Initialize();
	if (!FURLabRosContext::Get().IsAvailable())
	{
		UE_LOG(LogTemp, Display, TEXT("URLab.Ros.CtrlWire: no live ROS context; skipping."));
		return true;
	}

	FMjUESession S;
	if (!S.Init([](FMjUESession& Sess) {
			Sess.Joint->Type = EMjJointType::Slide;
			UMjActuator* A = NewObject<UMjActuator>(Sess.Robot, TEXT("TestActuator"));
			A->Type = EMjActuatorType::Position;
			A->TargetName = Sess.Joint->GetName();
			A->RegisterComponent();
			A->AttachToComponent(Sess.Robot->GetRootComponent(),
				FAttachmentTransformRules::KeepRelativeTransform);
		}))
	{
		AddInfo(FString::Printf(TEXT("Skipping CtrlWire: %s"), *S.LastError));
		return true;
	}

	mjModel* m = S.Manager->PhysicsEngine->GetModel();
	mjData* d = S.Manager->PhysicsEngine->GetData();
	if (!m || !d || m->nu == 0)
	{
		AddInfo(TEXT("Skipping CtrlWire: no actuators in compiled model"));
		S.Cleanup();
		return true;
	}

	FURLabRpcDispatcher* Disp = S.Manager->GetStepDispatcher();
	AMjArticulation* Art = S.Manager->GetAllArticulations()[0];
	const FString ArtName = Art->GetName();
	const FName Key(*ArtName);
	const FString RosSrc = UURLabRosRpcTransport::RosControlSourceId();

	UURLabRosRpcTransport* Ros = NewObject<UURLabRosRpcTransport>(S.Manager);
	Ros->SetOwningBridge(S.Manager->BridgeServer);

	// Own the art as the ROS source and stay in Live mode so the marshalled write
	// is applied rather than dropped.
	FString Cur;
	Disp->GetControlOwnership().Claim(Key, RosSrc, 0.0, false, Cur);
	Disp->SetActiveStepMode(EStepMode::Live);

	const FString Segment = FMjCanonicalName::ArtSegment(Art).ToString();
	const FString Topic = FString::Printf(TEXT("/%s/cmd_ctrl"), *Segment);

	const bool bFired = Ros->PublishAndPumpCtrlForTest(Topic, {0.42});
	TestTrue(TEXT("ROS cmd_ctrl delivered over the wire"), bFired);

	// The callback staged the value on the actuator's NetworkValue; a step copies
	// it into d->ctrl (mirroring the live physics tick).
	Art->ApplyControls(/*bSkipController=*/true);
	TestEqual(TEXT("wire ctrl landed in d->ctrl"), (double)d->ctrl[0], 0.42, 1e-6);

	Ros->TransportShutdown();
	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 11. Total sensor routing over the wire: an art carrying one of each typed
//     sensor plus an unmapped one builds and publishes through the provider set
//     against real rcl without error (Wrench / Range / MagneticField / Twist /
//     Float64MultiArray create + publish paths). Availability-gated.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosSensorRoutingWire,
	"URLab.Ros.SensorRoutingWire",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosSensorRoutingWire::RunTest(const FString& Parameters)
{
	FURLabRosContext::Get().Initialize();
	if (!FURLabRosContext::Get().IsAvailable())
	{
		UE_LOG(LogTemp, Display,
			TEXT("URLab.Ros.SensorRoutingWire: no live ROS context; skipping."));
		return true;
	}

	auto AddSensor = [](FMjArticulationState& Art, const TCHAR* Name,
		EMjSensorSemantic Sem, TArray<double> Values)
	{
		FMjSensorState S;
		S.Name = FName(Name);
		S.Semantic = Sem;
		S.Values = MoveTemp(Values);
		Art.Sensors.Add(S);
	};

	FMjStateSnapshot Snap;
	Snap.StructureVersion = 1;
	FMjArticulationState Art;
	Art.Name = FName(TEXT("go2"));
	AddSensor(Art, TEXT("ft_force"), EMjSensorSemantic::Force, {1.0, 2.0, 3.0});
	AddSensor(Art, TEXT("ft_torque"), EMjSensorSemantic::Torque, {4.0, 5.0, 6.0});
	AddSensor(Art, TEXT("front_range"), EMjSensorSemantic::Rangefinder, {0.42});
	AddSensor(Art, TEXT("mag0"), EMjSensorSemantic::Magnetometer, {0.1, 0.2, 0.3});
	AddSensor(Art, TEXT("base_vel"), EMjSensorSemantic::Velocity, {0.5, 0.0, 0.0});
	AddSensor(Art, TEXT("belly_touch"), EMjSensorSemantic::Touch, {1.0});
	Snap.Articulations.Add(Art);

	UURLabRosPublishTransport* Transport = NewObject<UURLabRosPublishTransport>();
	TestTrue(TEXT("transport init"), Transport->TransportInit());

	Transport->PublishState(Snap);
	Transport->PublishState(Snap);

	// Every registered provider instantiated, and the run did not tear the context
	// down (the wrench/range/mag/twist/multiarray wire paths all succeeded).
	TestEqual(TEXT("all registered providers built"),
		Transport->GetProviderCountForTest(), FMjRosOutputRegistry::Get().Num());
	TestTrue(TEXT("context still available after routing publish"),
		FURLabRosContext::Get().IsAvailable());

	Transport->TransportShutdown();
	return true;
}

// ---------------------------------------------------------------------------
// 12. State-estimation outputs over the wire: a free-base art builds + publishes
//     nav_msgs/Odometry (/<art>/odom) and geometry_msgs/PoseWithCovarianceStamped
//     (/<art>/pose), and the REP-105 map->odom->world static chain publishes on
//     /tf_static, all against real rcl without tearing the context down.
//     Availability-gated.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosStateEstimationWire,
	"URLab.Ros.StateEstimationWire",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosStateEstimationWire::RunTest(const FString& Parameters)
{
	FURLabRosContext::Get().Initialize();
	if (!FURLabRosContext::Get().IsAvailable())
	{
		UE_LOG(LogTemp, Display,
			TEXT("URLab.Ros.StateEstimationWire: no live ROS context; skipping."));
		return true;
	}

	// A free-base art with a matching base body and a genuinely asymmetric velocity
	// (world linear, body angular) so the Odometry twist rotation path runs live.
	FMjStateSnapshot Snap;
	Snap.StructureVersion = 1;
	FMjArticulationState Art;
	Art.Name = FName(TEXT("go2"));

	FMjBodyState Base;
	Base.Name = FName(TEXT("trunk"));
	Base.Xpos[0] = 0.0; Base.Xpos[1] = 0.0; Base.Xpos[2] = 0.5;
	Base.Xquat[0] = 1.0;
	Art.Bodies.Add(Base);

	FMjJointState Free;
	Free.Name = FName(TEXT("root"));
	Free.Type = EMjJointType::Free;
	Free.QPos = {0.0, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0};
	Free.QVel = {0.3, 0.0, 0.0, 0.0, 0.0, 0.5};
	Art.Joints.Add(Free);

	Snap.Articulations.Add(Art);

	UURLabRosPublishTransport* Transport = NewObject<UURLabRosPublishTransport>();
	TestTrue(TEXT("transport init"), Transport->TransportInit());

	Transport->PublishState(Snap);
	Transport->PublishState(Snap);

	TestEqual(TEXT("all registered providers built"),
		Transport->GetProviderCountForTest(), FMjRosOutputRegistry::Get().Num());
	TestTrue(TEXT("context still available after state-estimation publish"),
		FURLabRosContext::Get().IsAvailable());

	Transport->TransportShutdown();
	return true;
}

// ---------------------------------------------------------------------------
// 13. User channels over ROS: the "user_channels" provider is registered, and a
//     snapshot carrying a Bool + a Transform art channel plus a scene channel
//     builds + publishes its typed topics (std_msgs/Bool, geometry_msgs/Pose
//     Stamped, ...) against real rcl without tearing the context down.
//     Registry check runs unconditionally; the wire check is availability-gated.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosUserChannelsWire,
	"URLab.Ros.UserChannelsWire",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosUserChannelsWire::RunTest(const FString& Parameters)
{
	// Registry: the provider self-registers in every configuration.
	TestTrue(TEXT("user_channels provider registered"),
		FMjRosOutputRegistry::Get().GetRegisteredNames().Contains(FName(TEXT("user_channels"))));

	FURLabRosContext::Get().Initialize();
	if (!FURLabRosContext::Get().IsAvailable())
	{
		UE_LOG(LogTemp, Display,
			TEXT("URLab.Ros.UserChannelsWire: no live ROS context; skipping wire check."));
		return true;
	}

	FMjStateSnapshot Snap;
	Snap.StructureVersion = 1;

	FMjArticulationState Art;
	Art.Name = FName(TEXT("go2"));

	FMjUserChannel Done;
	Done.Name = FName(TEXT("task_done"));
	Done.Kind = EMjUserChannelKind::Bool;
	Done.Values = {1.0};
	Art.UserChannels.Add(Done);

	FMjUserChannel Target;
	Target.Name = FName(TEXT("target"));
	Target.Kind = EMjUserChannelKind::Transform;
	Target.Values = {1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0};  // pos + quat wxyz
	Art.UserChannels.Add(Target);

	Snap.Articulations.Add(Art);

	FMjUserChannel Phase;
	Phase.Name = FName(TEXT("episode_phase"));
	Phase.Kind = EMjUserChannelKind::Scalar;
	Phase.Values = {2.0};
	Snap.UserChannels.Add(Phase);

	UURLabRosPublishTransport* Transport = NewObject<UURLabRosPublishTransport>();
	TestTrue(TEXT("transport init"), Transport->TransportInit());

	Transport->PublishState(Snap);
	Transport->PublishState(Snap);

	TestEqual(TEXT("all registered providers built"),
		Transport->GetProviderCountForTest(), FMjRosOutputRegistry::Get().Num());
	TestTrue(TEXT("context still available after user-channel publish"),
		FURLabRosContext::Get().IsAvailable());

	Transport->TransportShutdown();
	return true;
}

// ---------------------------------------------------------------------------
// 14. claim_control as a ROS service: a service-sourced claim (routed through
//     Dispatch with the ROS source preset) claims the art, an RPC-session write
//     is then rejected naming the ROS owner, and once the RPC session owns it a
//     second ROS service claim fails. Drives Dispatch only; no live ROS runtime.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosClaimService,
	"URLab.Ros.ClaimService",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosClaimService::RunTest(const FString& Parameters)
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

	AMjArticulation* Art = S.Manager->GetAllArticulations()[0];
	const FString ArtName = Art->GetName();
	const FString RosSrc = UURLabRosRpcTransport::RosControlSourceId();
	const FString RpcSrc = TEXT("rpc-session-src");

	UURLabRosRpcTransport* Ros = NewObject<UURLabRosRpcTransport>(S.Manager);
	Ros->SetOwningBridge(S.Manager->BridgeServer);

	auto SetTwist = [Disp, &ArtName](const FString& Source) {
		TSharedPtr<FJsonObject> R = MakeShared<FJsonObject>();
		R->SetStringField(TEXT("op"), TEXT("set_twist"));
		R->SetStringField(TEXT("session_id"), TEXT("test-session"));
		R->SetStringField(TEXT("source"), Source);
		R->SetStringField(TEXT("articulation"), ArtName);
		TArray<TSharedPtr<FJsonValue>> Lin;
		Lin.Add(MakeShared<FJsonValueNumber>(1.0));
		Lin.Add(MakeShared<FJsonValueNumber>(0.0));
		R->SetArrayField(TEXT("linear"), Lin);
		return Disp->Dispatch(R);
	};

	// The ROS claim_control service claims the art.
	TestTrue(TEXT("ROS claim service succeeds"),
		Ros->ApplyRosClaimReleaseForTest(ArtName, /*bClaim=*/true));

	// An RPC-session write is now rejected, naming the ROS owner.
	{
		TSharedPtr<FJsonObject> R = SetTwist(RpcSrc);
		TestEqual(TEXT("RPC write rejected while ROS service owns"),
			RosReplyField(R, TEXT("code")), FString(TEXT("not_control_owner")));
		TestEqual(TEXT("rejection names the ROS owner"),
			RosReplyField(R, TEXT("owner")), RosSrc);
	}

	// The ROS release service frees it; the RPC session then claims it.
	TestTrue(TEXT("ROS release service succeeds"),
		Ros->ApplyRosClaimReleaseForTest(ArtName, /*bClaim=*/false));
	{
		TSharedPtr<FJsonObject> R = MakeShared<FJsonObject>();
		R->SetStringField(TEXT("op"), TEXT("claim_control"));
		R->SetStringField(TEXT("session_id"), TEXT("test-session"));
		R->SetStringField(TEXT("source"), RpcSrc);
		R->SetStringField(TEXT("articulation"), ArtName);
		TestEqual(TEXT("RPC claim ok"), RosReplyField(Disp->Dispatch(R), TEXT("op")),
			FString(TEXT("claim_control_ok")));
	}

	// A second ROS service claim now fails: the RPC session owns the art.
	TestFalse(TEXT("ROS claim service fails when RPC owns"),
		Ros->ApplyRosClaimReleaseForTest(ArtName, /*bClaim=*/true));

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 15. JointState jog input: a joint_command naming an actuator's canonical joint
//     is dropped outside Live mode and stages the actuator's position target in
//     Live mode. Ownership is granted first so mode is the only gate; drives
//     HandleRosJointCommand directly (no live ROS runtime needed).
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosJointCommandJog,
	"URLab.Ros.JointCommandJog",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosJointCommandJog::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init([](FMjUESession& Sess) {
			Sess.Joint->Type = EMjJointType::Slide;
			UMjActuator* A = NewObject<UMjActuator>(Sess.Robot, TEXT("TestActuator"));
			A->Type = EMjActuatorType::Position;
			A->TargetName = Sess.Joint->GetName();
			A->RegisterComponent();
			A->AttachToComponent(Sess.Robot->GetRootComponent(),
				FAttachmentTransformRules::KeepRelativeTransform);
		}))
	{
		AddInfo(FString::Printf(TEXT("Skipping JointCommandJog: %s"), *S.LastError));
		return true;
	}

	mjModel* m = S.Manager->PhysicsEngine->GetModel();
	mjData* d = S.Manager->PhysicsEngine->GetData();
	if (!m || !d || m->nu == 0)
	{
		AddInfo(TEXT("Skipping JointCommandJog: no actuators in compiled model"));
		S.Cleanup();
		return true;
	}

	FURLabRpcDispatcher* Disp = S.Manager->GetStepDispatcher();
	AMjArticulation* Art = S.Manager->GetAllArticulations()[0];
	const FString ArtName = Art->GetName();
	const FName Key(*ArtName);
	const FString RosSrc = UURLabRosRpcTransport::RosControlSourceId();

	// The jog names the joint the actuator drives (not the actuator's own name),
	// resolved through the same canonical-name convention JointState output uses.
	UMjActuator* Act = Art->GetActuators()[0];
	const FString JointName = FMjCanonicalName::PartSegment(Art, Act->TargetName).ToString();

	UURLabRosRpcTransport* Ros = NewObject<UURLabRosRpcTransport>(S.Manager);
	Ros->SetOwningBridge(S.Manager->BridgeServer);

	// Grant ROS ownership so the mode is the only gate.
	FString Cur;
	Disp->GetControlOwnership().Claim(Key, RosSrc, 0.0, false, Cur);

	// Direct mode: the jog is dropped, so d->ctrl stays at its initial value.
	Disp->SetActiveStepMode(EStepMode::Direct);
	Ros->ApplyRosJointCommandForTest(ArtName, {JointName}, {0.5});
	Art->ApplyControls(/*bSkipController=*/true);
	TestEqual(TEXT("direct-mode joint_command dropped"), (double)d->ctrl[0], 0.0, 1e-6);

	// Live mode: the same jog stages the actuator's position target.
	Disp->SetActiveStepMode(EStepMode::Live);
	Ros->ApplyRosJointCommandForTest(ArtName, {JointName}, {0.5});
	Art->ApplyControls(/*bSkipController=*/true);
	TestEqual(TEXT("live-mode joint_command applies"), (double)d->ctrl[0], 0.5, 1e-6);

	S.Cleanup();
	return true;
}

#endif  // URLAB_WITH_ROS2
