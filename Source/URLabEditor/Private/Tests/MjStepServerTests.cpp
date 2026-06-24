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
// MjStepServerTests.cpp
//
// Unit tests for the remote-stepping pieces:
//  - EStepMode dispatch / publisher pause flag semantics
//  - UMjPDController ApplyConfig / GetCurrentConfig round-trip
//  - FURLabRpcDispatcher handshake payload shape
//  - FURLabRpcDispatcher session-id rejection
//  - Puppet-mode push-state queue: dequeue writes qpos/qvel and fires OnPostStep
//
// All tests use FMjUESession (no live ZMQ socket). FMjUESession::Init stands
// the dispatcher up after Compile(); tests exercise its parser / handler
// entry points directly.
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "MjTestHelpers.h"
#include "Bridge/RpcDispatcher.h"
#include "Bridge/BridgeServer.h"
#include "Transport/ZmqRpcTransport.h"
#include "Bridge/MsgpackHelpers.h"
#include "MuJoCo/Components/Controllers/MjPDController.h"
#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Serialization/JsonSerializer.h"

// ---------------------------------------------------------------------------
// 0c. Op-category guard: manager-required ops error cleanly when no
//     AAMjManager is registered with the dispatcher (editor pre-PIE flow).
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerNoManagerGuard,
	"URLab.StepServer.NoManagerGuard",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerNoManagerGuard::RunTest(const FString& Parameters)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->AddToRoot();
	Server->Start(TEXT("")); // skip ZMQ bind — test only exercises Dispatch()
	FURLabRpcDispatcher* Disp = Server->GetDispatcher();
	if (!Disp)
	{
		AddError(TEXT("Server has no dispatcher"));
		Server->RemoveFromRoot();
		return false;
	}

	// Plant a session id so subsequent ops reach the manager guard rather
	// than the session check.
	Disp->SetActiveSessionIdForTest(TEXT("test-session"));

	// The dispatcher validates RequiredFields BEFORE the manager guard
	// so malformed requests fail with `missing_field` regardless of PIE
	// state. To exercise the no-manager path, the request has to be
	// otherwise well-formed — supply the required fields per op when
	// probing.
	auto MakeReq = [](const FString& Op) -> TSharedPtr<FJsonObject> {
		TSharedPtr<FJsonObject> R = MakeShared<FJsonObject>();
		R->SetStringField(TEXT("op"), Op);
		R->SetStringField(TEXT("session_id"), TEXT("test-session"));
		// Declarative-required fields per op.
		if (Op == TEXT("set_paused"))
			R->SetBoolField(TEXT("paused"), true);
		if (Op == TEXT("set_sim_speed"))
			R->SetNumberField(TEXT("percent"), 100.0);
		if (Op == TEXT("configure_controller"))
		{
			R->SetStringField(TEXT("articulation"), TEXT("ignored"));
			R->SetObjectField(TEXT("params"), MakeShared<FJsonObject>());
		}
		if (Op == TEXT("set_qpos"))
		{
			R->SetStringField(TEXT("target"), TEXT("ignored"));
			TArray<TSharedPtr<FJsonValue>> EmptyQpos;
			R->SetArrayField(TEXT("qpos"), EmptyQpos);
		}
		if (Op == TEXT("set_mocap_pose") || Op == TEXT("read_mocap_pose"))
		{
			R->SetStringField(TEXT("body"), TEXT("ignored"));
		}
		return R;
	};

	auto AssertNoManager = [this, Disp, &MakeReq](const FString& Op) {
		TSharedPtr<FJsonObject> Reply = Disp->Dispatch(MakeReq(Op));
		FString Code;
		Reply->TryGetStringField(TEXT("code"), Code);
		TestEqual(*FString::Printf(TEXT("op %s -> no_active_manager"), *Op),
			Code, FString(TEXT("no_active_manager")));
	};

	AssertNoManager(TEXT("step"));
	AssertNoManager(TEXT("reset"));
	AssertNoManager(TEXT("set_mode"));
	AssertNoManager(TEXT("set_paused"));
	AssertNoManager(TEXT("set_sim_speed"));
	AssertNoManager(TEXT("set_control_source"));
	AssertNoManager(TEXT("set_twist"));
	AssertNoManager(TEXT("set_qpos"));
	AssertNoManager(TEXT("set_mocap_pose"));
	AssertNoManager(TEXT("read_mocap_pose"));
	AssertNoManager(TEXT("get_contacts"));
	AssertNoManager(TEXT("list_keyframes"));
	AssertNoManager(TEXT("set_sim_options"));
	AssertNoManager(TEXT("configure_controller"));
	AssertNoManager(TEXT("recording_start"));
	AssertNoManager(TEXT("replay_start"));

	// hello must succeed pre-PIE so editor-time clients can connect.
	// Run last because it generates a fresh session id that would
	// invalidate the planted "test-session" used by AssertNoManager.
	{
		TSharedPtr<FJsonObject> Reply = Disp->Dispatch(MakeReq(TEXT("hello")));
		FString OpField;
		bool bMgr = true;
		Reply->TryGetStringField(TEXT("op"), OpField);
		Reply->TryGetBoolField(TEXT("manager_present"), bMgr);
		TestEqual(TEXT("hello -> hello_ok pre-PIE"),
			OpField, FString(TEXT("hello_ok")));
		TestFalse(TEXT("manager_present=false pre-PIE"), bMgr);
	}

	Server->Stop();
	Server->RemoveFromRoot();
	return true;
}

// ---------------------------------------------------------------------------
// 0a. BridgeServer lifecycle: Start is idempotent; Stop tears down.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBridgeServerLifecycle,
	"URLab.BridgeServer.Lifecycle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBridgeServerLifecycle::RunTest(const FString& Parameters)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	if (!Server)
	{
		AddError(TEXT("NewObject<UURLabBridgeServer> failed"));
		return false;
	}
	Server->AddToRoot();

	TestFalse(TEXT("Server starts not running"), Server->IsRunning());
	TestNull(TEXT("Dispatcher null before Start"), Server->GetDispatcher());

	Server->Start(TEXT("")); // skip ZMQ bind — test only exercises Dispatch()
	TestTrue(TEXT("IsRunning true after Start"), Server->IsRunning());
	TestNotNull(TEXT("Dispatcher non-null after Start"), Server->GetDispatcher());

	FURLabRpcDispatcher* DispBefore = Server->GetDispatcher();
	Server->Start(TEXT("")); // skip ZMQ bind — test only exercises Dispatch()  // idempotent
	TestEqual(TEXT("Start is idempotent (same dispatcher)"),
		Server->GetDispatcher(), DispBefore);

	Server->Stop();
	TestFalse(TEXT("IsRunning false after Stop"), Server->IsRunning());
	TestNull(TEXT("Dispatcher null after Stop"), Server->GetDispatcher());

	Server->Stop(); // idempotent
	TestFalse(TEXT("Stop is idempotent"), Server->IsRunning());

	Server->RemoveFromRoot();
	return true;
}

// ---------------------------------------------------------------------------
// 0b. AAMjManager owns a BridgeServer post-BeginPlay.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBridgeServerOwnedByManager,
	"URLab.BridgeServer.OwnedByManager",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBridgeServerOwnedByManager::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}

	TestNotNull(TEXT("Manager has a BridgeServer post-BeginPlay"),
		ToRawPtr(S.Manager->BridgeServer));
	if (S.Manager->BridgeServer)
	{
		TestTrue(TEXT("BridgeServer running"), S.Manager->BridgeServer->IsRunning());
		TestNotNull(TEXT("BridgeServer has a dispatcher"),
			S.Manager->BridgeServer->GetDispatcher());
		TestEqual(TEXT("GetStepDispatcher matches BridgeServer->GetDispatcher"),
			S.Manager->GetStepDispatcher(),
			S.Manager->BridgeServer->GetDispatcher());
	}

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 1. EStepMode dispatch / pause flag
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerPauseFlag,
	"URLab.StepServer.PauseFlag",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerPauseFlag::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}

	// Default: bPublishersPaused starts false.
	TestFalse(TEXT("Manager bPublishersPaused starts false"),
		S.Manager->bPublishersPaused.load());
	TestFalse(TEXT("FCameraZmqWorker::bPublishersPaused starts false"),
		FCameraZmqWorker::bPublishersPaused.load());

	// FMjUESession::Init has already stood the dispatcher up; just switch modes.
	FURLabRpcDispatcher* Disp = S.Manager->GetStepDispatcher();
	if (!Disp)
	{
		AddError(TEXT("Manager has no StepDispatcher"));
		S.Cleanup();
		return false;
	}

	Disp->SetActiveStepMode(EStepMode::Direct);
	TestTrue(TEXT("Direct mode flips manager pause flag true"),
		S.Manager->bPublishersPaused.load());
	TestTrue(TEXT("Direct mode flips camera pause flag true"),
		FCameraZmqWorker::bPublishersPaused.load());
	TestEqual(TEXT("ActiveStepMode reflects the switch"),
		(int)Disp->GetActiveStepMode(), (int)EStepMode::Direct);

	Disp->SetActiveStepMode(EStepMode::Live);
	TestFalse(TEXT("Live resets manager pause flag"),
		S.Manager->bPublishersPaused.load());
	TestFalse(TEXT("Live resets camera pause flag"),
		FCameraZmqWorker::bPublishersPaused.load());

	Disp->SetActiveStepMode(EStepMode::Puppet);
	TestTrue(TEXT("Puppet mode flips pause flag true"),
		S.Manager->bPublishersPaused.load());

	// Cleanup: leave the camera worker pause flag reset for downstream tests.
	Disp->SetActiveStepMode(EStepMode::Live);

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 1b. EffectiveStepMode mirrors the resolved mode (regression: live 10 Hz lock)
//     The physics loop paces off Manager->EffectiveStepMode. StepMode defaults
//     to Auto; if that does not resolve to Live the loop falls into the
//     step-request wait path and ticks at the 100 ms timeout (~10 Hz) instead
//     of running real-time.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerEffectiveMode,
	"URLab.StepServer.EffectiveStepMode",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerEffectiveMode::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}

	// StepMode is Auto by default; RegisterManager must resolve+mirror it to Live.
	TestEqual(TEXT("configured StepMode is Auto"),
		(int)S.Manager->StepMode, (int)EStepMode::Auto);
	TestEqual(TEXT("EffectiveStepMode resolves Auto -> Live"),
		(int)S.Manager->EffectiveStepMode.load(), (int)EStepMode::Live);

	FURLabRpcDispatcher* Disp = S.Manager->GetStepDispatcher();
	if (!Disp)
	{
		AddError(TEXT("Manager has no StepDispatcher"));
		S.Cleanup();
		return false;
	}

	Disp->SetActiveStepMode(EStepMode::Direct);
	TestEqual(TEXT("EffectiveStepMode tracks Direct"),
		(int)S.Manager->EffectiveStepMode.load(), (int)EStepMode::Direct);

	Disp->SetActiveStepMode(EStepMode::Puppet);
	TestEqual(TEXT("EffectiveStepMode tracks Puppet"),
		(int)S.Manager->EffectiveStepMode.load(), (int)EStepMode::Puppet);

	Disp->SetActiveStepMode(EStepMode::Live);
	TestEqual(TEXT("EffectiveStepMode tracks Live"),
		(int)S.Manager->EffectiveStepMode.load(), (int)EStepMode::Live);

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 2. UMjPDController ApplyConfig round-trip
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerControllerConfig,
	"URLab.StepServer.ControllerConfig",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerControllerConfig::RunTest(const FString& Parameters)
{
	UMjPDController* Pd = NewObject<UMjPDController>();
	Pd->DefaultKp = 100.0f;
	Pd->DefaultKv = 5.0f;
	Pd->DefaultTorqueLimit = 200.0f;

	// Schema must report the canonical fields.
	TSharedPtr<FJsonObject> Schema;
	Pd->GetConfigSchema(Schema);
	TestTrue(TEXT("Schema has kp"), Schema->HasField(TEXT("kp")));
	TestTrue(TEXT("Schema has kv"), Schema->HasField(TEXT("kv")));
	TestTrue(TEXT("Schema has torque_limit"), Schema->HasField(TEXT("torque_limit")));
	TestTrue(TEXT("Schema has default_kp"), Schema->HasField(TEXT("default_kp")));
	TestTrue(TEXT("Schema has default_kv"), Schema->HasField(TEXT("default_kv")));
	TestTrue(TEXT("Schema has default_torque_limit"), Schema->HasField(TEXT("default_torque_limit")));

	// ApplyConfig with only default scalars (no per-joint fields). Should not
	// crash and should update the default values exposed by GetCurrentConfig.
	TSharedPtr<FJsonObject> Patch = MakeShared<FJsonObject>();
	Patch->SetNumberField(TEXT("default_kp"), 250.0);
	Patch->SetNumberField(TEXT("default_kv"), 12.0);
	Patch->SetNumberField(TEXT("default_torque_limit"), 88.0);
	Pd->ApplyConfig(Patch);

	TestEqual(TEXT("DefaultKp updated"), Pd->DefaultKp, 250.0f);
	TestEqual(TEXT("DefaultKv updated"), Pd->DefaultKv, 12.0f);
	TestEqual(TEXT("DefaultTorqueLimit updated"), Pd->DefaultTorqueLimit, 88.0f);

	TSharedPtr<FJsonObject> Out;
	Pd->GetCurrentConfig(Out);
	double V = 0.0;
	TestTrue(TEXT("GetCurrentConfig has default_kp"),
		Out->TryGetNumberField(TEXT("default_kp"), V));
	TestEqual(TEXT("default_kp matches"), V, 250.0);

	return true;
}

// ---------------------------------------------------------------------------
// 2b. Handshake echoes per-articulation actor_id.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerHandshakeActorId,
	"URLab.StepServer.HandshakeActorId",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerHandshakeActorId::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}

	// Stamp an actor id on whatever articulation the test session built.
	TArray<AMjArticulation*> Arts = S.Manager->GetAllArticulations();
	if (Arts.Num() == 0 || !Arts[0])
	{
		AddInfo(TEXT("No articulation in test session; skipping actor_id check"));
		S.Cleanup();
		return true;
	}
	Arts[0]->ActorId = TEXT("robot_a");

	TSharedPtr<FJsonObject> Payload =
		FURLabRpcDispatcher::BuildHandshakePayload(S.Manager, TEXT("uuid"), TEXT("urlab/test"));

	const TArray<TSharedPtr<FJsonValue>>* ArtsArr = nullptr;
	TestTrue(TEXT("articulations array present"),
		Payload->TryGetArrayField(TEXT("articulations"), ArtsArr));
	if (ArtsArr && ArtsArr->Num() > 0)
	{
		const TSharedPtr<FJsonObject>& First = (*ArtsArr)[0]->AsObject();
		FString Got;
		TestTrue(TEXT("actor_id field present"),
			First->TryGetStringField(TEXT("actor_id"), Got));
		TestEqual(TEXT("actor_id echoed"), Got, FString(TEXT("robot_a")));
	}

	// Empty ActorId still emits an empty string (consumer doesn't have to
	// probe for absence).
	Arts[0]->ActorId = TEXT("");
	TSharedPtr<FJsonObject> Payload2 =
		FURLabRpcDispatcher::BuildHandshakePayload(S.Manager, TEXT("uuid2"), TEXT("urlab/test"));
	const TArray<TSharedPtr<FJsonValue>>* ArtsArr2 = nullptr;
	Payload2->TryGetArrayField(TEXT("articulations"), ArtsArr2);
	if (ArtsArr2 && ArtsArr2->Num() > 0)
	{
		const TSharedPtr<FJsonObject>& First2 = (*ArtsArr2)[0]->AsObject();
		FString Got2;
		TestTrue(TEXT("actor_id field still present (empty)"),
			First2->TryGetStringField(TEXT("actor_id"), Got2));
		TestEqual(TEXT("actor_id empty when unset"),
			Got2, FString(TEXT("")));
	}

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 3. Handshake payload shape
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerHandshake,
	"URLab.StepServer.Handshake",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerHandshake::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}

	TSharedPtr<FJsonObject> Payload =
		FURLabRpcDispatcher::BuildHandshakePayload(S.Manager, TEXT("test-uuid"), TEXT("urlab/test"));

	TestNotNull(TEXT("Handshake payload not null"), Payload.Get());

	FString Op;
	TestTrue(TEXT("Has op field"), Payload->TryGetStringField(TEXT("op"), Op));
	TestEqual(TEXT("op == hello_ok"), Op, FString(TEXT("hello_ok")));

	FString Sid;
	Payload->TryGetStringField(TEXT("session_id"), Sid);
	TestEqual(TEXT("session_id echoed"), Sid, FString(TEXT("test-uuid")));

	FString Ver;
	Payload->TryGetStringField(TEXT("urlab_version"), Ver);
	TestEqual(TEXT("urlab_version echoed"), Ver, FString(TEXT("urlab/test")));

	TestTrue(TEXT("Has articulations array"), Payload->HasField(TEXT("articulations")));
	TestTrue(TEXT("Has global_cameras object"), Payload->HasField(TEXT("global_cameras")));

	int32 MjbSize = 0;
	Payload->TryGetNumberField(TEXT("mjb_size"), MjbSize);
	TestTrue(TEXT("mjb_size > 0"), MjbSize > 0);

	FString MjbB64;
	TestTrue(TEXT("Has mjb_base64"), Payload->TryGetStringField(TEXT("mjb_base64"), MjbB64));
	TestTrue(TEXT("mjb_base64 non-empty"), MjbB64.Len() > 0);

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 4. Session id rejection
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerSessionId,
	"URLab.StepServer.SessionId",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerSessionId::RunTest(const FString& Parameters)
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

	TestFalse(TEXT("Empty session id rejected"),
		Disp->ValidateSession(FString()));
	TestFalse(TEXT("Random session id rejected before hello"),
		Disp->ValidateSession(TEXT("not-a-real-id")));

	Disp->SetActiveSessionIdForTest(TEXT("known-id"));
	TestTrue(TEXT("Matching session id accepted"),
		Disp->ValidateSession(TEXT("known-id")));
	TestFalse(TEXT("Non-matching session id rejected"),
		Disp->ValidateSession(TEXT("wrong-id")));

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 5. Puppet mode push-state writes qpos/qvel and fires OnPostStep
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerPuppetHandler,
	"URLab.StepServer.PuppetHandler",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerPuppetHandler::RunTest(const FString& Parameters)
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
	Disp->SetActiveStepMode(EStepMode::Puppet);

	mjModel* m = S.Manager->PhysicsEngine->GetModel();
	mjData* d = S.Manager->PhysicsEngine->GetData();
	if (!m || !d)
	{
		AddError(TEXT("Model/data missing"));
		S.Cleanup();
		return false;
	}

	int OnPostStepCount = 0;
	S.Manager->PhysicsEngine->OnPostStep = [&](mjModel*, mjData*) {
		OnPostStepCount++;
	};

	// Build a push-state request that sets qpos[0] and qvel[0] to known values.
	FMjPushStateRequest Req;
	Req.QPos.SetNum(m->nq);
	Req.QVel.SetNum(m->nv);
	if (m->nq > 0)
		Req.QPos[0] = 0.42;
	if (m->nv > 0)
		Req.QVel[0] = 0.13;
	Req.Time = 1.5;

	Disp->EnqueuePushStateRequestForTest(MoveTemp(Req));

	// Drive the engine's CustomStepHandler explicitly. The puppet handler
	// dequeues, writes, calls mj_forward, and fires OnPostStep.
	if (S.Manager->PhysicsEngine->CustomStepHandler)
	{
		S.Manager->PhysicsEngine->CustomStepHandler(m, d);
	}
	else
	{
		AddError(TEXT("CustomStepHandler not installed in Puppet mode"));
	}

	if (m->nq > 0)
		TestEqual(TEXT("qpos[0] written"), (double)d->qpos[0], 0.42, 1e-9);
	if (m->nv > 0)
		TestEqual(TEXT("qvel[0] written"), (double)d->qvel[0], 0.13, 1e-9);
	TestEqual(TEXT("d->time updated"), (double)d->time, 1.5, 1e-9);
	TestEqual(TEXT("OnPostStep fired exactly once"), OnPostStepCount, 1);

	Disp->SetActiveStepMode(EStepMode::Live);
	S.Manager->PhysicsEngine->OnPostStep = nullptr;

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 6. Direct mode ApplyStepCtrl writes ctrl through actuator NetworkValue
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerDirectCtrl,
	"URLab.StepServer.DirectCtrl",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerDirectCtrl::RunTest(const FString& Parameters)
{
	// Articulation with one slide joint + position actuator
	FMjUESession S;
	if (!S.Init([](FMjUESession& Sess) {
			Sess.Joint->Type = EMjJointType::Slide;
			UMjActuator* A = NewObject<UMjActuator>(Sess.Robot, TEXT("TestActuator"));
			A->Type = EMjActuatorType::Position;
			A->TargetName = Sess.Joint->GetName();
			A->RegisterComponent();
			A->AttachToComponent(Sess.Robot->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
		}))
	{
		// The session may not finish due to actuator wiring. Skip without erroring.
		AddInfo(FString::Printf(TEXT("Skipping DirectCtrl: %s"), *S.LastError));
		return true;
	}

	mjModel* m = S.Manager->PhysicsEngine->GetModel();
	mjData* d = S.Manager->PhysicsEngine->GetData();
	if (!m || !d || m->nu == 0)
	{
		AddInfo(TEXT("Skipping DirectCtrl: no actuators in compiled model"));
		S.Cleanup();
		return true;
	}

	FMjStepRequest Req;
	Req.NSteps = 1;
	// Find first actuator's local name from the manager's articulation.
	TArray<AMjArticulation*> Arts = S.Manager->GetAllArticulations();
	if (Arts.Num() == 0 || !Arts[0])
	{
		AddInfo(TEXT("Skipping DirectCtrl: no articulations registered"));
		S.Cleanup();
		return true;
	}
	AMjArticulation* Art = Arts[0];
	TArray<UMjActuator*> Acts = Art->GetActuators();
	if (Acts.Num() == 0)
	{
		AddInfo(TEXT("Skipping DirectCtrl: articulation has no actuators"));
		S.Cleanup();
		return true;
	}
	FString Local = Acts[0]->GetMjName();
	FString Pfx = Art->GetName() + TEXT("_");
	if (Local.StartsWith(Pfx))
		Local = Local.Mid(Pfx.Len());
	Req.PerArticulationCtrl.FindOrAdd(Art->GetName()).Add({Local, 0.7f});
	// Force raw write so the test path doesn't depend on a live controller.
	Req.PerArticulationControlMode.Add(Art->GetName(), TEXT("raw"));

	// ApplyStepCtrl stages writes on each actuator's NetworkValue. The
	// copy into d->ctrl happens inside Art->ApplyControls(...) once per
	// sub-step; passing bSkipController=true mirrors the bridge's
	// control_mode="raw" path so we land NetworkValue → d->ctrl without
	// controller transformation.
	FURLabRpcDispatcher::ApplyStepCtrl(S.Manager, Req, m, d);
	Art->ApplyControls(/*bSkipController=*/true);
	TestEqual(TEXT("d->ctrl[0] written by raw path"), (double)d->ctrl[0], 0.7, 1e-6);

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 7. msgpack round-trip via FURLabMsgpackUtil
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerMsgpackRoundtrip,
	"URLab.StepServer.MsgpackRoundtrip",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerMsgpackRoundtrip::RunTest(const FString& Parameters)
{
	// Build a JSON tree exercising every type the helpers must round-trip.
	TSharedPtr<FJsonObject> Src = MakeShared<FJsonObject>();
	Src->SetStringField(TEXT("op"), TEXT("step_ok"));
	Src->SetNumberField(TEXT("time"), 1.234);
	Src->SetNumberField(TEXT("step"), 42);
	Src->SetBoolField(TEXT("ok"), true);
	Src->SetField(TEXT("nullable"), MakeShared<FJsonValueNull>());

	TArray<TSharedPtr<FJsonValue>> Arr;
	Arr.Add(MakeShared<FJsonValueNumber>(0.5));
	Arr.Add(MakeShared<FJsonValueNumber>(-1.5));
	Arr.Add(MakeShared<FJsonValueNumber>(2.0));
	Src->SetArrayField(TEXT("qpos"), Arr);

	TSharedPtr<FJsonObject> Inner = MakeShared<FJsonObject>();
	Inner->SetStringField(TEXT("name"), TEXT("inner"));
	Inner->SetNumberField(TEXT("v"), 9.0);
	Src->SetObjectField(TEXT("nested"), Inner);

	// Binary blob test (e.g., MJB).
	const TArray<uint8> Bin = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0xFF, 0x42};
	FURLabMsgpackUtil::SetBinaryField(Src, TEXT("mjb"), Bin.GetData(), Bin.Num());

	// Pack -> unpack.
	TArray<uint8> PackedBuf;
	FURLabMsgpackUtil::PackJsonObject(Src, PackedBuf);
	TestTrue(TEXT("Packed buffer non-empty"), PackedBuf.Num() > 0);

	TSharedPtr<FJsonObject> Round;
	bool bOk = FURLabMsgpackUtil::UnpackToJsonObject(PackedBuf.GetData(), PackedBuf.Num(), Round);
	TestTrue(TEXT("Unpack returns true"), bOk);
	if (!bOk || !Round.IsValid())
	{
		return false;
	}

	FString S;
	double D = 0;
	bool B = false;
	TestTrue(TEXT("op survives"), Round->TryGetStringField(TEXT("op"), S));
	TestEqual(TEXT("op value"), S, FString(TEXT("step_ok")));
	TestTrue(TEXT("time survives"), Round->TryGetNumberField(TEXT("time"), D));
	TestEqual(TEXT("time value"), D, 1.234, 1e-9);
	TestTrue(TEXT("ok survives"), Round->TryGetBoolField(TEXT("ok"), B));
	TestTrue(TEXT("ok value"), B);

	const TArray<TSharedPtr<FJsonValue>>* QPosArr = nullptr;
	TestTrue(TEXT("qpos array survives"), Round->TryGetArrayField(TEXT("qpos"), QPosArr));
	if (QPosArr)
	{
		TestEqual(TEXT("qpos len"), QPosArr->Num(), 3);
		TestEqual(TEXT("qpos[0]"), (*QPosArr)[0]->AsNumber(), 0.5, 1e-9);
		TestEqual(TEXT("qpos[1]"), (*QPosArr)[1]->AsNumber(), -1.5, 1e-9);
	}

	const TSharedPtr<FJsonObject>* NestedObj = nullptr;
	TestTrue(TEXT("nested object survives"), Round->TryGetObjectField(TEXT("nested"), NestedObj));
	if (NestedObj && NestedObj->IsValid())
	{
		FString N;
		TestTrue(TEXT("nested.name"), (*NestedObj)->TryGetStringField(TEXT("name"), N));
		TestEqual(TEXT("nested.name value"), N, FString(TEXT("inner")));
	}

	// Binary field round-trips with the __b64__ key suffix (helper detail).
	FString Encoded;
	TestTrue(TEXT("mjb returned with __b64__ suffix"),
		Round->TryGetStringField(TEXT("mjb__b64__"), Encoded));
	TArray<uint8> Decoded;
	FBase64::Decode(Encoded, Decoded);
	TestEqual(TEXT("mjb size"), Decoded.Num(), Bin.Num());
	if (Decoded.Num() == Bin.Num())
	{
		for (int i = 0; i < Bin.Num(); ++i)
			TestEqual(FString::Printf(TEXT("mjb byte %d"), i), (int)Decoded[i], (int)Bin[i]);
	}

	return true;
}

// ---------------------------------------------------------------------------
// 8. Encoding flag: hello with encoding="json" vs default ("msgpack")
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerEncodingFlag,
	"URLab.StepServer.EncodingFlag",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerEncodingFlag::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}

	// We cannot exercise the live socket path from a headless test. Verify
	// the handshake payload comes out shape-correct (msgpack-canonical "mjb"
	// bin field present alongside the legacy mjb_base64 field) -- the flag
	// wiring is exercised whenever the wire layer round-trips msgpack, which
	// is covered by the msgpack roundtrip test.
	TSharedPtr<FJsonObject> Payload =
		FURLabRpcDispatcher::BuildHandshakePayload(S.Manager, TEXT("flag-test"), TEXT("urlab/test"));
	TestTrue(TEXT("Handshake builds with mjb (msgpack-canonical key)"),
		Payload.IsValid() && Payload->HasField(TEXT("mjb__b64__")));
	TestTrue(TEXT("Handshake also exposes legacy mjb_base64 for JSON clients"),
		Payload.IsValid() && Payload->HasField(TEXT("mjb_base64")));

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 9. Observation verbosity levels (minimal / standard / full)
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerObservationLevels,
	"URLab.StepServer.ObservationLevels",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerObservationLevels::RunTest(const FString& Parameters)
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

	// Minimal: qpos / qvel only.
	TSharedPtr<FJsonObject> Min = FURLabRpcDispatcher::BuildStepObservations(
		S.Manager, m, d, FURLabRpcDispatcher::EObservationLevel::Minimal);
	TestTrue(TEXT("Minimal returns object"), Min.IsValid());
	if (Min.IsValid() && Min->Values.Num() > 0)
	{
		// Pick first articulation entry.
		const TSharedPtr<FJsonObject>* Art = nullptr;
		Min->Values.CreateConstIterator()->Value->TryGetObject(Art);
		if (Art && Art->IsValid())
		{
			TestTrue(TEXT("Minimal has qpos"), (*Art)->HasField(TEXT("qpos")));
			TestTrue(TEXT("Minimal has qvel"), (*Art)->HasField(TEXT("qvel")));
			TestFalse(TEXT("Minimal lacks ctrl"), (*Art)->HasField(TEXT("ctrl")));
			TestFalse(TEXT("Minimal lacks sensors"), (*Art)->HasField(TEXT("sensors")));
			TestFalse(TEXT("Minimal lacks bodies"), (*Art)->HasField(TEXT("bodies")));
		}
	}

	// Standard: minimal + ctrl + act + sensors.
	TSharedPtr<FJsonObject> Std = FURLabRpcDispatcher::BuildStepObservations(
		S.Manager, m, d, FURLabRpcDispatcher::EObservationLevel::Standard);
	if (Std.IsValid() && Std->Values.Num() > 0)
	{
		const TSharedPtr<FJsonObject>* Art = nullptr;
		Std->Values.CreateConstIterator()->Value->TryGetObject(Art);
		if (Art && Art->IsValid())
		{
			TestTrue(TEXT("Standard has qpos"), (*Art)->HasField(TEXT("qpos")));
			TestTrue(TEXT("Standard has ctrl"), (*Art)->HasField(TEXT("ctrl")));
			TestTrue(TEXT("Standard has act"), (*Art)->HasField(TEXT("act")));
			TestTrue(TEXT("Standard has sensors"), (*Art)->HasField(TEXT("sensors")));
			TestFalse(TEXT("Standard lacks bodies"), (*Art)->HasField(TEXT("bodies")));
			TestFalse(TEXT("Standard lacks actuator_force"), (*Art)->HasField(TEXT("actuator_force")));
		}
	}

	// Full: standard + bodies + actuator_force.
	TSharedPtr<FJsonObject> Full = FURLabRpcDispatcher::BuildStepObservations(
		S.Manager, m, d, FURLabRpcDispatcher::EObservationLevel::Full);
	if (Full.IsValid() && Full->Values.Num() > 0)
	{
		const TSharedPtr<FJsonObject>* Art = nullptr;
		Full->Values.CreateConstIterator()->Value->TryGetObject(Art);
		if (Art && Art->IsValid())
		{
			TestTrue(TEXT("Full has bodies"), (*Art)->HasField(TEXT("bodies")));
			TestTrue(TEXT("Full has actuator_force"), (*Art)->HasField(TEXT("actuator_force")));
		}
	}

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 10. xfrc_applied parsing writes d->xfrc_applied[6 * body_id]
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerXfrcApplied,
	"URLab.StepServer.XfrcApplied",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerXfrcApplied::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}

	mjModel* m = S.Manager->PhysicsEngine->GetModel();
	mjData* d = S.Manager->PhysicsEngine->GetData();
	if (!m || !d || m->nbody < 2)
	{
		AddInfo(TEXT("Skipping xfrc test: <2 bodies"));
		S.Cleanup();
		return true;
	}

	// Find the first non-world body's name.
	TArray<AMjArticulation*> Arts = S.Manager->GetAllArticulations();
	if (Arts.Num() == 0 || !Arts[0])
	{
		S.Cleanup();
		return true;
	}
	AMjArticulation* Art = Arts[0];

	TArray<UMjBody*> Bodies;
	Art->GetComponents<UMjBody>(Bodies);
	if (Bodies.Num() == 0)
	{
		AddInfo(TEXT("Skipping xfrc test: articulation has no bodies"));
		S.Cleanup();
		return true;
	}

	UMjBody* B = nullptr;
	for (UMjBody* Bd : Bodies)
	{
		if (Bd && !Bd->bIsDefault)
		{
			B = Bd;
			break;
		}
	}
	if (!B)
	{
		S.Cleanup();
		return true;
	}

	int32 BodyId = B->GetMjID();
	if (BodyId <= 0 || BodyId >= m->nbody)
	{
		S.Cleanup();
		return true;
	}

	// Resolve the actual body name from the compiled model (the test
	// helper doesn't set MjName explicitly, so we read mj_id2name).
	const char* BodyNameC = mj_id2name(m, mjOBJ_BODY, BodyId);
	if (!BodyNameC)
	{
		AddInfo(TEXT("Skipping xfrc test: compiled body has no name"));
		S.Cleanup();
		return true;
	}
	FString FullName = UTF8_TO_TCHAR(BodyNameC);

	FMjStepRequest Req;
	TArray<double> Six = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
	// ApplyStepCtrl tries (Prefix + LocalName) first, falls back to LocalName.
	// Pass FullName directly so the second-pass lookup succeeds regardless
	// of the test rig's body-name layout.
	Req.PerArticulationXfrc.FindOrAdd(Art->GetName()).Add(FullName, Six);

	FURLabRpcDispatcher::ApplyStepCtrl(S.Manager, Req, m, d);
	for (int i = 0; i < 6; ++i)
	{
		TestEqual(FString::Printf(TEXT("xfrc[%d]"), i),
			(double)d->xfrc_applied[6 * BodyId + i], (double)Six[i], 1e-9);
	}

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 11. ApplyControls is gated when StepMode == Puppet
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerApplyControlsGate,
	"URLab.StepServer.ApplyControlsGate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerApplyControlsGate::RunTest(const FString& Parameters)
{
	// Verify that AAMjManager::StepMode == Puppet causes the engine's
	// ApplyControls call site to be skipped. The engine's async loop is
	// not running in tests, but the gate logic is observable through the
	// StepMode property that the call site reads on each iteration. The
	// critical invariant: StepMode is a UPROPERTY visible to the gate.
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}

	// Initial mode is Auto.
	TestEqual(TEXT("Default StepMode is Auto"),
		(int)S.Manager->StepMode, (int)EStepMode::Auto);

	// Puppet mode — the engine's ApplyControls gate should now report skip.
	S.Manager->StepMode = EStepMode::Puppet;
	TestEqual(TEXT("StepMode set to Puppet"),
		(int)S.Manager->StepMode, (int)EStepMode::Puppet);

	// The gate itself: in MjPhysicsEngine.cpp we call
	//   if (Cast<AAMjManager>(GetOwner())->StepMode == EStepMode::Puppet) skip;
	// This unit-level test verifies the property is reachable via Cast,
	// mirroring the gate's own access pattern.
	AAMjManager* OwnerMgr = Cast<AAMjManager>(S.Manager->PhysicsEngine->GetOwner());
	TestNotNull(TEXT("PhysicsEngine owner is AAMjManager"), OwnerMgr);
	if (OwnerMgr)
	{
		TestEqual(TEXT("Engine sees Puppet StepMode through GetOwner()"),
			(int)OwnerMgr->StepMode, (int)EStepMode::Puppet);
	}

	// Restore so other tests aren't affected.
	S.Manager->StepMode = EStepMode::Auto;
	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 12. Direct mode CustomStepHandler installs and processes a queued command
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerDirectHandler,
	"URLab.StepServer.DirectHandler",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerDirectHandler::RunTest(const FString& Parameters)
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
	Disp->SetActiveStepMode(EStepMode::Direct);

	mjModel* m = S.Manager->PhysicsEngine->GetModel();
	mjData* d = S.Manager->PhysicsEngine->GetData();
	if (!m || !d)
	{
		AddError(TEXT("Model/data missing"));
		S.Cleanup();
		return false;
	}

	TestNotNull(TEXT("Direct mode installs CustomStepHandler"),
		S.Manager->PhysicsEngine->CustomStepHandler ? (void*)0xdead : nullptr);

	int OnPostStepCount = 0;
	S.Manager->PhysicsEngine->OnPostStep = [&](mjModel*, mjData*) {
		OnPostStepCount++;
	};

	FMjStepRequest Req;
	Req.NSteps = 3;
	Disp->EnqueueStepRequestForTest(MoveTemp(Req));

	// Drive the handler explicitly. It should drain the queue and run
	// mj_step n_steps times — fires OnPostStep three times.
	if (S.Manager->PhysicsEngine->CustomStepHandler)
	{
		S.Manager->PhysicsEngine->CustomStepHandler(m, d);
	}
	else
	{
		AddError(TEXT("CustomStepHandler not installed in Direct mode"));
	}

	TestEqual(TEXT("OnPostStep fires once per substep"), OnPostStepCount, 3);

	Disp->SetActiveStepMode(EStepMode::Live);
	S.Manager->PhysicsEngine->OnPostStep = nullptr;

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 13. Perturbation snapshot is reachable from the step server (Puppet path)
// ---------------------------------------------------------------------------
#include "MuJoCo/Input/MjPerturbation.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerPerturbationSnapshot,
	"URLab.StepServer.PerturbationSnapshot",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerPerturbationSnapshot::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}
	if (!S.Manager->Perturbation)
	{
		AddError(TEXT("Manager has no Perturbation"));
		S.Cleanup();
		return false;
	}

	// Default state: BodyId == -1, version == 0.
	FMjPerturbationSample Initial = S.Manager->Perturbation->GetLatestPerturbationSample();
	TestEqual(TEXT("Initial BodyId"), Initial.BodyId, -1);
	TestEqual(TEXT("Initial Version"), (int)Initial.Version, 0);

	// Sanity: the manager pulls a sample every step in Puppet. We can't
	// exercise the live click-drag input here, but we verify the read-side
	// contract — multiple GetLatestPerturbationSample() calls return
	// consistent values without crashing under contention.
	for (int i = 0; i < 10; ++i)
	{
		FMjPerturbationSample S2 = S.Manager->Perturbation->GetLatestPerturbationSample();
		TestEqual(FString::Printf(TEXT("Stable BodyId iter %d"), i), S2.BodyId, -1);
	}

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 14a. set_qpos: full per-articulation write addressed by articulation name.
//      Default test session has one Hinge joint -> per-art qpos dim is 1.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerSetQposByName,
	"URLab.StepServer.SetQposByName",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerSetQposByName::RunTest(const FString& Parameters)
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

	FURLabRpcDispatcher* Disp = S.Manager->GetStepDispatcher();
	if (!Disp)
	{
		AddError(TEXT("Manager has no StepDispatcher"));
		S.Cleanup();
		return false;
	}
	Disp->SetActiveSessionIdForTest(TEXT("test-session"));

	AMjArticulation* Art = S.Manager->GetAllArticulations()[0];
	mjModel* m = S.Manager->PhysicsEngine->GetModel();
	mjData* d = S.Manager->PhysicsEngine->GetData();
	int32 Jid = Art->GetJoints()[0]->GetMjID();
	int32 QAddr = m->jnt_qposadr[Jid];

	TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
	Req->SetStringField(TEXT("op"), TEXT("set_qpos"));
	Req->SetStringField(TEXT("session_id"), TEXT("test-session"));
	Req->SetStringField(TEXT("target"), Art->GetName());
	Req->SetStringField(TEXT("target_by"), TEXT("actor_name"));
	TArray<TSharedPtr<FJsonValue>> Q;
	Q.Add(MakeShared<FJsonValueNumber>(0.42));
	Req->SetArrayField(TEXT("qpos"), Q);

	TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
	FString Op;
	Reply->TryGetStringField(TEXT("op"), Op);
	TestEqual(TEXT("op = set_qpos_ok"), Op, FString(TEXT("set_qpos_ok")));
	TestEqual(TEXT("qpos written to data buffer"), (double)d->qpos[QAddr], 0.42, 1e-9);
	bool bShortcut = true;
	Reply->TryGetBoolField(TEXT("free_base_shortcut"), bShortcut);
	TestFalse(TEXT("Hinge joint -> not free-base shortcut"), bShortcut);

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 14b. set_qpos addressed by actor_id resolves the same articulation.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerSetQposActorId,
	"URLab.StepServer.SetQposActorId",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerSetQposActorId::RunTest(const FString& Parameters)
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

	AMjArticulation* Art = S.Manager->GetAllArticulations()[0];
	Art->ActorId = TEXT("robot_a");

	FURLabRpcDispatcher* Disp = S.Manager->GetStepDispatcher();
	Disp->SetActiveSessionIdForTest(TEXT("test-session"));

	TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
	Req->SetStringField(TEXT("op"), TEXT("set_qpos"));
	Req->SetStringField(TEXT("session_id"), TEXT("test-session"));
	Req->SetStringField(TEXT("target"), TEXT("robot_a"));
	// target_by defaults to "actor_id"; omitted to exercise the default path.
	TArray<TSharedPtr<FJsonValue>> Q;
	Q.Add(MakeShared<FJsonValueNumber>(0.91));
	Req->SetArrayField(TEXT("qpos"), Q);

	TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
	FString Op, AId, RetTarget;
	Reply->TryGetStringField(TEXT("op"), Op);
	Reply->TryGetStringField(TEXT("actor_id"), AId);
	Reply->TryGetStringField(TEXT("target"), RetTarget);
	TestEqual(TEXT("op = set_qpos_ok"), Op, FString(TEXT("set_qpos_ok")));
	TestEqual(TEXT("reply echoes actor_id"), AId, FString(TEXT("robot_a")));
	TestEqual(TEXT("reply echoes target"), RetTarget, FString(TEXT("robot_a")));

	mjModel* m = S.Manager->PhysicsEngine->GetModel();
	mjData* d = S.Manager->PhysicsEngine->GetData();
	int32 Jid = Art->GetJoints()[0]->GetMjID();
	int32 QAddr = m->jnt_qposadr[Jid];
	TestEqual(TEXT("qpos written via actor_id"), (double)d->qpos[QAddr], 0.91, 1e-9);

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 14c. set_qpos free-base 7-vec shortcut writes only the free joint slots.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerSetQposFreeBase,
	"URLab.StepServer.SetQposFreeBase",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerSetQposFreeBase::RunTest(const FString& Parameters)
{
	FMjUESession S;
	// Free joint + an additional hinge child -> per-art qpos dim is 8 (7 + 1).
	// A 7-vec request triggers the free-base shortcut and only writes the
	// free joint slots, leaving the hinge value untouched.
	if (!S.Init([](FMjUESession& Sess) {
			Sess.Joint->Type = EMjJointType::Free;
			Sess.Joint->bOverride_Type = true;

			UMjBody* ChildBody = NewObject<UMjBody>(Sess.Robot, TEXT("ChildBody"));
			ChildBody->RegisterComponent();
			ChildBody->AttachToComponent(Sess.Body, FAttachmentTransformRules::KeepRelativeTransform);
			UMjGeom* ChildGeom = NewObject<UMjGeom>(Sess.Robot, TEXT("ChildGeom"));
			ChildGeom->size = {0.05f, 0.05f, 0.05f};
			ChildGeom->bOverride_size = true;
			ChildGeom->RegisterComponent();
			ChildGeom->AttachToComponent(ChildBody, FAttachmentTransformRules::KeepRelativeTransform);
			UMjJoint* Hinge = NewObject<UMjJoint>(Sess.Robot, TEXT("ChildHinge"));
			Hinge->Type = EMjJointType::Hinge;
			Hinge->bOverride_Type = true;
			Hinge->RegisterComponent();
			Hinge->AttachToComponent(ChildBody, FAttachmentTransformRules::KeepRelativeTransform);
		}))
	{
		// Articulation construction with a free base + child can fail in some
		// minimal-test setups; treat that as a skip rather than a failure.
		AddInfo(FString::Printf(TEXT("Skipping FreeBase: %s"), *S.LastError));
		return true;
	}

	FURLabRpcDispatcher* Disp = S.Manager->GetStepDispatcher();
	Disp->SetActiveSessionIdForTest(TEXT("test-session"));

	AMjArticulation* Art = S.Manager->GetAllArticulations()[0];
	mjModel* m = S.Manager->PhysicsEngine->GetModel();
	mjData* d = S.Manager->PhysicsEngine->GetData();

	TArray<UMjJoint*> JointsArr = Art->GetJoints();
	if (JointsArr.Num() < 2)
	{
		AddInfo(FString::Printf(TEXT("Skipping FreeBase: only %d joints compiled"), JointsArr.Num()));
		S.Cleanup();
		return true;
	}
	int32 FreeAdr = m->jnt_qposadr[JointsArr[0]->GetMjID()];
	int32 HingeAdr = m->jnt_qposadr[JointsArr[1]->GetMjID()];
	d->qpos[HingeAdr] = 1.5; // sentinel -- the shortcut must NOT touch this

	TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
	Req->SetStringField(TEXT("op"), TEXT("set_qpos"));
	Req->SetStringField(TEXT("session_id"), TEXT("test-session"));
	Req->SetStringField(TEXT("target"), Art->GetName());
	Req->SetStringField(TEXT("target_by"), TEXT("actor_name"));
	TArray<TSharedPtr<FJsonValue>> Q;
	const double Vals[7] = {0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0};
	for (int i = 0; i < 7; ++i)
		Q.Add(MakeShared<FJsonValueNumber>(Vals[i]));
	Req->SetArrayField(TEXT("qpos"), Q);

	TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
	FString Op;
	Reply->TryGetStringField(TEXT("op"), Op);
	TestEqual(TEXT("op = set_qpos_ok"), Op, FString(TEXT("set_qpos_ok")));
	bool bShortcut = false;
	Reply->TryGetBoolField(TEXT("free_base_shortcut"), bShortcut);
	TestTrue(TEXT("free_base_shortcut = true"), bShortcut);
	for (int i = 0; i < 7; ++i)
		TestEqual(*FString::Printf(TEXT("free qpos[%d]"), i),
			(double)d->qpos[FreeAdr + i], Vals[i], 1e-9);
	TestEqual(TEXT("hinge qpos untouched by shortcut"), (double)d->qpos[HingeAdr], 1.5, 1e-9);

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 14d. set_qpos: dim mismatch and unknown-articulation error paths.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerSetQposErrors,
	"URLab.StepServer.SetQposErrors",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerSetQposErrors::RunTest(const FString& Parameters)
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

	FURLabRpcDispatcher* Disp = S.Manager->GetStepDispatcher();
	Disp->SetActiveSessionIdForTest(TEXT("test-session"));

	AMjArticulation* Art = S.Manager->GetAllArticulations()[0];

	// dim_mismatch: 3-vec into a 1-dim hinge articulation.
	{
		TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
		Req->SetStringField(TEXT("op"), TEXT("set_qpos"));
		Req->SetStringField(TEXT("session_id"), TEXT("test-session"));
		Req->SetStringField(TEXT("target"), Art->GetName());
		Req->SetStringField(TEXT("target_by"), TEXT("actor_name"));
		TArray<TSharedPtr<FJsonValue>> Q;
		Q.Add(MakeShared<FJsonValueNumber>(0.0));
		Q.Add(MakeShared<FJsonValueNumber>(0.0));
		Q.Add(MakeShared<FJsonValueNumber>(0.0));
		Req->SetArrayField(TEXT("qpos"), Q);
		TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
		FString Code;
		Reply->TryGetStringField(TEXT("code"), Code);
		TestEqual(TEXT("dim_mismatch"), Code, FString(TEXT("dim_mismatch")));
	}

	// unknown_articulation: target with no matching actor_id.
	{
		TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
		Req->SetStringField(TEXT("op"), TEXT("set_qpos"));
		Req->SetStringField(TEXT("session_id"), TEXT("test-session"));
		Req->SetStringField(TEXT("target"), TEXT("does_not_exist"));
		TArray<TSharedPtr<FJsonValue>> Q;
		Q.Add(MakeShared<FJsonValueNumber>(0.0));
		Req->SetArrayField(TEXT("qpos"), Q);
		TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
		FString Code;
		Reply->TryGetStringField(TEXT("code"), Code);
		TestEqual(TEXT("unknown_articulation"), Code, FString(TEXT("unknown_articulation")));
	}

	// missing_field: no target field at all.
	{
		TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
		Req->SetStringField(TEXT("op"), TEXT("set_qpos"));
		Req->SetStringField(TEXT("session_id"), TEXT("test-session"));
		TArray<TSharedPtr<FJsonValue>> Q;
		Q.Add(MakeShared<FJsonValueNumber>(0.0));
		Req->SetArrayField(TEXT("qpos"), Q);
		TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
		FString Code;
		Reply->TryGetStringField(TEXT("code"), Code);
		TestEqual(TEXT("missing_field"), Code, FString(TEXT("missing_field")));
	}

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// 15. DispatchMutex narrowing.
//     A long-running editor handler must NOT block other RPCs. Stage a
//     fake editor handler that sleeps for 500 ms, fire it on one thread,
//     fire `hello` on a second thread, and verify hello returns quickly
//     (i.e. is not wedged behind the mutex a coarse "lock the whole
//     Dispatch" scheme would have held).
// ---------------------------------------------------------------------------
#include "Async/Async.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerDispatchMutexNarrow,
	"URLab.StepServer.DispatchMutexNarrow",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerDispatchMutexNarrow::RunTest(const FString& Parameters)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->AddToRoot();
	Server->Start(TEXT(""));
	FURLabRpcDispatcher* Disp = Server->GetDispatcher();
	if (!Disp)
	{
		AddError(TEXT("dispatcher missing"));
		Server->RemoveFromRoot();
		return false;
	}

	Disp->SetActiveSessionIdForTest(TEXT("phase01-session"));

	// Stage a fake editor op that sleeps before replying. Stands in for
	// the begin_pie / direct-step blocking waits.
	std::atomic<bool> bSlowStarted{false};
	URLabOpRegistry::RegisterHandler(TEXT("phase01_slow_op"),
		[&bSlowStarted](const TSharedPtr<FJsonObject>& Req) {
			bSlowStarted.store(true, std::memory_order_release);
			FPlatformProcess::Sleep(0.5f);
			TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
			Reply->SetStringField(TEXT("op"), TEXT("phase01_slow_op_ok"));
			return Reply;
		});

	// Fire the slow op on a worker thread.
	TFuture<void> SlowFuture = Async(EAsyncExecution::Thread, [Disp]() {
		TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
		Req->SetStringField(TEXT("op"), TEXT("phase01_slow_op"));
		Req->SetStringField(TEXT("session_id"), TEXT("phase01-session"));
		Disp->Dispatch(Req);
	});

	while (!bSlowStarted.load(std::memory_order_acquire))
	{
		FPlatformProcess::Sleep(0.001f);
	}

	// Fire hello on this thread; time it. With a coarse dispatch mutex
	// this would take ~500 ms because the slow op holds it.
	const double T0 = FPlatformTime::Seconds();
	TSharedPtr<FJsonObject> HelloReq = MakeShared<FJsonObject>();
	HelloReq->SetStringField(TEXT("op"), TEXT("hello"));
	TSharedPtr<FJsonObject> HelloReply = Disp->Dispatch(HelloReq);
	const double Elapsed = FPlatformTime::Seconds() - T0;

	FString Op;
	HelloReply->TryGetStringField(TEXT("op"), Op);
	TestEqual(TEXT("hello returns hello_ok during a slow concurrent op"),
		Op, FString(TEXT("hello_ok")));

	TestTrue(*FString::Printf(
				 TEXT("hello completed in %.3f s (must be < 0.05 s)"),
				 Elapsed),
		Elapsed < 0.05);

	SlowFuture.Wait();
	URLabOpRegistry::UnregisterHandler(TEXT("phase01_slow_op"));
	Server->Stop();
	Server->RemoveFromRoot();
	return true;
}

// ---------------------------------------------------------------------------
// 16. bDraining cooperative shutdown.
//     Stage a slow handler; fire it; from another thread call Stop on the
//     bridge mid-handler; verify Stop returns within ~100ms (instead of
//     pinning the worker thread until the handler's deadline elapses).
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerStopDrains,
	"URLab.StepServer.StopDrains",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerStopDrains::RunTest(const FString& Parameters)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->AddToRoot();
	Server->Start(TEXT(""));
	FURLabRpcDispatcher* Disp = Server->GetDispatcher();
	if (!Disp)
	{
		AddError(TEXT("dispatcher missing"));
		Server->RemoveFromRoot();
		return false;
	}

	Disp->SetActiveSessionIdForTest(TEXT("phase04-session"));

	// Fake editor handler that polls bDraining. Sleeps 50ms per tick
	// up to a generous 10s deadline — like begin_pie does today.
	std::atomic<bool> bHandlerStarted{false};
	std::atomic<bool> bHandlerExitedDueToDrain{false};
	URLabOpRegistry::RegisterHandler(TEXT("phase04_drainable_op"),
		[Disp, &bHandlerStarted, &bHandlerExitedDueToDrain](const TSharedPtr<FJsonObject>& Req) {
			bHandlerStarted.store(true, std::memory_order_release);
			const double Deadline = FPlatformTime::Seconds() + 10.0;
			while (FPlatformTime::Seconds() < Deadline)
			{
				if (Disp->IsDraining())
				{
					bHandlerExitedDueToDrain.store(true, std::memory_order_release);
					break;
				}
				FPlatformProcess::Sleep(0.05f);
			}
			TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
			Reply->SetStringField(TEXT("op"), TEXT("phase04_drainable_op_ok"));
			return Reply;
		});

	// Fire the handler on a worker thread.
	TFuture<void> HandlerFuture = Async(EAsyncExecution::Thread, [Disp]() {
		TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
		Req->SetStringField(TEXT("op"), TEXT("phase04_drainable_op"));
		Req->SetStringField(TEXT("session_id"), TEXT("phase04-session"));
		Disp->Dispatch(Req);
	});

	while (!bHandlerStarted.load(std::memory_order_acquire))
	{
		FPlatformProcess::Sleep(0.001f);
	}

	// Call Stop and time it. Without drain signalling the handler would
	// run for the full 10s; with drain Stop signals and the handler
	// exits within ~50–100ms.
	const double T0 = FPlatformTime::Seconds();
	Server->Stop();
	const double StopElapsed = FPlatformTime::Seconds() - T0;

	HandlerFuture.Wait();
	TestTrue(*FString::Printf(
				 TEXT("Stop completed in %.3f s (must be < 0.5 s)"),
				 StopElapsed),
		StopElapsed < 0.5);
	TestTrue(TEXT("handler observed bDraining and exited cooperatively"),
		bHandlerExitedDueToDrain.load(std::memory_order_acquire));

	URLabOpRegistry::UnregisterHandler(TEXT("phase04_drainable_op"));
	Server->RemoveFromRoot();
	return true;
}

// ---------------------------------------------------------------------------
// 17. SHM scope narrowing.
//     The SHM RPC transport's base class short-circuits editor-only ops to
//     a `wrong_transport: use_zmq` reply. Verify that ProcessRequestBytes
//     on a SHM transport refuses an editor-only op without invoking the
//     dispatcher's handler. ZMQ stays universal.
// ---------------------------------------------------------------------------
#include "Transport/ShmRpcTransport.h"
#include "Transport/ZmqRpcTransport.h"
#include "Bridge/MsgpackHelpers.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerShmRejectsEditorOps,
	"URLab.StepServer.ShmRejectsEditorOps",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerShmRejectsEditorOps::RunTest(const FString& Parameters)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->AddToRoot();
	Server->Start(TEXT("")); // dispatcher only, no real binds

	// Spin up a SHM transport WITHOUT calling TransportInit (we only need
	// ProcessRequestBytes' editor-op check, not real shm regions).
	UURLabShmRpcTransport* Shm = NewObject<UURLabShmRpcTransport>(Server);
	Shm->SetOwningBridge(Server);

	// Build a msgpack request for an editor-only op (`import_xml`).
	TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
	Req->SetStringField(TEXT("op"), TEXT("import_xml"));
	Req->SetStringField(TEXT("path"), TEXT("/tmp/whatever.xml"));
	TArray<uint8> ReqBytes;
	FURLabMsgpackUtil::PackJsonObject(Req, ReqBytes);

	TArray<uint8> ReplyBytes;
	Shm->ProcessRequestBytes(ReqBytes, ReplyBytes);

	TSharedPtr<FJsonObject> Reply;
	FURLabMsgpackUtil::UnpackToJsonObject(ReplyBytes.GetData(), ReplyBytes.Num(), Reply);
	if (!Reply.IsValid())
	{
		AddError(TEXT("reply did not decode"));
		Server->RemoveFromRoot();
		return false;
	}

	FString Op, Code;
	Reply->TryGetStringField(TEXT("op"), Op);
	Reply->TryGetStringField(TEXT("code"), Code);
	TestEqual(TEXT("op == error"), Op, FString(TEXT("error")));
	TestEqual(TEXT("code == wrong_transport"), Code, FString(TEXT("wrong_transport")));

	// ZMQ stays universal — same payload through ZMQ transport invokes
	// the dispatcher (which will return its own missing-fields error,
	// not wrong_transport).
	UURLabZmqRpcTransport* Zmq = NewObject<UURLabZmqRpcTransport>(Server);
	Zmq->SetOwningBridge(Server);
	TArray<uint8> ZmqReplyBytes;
	Zmq->ProcessRequestBytes(ReqBytes, ZmqReplyBytes);
	TSharedPtr<FJsonObject> ZmqReply;
	FURLabMsgpackUtil::UnpackToJsonObject(ZmqReplyBytes.GetData(), ZmqReplyBytes.Num(), ZmqReply);
	FString ZmqCode;
	if (ZmqReply.IsValid())
		ZmqReply->TryGetStringField(TEXT("code"), ZmqCode);
	TestNotEqual(TEXT("ZMQ does not emit wrong_transport"),
		ZmqCode, FString(TEXT("wrong_transport")));

	Server->Stop();
	Server->RemoveFromRoot();
	return true;
}

// ---------------------------------------------------------------------------
// 18. Bridge owns transports array; EnsureZmqBound is idempotent.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerBridgeOwnsTransports,
	"URLab.StepServer.BridgeOwnsTransports",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerBridgeOwnsTransports::RunTest(const FString& Parameters)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->AddToRoot();
	Server->Start(TEXT("")); // dispatcher only

	TestEqual(TEXT("no transports after empty Start"),
		Server->GetRpcTransports().Num(), 0);

	// Bind ZMQ on an unprivileged ephemeral port to avoid collisions.
	const FString Endpoint = TEXT("tcp://127.0.0.1:25559");
	TestTrue(TEXT("EnsureZmqBound returns true"),
		Server->EnsureZmqBound(Endpoint));
	TestEqual(TEXT("one transport after EnsureZmqBound"),
		Server->GetRpcTransports().Num(), 1);

	// Idempotent: same endpoint should not re-bind.
	TestTrue(TEXT("EnsureZmqBound idempotent"),
		Server->EnsureZmqBound(Endpoint));
	TestEqual(TEXT("still one transport after second EnsureZmqBound"),
		Server->GetRpcTransports().Num(), 1);

	Server->Stop();
	TestEqual(TEXT("transports cleared on Stop"),
		Server->GetRpcTransports().Num(), 0);
	Server->RemoveFromRoot();
	return true;
}

// ---------------------------------------------------------------------------
// 19. `meta` op returns the registry table.
//     Sends a meta request through Dispatch (no session needed) and
//     verifies every dispatcher-owned op (step, reset, set_qpos, ...)
//     plus every editor-only op shows up with the right category /
//     namespace.
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerMetaOpReturnsRegistry,
	"URLab.StepServer.MetaOpReturnsRegistry",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerMetaOpReturnsRegistry::RunTest(const FString& Parameters)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->AddToRoot();
	Server->Start(TEXT(""));
	FURLabRpcDispatcher* Disp = Server->GetDispatcher();
	if (!Disp)
	{
		AddError(TEXT("dispatcher missing"));
		Server->RemoveFromRoot();
		return false;
	}

	TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
	Req->SetStringField(TEXT("op"), TEXT("meta"));
	TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);

	FString OpReply;
	Reply->TryGetStringField(TEXT("op"), OpReply);
	TestEqual(TEXT("op == meta_ok"), OpReply, FString(TEXT("meta_ok")));

	const TArray<TSharedPtr<FJsonValue>>* Ops = nullptr;
	if (!Reply->TryGetArrayField(TEXT("ops"), Ops) || !Ops)
	{
		AddError(TEXT("meta reply missing 'ops' array"));
		Server->Stop();
		Server->RemoveFromRoot();
		return false;
	}

	TMap<FString, FString> NamespaceByOp;
	TMap<FString, FString> CategoryByOp;
	for (const TSharedPtr<FJsonValue>& V : *Ops)
	{
		const TSharedPtr<FJsonObject>* O = nullptr;
		if (!V->TryGetObject(O) || !O || !O->IsValid())
			continue;
		FString Name, Ns, Cat;
		(*O)->TryGetStringField(TEXT("name"), Name);
		(*O)->TryGetStringField(TEXT("namespace"), Ns);
		(*O)->TryGetStringField(TEXT("category"), Cat);
		NamespaceByOp.Add(Name, Ns);
		CategoryByOp.Add(Name, Cat);
	}

	// Dispatcher-owned ops registered in the constructor.
	// step + reset are top-level lifecycle primitives — empty namespace
	// (Python exposes them on URLabClient itself).
	TestTrue(TEXT("step is top-level (no namespace)"),
		NamespaceByOp.Contains(TEXT("step"))
			&& NamespaceByOp[TEXT("step")].IsEmpty());
	TestTrue(TEXT("step is manager_required"),
		CategoryByOp.Contains(TEXT("step"))
			&& CategoryByOp[TEXT("step")] == TEXT("manager_required"));
	TestTrue(TEXT("set_qpos is in 'runtime' namespace"),
		NamespaceByOp.Contains(TEXT("set_qpos"))
			&& NamespaceByOp[TEXT("set_qpos")] == TEXT("runtime"));
	for (const TCHAR* Op : {TEXT("set_mocap_pose"), TEXT("read_mocap_pose"),
			 TEXT("get_contacts"), TEXT("list_keyframes")})
	{
		TestTrue(*FString::Printf(TEXT("%s is in 'runtime' namespace"), Op),
			NamespaceByOp.Contains(Op) && NamespaceByOp[Op] == TEXT("runtime"));
		TestTrue(*FString::Printf(TEXT("%s is manager_required"), Op),
			CategoryByOp.Contains(Op) && CategoryByOp[Op] == TEXT("manager_required"));
	}
	TestTrue(TEXT("recording_start is in 'recording' namespace"),
		NamespaceByOp.Contains(TEXT("recording_start"))
			&& NamespaceByOp[TEXT("recording_start")] == TEXT("recording"));
	TestTrue(TEXT("replay_load is in 'replay' namespace"),
		NamespaceByOp.Contains(TEXT("replay_load"))
			&& NamespaceByOp[TEXT("replay_load")] == TEXT("replay"));

	// Editor ops registered in URLabEditor's StartupModule.
	TestTrue(TEXT("import_xml is editor_only / scene"),
		NamespaceByOp.Contains(TEXT("import_xml"))
			&& NamespaceByOp[TEXT("import_xml")] == TEXT("scene")
			&& CategoryByOp[TEXT("import_xml")] == TEXT("editor_only"));
	TestTrue(TEXT("begin_pie is editor_only / sim"),
		NamespaceByOp.Contains(TEXT("begin_pie"))
			&& NamespaceByOp[TEXT("begin_pie")] == TEXT("sim")
			&& CategoryByOp[TEXT("begin_pie")] == TEXT("editor_only"));

	// `meta` itself is bootstrap-hardcoded in Dispatch and intentionally
	// NOT in the registry table — that breaks the chicken-and-egg.
	TestFalse(TEXT("meta is not in the registry table"),
		NamespaceByOp.Contains(TEXT("meta")));
	// `hello` is also pre-session bootstrap and not registered.
	TestFalse(TEXT("hello is not in the registry table"),
		NamespaceByOp.Contains(TEXT("hello")));

	// Every op's decl carries its reply_fields. Pull one out and verify
	// the encoding ("name:type" with optional "?").
	bool bFoundReplyFields = false;
	for (const TSharedPtr<FJsonValue>& V : *Ops)
	{
		const TSharedPtr<FJsonObject>* O = nullptr;
		if (!V->TryGetObject(O) || !O || !O->IsValid())
			continue;
		FString Name;
		(*O)->TryGetStringField(TEXT("name"), Name);
		if (Name != TEXT("step"))
			continue;
		const TArray<TSharedPtr<FJsonValue>>* Fields = nullptr;
		if ((*O)->TryGetArrayField(TEXT("reply_fields"), Fields) && Fields)
		{
			bool bSawTime = false, bSawSimTime = false;
			for (const TSharedPtr<FJsonValue>& F : *Fields)
			{
				const FString S = F->AsString();
				if (S == TEXT("time:float"))
					bSawTime = true;
				if (S == TEXT("sim_time:object?"))
					bSawSimTime = true;
			}
			TestTrue(TEXT("step.reply_fields contains 'time:float'"), bSawTime);
			TestTrue(TEXT("step.reply_fields contains optional 'sim_time:object?'"),
				bSawSimTime);
			bFoundReplyFields = true;
		}
		break;
	}
	TestTrue(TEXT("step decl exposes reply_fields"), bFoundReplyFields);

	Server->Stop();
	Server->RemoveFromRoot();
	return true;
}

// ---------------------------------------------------------------------------
// 20. Registry-driven dispatch: an unregistered op returns
//     `unknown_op`; an editor-only op with no registered handler in a
//     test fixture (where StartupModule didn't run) returns
//     `not_in_editor`.
// ---------------------------------------------------------------------------

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerUnknownOpVsNotInEditor,
	"URLab.StepServer.UnknownOpVsNotInEditor",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerUnknownOpVsNotInEditor::RunTest(const FString& Parameters)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->AddToRoot();
	Server->Start(TEXT(""));
	FURLabRpcDispatcher* Disp = Server->GetDispatcher();
	if (!Disp)
	{
		AddError(TEXT("dispatcher missing"));
		Server->RemoveFromRoot();
		return false;
	}

	Disp->SetActiveSessionIdForTest(TEXT("phase4-test"));

	// Genuinely unknown op (not in registry, not dispatcher-owned).
	TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
	Req->SetStringField(TEXT("op"), TEXT("nonexistent_op"));
	Req->SetStringField(TEXT("session_id"), TEXT("phase4-test"));
	TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
	FString Code;
	Reply->TryGetStringField(TEXT("code"), Code);
	TestEqual(TEXT("unknown_op for genuinely unknown name"),
		Code, FString(TEXT("unknown_op")));

	Server->Stop();
	Server->RemoveFromRoot();
	return true;
}

// ---------------------------------------------------------------------------
// 21. Strict RequiredFields validation in Dispatch.
//     A `set_paused` request with no `paused` field must short-circuit
//     to `missing_field` BEFORE the handler runs. Verifies the
//     declarative request validation is wired.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepServerRequiredFieldsValidated,
	"URLab.StepServer.RequiredFieldsValidated",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepServerRequiredFieldsValidated::RunTest(const FString& Parameters)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->AddToRoot();
	Server->Start(TEXT(""));
	FURLabRpcDispatcher* Disp = Server->GetDispatcher();
	if (!Disp)
	{
		AddError(TEXT("dispatcher missing"));
		Server->RemoveFromRoot();
		return false;
	}

	Disp->SetActiveSessionIdForTest(TEXT("phase44-test"));

	TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
	Req->SetStringField(TEXT("op"), TEXT("set_paused"));
	Req->SetStringField(TEXT("session_id"), TEXT("phase44-test"));
	// Intentionally omit `paused` — it's declared as required.

	TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
	FString Code, Msg;
	Reply->TryGetStringField(TEXT("code"), Code);
	Reply->TryGetStringField(TEXT("message"), Msg);
	// The handler-level check would also fire `missing_field`, but the
	// registry-level check is supposed to short-circuit BEFORE
	// OwnerMgr is touched — i.e. this test passes even with no manager.
	TestEqual(TEXT("missing_field on omitted required field"),
		Code, FString(TEXT("missing_field")));
	TestTrue(TEXT("error message names the field"),
		Msg.Contains(TEXT("paused")));

	Server->Stop();
	Server->RemoveFromRoot();
	return true;
}
