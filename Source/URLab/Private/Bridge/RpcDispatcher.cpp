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

#include "Bridge/RpcDispatcher.h"
#include "Bridge/OpRegistry.h"
#include "Transport/ZmqRpcTransport.h"
#include "Bridge/MsgpackHelpers.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "MuJoCo/Components/Sensors/MjSensor.h"
#include "MuJoCo/Components/Sensors/MjCamera.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Components/Controllers/MjArticulationController.h"
#include "MuJoCo/Input/MjPerturbation.h"
#include "MuJoCo/Input/MjTwistController.h"
#include "Transport/NetworkManager.h"
#include "Transport/ShmPublishTransport.h"
#include "Transport/ShmRpcTransport.h"
#include "Transport/RpcTransport.h"
#include "Transport/ShmRegion.h" // FMjShmHeader (header_size in shm_rpc block)
#include "Bridge/BridgeServer.h"
#include "Replay/MjReplayManager.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/Base64.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "Internationalization/Regex.h"
#include "HAL/FileManager.h"
#include "EngineUtils.h"
#include "Engine/World.h"
#include "Misc/Guid.h"
#include "Utils/URLabLogging.h"

namespace
{
/** Map URLab actuator enum -> wire-format string. */
FString ActuatorTypeToString(EMjActuatorType T)
{
	switch (T)
	{
		case EMjActuatorType::Motor:
			return TEXT("motor");
		case EMjActuatorType::Position:
			return TEXT("position");
		case EMjActuatorType::Velocity:
			return TEXT("velocity");
		case EMjActuatorType::IntVelocity:
			return TEXT("intvelocity");
		case EMjActuatorType::Damper:
			return TEXT("damper");
		case EMjActuatorType::Cylinder:
			return TEXT("cylinder");
		case EMjActuatorType::Muscle:
			return TEXT("muscle");
		case EMjActuatorType::Adhesion:
			return TEXT("adhesion");
		case EMjActuatorType::DcMotor:
			return TEXT("dcmotor");
	}
	return TEXT("motor");
}
} // namespace

FURLabRpcDispatcher::FURLabRpcDispatcher()
{
	RegisterDispatcherOps();
}

FURLabRpcDispatcher::~FURLabRpcDispatcher()
{
	UnregisterDispatcherOps();
}

void FURLabRpcDispatcher::RegisterDispatcherOps()
{
	using EOpCategory = URLabOpRegistry::EOpCategory;

	RegisteredOpNames.Reset();
	auto Reg = [this](const TCHAR* Name, EOpCategory Cat, const TCHAR* Ns,
				   TFunction<TSharedPtr<FJsonObject>(const TSharedPtr<FJsonObject>&)> Body,
				   std::initializer_list<const TCHAR*> ReplyFields = {},
				   std::initializer_list<const TCHAR*> RequiredFields = {}) {
		URLabOpRegistry::FOpDecl Decl;
		Decl.Name = Name;
		Decl.Category = Cat;
		Decl.Namespace = Ns;
		Decl.Body = MoveTemp(Body);
		for (const TCHAR* F : ReplyFields)
			Decl.ReplyFields.Add(F);
		for (const TCHAR* F : RequiredFields)
			Decl.RequiredFields.Add(F);
		// Record before MoveTemp consumes Decl.
		RegisteredOpNames.Add(Decl.Name);
		URLabOpRegistry::RegisterOp(MoveTemp(Decl));
	};

	Reg(TEXT("step"), EOpCategory::ManagerRequired, TEXT(""),
		[this](auto& R) { return HandleStep(R); },
		{TEXT("op:string"), TEXT("time:float"), TEXT("step:int"),
			TEXT("per_articulation:object"),
			TEXT("sim_time:object?"), TEXT("wall_time:object?")});
	Reg(TEXT("reset"), EOpCategory::ManagerRequired, TEXT(""),
		[this](auto& R) { return HandleReset(R); },
		{TEXT("op:string"), TEXT("time:float"), TEXT("step:int")});
	Reg(TEXT("forward"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleForward(R); },
		{TEXT("op:string"), TEXT("time:float"), TEXT("step:int"), TEXT("per_articulation:object")});
	Reg(TEXT("set_mode"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleSetMode(R); },
		{TEXT("op:string"), TEXT("previous_mode:string"), TEXT("current_mode:string")});
	Reg(TEXT("set_paused"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleSetPaused(R); },
		/*Reply=*/{TEXT("op:string"), TEXT("paused:bool")},
		/*Required=*/{TEXT("paused")});
	Reg(TEXT("set_camera_streaming"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleSetCameraStreaming(R); },
		/*Reply=*/{TEXT("op:string"), TEXT("cameras:object")},
		/*Required=*/{TEXT("cameras")});
	Reg(TEXT("set_camera_delay"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleSetCameraDelay(R); },
		/*Reply=*/{TEXT("op:string"), TEXT("cameras:object")},
		/*Required=*/{TEXT("cameras")});
	Reg(TEXT("configure_controller"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleConfigureController(R); },
		/*Reply=*/{TEXT("op:string"), TEXT("articulation:string"), TEXT("params:object")},
		/*Required=*/{TEXT("articulation"), TEXT("params")});
	Reg(TEXT("set_sim_options"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleSetSimOptions(R); },
		{TEXT("op:string")});
	Reg(TEXT("set_sim_speed"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleSetSimSpeed(R); },
		/*Reply=*/{TEXT("op:string"), TEXT("percent:float")},
		/*Required=*/{TEXT("percent")});
	Reg(TEXT("set_control_source"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleSetControlSource(R); },
		{TEXT("op:string")});
	Reg(TEXT("set_twist"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleSetTwist(R); },
		{TEXT("op:string")});
	Reg(TEXT("set_qpos"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleSetQpos(R); },
		/*Reply=*/{TEXT("op:string"), TEXT("target:string"), TEXT("actor_id:string?"), TEXT("actor_name:string?"), TEXT("qpos:array"), TEXT("free_base_shortcut:bool")},
		/*Required=*/{TEXT("target"), TEXT("qpos")});
	Reg(TEXT("set_mocap_pose"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleSetMocapPose(R); },
		/*Reply=*/{TEXT("op:string"), TEXT("body:string"), TEXT("pos:array"), TEXT("quat:array")},
		/*Required=*/{TEXT("body")});
	Reg(TEXT("read_mocap_pose"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleReadMocapPose(R); },
		/*Reply=*/{TEXT("op:string"), TEXT("body:string"), TEXT("pos:array"), TEXT("quat:array")},
		/*Required=*/{TEXT("body")});
	Reg(TEXT("get_contacts"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleGetContacts(R); },
		/*Reply=*/{TEXT("op:string"), TEXT("n_contacts:int"), TEXT("truncated:bool"), TEXT("contacts:array")});
	Reg(TEXT("list_keyframes"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleListKeyframes(R); },
		{TEXT("op:string"), TEXT("keyframes:array")});

	auto RecBody = [this](auto& R) {
		FString OpName;
		R->TryGetStringField(TEXT("op"), OpName);
		return HandleRecording(OpName, R);
	};
	Reg(TEXT("recording_start"), EOpCategory::ManagerRequired, TEXT("recording"), RecBody);
	Reg(TEXT("recording_stop"), EOpCategory::ManagerRequired, TEXT("recording"), RecBody);
	Reg(TEXT("recording_save"), EOpCategory::ManagerRequired, TEXT("recording"), RecBody);
	Reg(TEXT("recording_clear"), EOpCategory::ManagerRequired, TEXT("recording"), RecBody);

	auto RepBody = [this](auto& R) {
		FString OpName;
		R->TryGetStringField(TEXT("op"), OpName);
		return HandleReplay(OpName, R);
	};
	Reg(TEXT("replay_load"), EOpCategory::ManagerRequired, TEXT("replay"), RepBody);
	Reg(TEXT("replay_list_sessions"), EOpCategory::ManagerRequired, TEXT("replay"), RepBody);
	Reg(TEXT("replay_set_active"), EOpCategory::ManagerRequired, TEXT("replay"), RepBody);
	Reg(TEXT("replay_start"), EOpCategory::ManagerRequired, TEXT("replay"), RepBody);
	Reg(TEXT("replay_stop"), EOpCategory::ManagerRequired, TEXT("replay"), RepBody);
}

void FURLabRpcDispatcher::UnregisterDispatcherOps()
{
	for (const FString& Name : RegisteredOpNames)
	{
		URLabOpRegistry::UnregisterHandler(Name);
	}
	RegisteredOpNames.Reset();
}

void FURLabRpcDispatcher::Init(AAMjManager* InManager)
{
	bDraining.store(false, std::memory_order_release);

	OwnerMgr = InManager;
	if (!OwnerMgr.IsValid())
		return;

	// If the project pinned a non-Auto mode, lock it now so set_mode
	// is rejected until the project relaxes.
	const EStepMode InitMode = (OwnerMgr->StepMode == EStepMode::Auto)
								 ? EStepMode::Live
								 : OwnerMgr->StepMode;
	ActiveStepMode.store(InitMode, std::memory_order_release);
	// Camera publishers stream in every mode; the strategy's OnEnter handles the
	// state/ctrl publishers, the engine step mode, and the handler install.
	FCameraZmqWorker::bPublishersPaused.store(false, std::memory_order_release);
	CurrentStepStrategy = MakeStepStrategy(InitMode);
	CurrentStepStrategy->OnEnter(*this, *OwnerMgr);

	// Cached on the game thread; worker threads later use Get() (TActorIterator
	// asserts IsInGameThread).
	if (UWorld* World = OwnerMgr->GetWorld())
	{
		AActor* Actor = UGameplayStatics::GetActorOfClass(
			World, AMjReplayManager::StaticClass());
		CachedReplayManager = Cast<AMjReplayManager>(Actor);
	}
}

void FURLabRpcDispatcher::OnManagerGone()
{
	UninstallPuppetHandler();
	UninstallDirectHandler();
	CurrentStepStrategy.Reset();
	DrainQueuesForTest();

	if (OwnerMgr.IsValid())
	{
		OwnerMgr->bPublishersPaused.store(false, std::memory_order_release);
		if (OwnerMgr->PhysicsEngine)
			OwnerMgr->PhysicsEngine->SetStepMode(EStepMode::Live);
	}
	FCameraZmqWorker::bPublishersPaused.store(false, std::memory_order_release);
	OwnerMgr.Reset();
	// Drop the cached replay manager too — its actor was destroyed
	// with the level, and the next PIE cycle's Init() is the only
	// writer that should ever set it. Without this clear, recording
	// RPCs after a PIE cycle could resolve to a stale (or null-via-
	// weak-deref) pointer and silently fail.
	CachedReplayManager.Reset();

	// Per-PIE state resets; the bridge-level session and observation
	// level stay so the connected client doesn't get session_expired
	// when a PIE cycle ends or the editor level changes.
	ActiveStepMode.store(EStepMode::Live, std::memory_order_release);
	StepCounter.store(0, std::memory_order_relaxed);
}

void FURLabRpcDispatcher::Shutdown()
{
	OnManagerGone();

	// Bridge-level state — only cleared when the bridge server itself
	// stops. Spans PIE cycles.
	{
		FScopeLock Lock(&DispatchMutex);
		ActiveSessionId.Empty();
	}
	ActiveObservationLevel.store(EObservationLevel::Standard, std::memory_order_release);
	bUseJsonEncoding.store(false, std::memory_order_release);
}

void FURLabRpcDispatcher::SetCachedReplayManager(AMjReplayManager* RM)
{
	CachedReplayManager = RM;
}

void FURLabRpcDispatcher::EnqueueStepRequestForTest(FMjStepRequest&& Req)
{
	TSharedPtr<FMjDirectStepCommand> Cmd = MakeShared<FMjDirectStepCommand>();
	Cmd->Request = MoveTemp(Req);
	StepQueue.Enqueue(Cmd);
	if (AAMjManager* Mgr = OwnerMgr.Get())
	{
		if (Mgr->PhysicsEngine && Mgr->PhysicsEngine->StepRequestEvent)
		{
			Mgr->PhysicsEngine->StepRequestEvent->Trigger();
		}
	}
}

void FURLabRpcDispatcher::EnqueuePushStateRequestForTest(FMjPushStateRequest&& Req)
{
	PushStateQueue.Enqueue(MoveTemp(Req));
}

void FURLabRpcDispatcher::DrainQueuesForTest()
{
	TSharedPtr<FMjDirectStepCommand> Cmd;
	while (StepQueue.Dequeue(Cmd))
	{
	} // shared_ptr deallocates on scope exit
	FMjPushStateRequest P;
	while (PushStateQueue.Dequeue(P))
	{
	}
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::Dispatch(const TSharedPtr<FJsonObject>& Req)
{
	const double StartTime = FPlatformTime::Seconds();
	FString InOp = TEXT("<no-op>");
	if (Req.IsValid())
		Req->TryGetStringField(TEXT("op"), InOp);
	UE_LOG(LogURLabNet, Log, TEXT("RPC -> %s"), *InOp);

	TSharedPtr<FJsonObject> Reply = DispatchInternal(Req);

	const double DurationMs = (FPlatformTime::Seconds() - StartTime) * 1000.0;
	FString ReplyOp;
	if (Reply.IsValid())
		Reply->TryGetStringField(TEXT("op"), ReplyOp);

	if (ReplyOp.Equals(TEXT("error")))
	{
		FString Code, Message;
		Reply->TryGetStringField(TEXT("code"), Code);
		Reply->TryGetStringField(TEXT("message"), Message);
		UE_LOG(LogURLabNet, Warning,
			TEXT("RPC <- %s ERROR code=%s (%.1fms): %s"),
			*InOp, *Code, DurationMs, *Message);
	}
	else
	{
		UE_LOG(LogURLabNet, Log,
			TEXT("RPC <- %s -> %s (%.1fms)"), *InOp, *ReplyOp, DurationMs);
	}
	return Reply;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::DispatchInternal(const TSharedPtr<FJsonObject>& Req)
{
	// Do NOT hold DispatchMutex across the handler body. Direct-mode
	// HandleStep waits up to 5s and HandleBeginPie polls for up to 30s;
	// holding the mutex across either wedges every other RPC.
	if (!Req.IsValid())
	{
		return MakeError(TEXT("bad_request"), TEXT("Failed to parse request (json or msgpack)"));
	}

	FString Op;
	if (!Req->TryGetStringField(TEXT("op"), Op))
	{
		return MakeError(TEXT("missing_op"), TEXT("Request missing 'op' field"));
	}

	// hello / meta are pre-session bootstrap endpoints.
	if (Op.Equals(TEXT("hello")))
		return HandleHello(Req);
	if (Op.Equals(TEXT("meta")))
		return HandleMeta(Req);

	{
		FScopeLock Lock(&DispatchMutex);
		FString SessionId;
		Req->TryGetStringField(TEXT("session_id"), SessionId);
		if (!ValidateSession(SessionId))
		{
			return MakeError(TEXT("session_expired"),
				FString::Printf(TEXT("Session id '%s' does not match active session"), *SessionId));
		}
	}

	const TOptional<URLabOpRegistry::FOpDecl> Decl = URLabOpRegistry::FindOp(Op);
	if (!Decl.IsSet() || !Decl->Body)
	{
		if (URLabOpRegistry::IsEditorOnlyOp(Op))
		{
			return MakeError(TEXT("not_in_editor"),
				FString::Printf(TEXT("op '%s' is editor-only and has no registered handler"), *Op));
		}
		return MakeError(TEXT("unknown_op"), FString::Printf(TEXT("Unknown op '%s'"), *Op));
	}

	// RequiredFields runs before the manager-required check so malformed
	// requests get `missing_field` regardless of PIE state.
	for (const FString& Field : Decl->RequiredFields)
	{
		if (!Req->HasField(Field))
		{
			return MakeError(TEXT("missing_field"),
				FString::Printf(TEXT("op '%s' missing required field '%s'"),
					*Op, *Field));
		}
	}

	if (Decl->Category == URLabOpRegistry::EOpCategory::ManagerRequired
		&& !OwnerMgr.IsValid())
	{
		return MakeError(TEXT("no_active_manager"),
			FString::Printf(
				TEXT("op '%s' requires an active AAMjManager (PIE not running?)"),
				*Op));
	}

	return Decl->Body(Req);
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleMeta(const TSharedPtr<FJsonObject>& /*Req*/)
{
	// Pre-session bootstrap reply: ship the full op table so the Python
	// client can synthesise `URLabClient.<namespace>.<op>` methods at
	// discover time. `meta` is intentionally NOT registered through the
	// table — it's hardcoded above, which breaks the chicken-and-egg.
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("meta_ok"));

	auto CategoryString = [](URLabOpRegistry::EOpCategory C) {
		switch (C)
		{
			case URLabOpRegistry::EOpCategory::EditorOnly:
				return TEXT("editor_only");
			case URLabOpRegistry::EOpCategory::ManagerRequired:
				return TEXT("manager_required");
			case URLabOpRegistry::EOpCategory::NoManager:
				return TEXT("no_manager");
		}
		return TEXT("unknown");
	};

	TArray<TSharedPtr<FJsonValue>> Ops;
	for (const URLabOpRegistry::FOpDecl& D : URLabOpRegistry::GetAllOps())
	{
		TSharedPtr<FJsonObject> O = MakeShared<FJsonObject>();
		O->SetStringField(TEXT("name"), D.Name);
		O->SetStringField(TEXT("category"), CategoryString(D.Category));
		O->SetStringField(TEXT("namespace"), D.Namespace);
		if (D.RequiredFields.Num() > 0)
		{
			TArray<TSharedPtr<FJsonValue>> Fields;
			for (const FString& F : D.RequiredFields)
				Fields.Add(MakeShared<FJsonValueString>(F));
			O->SetArrayField(TEXT("required_fields"), Fields);
		}
		if (D.ReplyFields.Num() > 0)
		{
			TArray<TSharedPtr<FJsonValue>> Fields;
			for (const FString& F : D.ReplyFields)
				Fields.Add(MakeShared<FJsonValueString>(F));
			O->SetArrayField(TEXT("reply_fields"), Fields);
		}
		Ops.Add(MakeShared<FJsonValueObject>(O));
	}
	Reply->SetArrayField(TEXT("ops"), Ops);
	return Reply;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::MakeError(const FString& Code, const FString& Message)
{
	TSharedPtr<FJsonObject> Err = MakeShared<FJsonObject>();
	Err->SetStringField(TEXT("op"), TEXT("error"));
	Err->SetStringField(TEXT("code"), Code);
	Err->SetStringField(TEXT("message"), Message);
	return Err;
}

void FURLabRpcDispatcher::AppendClockFields(TSharedPtr<FJsonObject>& Reply, double SimTimeSec)
{
	// ROS-Time-compatible: int32 sec + int32 nsec both fit double exactly,
	// unlike int64 ns which overflows double's 2^53 mantissa.
	const int32 SimSecI = static_cast<int32>(SimTimeSec);
	const int32 SimNsec = static_cast<int32>((SimTimeSec - SimSecI) * 1.0e9);
	{
		TSharedPtr<FJsonObject> SimTime = MakeShared<FJsonObject>();
		SimTime->SetNumberField(TEXT("sec"), SimSecI);
		SimTime->SetNumberField(TEXT("nsec"), SimNsec);
		Reply->SetObjectField(TEXT("sim_time"), SimTime);
	}

	const FDateTime Now = FDateTime::UtcNow();
	const FTimespan Delta = Now - FDateTime(1970, 1, 1);
	const int64 TotalSec = Delta.GetTotalSeconds();
	const int64 SubNs = (Delta.GetTicks() % ETimespan::TicksPerSecond) * 100;
	{
		TSharedPtr<FJsonObject> WallTime = MakeShared<FJsonObject>();
		WallTime->SetNumberField(TEXT("sec"), static_cast<double>(TotalSec));
		WallTime->SetNumberField(TEXT("nsec"), static_cast<double>(SubNs));
		Reply->SetObjectField(TEXT("wall_time"), WallTime);
	}
}

// =============================================================================
// hello
// =============================================================================

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleHello(const TSharedPtr<FJsonObject>& Req)
{
	// Editor-time clients connect before PIE to call import_xml / spawn_*;
	// BuildHandshakePayload sets manager_present=false so the bridge
	// skips PIE-only follow-ups (set_mode auto-promote, streaming SUBs).
	AAMjManager* Mgr = OwnerMgr.Get();

	// ActiveSessionId is a non-atomic FString — guard the write so a
	// concurrent ValidateSession on another transport thread reads a
	// stable value. Hello is the only writer.
	FString NewSessionId = FGuid::NewGuid().ToString(
											   EGuidFormats::DigitsWithHyphens)
							   .ToLower();
	{
		FScopeLock Lock(&DispatchMutex);
		ActiveSessionId = NewSessionId;
	}

	// Reset to msgpack default on each handshake; per-session opt-in via encoding=json.
	bUseJsonEncoding.store(false, std::memory_order_release);
	FString Encoding;
	if (Req.IsValid() && Req->TryGetStringField(TEXT("encoding"), Encoding))
	{
		const bool bJson = Encoding.Equals(TEXT("json"), ESearchCase::IgnoreCase);
		bUseJsonEncoding.store(bJson, std::memory_order_release);
	}

	// Observation level handshake. Atomic store — read on the physics
	// thread without locking.
	FString ObsLevel;
	if (Req.IsValid() && Req->TryGetStringField(TEXT("observations"), ObsLevel))
	{
		EObservationLevel Lvl = EObservationLevel::Standard;
		if (ObsLevel.Equals(TEXT("minimal"), ESearchCase::IgnoreCase))
			Lvl = EObservationLevel::Minimal;
		else if (ObsLevel.Equals(TEXT("full"), ESearchCase::IgnoreCase))
			Lvl = EObservationLevel::Full;
		ActiveObservationLevel.store(Lvl, std::memory_order_release);
	}
	StepCounter.store(0, std::memory_order_relaxed);

	bool bIncludeAssets = false;
	if (Req.IsValid())
		Req->TryGetBoolField(TEXT("include_assets"), bIncludeAssets);

	return BuildHandshakePayload(Mgr, NewSessionId, URLabVersion, bIncludeAssets);
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::BuildHandshakePayload(AAMjManager* Manager,
	const FString& SessionId,
	const FString& URLabVer,
	bool bIncludeAssets)
{
	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("hello_ok"));
	Reply->SetStringField(TEXT("session_id"), SessionId);
	Reply->SetStringField(TEXT("urlab_version"), URLabVer);
	Reply->SetNumberField(TEXT("mujoco_version_int"), mj_version());
	Reply->SetStringField(TEXT("mujoco_version"), UTF8_TO_TCHAR(mj_versionString()));

	const bool bManagerPresent = (Manager && Manager->PhysicsEngine);
	Reply->SetBoolField(TEXT("manager_present"), bManagerPresent);

	if (!bManagerPresent)
	{
		// Editor-time / pre-PIE handshake. Only editor-only ops can run
		// until PIE starts and a manager registers. Bridge sees the empty
		// articulations array + manager_present=false and skips
		// auto-promote / streaming SUB startup.
		Reply->SetArrayField(TEXT("articulations"),
			TArray<TSharedPtr<FJsonValue>>());
		return Reply;
	}

	mjModel* m = Manager->PhysicsEngine->GetModel();

	// SHM session directory. Ship an absolute path so the bridge can
	// open the SHM regions regardless of its own working directory --
	// UE's path APIs return strings relative to the engine binary, which
	// would mmap from the bridge's CWD otherwise.
	{
		FString ShmDir;
		// Snapshot publisher is a UObject in
		// ManagerOwnedPublishTransports. Walk the manager's transport
		// array to find it.
		for (const TObjectPtr<UURLabPublishTransport>& T : Manager->ManagerOwnedPublishTransports)
		{
			if (UURLabShmPublishTransport* ShmPub = Cast<UURLabShmPublishTransport>(T.Get()))
			{
				const FString StatePath = ShmPub->GetStatePath();
				if (!StatePath.IsEmpty())
				{
					ShmDir = FPaths::GetPath(StatePath);
				}
				break;
			}
		}
		if (ShmDir.IsEmpty())
		{
			ShmDir = UURLabShmPublishTransport::ResolveSessionDir(SessionId);
		}
		Reply->SetStringField(TEXT("shm_session_dir"),
			FPaths::ConvertRelativePathToFull(ShmDir));
	}

	// Explicit SHM RPC contract. The bridge must use these verbatim instead of
	// assuming session="live" / re-deriving paths or event names — a mismatch
	// is the likely cause of the SHM 5s-per-step stall. Also gives the bridge
	// everything it needs (rep_path + strides + n_buffers) to POLL the rep
	// sequence as a fallback when the named-event wakeup doesn't cross its
	// process/session boundary, instead of blocking out its full recv timeout.
	if (Manager->BridgeServer)
	{
		for (const TObjectPtr<UURLabRpcTransport>& T : Manager->BridgeServer->GetRpcTransports())
		{
			UURLabShmRpcTransport* ShmRpc = Cast<UURLabShmRpcTransport>(T.Get());
			if (!ShmRpc)
				continue;
			TSharedPtr<FJsonObject> Rpc = MakeShared<FJsonObject>();
			Rpc->SetStringField(TEXT("session"), ShmRpc->GetSessionId());
			Rpc->SetStringField(TEXT("req_path"),
				FPaths::ConvertRelativePathToFull(ShmRpc->GetReqPath()));
			Rpc->SetStringField(TEXT("rep_path"),
				FPaths::ConvertRelativePathToFull(ShmRpc->GetRepPath()));
			Rpc->SetStringField(TEXT("req_event"), ShmRpc->GetReqEventName());
			Rpc->SetStringField(TEXT("rep_event"), ShmRpc->GetRepEventName());
			Rpc->SetNumberField(TEXT("req_stride"), ShmRpc->GetReqStride());
			Rpc->SetNumberField(TEXT("rep_stride"), ShmRpc->GetRepStride());
			Rpc->SetNumberField(TEXT("n_buffers"), ShmRpc->GetNumBuffers());
			Rpc->SetNumberField(TEXT("header_size"), static_cast<double>(sizeof(FMjShmHeader)));
			Reply->SetObjectField(TEXT("shm_rpc"), Rpc);
			break;
		}
	}

	// MJB bytes: real msgpack bin under "mjb" for msgpack clients;
	// legacy "mjb_base64" / "mjb_size" stays available for JSON clients.
	if (m)
	{
		int Sz = mj_sizeModel(m);
		TArray<uint8> Buf;
		Buf.SetNum(Sz);
		mj_saveModel(m, nullptr, Buf.GetData(), Sz);
		FURLabMsgpackUtil::SetBinaryField(Reply, TEXT("mjb"), Buf.GetData(), Sz);
		FString B64 = FBase64::Encode(Buf.GetData(), Sz);
		Reply->SetStringField(TEXT("mjb_base64"), B64);
		Reply->SetNumberField(TEXT("mjb_size"), Sz);
	}

	// Optional: ship the compiled MJCF + every VFS-registered asset so
	// the client can reload the model offline (MJX, custom integrators,
	// headless renderers). Opt-in because the payload can be tens of MB
	// for typical robots.
	if (bIncludeAssets && m && Manager->PhysicsEngine->m_spec)
	{
		// Two-pass XML serialise. mj_saveXMLString returns -1 on buffer
		// overflow; start at 256 KB (covers most robots, including G1)
		// and double up to 32 MB before giving up.
		TArray<uint8> XmlBuf;
		FString XmlContent;
		for (int32 Cap = 256 * 1024; Cap <= 32 * 1024 * 1024; Cap *= 2)
		{
			XmlBuf.SetNumUninitialized(Cap);
			FMemory::Memzero(XmlBuf.GetData(), Cap);
			char SaveError[1024] = "";
			const int XmlResult = mj_saveXMLString(
				Manager->PhysicsEngine->m_spec,
				reinterpret_cast<char*>(XmlBuf.GetData()), Cap,
				SaveError, sizeof(SaveError));
			if (XmlResult == 0)
			{
				XmlContent = UTF8_TO_TCHAR(reinterpret_cast<const char*>(XmlBuf.GetData()));
				break;
			}
			// Overflow or another fault. Grow + retry. The error string
			// distinguishes "buffer too small" from real failures; for
			// any non-overflow we still break to avoid silent corruption.
			const FString Err = UTF8_TO_TCHAR(SaveError);
			if (!Err.Contains(TEXT("buffer"), ESearchCase::IgnoreCase))
			{
				UE_LOG(LogURLabNet, Warning,
					TEXT("BuildHandshake: mj_saveXMLString failed (%s); skipping mjcf_compiled"),
					*Err);
				XmlContent.Reset();
				break;
			}
		}
		if (!XmlContent.IsEmpty())
		{
			// Flatten file="dir/sub/foo.STL" -> file="foo.STL" so a VFS
			// keyed by bare filename (which is what mj_addFileVFS does)
			// can resolve the references on the client side.
			FRegexPattern Pattern(TEXT("file=\"([^\"]*?)([^/\\\\\"]+)\""));
			FRegexMatcher Matcher(Pattern, XmlContent);
			FString Rewritten;
			int32 Cursor = 0;
			while (Matcher.FindNext())
			{
				const int32 MatchStart = Matcher.GetMatchBeginning();
				const int32 MatchEnd = Matcher.GetMatchEnding();
				const FString Filename = Matcher.GetCaptureGroup(2);
				Rewritten += XmlContent.Mid(Cursor, MatchStart - Cursor);
				Rewritten += FString::Printf(TEXT("file=\"%s\""), *Filename);
				Cursor = MatchEnd;
			}
			Rewritten += XmlContent.Mid(Cursor);
			Reply->SetStringField(TEXT("mjcf_compiled"), Rewritten);
		}

		// Asset bytes: one msgpack-bin field per file, keyed by bare
		// filename (matches the flattened file= refs above). No base64
		// duplicate — clients should use the msgpack decoder.
		TSharedPtr<FJsonObject> VfsAssets = MakeShared<FJsonObject>();
		int64 TotalBytes = 0;
		for (const FString& FilePath : Manager->PhysicsEngine->ActiveAssetPaths)
		{
			TArray<uint8> FileData;
			if (FFileHelper::LoadFileToArray(FileData, *FilePath))
			{
				const FString Filename = FPaths::GetCleanFilename(FilePath);
				FURLabMsgpackUtil::SetBinaryField(VfsAssets, *Filename,
					FileData.GetData(), FileData.Num());
				TotalBytes += FileData.Num();
			}
		}
		Reply->SetObjectField(TEXT("vfs_assets"), VfsAssets);
		UE_LOG(LogURLabNet, Log,
			TEXT("BuildHandshake: shipped %d assets (%lld bytes) + mjcf_compiled (%d chars)"),
			VfsAssets->Values.Num(), TotalBytes, XmlContent.Len());
	}

	// Articulations block.
	TArray<TSharedPtr<FJsonValue>> ArtsArray;
	for (AMjArticulation* Art : Manager->GetAllArticulations())
	{
		if (!Art)
			continue;

		TSharedPtr<FJsonObject> ArtObj = MakeShared<FJsonObject>();
		ArtObj->SetStringField(TEXT("prefix"), Art->GetName());
		ArtObj->SetStringField(TEXT("actor_id"), Art->ActorId);

		// Default control mode follows whether a controller is attached.
		UMjArticulationController* Ctrl = Art->FindComponentByClass<UMjArticulationController>();
		ArtObj->SetStringField(TEXT("default_control_mode"),
			Ctrl ? TEXT("ue_controller") : TEXT("raw"));

		// Controller block — only when attached. Asks the controller for its
		// own kind, current params, and schema. ApplyConfig is not called here.
		if (Ctrl)
		{
			TSharedPtr<FJsonObject> CtrlObj = MakeShared<FJsonObject>();
			CtrlObj->SetStringField(TEXT("kind"), Ctrl->GetKindName());

			TSharedPtr<FJsonObject> Params = MakeShared<FJsonObject>();
			Ctrl->GetCurrentConfig(Params);
			if (Params.IsValid())
				CtrlObj->SetObjectField(TEXT("params"), Params);

			TSharedPtr<FJsonObject> Schema = MakeShared<FJsonObject>();
			Ctrl->GetConfigSchema(Schema);
			if (Schema.IsValid())
				CtrlObj->SetObjectField(TEXT("schema"), Schema);

			ArtObj->SetObjectField(TEXT("controller"), CtrlObj);
		}

		// Per-actuator authored kind. The MJB doesn't carry the original
		// <position> / <velocity> shortcut — they all compile to <general>.
		TSharedPtr<FJsonObject> ActTypes = MakeShared<FJsonObject>();
		for (UMjActuator* Act : Art->GetActuators())
		{
			if (!Act)
				continue;
			FString Local = Act->GetMjName();
			FString Prefix = Art->GetName() + TEXT("_");
			if (Local.StartsWith(Prefix))
				Local = Local.Mid(Prefix.Len());
			ActTypes->SetStringField(Local, ActuatorTypeToString(Act->Type));
		}
		ArtObj->SetObjectField(TEXT("actuator_types"), ActTypes);

		// Per-category map { live_short_name: original_xml_name } for
		// components whose live name was renamed by SCS / spec-time
		// dedup. The bridge resolves mjlab patterns against original
		// names. Identity entries + default-class templates are skipped.
		{
			const FString ArtPrefix = Art->GetName() + TEXT("_");
			auto MakeMap = [&ArtPrefix](auto& Components) {
				TSharedPtr<FJsonObject> Obj = MakeShared<FJsonObject>();
				for (auto* C : Components)
				{
					if (!C || C->bIsDefault)
						continue;
					if (C->OriginalMjName.IsEmpty())
						continue;
					FString Live = C->GetMjName();
					if (Live.StartsWith(ArtPrefix))
						Live = Live.Mid(ArtPrefix.Len());
					if (Live.IsEmpty() || Live == C->OriginalMjName)
						continue;
					Obj->SetStringField(Live, C->OriginalMjName);
				}
				return Obj;
			};

			TSharedPtr<FJsonObject> OriginalNames = MakeShared<FJsonObject>();

			TArray<UMjActuator*> ActComps = Art->GetActuators();
			OriginalNames->SetObjectField(TEXT("actuators"), MakeMap(ActComps));

			TArray<UMjJoint*> JointComps = Art->GetJoints();
			OriginalNames->SetObjectField(TEXT("joints"), MakeMap(JointComps));

			TArray<UMjSensor*> SensorComps;
			Art->GetComponents<UMjSensor>(SensorComps);
			OriginalNames->SetObjectField(TEXT("sensors"), MakeMap(SensorComps));

			TArray<UMjBody*> BodyComps;
			Art->GetComponents<UMjBody>(BodyComps);
			OriginalNames->SetObjectField(TEXT("bodies"), MakeMap(BodyComps));

			ArtObj->SetObjectField(TEXT("original_names"), OriginalNames);
		}

		// Camera metadata (mode, resolution, fovy, zmq endpoint/topic).
		TSharedPtr<FJsonObject> CamMap = MakeShared<FJsonObject>();
		TArray<UMjCamera*> Cameras;
		Art->GetComponents<UMjCamera>(Cameras);
		for (UMjCamera* Cam : Cameras)
		{
			if (!Cam || Cam->bIsDefault)
				continue;
			TSharedPtr<FJsonObject> CamObj = MakeShared<FJsonObject>();

			FString ModeStr = TEXT("real");
			switch (Cam->CaptureMode)
			{
				case EMjCameraMode::Real:
					ModeStr = TEXT("real");
					break;
				case EMjCameraMode::Depth:
					ModeStr = TEXT("depth");
					break;
				case EMjCameraMode::SemanticSegmentation:
					ModeStr = TEXT("semantic");
					break;
				case EMjCameraMode::InstanceSegmentation:
					ModeStr = TEXT("instance");
					break;
			}
			CamObj->SetStringField(TEXT("mode"), ModeStr);

			TArray<TSharedPtr<FJsonValue>> Res;
			Res.Add(MakeShared<FJsonValueNumber>(Cam->resolution.Num() > 0 ? Cam->resolution[0] : 0));
			Res.Add(MakeShared<FJsonValueNumber>(Cam->resolution.Num() > 1 ? Cam->resolution[1] : 0));
			CamObj->SetArrayField(TEXT("resolution"), Res);
			CamObj->SetNumberField(TEXT("fovy"), Cam->fovy);

			FString Endpoint = Cam->GetActualZmqEndpoint();
			Endpoint.ReplaceInline(TEXT("*"), TEXT("127.0.0.1"));
			CamObj->SetStringField(TEXT("zmq_endpoint"), Endpoint);
			const FString CamCanon = Cam->GetCanonicalName();
			CamObj->SetStringField(TEXT("zmq_topic"),
				FString::Printf(TEXT("%s/camera/%s"), *Art->GetName(), *CamCanon));
			CamMap->SetObjectField(CamCanon, CamObj);
		}
		ArtObj->SetObjectField(TEXT("camera_topics"), CamMap);

		ArtsArray.Add(MakeShared<FJsonValueObject>(ArtObj));
	}
	Reply->SetArrayField(TEXT("articulations"), ArtsArray);

	// Non-articulation entities. Anything dynamic in the world that isn't
	// an articulation (props, free-jointed scene objects) is keyed by
	// name with id + free-base flag so the bridge can wrap it as a
	// `URLabEntity` at handshake time. Articulations don't appear here --
	// they have their own typed block above.
	{
		TSharedPtr<FJsonObject> EntitiesObj = MakeShared<FJsonObject>();
		for (const FMjEntityRecord& R : Manager->GetEntities())
		{
			TSharedPtr<FJsonObject> EntObj = MakeShared<FJsonObject>();
			EntObj->SetNumberField(TEXT("id"), R.MjId);
			EntObj->SetBoolField(TEXT("has_free_base"), R.bHasFreeBase);
			// For free-base entities, also report the joint's qpos / qvel
			// offsets in MjData so puppet-mode clients can write back.
			if (R.bHasFreeBase && R.MjId >= 0 && R.MjId < m->nbody && m->body_jntnum && m->body_jntadr)
			{
				int FirstJnt = m->body_jntadr[R.MjId];
				int NumJnt = m->body_jntnum[R.MjId];
				if (FirstJnt >= 0 && NumJnt > 0 && FirstJnt < m->njnt && m->jnt_type[FirstJnt] == mjJNT_FREE)
				{
					EntObj->SetNumberField(TEXT("free_joint_id"), FirstJnt);
					EntObj->SetNumberField(TEXT("qpos_offset"), m->jnt_qposadr[FirstJnt]);
					EntObj->SetNumberField(TEXT("qvel_offset"), m->jnt_dofadr[FirstJnt]);
					const char* JntName = mj_id2name(m, mjOBJ_JOINT, FirstJnt);
					if (JntName)
					{
						EntObj->SetStringField(TEXT("free_joint"),
							UTF8_TO_TCHAR(JntName));
					}
				}
			}
			EntitiesObj->SetObjectField(R.Name, EntObj);
		}
		Reply->SetObjectField(TEXT("entities"), EntitiesObj);
	}

	// Reserved for future scene-level cameras.
	Reply->SetObjectField(TEXT("global_cameras"), MakeShared<FJsonObject>());

	return Reply;
}

