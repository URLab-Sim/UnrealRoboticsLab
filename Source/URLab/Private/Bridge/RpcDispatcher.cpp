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
#include "Bridge/RpcErrorCodes.h"
#include "Bridge/OpRegistry.h"
#include "Bridge/StepCommands.h"
#include "Bridge/MsgpackHelpers.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Spec/MjElementIdentity.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "State/MjCanonicalName.h"
#include "MuJoCo/Elements/MjActuatorRuntime.h"
#include "MuJoCo/Elements/MjSensorRuntime.h"
#include "MuJoCo/Elements/MjCamera.h"
#include "MuJoCo/Elements/MjJointRuntime.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Controllers/MjArticulationController.h"
#include "MuJoCo/Input/MjPerturbation.h"
#include "MuJoCo/Input/MjTwistController.h"
#include "Transport/NetworkManager.h"
#include "Transport/ShmPublishTransport.h"
#include "Transport/ShmRpcTransport.h"
#include "Transport/RpcTransport.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/BridgeServerConfig.h"
#include "Bridge/InstanceRegistry.h"
#include "HAL/PlatformProcess.h"
#include "Transport/ShmRegion.h" // FMjShmHeader (header_size in shm_rpc block)
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
/**
 * The authored kind of an actuator, as the wire spells it.
 *
 * The kind is the element itself -- a `<position>` is a different element from a
 * `<motor>` -- so this is the element's own MJCF tag and there is no mapping
 * table to keep in step with the schema. The compiled model cannot answer it:
 * every shortcut compiles down to `<general>`.
 */
FString ActuatorTypeToString(const UMjNodeComponent* Actuator)
{
#if URLAB_MJ_GEN
	urlab::spec::psm::ElementType Type;
	if (Actuator != nullptr && urlab::spec::MjElementTypeOfNode(*Actuator, Type))
	{
		return urlab::spec::MjTagOf(Type);
	}
#endif
	return TEXT("motor");
}

/** An element's authored MJCF name with the articulation prefix taken off. */
FString LocalElementName(const UMjNodeComponent& Element, const FString& ArtPrefix)
{
	FString Name = Element.MjName.Get(Element.GetName());
	if (Name.StartsWith(ArtPrefix))
	{
		Name = Name.Mid(ArtPrefix.Len());
	}
	return Name;
}

/** Ops that do NOT count as lease-owner activity: discovery, bootstrap, and
 *  lifecycle/status polls that a non-owner (e.g. a pool client probing for a
 *  free instance) issues to inspect an instance. Refreshing the lease on these
 *  would let steady discovery polling keep a held lease alive forever and
 *  defeat TTL auto-release. Real owner activity (step / reset / forward / set_*
 *  and every other state-advancing op) still refreshes. */
bool OpRefreshesLease(const FString& Op)
{
	static const TSet<FString> NonActivityOps = {
		TEXT("hello"),
		TEXT("meta"),
		TEXT("pie_status"),
		TEXT("op_status"),
		TEXT("acquire_lease"),
		TEXT("release_lease"),
	};
	return !NonActivityOps.Contains(Op);
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
	Reg(TEXT("claim_control"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleClaimControl(R); },
		/*Reply=*/{TEXT("op:string"), TEXT("articulation:string"), TEXT("owner:string"), TEXT("ttl_s:float")},
		/*Required=*/{TEXT("articulation")});
	Reg(TEXT("release_control"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleReleaseControl(R); },
		/*Reply=*/{TEXT("op:string"), TEXT("articulation:string")},
		/*Required=*/{TEXT("articulation")});
	Reg(TEXT("set_twist"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleSetTwist(R); },
		{TEXT("op:string")});
	Reg(TEXT("set_user_channels"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleSetUserChannels(R); },
		/*Reply=*/{TEXT("op:string"), TEXT("applied:int"), TEXT("rejected:object")});
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

	// Cooperative render-farm lease. No-manager: a pool client claims the
	// process itself, whether or not a scene is loaded.
	Reg(TEXT("acquire_lease"), EOpCategory::NoManager, TEXT("farm"),
		[this](auto& R) { return HandleAcquireLease(R); },
		{TEXT("op:string"), TEXT("lease_id:string"), TEXT("ttl_s:float")});
	// lease_id is validated in the handler (not declaratively) so a missing
	// value returns `bad_request` per the schema, not the generic
	// `missing_field`.
	Reg(TEXT("release_lease"), EOpCategory::NoManager, TEXT("farm"),
		[this](auto& R) { return HandleReleaseLease(R); },
		/*Reply=*/{TEXT("op:string")});

	// Network model upload (RpcHandlers_ModelUpload.cpp). Manifest + chunk are
	// pure data staging (no manager, no editor). Commit drives the existing
	// import_xml editor job on a materialised temp dir; it self-checks for the
	// editor import handler, so it stays NoManager and returns `import_failed`
	// when the editor module isn't loaded.
	Reg(TEXT("upload_model_manifest"), EOpCategory::NoManager, TEXT("scene"),
		[this](auto& R) { return HandleUploadModelManifest(R); },
		/*Reply=*/{TEXT("op:string"), TEXT("upload_id:string"), TEXT("need_xml:bool"), TEXT("need_assets:array"), TEXT("max_asset_bytes:int"), TEXT("max_total_bytes:int")},
		/*Required=*/{TEXT("xml_sha256")});
	Reg(TEXT("upload_model_chunk"), EOpCategory::NoManager, TEXT("scene"),
		[this](auto& R) { return HandleUploadModelChunk(R); },
		/*Reply=*/{TEXT("op:string"), TEXT("name:string"), TEXT("received:int"), TEXT("complete:bool")},
		/*Required=*/{TEXT("upload_id"), TEXT("kind")});
	Reg(TEXT("upload_model_commit"), EOpCategory::NoManager, TEXT("scene"),
		[this](auto& R) { return HandleUploadModelCommit(R); },
		/*Reply=*/{TEXT("op:string"), TEXT("imported:bool"), TEXT("nq:int?"), TEXT("nv:int?"), TEXT("nu:int?"), TEXT("nbody:int?"), TEXT("ngeom:int?"), TEXT("mjb:object?"), TEXT("warnings:array")},
		/*Required=*/{TEXT("upload_id")});

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
	{
		// Serialise strategy construction + OnEnter against a concurrent
		// set_mode / OnManagerGone (PIE-end) touching the same members.
		FScopeLock Lock(&DispatchMutex);
		ActiveStepMode.store(InitMode, std::memory_order_release);
		// Camera publishers stream in every mode; the strategy's OnEnter handles
		// the state/ctrl publishers, the engine step mode, and the handler install.
		FCameraZmqWorker::bPublishersPaused.store(false, std::memory_order_release);
		CurrentStepStrategy = MakeStepStrategy(InitMode);
		CurrentStepStrategy->OnEnter(*this, *OwnerMgr);
	}

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
	// Serialise handler teardown + strategy reset against a concurrent set_mode
	// racing PIE-end (would otherwise be a UAF on CurrentStepStrategy / the
	// handler flags).
	FScopeLock Lock(&DispatchMutex);

	UninstallDirectHandler();
	CurrentStepStrategy.Reset();
	DrainQueues();

	// Claims are per-PIE: the articulations die with the world.
	ControlOwnership.Reset();

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

// Out-of-line so the TWeakObjectPtr assignment sees the full UURLabBridgeServer
// type (the header only forward-declares it to avoid an include cycle).
void FURLabRpcDispatcher::SetOwningBridge(UURLabBridgeServer* InBridge)
{
	OwningBridge = InBridge;
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

void FURLabRpcDispatcher::DrainQueues()
{
	TSharedPtr<FMjDirectStepCommand> Cmd;
	while (StepQueue.Dequeue(Cmd))
	{
		if (!Cmd.IsValid())
			continue;
		// Abandon + wake any RPC thread blocked on this command so a mode switch
		// / PIE-end returns immediately instead of eating the 5s step deadline.
		Cmd->bAbandoned.store(true, std::memory_order_release);
		if (Cmd->Completion)
			Cmd->Completion->Trigger();
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
		return MakeError(URLabError::BadRequest, TEXT("Failed to parse request (json or msgpack)"));
	}

	FString Op;
	if (!Req->TryGetStringField(TEXT("op"), Op))
	{
		return MakeError(URLabError::MissingOp, TEXT("Request missing 'op' field"));
	}

	// Only real owner activity refreshes the lease. Discovery / bootstrap /
	// status-poll ops (see OpRefreshesLease) must NOT, or a pool client's
	// steady `hello`/status polling would keep a held lease alive forever and
	// defeat TTL auto-release. No-op when no lease is held.
	if (OpRefreshesLease(Op))
	{
		if (UURLabBridgeServer* Bridge = OwningBridge.Get())
			Bridge->TouchLease();
	}

	// hello / meta are pre-session ops that run before a client has a
	// session_id, so they bypass the registry. Both validate fields inline
	// (all hello fields are optional with sensible defaults; meta takes none).
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
			return MakeError(URLabError::SessionExpired,
				FString::Printf(TEXT("Session id '%s' does not match active session"), *SessionId));
		}
	}

	const TOptional<URLabOpRegistry::FOpDecl> Decl = URLabOpRegistry::FindOp(Op);
	if (!Decl.IsSet() || !Decl->Body)
	{
		if (URLabOpRegistry::IsEditorOnlyOp(Op))
		{
			return MakeError(URLabError::NotInEditor,
				FString::Printf(TEXT("op '%s' is editor-only and has no registered handler"), *Op));
		}
		return MakeError(URLabError::UnknownOp, FString::Printf(TEXT("Unknown op '%s'"), *Op));
	}

	// RequiredFields runs before the manager-required check so malformed
	// requests get `missing_field` regardless of PIE state.
	for (const FString& Field : Decl->RequiredFields)
	{
		if (!Req->HasField(Field))
		{
			return MakeError(URLabError::MissingField,
				FString::Printf(TEXT("op '%s' missing required field '%s'"),
					*Op, *Field));
		}
	}

	if (Decl->Category == URLabOpRegistry::EOpCategory::ManagerRequired
		&& !OwnerMgr.IsValid())
	{
		return MakeError(URLabError::NoActiveManager,
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

	FString ClientVersion;
	if (Req.IsValid())
		Req->TryGetStringField(TEXT("client_version"), ClientVersion);
	UE_LOG(LogURLabNet, Log, TEXT("URLab client connected: %s (server %s)"),
		*ClientVersion, *URLabVersion);

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

	// Per-instance identity for render-farm discovery. Values come from the
	// bridge server's resolved config (env / command line / INI), so a client
	// learns exactly which instance answered and on which ports.
	if (Manager && Manager->BridgeServer)
	{
		const FURLabBridgeServerConfig& Cfg = Manager->BridgeServer->GetInstanceConfig();
		TSharedPtr<FJsonObject> Instance = MakeShared<FJsonObject>();
		Instance->SetStringField(TEXT("instance_id"),
			Cfg.InstanceId.IsEmpty() ? FString(TEXT("live")) : Cfg.InstanceId);
		Instance->SetNumberField(TEXT("index"), Cfg.InstanceIndex);
		Instance->SetNumberField(TEXT("pid"),
			static_cast<double>(FPlatformProcess::GetCurrentProcessId()));
		Instance->SetStringField(TEXT("host"), FPlatformProcess::ComputerName());
		Instance->SetNumberField(TEXT("step_port"), Cfg.StepPort);
		Instance->SetNumberField(TEXT("state_port"), Cfg.StatePort);
		Instance->SetNumberField(TEXT("cam_base_port"), Cfg.CamBasePort);
		Instance->SetBoolField(TEXT("manager_present"), bManagerPresent);
		Instance->SetBoolField(TEXT("busy"), Manager->BridgeServer->IsLeaseHeld());
		Instance->SetStringField(TEXT("urlab_version"), URLabVer);

		TArray<TSharedPtr<FJsonValue>> Capabilities;
		for (const FString& Cap : FURLabInstanceRegistry::Capabilities())
			Capabilities.Add(MakeShared<FJsonValueString>(Cap));
		Instance->SetArrayField(TEXT("capabilities"), Capabilities);

		Reply->SetObjectField(TEXT("instance"), Instance);
	}

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

	// Let each bound transport append its own block to the handshake.
	// SHM publish transport sets shm_session_dir; SHM RPC transport sets
	// shm_rpc (paths/events/strides). Adding a third transport type
	// requires no changes here — just override AppendHandshakeBlock.
	for (const TObjectPtr<UURLabPublishTransport>& T : Manager->ManagerOwnedPublishTransports)
	{
		T->AppendHandshakeBlock(Reply);
	}

	// Fall back to the static helper when no publish transport set
	// shm_session_dir (e.g. no SHM transport bound).
	if (!Reply->HasField(TEXT("shm_session_dir")))
	{
		const FString ShmDir = UURLabShmPublishTransport::ResolveSessionDir(SessionId);
		Reply->SetStringField(TEXT("shm_session_dir"),
			FPaths::ConvertRelativePathToFull(ShmDir));
	}

	if (Manager->BridgeServer)
	{
		for (const TObjectPtr<UURLabRpcTransport>& T : Manager->BridgeServer->GetRpcTransports())
		{
			T->AppendHandshakeBlock(Reply);
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
	if (bIncludeAssets && m)
	{
		// Flatten file="dir/sub/foo.STL" -> file="foo.STL" so a VFS keyed by
		// bare filename (which is what mj_addFileVFS does) can resolve the
		// references on the client side.
		auto FlattenAssetRefs = [](const FString& Xml) {
			FRegexPattern Pattern(TEXT("file=\"([^\"]*?)([^/\\\\\"]+)\""));
			FRegexMatcher Matcher(Pattern, Xml);
			FString Rewritten;
			int32 Cursor = 0;
			while (Matcher.FindNext())
			{
				const int32 MatchStart = Matcher.GetMatchBeginning();
				const int32 MatchEnd = Matcher.GetMatchEnding();
				const FString Filename = Matcher.GetCaptureGroup(2);
				Rewritten += Xml.Mid(Cursor, MatchStart - Cursor);
				Rewritten += FString::Printf(TEXT("file=\"%s\""), *Filename);
				Cursor = MatchEnd;
			}
			Rewritten += Xml.Mid(Cursor);
			return Rewritten;
		};

		FMjCompiledScene Scene;
		FString SceneError;
		if (Manager->PhysicsEngine->BuildCompiledScene(Scene, SceneError))
		{
			Reply->SetStringField(TEXT("mjcf_compiled"), FlattenAssetRefs(Scene.Xml));
		}
		else
		{
			UE_LOG(LogURLabNet, Warning,
				TEXT("BuildHandshake: no scene MJCF (%s); skipping mjcf_compiled"), *SceneError);
		}

		// VFS bytes: one msgpack-bin field per entry, keyed by the bare name
		// the flattened file= refs above resolve against. Asset files come off
		// disk; a scene that references participant specs carries their
		// text the same way, because to the client's VFS the two are the same
		// kind of thing. No base64 duplicate — clients should use the msgpack
		// decoder. Shipped whether or not the MJCF made it, because a client
		// that already has the model still needs the meshes.
		TSharedPtr<FJsonObject> VfsAssets = MakeShared<FJsonObject>();
		int64 TotalBytes = 0;
		for (const TPair<FString, FString>& Asset : Scene.AssetFiles)
		{
			TArray<uint8> FileData;
			if (FFileHelper::LoadFileToArray(FileData, *Asset.Value))
			{
				// The name the scene mounted it under, not the file's own. A
				// scene prefixes every `file=` so participants cannot collide in
				// MuJoCo's flat VFS namespace, and the refs flattened above keep
				// that prefix -- so a key rebuilt from the path names a file the
				// spec never asks for.
				FURLabMsgpackUtil::SetBinaryField(VfsAssets, *Asset.Key,
					FileData.GetData(), FileData.Num());
				TotalBytes += FileData.Num();
			}
		}
		for (const TPair<FString, FString>& Participant : Scene.ParticipantXml)
		{
			const FTCHARToUTF8 Utf8(*FlattenAssetRefs(Participant.Value));
			FURLabMsgpackUtil::SetBinaryField(VfsAssets, *Participant.Key,
				reinterpret_cast<const uint8*>(Utf8.Get()), Utf8.Length());
			TotalBytes += Utf8.Length();
		}
		Reply->SetObjectField(TEXT("vfs_assets"), VfsAssets);
		UE_LOG(LogURLabNet, Log,
			TEXT("BuildHandshake: shipped %d VFS entries (%lld bytes) + mjcf_compiled (%d chars)"),
			VfsAssets->Values.Num(), TotalBytes, Scene.Xml.Len());
	}

	// Articulations block.
	TArray<TSharedPtr<FJsonValue>> ArtsArray;
	for (AMjArticulation* Art : Manager->GetAllArticulations())
	{
		if (!Art)
			continue;

		TSharedPtr<FJsonObject> ArtObj = MakeShared<FJsonObject>();
		// The public segment (ActorId-based) is the art's topic / observation
		// namespace, matching the IR Art.Name the state stream keys arts under.
		ArtObj->SetStringField(TEXT("prefix"), FMjCanonicalName::ArtSegment(Art).ToString());
		ArtObj->SetStringField(TEXT("actor_id"), Art->ActorId);
		ArtObj->SetStringField(TEXT("actor_name"), Art->GetName());

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
		for (const UMjNodeComponent* Act : Art->GetActuators())
		{
			if (Act == nullptr)
				continue;
			ActTypes->SetStringField(LocalElementName(*Act, Art->GetCompiledPrefix()),
				ActuatorTypeToString(Act));
		}
		ArtObj->SetObjectField(TEXT("actuator_types"), ActTypes);

		// Per-category map { live_short_name: original_xml_name } for
		// components whose live name was renamed by SCS / spec-time
		// dedup. The bridge resolves mjlab patterns against original
		// names. Identity entries + default-class templates are skipped.
		// Live-to-authored name maps, one per category. They are empty and the
		// wire contract keeps them: the reader writes the MJCF `name` attribute
		// into the element and nothing renames it afterwards, so an element's
		// live name IS its authored name. The maps existed because the component
		// tree went through Blueprint variable naming, which deduplicated names
		// the MJCF had kept distinct.
		{
			TSharedPtr<FJsonObject> OriginalNames = MakeShared<FJsonObject>();
			OriginalNames->SetObjectField(TEXT("actuators"), MakeShared<FJsonObject>());
			OriginalNames->SetObjectField(TEXT("joints"), MakeShared<FJsonObject>());
			OriginalNames->SetObjectField(TEXT("sensors"), MakeShared<FJsonObject>());
			OriginalNames->SetObjectField(TEXT("bodies"), MakeShared<FJsonObject>());
			ArtObj->SetObjectField(TEXT("original_names"), OriginalNames);
		}

		// Camera metadata (mode, resolution, fovy, zmq endpoint/topic).
		TSharedPtr<FJsonObject> CamMap = MakeShared<FJsonObject>();
		TArray<UMjCamera*> Cameras;
		Art->GetComponents<UMjCamera>(Cameras);
		for (UMjCamera* Cam : Cameras)
		{
			if (!Cam)
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

			const FIntPoint CamRes = Cam->CaptureResolution();
			TArray<TSharedPtr<FJsonValue>> Res;
			Res.Add(MakeShared<FJsonValueNumber>(CamRes.X));
			Res.Add(MakeShared<FJsonValueNumber>(CamRes.Y));
			CamObj->SetArrayField(TEXT("resolution"), Res);
			CamObj->SetNumberField(TEXT("fovy"), Cam->DerivedFovy);

			FString Endpoint = Cam->GetActualZmqEndpoint();
			Endpoint.ReplaceInline(TEXT("*"), TEXT("127.0.0.1"));
			CamObj->SetStringField(TEXT("zmq_endpoint"), Endpoint);
			const FString CamCanon = Cam->GetCanonicalName();
			CamObj->SetStringField(TEXT("zmq_topic"), CamCanon);
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
