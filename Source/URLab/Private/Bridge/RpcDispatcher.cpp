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
#include "MuJoCo/Spec/MjSceneMjcf.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "State/MjCanonicalName.h"
#include "MuJoCo/Elements/MjActuatorRuntime.h"
#include "MuJoCo/Elements/MjSensorRuntime.h"
#include "MuJoCo/Elements/MjCamera.h"
#include "MuJoCo/Elements/MjJointRuntime.h"
#include "MuJoCo/Elements/MjBody.h"
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
#include "HAL/FileManager.h"
#include "EngineUtils.h"
#include "Engine/World.h"
#include "Misc/Guid.h"
#include "Utils/URLabLogging.h"

namespace
{
/**
 * The authored kind of a compiled actuator, recovered from the model.
 *
 * Every shortcut (`<position>` / `<velocity>` / `<motor>`) compiles down to
 * `<general>`, but the gain/bias/dyn triple still separates them: a fixed-gain,
 * no-bias, no-dyn actuator is a `motor`; an affine bias whose position term is
 * zero (bias = [0, 0, -kv]) is a `velocity` servo, otherwise it is a `position`
 * servo; an integrator dynamic with an affine bias is `intvelocity`. Anything
 * else is reported as the compiled `general`. Read straight off the mjModel so
 * the handshake needs no live articulation.
 */
FString ActuatorKindFromModel(const mjModel* M, int32 A)
{
	const int Dyn = M->actuator_dyntype[A];
	const int Gain = M->actuator_gaintype[A];
	const int Bias = M->actuator_biastype[A];
	if (Gain == mjGAIN_FIXED && Bias == mjBIAS_NONE && Dyn == mjDYN_NONE)
	{
		return TEXT("motor");
	}
	if (Gain == mjGAIN_FIXED && Bias == mjBIAS_AFFINE && Dyn == mjDYN_NONE)
	{
		const mjtNum* Bp = &M->actuator_biasprm[A * mjNBIAS];
		return (FMath::Abs(Bp[1]) < 1e-12 && FMath::Abs(Bp[2]) > 0.0) ? TEXT("velocity")
																	  : TEXT("position");
	}
	if (Gain == mjGAIN_FIXED && Bias == mjBIAS_AFFINE && Dyn == mjDYN_INTEGRATOR)
	{
		return TEXT("intvelocity");
	}
	return TEXT("general");
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
	Reg(TEXT("set_sim_options"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleSetSimOptions(R); },
		{TEXT("op:string")});
	Reg(TEXT("set_sim_speed"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleSetSimSpeed(R); },
		/*Reply=*/{TEXT("op:string"), TEXT("percent:float")},
		/*Required=*/{TEXT("percent")});
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
	Reg(TEXT("set_possess"), EOpCategory::ManagerRequired, TEXT("runtime"),
		[this](auto& R) { return HandleSetPossess(R); },
		/*Reply=*/{TEXT("op:string"), TEXT("articulation:string"), TEXT("possessed:bool")},
		/*Required=*/{TEXT("articulation")});
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

		// Visual domain randomization. resolve_geom reconciles a geom name to a
		// stable handle (compiled id + live-component census); set_geom_appearance
		// re-drives that geom's live material off the mjModel (parametric + texture
		// swap) or, with clear=true, restores its authored appearance.
		Reg(TEXT("resolve_geom"), EOpCategory::ManagerRequired, TEXT("scene"),
			[this](auto& R) { return HandleResolveGeom(R); },
			/*Reply=*/{TEXT("op:string"), TEXT("geom:string"), TEXT("found:bool"),
				TEXT("handle:string"), TEXT("mj_id:int"), TEXT("authoring:int"), TEXT("fastpath:int")},
			/*Required=*/{TEXT("geom")});
		Reg(TEXT("set_geom_appearance"), EOpCategory::ManagerRequired, TEXT("scene"),
			[this](auto& R) { return HandleSetGeomAppearance(R); },
			/*Reply=*/{TEXT("op:string"), TEXT("geom:string"), TEXT("applied:int"),
				TEXT("found:bool"), TEXT("cleared:bool")},
			/*Required=*/{TEXT("geom")});

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

	// Fast-path owner handshake (RpcHandlers_Fastpath.cpp): serve the live MJB +
	// transform-bus endpoint to a fast-path renderer. NoManager so it can report
	// `not_ready` cleanly when no session is live, matching upload_model_commit.
	Reg(TEXT("fastpath_hello"), EOpCategory::NoManager, TEXT("farm"),
		[this](auto& R) { return HandleFastpathHello(R); },
		/*Reply=*/{TEXT("op:string"), TEXT("mjb:object"), TEXT("bus:string"), TEXT("ngeom:int")});

	// Fast-path live scene swap (RpcHandlers_Fastpath.cpp): an owner/controller
	// ships new MJB bytes to a running render-server renderer, which retires its
	// current model and rebuilds from the new one without a relaunch.
	Reg(TEXT("fastpath_load"), EOpCategory::NoManager, TEXT("farm"),
		[this](auto& R) { return HandleFastpathLoad(R); },
		/*Reply=*/{TEXT("op:string"), TEXT("bytes:int")});

	// Fast-path interactive perturbation (RpcHandlers_Fastpath.cpp): a renderer
	// forwards a viewer drag as an external wrench on a body back to the owner,
	// which stamps it into the live model's xfrc_applied for its next step.
	Reg(TEXT("fastpath_perturb"), EOpCategory::NoManager, TEXT("farm"),
		[this](auto& R) { return HandleFastpathPerturb(R); },
		/*Reply=*/{TEXT("op:string"), TEXT("body:int")});

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

	// If the project pinned a pose source, lock it now so set_mode is rejected
	// until the project relaxes; an unpinned session resolves to FreeRun and the
	// client is free to promote.
	const EMjPoseSource InitMode = OwnerMgr->bPinStepMode
								 ? OwnerMgr->StepMode
								 : EMjPoseSource::FreeRun;
	{
		// Serialise the mode-enter side effects against a concurrent
		// set_mode / OnManagerGone (PIE-end) touching the same members.
		FScopeLock Lock(&DispatchMutex);
		ActiveStepMode.store(InitMode, std::memory_order_release);
		// Camera publishers stream in every mode; EnterPoseSource handles the
		// state/ctrl publishers, the engine pose source, and the handler install.
		FCameraZmqWorker::bPublishersPaused.store(false, std::memory_order_release);
		EnterPoseSource(InitMode, *OwnerMgr);
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
	// Serialise handler teardown against a concurrent set_mode racing PIE-end
	// (would otherwise be a UAF on the handler flags).
	FScopeLock Lock(&DispatchMutex);

	UninstallDirectHandler();
	DrainQueues();

	// Claims are per-PIE: the articulations die with the world.
	ControlOwnership.Reset();

	if (OwnerMgr.IsValid())
	{
		OwnerMgr->bPublishersPaused.store(false, std::memory_order_release);
		if (OwnerMgr->PhysicsEngine)
			OwnerMgr->PhysicsEngine->SetPoseSource(EMjPoseSource::FreeRun);
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
	ActiveStepMode.store(EMjPoseSource::FreeRun, std::memory_order_release);
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
	// fastpath_hello is also pre-session: a fast-path renderer only wants the MJB
	// and bus endpoint and never establishes a session. It reads no session_id.
	if (Op.Equals(TEXT("fastpath_hello")))
		return HandleFastpathHello(Req);
	// fastpath_load likewise: an owner/controller pushes a new scene to a render
	// server without a session handshake -- it just ships the MJB and returns.
	if (Op.Equals(TEXT("fastpath_load")))
		return HandleFastpathLoad(Req);
	// fastpath_perturb likewise: a renderer forwards a viewer drag over a short-lived
	// REQ with no session_id, so it bypasses the registry session gate too.
	if (Op.Equals(TEXT("fastpath_perturb")))
		return HandleFastpathPerturb(Req);

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
		// Instance capabilities (composable, orthogonal to the pose source): whether this instance
		// streams cameras and/or accepts interactive input, so a peer knows what it can ask of it.
		if (Manager->HasCapability(EMjCapability::StreamCameras))
			Capabilities.Add(MakeShared<FJsonValueString>(TEXT("stream_cameras")));
		if (Manager->HasCapability(EMjCapability::AcceptInput))
			Capabilities.Add(MakeShared<FJsonValueString>(TEXT("accept_input")));
		Instance->SetArrayField(TEXT("capabilities"), Capabilities);

		Reply->SetObjectField(TEXT("instance"), Instance);
	}

	if (!bManagerPresent)
	{
		// Editor-time / pre-PIE handshake. Only editor-only ops can run
		// until PIE starts and a manager registers. Bridge sees the empty
		// entities array + manager_present=false and skips
		// auto-promote / streaming SUB startup.
		Reply->SetArrayField(TEXT("entities"),
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
		// The text goes out exactly as the scene writer produced it. Every
		// `file=` in it already names the mount the asset pass chose, and those
		// mount names are what the VFS below is keyed by; rewriting either end
		// here would be a second naming convention for the same bytes, and the
		// one it used to apply -- reducing every reference to its basename --
		// collapsed two participants' `base.obj` onto one mount.
		FMjCompiledScene Scene;
		FString SceneError;
		if (Manager->PhysicsEngine->BuildCompiledScene(Scene, SceneError))
		{
			Reply->SetStringField(TEXT("mjcf_compiled"), Scene.Xml);
		}
		else
		{
			UE_LOG(LogURLabNet, Warning,
				TEXT("BuildHandshake: no scene MJCF (%s); skipping mjcf_compiled"), *SceneError);
		}

		// VFS bytes: one msgpack-bin field per entry, keyed by the name the
		// text asks for. Asset files come off disk; a scene that references
		// participant specs carries their text the same way, because to the
		// client's VFS the two are the same kind of thing. No base64 duplicate —
		// clients should use the msgpack decoder. Shipped whether or not the
		// MJCF made it, because a client that already has the model still needs
		// the meshes.
		TSharedPtr<FJsonObject> VfsAssets = MakeShared<FJsonObject>();
		int64 TotalBytes = 0;
		MjForEachSceneVfsEntry(Scene.AssetFiles, Scene.ParticipantXml,
			[&VfsAssets, &TotalBytes](const FString& Name, TArrayView<const uint8> Bytes) {
				FURLabMsgpackUtil::SetBinaryField(VfsAssets, Name, Bytes.GetData(), Bytes.Num());
				TotalBytes += Bytes.Num();
			});
		Reply->SetObjectField(TEXT("vfs_assets"), VfsAssets);
		UE_LOG(LogURLabNet, Log,
			TEXT("BuildHandshake: shipped %d VFS entries (%lld bytes) + mjcf_compiled (%d chars)"),
			VfsAssets->Values.Num(), TotalBytes, Scene.Xml.Len());
	}

	// Articulations block.
	TArray<TSharedPtr<FJsonValue>> ArtsArray;
	// Every camera the RPC surface serves, enumerated once (render-view cameras included).
	// Each entity's camera_topics filters this by canonical art segment, so a re-homed
	// camera is reported whether or not the authoring articulation is still alive.
	TArray<UMjCamera*> AllCameras;
	Manager->CollectCameras(AllCameras);
	for (const FMjEntity& E : Manager->PhysicsEngine->GetEntityPartition())
	{
		TSharedPtr<FJsonObject> ArtObj = MakeShared<FJsonObject>();

		// The public segment is the stable partition-derived addressing key that the state
		// stream keys arts under. The ActorId is carried on the entity from the partition
		// build, so the echo survives the authoring articulation's retirement at PIE start.
		ArtObj->SetStringField(TEXT("prefix"), E.PublicName.ToString());
		ArtObj->SetStringField(TEXT("actor_id"), E.ActorId);
		ArtObj->SetStringField(TEXT("actor_name"), E.Name.ToString());

		// Root body + free-base offsets, folded in from the retired standalone `entities` block so
		// every entity (robot or free-base body) carries them on one wire element. A state-pushing
		// client writes the base pose/vel back through the free joint's qpos/qvel addresses.
		ArtObj->SetNumberField(TEXT("id"), E.RootBodyId);
		ArtObj->SetBoolField(TEXT("has_free_base"), E.bFreeBase);
		if (E.bFreeBase && Manager->PhysicsEngine)
		{
			FScopeLock ModelLock(&Manager->PhysicsEngine->CallbackMutex);
			const mjModel* Fm = Manager->PhysicsEngine->GetModel();
			if (Fm != nullptr && E.RootBodyId >= 0 && E.RootBodyId < Fm->nbody
				&& Fm->body_jntnum && Fm->body_jntadr)
			{
				const int FirstJnt = Fm->body_jntadr[E.RootBodyId];
				const int NumJnt = Fm->body_jntnum[E.RootBodyId];
				if (FirstJnt >= 0 && NumJnt > 0 && FirstJnt < Fm->njnt
					&& Fm->jnt_type[FirstJnt] == mjJNT_FREE)
				{
					ArtObj->SetNumberField(TEXT("free_joint_id"), FirstJnt);
					ArtObj->SetNumberField(TEXT("qpos_offset"), Fm->jnt_qposadr[FirstJnt]);
					ArtObj->SetNumberField(TEXT("qvel_offset"), Fm->jnt_dofadr[FirstJnt]);
					const char* JntName = mj_id2name(Fm, mjOBJ_JOINT, FirstJnt);
					if (JntName)
					{
						ArtObj->SetStringField(TEXT("free_joint"), UTF8_TO_TCHAR(JntName));
					}
				}
			}
		}

		const bool bRawModel = (Manager->PhysicsEngine != nullptr)
			&& Manager->PhysicsEngine->IsRawModelInstalled();

		// Per-actuator authored kind. The MJB doesn't carry the original
		// <position> / <velocity> shortcut (they all compile to <general>), but the
		// gain/bias/dyn triple still distinguishes them, so recover the kind straight off
		// the compiled model for the entity's actuator ids, the same model-derived way the
		// raw path below emits its actuator block. Skipped for a raw model: its drive
		// metadata rides raw_actuators. Fenced against a concurrent compile/uninstall.
		if (!bRawModel && Manager->PhysicsEngine)
		{
			FScopeLock ModelLock(&Manager->PhysicsEngine->CallbackMutex);
			const mjModel* Cm = Manager->PhysicsEngine->GetModel();
			if (Cm != nullptr)
			{
				const FString Prefix = E.Name.IsNone() ? FString() : (E.Name.ToString() + TEXT("_"));
				TSharedPtr<FJsonObject> ActTypes = MakeShared<FJsonObject>();
				for (int32 A : E.ActuatorIds)
				{
					const char* AName = mj_id2name(Cm, mjOBJ_ACTUATOR, A);
					FString Local = (AName && *AName) ? FString(ANSI_TO_TCHAR(AName))
						: FString::Printf(TEXT("act_%d"), A);
					if (!Prefix.IsEmpty() && Local.StartsWith(Prefix))
						Local = Local.Mid(Prefix.Len());
					ActTypes->SetStringField(Local, ActuatorKindFromModel(Cm, A));
				}
				ArtObj->SetObjectField(TEXT("actuator_types"), ActTypes);
			}
		}

		// Fast-path shadow: element-only, name-bound to a raw mjModel with no
		// import/compile and no per-articulation name prefix. A client cannot load
		// the fork MJB (version skew) or prefix-match unprefixed raw names, so ship
		// the element metadata it needs to drive the model inline, read straight from
		// the installed raw model -- under the engine's fence, since a concurrent
		// compile/uninstall can retire the model pointer. Each entity ships only the
		// actuator / joint ids it owns.
		if (bRawModel && Manager->PhysicsEngine)
		{
			FScopeLock ModelLock(&Manager->PhysicsEngine->CallbackMutex);
			const mjModel* Rm = Manager->PhysicsEngine->GetModel();
			if (Rm != nullptr)
			{
				ArtObj->SetBoolField(TEXT("raw_model"), true);

				TArray<TSharedPtr<FJsonValue>> RawActs;
				for (int32 A : E.ActuatorIds)
				{
					TSharedPtr<FJsonObject> AO = MakeShared<FJsonObject>();
					const char* AName = mj_id2name(Rm, mjOBJ_ACTUATOR, A);
					AO->SetStringField(TEXT("name"),
						(AName && *AName) ? ANSI_TO_TCHAR(AName) : *FString::Printf(TEXT("act_%d"), A));
					AO->SetNumberField(TEXT("id"), A);
					if (Rm->actuator_ctrllimited[A])
					{
						TArray<TSharedPtr<FJsonValue>> R;
						R.Add(MakeShared<FJsonValueNumber>(Rm->actuator_ctrlrange[2 * A]));
						R.Add(MakeShared<FJsonValueNumber>(Rm->actuator_ctrlrange[2 * A + 1]));
						AO->SetArrayField(TEXT("ctrlrange"), R);
					}
					// Full 6-vector gear, matching the compiled-model client path.
					TArray<TSharedPtr<FJsonValue>> Gear;
					for (int32 K = 0; K < 6; ++K)
					{
						Gear.Add(MakeShared<FJsonValueNumber>(Rm->actuator_gear[6 * A + K]));
					}
					AO->SetArrayField(TEXT("gear"), Gear);
					AO->SetNumberField(TEXT("trn_type"), Rm->actuator_trntype[A]);
					if (Rm->actuator_trntype[A] == mjTRN_JOINT || Rm->actuator_trntype[A] == mjTRN_JOINTINPARENT)
					{
						const int32 Jid = Rm->actuator_trnid[2 * A];
						const char* JName = (Jid >= 0) ? mj_id2name(Rm, mjOBJ_JOINT, Jid) : nullptr;
						if (JName && *JName)
						{
							AO->SetStringField(TEXT("joint"), ANSI_TO_TCHAR(JName));
						}
					}
					RawActs.Add(MakeShared<FJsonValueObject>(AO));
				}
				ArtObj->SetArrayField(TEXT("raw_actuators"), RawActs);

				TArray<TSharedPtr<FJsonValue>> RawJnts;
				for (int32 J : E.JointIds)
				{
					TSharedPtr<FJsonObject> JO = MakeShared<FJsonObject>();
					const char* JName = mj_id2name(Rm, mjOBJ_JOINT, J);
					JO->SetStringField(TEXT("name"),
						(JName && *JName) ? ANSI_TO_TCHAR(JName) : *FString::Printf(TEXT("joint_%d"), J));
					JO->SetNumberField(TEXT("id"), J);
					JO->SetNumberField(TEXT("type"), Rm->jnt_type[J]);
					JO->SetNumberField(TEXT("body_id"), Rm->jnt_bodyid[J]);
					JO->SetNumberField(TEXT("qpos_adr"), Rm->jnt_qposadr[J]);
					JO->SetNumberField(TEXT("qvel_adr"), Rm->jnt_dofadr[J]);
					if (Rm->jnt_limited[J])
					{
						TArray<TSharedPtr<FJsonValue>> R;
						R.Add(MakeShared<FJsonValueNumber>(Rm->jnt_range[2 * J]));
						R.Add(MakeShared<FJsonValueNumber>(Rm->jnt_range[2 * J + 1]));
						JO->SetArrayField(TEXT("range"), R);
					}
					RawJnts.Add(MakeShared<FJsonValueObject>(JO));
				}
				ArtObj->SetArrayField(TEXT("raw_joints"), RawJnts);
			}
		}

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

		// Camera metadata (mode, resolution, fovy, zmq endpoint/topic). Selected from
		// the one collected camera list by canonical art segment == this entity's public
		// name, so a body-fixed camera re-homed onto the render view is reported here even
		// after the authoring articulation is gone.
		TSharedPtr<FJsonObject> CamMap = MakeShared<FJsonObject>();
		const FString EntitySeg = E.PublicName.ToString();
		for (UMjCamera* Cam : AllCameras)
		{
			if (!Cam)
				continue;
			const FString CamCanon = Cam->GetCanonicalName();
			FString CamArtSeg;
			FString CamPartSeg;
			if (!CamCanon.Split(TEXT("/"), &CamArtSeg, &CamPartSeg) || CamArtSeg != EntitySeg)
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
			CamObj->SetStringField(TEXT("zmq_topic"), CamCanon);
			CamMap->SetObjectField(CamCanon, CamObj);
		}
		ArtObj->SetObjectField(TEXT("camera_topics"), CamMap);

		ArtsArray.Add(MakeShared<FJsonValueObject>(ArtObj));
	}
	// One unified entity list: robots and free-base bodies alike are partition entities, each
	// carrying its full descriptor plus the folded-in root-body / free-base fields. The bridge
	// wraps every element as an entity (the rich subtype when it owns actuators/joints).
	Reply->SetArrayField(TEXT("entities"), ArtsArray);

	// Reserved for future scene-level cameras.
	Reply->SetObjectField(TEXT("global_cameras"), MakeShared<FJsonObject>());

	return Reply;
}
