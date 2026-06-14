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
    /** Map enum to wire-format string, matching the Python StepMode enum values. */
    FString StepModeToString(EStepMode Mode)
    {
        switch (Mode)
        {
        case EStepMode::Live:   return TEXT("live");
        case EStepMode::Direct: return TEXT("direct");
        case EStepMode::Puppet: return TEXT("puppet");
        case EStepMode::Auto:   return TEXT("auto");
        }
        return TEXT("live");
    }

    bool StepModeFromString(const FString& Str, EStepMode& OutMode)
    {
        if (Str.Equals(TEXT("live"),      ESearchCase::IgnoreCase) ||
            Str.Equals(TEXT("streaming"), ESearchCase::IgnoreCase))
        { OutMode = EStepMode::Live; return true; }
        if (Str.Equals(TEXT("direct"), ESearchCase::IgnoreCase))
        { OutMode = EStepMode::Direct; return true; }
        if (Str.Equals(TEXT("puppet"), ESearchCase::IgnoreCase))
        { OutMode = EStepMode::Puppet; return true; }
        if (Str.Equals(TEXT("auto"), ESearchCase::IgnoreCase))
        { OutMode = EStepMode::Auto; return true; }
        return false;
    }

    /** Map URLab actuator enum -> wire-format string. */
    FString ActuatorTypeToString(EMjActuatorType T)
    {
        switch (T)
        {
        case EMjActuatorType::Motor:        return TEXT("motor");
        case EMjActuatorType::Position:     return TEXT("position");
        case EMjActuatorType::Velocity:     return TEXT("velocity");
        case EMjActuatorType::IntVelocity:  return TEXT("intvelocity");
        case EMjActuatorType::Damper:       return TEXT("damper");
        case EMjActuatorType::Cylinder:     return TEXT("cylinder");
        case EMjActuatorType::Muscle:       return TEXT("muscle");
        case EMjActuatorType::Adhesion:     return TEXT("adhesion");
        case EMjActuatorType::DcMotor:      return TEXT("dcmotor");
        }
        return TEXT("motor");
    }

    /** Resolve a save / load path. Bare filename -> <Project>/Saved/URLab/Replays/.
     *  Must match AMjReplayManager::SaveRecordingToFile so the
     *  recording_save_ok absolute_path actually resolves on the bridge. */
    FString ResolveReplayPath(const FString& UserPath, const FString& DefaultBaseName = TEXT(""))
    {
        FString BaseDir = FPaths::ProjectSavedDir() / TEXT("URLab") / TEXT("Replays");
        IFileManager::Get().MakeDirectory(*BaseDir, true);

        FString Path = UserPath;
        if (Path.IsEmpty())
        {
            Path = DefaultBaseName.IsEmpty() ? TEXT("recording.json") : DefaultBaseName;
        }

        if (FPaths::IsRelative(Path))
        {
            Path = FPaths::Combine(BaseDir, Path);
        }
        return FPaths::ConvertRelativePathToFull(Path);
    }
}

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
                       std::initializer_list<const TCHAR*> RequiredFields = {})
    {
        URLabOpRegistry::FOpDecl Decl;
        Decl.Name = Name;
        Decl.Category = Cat;
        Decl.Namespace = Ns;
        Decl.Body = MoveTemp(Body);
        for (const TCHAR* F : ReplyFields)    Decl.ReplyFields.Add(F);
        for (const TCHAR* F : RequiredFields) Decl.RequiredFields.Add(F);
        // Record before MoveTemp consumes Decl.
        RegisteredOpNames.Add(Decl.Name);
        URLabOpRegistry::RegisterOp(MoveTemp(Decl));
    };

    Reg(TEXT("step"),                  EOpCategory::ManagerRequired, TEXT(""),
        [this](auto& R){ return HandleStep(R); },
        {TEXT("op:string"), TEXT("time:float"), TEXT("step:int"),
         TEXT("per_articulation:object"),
         TEXT("sim_time:object?"), TEXT("wall_time:object?")});
    Reg(TEXT("reset"),                 EOpCategory::ManagerRequired, TEXT(""),
        [this](auto& R){ return HandleReset(R); },
        {TEXT("op:string"), TEXT("time:float"), TEXT("step:int")});
    Reg(TEXT("forward"),               EOpCategory::ManagerRequired, TEXT("runtime"),
        [this](auto& R){ return HandleForward(R); },
        {TEXT("op:string"), TEXT("time:float"), TEXT("step:int"), TEXT("per_articulation:object")});
    Reg(TEXT("set_mode"),              EOpCategory::ManagerRequired, TEXT("runtime"),
        [this](auto& R){ return HandleSetMode(R); },
        {TEXT("op:string"), TEXT("previous_mode:string"), TEXT("current_mode:string")});
    Reg(TEXT("set_paused"),            EOpCategory::ManagerRequired, TEXT("runtime"),
        [this](auto& R){ return HandleSetPaused(R); },
        /*Reply=*/   {TEXT("op:string"), TEXT("paused:bool")},
        /*Required=*/{TEXT("paused")});
    Reg(TEXT("configure_controller"), EOpCategory::ManagerRequired, TEXT("runtime"),
        [this](auto& R){ return HandleConfigureController(R); },
        /*Reply=*/   {TEXT("op:string"), TEXT("articulation:string"), TEXT("params:object")},
        /*Required=*/{TEXT("articulation"), TEXT("params")});
    Reg(TEXT("set_sim_options"),       EOpCategory::ManagerRequired, TEXT("runtime"),
        [this](auto& R){ return HandleSetSimOptions(R); },
        {TEXT("op:string")});
    Reg(TEXT("set_sim_speed"),         EOpCategory::ManagerRequired, TEXT("runtime"),
        [this](auto& R){ return HandleSetSimSpeed(R); },
        /*Reply=*/   {TEXT("op:string"), TEXT("percent:float")},
        /*Required=*/{TEXT("percent")});
    Reg(TEXT("set_control_source"),    EOpCategory::ManagerRequired, TEXT("runtime"),
        [this](auto& R){ return HandleSetControlSource(R); },
        {TEXT("op:string")});
    Reg(TEXT("set_twist"),             EOpCategory::ManagerRequired, TEXT("runtime"),
        [this](auto& R){ return HandleSetTwist(R); },
        {TEXT("op:string")});
    Reg(TEXT("set_qpos"),              EOpCategory::ManagerRequired, TEXT("runtime"),
        [this](auto& R){ return HandleSetQpos(R); },
        /*Reply=*/   {TEXT("op:string"), TEXT("target:string"), TEXT("actor_id:string?"),
                      TEXT("actor_name:string?"), TEXT("qpos:array"),
                      TEXT("free_base_shortcut:bool")},
        /*Required=*/{TEXT("target"), TEXT("qpos")});
    Reg(TEXT("set_mocap_pose"),        EOpCategory::ManagerRequired, TEXT("runtime"),
        [this](auto& R){ return HandleSetMocapPose(R); },
        /*Reply=*/   {TEXT("op:string"), TEXT("body:string"),
                      TEXT("pos:array"), TEXT("quat:array")},
        /*Required=*/{TEXT("body")});
    Reg(TEXT("read_mocap_pose"),       EOpCategory::ManagerRequired, TEXT("runtime"),
        [this](auto& R){ return HandleReadMocapPose(R); },
        /*Reply=*/   {TEXT("op:string"), TEXT("body:string"),
                      TEXT("pos:array"), TEXT("quat:array")},
        /*Required=*/{TEXT("body")});
    Reg(TEXT("get_contacts"),          EOpCategory::ManagerRequired, TEXT("runtime"),
        [this](auto& R){ return HandleGetContacts(R); },
        /*Reply=*/   {TEXT("op:string"), TEXT("n_contacts:int"),
                      TEXT("truncated:bool"), TEXT("contacts:array")});
    Reg(TEXT("list_keyframes"),        EOpCategory::ManagerRequired, TEXT("runtime"),
        [this](auto& R){ return HandleListKeyframes(R); },
        {TEXT("op:string"), TEXT("keyframes:array")});

    auto RecBody = [this](auto& R)
    {
        FString OpName;
        R->TryGetStringField(TEXT("op"), OpName);
        return HandleRecording(OpName, R);
    };
    Reg(TEXT("recording_start"), EOpCategory::ManagerRequired, TEXT("recording"), RecBody);
    Reg(TEXT("recording_stop"),  EOpCategory::ManagerRequired, TEXT("recording"), RecBody);
    Reg(TEXT("recording_save"),  EOpCategory::ManagerRequired, TEXT("recording"), RecBody);
    Reg(TEXT("recording_clear"), EOpCategory::ManagerRequired, TEXT("recording"), RecBody);

    auto RepBody = [this](auto& R)
    {
        FString OpName;
        R->TryGetStringField(TEXT("op"), OpName);
        return HandleReplay(OpName, R);
    };
    Reg(TEXT("replay_load"),          EOpCategory::ManagerRequired, TEXT("replay"), RepBody);
    Reg(TEXT("replay_list_sessions"), EOpCategory::ManagerRequired, TEXT("replay"), RepBody);
    Reg(TEXT("replay_set_active"),    EOpCategory::ManagerRequired, TEXT("replay"), RepBody);
    Reg(TEXT("replay_start"),         EOpCategory::ManagerRequired, TEXT("replay"), RepBody);
    Reg(TEXT("replay_stop"),          EOpCategory::ManagerRequired, TEXT("replay"), RepBody);
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
    if (!OwnerMgr.IsValid()) return;

    // If the project pinned a non-Auto mode, lock it now so set_mode
    // is rejected until the project relaxes.
    const EStepMode InitMode = (OwnerMgr->StepMode == EStepMode::Auto)
        ? EStepMode::Live : OwnerMgr->StepMode;
    ActiveStepMode.store(InitMode, std::memory_order_release);
    const bool bPaused = (InitMode != EStepMode::Live);
    OwnerMgr->bPublishersPaused.store(bPaused, std::memory_order_release);
    FCameraZmqWorker::bPublishersPaused.store(bPaused, std::memory_order_release);

    if (InitMode == EStepMode::Puppet)      InstallPuppetHandler();
    else if (InitMode == EStepMode::Direct) InstallDirectHandler();

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
    DrainQueuesForTest();

    if (OwnerMgr.IsValid())
    {
        OwnerMgr->bPublishersPaused.store(false, std::memory_order_release);
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
    while (StepQueue.Dequeue(Cmd)) {}  // shared_ptr deallocates on scope exit
    FMjPushStateRequest P;
    while (PushStateQueue.Dequeue(P)) {}
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::Dispatch(const TSharedPtr<FJsonObject>& Req)
{
    const double StartTime = FPlatformTime::Seconds();
    FString InOp = TEXT("<no-op>");
    if (Req.IsValid()) Req->TryGetStringField(TEXT("op"), InOp);
    UE_LOG(LogURLabNet, Log, TEXT("RPC -> %s"), *InOp);

    TSharedPtr<FJsonObject> Reply = DispatchInternal(Req);

    const double DurationMs = (FPlatformTime::Seconds() - StartTime) * 1000.0;
    FString ReplyOp;
    if (Reply.IsValid()) Reply->TryGetStringField(TEXT("op"), ReplyOp);

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
    if (Op.Equals(TEXT("hello"))) return HandleHello(Req);
    if (Op.Equals(TEXT("meta")))  return HandleMeta(Req);

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

    auto CategoryString = [](URLabOpRegistry::EOpCategory C)
    {
        switch (C)
        {
            case URLabOpRegistry::EOpCategory::EditorOnly:      return TEXT("editor_only");
            case URLabOpRegistry::EOpCategory::ManagerRequired: return TEXT("manager_required");
            case URLabOpRegistry::EOpCategory::NoManager:       return TEXT("no_manager");
        }
        return TEXT("unknown");
    };

    TArray<TSharedPtr<FJsonValue>> Ops;
    for (const URLabOpRegistry::FOpDecl& D : URLabOpRegistry::GetAllOps())
    {
        TSharedPtr<FJsonObject> O = MakeShared<FJsonObject>();
        O->SetStringField(TEXT("name"),       D.Name);
        O->SetStringField(TEXT("category"),   CategoryString(D.Category));
        O->SetStringField(TEXT("namespace"),  D.Namespace);
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

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetPaused(const TSharedPtr<FJsonObject>& Req)
{
    AAMjManager* Mgr = OwnerMgr.Get();
    if (!Mgr || !Mgr->PhysicsEngine)
        return MakeError(TEXT("not_ready"), TEXT("PhysicsEngine not initialised"));

    bool bPause = false;
    if (!Req->TryGetBoolField(TEXT("paused"), bPause))
        return MakeError(TEXT("missing_field"), TEXT("set_paused requires 'paused' bool"));

    Mgr->PhysicsEngine->SetPaused(bPause);
    UE_LOG(LogURLabNet, Log, TEXT("FURLabRpcDispatcher: set_paused -> %s"),
        bPause ? TEXT("true") : TEXT("false"));

    TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
    Reply->SetStringField(TEXT("op"),     TEXT("set_paused_ok"));
    Reply->SetBoolField(TEXT("paused"),   Mgr->PhysicsEngine->bIsPaused);
    return Reply;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::MakeError(const FString& Code, const FString& Message)
{
    TSharedPtr<FJsonObject> Err = MakeShared<FJsonObject>();
    Err->SetStringField(TEXT("op"),      TEXT("error"));
    Err->SetStringField(TEXT("code"),    Code);
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
        SimTime->SetNumberField(TEXT("sec"),  SimSecI);
        SimTime->SetNumberField(TEXT("nsec"), SimNsec);
        Reply->SetObjectField(TEXT("sim_time"), SimTime);
    }

    const FDateTime Now = FDateTime::UtcNow();
    const FTimespan Delta = Now - FDateTime(1970, 1, 1);
    const int64 TotalSec = Delta.GetTotalSeconds();
    const int64 SubNs = (Delta.GetTicks() % ETimespan::TicksPerSecond) * 100;
    {
        TSharedPtr<FJsonObject> WallTime = MakeShared<FJsonObject>();
        WallTime->SetNumberField(TEXT("sec"),  static_cast<double>(TotalSec));
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
        EGuidFormats::DigitsWithHyphens).ToLower();
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
        if      (ObsLevel.Equals(TEXT("minimal"), ESearchCase::IgnoreCase)) Lvl = EObservationLevel::Minimal;
        else if (ObsLevel.Equals(TEXT("full"),    ESearchCase::IgnoreCase)) Lvl = EObservationLevel::Full;
        ActiveObservationLevel.store(Lvl, std::memory_order_release);
    }
    StepCounter.store(0, std::memory_order_relaxed);

    bool bIncludeAssets = false;
    if (Req.IsValid()) Req->TryGetBoolField(TEXT("include_assets"), bIncludeAssets);

    return BuildHandshakePayload(Mgr, NewSessionId, URLabVersion, bIncludeAssets);
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::BuildHandshakePayload(AAMjManager* Manager,
                                                              const FString& SessionId,
                                                              const FString& URLabVer,
                                                              bool bIncludeAssets)
{
    TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
    Reply->SetStringField(TEXT("op"),         TEXT("hello_ok"));
    Reply->SetStringField(TEXT("session_id"), SessionId);
    Reply->SetStringField(TEXT("urlab_version"), URLabVer);
    Reply->SetNumberField(TEXT("mujoco_version_int"), mj_version());
    Reply->SetStringField(TEXT("mujoco_version"),     UTF8_TO_TCHAR(mj_versionString()));

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
        Reply->SetNumberField(TEXT("mjb_size"),   Sz);
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
                const int32 MatchEnd   = Matcher.GetMatchEnding();
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
        if (!Art) continue;

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
            if (!Act) continue;
            FString Local = Act->GetMjName();
            FString Prefix = Art->GetName() + TEXT("_");
            if (Local.StartsWith(Prefix)) Local = Local.Mid(Prefix.Len());
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
                    if (!C || C->bIsDefault) continue;
                    if (C->OriginalMjName.IsEmpty()) continue;
                    FString Live = C->GetMjName();
                    if (Live.StartsWith(ArtPrefix)) Live = Live.Mid(ArtPrefix.Len());
                    if (Live.IsEmpty() || Live == C->OriginalMjName) continue;
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
            if (!Cam || Cam->bIsDefault) continue;
            TSharedPtr<FJsonObject> CamObj = MakeShared<FJsonObject>();

            FString ModeStr = TEXT("real");
            switch (Cam->CaptureMode)
            {
            case EMjCameraMode::Real:                 ModeStr = TEXT("real");     break;
            case EMjCameraMode::Depth:                ModeStr = TEXT("depth");    break;
            case EMjCameraMode::SemanticSegmentation: ModeStr = TEXT("semantic"); break;
            case EMjCameraMode::InstanceSegmentation: ModeStr = TEXT("instance"); break;
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
            CamObj->SetStringField(TEXT("zmq_topic"),
                FString::Printf(TEXT("%s/camera/%s"), *Art->GetName(), *Cam->GetName()));
            CamMap->SetObjectField(Cam->GetName(), CamObj);
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
            if (R.bHasFreeBase && R.MjId >= 0 && R.MjId < m->nbody &&
                m->body_jntnum && m->body_jntadr)
            {
                int FirstJnt = m->body_jntadr[R.MjId];
                int NumJnt   = m->body_jntnum[R.MjId];
                if (FirstJnt >= 0 && NumJnt > 0 && FirstJnt < m->njnt &&
                    m->jnt_type[FirstJnt] == mjJNT_FREE)
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

// =============================================================================
// step
// =============================================================================

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleStep(const TSharedPtr<FJsonObject>& Req)
{
    AAMjManager* Mgr = OwnerMgr.Get();
    if (!Mgr || !Mgr->PhysicsEngine || !Mgr->PhysicsEngine->m_model)
    {
        return MakeError(TEXT("not_ready"), TEXT("PhysicsEngine not initialised"));
    }

    if (ActiveStepMode == EStepMode::Live)
    {
        // Live mode: UE drives its own physics. step() applies ctrl and
        // returns current state. n_steps is ignored (UE steps at its own
        // rate). Same per_articulation shape as direct mode.
        mjModel* m = Mgr->PhysicsEngine->GetModel();
        mjData*  d = Mgr->PhysicsEngine->GetData();

        FMjStepRequest TmpReq;
        const TSharedPtr<FJsonObject>* PerArt = nullptr;
        if (Req->TryGetObjectField(TEXT("per_articulation"), PerArt) && PerArt && PerArt->IsValid())
        {
            for (auto& Pair : (*PerArt)->Values)
            {
                const TSharedPtr<FJsonObject>* ArtObj = nullptr;
                if (!Pair.Value->TryGetObject(ArtObj) || !ArtObj || !ArtObj->IsValid()) continue;

                FString CtlMode;
                if ((*ArtObj)->TryGetStringField(TEXT("control_mode"), CtlMode))
                    TmpReq.PerArticulationControlMode.Add(Pair.Key, CtlMode);

                const TArray<TSharedPtr<FJsonValue>>* CtrlList = nullptr;
                if ((*ArtObj)->TryGetArrayField(TEXT("ctrl"), CtrlList) && CtrlList)
                {
                    if (AMjArticulation* Art = Cast<AMjArticulation>(Mgr->GetArticulation(Pair.Key)))
                    {
                        TArray<UMjActuator*> Acts = Art->GetActuators();
                        for (int32 i = 0; i < CtrlList->Num() && i < Acts.Num(); ++i)
                        {
                            UMjActuator* A = Acts[i];
                            if (!A) continue;
                            FString Local = A->GetMjName();
                            FString Prefix = Art->GetName() + TEXT("_");
                            if (Local.StartsWith(Prefix)) Local = Local.Mid(Prefix.Len());
                            TmpReq.PerArticulationCtrl.FindOrAdd(Pair.Key).Add(
                                { Local, (float)(*CtrlList)[i]->AsNumber() });
                        }
                    }
                }
            }
        }

        TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
        {
            FScopeLock Lock(&Mgr->PhysicsEngine->CallbackMutex);
            ApplyStepCtrl(Mgr, TmpReq, m, d);
            Reply->SetStringField(TEXT("op"),    TEXT("step_ok"));
            Reply->SetNumberField(TEXT("time"),  d->time);
            Reply->SetNumberField(TEXT("step"),  StepCounter.load(std::memory_order_relaxed));
            AppendClockFields(Reply, d->time);
            TSharedPtr<FJsonObject> Obs = BuildStepObservations(Mgr, m, d, ActiveObservationLevel);
            if (Obs.IsValid())   Reply->SetObjectField(TEXT("per_articulation"), Obs);
            TSharedPtr<FJsonObject> Scene = BuildEntitiesBlock(Mgr, m, d);
            if (Scene.IsValid()) Reply->SetObjectField(TEXT("entities"), Scene);
        }
        return Reply;
    }

    // Per-step observations override (does not change the session default).
    FString StepObs;
    if (Req->TryGetStringField(TEXT("observations"), StepObs))
    {
        if      (StepObs.Equals(TEXT("minimal"),  ESearchCase::IgnoreCase)) ActiveObservationLevel = EObservationLevel::Minimal;
        else if (StepObs.Equals(TEXT("full"),     ESearchCase::IgnoreCase)) ActiveObservationLevel = EObservationLevel::Full;
        else if (StepObs.Equals(TEXT("standard"), ESearchCase::IgnoreCase)) ActiveObservationLevel = EObservationLevel::Standard;
    }

    // Parse include_cameras: accepts either
    //   - object: { "<cam_name>": "sync"|"latest" } -- per-camera mode
    //   - bool true: all registered cameras at "latest" mode (legacy form)
    //   - false/absent: no cameras
    TMap<FString, ECameraInclude> CameraSpec;
    {
        const TSharedPtr<FJsonObject>* CamObj = nullptr;
        if (Req->TryGetObjectField(TEXT("include_cameras"), CamObj) && CamObj && CamObj->IsValid())
        {
            for (const auto& Kv : (*CamObj)->Values)
            {
                FString Mode;
                if (Kv.Value.IsValid() && Kv.Value->TryGetString(Mode))
                {
                    ECameraInclude E = ECameraInclude::Latest;
                    if (Mode.Equals(TEXT("sync"), ESearchCase::IgnoreCase)) E = ECameraInclude::Sync;
                    CameraSpec.Add(Kv.Key, E);
                }
            }
        }
        else
        {
            bool bAll = false;
            if (Req->TryGetBoolField(TEXT("include_cameras"), bAll) && bAll)
            {
                for (AMjArticulation* Art : Mgr->GetAllArticulations())
                {
                    if (!Art) continue;
                    TArray<UMjCamera*> Cams;
                    Art->GetComponents<UMjCamera>(Cams);
                    for (UMjCamera* C : Cams)
                    {
                        if (!C || C->bIsDefault) continue;
                        CameraSpec.Add(C->GetName(), ECameraInclude::Latest);
                    }
                }
            }
        }
    }

    if (ActiveStepMode == EStepMode::Puppet)
    {
        FMjPushStateRequest Push;
        const TArray<TSharedPtr<FJsonValue>>* QPosArr = nullptr;
        const TArray<TSharedPtr<FJsonValue>>* QVelArr = nullptr;
        const TArray<TSharedPtr<FJsonValue>>* CtrlArr = nullptr;
        if (Req->TryGetArrayField(TEXT("qpos"), QPosArr))
        {
            Push.QPos.Reserve(QPosArr->Num());
            for (auto& V : *QPosArr) Push.QPos.Add(V->AsNumber());
        }
        if (Req->TryGetArrayField(TEXT("qvel"), QVelArr))
        {
            Push.QVel.Reserve(QVelArr->Num());
            for (auto& V : *QVelArr) Push.QVel.Add(V->AsNumber());
        }
        if (Req->TryGetArrayField(TEXT("ctrl"), CtrlArr))
        {
            Push.bIncludeCtrl = true;
            Push.Ctrl.Reserve(CtrlArr->Num());
            for (auto& V : *CtrlArr) Push.Ctrl.Add(V->AsNumber());
        }
        double TimeVal = 0.0;
        Req->TryGetNumberField(TEXT("time"), TimeVal);
        Push.Time = TimeVal;

        mjModel* m = Mgr->PhysicsEngine->GetModel();
        mjData*  d = Mgr->PhysicsEngine->GetData();

        {
            FScopeLock Lock(&Mgr->PhysicsEngine->CallbackMutex);
            if (Push.QPos.Num() == m->nq)
                FMemory::Memcpy(d->qpos, Push.QPos.GetData(), m->nq * sizeof(mjtNum));
            if (Push.QVel.Num() == m->nv)
                FMemory::Memcpy(d->qvel, Push.QVel.GetData(), m->nv * sizeof(mjtNum));
            if (Push.bIncludeCtrl && Push.Ctrl.Num() == m->nu)
                FMemory::Memcpy(d->ctrl, Push.Ctrl.GetData(), m->nu * sizeof(mjtNum));
            d->time = Push.Time;
            mj_forward(m, d);

            if (Mgr->PhysicsEngine->OnPostStep)
                Mgr->PhysicsEngine->OnPostStep(m, d);
        }

        TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
        Reply->SetStringField(TEXT("op"),   TEXT("step_ok"));
        Reply->SetNumberField(TEXT("time"), d->time);
        Reply->SetNumberField(TEXT("step"), StepCounter.fetch_add(1, std::memory_order_relaxed) + 1);
        AppendClockFields(Reply, d->time);
        TSharedPtr<FJsonObject> Obs = BuildStepObservations(Mgr, m, d, ActiveObservationLevel);
        if (Obs.IsValid()) Reply->SetObjectField(TEXT("per_articulation"), Obs);
        TSharedPtr<FJsonObject> Scene = BuildEntitiesBlock(Mgr, m, d);
        if (Scene.IsValid()) Reply->SetObjectField(TEXT("entities"), Scene);
        if (CameraSpec.Num() > 0)
        {
            TSharedPtr<FJsonObject> Cams = BuildCamerasBlock(Mgr, CameraSpec);
            if (Cams.IsValid() && Cams->Values.Num() > 0)
                Reply->SetObjectField(TEXT("cameras"), Cams);
        }
        // Puppet-mode perturbation: include the latest sample so the client
        // can apply the editor click-drag widget's force to its own MjData.
        if (Mgr->Perturbation)
        {
            FMjPerturbationSample Sample = Mgr->Perturbation->GetLatestPerturbationSample();
            if (Sample.BodyId > 0)
            {
                TSharedPtr<FJsonObject> Pert = MakeShared<FJsonObject>();
                Pert->SetNumberField(TEXT("body_id"), Sample.BodyId);
                Pert->SetNumberField(TEXT("version"), Sample.Version);
                TArray<TSharedPtr<FJsonValue>> Six;
                for (int i = 0; i < 6; ++i)
                    Six.Add(MakeShared<FJsonValueNumber>(Sample.Xfrc[i]));
                Pert->SetArrayField(TEXT("xfrc"), Six);
                Reply->SetObjectField(TEXT("perturbation"), Pert);
            }
        }
        return Reply;
    }

    // Direct mode.
    TSharedPtr<FMjDirectStepCommand> Cmd = MakeShared<FMjDirectStepCommand>();
    int32 NSteps = 1;
    Req->TryGetNumberField(TEXT("n_steps"), NSteps);
    Cmd->Request.NSteps = NSteps > 0 ? NSteps : 1;

    const TSharedPtr<FJsonObject>* PerArt = nullptr;
    if (Req->TryGetObjectField(TEXT("per_articulation"), PerArt) && PerArt && PerArt->IsValid())
    {
        for (auto& Pair : (*PerArt)->Values)
        {
            const TSharedPtr<FJsonObject>* ArtObj = nullptr;
            if (!Pair.Value->TryGetObject(ArtObj) || !ArtObj || !ArtObj->IsValid()) continue;

            // control_mode override
            FString CtlMode;
            if ((*ArtObj)->TryGetStringField(TEXT("control_mode"), CtlMode))
            {
                Cmd->Request.PerArticulationControlMode.Add(Pair.Key, CtlMode);
            }

            const TArray<TSharedPtr<FJsonValue>>* CtrlList = nullptr;
            if ((*ArtObj)->TryGetArrayField(TEXT("ctrl"), CtrlList) && CtrlList)
            {
                // Positional ctrl array: indexed in articulation actuator order.
                if (AMjArticulation* Art = Cast<AMjArticulation>(Mgr->GetArticulation(Pair.Key)))
                {
                    TArray<UMjActuator*> Acts = Art->GetActuators();
                    for (int32 i = 0; i < CtrlList->Num() && i < Acts.Num(); ++i)
                    {
                        UMjActuator* A = Acts[i];
                        if (!A) continue;
                        FString LocalName = A->GetMjName();
                        FString Prefix = Art->GetName() + TEXT("_");
                        if (LocalName.StartsWith(Prefix)) LocalName = LocalName.Mid(Prefix.Len());
                        Cmd->Request.PerArticulationCtrl.FindOrAdd(Pair.Key).Add(
                            { LocalName, (float)(*CtrlList)[i]->AsNumber() });
                    }
                }
            }

            // Named ctrl map alternative.
            const TSharedPtr<FJsonObject>* CtrlMap = nullptr;
            if ((*ArtObj)->TryGetObjectField(TEXT("ctrl_map"), CtrlMap) && CtrlMap && CtrlMap->IsValid())
            {
                for (auto& KV : (*CtrlMap)->Values)
                {
                    Cmd->Request.PerArticulationCtrl.FindOrAdd(Pair.Key).Add(
                        { KV.Key, (float)KV.Value->AsNumber() });
                }
            }

            // xfrc_applied: { body_name: [fx,fy,fz,tx,ty,tz] }. Cleared after step.
            const TSharedPtr<FJsonObject>* XfrcMap = nullptr;
            if ((*ArtObj)->TryGetObjectField(TEXT("xfrc_applied"), XfrcMap) && XfrcMap && XfrcMap->IsValid())
            {
                TMap<FString, TArray<double>>& BodyMap = Cmd->Request.PerArticulationXfrc.FindOrAdd(Pair.Key);
                for (auto& KV : (*XfrcMap)->Values)
                {
                    const TArray<TSharedPtr<FJsonValue>>* Arr = nullptr;
                    if (KV.Value->TryGetArray(Arr) && Arr && Arr->Num() == 6)
                    {
                        TArray<double>& Six = BodyMap.FindOrAdd(KV.Key);
                        Six.SetNum(6);
                        for (int i = 0; i < 6; ++i) Six[i] = (*Arr)[i]->AsNumber();
                    }
                }
            }
        }
    }

    // Submit to physics-thread custom handler (no race) then wait. If the
    // engine isn't running its async loop (test path), fall back to inline.
    Cmd->Completion = FPlatformProcess::GetSynchEventFromPool(true);

    const bool bAsyncRunning = Mgr->PhysicsEngine->AsyncPhysicsFuture.IsValid();
    StepQueue.Enqueue(Cmd);
    if (Mgr->PhysicsEngine->StepRequestEvent)
    {
        Mgr->PhysicsEngine->StepRequestEvent->Trigger();
    }

    if (!bAsyncRunning)
    {
        // Test / editor path: pump the handler synchronously so we don't
        // block forever waiting for an engine that isn't ticking.
        if (Mgr->PhysicsEngine->CustomStepHandler)
        {
            Mgr->PhysicsEngine->CustomStepHandler(
                Mgr->PhysicsEngine->GetModel(),
                Mgr->PhysicsEngine->GetData());
        }
    }

    // 5-second hard cap so a wedged engine returns an error rather than
    // wedging the RPC thread. Polled in 50ms slices so the bDraining
    // flag (set when the bridge is being stopped) can short-circuit the
    // wait without forcing the user to sit through the full 5s while
    // the editor closes.
    bool bSignaled = false;
    {
        const double Deadline = FPlatformTime::Seconds() + 5.0;
        while (FPlatformTime::Seconds() < Deadline)
        {
            if (bDraining.load(std::memory_order_acquire))
            {
                break;
            }
            if (Cmd->Completion->Wait(FTimespan::FromMilliseconds(50)))
            {
                bSignaled = true;
                break;
            }
        }
    }

    TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
    if (bSignaled && Cmd->bDone)
    {
        Reply->SetStringField(TEXT("op"),   TEXT("step_ok"));
        Reply->SetNumberField(TEXT("time"), Cmd->ResultTime);
        Reply->SetNumberField(TEXT("step"), Cmd->ResultStep);
        AppendClockFields(Reply, Cmd->ResultTime);
        if (Cmd->Observations.IsValid()) Reply->SetObjectField(TEXT("per_articulation"), Cmd->Observations);
        if (Cmd->Entities.IsValid())  Reply->SetObjectField(TEXT("entities"),    Cmd->Entities);

        if (CameraSpec.Num() > 0)
        {
            TSharedPtr<FJsonObject> Cams = BuildCamerasBlock(Mgr, CameraSpec);
            if (Cams.IsValid() && Cams->Values.Num() > 0)
                Reply->SetObjectField(TEXT("cameras"), Cams);
        }
    }
    else
    {
        if (bDraining.load(std::memory_order_acquire))
        {
            Reply = MakeError(TEXT("shutting_down"),
                TEXT("Bridge stopping; Direct-mode step abandoned"));
        }
        else
        {
            Reply = MakeError(TEXT("step_timeout"),
                TEXT("Direct-mode step did not complete within 5s"));
        }
    }

    return Reply;
}

void FURLabRpcDispatcher::ApplyStepCtrl(AAMjManager* Manager, const FMjStepRequest& Req,
                                   mjModel* m, mjData* d)
{
    if (!Manager) return;
    for (auto& Pair : Req.PerArticulationCtrl)
    {
        AMjArticulation* Art = Manager->GetArticulation(Pair.Key);
        if (!Art) continue;

        const FString Prefix = Art->GetName() + TEXT("_");
        TMap<FString, UMjActuator*> ByName;
        ByName.Reserve(Art->GetActuators().Num() * 2);
        for (UMjActuator* A : Art->GetActuators())
        {
            if (!A) continue;
            FString FullName = A->GetMjName();
            FString Local    = FullName.StartsWith(Prefix) ? FullName.Mid(Prefix.Len()) : FullName;
            ByName.Add(Local, A);
            ByName.Add(FullName, A);
        }

        // Stage to actuator NetworkValue; AMjArticulation::ApplyControls
        // copies it into d->ctrl every sub-step. Writing d->ctrl directly
        // would be overwritten on the next sub-step.
        for (const TPair<FString, float>& KV : Pair.Value)
        {
            UMjActuator** Found = ByName.Find(KV.Key);
            if (!Found || !*Found) continue;
            (*Found)->SetNetworkControl(KV.Value);
        }
    }

    // xfrc_applied writes: per_articulation -> body_name -> 6-vec.
    // MuJoCo clears d->xfrc_applied on every mj_step, so this is a one-shot
    // impulse for the next mj_step n_steps loop. Body name lookup tries both
    // the local (no-prefix) form and the prefixed full name.
    if (m && d)
    {
        for (auto& APair : Req.PerArticulationXfrc)
        {
            FString ArtPrefix = APair.Key + TEXT("_");
            for (auto& BPair : APair.Value)
            {
                if (BPair.Value.Num() != 6) continue;
                FString FullName = ArtPrefix + BPair.Key;
                int Bid = mj_name2id(m, mjOBJ_BODY, TCHAR_TO_UTF8(*FullName));
                if (Bid < 0)
                    Bid = mj_name2id(m, mjOBJ_BODY, TCHAR_TO_UTF8(*BPair.Key));
                if (Bid < 0 || Bid >= m->nbody) continue;
                for (int i = 0; i < 6; ++i)
                    d->xfrc_applied[6 * Bid + i] = (mjtNum)BPair.Value[i];
            }
        }
    }
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::BuildStepObservations(AAMjManager* Manager, mjModel* m, mjData* d,
                                                                EObservationLevel Level)
{
    TSharedPtr<FJsonObject> PerArt = MakeShared<FJsonObject>();
    if (!Manager || !m || !d) return PerArt;

    const bool bWantStandard = (Level == EObservationLevel::Standard) || (Level == EObservationLevel::Full);
    const bool bWantFull     = (Level == EObservationLevel::Full);

    for (AMjArticulation* Art : Manager->GetAllArticulations())
    {
        if (!Art) continue;
        TSharedPtr<FJsonObject> ArtObj = MakeShared<FJsonObject>();

        // qpos / qvel — present at every level. Joints are emitted in the
        // discovery order that GetJoints() returns, which matches the MjModel
        // jnt_id order at compile time. Per-joint qpos/qvel slot widths follow
        // jnt_type. Wire-side consumers should always rebuild full m.nq / m.nv
        // arrays from per-articulation slices in this same order.
        TArray<UMjJoint*> Joints = Art->GetJoints();
        TArray<TSharedPtr<FJsonValue>> QPos;
        TArray<TSharedPtr<FJsonValue>> QVel;
        for (UMjJoint* J : Joints)
        {
            if (!J) continue;
            int32 Id = J->GetMjID();
            if (Id < 0 || Id >= m->njnt) continue;
            int QAddr = m->jnt_qposadr[Id];
            int VAddr = m->jnt_dofadr[Id];
            int QSize = 1, VSize = 1;
            switch (m->jnt_type[Id])
            {
            case mjJNT_FREE:  QSize = 7; VSize = 6; break;
            case mjJNT_BALL:  QSize = 4; VSize = 3; break;
            case mjJNT_SLIDE:
            case mjJNT_HINGE: QSize = 1; VSize = 1; break;
            }
            for (int i = 0; i < QSize; ++i)
                QPos.Add(MakeShared<FJsonValueNumber>(d->qpos[QAddr + i]));
            for (int i = 0; i < VSize; ++i)
                QVel.Add(MakeShared<FJsonValueNumber>(d->qvel[VAddr + i]));
        }
        ArtObj->SetArrayField(TEXT("qpos"), QPos);
        ArtObj->SetArrayField(TEXT("qvel"), QVel);

        if (bWantStandard)
        {
            // ctrl positional array, same order as GetActuators().
            TArray<TSharedPtr<FJsonValue>> Ctrl;
            TArray<TSharedPtr<FJsonValue>> Act;
            for (UMjActuator* A : Art->GetActuators())
            {
                if (!A) continue;
                int32 Id = A->GetMjID();
                if (Id < 0 || Id >= m->nu) continue;
                Ctrl.Add(MakeShared<FJsonValueNumber>(d->ctrl[Id]));
                // Each actuator's "act" slot, if it has one (intvelocity, muscle, ...)
                int ActAddr = m->actuator_actadr ? m->actuator_actadr[Id] : -1;
                if (ActAddr >= 0 && ActAddr < m->na)
                    Act.Add(MakeShared<FJsonValueNumber>(d->act[ActAddr]));
                else
                    Act.Add(MakeShared<FJsonValueNumber>(0.0));
            }
            ArtObj->SetArrayField(TEXT("ctrl"), Ctrl);
            ArtObj->SetArrayField(TEXT("act"),  Act);

            // sensors by name — use sensor MjID + dim.
            TSharedPtr<FJsonObject> Sensors = MakeShared<FJsonObject>();
            TArray<UMjSensor*> SensorComponents;
            Art->GetComponents<UMjSensor>(SensorComponents);
            FString Prefix = Art->GetName() + TEXT("_");
            for (UMjSensor* S : SensorComponents)
            {
                if (!S) continue;
                int32 Sid = S->GetMjID();
                if (Sid < 0 || Sid >= m->nsensor) continue;
                int Adr = m->sensor_adr[Sid];
                int Dim = m->sensor_dim[Sid];
                if (Adr < 0 || Dim <= 0 || (Adr + Dim) > m->nsensordata) continue;
                TArray<TSharedPtr<FJsonValue>> Vals;
                for (int i = 0; i < Dim; ++i)
                    Vals.Add(MakeShared<FJsonValueNumber>(d->sensordata[Adr + i]));
                FString LocalName = S->GetMjName();
                if (LocalName.StartsWith(Prefix)) LocalName = LocalName.Mid(Prefix.Len());
                Sensors->SetArrayField(LocalName, Vals);
            }
            ArtObj->SetObjectField(TEXT("sensors"), Sensors);
        }

        if (bWantFull)
        {
            // body xpos/xquat — discovered through articulation's MjBody components.
            TSharedPtr<FJsonObject> Bodies = MakeShared<FJsonObject>();
            TArray<UMjBody*> BodyComponents;
            Art->GetComponents<UMjBody>(BodyComponents);
            FString Prefix = Art->GetName() + TEXT("_");
            for (UMjBody* B : BodyComponents)
            {
                if (!B || B->bIsDefault) continue;
                int32 Bid = B->GetMjID();
                if (Bid < 0 || Bid >= m->nbody) continue;
                TSharedPtr<FJsonObject> Bo = MakeShared<FJsonObject>();
                TArray<TSharedPtr<FJsonValue>> XPos, XQuat;
                for (int i = 0; i < 3; ++i) XPos.Add(MakeShared<FJsonValueNumber>(d->xpos[Bid*3 + i]));
                for (int i = 0; i < 4; ++i) XQuat.Add(MakeShared<FJsonValueNumber>(d->xquat[Bid*4 + i]));
                Bo->SetArrayField(TEXT("xpos"),  XPos);
                Bo->SetArrayField(TEXT("xquat"), XQuat);
                FString LocalName = B->GetMjName();
                if (LocalName.StartsWith(Prefix)) LocalName = LocalName.Mid(Prefix.Len());
                Bodies->SetObjectField(LocalName, Bo);
            }
            ArtObj->SetObjectField(TEXT("bodies"), Bodies);

            // actuator_force per actuator (positional, same order as ctrl)
            TArray<TSharedPtr<FJsonValue>> AForce;
            for (UMjActuator* A : Art->GetActuators())
            {
                if (!A) continue;
                int32 Id = A->GetMjID();
                if (Id < 0 || Id >= m->nu) continue;
                AForce.Add(MakeShared<FJsonValueNumber>(d->actuator_force[Id]));
            }
            ArtObj->SetArrayField(TEXT("actuator_force"), AForce);
        }

        // geometry_msgs/Twist-aligned: (linear.x, linear.y, angular.z)
        // filled; rest stays zero. Only when a TwistController is attached.
        if (UMjTwistController* TwistCtrl = Art->FindComponentByClass<UMjTwistController>())
        {
            const FVector Twist = TwistCtrl->GetTwist();  // (Vx, Vy, YawRate)

            TArray<TSharedPtr<FJsonValue>> Linear;
            Linear.Add(MakeShared<FJsonValueNumber>(Twist.X));
            Linear.Add(MakeShared<FJsonValueNumber>(Twist.Y));
            Linear.Add(MakeShared<FJsonValueNumber>(0.0));

            TArray<TSharedPtr<FJsonValue>> Angular;
            Angular.Add(MakeShared<FJsonValueNumber>(0.0));
            Angular.Add(MakeShared<FJsonValueNumber>(0.0));
            Angular.Add(MakeShared<FJsonValueNumber>(Twist.Z));

            TSharedPtr<FJsonObject> TwistObj = MakeShared<FJsonObject>();
            TwistObj->SetArrayField(TEXT("linear"),  Linear);
            TwistObj->SetArrayField(TEXT("angular"), Angular);
            ArtObj->SetObjectField(TEXT("twist"), TwistObj);

            ArtObj->SetNumberField(TEXT("actions"),
                static_cast<double>(TwistCtrl->GetActiveActions()));
        }

        PerArt->SetObjectField(Art->GetName(), ArtObj);
    }
    return PerArt;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::BuildCamerasBlock(AAMjManager* Manager,
    const TMap<FString, ECameraInclude>& CameraSpec, int32 TimeoutMs)
{
    TSharedPtr<FJsonObject> Cams = MakeShared<FJsonObject>();
    if (!Manager || CameraSpec.Num() == 0) return Cams;

    UWorld* World = Manager->GetWorld();
    if (!World) return Cams;

    // Build a name -> camera lookup using the canonical "<articulation>/<camera>"
    // form the handshake exposes plus the bare camera name as a fallback.
    TMap<FString, UMjCamera*> ByName;
    for (AMjArticulation* Art : Manager->GetAllArticulations())
    {
        if (!Art) continue;
        TArray<UMjCamera*> Cameras;
        Art->GetComponents<UMjCamera>(Cameras);
        for (UMjCamera* C : Cameras)
        {
            if (!C || C->bIsDefault) continue;
            FString Qualified = Art->GetName() + TEXT("/") + C->GetName();
            ByName.Add(Qualified, C);
            ByName.Add(C->GetName(), C);
        }
    }

    for (const TPair<FString, ECameraInclude>& Spec : CameraSpec)
    {
        UMjCamera** Found = ByName.Find(Spec.Key);
        if (!Found || !*Found)
        {
            UE_LOG(LogURLabNet, Verbose,
                TEXT("[BuildCamerasBlock] camera '%s' not found"), *Spec.Key);
            continue;
        }
        UMjCamera* Cam = *Found;

        // Streaming auto-enables in UMjCamera::BeginPlay via
        // UMjNetworkManager::RegisterCamera (bEnableAllCameras=true by
        // default), so by the time we hit this path the camera is
        // already streaming. The earlier worker-thread
        // SetStreamingEnabled call here was dead code; left only the
        // game-thread RequestReadback marshalling below, which is
        // the actual fix — RenderTarget->GameThread_GetRenderTargetResource
        // returns null on non-game threads and silently bails the
        // readback, which is why include_cameras saw empty frames.

        // For "sync" we kick a readback and poll. RequestReadback
        // must run on the game thread (it touches
        // RenderTarget->GameThread_GetRenderTargetResource and enqueues
        // a render command); calling it from the bridge worker thread
        // makes Resource null and the readback never lands. We marshal
        // here and wait briefly; the per-tick auto-readback in
        // UMjCamera::TickComponent keeps PendingPixels fresh for the
        // "latest" path so consumers without an explicit sync call
        // still get frames.
        if (Spec.Value == ECameraInclude::Sync)
        {
            FEvent* ReqDone = FPlatformProcess::GetSynchEventFromPool(/*bIsManualReset=*/false);
            TWeakObjectPtr<UMjCamera> WeakCam(Cam);
            AsyncTask(ENamedThreads::GameThread, [WeakCam, ReqDone]()
            {
                if (UMjCamera* C = WeakCam.Get())
                {
                    C->RequestReadback();
                }
                ReqDone->Trigger();
            });
            ReqDone->Wait(2000);
            FPlatformProcess::ReturnSynchEventToPool(ReqDone);

            const double DeadlineSec = FPlatformTime::Seconds() + (TimeoutMs / 1000.0);
            while (!Cam->IsReadbackReady() && FPlatformTime::Seconds() < DeadlineSec)
            {
                FPlatformProcess::Sleep(0.001f);
            }
        }

        TSharedPtr<FJsonObject> CamObj = MakeShared<FJsonObject>();
        CamObj->SetNumberField(TEXT("width"),  Cam->resolution.Num() > 0 ? Cam->resolution[0] : 0);
        CamObj->SetNumberField(TEXT("height"), Cam->resolution.Num() > 1 ? Cam->resolution[1] : 0);

        if (Cam->CaptureMode == EMjCameraMode::Depth)
        {
            TArray<float> Pixels = Cam->ConsumeFloatPixels();
            if (Pixels.Num() == 0) continue;
            CamObj->SetStringField(TEXT("dtype"), TEXT("float32"));
            FURLabMsgpackUtil::SetBinaryField(CamObj, TEXT("data"),
                reinterpret_cast<const uint8*>(Pixels.GetData()),
                Pixels.Num() * sizeof(float));
        }
        else
        {
            TArray<FColor> Pixels = Cam->ConsumePixels();
            if (Pixels.Num() == 0) continue;
            // Real / SemSeg / InstanceSeg all ship 4-byte BGRA. Bridge
            // discriminates the seg modes by the camera_topics handshake.
            CamObj->SetStringField(TEXT("dtype"), TEXT("bgra8"));
            FURLabMsgpackUtil::SetBinaryField(CamObj, TEXT("data"),
                reinterpret_cast<const uint8*>(Pixels.GetData()),
                Pixels.Num() * sizeof(FColor));
        }
        Cams->SetObjectField(Spec.Key, CamObj);
    }
    return Cams;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::BuildEntitiesBlock(AAMjManager* Manager, mjModel* m, mjData* d)
{
    TSharedPtr<FJsonObject> Scene = MakeShared<FJsonObject>();
    if (!Manager || !m || !d) return Scene;

    // Prefer the cached scene-body record table when populated. Avoids a
    // per-call TActorIterator walk on the physics thread.
    auto BuildFromBody = [&](int32 Id, const FString& Name)
    {
        if (Id < 0 || Id >= m->nbody) return;
        TSharedPtr<FJsonObject> Obj = MakeShared<FJsonObject>();
        TArray<TSharedPtr<FJsonValue>> XPos, XQuat;
        for (int i = 0; i < 3; ++i) XPos.Add(MakeShared<FJsonValueNumber>(d->xpos[Id*3 + i]));
        for (int i = 0; i < 4; ++i) XQuat.Add(MakeShared<FJsonValueNumber>(d->xquat[Id*4 + i]));
        Obj->SetArrayField(TEXT("xpos"),  XPos);
        Obj->SetArrayField(TEXT("xquat"), XQuat);

        // Free-joint detection: a body with a single jntnum=1 of mjJNT_FREE
        // owns a 7-vec qpos and 6-vec qvel. Stream both. Other joint types
        // get xpos/xquat only — a kinematic-driven heightfield base, etc.
        if (Id < m->nbody && m->body_jntnum && m->body_jntadr)
        {
            int FirstJnt = m->body_jntadr[Id];
            int NumJnt   = m->body_jntnum[Id];
            if (FirstJnt >= 0 && NumJnt > 0 && FirstJnt < m->njnt &&
                m->jnt_type[FirstJnt] == mjJNT_FREE)
            {
                int QAddr = m->jnt_qposadr[FirstJnt];
                int VAddr = m->jnt_dofadr[FirstJnt];
                TArray<TSharedPtr<FJsonValue>> QPos, QVel;
                for (int i = 0; i < 7; ++i)
                    QPos.Add(MakeShared<FJsonValueNumber>(d->qpos[QAddr + i]));
                for (int i = 0; i < 6; ++i)
                    QVel.Add(MakeShared<FJsonValueNumber>(d->qvel[VAddr + i]));
                Obj->SetArrayField(TEXT("qpos"), QPos);
                Obj->SetArrayField(TEXT("qvel"), QVel);
            }
        }
        Scene->SetObjectField(Name, Obj);
    };

    // Cache fast path.
    const TArray<FMjEntityRecord>& Cache = Manager->GetEntities();
    if (Cache.Num() > 0)
    {
        for (const FMjEntityRecord& R : Cache)
            BuildFromBody(R.MjId, R.Name);
        return Scene;
    }

    // Fallback: walk the world via TActorIterator. ONLY safe from the game
    // thread -- the iterator asserts IsInGameThread(). DirectStepHandler /
    // PuppetStepHandler run on the physics async thread, so when called from
    // there with an empty cache (no scene bodies were registered), return an
    // empty block rather than crashing. Tests / pre-cache callers on the
    // game thread still use the fallback path.
    if (!IsInGameThread()) return Scene;

    UWorld* World = Manager->GetWorld();
    if (!World) return Scene;

    TSet<AMjArticulation*> ArticSet;
    for (AMjArticulation* A : Manager->GetAllArticulations()) ArticSet.Add(A);

    for (TActorIterator<AActor> It(World); It; ++It)
    {
        AActor* Actor = *It;
        if (!Actor) continue;
        if (AMjArticulation* AsArt = Cast<AMjArticulation>(Actor))
        {
            if (ArticSet.Contains(AsArt)) continue;
        }
        TArray<UMjBody*> Bodies;
        Actor->GetComponents<UMjBody>(Bodies);
        for (UMjBody* B : Bodies)
        {
            if (!B || B->bIsDefault) continue;
            BuildFromBody(B->GetMjID(), B->GetMjName());
        }
    }
    return Scene;
}

// =============================================================================
// reset / set_mode
// =============================================================================

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleReset(const TSharedPtr<FJsonObject>& Req)
{
    AAMjManager* Mgr = OwnerMgr.Get();
    if (!Mgr || !Mgr->PhysicsEngine || !Mgr->PhysicsEngine->m_model)
    {
        return MakeError(TEXT("not_ready"), TEXT("PhysicsEngine not initialised"));
    }

    mjModel* m = Mgr->PhysicsEngine->GetModel();
    mjData*  d = Mgr->PhysicsEngine->GetData();

    int32 SeedVal = 0;
    if (Req->TryGetNumberField(TEXT("seed"), SeedVal))
    {
        Mgr->Seed = SeedVal;
        // Modern mjOption has no "seed" field; mj_step is deterministic and
        // doesn't depend on a stored seed (random elements come from
        // user-set noise inputs, not an integrator-internal RNG). The seed
        // is recorded on the manager so any RNG used by client code or by
        // the recording layer can mirror it for reproducibility. UE itself
        // does not reseed the integrator here.
    }

    {
        FScopeLock Lock(&Mgr->PhysicsEngine->CallbackMutex);

        FString KfName;
        if (Req->TryGetStringField(TEXT("keyframe_name"), KfName) && !KfName.IsEmpty())
        {
            int Kid = mj_name2id(m, mjOBJ_KEY, TCHAR_TO_UTF8(*KfName));
            if (Kid < 0) return MakeError(TEXT("unknown_keyframe"), KfName);
            mj_resetDataKeyframe(m, d, Kid);
        }
        else
        {
            mj_resetData(m, d);
        }

        // Per-articulation qpos overrides (joint-name -> value).
        const TSharedPtr<FJsonObject>* PerArt = nullptr;
        if (Req->TryGetObjectField(TEXT("per_articulation_qpos"), PerArt) && PerArt && PerArt->IsValid())
        {
            for (auto& APair : (*PerArt)->Values)
            {
                AMjArticulation* Art = Mgr->GetArticulation(APair.Key);
                if (!Art) continue;
                const TSharedPtr<FJsonObject>* QObj = nullptr;
                if (!APair.Value->TryGetObject(QObj) || !QObj || !QObj->IsValid()) continue;

                FString Prefix = Art->GetName() + TEXT("_");
                for (auto& JPair : (*QObj)->Values)
                {
                    FString FullName = Prefix + JPair.Key;
                    int Jid = mj_name2id(m, mjOBJ_JOINT, TCHAR_TO_UTF8(*FullName));
                    if (Jid < 0)
                        Jid = mj_name2id(m, mjOBJ_JOINT, TCHAR_TO_UTF8(*JPair.Key));
                    if (Jid < 0) continue;
                    int QAddr = m->jnt_qposadr[Jid];
                    d->qpos[QAddr] = (mjtNum)JPair.Value->AsNumber();
                }
            }
        }
        mj_forward(m, d);
    }

    StepCounter.store(0, std::memory_order_relaxed);

    TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
    Reply->SetStringField(TEXT("op"),   TEXT("reset_ok"));
    Reply->SetNumberField(TEXT("time"), d->time);
    Reply->SetNumberField(TEXT("step"), 0);
    AppendClockFields(Reply, d->time);
    TSharedPtr<FJsonObject> Obs = BuildStepObservations(Mgr, m, d, ActiveObservationLevel);
    if (Obs.IsValid()) Reply->SetObjectField(TEXT("per_articulation"), Obs);
    return Reply;
}

// Run mj_forward (kinematics + dynamics, no integration) and return
// observations. Lets a client write qpos / qvel then read consistent
// derived state (xpos, sensors, contacts, ...) without advancing time.
TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleForward(const TSharedPtr<FJsonObject>& /*Req*/)
{
    AAMjManager* Mgr = OwnerMgr.Get();
    if (!Mgr || !Mgr->PhysicsEngine || !Mgr->PhysicsEngine->m_model)
    {
        return MakeError(TEXT("not_ready"), TEXT("PhysicsEngine not initialised"));
    }

    mjModel* m = Mgr->PhysicsEngine->GetModel();
    mjData*  d = Mgr->PhysicsEngine->GetData();

    {
        FScopeLock Lock(&Mgr->PhysicsEngine->CallbackMutex);
        mj_forward(m, d);
    }

    TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
    Reply->SetStringField(TEXT("op"),   TEXT("forward_ok"));
    Reply->SetNumberField(TEXT("time"), d->time);
    Reply->SetNumberField(TEXT("step"), StepCounter.load(std::memory_order_relaxed));
    AppendClockFields(Reply, d->time);
    TSharedPtr<FJsonObject> Obs = BuildStepObservations(Mgr, m, d, ActiveObservationLevel);
    if (Obs.IsValid()) Reply->SetObjectField(TEXT("per_articulation"), Obs);
    return Reply;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetMode(const TSharedPtr<FJsonObject>& Req)
{
    AAMjManager* Mgr = OwnerMgr.Get();
    if (!Mgr) return MakeError(TEXT("not_ready"), TEXT("Manager missing"));

    if (Mgr->StepMode != EStepMode::Auto)
    {
        return MakeError(TEXT("mode_locked_by_server"),
            FString::Printf(TEXT("Project pinned StepMode to %s"), *StepModeToString(Mgr->StepMode)));
    }

    FString ModeStr;
    if (!Req->TryGetStringField(TEXT("mode"), ModeStr))
        return MakeError(TEXT("missing_field"), TEXT("set_mode requires 'mode'"));

    EStepMode NewMode;
    if (!StepModeFromString(ModeStr, NewMode))
        return MakeError(TEXT("bad_mode"), FString::Printf(TEXT("Unknown mode '%s'"), *ModeStr));

    EStepMode Prev = ActiveStepMode;
    SetActiveStepMode(NewMode);

    TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
    Reply->SetStringField(TEXT("op"),            TEXT("set_mode_ok"));
    Reply->SetStringField(TEXT("previous_mode"), StepModeToString(Prev));
    Reply->SetStringField(TEXT("current_mode"),  StepModeToString(ActiveStepMode));
    return Reply;
}

void FURLabRpcDispatcher::SetActiveStepMode(EStepMode NewMode)
{
    // Serialises install/uninstall side effects against concurrent
    // set_mode calls; Dispatch releases DispatchMutex before handlers.
    FScopeLock Lock(&DispatchMutex);

    const EStepMode CurMode = ActiveStepMode.load(std::memory_order_acquire);
    if (NewMode == CurMode) return;

    AAMjManager* Mgr = OwnerMgr.Get();
    if (!Mgr) return;

    if (CurMode == EStepMode::Puppet) UninstallPuppetHandler();
    if (CurMode == EStepMode::Direct) UninstallDirectHandler();
    DrainQueuesForTest();

    ActiveStepMode.store(NewMode, std::memory_order_release);
    const bool bPaused = (NewMode != EStepMode::Live);
    Mgr->bPublishersPaused.store(bPaused, std::memory_order_release);
    FCameraZmqWorker::bPublishersPaused.store(bPaused, std::memory_order_release);

    if (NewMode == EStepMode::Puppet) InstallPuppetHandler();
    else if (NewMode == EStepMode::Direct) InstallDirectHandler();

    // Engine defaults bIsPaused=true and is normally unpaused via the editor
    // UI / hotkey. A remote client has no UI handle, so entering Direct or
    // Direct/Puppet imply "client drives physics" — force unpause so the
    // async loop calls CustomStepHandler and the request queue drains.
    if (Mgr->PhysicsEngine && NewMode != EStepMode::Live)
    {
        if (Mgr->PhysicsEngine->bIsPaused)
        {
            Mgr->PhysicsEngine->SetPaused(false);
            UE_LOG(LogURLabNet, Log,
                TEXT("FURLabRpcDispatcher: unpaused PhysicsEngine for %s mode"),
                *StepModeToString(NewMode));
        }
    }

    UE_LOG(LogURLabNet, Log, TEXT("FURLabRpcDispatcher: step mode -> %s (publishers_paused=%s)"),
        *StepModeToString(NewMode), bPaused ? TEXT("true") : TEXT("false"));
}

void FURLabRpcDispatcher::InstallPuppetHandler()
{
    AAMjManager* Mgr = OwnerMgr.Get();
    if (!Mgr || !Mgr->PhysicsEngine) return;
    if (bPuppetHandlerInstalled) return;

    UMjPhysicsEngine* Engine = Mgr->PhysicsEngine;
    PuppetStepHandler = [this, Engine](mjModel* m, mjData* d)
    {
        FMjPushStateRequest Req;
        if (!PushStateQueue.Dequeue(Req)) return;
        if (Req.QPos.Num() == m->nq)
            FMemory::Memcpy(d->qpos, Req.QPos.GetData(), m->nq * sizeof(mjtNum));
        if (Req.QVel.Num() == m->nv)
            FMemory::Memcpy(d->qvel, Req.QVel.GetData(), m->nv * sizeof(mjtNum));
        if (Req.bIncludeCtrl && Req.Ctrl.Num() == m->nu)
            FMemory::Memcpy(d->ctrl, Req.Ctrl.GetData(), m->nu * sizeof(mjtNum));
        d->time = Req.Time;
        mj_forward(m, d);
        if (Engine->OnPostStep) Engine->OnPostStep(m, d);
    };
    Engine->SetCustomStepHandler(PuppetStepHandler);
    bPuppetHandlerInstalled = true;
}

void FURLabRpcDispatcher::UninstallPuppetHandler()
{
    if (!bPuppetHandlerInstalled) return;
    if (AAMjManager* Mgr = OwnerMgr.Get())
    {
        if (Mgr->PhysicsEngine) Mgr->PhysicsEngine->ClearCustomStepHandler();
    }
    bPuppetHandlerInstalled = false;
    PuppetStepHandler = nullptr;
}

void FURLabRpcDispatcher::InstallDirectHandler()
{
    AAMjManager* Mgr = OwnerMgr.Get();
    if (!Mgr || !Mgr->PhysicsEngine) return;
    if (bDirectHandlerInstalled) return;

    UMjPhysicsEngine* Engine = Mgr->PhysicsEngine;
    DirectStepHandler = [this, Engine, Mgr](mjModel* m, mjData* d)
    {
        // Only the physics-engine async worker thread runs this handler,
        // so d is exclusively owned for its duration.
        TSharedPtr<FMjDirectStepCommand> Cmd;
        if (!StepQueue.Dequeue(Cmd) || !Cmd.IsValid()) return;

        ApplyStepCtrl(Mgr, Cmd->Request, m, d);

        // control_mode="raw" per articulation bypasses the UE controller
        // (NetworkValue treated as direct ctrl setpoint). Name-keyed so
        // adding/removing articulations doesn't shift the mapping.
        TMap<AMjArticulation*, bool> SkipController;
        for (AMjArticulation* Art : Mgr->GetAllArticulations())
        {
            if (!Art) continue;
            const FString* Mode = Cmd->Request.PerArticulationControlMode.Find(Art->GetName());
            const bool bRaw = Mode && Mode->Equals(TEXT("raw"), ESearchCase::IgnoreCase);
            SkipController.Add(Art, bRaw);
        }

        for (int32 i = 0; i < Cmd->Request.NSteps; ++i)
        {
            for (AMjArticulation* Art : Mgr->GetAllArticulations())
            {
                if (!Art) continue;
                const bool* bSkip = SkipController.Find(Art);
                Art->ApplyControls(bSkip != nullptr && *bSkip);
            }
            mj_step(m, d);
            if (Engine->OnPostStep) Engine->OnPostStep(m, d);
        }
        Cmd->ResultTime = d->time;
        Cmd->ResultStep = StepCounter.fetch_add(Cmd->Request.NSteps, std::memory_order_relaxed)
                          + Cmd->Request.NSteps;
        Cmd->Observations = BuildStepObservations(Mgr, m, d, ActiveObservationLevel);
        Cmd->Entities  = BuildEntitiesBlock(Mgr, m, d);
        Cmd->bDone = true;
        if (Cmd->Completion) Cmd->Completion->Trigger();
    };
    Engine->SetCustomStepHandler(DirectStepHandler);
    bDirectHandlerInstalled = true;
}

void FURLabRpcDispatcher::UninstallDirectHandler()
{
    if (!bDirectHandlerInstalled) return;
    if (AAMjManager* Mgr = OwnerMgr.Get())
    {
        if (Mgr->PhysicsEngine) Mgr->PhysicsEngine->ClearCustomStepHandler();
    }
    bDirectHandlerInstalled = false;
    DirectStepHandler = nullptr;
}

// =============================================================================
// configure_controller
// =============================================================================

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleConfigureController(const TSharedPtr<FJsonObject>& Req)
{
    AAMjManager* Mgr = OwnerMgr.Get();
    if (!Mgr) return MakeError(TEXT("not_ready"), TEXT("Manager missing"));

    FString ArtName;
    if (!Req->TryGetStringField(TEXT("articulation"), ArtName))
        return MakeError(TEXT("missing_field"), TEXT("configure_controller requires 'articulation'"));

    AMjArticulation* Art = Mgr->GetArticulation(ArtName);
    if (!Art)
        return MakeError(TEXT("unknown_articulation"), ArtName);

    UMjArticulationController* Ctrl = Art->FindComponentByClass<UMjArticulationController>();
    if (!Ctrl)
        return MakeError(TEXT("no_controller"), FString::Printf(TEXT("Articulation '%s' has no controller"), *ArtName));

    const TSharedPtr<FJsonObject>* Params = nullptr;
    if (Req->TryGetObjectField(TEXT("params"), Params) && Params && Params->IsValid())
    {
        Ctrl->ApplyConfig(*Params);
    }

    TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
    Reply->SetStringField(TEXT("op"),           TEXT("configure_controller_ok"));
    Reply->SetStringField(TEXT("articulation"), ArtName);

    TSharedPtr<FJsonObject> Out = MakeShared<FJsonObject>();
    Ctrl->GetCurrentConfig(Out);
    Reply->SetObjectField(TEXT("params"), Out);
    return Reply;
}

// =============================================================================
// set_sim_options
// =============================================================================

namespace
{
    bool ParseIntegrator(const FString& S, EMjIntegrator& Out)
    {
        if (S.Equals(TEXT("euler"),        ESearchCase::IgnoreCase)) { Out = EMjIntegrator::Euler; return true; }
        if (S.Equals(TEXT("rk4"),          ESearchCase::IgnoreCase)) { Out = EMjIntegrator::RK4; return true; }
        if (S.Equals(TEXT("implicit"),     ESearchCase::IgnoreCase)) { Out = EMjIntegrator::Implicit; return true; }
        if (S.Equals(TEXT("implicitfast"), ESearchCase::IgnoreCase)) { Out = EMjIntegrator::ImplicitFast; return true; }
        return false;
    }
    FString IntegratorToString(EMjIntegrator I)
    {
        switch (I)
        {
        case EMjIntegrator::Euler:        return TEXT("euler");
        case EMjIntegrator::RK4:          return TEXT("rk4");
        case EMjIntegrator::Implicit:     return TEXT("implicit");
        case EMjIntegrator::ImplicitFast: return TEXT("implicitfast");
        }
        return TEXT("euler");
    }
    bool ParseCone(const FString& S, EMjCone& Out)
    {
        if (S.Equals(TEXT("pyramidal"), ESearchCase::IgnoreCase)) { Out = EMjCone::Pyramidal; return true; }
        if (S.Equals(TEXT("elliptic"),  ESearchCase::IgnoreCase)) { Out = EMjCone::Elliptic;  return true; }
        return false;
    }
    FString ConeToString(EMjCone C)
    {
        return C == EMjCone::Elliptic ? TEXT("elliptic") : TEXT("pyramidal");
    }
    bool ParseSolver(const FString& S, EMjSolver& Out)
    {
        if (S.Equals(TEXT("pgs"),    ESearchCase::IgnoreCase)) { Out = EMjSolver::PGS;    return true; }
        if (S.Equals(TEXT("cg"),     ESearchCase::IgnoreCase)) { Out = EMjSolver::CG;     return true; }
        if (S.Equals(TEXT("newton"), ESearchCase::IgnoreCase)) { Out = EMjSolver::Newton; return true; }
        return false;
    }
    FString SolverToString(EMjSolver S)
    {
        switch (S)
        {
        case EMjSolver::PGS:    return TEXT("pgs");
        case EMjSolver::CG:     return TEXT("cg");
        case EMjSolver::Newton: return TEXT("newton");
        }
        return TEXT("newton");
    }

    bool TryReadVec3(const TSharedPtr<FJsonObject>& Obj, const TCHAR* Key, double Out[3])
    {
        const TArray<TSharedPtr<FJsonValue>>* Arr = nullptr;
        if (!Obj->TryGetArrayField(Key, Arr) || !Arr || Arr->Num() != 3) return false;
        Out[0] = (*Arr)[0]->AsNumber();
        Out[1] = (*Arr)[1]->AsNumber();
        Out[2] = (*Arr)[2]->AsNumber();
        return true;
    }
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetSimOptions(const TSharedPtr<FJsonObject>& Req)
{
    AAMjManager* Mgr = OwnerMgr.Get();
    if (!Mgr || !Mgr->PhysicsEngine)
        return MakeError(TEXT("not_ready"), TEXT("PhysicsEngine not initialised"));

    mjModel* m = Mgr->PhysicsEngine->GetModel();
    if (!m) return MakeError(TEXT("not_ready"), TEXT("mjModel not compiled"));

    const TSharedPtr<FJsonObject>* OptsPtr = nullptr;
    if (!Req->TryGetObjectField(TEXT("options"), OptsPtr) || !OptsPtr || !(*OptsPtr).IsValid())
        return MakeError(TEXT("missing_field"), TEXT("set_sim_options requires 'options' object"));
    const TSharedPtr<FJsonObject>& Opts = *OptsPtr;

    FMjOptionGenerated& O = Mgr->PhysicsEngine->Options;

    double DNum = 0.0;
    if (Opts->TryGetNumberField(TEXT("timestep"), DNum))
    {
        O.Timestep = (float)DNum;
        O.bOverride_Timestep = true;
    }

    // Wire is MJ-native SI; FMjOptionGenerated stores UE cm/s² with Y-flip and
    // ApplyOverridesToModel reverses that, so pre-bake the inverse here.
    double V3[3];
    if (TryReadVec3(Opts, TEXT("gravity"), V3))
    {
        O.Gravity = FVector((float)(V3[0] * 100.0), (float)(-V3[1] * 100.0), (float)(V3[2] * 100.0));
        O.bOverride_Gravity = true;
    }
    if (TryReadVec3(Opts, TEXT("wind"), V3))
    {
        O.Wind = FVector((float)(V3[0] * 100.0), (float)(-V3[1] * 100.0), (float)(V3[2] * 100.0));
        O.bOverride_Wind = true;
    }
    if (TryReadVec3(Opts, TEXT("magnetic"), V3))
    {
        O.Magnetic = FVector((float)V3[0], (float)-V3[1], (float)V3[2]);
        O.bOverride_Magnetic = true;
    }

    if (Opts->TryGetNumberField(TEXT("density"),   DNum)) { O.Density   = (float)DNum; O.bOverride_Density   = true; }
    if (Opts->TryGetNumberField(TEXT("viscosity"), DNum)) { O.Viscosity = (float)DNum; O.bOverride_Viscosity = true; }
    if (Opts->TryGetNumberField(TEXT("impratio"),  DNum)) { O.Impratio  = (float)DNum; O.bOverride_Impratio  = true; }
    if (Opts->TryGetNumberField(TEXT("tolerance"), DNum)) { O.Tolerance = (float)DNum; O.bOverride_Tolerance = true; }

    int32 INum = 0;
    if (Opts->TryGetNumberField(TEXT("iterations"),    INum)) { O.Iterations    = INum; O.bOverride_Iterations    = true; }
    if (Opts->TryGetNumberField(TEXT("ls_iterations"), INum)) { O.LsIterations  = INum; O.bOverride_LsIterations  = true; }

    FString SNum;
    if (Opts->TryGetStringField(TEXT("integrator"), SNum))
    {
        EMjIntegrator E;
        if (!ParseIntegrator(SNum, E)) return MakeError(TEXT("bad_value"), FString::Printf(TEXT("unknown integrator '%s'"), *SNum));
        O.Integrator = E; O.bOverride_Integrator = true;
    }
    if (Opts->TryGetStringField(TEXT("cone"), SNum))
    {
        EMjCone E;
        if (!ParseCone(SNum, E)) return MakeError(TEXT("bad_value"), FString::Printf(TEXT("unknown cone '%s'"), *SNum));
        O.Cone = E; O.bOverride_Cone = true;
    }
    if (Opts->TryGetStringField(TEXT("solver"), SNum))
    {
        EMjSolver E;
        if (!ParseSolver(SNum, E)) return MakeError(TEXT("bad_value"), FString::Printf(TEXT("unknown solver '%s'"), *SNum));
        O.Solver = E; O.bOverride_Solver = true;
    }

    if (Opts->TryGetNumberField(TEXT("noslip_iterations"), INum)) { O.NoslipIterations = INum;       O.bOverride_NoslipIterations = true; }
    if (Opts->TryGetNumberField(TEXT("noslip_tolerance"),  DNum)) { O.NoslipTolerance  = (float)DNum; O.bOverride_NoslipTolerance  = true; }
    if (Opts->TryGetNumberField(TEXT("ccd_iterations"),    INum)) { O.CCD_Iterations   = INum;       O.bOverride_CCD_Iterations   = true; }
    if (Opts->TryGetNumberField(TEXT("ccd_tolerance"),     DNum)) { O.CCD_Tolerance    = (float)DNum; O.bOverride_CCD_Tolerance    = true; }

    bool BNum = false;
    if (Opts->TryGetBoolField(TEXT("enable_multiccd"), BNum)) { O.bEnableMultiCCD = BNum; }
    if (Opts->TryGetBoolField(TEXT("enable_sleep"),    BNum)) { O.bEnableSleep    = BNum; }
    if (Opts->TryGetNumberField(TEXT("sleep_tolerance"), DNum)) { O.SleepTolerance = (float)DNum; }

    // Raw disable / enable bit masks. Values are bitwise-ORs of
    // mujoco/mjmodel.h mjtDisableBit / mjtEnableBit constants.
    // Applied BEFORE FMjOptionGenerated::ApplyOverridesToModel so any named
    // bits the caller also set (enable_sleep / enable_multiccd) win on
    // top of the raw mask. Treat the raw masks as a coarse baseline.
    int32 DisableMask = 0;
    if (Opts->TryGetNumberField(TEXT("disableflags"), DisableMask)) { m->opt.disableflags = DisableMask; }
    int32 EnableMask = 0;
    if (Opts->TryGetNumberField(TEXT("enableflags"),  EnableMask))  { m->opt.enableflags  = EnableMask; }

    O.ApplyOverridesToModel(m);

    UE_LOG(LogURLabNet, Log,
        TEXT("FURLabRpcDispatcher: set_sim_options applied (timestep=%.5fs, gravity=[%.3f %.3f %.3f] m/s²)"),
        m->opt.timestep, m->opt.gravity[0], m->opt.gravity[1], m->opt.gravity[2]);

    TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
    Reply->SetStringField(TEXT("op"), TEXT("set_sim_options_ok"));

    TSharedPtr<FJsonObject> Out = MakeShared<FJsonObject>();
    Out->SetNumberField(TEXT("timestep"), m->opt.timestep);
    {
        TArray<TSharedPtr<FJsonValue>> G;
        G.Add(MakeShared<FJsonValueNumber>(m->opt.gravity[0]));
        G.Add(MakeShared<FJsonValueNumber>(m->opt.gravity[1]));
        G.Add(MakeShared<FJsonValueNumber>(m->opt.gravity[2]));
        Out->SetArrayField(TEXT("gravity"), G);
        TArray<TSharedPtr<FJsonValue>> W;
        W.Add(MakeShared<FJsonValueNumber>(m->opt.wind[0]));
        W.Add(MakeShared<FJsonValueNumber>(m->opt.wind[1]));
        W.Add(MakeShared<FJsonValueNumber>(m->opt.wind[2]));
        Out->SetArrayField(TEXT("wind"), W);
        TArray<TSharedPtr<FJsonValue>> Mg;
        Mg.Add(MakeShared<FJsonValueNumber>(m->opt.magnetic[0]));
        Mg.Add(MakeShared<FJsonValueNumber>(m->opt.magnetic[1]));
        Mg.Add(MakeShared<FJsonValueNumber>(m->opt.magnetic[2]));
        Out->SetArrayField(TEXT("magnetic"), Mg);
    }
    Out->SetNumberField(TEXT("density"),       m->opt.density);
    Out->SetNumberField(TEXT("viscosity"),     m->opt.viscosity);
    Out->SetNumberField(TEXT("impratio"),      m->opt.impratio);
    Out->SetNumberField(TEXT("tolerance"),     m->opt.tolerance);
    Out->SetNumberField(TEXT("iterations"),    m->opt.iterations);
    Out->SetNumberField(TEXT("ls_iterations"), m->opt.ls_iterations);
    Out->SetStringField(TEXT("integrator"), IntegratorToString((EMjIntegrator)m->opt.integrator));
    Out->SetStringField(TEXT("cone"),       ConeToString((EMjCone)m->opt.cone));
    Out->SetStringField(TEXT("solver"),     SolverToString((EMjSolver)m->opt.solver));
    Out->SetNumberField(TEXT("noslip_iterations"), m->opt.noslip_iterations);
    Out->SetNumberField(TEXT("noslip_tolerance"),  m->opt.noslip_tolerance);
    Out->SetNumberField(TEXT("ccd_iterations"),    m->opt.ccd_iterations);
    Out->SetNumberField(TEXT("ccd_tolerance"),     m->opt.ccd_tolerance);

    constexpr int MJ_ENBL_MULTICCD = 1 << 4;
    constexpr int MJ_ENBL_SLEEP    = 1 << 5;
    Out->SetBoolField(TEXT("enable_multiccd"), (m->opt.enableflags & MJ_ENBL_MULTICCD) != 0);
    Out->SetBoolField(TEXT("enable_sleep"),    (m->opt.enableflags & MJ_ENBL_SLEEP) != 0);
    Out->SetNumberField(TEXT("sleep_tolerance"), m->opt.sleep_tolerance);

    // Echo the raw bit masks so callers using disableflags / enableflags
    // can verify the final composed state (named overrides + raw mask).
    Out->SetNumberField(TEXT("disableflags"), (int32)m->opt.disableflags);
    Out->SetNumberField(TEXT("enableflags"),  (int32)m->opt.enableflags);

    Reply->SetObjectField(TEXT("options"), Out);
    return Reply;
}

// =============================================================================
// set_sim_speed
// =============================================================================

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetSimSpeed(const TSharedPtr<FJsonObject>& Req)
{
    AAMjManager* Mgr = OwnerMgr.Get();
    if (!Mgr || !Mgr->PhysicsEngine)
        return MakeError(TEXT("not_ready"), TEXT("PhysicsEngine not initialised"));

    double Pct = 0.0;
    if (!Req->TryGetNumberField(TEXT("percent"), Pct))
        return MakeError(TEXT("missing_field"), TEXT("set_sim_speed requires 'percent'"));

    // Engine clamps internally (5..100); echo back so the caller sees what stuck.
    Mgr->PhysicsEngine->SimSpeedPercent = (float)Pct;
    const float Effective = FMath::Clamp((float)Pct, 5.0f, 100.0f);

    TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
    Reply->SetStringField(TEXT("op"), TEXT("set_sim_speed_ok"));
    Reply->SetNumberField(TEXT("percent"), Effective);
    return Reply;
}

// =============================================================================
// set_control_source
// =============================================================================

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetControlSource(const TSharedPtr<FJsonObject>& Req)
{
    AAMjManager* Mgr = OwnerMgr.Get();
    if (!Mgr || !Mgr->PhysicsEngine)
        return MakeError(TEXT("not_ready"), TEXT("PhysicsEngine not initialised"));

    FString SourceStr;
    if (!Req->TryGetStringField(TEXT("source"), SourceStr))
        return MakeError(TEXT("missing_field"), TEXT("set_control_source requires 'source' (\"zmq\" | \"ui\")"));

    EControlSource NewSource;
    if (SourceStr.Equals(TEXT("zmq"), ESearchCase::IgnoreCase))      NewSource = EControlSource::ZMQ;
    else if (SourceStr.Equals(TEXT("ui"), ESearchCase::IgnoreCase))  NewSource = EControlSource::UI;
    else return MakeError(TEXT("bad_value"), FString::Printf(TEXT("unknown source '%s'"), *SourceStr));

    FString ArtName;
    Req->TryGetStringField(TEXT("articulation"), ArtName);

    TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
    Reply->SetStringField(TEXT("op"), TEXT("set_control_source_ok"));
    Reply->SetStringField(TEXT("source"), SourceStr.ToLower());

    if (ArtName.IsEmpty())
    {
        // Global: update engine + every articulation so the per-actor field
        // doesn't keep stale state after a global flip.
        Mgr->PhysicsEngine->SetControlSource(NewSource);
        for (AMjArticulation* Art : Mgr->GetAllArticulations())
        {
            if (Art) Art->ControlSource = (uint8)NewSource;
        }
        Reply->SetStringField(TEXT("scope"), TEXT("global"));
    }
    else
    {
        AMjArticulation* Art = Mgr->GetArticulation(ArtName);
        if (!Art) return MakeError(TEXT("unknown_articulation"), ArtName);
        Art->ControlSource = (uint8)NewSource;
        Reply->SetStringField(TEXT("scope"), TEXT("articulation"));
        Reply->SetStringField(TEXT("articulation"), ArtName);
    }
    return Reply;
}

// =============================================================================
// set_twist
// =============================================================================

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetTwist(const TSharedPtr<FJsonObject>& Req)
{
    AAMjManager* Mgr = OwnerMgr.Get();
    if (!Mgr) return MakeError(TEXT("not_ready"), TEXT("Manager missing"));

    FString ArtName;
    if (!Req->TryGetStringField(TEXT("articulation"), ArtName))
        return MakeError(TEXT("missing_field"), TEXT("set_twist requires 'articulation'"));

    AMjArticulation* Art = Mgr->GetArticulation(ArtName);
    if (!Art) return MakeError(TEXT("unknown_articulation"), ArtName);

    UMjTwistController* TC = Art->FindComponentByClass<UMjTwistController>();
    if (!TC) return MakeError(TEXT("no_twist_controller"),
        FString::Printf(TEXT("Articulation '%s' has no UMjTwistController"), *ArtName));

    // Wire format mirrors how the bridge already reads twist: linear is
    // (vx, vy, _) m/s, angular is (_, _, yaw_rate) rad/s. Tuple slots
    // beyond the ones used are accepted but ignored.
    auto ReadAxis = [](const TArray<TSharedPtr<FJsonValue>>* Arr, int32 Idx, float& Out)
    {
        if (Arr && Arr->IsValidIndex(Idx)) Out = (float)(*Arr)[Idx]->AsNumber();
    };

    float Vx = 0.f, Vy = 0.f, YawRate = 0.f;
    const TArray<TSharedPtr<FJsonValue>>* LinArr = nullptr;
    const TArray<TSharedPtr<FJsonValue>>* AngArr = nullptr;
    Req->TryGetArrayField(TEXT("linear"),  LinArr);
    Req->TryGetArrayField(TEXT("angular"), AngArr);
    ReadAxis(LinArr, 0, Vx);
    ReadAxis(LinArr, 1, Vy);
    ReadAxis(AngArr, 2, YawRate);

    TC->SetTwist(Vx, Vy, YawRate);

    TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
    Reply->SetStringField(TEXT("op"), TEXT("set_twist_ok"));
    Reply->SetStringField(TEXT("articulation"), ArtName);
    {
        TArray<TSharedPtr<FJsonValue>> L;
        L.Add(MakeShared<FJsonValueNumber>(Vx));
        L.Add(MakeShared<FJsonValueNumber>(Vy));
        L.Add(MakeShared<FJsonValueNumber>(0.0));
        Reply->SetArrayField(TEXT("linear"), L);
        TArray<TSharedPtr<FJsonValue>> A;
        A.Add(MakeShared<FJsonValueNumber>(0.0));
        A.Add(MakeShared<FJsonValueNumber>(0.0));
        A.Add(MakeShared<FJsonValueNumber>(YawRate));
        Reply->SetArrayField(TEXT("angular"), A);
    }
    return Reply;
}

// =============================================================================
// set_qpos — manager-required runtime write to a single articulation's qpos.
//
// Two write modes:
//   - Free-base 7-vec shortcut: len=7 and the first joint is mjJNT_FREE,
//     writes only the 7 free-joint slots (xyz + quat). Skips dof joints.
//   - Full per-articulation qpos: len matches the articulation's total qpos
//     dim (sum of per-joint slot widths in GetJoints() order). Writes the
//     whole slice.
// Always calls mj_forward after the write, mirroring the puppet push-state
// path so derived quantities (xpos, sensors) reflect the new state.
// =============================================================================
TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetQpos(const TSharedPtr<FJsonObject>& Req)
{
    AAMjManager* Mgr = OwnerMgr.Get();
    if (!Mgr || !Mgr->PhysicsEngine || !Mgr->PhysicsEngine->IsInitialized())
        return MakeError(TEXT("not_ready"), TEXT("Manager not initialised"));

    mjModel* m = Mgr->PhysicsEngine->GetModel();
    mjData*  d = Mgr->PhysicsEngine->GetData();
    if (!m || !d) return MakeError(TEXT("not_ready"), TEXT("MjModel/MjData missing"));

    // target/target_by wire shape. target_by="actor_name" looks up via
    // the manager's GetArticulation (UE name match); default
    // "actor_id" walks ActorId.
    FString Target, By;
    Req->TryGetStringField(TEXT("target"),    Target);
    Req->TryGetStringField(TEXT("target_by"), By);
    if (Target.IsEmpty())
    {
        return MakeError(TEXT("missing_field"),
            TEXT("set_qpos: missing 'target' field"));
    }
    const bool bByName = By.Equals(TEXT("actor_name"), ESearchCase::IgnoreCase);

    AMjArticulation* Art = nullptr;
    if (bByName)
    {
        Art = Mgr->GetArticulation(Target);
    }
    else
    {
        for (AMjArticulation* A : Mgr->GetAllArticulations())
        {
            if (A && A->ActorId.Equals(Target)) { Art = A; break; }
        }
    }
    if (!Art)
    {
        return MakeError(TEXT("unknown_articulation"), Target);
    }

    const TArray<TSharedPtr<FJsonValue>>* QPosArr = nullptr;
    if (!Req->TryGetArrayField(TEXT("qpos"), QPosArr) || !QPosArr)
        return MakeError(TEXT("missing_field"), TEXT("set_qpos requires 'qpos' array"));

    struct FJointSlot { int32 Adr; int32 Size; int32 Type; };
    TArray<FJointSlot> Slots;
    int32 ArtQDim = 0;
    for (UMjJoint* J : Art->GetJoints())
    {
        if (!J) continue;
        int32 Id = J->GetMjID();
        if (Id < 0 || Id >= m->njnt) continue;
        int32 Size = 1;
        switch (m->jnt_type[Id])
        {
        case mjJNT_FREE:  Size = 7; break;
        case mjJNT_BALL:  Size = 4; break;
        case mjJNT_SLIDE:
        case mjJNT_HINGE: Size = 1; break;
        }
        Slots.Add({ m->jnt_qposadr[Id], Size, m->jnt_type[Id] });
        ArtQDim += Size;
    }

    if (Slots.Num() == 0)
        return MakeError(TEXT("no_joints"),
            TEXT("Articulation has no joints; nothing to write"));

    const int32 InN = QPosArr->Num();
    bool bFreeBaseShortcut = false;
    if (InN == 7 && Slots[0].Type == mjJNT_FREE && ArtQDim != 7)
        bFreeBaseShortcut = true;
    else if (InN != ArtQDim)
        return MakeError(TEXT("dim_mismatch"),
            FString::Printf(
                TEXT("qpos length %d != articulation qpos dim %d (free-base shortcut requires len=7 with FREE root)"),
                InN, ArtQDim));

    {
        FScopeLock Lock(&Mgr->PhysicsEngine->CallbackMutex);
        if (bFreeBaseShortcut)
        {
            const int32 Adr = Slots[0].Adr;
            for (int32 i = 0; i < 7; ++i)
                d->qpos[Adr + i] = (mjtNum)(*QPosArr)[i]->AsNumber();
        }
        else
        {
            int32 Cursor = 0;
            for (const FJointSlot& S : Slots)
            {
                for (int32 i = 0; i < S.Size; ++i, ++Cursor)
                    d->qpos[S.Adr + i] = (mjtNum)(*QPosArr)[Cursor]->AsNumber();
            }
        }
        mj_forward(m, d);
    }

    TArray<TSharedPtr<FJsonValue>> Out;
    if (bFreeBaseShortcut)
    {
        const int32 Adr = Slots[0].Adr;
        for (int32 i = 0; i < 7; ++i)
            Out.Add(MakeShared<FJsonValueNumber>(d->qpos[Adr + i]));
    }
    else
    {
        for (const FJointSlot& S : Slots)
            for (int32 i = 0; i < S.Size; ++i)
                Out.Add(MakeShared<FJsonValueNumber>(d->qpos[S.Adr + i]));
    }

    TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
    Reply->SetStringField(TEXT("op"),                TEXT("set_qpos_ok"));
    // Echo back the resolved actor identifiers so the caller can
    // confirm which articulation actually got the write. `target`
    // matches the request's target field; `actor_name` is the UE
    // name (always present, even if actor_id was the lookup key).
    Reply->SetStringField(TEXT("target"),            Target);
    Reply->SetStringField(TEXT("actor_name"),        Art->GetName());
    if (!Art->ActorId.IsEmpty())
        Reply->SetStringField(TEXT("actor_id"),      Art->ActorId);
    Reply->SetArrayField(TEXT("qpos"),               Out);
    Reply->SetBoolField(TEXT("free_base_shortcut"),  bFreeBaseShortcut);
    return Reply;
}

// =============================================================================
// set_mocap_pose / read_mocap_pose / get_contacts — runtime MJ-side reads/writes.
//
// All three operate directly on the live mjModel/mjData under the engine's
// CallbackMutex (same as set_qpos). Body name lookup uses mj_name2id with
// the full compiled MJ name (URLab prefixes are already included).
// =============================================================================

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetMocapPose(const TSharedPtr<FJsonObject>& Req)
{
    AAMjManager* Mgr = OwnerMgr.Get();
    if (!Mgr || !Mgr->PhysicsEngine || !Mgr->PhysicsEngine->IsInitialized())
        return MakeError(TEXT("not_ready"), TEXT("Manager not initialised"));

    mjModel* m = Mgr->PhysicsEngine->GetModel();
    mjData*  d = Mgr->PhysicsEngine->GetData();
    if (!m || !d) return MakeError(TEXT("not_ready"), TEXT("MjModel/MjData missing"));

    FString Body;
    Req->TryGetStringField(TEXT("body"), Body);
    if (Body.IsEmpty())
        return MakeError(TEXT("missing_field"), TEXT("set_mocap_pose: missing 'body'"));

    const int32 BodyId = mj_name2id(m, mjOBJ_BODY, TCHAR_TO_UTF8(*Body));
    if (BodyId < 0)
        return MakeError(TEXT("unknown_body"), Body);

    const int32 MocapId = m->body_mocapid[BodyId];
    if (MocapId < 0)
        return MakeError(TEXT("not_mocap_body"),
            FString::Printf(TEXT("Body '%s' is not a mocap body"), *Body));

    const TArray<TSharedPtr<FJsonValue>>* PosArr = nullptr;
    const TArray<TSharedPtr<FJsonValue>>* QuatArr = nullptr;
    const bool bHasPos  = Req->TryGetArrayField(TEXT("pos"),  PosArr)  && PosArr  && PosArr->Num()  == 3;
    const bool bHasQuat = Req->TryGetArrayField(TEXT("quat"), QuatArr) && QuatArr && QuatArr->Num() == 4;
    if (!bHasPos && !bHasQuat)
        return MakeError(TEXT("missing_field"),
            TEXT("set_mocap_pose requires at least one of pos[3] or quat[4]"));

    {
        FScopeLock Lock(&Mgr->PhysicsEngine->CallbackMutex);
        if (bHasPos)
        {
            for (int32 i = 0; i < 3; ++i)
                d->mocap_pos[3 * MocapId + i] = (mjtNum)(*PosArr)[i]->AsNumber();
        }
        if (bHasQuat)
        {
            for (int32 i = 0; i < 4; ++i)
                d->mocap_quat[4 * MocapId + i] = (mjtNum)(*QuatArr)[i]->AsNumber();
        }
    }

    TArray<TSharedPtr<FJsonValue>> PosOut, QuatOut;
    for (int32 i = 0; i < 3; ++i) PosOut.Add(MakeShared<FJsonValueNumber>(d->mocap_pos [3 * MocapId + i]));
    for (int32 i = 0; i < 4; ++i) QuatOut.Add(MakeShared<FJsonValueNumber>(d->mocap_quat[4 * MocapId + i]));

    TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
    Reply->SetStringField(TEXT("op"),   TEXT("set_mocap_pose_ok"));
    Reply->SetStringField(TEXT("body"), Body);
    Reply->SetArrayField (TEXT("pos"),  PosOut);
    Reply->SetArrayField (TEXT("quat"), QuatOut);
    return Reply;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleReadMocapPose(const TSharedPtr<FJsonObject>& Req)
{
    AAMjManager* Mgr = OwnerMgr.Get();
    if (!Mgr || !Mgr->PhysicsEngine || !Mgr->PhysicsEngine->IsInitialized())
        return MakeError(TEXT("not_ready"), TEXT("Manager not initialised"));

    mjModel* m = Mgr->PhysicsEngine->GetModel();
    mjData*  d = Mgr->PhysicsEngine->GetData();
    if (!m || !d) return MakeError(TEXT("not_ready"), TEXT("MjModel/MjData missing"));

    FString Body;
    Req->TryGetStringField(TEXT("body"), Body);
    if (Body.IsEmpty())
        return MakeError(TEXT("missing_field"), TEXT("read_mocap_pose: missing 'body'"));

    const int32 BodyId = mj_name2id(m, mjOBJ_BODY, TCHAR_TO_UTF8(*Body));
    if (BodyId < 0)
        return MakeError(TEXT("unknown_body"), Body);

    const int32 MocapId = m->body_mocapid[BodyId];
    if (MocapId < 0)
        return MakeError(TEXT("not_mocap_body"),
            FString::Printf(TEXT("Body '%s' is not a mocap body"), *Body));

    TArray<TSharedPtr<FJsonValue>> PosOut, QuatOut;
    {
        FScopeLock Lock(&Mgr->PhysicsEngine->CallbackMutex);
        for (int32 i = 0; i < 3; ++i) PosOut.Add(MakeShared<FJsonValueNumber>(d->mocap_pos [3 * MocapId + i]));
        for (int32 i = 0; i < 4; ++i) QuatOut.Add(MakeShared<FJsonValueNumber>(d->mocap_quat[4 * MocapId + i]));
    }

    TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
    Reply->SetStringField(TEXT("op"),   TEXT("read_mocap_pose_ok"));
    Reply->SetStringField(TEXT("body"), Body);
    Reply->SetArrayField (TEXT("pos"),  PosOut);
    Reply->SetArrayField (TEXT("quat"), QuatOut);
    return Reply;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleGetContacts(const TSharedPtr<FJsonObject>& Req)
{
    AAMjManager* Mgr = OwnerMgr.Get();
    if (!Mgr || !Mgr->PhysicsEngine || !Mgr->PhysicsEngine->IsInitialized())
        return MakeError(TEXT("not_ready"), TEXT("Manager not initialised"));

    mjModel* m = Mgr->PhysicsEngine->GetModel();
    mjData*  d = Mgr->PhysicsEngine->GetData();
    if (!m || !d) return MakeError(TEXT("not_ready"), TEXT("MjModel/MjData missing"));

    int32 MaxContacts = 64;
    {
        int32 Cap = 0;
        if (Req->TryGetNumberField(TEXT("max_contacts"), Cap) && Cap > 0)
            MaxContacts = Cap;
    }

    // Optional filter: {body1?, body2?, geom1?, geom2?}. AND across set fields.
    FString FBody1, FBody2, FGeom1, FGeom2;
    const TSharedPtr<FJsonObject>* FilterObj = nullptr;
    if (Req->TryGetObjectField(TEXT("filter"), FilterObj) && FilterObj && *FilterObj)
    {
        (*FilterObj)->TryGetStringField(TEXT("body1"), FBody1);
        (*FilterObj)->TryGetStringField(TEXT("body2"), FBody2);
        (*FilterObj)->TryGetStringField(TEXT("geom1"), FGeom1);
        (*FilterObj)->TryGetStringField(TEXT("geom2"), FGeom2);
    }

    auto NameOrEmpty = [&](int Type, int Id) -> FString
    {
        if (Id < 0) return FString();
        const char* p = mj_id2name(m, Type, Id);
        return p ? FString(UTF8_TO_TCHAR(p)) : FString();
    };

    TArray<TSharedPtr<FJsonValue>> Out;
    bool bTruncated = false;
    int32 Matched = 0;

    {
        FScopeLock Lock(&Mgr->PhysicsEngine->CallbackMutex);
        const int32 N = d->ncon;
        for (int32 i = 0; i < N; ++i)
        {
            const mjContact& c = d->contact[i];
            const int32 G1 = c.geom[0];
            const int32 G2 = c.geom[1];
            const int32 B1 = (G1 >= 0 && G1 < m->ngeom) ? m->geom_bodyid[G1] : -1;
            const int32 B2 = (G2 >= 0 && G2 < m->ngeom) ? m->geom_bodyid[G2] : -1;
            const FString G1Name = NameOrEmpty(mjOBJ_GEOM, G1);
            const FString G2Name = NameOrEmpty(mjOBJ_GEOM, G2);
            const FString B1Name = NameOrEmpty(mjOBJ_BODY, B1);
            const FString B2Name = NameOrEmpty(mjOBJ_BODY, B2);

            if (!FGeom1.IsEmpty() && !G1Name.Equals(FGeom1)) continue;
            if (!FGeom2.IsEmpty() && !G2Name.Equals(FGeom2)) continue;
            if (!FBody1.IsEmpty() && !B1Name.Equals(FBody1)) continue;
            if (!FBody2.IsEmpty() && !B2Name.Equals(FBody2)) continue;

            if (Matched >= MaxContacts)
            {
                bTruncated = true;
                break;
            }

            mjtNum Force[6] = {0};
            mj_contactForce(m, d, i, Force);

            TArray<TSharedPtr<FJsonValue>> Pos;
            for (int32 k = 0; k < 3; ++k) Pos.Add(MakeShared<FJsonValueNumber>(c.pos[k]));
            // First row of the contact frame is the contact normal.
            TArray<TSharedPtr<FJsonValue>> Normal;
            for (int32 k = 0; k < 3; ++k) Normal.Add(MakeShared<FJsonValueNumber>(c.frame[k]));
            TArray<TSharedPtr<FJsonValue>> ForceArr;
            for (int32 k = 0; k < 6; ++k) ForceArr.Add(MakeShared<FJsonValueNumber>(Force[k]));

            TSharedPtr<FJsonObject> CObj = MakeShared<FJsonObject>();
            CObj->SetStringField(TEXT("geom1"),  G1Name);
            CObj->SetStringField(TEXT("geom2"),  G2Name);
            CObj->SetStringField(TEXT("body1"),  B1Name);
            CObj->SetStringField(TEXT("body2"),  B2Name);
            CObj->SetArrayField (TEXT("pos"),    Pos);
            CObj->SetArrayField (TEXT("normal"), Normal);
            CObj->SetNumberField(TEXT("dist"),   c.dist);
            CObj->SetArrayField (TEXT("force"),  ForceArr);
            Out.Add(MakeShared<FJsonValueObject>(CObj));
            ++Matched;
        }
    }

    TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
    Reply->SetStringField(TEXT("op"),          TEXT("get_contacts_ok"));
    Reply->SetNumberField(TEXT("n_contacts"),  Matched);
    Reply->SetBoolField  (TEXT("truncated"),   bTruncated);
    Reply->SetArrayField (TEXT("contacts"),    Out);
    return Reply;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleListKeyframes(const TSharedPtr<FJsonObject>& /*Req*/)
{
    AAMjManager* Mgr = OwnerMgr.Get();
    if (!Mgr || !Mgr->PhysicsEngine || !Mgr->PhysicsEngine->IsInitialized())
        return MakeError(TEXT("not_ready"), TEXT("Manager not initialised"));

    mjModel* m = Mgr->PhysicsEngine->GetModel();
    if (!m) return MakeError(TEXT("not_ready"), TEXT("MjModel missing"));

    auto Slice = [](const mjtNum* src, int32 stride, int32 idx, int32 width)
    {
        TArray<TSharedPtr<FJsonValue>> Out;
        if (!src || width <= 0) return Out;
        for (int32 k = 0; k < width; ++k)
            Out.Add(MakeShared<FJsonValueNumber>(src[idx * stride + k]));
        return Out;
    };

    TArray<TSharedPtr<FJsonValue>> Keys;
    for (int32 i = 0; i < m->nkey; ++i)
    {
        const char* NameC = mj_id2name(m, mjOBJ_KEY, i);
        TSharedPtr<FJsonObject> K = MakeShared<FJsonObject>();
        K->SetStringField(TEXT("name"), NameC ? UTF8_TO_TCHAR(NameC) : TEXT(""));
        K->SetNumberField(TEXT("time"), m->key_time ? m->key_time[i] : 0.0);
        K->SetArrayField(TEXT("qpos"),       Slice(m->key_qpos, m->nq,        i, m->nq));
        K->SetArrayField(TEXT("qvel"),       Slice(m->key_qvel, m->nv,        i, m->nv));
        K->SetArrayField(TEXT("ctrl"),       Slice(m->key_ctrl, m->nu,        i, m->nu));
        K->SetArrayField(TEXT("mocap_pos"),  Slice(m->key_mpos, m->nmocap * 3, i, m->nmocap * 3));
        K->SetArrayField(TEXT("mocap_quat"), Slice(m->key_mquat, m->nmocap * 4, i, m->nmocap * 4));
        Keys.Add(MakeShared<FJsonValueObject>(K));
    }

    TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
    Reply->SetStringField(TEXT("op"),        TEXT("list_keyframes_ok"));
    Reply->SetArrayField (TEXT("keyframes"), Keys);
    return Reply;
}

// =============================================================================
// recording_* / replay_*  delegate to AMjReplayManager
// =============================================================================

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleRecording(const FString& Op, const TSharedPtr<FJsonObject>& Req)
{
    AAMjManager* Mgr = OwnerMgr.Get();
    if (!Mgr) return MakeError(TEXT("not_ready"), TEXT("Manager missing"));
    // Use the game-thread-cached pointer; TActorIterator from this worker
    // thread would assert IsInGameThread() and crash.
    AMjReplayManager* RM = CachedReplayManager.Get();
    if (!RM) return MakeError(TEXT("not_ready"), TEXT("AMjReplayManager not present in scene"));

    if (Op.Equals(TEXT("recording_start")))
    {
        if (RM->bIsRecording) return MakeError(TEXT("recording_already_active"), TEXT("Recording already active"));
        double MaxDur = 0.0;
        if (Req->TryGetNumberField(TEXT("max_duration_s"), MaxDur))
            RM->MaxRecordDuration = (float)MaxDur;
        else
            RM->MaxRecordDuration = FLT_MAX;
        RM->StartRecording();
        TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
        Reply->SetStringField(TEXT("op"),             TEXT("recording_start_ok"));
        Reply->SetStringField(TEXT("name"),           AMjReplayManager::LiveSessionName);
        Reply->SetNumberField(TEXT("max_duration_s"), RM->MaxRecordDuration);
        return Reply;
    }
    if (Op.Equals(TEXT("recording_stop")))
    {
        if (!RM->bIsRecording) return MakeError(TEXT("recording_not_active"), TEXT("Recording is not active"));
        RM->StopRecording();
        TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
        Reply->SetStringField(TEXT("op"),   TEXT("recording_stop_ok"));
        Reply->SetStringField(TEXT("name"), AMjReplayManager::LiveSessionName);
        // Populate the summary fields the Python client maps to
        // RecordingSummary. Recording has been stopped (StopRecording
        // above set bIsRecording=false) so the OnPostStep hook isn't
        // mutating Frames in parallel.
        Reply->SetNumberField(TEXT("frame_count"),    static_cast<double>(RM->GetLiveFrameCount()));
        Reply->SetNumberField(TEXT("sim_duration_s"), RM->GetLiveSimDurationS());
        return Reply;
    }
    if (Op.Equals(TEXT("recording_save")))
    {
        FString Path;
        Req->TryGetStringField(TEXT("path"), Path);
        FString FileName = Path.IsEmpty() ? TEXT("recording.json") : FPaths::GetCleanFilename(Path);
        if (Path.IsEmpty()) Path = FileName;
        bool bOk = RM->SaveRecordingToFile(FileName);
        // ResolveReplayPath now matches the manager's
        // ProjectSavedDir/URLab/Replays/ output dir, so the absolute_path
        // returned to the client points at the file the manager actually wrote.
        FString Abs = ResolveReplayPath(Path);
        TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
        Reply->SetStringField(TEXT("op"),            bOk ? TEXT("recording_save_ok") : TEXT("error"));
        Reply->SetStringField(TEXT("absolute_path"), Abs);
        if (!bOk) Reply->SetStringField(TEXT("code"), TEXT("path_not_writable"));
        return Reply;
    }
    if (Op.Equals(TEXT("recording_clear")))
    {
        RM->ClearRecording();
        TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
        Reply->SetStringField(TEXT("op"), TEXT("recording_clear_ok"));
        return Reply;
    }
    return MakeError(TEXT("unknown_op"), Op);
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleReplay(const FString& Op, const TSharedPtr<FJsonObject>& Req)
{
    AAMjManager* Mgr = OwnerMgr.Get();
    if (!Mgr) return MakeError(TEXT("not_ready"), TEXT("Manager missing"));
    AMjReplayManager* RM = CachedReplayManager.Get();
    if (!RM) return MakeError(TEXT("not_ready"), TEXT("AMjReplayManager not present in scene"));

    if (Op.Equals(TEXT("replay_load")))
    {
        FString P;
        if (!Req->TryGetStringField(TEXT("path"), P))
            return MakeError(TEXT("missing_field"), TEXT("replay_load requires 'path'"));
        FString FileName = FPaths::GetCleanFilename(P);
        bool bOk = RM->LoadRecordingFromFile(FileName);
        if (!bOk) return MakeError(TEXT("path_not_readable"), P);
        TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
        Reply->SetStringField(TEXT("op"),   TEXT("replay_load_ok"));
        Reply->SetStringField(TEXT("name"), FPaths::GetBaseFilename(FileName));
        return Reply;
    }
    if (Op.Equals(TEXT("replay_list_sessions")))
    {
        TArray<TSharedPtr<FJsonValue>> Names;
        for (const FString& N : RM->GetSessionNames())
            Names.Add(MakeShared<FJsonValueString>(N));
        TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
        Reply->SetStringField(TEXT("op"), TEXT("replay_list_sessions_ok"));
        Reply->SetArrayField(TEXT("sessions"), Names);
        return Reply;
    }
    if (Op.Equals(TEXT("replay_set_active")))
    {
        FString N;
        if (!Req->TryGetStringField(TEXT("name"), N))
            return MakeError(TEXT("missing_field"), TEXT("replay_set_active requires 'name'"));
        if (!RM->Sessions.Contains(N)) return MakeError(TEXT("replay_session_not_found"), N);
        RM->SetActiveSession(N);
        TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
        Reply->SetStringField(TEXT("op"), TEXT("replay_set_active_ok"));
        return Reply;
    }
    if (Op.Equals(TEXT("replay_start")))
    {
        if (ActiveStepMode == EStepMode::Live)
            return MakeError(TEXT("replay_requires_stepped"),
                TEXT("Switch to direct or puppet before starting replay"));
        RM->StartReplay();
        int32 Total = RM->Sessions.Contains(RM->GetActiveSessionName())
            ? RM->Sessions[RM->GetActiveSessionName()].Frames.Num() : 0;
        TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
        Reply->SetStringField(TEXT("op"),             TEXT("replay_start_ok"));
        Reply->SetStringField(TEXT("active_session"), RM->GetActiveSessionName());
        Reply->SetNumberField(TEXT("total_frames"),   Total);
        return Reply;
    }
    if (Op.Equals(TEXT("replay_stop")))
    {
        RM->StopReplay();
        TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
        Reply->SetStringField(TEXT("op"), TEXT("replay_stop_ok"));
        return Reply;
    }
    return MakeError(TEXT("unknown_op"), Op);
}
