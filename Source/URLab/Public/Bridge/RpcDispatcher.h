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

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "Dom/JsonObject.h"
#include "Containers/Queue.h"
#include <atomic>

class AAMjManager;
class AMjReplayManager;
struct FMjStepRequest;
struct FMjDirectStepCommand;
struct FMjPushStateRequest;

/**
 * @brief Transport-agnostic step-server core.
 *
 * Owns session id, mode state, step counter, request queues, and all the
 * Handle* / dispatch logic. Knows nothing about ZMQ, SHM, msgpack, or any
 * specific wire transport. Multiple transports (ZMQ, SHM) call into the
 * same dispatcher; they only differ in how bytes get from the wire to
 * `Dispatch()` and back.
 *
 * Lives on `AAMjManager` so all transports share one instance; the
 * dispatcher's serializing mutex makes concurrent dispatches from
 * multiple transports safe.
 */
class URLAB_API FURLabRpcDispatcher
{
public:
    /** Observation verbosity. minimal=qpos+qvel; standard=+ctrl+act+sensors;
     *  full=+body xpos/xquat+actuator forces. */
    enum class EObservationLevel : uint8 { Minimal, Standard, Full };

    /** Per-camera include mode for step replies. */
    enum class ECameraInclude : uint8 { Sync, Latest };

    FURLabRpcDispatcher();
    ~FURLabRpcDispatcher();

    /** Bind the dispatcher to a manager (game thread, called from BeginPlay). */
    void Init(AAMjManager* InManager);

    /** Per-manager teardown: drop the manager pointer, uninstall step
     *  handlers, drain queues, reset per-PIE state (mode, step counter).
     *  KEEPS the bridge-level session id, encoding flag, and observation
     *  level — those belong to the connected client, not to a single
     *  PIE manager. Called when a PIE cycle ends or the editor level
     *  changes; the same bridge session should survive both. */
    void OnManagerGone();

    /** Full bridge-server teardown. Resets everything OnManagerGone
     *  does PLUS the session id, encoding flag, etc. Called from
     *  BridgeServer::Stop. */
    void Shutdown();

    /** Cooperative shutdown signal. Set by BridgeServer::Stop() so blocking
     *  handlers can exit early. Reset in Init() for the next session. */
    void SetDraining(bool bIn) { bDraining.store(bIn, std::memory_order_release); }
    bool IsDraining() const { return bDraining.load(std::memory_order_acquire); }

    /** Wire format echoed to the client as the urlab plugin version. */
    FString URLabVersion = TEXT("urlab/0.1");

    // --- Top-level dispatch ---

    /** Parse one inbound (already-decoded) request and run the matching
     *  Handle* method. Returns the reply as an FJsonObject; the transport
     *  layer is responsible for serialising to bytes (msgpack/JSON). */
    TSharedPtr<FJsonObject> Dispatch(const TSharedPtr<FJsonObject>& Req);

    // --- Session / mode introspection (used by transports + UI) ---

    bool ValidateSession(const FString& ClientSessionId) const
    {
        if (ClientSessionId.IsEmpty()) return false;
        return ClientSessionId.Equals(ActiveSessionId);
    }
    FString GetActiveSessionId() const { return ActiveSessionId; }
    void SetActiveSessionIdForTest(const FString& Id) { ActiveSessionId = Id; }

    EStepMode GetActiveStepMode() const { return ActiveStepMode; }
    void SetActiveStepMode(EStepMode NewMode);

    EObservationLevel GetActiveObservationLevel() const { return ActiveObservationLevel; }
    void SetActiveObservationLevelForTest(EObservationLevel L) { ActiveObservationLevel = L; }

    bool GetUseJsonEncoding() const { return bUseJsonEncoding.load(std::memory_order_acquire); }
    void SetUseJsonEncoding(bool bUse) { bUseJsonEncoding.store(bUse, std::memory_order_release); }

    int64 GetStepCounter() const { return StepCounter.load(std::memory_order_acquire); }

    /** Worker threads read the replay manager via this cache instead of
     *  TActorIterator (which asserts IsInGameThread). */
    void SetCachedReplayManager(AMjReplayManager* RM);

    // --- Test seams (lifted from UURLabZmqRpcTransport) ---

    void EnqueueStepRequestForTest(FMjStepRequest&& Req);
    void EnqueuePushStateRequestForTest(FMjPushStateRequest&& Req);
    void DrainQueuesForTest();

    // --- Static helpers (transport-agnostic, callable from anywhere) ---

    /** Build the hello-reply payload. When bIncludeAssets is true (caller
     *  asked via hello.include_assets=true), the payload also carries:
     *    - mjcf_compiled (string): the compiled MJCF re-serialised via
     *      mj_saveXMLString, with all mesh file= paths flattened to the
     *      bare filename so a VFS keyed by clean filename resolves them.
     *    - vfs_assets (msgpack-bin object): each VFS-registered asset
     *      shipped as a raw binary field (key = filename).
     *  Without the flag, none of that is sent. Asset payloads can be
     *  multi-MB for typical robots; the legacy bridge handshake stays
     *  light by default. */
    static TSharedPtr<FJsonObject> BuildHandshakePayload(AAMjManager* Manager,
                                                          const FString& SessionId,
                                                          const FString& URLabVer,
                                                          bool bIncludeAssets = false);

    static void ApplyStepCtrl(AAMjManager* Manager, const FMjStepRequest& Req,
                              mjModel* m, mjData* d);

    static TSharedPtr<FJsonObject> BuildStepObservations(AAMjManager* Manager,
                                                          mjModel* m, mjData* d,
                                                          EObservationLevel Level = EObservationLevel::Standard);

    static TSharedPtr<FJsonObject> BuildEntitiesBlock(AAMjManager* Manager,
                                                          mjModel* m, mjData* d);

    static TSharedPtr<FJsonObject> BuildCamerasBlock(AAMjManager* Manager,
                                                      const TMap<FString, ECameraInclude>& CameraSpec,
                                                      int32 TimeoutMs = 1000);

    static TSharedPtr<FJsonObject> MakeError(const FString& Code, const FString& Message);

    /** Stamps `sim_time` + `wall_time` blocks (ROS `builtin_interfaces/Time`
     *  layout: `{sec, nsec}`). */
    static void AppendClockFields(TSharedPtr<FJsonObject>& Reply, double SimTimeSec);

private:
    /** Inner body of Dispatch; wrapped by the public method so every
     *  call gets one in-and-out log line regardless of which early
     *  return fires. */
    TSharedPtr<FJsonObject> DispatchInternal(const TSharedPtr<FJsonObject>& Req);

    /** Weak so stale-manager bugs return nullptr on .Get() instead of
     *  dangling. The dispatcher is a plain C++ class so the pointer is
     *  weak via TWeakObjectPtr rather than UPROPERTY. */
    TWeakObjectPtr<AAMjManager> OwnerMgr;

    /** Guards ActiveSessionId + step-handler install/uninstall. NOT held
     *  across handler bodies — Dispatch releases it before invoking. */
    FCriticalSection DispatchMutex;

    FString ActiveSessionId;

    std::atomic<EStepMode> ActiveStepMode{EStepMode::Live};
    std::atomic<EObservationLevel> ActiveObservationLevel{EObservationLevel::Standard};
    std::atomic<int64> StepCounter{0};

    /** false = msgpack (default), true = JSON. */
    std::atomic<bool> bUseJsonEncoding{false};

    /** Set by BridgeServer::Stop() so blocking handlers exit early. */
    std::atomic<bool> bDraining{false};

    TWeakObjectPtr<AMjReplayManager> CachedReplayManager;

    /** SPSC. Shared ownership so both RPC + physics threads can drop their
     *  ref independently — avoids UAF on RPC-side timeout while the handler
     *  is still processing. */
    TQueue<TSharedPtr<FMjDirectStepCommand>, EQueueMode::Spsc> StepQueue;

    /** Puppet-mode request queue. */
    TQueue<FMjPushStateRequest, EQueueMode::Spsc> PushStateQueue;

    /** Custom step handlers installed on the engine for Direct / Puppet. */
    UMjPhysicsEngine::FMujocoStepCallback PuppetStepHandler;
    UMjPhysicsEngine::FMujocoStepCallback DirectStepHandler;
    bool bPuppetHandlerInstalled = false;
    bool bDirectHandlerInstalled = false;

    void InstallPuppetHandler();
    void UninstallPuppetHandler();
    void InstallDirectHandler();
    void UninstallDirectHandler();

    /** Register every dispatcher-owned op (manager-required +
     *  no-manager) on the URLabOpRegistry with the right Category and
     *  Namespace metadata. Bound `this` lambdas — paired with
     *  UnregisterDispatcherOps() in the destructor.
     *
     *  `RegisteredOpNames` records every name the dispatcher registered
     *  so the destructor can unregister exactly that set without
     *  hardcoding a parallel list. */
    void RegisterDispatcherOps();
    void UnregisterDispatcherOps();
    TArray<FString> RegisteredOpNames;

    // Op handlers
    TSharedPtr<FJsonObject> HandleHello(const TSharedPtr<FJsonObject>& Req);
    TSharedPtr<FJsonObject> HandleMeta(const TSharedPtr<FJsonObject>& Req);
    TSharedPtr<FJsonObject> HandleStep(const TSharedPtr<FJsonObject>& Req);
    TSharedPtr<FJsonObject> HandleReset(const TSharedPtr<FJsonObject>& Req);
    TSharedPtr<FJsonObject> HandleForward(const TSharedPtr<FJsonObject>& Req);
    TSharedPtr<FJsonObject> HandleSetMode(const TSharedPtr<FJsonObject>& Req);
    TSharedPtr<FJsonObject> HandleSetPaused(const TSharedPtr<FJsonObject>& Req);
    TSharedPtr<FJsonObject> HandleConfigureController(const TSharedPtr<FJsonObject>& Req);
    TSharedPtr<FJsonObject> HandleSetSimOptions(const TSharedPtr<FJsonObject>& Req);
    TSharedPtr<FJsonObject> HandleSetSimSpeed(const TSharedPtr<FJsonObject>& Req);
    TSharedPtr<FJsonObject> HandleSetControlSource(const TSharedPtr<FJsonObject>& Req);
    TSharedPtr<FJsonObject> HandleSetTwist(const TSharedPtr<FJsonObject>& Req);
    TSharedPtr<FJsonObject> HandleSetQpos(const TSharedPtr<FJsonObject>& Req);
    TSharedPtr<FJsonObject> HandleSetMocapPose(const TSharedPtr<FJsonObject>& Req);
    TSharedPtr<FJsonObject> HandleReadMocapPose(const TSharedPtr<FJsonObject>& Req);
    TSharedPtr<FJsonObject> HandleGetContacts(const TSharedPtr<FJsonObject>& Req);
    TSharedPtr<FJsonObject> HandleListKeyframes(const TSharedPtr<FJsonObject>& Req);
    TSharedPtr<FJsonObject> HandleRecording(const FString& Op, const TSharedPtr<FJsonObject>& Req);
    TSharedPtr<FJsonObject> HandleReplay(const FString& Op, const TSharedPtr<FJsonObject>& Req);
};
