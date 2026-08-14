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
#include "Bridge/ControlOwnership.h"
#include "State/MjObservationLevel.h"
#include "Dom/JsonObject.h"
#include "Containers/Queue.h"
#include <atomic>

class AAMjManager;
class AMjReplayManager;
class UURLabBridgeServer;
struct FMjStateSnapshot;
struct FMjStepRequest;
struct FMjDirectStepCommand;
struct FStepModeStrategy;
struct FLiveStepMode;
struct FDirectStepMode;
struct FPuppetStepMode;

/**
 * @brief Transport-agnostic step-server core.
 *
 * Owns session id, mode state, step counter, request queues, and all the
 * Handle* / dispatch logic. Knows nothing about ZMQ, SHM, msgpack, or any
 * specific wire transport. Multiple transports (ZMQ, SHM) call into the
 * same dispatcher; they only differ in how bytes get from the wire to
 * `Dispatch()` and back.
 *
 * Lives on `AAMjManager` so all transports share one instance. DispatchMutex
 * serialises only the session / mode / step-handler mutation; it is released
 * before the handler bodies, so those run concurrently across transports
 * (ZMQ + SHM). The direct-mode step queue is therefore MPSC and each command's
 * completion / abandon flag is atomic.
 */
class URLAB_API FURLabRpcDispatcher
{
	// The per-mode strategies own the step body (RpcHandlers_Step.cpp) and reach
	// into dispatcher internals (queue, counters, camera helpers) to run it.
	friend struct FLiveStepMode;
	friend struct FDirectStepMode;
	friend struct FPuppetStepMode;

public:
	/** Observation verbosity. minimal=qpos+qvel; standard=+ctrl+act+sensors;
	 *  full=+body xpos/xquat+actuator forces. */
	/** Per-camera include mode for step replies. */
	enum class ECameraInclude : uint8
	{
		Sync,
		Latest
	};

	FURLabRpcDispatcher();
	~FURLabRpcDispatcher();

	/** Bind the dispatcher to a manager (game thread, called from BeginPlay). */
	void Init(AAMjManager* InManager);

	/** Back-pointer to the owning bridge server, set once at construction so
	 *  no-manager ops (leasing) can reach per-instance state even before a
	 *  scene/manager exists. */
	void SetOwningBridge(UURLabBridgeServer* InBridge);

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
	FString URLabVersion = TEXT("urlab/0.2");

	// --- Top-level dispatch ---

	/** Parse one inbound (already-decoded) request and run the matching
	 *  Handle* method. Returns the reply as an FJsonObject; the transport
	 *  layer is responsible for serialising to bytes (msgpack/JSON). */
	TSharedPtr<FJsonObject> Dispatch(const TSharedPtr<FJsonObject>& Req);

	// --- Session / mode introspection (used by transports + UI) ---

	bool ValidateSession(const FString& ClientSessionId) const
	{
		if (ClientSessionId.IsEmpty())
			return false;
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

	/** Per-articulation control arbitration shared by every control write.
	 *  Exposed so tests can drive claims and the deterministic clock seam. */
	FMjControlOwnership& GetControlOwnership() { return ControlOwnership; }

	/** Worker threads read the replay manager via this cache instead of
	 *  TActorIterator (which asserts IsInGameThread). */
	void SetCachedReplayManager(AMjReplayManager* RM);

	// --- Test seams (lifted from UURLabZmqRpcTransport) ---

	void EnqueueStepRequestForTest(FMjStepRequest&& Req);

	/** Empty the direct-mode step queue, abandoning + waking every pending
	 *  command so a blocked RPC thread returns at once. Called on mode switch,
	 *  PIE-end, and from tests. */
	void DrainQueues();

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

	/**
	 * Non-blocking camera retrieval from each camera's frame-history ring.
	 * CameraSpec selects the cameras; MinFrameIds optionally requests, per
	 * camera, the frame showing state >= that id (0 / absent = latest
	 * available). Requested cameras are touched so per-camera capture gating
	 * keeps them live. Each emitted camera carries its frame's `frame_id` /
	 * `sim_time` so the bridge can associate an image with a step.
	 */
	static TSharedPtr<FJsonObject> BuildCamerasBlock(AAMjManager* Manager,
		const TMap<FString, ECameraInclude>& CameraSpec,
		const TMap<FString, uint64>& MinFrameIds = TMap<FString, uint64>(),
		int32 TimeoutMs = 1000);

	/** Assemble the base step_ok reply (op/time/step/clock/frame_id plus the
	 *  encoded `arts` / `scene` blocks) from the state IR. Must be called while
	 *  the snapshot is valid (under the engine's CallbackMutex). Cameras and any
	 *  mode-specific fields (e.g. puppet perturbation) are appended by the caller
	 *  after the lock is released. */
	TSharedPtr<FJsonObject> BuildStepReply(const FMjStateSnapshot& Snapshot,
		uint64 FrameId, EObservationLevel Level);

	/** Append a `cameras` block to a step reply for the requested cameras, if any
	 *  produced a frame. No-op when CameraSpec is empty. */
	void AppendCamerasBlock(TSharedPtr<FJsonObject>& Reply, AAMjManager* Mgr,
		const TMap<FString, ECameraInclude>& CameraSpec,
		const TMap<FString, uint64>& CameraMinFrameIds);

	/** Block the calling (RPC) thread until each requested camera has a frame
	 *  with id >= MinFrameId, or the timeout elapses, or the bridge drains.
	 *  For cameras that reach it, records MinFrameId as their floor in
	 *  CameraMinFrameIds so the reply returns the frame showing this step's
	 *  state; cameras that time out are left to return their latest frame
	 *  (its smaller frame_id signals staleness to the client). Returns true if
	 *  every requested camera reached MinFrameId. */
	bool WaitForCameraFrames(AAMjManager* Mgr,
		const TMap<FString, ECameraInclude>& CameraSpec,
		uint64 MinFrameId, int32 TimeoutMs,
		TMap<FString, uint64>& CameraMinFrameIds);

	/** Render-on-demand: on the game thread, apply the latest physics snapshot,
	 *  capture + read back every requested camera, flush the render thread once,
	 *  and harvest — so the frame for MinFrameId exists within a single pump
	 *  instead of over several ticks. Blocks the RPC thread until done or
	 *  TimeoutMs elapses. Records MinFrameId as the floor for cameras that
	 *  produced it. Opt-in (`render: "sync"`); flushes the game thread, so it is
	 *  for eval, not interactive use. */
	bool RenderCamerasSync(AAMjManager* Mgr,
		const TMap<FString, ECameraInclude>& CameraSpec,
		uint64 MinFrameId, int32 TimeoutMs,
		TMap<FString, uint64>& CameraMinFrameIds,
		bool bWait = true);

	/** Build the name->camera lookup used to resolve include_cameras /
	 *  set_camera_streaming keys. Keyed solely by the canonical "<art>/<part>"
	 *  name (FMjCanonicalName), matching the handshake zmq_topic. First writer
	 *  wins per key so a sanitize-collision can't hide a distinct camera. */
	static void BuildCameraNameMap(AAMjManager* Manager,
		TMap<FString, class UMjCamera*>& OutByName);

	/** Game-thread: apply per-camera streaming requests (key -> {zmq, shm}),
	 *  toggling the broadcast flags + SetStreamingEnabled, and return the
	 *  per-camera reply object (streaming flag + bound zmq endpoint/topic). */
	static TSharedPtr<FJsonObject> ApplyCameraStreamingGameThread(AAMjManager* Manager,
		const TMap<FString, TPair<bool, bool>>& Requests);

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

	/** Owning bridge server (set at construction). Weak so a torn-down
	 *  server leaves this null rather than dangling. Reached by the lease
	 *  ops, which are per-process and independent of any manager. */
	TWeakObjectPtr<UURLabBridgeServer> OwningBridge;

	/** Guards ActiveSessionId + step-handler install/uninstall. NOT held
	 *  across handler bodies — Dispatch releases it before invoking. */
	FCriticalSection DispatchMutex;

	FString ActiveSessionId;

	/** Per-articulation control ownership. Reset per PIE from OnManagerGone. */
	FMjControlOwnership ControlOwnership;

	std::atomic<EStepMode> ActiveStepMode{EStepMode::Live};
	std::atomic<EObservationLevel> ActiveObservationLevel{EObservationLevel::Standard};
	std::atomic<int64> StepCounter{0};

	/** false = msgpack (default), true = JSON. */
	std::atomic<bool> bUseJsonEncoding{false};

	/** Set by BridgeServer::Stop() so blocking handlers exit early. */
	std::atomic<bool> bDraining{false};

	TWeakObjectPtr<AMjReplayManager> CachedReplayManager;

	/** MPSC. Two transport threads (ZMQ + SHM) can be inside the direct-mode
	 *  step body at once (DispatchMutex is released before handler bodies), so
	 *  both may enqueue. Shared ownership so the RPC + physics threads can drop
	 *  their ref independently — avoids UAF on an RPC-side timeout while the
	 *  handler is still processing. */
	TQueue<TSharedPtr<FMjDirectStepCommand>, EQueueMode::Mpsc> StepQueue;

	/** Custom step handler installed on the engine for Direct mode. */
	UMjPhysicsEngine::FMujocoStepCallback DirectStepHandler;
	bool bDirectHandlerInstalled = false;

public:
	// Public so the per-mode strategy objects can drive them on enter/exit.
	void InstallDirectHandler();
	void UninstallDirectHandler();

	/** After a mid-session recompile rebuilt mjModel/mjData, re-run the active
	 *  strategy's OnEnter (under DispatchMutex) so its step handler is
	 *  reinstalled onto the fresh engine and the pause / pacing invariants are
	 *  restored. Called from the engine's recompile path. */
	void ReapplyActiveStepMode();

private:
	/** Per-mode lifecycle + step body strategy: OnEnter installs the handler +
	 *  sets pause / publisher state, OnExit uninstalls, HandleStep runs the
	 *  mode's per-step work. Swapped by SetActiveStepMode. Shared so an in-flight
	 *  HandleStep keeps the strategy alive across a concurrent set_mode swap. */
	TSharedPtr<struct FStepModeStrategy> CurrentStepStrategy;
	static TSharedPtr<struct FStepModeStrategy> MakeStepStrategy(EStepMode Mode);

	/** Parse the request-scoped step fields (observation override, camera spec,
	 *  wait / render flags) shared by every mode into Out. Does NOT mutate any
	 *  session-level state. */
	void ParseStepCommon(const TSharedPtr<FJsonObject>& Req, AAMjManager* Mgr,
		struct FStepRequestCommon& Out) const;

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

	/** Who a request writes control as: the optional `control_owner` field,
	 *  falling back to `session_id`. Named for the ownership gate rather than
	 *  `source`, which `set_control_source` already spends on "zmq" | "ui". */
	FString ResolveControlSource(const TSharedPtr<FJsonObject>& Req) const;

	/** Consult the control gate for a write to `ArtKey`. Returns nullptr when
	 *  the request's source owns it; otherwise a `not_control_owner` error reply
	 *  carrying the current owner. */
	TSharedPtr<FJsonObject> RejectIfNotControlOwner(FName ArtKey,
		const TSharedPtr<FJsonObject>& Req);

	// Op handlers
	TSharedPtr<FJsonObject> HandleHello(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleMeta(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleAcquireLease(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleReleaseLease(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleFastpathHello(const TSharedPtr<FJsonObject>& Req);
	// Network model upload (RpcHandlers_ModelUpload.cpp). Manifest + chunk are
	// pure data staging on the RPC thread; commit materialises to a temp dir and
	// drives the existing import_xml editor job.
	TSharedPtr<FJsonObject> HandleUploadModelManifest(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleUploadModelChunk(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleUploadModelCommit(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleStep(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleReset(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleForward(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleSetMode(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleSetPaused(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleSetCameraStreaming(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleSetCameraDelay(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleConfigureController(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleSetSimOptions(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleSetSimSpeed(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleSetControlSource(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleClaimControl(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleReleaseControl(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleSetUserChannels(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleSetTwist(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleSetQpos(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleSetMocapPose(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleReadMocapPose(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleGetContacts(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleListKeyframes(const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleRecording(const FString& Op, const TSharedPtr<FJsonObject>& Req);
	TSharedPtr<FJsonObject> HandleReplay(const FString& Op, const TSharedPtr<FJsonObject>& Req);
};

/** Request-scoped step fields parsed once per step (ParseStepCommon) and handed
 *  to the active strategy's HandleStep. Nothing here is session state — the
 *  observation override applies to this step only. */
struct FStepRequestCommon
{
	EObservationLevel ObservationLevel = EObservationLevel::Standard;
	TMap<FString, FURLabRpcDispatcher::ECameraInclude> CameraSpec;
	TMap<FString, uint64> CameraMinFrameIds;
	bool bWaitCameras = false;
	bool bRenderSync = false;
	bool bRenderAsync = false;
	int32 CameraTimeoutMs = 200;
};

/** Per-mode step lifecycle + body. Concrete Live/Direct/Puppet strategies (in
 *  RpcHandlers_Step.cpp) install/uninstall the step handler and set the engine
 *  pause + publisher state on transition, and own the per-step work in
 *  HandleStep, so the mode logic isn't a growing if-chain in HandleStep /
 *  SetActiveStepMode. */
struct FStepModeStrategy
{
	virtual ~FStepModeStrategy() = default;
	virtual EStepMode Mode() const = 0;
	virtual void OnEnter(FURLabRpcDispatcher& Dispatcher, AAMjManager& Mgr) = 0;
	virtual void OnExit(FURLabRpcDispatcher& Dispatcher, AAMjManager& Mgr) = 0;
	virtual TSharedPtr<FJsonObject> HandleStep(FURLabRpcDispatcher& D,
		const TSharedPtr<FJsonObject>& Req, const FStepRequestCommon& Common) = 0;
};
