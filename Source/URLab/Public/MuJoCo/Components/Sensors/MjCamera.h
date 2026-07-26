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

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "MuJoCo/Components/MjComponent.h"
#include "HAL/Runnable.h"
#include "Containers/Queue.h"
#include "Templates/PimplPtr.h"
#include "MuJoCo/Components/Sensors/MjCameraTypes.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "RHIGPUReadback.h"
#include "Math/RandomStream.h"
#include <atomic>
#include "MjCamera.generated.h"

// Opaque ROS image publisher handle from the rcl seam; defined in
// UrlabRclCore.cpp. Held by pointer so this header pulls in no ROS types.
struct UrlabRclImagePub;

/**
 * @class FCameraZmqWorker
 * @brief Background thread that publishes high-bandwidth camera frames over
 *        ZeroMQ. All transport internals (the libzmq context/socket handles, the
 *        per-format frame queues, the bind bookkeeping) live in an opaque state
 *        object defined in the .cpp, so this header stays free of raw handles.
 */
class FCameraZmqWorker : public FRunnable
{
public:
	FCameraZmqWorker(const FString& InEndpoint, const FString& InTopic, FIntPoint InRes);
	virtual ~FCameraZmqWorker();

	virtual bool Init() override;
	virtual uint32 Run() override;
	virtual void Stop() override;
	virtual void Exit() override;

	void PushFrame(const TArray<FColor>& FrameData, const FMjCameraFrameMeta& Meta);
	void PushFrame(const TArray<float>& FrameData, const FMjCameraFrameMeta& Meta);
	FString GetBoundEndpoint() const;

	/** Process-wide pause gate. Workers drain without sending while set, to bound
	 *  render-thread memory in Direct/Puppet mode. */
	static URLAB_API std::atomic<bool> bPublishersPaused;

private:
	struct FState;
	TPimplPtr<FState> State;
};

/**
 * @struct FMjCameraFrame
 * @brief One retained camera frame, tagged with the post-step physics state it
 *        shows. CaptureMode decides which pixel buffer is populated.
 *
 * Frames are retained in the history ring as `TSharedPtr<const FMjCameraFrame>`,
 * so history retention, the RPC fetch, and the streaming publish all share one
 * allocation by refcount rather than deep-copying the multi-MB pixel array.
 */
struct FMjCameraFrame
{
	uint64 FrameId = 0; // post-step render-snapshot id this frame shows
	double SimTime = 0.0;
	int32 Width = 0;
	int32 Height = 0;
	TArray<FColor> Color; // Real / seg modes (BGRA8)
	TArray<float> Depth;  // Depth mode (float32)
	// Unix-epoch capture time (FDateTime::UtcNow at readback request). Carried
	// here so a delayed re-publish can rebuild the v2 wire meta with the
	// ORIGINAL capture time, so the client's content-age math reflects the
	// injected latency rather than the moment we re-sent the bytes.
	double CaptureUnixTime = 0.0;
	// Camera-latency emulation bookkeeping (see UMjCamera delay API).
	//  - RevealValue: the clock value (SimTime or CaptureUnixTime, per
	//    bDelayUseWallClock) at which this frame becomes eligible to publish,
	//    i.e. capture_clock + sampled_delay.
	//  - Seq: monotonic per-harvest counter so each delayed frame publishes
	//    exactly once (FrameId can repeat across intra-step captures).
	double RevealValue = 0.0;
	uint64 Seq = 0;
};

/**
 * @class UMjCamera
 * @brief Represents a MuJoCo <camera> element as an Unreal sensor component.
 *
 * Placed in the Sensors group because cameras are observation devices, not geometry.
 * The component is cheap by default: no render target or GPU cost is incurred until
 * SetStreamingEnabled(true) is called.
 *
 * Key design points:
 *  - No ExportTo / RegisterToSpec: camera is UE-side only, not fed back to MuJoCo.
 *  - SetStreamingEnabled() allocates the RT and registers the viewpoint with the
 *    streaming manager so textures load correctly even when the pawn is far away.
 *  - The GPU readback is fully asynchronous and decoupled from stepping: the
 *    render thread maps + copies each finished readback and pushes the pixels
 *    onto a results queue; the game thread pops them next tick into a history
 *    ring tagged with the post-step FrameId/SimTime they show. No render-thread
 *    flush is taken on the steady-state or synchronous paths.
 *  - Per-camera capture gating: a camera only captures while broadcast-enabled
 *    or recently requested (TouchRequested), so idle cameras cost no GPU.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjCamera : public UMjComponent
{
	GENERATED_BODY()

public:
	// --- CODEGEN_PROPERTIES_START ---
	UPROPERTY(EditAnywhere, Category = "MuJoCo|Camera|Spatial Pose", meta = (InlineEditConditionToggle))
	bool bOverride_Pos = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera|Spatial Pose", meta = (EditCondition = "false", EditConditionHides))
	FVector Pos = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, Category = "MuJoCo|Camera|Orientation", meta = (InlineEditConditionToggle))
	bool bOverride_Quat = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera|Orientation", meta = (EditCondition = "false", EditConditionHides))
	FQuat Quat = FQuat::Identity;

	UPROPERTY(EditAnywhere, Category = "MuJoCo|Camera", meta = (InlineEditConditionToggle))
	bool bOverride_fovy = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera", meta = (EditCondition = "bOverride_fovy", Units = "deg"))
	float fovy = 0.0f;

	UPROPERTY(EditAnywhere, Category = "MuJoCo|Camera", meta = (InlineEditConditionToggle))
	bool bOverride_ipd = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera", meta = (EditCondition = "bOverride_ipd", Units = "m"))
	float ipd = 0.0f;

	UPROPERTY(EditAnywhere, Category = "MuJoCo|Camera", meta = (InlineEditConditionToggle))
	bool bOverride_resolution = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera", meta = (EditCondition = "bOverride_resolution"))
	TArray<int32> resolution = {};

	UPROPERTY(EditAnywhere, Category = "MuJoCo|Camera", meta = (InlineEditConditionToggle))
	bool bOverride_output = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera", meta = (EditCondition = "bOverride_output"))
	float output = 0.0f;

	UPROPERTY(EditAnywhere, Category = "MuJoCo|Camera", meta = (InlineEditConditionToggle))
	bool bOverride_target = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera", meta = (EditCondition = "bOverride_target"))
	FString target = TEXT("");

	UPROPERTY(EditAnywhere, Category = "MuJoCo|Camera", meta = (InlineEditConditionToggle))
	bool bOverride_focal = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera", meta = (EditCondition = "bOverride_focal", MjUnit = "m"))
	TArray<float> focal = {};

	UPROPERTY(EditAnywhere, Category = "MuJoCo|Camera", meta = (InlineEditConditionToggle))
	bool bOverride_focalpixel = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera", meta = (EditCondition = "bOverride_focalpixel"))
	TArray<int32> focalpixel = {};

	UPROPERTY(EditAnywhere, Category = "MuJoCo|Camera", meta = (InlineEditConditionToggle))
	bool bOverride_principal = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera", meta = (EditCondition = "bOverride_principal", MjUnit = "m"))
	TArray<float> principal = {};

	UPROPERTY(EditAnywhere, Category = "MuJoCo|Camera", meta = (InlineEditConditionToggle))
	bool bOverride_principalpixel = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera", meta = (EditCondition = "bOverride_principalpixel"))
	TArray<int32> principalpixel = {};

	UPROPERTY(EditAnywhere, Category = "MuJoCo|Camera", meta = (InlineEditConditionToggle))
	bool bOverride_sensorsize = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera", meta = (EditCondition = "bOverride_sensorsize", MjUnit = "m"))
	TArray<float> sensorsize = {};
	// --- CODEGEN_PROPERTIES_END ---

	// Hand-declared because the UPROPERTY type is a URLab enum. Codegen
	// owns the XML "mode" attr <-> enum mapping + mjsCamera.mode write
	// via xml_enum_attrs in codegen_rules.json. Default = Fixed (camera
	// moves with its body's transform; matches MuJoCo's default).
	UPROPERTY(EditAnywhere, Category = "MuJoCo|MjCamera", meta = (InlineEditConditionToggle))
	bool bOverride_TrackingMode = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjCamera",
		meta = (EditCondition = "bOverride_TrackingMode",
			ToolTip = "MJCF tracking mode (separate from CaptureMode which is the URLab render-mode selector)."))
	EMjCameraTrackingMode TrackingMode = EMjCameraTrackingMode::Fixed;

	// Hand-declared because the UPROPERTY type is a URLab enum. Codegen
	// owns the XML "projection" attr <-> enum mapping + mjsCamera.proj
	// write via xml_enum_attrs in codegen_rules.json. Default =
	// Perspective (matches MuJoCo's mjPROJECTION_PERSPECTIVE).
	UPROPERTY(EditAnywhere, Category = "MuJoCo|MjCamera", meta = (InlineEditConditionToggle))
	bool bOverride_Projection = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|MjCamera",
		meta = (EditCondition = "bOverride_Projection"))
	EMjCameraProjection Projection = EMjCameraProjection::Perspective;

	// ---- Capture configuration ----

	/** What this camera captures. Read at SetStreamingEnabled(true) time:
	 *  toggle streaming off/on after changing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera")
	EMjCameraMode CaptureMode = EMjCameraMode::Real;

	/** @brief Near clip plane for Depth capture, centimetres. Values below this read as 0. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera",
		meta = (EditCondition = "CaptureMode == EMjCameraMode::Depth", ClampMin = "0.1"))
	float DepthNearCm = 10.0f;

	/** @brief Far clip plane for Depth capture, centimetres. Values beyond read as the maximum. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera",
		meta = (EditCondition = "CaptureMode == EMjCameraMode::Depth", ClampMin = "1.0"))
	float DepthFarCm = 10000.0f;

	/** @brief Boost factor passed to IStreamingManager::AddViewInformation.
	 *  Increase to force higher-quality texture mips near this camera. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera|Streaming")
	float StreamingBoost = 1.0f;

	/** Render this capture as nested passes of the main renderer (UE 5.3+),
	 *  sharing visibility/GPU-scene setup, instead of a standalone scene render
	 *  (cheaper with many live cameras). Off by default: it renders on the main
	 *  render cadence, so it is NOT compatible with the render:sync fast path
	 *  (which captures on demand). Enable only for pure live-streaming cameras
	 *  that are never requested with render:"sync". */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera|Streaming")
	bool bRenderInMainRenderer = false;

	/** @brief The underlying SceneCaptureComponent2D. Capture is disabled by default. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Camera")
	USceneCaptureComponent2D* CaptureComponent;

	/** @brief The render target. Null until SetStreamingEnabled(true) is first called. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Camera")
	UTextureRenderTarget2D* RenderTarget = nullptr;

	/** @brief If true, the camera automatically broadcasts its frames over ZeroMQ when streaming is enabled. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera|Network")
	bool bEnableZmqBroadcast = false;

	/** @brief The ZMQ endpoint this camera's PUB socket binds. Resolved when
	 *  streaming is enabled from the instance's camera port block
	 *  (BindAddress + CamBasePort + StreamPortIndex), so N farm instances on
	 *  distinct CamBasePort blocks never collide. This authored value is only a
	 *  fallback used when no instance config is reachable (e.g. an isolated
	 *  component test). The port is still auto-incremented on bind conflict. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera|Network")
	FString ZmqEndpoint = TEXT("tcp://0.0.0.0:5558");

	/** @brief If true, the camera also writes each frame into a per-camera SHM
	 *  region (`<Saved>/URLabShm/<session>/cam_<owner>_<name>.shm`). The ZMQ
	 *  broadcast is unaffected: both transports can run in parallel. The session
	 *  segment is currently the literal "live", so multiple editor processes on
	 *  one host collide on the same file; parameterise the session per instance
	 *  before running a multi-process render farm. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera|Network")
	bool bEnableShmBroadcast = false;

	// ---- Camera latency emulation + capture-rate control ----

	/** Simulated camera latency (seconds). The streamed / served frame is the
	 *  newest whose reveal time <= now, where reveal = capture_clock +
	 *  sampled_delay. 0 = no delay (frames published as soon as harvested).
	 *  Measured in SimTime by default, or wall-clock when bDelayUseWallClock. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera|Delay", meta = (ClampMin = "0.0"))
	float DelaySeconds = 0.0f;

	/** Symmetric uniform jitter half-range (seconds): per-frame effective delay
	 *  ~ U(DelaySeconds - this, DelaySeconds + this), clamped >= 0. 0 = a fixed
	 *  delay. Sampled from a seeded per-camera RNG so jittered latency is
	 *  reproducible across runs. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera|Delay", meta = (ClampMin = "0.0"))
	float DelayJitterSeconds = 0.0f;

	/** If true, delay / jitter / reveal selection use wall-clock (frame capture
	 *  unix time) instead of SimTime. Wall-clock suits real-latency emulation in
	 *  live mode; SimTime is deterministic for stepped runs. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera|Delay")
	bool bDelayUseWallClock = false;

	/** Capture + read back only when the applied physics state advances (the
	 *  manager's FrameId changes). Between steps the world is unchanged, so a
	 *  re-render + GPU readback is wasted work. No-op in live (state advances
	 *  every frame); a large GPU saving while stepping, and zero capture cost
	 *  while paused. Disable to force a capture every engine frame. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera|Capture")
	bool bCaptureOnStateChange = true;

	/** Optional hard cap on capture rate (frames/sec, wall-clock). 0 = uncapped.
	 *  Applied on top of bCaptureOnStateChange to further throttle a high-rate
	 *  feed when the consumer needs fewer frames than the sim emits. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera|Capture", meta = (ClampMin = "0.0"))
	float CaptureMaxFps = 0.0f;

	/** How many recent frames to retain for by-id retrieval. ClampMax mirrors
	 *  MaxHistoryCapacity, the absolute ceiling the eviction paths enforce; keep
	 *  the two in sync. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera", meta = (ClampMin = "1", ClampMax = "64"))
	int32 HistoryCapacity = 8;

	/** How long after a request a camera keeps capturing before going dormant. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera", meta = (ClampMin = "0.0"))
	float RequestActiveTtlSeconds = 2.0f;

	UMjCamera();

	// ---- Runtime configuration ----

	/** Configure latency emulation at runtime (RPC-driven; applied on the game
	 *  thread alongside the per-tick capture, so no extra locking is needed).
	 *  Seed 0 derives a stable seed from the canonical name. */
	void SetCameraDelay(float InDelaySeconds, float InJitterSeconds, bool bInUseWallClock, int32 InSeed);

	/** Configure capture-rate control at runtime. */
	void SetCaptureRate(bool bInOnStateChange, float InMaxFps);

	/**
	 * @brief Allocates the render target and begins streaming / scene capture.
	 *        Call with bEnable=false to stop rendering and free the capture budget.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Camera")
	void SetStreamingEnabled(bool bEnable);

	/** @brief Returns true once SetStreamingEnabled(true) has set up the RT and
	 *  the capture component. Cheap, lock-free read intended for best-effort
	 *  gating from the bridge worker thread (a stale read is benign). */
	bool IsStreamingActive() const { return bStreamingEnabled; }

	/** Resolution as a validated {width, height} pair, substituting the 640x480
	 *  default for any missing or non-positive element. Every pixel-sizing site
	 *  reads through this so a malformed `resolution` array can never index out
	 *  of bounds. */
	FIntPoint GetResolution() const;

	/**
	 * @brief Enqueues a non-blocking asynchronous GPU->CPU pixel readback for the
	 *        current applied state. No-op if streaming is not enabled or the
	 *        in-flight cap is reached. Completed readbacks land in the history
	 *        ring (see GetFrame), tagged with the post-step state id they show.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Camera")
	void RequestReadback();

	/** One per-frame capture pass: capture gating (lazy streaming enable /
	 *  every-frame vs state-change), advance the async readback pipeline, and
	 *  (while active) issue a capture and publish any due delayed frames. Driven
	 *  by UMjCameraSubsystem once per frame rather than a per-component tick. */
	void UpdateCapturePipeline();

	/** Render-on-demand: capture the scene and enqueue a readback right now for
	 *  the current applied state. Requires streaming already enabled. Pair with
	 *  HarvestCompletedReadbacks (called each tick and by the sync render path)
	 *  to pull the finished frame into history. */
	void IssueSyncCapture();

	/** Advance the async readback pipeline: dispatch every GPU-ready readback to
	 *  the render thread for map/copy, and drain the render thread's finished
	 *  frames into the history ring (publishing inline when no delay is
	 *  configured). FIFO; takes no render-thread flush. Called each tick and by
	 *  the synchronous render path. */
	void HarvestCompletedReadbacks();

	/** Poll the async readback pipeline (dispatch + harvest) up to TimeoutSeconds,
	 *  yielding between passes so the render thread can finish its map/copy. Used
	 *  by the synchronous render path to obtain a fresh frame without flushing the
	 *  render thread. */
	void WaitAndHarvestReadbacks(double TimeoutSeconds);

	/** Count of async readbacks awaiting GPU completion (diagnostics). Excludes
	 *  frames already dispatched to the render thread's map/copy. */
	int32 NumInFlightReadbacks() const { return InFlightReadbacks.Num(); }

	/** True while any readback is still awaiting GPU completion or its finished
	 *  frame is still queued for harvest. Lets the synchronous render path avoid
	 *  re-issuing a capture while one is already in flight. */
	bool HasPendingReadbacks() const { return InFlightReadbacks.Num() > 0 || PendingMapCommands > 0; }

	/**
	 * @brief Fetch a frame from this camera's history ring (thread-safe).
	 *
	 * MinFrameId == 0 returns the most recent retained frame. Otherwise returns
	 * the oldest retained frame whose FrameId >= MinFrameId, i.e. the frame
	 * showing the state at/after that step. Returns false if no matching frame is
	 * retained yet. Fills Out.Color for Real/seg modes or Out.Depth for Depth.
	 * Deep-copies the pixels into Out; production code should prefer GetFrameShared
	 * / GetFrameForRequest, which hand back the shared retained frame by refcount.
	 */
	bool GetFrame(uint64 MinFrameId, FMjCameraFrame& Out) const;

	/** Shared-refcount variant of GetFrame: returns the retained frame without
	 *  copying its pixels. Null if no matching frame is retained. */
	TSharedPtr<const FMjCameraFrame> GetFrameShared(uint64 MinFrameId) const;

	/** Resolve the frame to return for an RPC request. When latency emulation is
	 *  active and bIgnoreDelay is false, returns the frame currently revealed by
	 *  the delay policy (so an RPC read agrees with what the stream is showing);
	 *  otherwise returns the frame at/after MinFrameId. Null if none available. */
	TSharedPtr<const FMjCameraFrame> GetFrameForRequest(uint64 MinFrameId, bool bIgnoreDelay) const;

	/** Most recent frame id retained in history (0 if none). */
	uint64 GetLatestFrameId() const;

	/** Select the newest history frame eligible at NowValue (clock chosen per
	 *  bDelayUseWallClock) whose Seq > AfterSeq. Returns false if none. Thread-
	 *  safe; public so automation tests can drive the selection directly. */
	bool SelectDelayedFrame(double NowValue, uint64 AfterSeq, FMjCameraFrame& Out) const;

	/** Push a completed frame into the history ring, evicting the oldest beyond
	 *  the retention window. Exposed (not a UFUNCTION) so automation tests can
	 *  drive the ring with synthetic frames without a live GPU. */
	void PushFrameToHistory(FMjCameraFrame&& Frame);

	/** Mark this camera as actively consumed right now (called when a client
	 *  requests it via include_cameras / get_frame). Per-camera capture gating
	 *  keeps a camera capturing only while broadcast-enabled or touched within
	 *  RequestActiveTtlSeconds, so idle cameras cost no GPU. Thread-safe. */
	void TouchRequested();

	/** True if this camera should be capturing right now: it has a streaming
	 *  broadcast enabled, or was requested within the active TTL. */
	bool IsCaptureActive() const;

	/**
	 * @brief Returns the ZMQ endpoint actually bound (may differ from ZmqEndpoint if auto-incremented).
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Camera")
	FString GetActualZmqEndpoint() const;

	/** Ordinal of this camera within its instance's camera port block. Assigned
	 *  by UMjNetworkManager at registration so each camera seeds a distinct port
	 *  (CamBasePort + index) upward from the instance's CamBasePort. */
	void SetStreamPortIndex(int32 InIndex) { StreamPortIndex = InIndex; }

	/**
	 * @brief Canonical transport identity for this camera: "<art>/<part>".
	 *
	 * Routed through FMjCanonicalName (the single naming owner): the art segment
	 * is the owning articulation's name (or the owning actor's name for a
	 * manager-level global camera) and the part is the MJCF name (or UE component
	 * name if unset) with the art prefix stripped, both sanitized to
	 * [A-Za-z0-9_]. This one string is the ZMQ topic, the hello handshake key +
	 * zmq_topic, the set_camera_streaming key, and the include_cameras lookup;
	 * the SHM filename is "cam_<art>_<part>.shm". UE writer and bridge reader
	 * apply the same scheme so both ends rendezvous on the same name.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Camera")
	FString GetCanonicalName() const;

	/**
	 * @brief Exports camera properties to a MuJoCo spec camera structure.
	 * @param cam Pointer to the target mjsCamera structure.
	 * @param def Optional default structure (unused, kept for API consistency).
	 */
	void ExportTo(mjsCamera* Element, mjsDefault* def = nullptr);

	/**
	 * @brief Imports properties from a MuJoCo XML <camera> node.
	 *        Reads: name, fovy, pos, quat, euler, xyaxes, zaxis,
	 *               width/height (MuJoCo 3.x resolution hints).
	 */
	void ImportFromXml(const class FXmlNode* Node, const struct FMjCompilerSettings& CompilerSettings);

protected:
	virtual void BeginPlay() override;
	virtual void OnRegister() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	// ---- Streaming setup helpers ----
	void SetupRenderTarget();
	void RegisterWithStreamingManager();

	/** Resolve the ZMQ endpoint this camera's PUB socket should bind from the
	 *  live instance config (BindAddress + CamBasePort + StreamPortIndex), so
	 *  cameras land in their instance's own port block. Falls back to the
	 *  authored ZmqEndpoint when no manager / bridge config is reachable. */
	FString ResolveStreamEndpoint() const;

	/** Ordinal within the instance's camera port block (see SetStreamPortIndex). */
	int32 StreamPortIndex = 0;

	/** Ensure `resolution` holds exactly two positive elements (defaults applied). */
	void NormalizeResolution();

	/** Issue a scene capture + readback for the current applied state when the
	 *  fps cap and state-change gate allow it. */
	void MaybeCapture(class AAMjManager* Mgr);

	/** With latency emulation on, publish the newest frame whose reveal time has
	 *  passed (each Seq once). No-op when no delay is configured. */
	void PublishDueDelayedFrames(class AAMjManager* Mgr);

	/** Refresh HiddenComponents from live seg pools so a late-starting seg camera
	 *  doesn't contaminate an already-streaming RGB/Depth capture. */
	void RefreshHiddenComponentsFromSegPools();

	/** Enqueue one async GPU->staging copy, stamping the frame with the id/time of
	 *  the applied state the pixels will show. Returns false without enqueuing when
	 *  the render target is not yet renderable (no RHI texture, or the in-flight cap
	 *  is reached), so a caller can keep retrying a cold camera. When bForceSubmit
	 *  is set (the on-demand sync path) the copy is dispatched to the RHI thread
	 *  immediately so its fence can signal mid-frame instead of at the next frame
	 *  boundary; the steady streaming path leaves it false to avoid the per-capture
	 *  submit cost. */
	bool EnqueueReadback(uint64 ShowFrameId, double ShowSimTime, bool bForceSubmit = false);

	/** Dispatch every GPU-ready readback to the render thread for map/copy (FIFO). */
	void DispatchReadyReadbacks();

	/** Drain the render thread's finished frames into history + recycle readbacks. */
	void DrainCompletedFrames();

	/** Newest revealed frame under the delay policy at NowValue, refcount-shared.
	 *  Null when nothing is eligible or (with AfterSeq) nothing new. */
	TSharedPtr<const FMjCameraFrame> SelectDelayedFrameShared(double NowValue, uint64 AfterSeq) const;

	/** Store a completed frame into the ring; shared so retention is a refcount
	 *  bump rather than a pixel deep-copy. */
	void PushFrameToHistoryShared(const TSharedPtr<const FMjCameraFrame>& Frame);

	/** Current delay-policy clock value (wall-clock or applied SimTime). */
	double NowClockValue() const;

	/** Sample an effective delay (seconds) from DelaySeconds +/- jitter via the
	 *  seeded RNG, clamped >= 0. */
	double SampleDelaySeconds();

	/** Clock value (SimTime or CaptureUnixTime) used for delay maths on Frame,
	 *  per bDelayUseWallClock. */
	double FrameClock(const FMjCameraFrame& Frame) const;

	/** Push one frame onto both streaming transports (ZMQ + SHM), reconstructing
	 *  the v2 wire meta. Shared by the no-delay (inline) and delayed publish. */
	void PublishFrameToWorkers(const FMjCameraFrame& Frame);

	/** True when latency emulation is configured (delay or jitter > 0). */
	bool IsDelayActive() const { return DelaySeconds > 0.0f || DelayJitterSeconds > 0.0f; }

	// ---- Async readback pipeline ----
	// GPU->CPU pixel readback uses FRHIGPUTextureReadback (async, non-stalling)
	// rather than a synchronous RHICmdList.ReadSurfaceData, so the readback rate
	// tracks the render rate instead of collapsing to 1/(GPU render time). The
	// game thread enqueues the copy and later, once IsReady(), dispatches a
	// render-thread command that maps + copies the staging buffer and pushes the
	// finished frame onto ResultsQueue. The game thread pops that queue next tick.
	// No render-thread flush is taken on either path, which structurally removes
	// the re-entrancy the earlier per-readback flush created.
	struct FInFlightReadback
	{
		TSharedPtr<FRHIGPUTextureReadback> Gpu;
		uint64 FrameId = 0;
		double SimTime = 0.0;
		int32 Width = 0;
		int32 Height = 0;
		// Unix-epoch seconds (FDateTime::UtcNow) at request time, stamped into the
		// streamed frame's meta so clients can measure content latency.
		double CaptureUnixSeconds = 0.0;
	};
	TArray<FInFlightReadback> InFlightReadbacks;
	static constexpr int32 MaxInFlightReadbacks = 3;

	// A frame whose GPU->staging map+copy has completed on the render thread,
	// carrying the readback object back for recycling on the game thread.
	struct FCompletedReadback
	{
		TSharedPtr<FMjCameraFrame> Frame;
		TSharedPtr<FRHIGPUTextureReadback> Gpu;
		bool bCopied = false;
	};
	using FCompletedReadbackQueue = TQueue<FCompletedReadback, EQueueMode::Spsc>;
	// Render thread produces, game thread consumes. Held by shared pointer so a
	// render command outliving the component (teardown race) references the queue,
	// not a destroyed UObject.
	TSharedPtr<FCompletedReadbackQueue, ESPMode::ThreadSafe> ResultsQueue;
	// Map/copy commands dispatched to the render thread but not yet drained. Only
	// the game thread touches it (dispatch increments, drain decrements).
	int32 PendingMapCommands = 0;
	// Guards against a nested drain if a render flush ever pumps the game thread
	// while a drain is in progress (the async design avoids such flushes, but the
	// guard keeps the single-consumer queue contract structurally safe).
	bool bDrainingResults = false;

	// Recycle pool for the readback objects. FRHIGPUTextureReadback owns a staging
	// texture, so reuse them (EnqueueCopy re-arms in place) rather than allocating
	// one per request.
	TArray<TSharedPtr<FRHIGPUTextureReadback>> FreeReadbacks;

	// ---- Frame history ring ----
	// Retains recent frames so a client can fetch the frame for a specific
	// post-step state (by FrameId) or the latest. HistoryLock serialises the game
	// thread (push) against the bridge worker thread (GetFrame). Oldest-first;
	// newest is Last(). Frames are shared + const so a fetch is a refcount bump.
	mutable FCriticalSection HistoryLock;
	TArray<TSharedPtr<const FMjCameraFrame>> History;
	// Absolute ceiling on retained frames, shared by the no-delay (fixed-capacity)
	// and delay (time-windowed) eviction paths so neither can grow past the
	// configured maximum. Must match the HistoryCapacity ClampMax meta above.
	static constexpr int32 MaxHistoryCapacity = 64;

	// ---- Camera latency emulation state ----
	// DelayRng seeds the per-frame jitter sample; HarvestSeq tags each harvested
	// frame so the streaming publish emits each delayed frame exactly once
	// (LastPublishedSeq is the last Seq sent). Touched only on the game thread.
	FRandomStream DelayRng;
	uint64 HarvestSeq = 0;
	uint64 LastPublishedSeq = 0;

	// ---- Capture-rate gating state ----
	// LastCapturedFrameId is the applied FrameId at the last capture; a capture
	// fires only when it changes (bCaptureOnStateChange). LastCaptureWallSeconds
	// backs the optional CaptureMaxFps cap. LastRenderedAppliedId is the applied
	// id whose render the every-frame RT currently shows, used to stamp a readback
	// in every-frame mode with the state its pixels actually contain (the RT lags
	// the game tick by one automatic capture).
	uint64 LastCapturedFrameId = 0;
	double LastCaptureWallSeconds = 0.0;
	uint64 LastRenderedAppliedId = 0;
	double LastRenderedAppliedTime = 0.0;

	// ---- Streaming state ----
	bool bStreamingEnabled = false;

	// A camera captures only while "active": a streaming broadcast is enabled, or
	// it was requested within RequestActiveTtlSeconds. LastRequestedSeconds is an
	// FPlatformTime::Seconds() stamp, stored atomically so TouchRequested is
	// callable from the bridge worker thread.
	std::atomic<double> LastRequestedSeconds{0.0};

	// ---- Transports ----
	FCameraZmqWorker* ZmqWorker = nullptr;
	FRunnableThread* WorkerThread = nullptr;
	// Forward-declared to keep the header light; full type pulled in by the cpp.
	class FCameraShmWriter* ShmWriter = nullptr;

	// Per-camera ROS `sensor_msgs/Image` publisher. Created alongside the ZMQ / SHM
	// sinks when streaming is enabled and a ROS context is live; parallel to them
	// and NOT part of the state fan-out. Null when ROS is unavailable.
	UrlabRclImagePub* RosImagePub = nullptr;

	/** Create / destroy the ROS image publisher. No-ops when ROS is unavailable
	 *  (feature off or no live context). */
	void SetupRosImagePublisher();
	void TeardownRosImagePublisher();
};
