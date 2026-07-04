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
#include "HAL/ThreadSafeBool.h"
#include "Containers/Queue.h"
#include "MuJoCo/Components/Sensors/MjCameraTypes.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "RHIGPUReadback.h"
#include "Math/RandomStream.h"
#include <atomic>
#include "MjCamera.generated.h"

/**
 * @class FCameraZmqWorker
 * @brief Background thread for publishing high-bandwidth camera frames via ZeroMQ.
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
	FString GetBoundEndpoint() const { return BoundEndpoint; }

	/** Process-wide pause gate. Workers drain without sending while set,
	 *  to bound RT memory in Direct/Puppet mode. */
	static URLAB_API std::atomic<bool> bPublishersPaused;

private:
	FString RequestedEndpoint;
	FString BoundEndpoint;
	FString Topic;
	FIntPoint resolution;

	void* ZmqContext = nullptr;
	void* ZmqPublisher = nullptr;

	FThreadSafeBool bStopThread;

	// Each queued frame carries its metadata header so the Run() loop can
	// prepend it to the published bytes (the client associates the streamed
	// frame with the step that produced it via Meta.FrameId).
	struct FQueuedColorFrame
	{
		FMjCameraFrameMeta Meta;
		TArray<FColor> Pixels;
	};
	struct FQueuedFloatFrame
	{
		FMjCameraFrameMeta Meta;
		TArray<float> Pixels;
	};

	// Two queues -- one per pixel format. Real / seg cameras drive the
	// FColor queue, depth cameras drive the float queue. The Run() loop
	// drains both and ships whatever it finds. Per-camera CaptureMode
	// never changes after streaming starts, so only one queue is ever
	// active per worker instance.
	TQueue<FQueuedColorFrame, EQueueMode::Spsc> FrameQueue;
	TQueue<FQueuedFloatFrame, EQueueMode::Spsc> FloatFrameQueue;
};

/**
 * @struct FMjCameraFrame
 * @brief One retained camera frame, tagged with the post-step physics state it
 *        shows. CaptureMode decides which pixel buffer is populated.
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
 *  - No ExportTo / RegisterToSpec — camera is UE-side only, not fed back to MuJoCo.
 *  - SetStreamingEnabled() allocates the RT and calls IStreamingManager::AddViewInformation
 *    so textures load correctly even when the player pawn is far away.
 *  - The per-tick readback is decoupled from stepping: completed frames land
 *    in a history ring tagged with the post-step FrameId/SimTime they show.
 *    Consumers fetch by step (GetFrame(frameId)) or latest (GetFrame(0)).
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

	UMjCamera();

	/** What this camera captures. Read at SetStreamingEnabled(true) time —
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

	// ---- Streaming ----

	/** @brief Boost factor passed to IStreamingManager::AddViewInformation.
	 *  Increase to force higher-quality texture mips near this camera. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera|Streaming")
	float StreamingBoost = 1.0f;

	// ---- Capture Components ----

	/** @brief The underlying SceneCaptureComponent2D. Capture is disabled by default. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Camera")
	USceneCaptureComponent2D* CaptureComponent;

	/** @brief The render target. Null until SetStreamingEnabled(true) is first called. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Camera")
	UTextureRenderTarget2D* RenderTarget = nullptr;

	// ---- ZeroMQ Streaming ----

	/** @brief If true, the camera will automatically broadcast its frames over ZeroMQ when streaming is enabled. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera|Network")
	bool bEnableZmqBroadcast = false;

	/** @brief The ZMQ Endpoint for this specific camera (e.g., tcp://0.0.0.0:5558). Must be unique per camera. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera|Network")
	FString ZmqEndpoint = TEXT("tcp://0.0.0.0:5558");

	// ---- Shared-memory streaming ----

	/** @brief If true, the camera also writes each frame into a per-camera
	 *  SHM region (`<Saved>/URLabShm/<session>/cam_<owner>_<name>.shm`). The
	 *  ZMQ broadcast is unaffected -- both transports can run in parallel. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera|Network")
	bool bEnableShmBroadcast = false;

	// ---- Camera latency emulation + capture-rate control ----

	/** Simulated camera latency (seconds). The streamed / served frame is the
	 *  newest whose reveal time <= now, where reveal = capture_clock +
	 *  sampled_delay. 0 = no delay (frames published as soon as harvested, the
	 *  legacy path). Measured in SimTime by default, or wall-clock when
	 *  bDelayUseWallClock is set. */
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

	/** Configure latency emulation at runtime (RPC-driven; applied on the game
	 *  thread alongside the per-tick capture, so no extra locking is needed).
	 *  Seed 0 derives a stable seed from the canonical name. */
	void SetCameraDelay(float InDelaySeconds, float InJitterSeconds, bool bInUseWallClock, int32 InSeed);

	/** Configure capture-rate control at runtime. */
	void SetCaptureRate(bool bInOnStateChange, float InMaxFps);

	/** Select the newest history frame eligible at NowValue (clock chosen per
	 *  bDelayUseWallClock) whose Seq > AfterSeq. Returns false if none. Thread-
	 *  safe; public so automation tests can drive the selection directly. */
	bool SelectDelayedFrame(double NowValue, uint64 AfterSeq, FMjCameraFrame& Out) const;

	// ---- Public API ----

	/**
	 * @brief Allocates the render target and begins streaming / scene capture.
	 *        Call with bEnable=false to stop rendering and free the capture budget.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Camera")
	void SetStreamingEnabled(bool bEnable);

	/** @brief Returns true once SetStreamingEnabled(true) has set up the
	 *  RT and the capture component. Cheap, lock-free read intended for
	 *  best-effort gating from the bridge worker thread (a stale read is
	 *  benign — at worst we marshal an extra idempotent
	 *  SetStreamingEnabled to the game thread). */
	bool IsStreamingActive() const { return bStreamingEnabled; }

	/**
	 * @brief Enqueues a non-blocking asynchronous GPU→CPU pixel readback.
	 *        No-op if streaming is not enabled or a readback is already in
	 *        flight. Completed readbacks land in the frame-history ring
	 *        (see GetFrame), tagged with the post-step state id they show.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Camera")
	void RequestReadback();

	/**
	 * @brief Fetch a frame from this camera's history ring (thread-safe).
	 *
	 * MinFrameId == 0 returns the most recent retained frame. Otherwise returns
	 * the oldest retained frame whose FrameId >= MinFrameId — i.e. the frame
	 * showing the state at/after that step. Returns false if no matching frame
	 * is retained yet (caller can retry on a later step). Fills Out.Color for
	 * Real/seg modes or Out.Depth for Depth; FrameId / SimTime identify which
	 * post-step state the frame shows.
	 */
	bool GetFrame(uint64 MinFrameId, FMjCameraFrame& Out) const;

	/** Most recent frame id retained in history (0 if none). */
	uint64 GetLatestFrameId() const;

	/** Mark this camera as actively consumed right now (called when a client
	 *  requests it via include_cameras / get_frame). Per-camera capture gating
	 *  keeps a camera capturing only while it is broadcast-enabled or has been
	 *  touched within RequestActiveTtlSeconds, so idle cameras cost no GPU.
	 *  Thread-safe (atomic store); safe to call from the bridge worker. */
	void TouchRequested();

	/** True if this camera should be capturing right now: it has a streaming
	 *  broadcast enabled, or was requested within the active TTL. */
	bool IsCaptureActive() const;

	/**
	 * @brief Returns the ZMQ endpoint actually bound (may differ from ZmqEndpoint if auto-incremented).
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Camera")
	FString GetActualZmqEndpoint() const;

	/**
	 * @brief Canonical transport identity for this camera.
	 *
	 * The MJCF name (GetMjName(), the identity the rest of the handshake keys
	 * on) with path separators collapsed to '_', falling back to the UE
	 * component name if MjName is unset. Body-qualified MJCF cameras carry a
	 * '/' (e.g. "left_arm/wrist_camera") which is illegal in a filename, so
	 * this single sanitized string is what every transport site uses: the SHM
	 * filename, the ZMQ topic leaf, the hello handshake key, and the
	 * include_cameras lookup. UE writer and bridge reader must apply the same
	 * rule so both ends rendezvous on the same name.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Camera")
	FString GetCanonicalName() const;

	/**
	 * @brief Returns the bound camera component pointer (for UI wiring).
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Camera")
	UMjCamera* GetSelf() { return this; }

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
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;
	virtual void OnRegister() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	// ---- Internal helpers ----
	void SetupRenderTarget();
	void RegisterWithStreamingManager();

	/** Drain completed async readbacks (FIFO) into pixel frames: publish
	 *  inline when no delay is configured and push each to the history ring.
	 *  Stops at the first not-ready entry. Reusable by the tick and by any
	 *  synchronous capture path that flushes the render thread first. */
	void HarvestCompletedReadbacks();

	/** Issue a scene capture + readback for the current applied state when the
	 *  fps cap and state-change gate allow it. */
	void MaybeCapture(class AAMjManager* Mgr);

	/** With latency emulation on, publish the newest frame whose reveal time
	 *  has passed (each Seq once). No-op when no delay is configured. */
	void PublishDueDelayedFrames(class AAMjManager* Mgr);

	/** Refresh HiddenComponents from live seg pools so a late-starting seg
	 *  camera doesn't contaminate an already-streaming RGB/Depth capture. */
	void RefreshHiddenComponentsFromSegPools();

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

	/** Push a completed frame into the history ring under HistoryLock, evicting
	 *  the oldest beyond HistoryCapacity. Exposed (not UFUNCTION) so automation
	 *  tests can drive the ring with synthetic frames without a live GPU. */
public:
	void PushFrameToHistory(FMjCameraFrame&& Frame);

private:

	// ---- Async readback pipeline ----
	// GPU->CPU pixel readback uses FRHIGPUTextureReadback (async, non-stalling)
	// rather than a synchronous RHICmdList.ReadSurfaceData. The sync read forces
	// a render-thread flush + GPU wait, so its rate collapses to 1/(GPU render
	// time) on a heavy scene -- the camera feed throttles to ~10fps even though
	// the editor renders at realtime. EnqueueCopy schedules the copy without
	// blocking; we poll IsReady() and pipeline several deep so the readback rate
	// tracks the render rate. Each entry carries the post-step FrameId/SimTime
	// applied when it was issued, so consumers can still ask for "the frame for
	// step N". FIFO: harvested oldest-first into the History ring (below).
	struct FInFlightReadback
	{
		TUniquePtr<FRHIGPUTextureReadback> Gpu;
		uint64 FrameId = 0;
		double SimTime = 0.0;
		int32 Width = 0;
		int32 Height = 0;
		// Game-thread wall clock (FPlatformTime::Seconds) when this readback was
		// requested. Diagnostic: latency = harvest time - this, i.e. how long
		// the GPU copy took to become ready -- the readback-pipeline delay.
		double EnqueueSeconds = 0.0;
		// Unix-epoch seconds (FDateTime::UtcNow) at request time, stamped into
		// the streamed frame's meta so clients can measure content latency.
		double CaptureUnixSeconds = 0.0;
	};
	TArray<FInFlightReadback> InFlightReadbacks;
	static constexpr int32 MaxInFlightReadbacks = 3;

	// Diagnostic accumulators for the readback enqueue->ready latency log (~1/s).
	double ReadbackLatencyAccumMs = 0.0;
	double ReadbackLatencyMaxMs = 0.0;
	int32 ReadbackLatencySamples = 0;
	double ReadbackLatencyLastLogSec = 0.0;

	// ---- Frame history ring ----
	// Retains the last HistoryCapacity frames so a client can fetch the frame
	// for a specific post-step state (by FrameId) or the latest. HistoryLock
	// serialises between the game thread (push) and the bridge worker thread
	// (GetFrame). Oldest-first; newest is Last().
	mutable FCriticalSection HistoryLock;
	TArray<FMjCameraFrame> History;

	// ---- Camera latency emulation state ----
	// DelayRng seeds the per-frame jitter sample; HarvestSeq tags each harvested
	// frame so the streaming publish emits each delayed frame exactly once
	// (LastPublishedSeq is the last Seq sent). All touched only on the game
	// thread (harvest + RPC apply marshalled to GameThread).
	FRandomStream DelayRng;
	uint64 HarvestSeq = 0;
	uint64 LastPublishedSeq = 0;

	// ---- Capture-rate gating state ----
	// LastCapturedFrameId is the applied FrameId at the last capture; a capture
	// fires only when it changes (bCaptureOnStateChange). LastCaptureWallSeconds
	// backs the optional CaptureMaxFps cap.
	uint64 LastCapturedFrameId = 0;
	double LastCaptureWallSeconds = 0.0;

public:
	/** How many recent frames to retain for by-id retrieval. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera", meta = (ClampMin = "1", ClampMax = "64"))
	int32 HistoryCapacity = 8;

private:

	// ---- Streaming state ----
	bool bStreamingEnabled = false;

	// ---- Per-camera capture gating ----
	// A camera captures only while "active": a streaming broadcast is enabled,
	// or it was requested within RequestActiveTtlSeconds. LastRequestedSeconds
	// is an FPlatformTime::Seconds() stamp, stored atomically so TouchRequested
	// is callable from the bridge worker thread. TickComponent gates the
	// per-tick capture/readback on IsCaptureActive() so idle cameras cost no
	// GPU even when many are registered.
	std::atomic<double> LastRequestedSeconds{0.0};

public:
	/** How long after a request a camera keeps capturing before going dormant. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera", meta = (ClampMin = "0.0"))
	float RequestActiveTtlSeconds = 2.0f;

private:

	// ---- ZMQ Worker ----
	FCameraZmqWorker* ZmqWorker = nullptr;
	FRunnableThread* WorkerThread = nullptr;

	// ---- SHM Writer ----
	// Forward-declared to keep the header light; full type pulled in by the cpp.
	class FCameraShmWriter* ShmWriter = nullptr;
};
