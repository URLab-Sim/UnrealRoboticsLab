// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Everything a <camera> needs that is not one of its schema attributes.
//
// The generated base owns fovy, resolution, the intrinsics family, pose and
// mode; those are spec, they round-trip, and nothing here re-declares one.
// What lives here is the other half of a camera: the render target it draws
// into, the scene capture component that drives it, the async GPU readback
// pipeline, the history ring a client fetches frames from, the latency
// emulation, and the two streaming transports. All of it is per-instance state
// the reflection system has to see or the render thread has to reach, which is
// exactly the criterion that justifies a subclass over a function library.
//
// The camera is cheap until asked for: no render target and no GPU cost exist
// until SetStreamingEnabled(true), and a camera that is neither broadcasting
// nor recently requested stops capturing again on its own.

#include "CoreMinimal.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Containers/Queue.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Math/RandomStream.h"
#include "RHIGPUReadback.h"

#include "MuJoCo/Capture/MjCameraTypes.h"
#include "MuJoCo/Gen/Elements/Cameras/MjCamera.gen.h"

#include <atomic>

#include "MjCamera.generated.h"

class AAMjManager;
class FCameraShmWriter;
class FRunnableThread;

/**
 * The ZMQ publisher thread, defined in MjCamera.cpp.
 *
 * Nothing outside the capture pipeline constructs one, so its queues and its
 * libzmq handles stay in that translation unit and this header stays free of
 * both.
 */
class FCameraZmqPublisher;

/**
 * The process-wide publish gate.
 *
 * Direct and Puppet stepping pause every camera publisher for the duration of a
 * step burst, so a run of renders cannot pile frames up in render-thread memory
 * faster than a subscriber drains them, and the step handlers clear it again
 * when they hand control back. One camera's publisher knows nothing about
 * another's, so the gate is a process-wide flag rather than per-component
 * state, and it keeps this spelling because `FCameraZmqWorker::bPublishersPaused`
 * is what the step RPC handlers already write.
 */
class FCameraZmqWorker
{
public:
	/** Publishers drain their queues without sending while this is set. */
	static URLAB_API std::atomic<bool> bPublishersPaused;
};

/**
 * One retained camera frame, tagged with the post-step physics state it shows.
 *
 * CaptureMode decides which pixel buffer is populated. Frames are retained in
 * the history ring as `TSharedPtr<const FMjCameraFrame>`, so retention, the RPC
 * fetch and the streaming publish all share one allocation by refcount rather
 * than deep-copying the multi-MB pixel array.
 */
struct FMjCameraFrame
{
	/** The post-step render-snapshot id these pixels show. */
	uint64 FrameId = 0;
	double SimTime = 0.0;
	int32 Width = 0;
	int32 Height = 0;

	/** Real and segmentation modes (BGRA8). */
	TArray<FColor> Color;

	/** Depth mode (float32). */
	TArray<float> Depth;

	/**
	 * Unix-epoch capture time, stamped when the readback was requested.
	 *
	 * Carried on the frame so a delayed re-publish rebuilds the wire meta with
	 * the ORIGINAL capture time: the client's content-age maths then reflects
	 * the injected latency rather than the moment the bytes were re-sent.
	 */
	double CaptureUnixTime = 0.0;

	/**
	 * The clock value at which this frame becomes eligible to publish, i.e.
	 * capture_clock + sampled_delay, on whichever clock bDelayUseWallClock
	 * selects.
	 */
	double RevealValue = 0.0;

	/**
	 * Monotonic per-harvest counter, so each delayed frame publishes exactly
	 * once. FrameId cannot serve: it repeats across intra-step captures.
	 */
	uint64 Seq = 0;
};

/**
 * A `<camera>` as an observation device: scene capture, readback, streaming.
 *
 * The GPU readback is fully asynchronous and decoupled from stepping. The
 * render thread maps and copies each finished readback and pushes the pixels
 * onto a results queue; the game thread pops them next tick into a history ring
 * tagged with the post-step FrameId and SimTime they show. No render-thread
 * flush is taken on the steady-state or the synchronous path; teardown is the
 * one place a flush is correct.
 */
UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjCamera : public UMjCameraBase
{
	GENERATED_BODY()

public:
	UMjCamera();

	// --- Capture configuration --------------------------------------------- //

	/**
	 * What this camera captures.
	 *
	 * Read when streaming is enabled, because it decides the render target's
	 * pixel format and the capture source; to change it on a running camera,
	 * toggle streaming off and on.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Capture")
	EMjCameraMode CaptureMode = EMjCameraMode::Real;

	/** Near clip for Depth capture, centimetres. Nearer than this reads as 0. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Capture",
		meta = (EditCondition = "CaptureMode == EMjCameraMode::Depth", ClampMin = "0.1"))
	float DepthNearCm = 10.0f;

	/** Far clip for Depth capture, centimetres. Beyond it reads as the maximum. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Capture",
		meta = (EditCondition = "CaptureMode == EMjCameraMode::Depth", ClampMin = "1.0"))
	float DepthFarCm = 10000.0f;

	/**
	 * Capture and read back only when the applied physics state advances.
	 *
	 * Between steps the world is unchanged, so a re-render plus GPU readback is
	 * wasted work. A no-op in live mode, where the state advances every frame; a
	 * large saving while stepping, and zero capture cost while paused. Clear it
	 * to force a capture every engine frame.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Capture")
	bool bCaptureOnStateChange = true;

	/**
	 * Hard cap on capture rate in wall-clock frames per second; 0 is uncapped.
	 * Applied on top of bCaptureOnStateChange, for a consumer that needs fewer
	 * frames than the sim emits.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Capture", meta = (ClampMin = "0.0"))
	float CaptureMaxFps = 0.0f;

	/**
	 * How many recent frames to retain for by-id retrieval. The ClampMax mirrors
	 * MaxHistoryCapacity, the ceiling both eviction paths enforce; the two are
	 * one limit and must stay in step.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Capture", meta = (ClampMin = "1", ClampMax = "64"))
	int32 HistoryCapacity = 8;

	/** How long after a request a camera keeps capturing before going dormant. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Capture", meta = (ClampMin = "0.0"))
	float RequestActiveTtlSeconds = 2.0f;

	// --- Rendering ---------------------------------------------------------- //

	/** The scene capture doing the work. Dormant until streaming is enabled. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera|Rendering")
	TObjectPtr<USceneCaptureComponent2D> CaptureComponent;

	/** The render target. Null until SetStreamingEnabled(true) is first called. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera|Rendering")
	TObjectPtr<UTextureRenderTarget2D> RenderTarget;

	/**
	 * The vertical field of view this camera actually renders at, degrees.
	 *
	 * Derived, never authored: an MJCF camera specified by intrinsics carries no
	 * `fovy` attribute, and writing one back would flip it from
	 * intrinsics-specified to fov-specified and change what the file means. An
	 * explicit `fovy` wins whenever it is set; otherwise this is what the
	 * focal/sensor pair implies. RefreshCaptureFov recomputes it.
	 */
	UPROPERTY(VisibleAnywhere, Transient, Category = "Camera|Rendering")
	float DerivedFovy = 45.0f;

	/**
	 * Render this capture as nested passes of the main renderer, sharing its
	 * visibility and GPU-scene setup instead of taking a standalone scene
	 * render. Cheaper with many live cameras, but it renders on the main render
	 * cadence, so it is NOT compatible with the render:sync fast path, which
	 * captures on demand. Enable it only for pure live-streaming cameras that
	 * are never requested synchronously.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Rendering")
	bool bRenderInMainRenderer = false;

	/**
	 * Boost passed to IStreamingManager::AddViewInformation. Raise it to force
	 * higher-quality texture mips near this camera.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Rendering")
	float StreamingBoost = 1.0f;

	// --- Transports --------------------------------------------------------- //

	/** Broadcast frames over ZeroMQ while streaming is enabled. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Network")
	bool bEnableZmqBroadcast = false;

	/**
	 * The endpoint this camera's PUB socket binds.
	 *
	 * Resolved when streaming is enabled from the instance's camera port block
	 * (BindAddress + CamBasePort + StreamPortIndex), so N farm instances on
	 * distinct blocks never collide. The authored value is only the fallback for
	 * when no instance config is reachable, e.g. an isolated component test. The
	 * port is still auto-incremented on a bind conflict.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Network")
	FString ZmqEndpoint = TEXT("tcp://0.0.0.0:5558");

	/**
	 * Also write each frame into a per-camera SHM region,
	 * `<Saved>/URLabShm/<session>/cam_<art>_<part>.shm`. Independent of the ZMQ
	 * broadcast: both transports can run at once. The session segment is
	 * currently the literal "live", so two editor processes on one host collide
	 * on the same file.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Network")
	bool bEnableShmBroadcast = false;

	// --- Latency emulation --------------------------------------------------- //

	/**
	 * Simulated camera latency, seconds. The streamed and served frame is the
	 * newest whose reveal time has passed, where reveal = capture_clock +
	 * sampled_delay. 0 publishes frames as soon as they are harvested.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Delay", meta = (ClampMin = "0.0"))
	float DelaySeconds = 0.0f;

	/**
	 * Symmetric uniform jitter half-range, seconds: the per-frame delay is
	 * U(DelaySeconds - this, DelaySeconds + this), clamped at zero. Drawn from a
	 * seeded per-camera RNG, so jittered latency reproduces across runs.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Delay", meta = (ClampMin = "0.0"))
	float DelayJitterSeconds = 0.0f;

	/**
	 * Measure the delay in wall-clock rather than SimTime. Wall-clock suits real
	 * latency emulation in live mode; SimTime is deterministic for stepped runs.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Delay")
	bool bDelayUseWallClock = false;

	// --- Runtime configuration ----------------------------------------------- //

	/**
	 * Configure latency emulation at runtime. Applied on the game thread
	 * alongside the per-tick capture, so it needs no extra locking. Seed 0
	 * derives a stable seed from the canonical name.
	 */
	void SetCameraDelay(float InDelaySeconds, float InJitterSeconds, bool bInUseWallClock, int32 InSeed);

	/** Configure capture-rate control at runtime. */
	void SetCaptureRate(bool bInOnStateChange, float InMaxFps);

	/**
	 * Allocate the render target and begin capturing, or stop and give the
	 * capture budget back.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Camera")
	void SetStreamingEnabled(bool bEnable);

	/**
	 * True once SetStreamingEnabled(true) has set up the render target and the
	 * capture component. A cheap lock-free read, intended for best-effort gating
	 * from the bridge worker thread where a stale answer is benign.
	 */
	bool IsStreamingActive() const { return bStreamingEnabled; }

	/**
	 * The capture resolution as a validated {width, height} pair.
	 *
	 * Substitutes 640x480 for a missing or non-positive element, so a malformed
	 * `resolution` can never size a render target or index a pixel buffer out of
	 * bounds. Named for what it produces rather than the attribute it reads: the
	 * generated GetResolution hands back the raw schema array.
	 */
	FIntPoint CaptureResolution() const;

	/**
	 * Recompute DerivedFovy from the spec and push it at the capture
	 * component. Call after anything writes fovy, the intrinsics or the
	 * resolution; enabling streaming and registering already do.
	 */
	void RefreshCaptureFov();

	// --- Capture pipeline ---------------------------------------------------- //

	/**
	 * One per-frame capture pass: capture gating, advancing the async readback
	 * pipeline, and, while active, issuing a capture and publishing any frame
	 * whose reveal time has come. Driven by UMjCameraSubsystem once per frame
	 * rather than by a per-component tick.
	 */
	void UpdateCapturePipeline();

	/**
	 * Enqueue a non-blocking GPU-to-CPU readback for the current applied state.
	 * A no-op when streaming is off or the in-flight cap is reached. Completed
	 * readbacks land in the history ring, tagged with the state they show.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Camera")
	void RequestReadback();

	/**
	 * Render on demand: capture the scene and enqueue a readback right now, for
	 * the current applied state. Requires streaming to be enabled already. Pair
	 * it with HarvestCompletedReadbacks to pull the finished frame into history.
	 */
	void IssueSyncCapture();

	/**
	 * Advance the async readback pipeline: dispatch every GPU-ready readback to
	 * the render thread for its map and copy, then drain the render thread's
	 * finished frames into the history ring, publishing inline when no delay is
	 * configured. FIFO, and takes no render-thread flush.
	 */
	void HarvestCompletedReadbacks();

	/**
	 * Poll the pipeline for up to TimeoutSeconds, yielding between passes so the
	 * render thread can finish its map and copy. This is how the synchronous
	 * render path gets a fresh frame without flushing the render thread.
	 */
	void WaitAndHarvestReadbacks(double TimeoutSeconds);

	/** Readbacks awaiting GPU completion, excluding those already dispatched. */
	int32 NumInFlightReadbacks() const { return InFlightReadbacks.Num(); }

	/**
	 * True while any readback is still awaiting the GPU or its finished frame is
	 * still queued for harvest, so the synchronous render path can avoid
	 * re-issuing a capture over one already in flight.
	 */
	bool HasPendingReadbacks() const { return InFlightReadbacks.Num() > 0 || PendingMapCommands > 0; }

	// --- Frame history -------------------------------------------------------- //

	/**
	 * Fetch a frame from the history ring, thread-safe.
	 *
	 * MinFrameId 0 returns the most recent retained frame; otherwise the oldest
	 * retained frame whose FrameId is at least MinFrameId, i.e. the frame
	 * showing the state at or after that step. False when nothing matching is
	 * retained yet. This deep-copies the pixels: production callers should
	 * prefer GetFrameShared or GetFrameForRequest, which hand back the retained
	 * frame by refcount.
	 */
	bool GetFrame(uint64 MinFrameId, FMjCameraFrame& Out) const;

	/** GetFrame without the pixel copy. Null when nothing matching is retained. */
	TSharedPtr<const FMjCameraFrame> GetFrameShared(uint64 MinFrameId) const;

	/**
	 * The frame to answer an RPC request with.
	 *
	 * While latency emulation is active an RPC read must see what the stream is
	 * currently showing, not the undelayed newest frame, so the two agree;
	 * bIgnoreDelay opts a caller out when it wants the freshest rendered truth.
	 */
	TSharedPtr<const FMjCameraFrame> GetFrameForRequest(uint64 MinFrameId, bool bIgnoreDelay) const;

	/** The most recent frame id retained, or 0. */
	uint64 GetLatestFrameId() const;

	/**
	 * The newest history frame eligible at NowValue whose Seq is beyond
	 * AfterSeq, on whichever clock bDelayUseWallClock selects. Thread-safe, and
	 * public so automation can drive the selection directly.
	 */
	bool SelectDelayedFrame(double NowValue, uint64 AfterSeq, FMjCameraFrame& Out) const;

	/**
	 * Push a completed frame into the ring, evicting past the retention window.
	 * Public so automation can drive the ring with synthetic frames on a machine
	 * with no GPU.
	 */
	void PushFrameToHistory(FMjCameraFrame&& Frame);

	// --- Per-camera capture gating -------------------------------------------- //

	/**
	 * Mark this camera as actively consumed right now, i.e. a client asked for
	 * it through include_cameras or get_frame. A camera captures only while
	 * broadcast-enabled or touched within RequestActiveTtlSeconds, so idle
	 * cameras cost no GPU. Thread-safe.
	 */
	void TouchRequested();

	/** True when this camera should be capturing: broadcasting, or recently asked for. */
	bool IsCaptureActive() const;

	// --- Transport identity ---------------------------------------------------- //

	/** The endpoint actually bound, which differs from ZmqEndpoint after an auto-increment. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Camera")
	FString GetActualZmqEndpoint() const;

	/**
	 * This camera's ordinal within its instance's camera port block, assigned by
	 * UMjNetworkManager at registration so each camera seeds a distinct port
	 * upward from the instance's CamBasePort.
	 */
	void SetStreamPortIndex(int32 InIndex) { StreamPortIndex = InIndex; }

	/**
	 * Canonical transport identity, "<art>/<part>".
	 *
	 * Routed through FMjCanonicalName, the single naming owner: the art segment
	 * is the owning articulation's name, or the owning actor's name for a
	 * manager-level global camera, and the part is the MJCF name (or the
	 * component name when unset) with the art prefix stripped, both sanitized to
	 * [A-Za-z0-9_]. This one string is the ZMQ topic, the hello handshake key
	 * and zmq_topic, the set_camera_streaming key and the include_cameras
	 * lookup; the SHM filename is "cam_<art>_<part>.shm". UE writer and bridge
	 * reader derive it the same way, so both ends rendezvous on one name.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Camera")
	FString GetCanonicalName() const;

protected:
	virtual void OnRegister() override;
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	// --- Streaming setup ------------------------------------------------------- //

	void SetupRenderTarget();
	void RegisterWithStreamingManager();

	/**
	 * The endpoint this camera's PUB socket should bind, taken from the live
	 * instance config so cameras land in their own instance's port block. Falls
	 * back to the authored ZmqEndpoint when no manager or bridge is reachable.
	 */
	FString ResolveStreamEndpoint() const;

	/** Ordinal within the instance's camera port block. */
	int32 StreamPortIndex = 0;

	/**
	 * Refresh HiddenComponents from the live segmentation pools, so a
	 * late-starting seg camera cannot contaminate an already-streaming RGB or
	 * depth capture.
	 */
	void RefreshHiddenComponentsFromSegPools();

	/** Issue a capture and readback when the fps cap and the state-change gate allow. */
	void MaybeCapture(AAMjManager* Mgr);

	/** With latency emulation on, publish the newest revealed frame, each Seq once. */
	void PublishDueDelayedFrames(AAMjManager* Mgr);

	// --- Async readback pipeline ------------------------------------------------ //
	//
	// The readback uses FRHIGPUTextureReadback rather than a synchronous
	// RHICmdList.ReadSurfaceData, so the readback rate tracks the render rate
	// instead of collapsing to 1/(GPU render time). The game thread enqueues the
	// copy and later, once IsReady(), dispatches a render-thread command that
	// maps and copies the staging buffer and pushes the finished frame onto
	// ResultsQueue; the game thread pops that queue next tick. Neither path
	// takes a render-thread flush, which structurally removes the re-entrancy an
	// earlier per-readback flush created.

	/**
	 * Enqueue one async GPU-to-staging copy, stamped with the id and time of the
	 * applied state its pixels will show.
	 *
	 * Returns false without enqueuing while the render target is not renderable
	 * yet (no RHI texture) or the in-flight cap is reached, so a caller can keep
	 * retrying a cold camera. bForceSubmit dispatches the copy to the RHI thread
	 * immediately so its fence can signal mid-frame instead of at the next frame
	 * boundary; the steady streaming path leaves it clear to avoid paying the
	 * submit cost per capture.
	 */
	bool EnqueueReadback(uint64 ShowFrameId, double ShowSimTime, bool bForceSubmit = false);

	/** Dispatch every GPU-ready readback to the render thread for map and copy, FIFO. */
	void DispatchReadyReadbacks();

	/** Drain the render thread's finished frames into history and recycle the readbacks. */
	void DrainCompletedFrames();

	struct FInFlightReadback
	{
		TSharedPtr<FRHIGPUTextureReadback> Gpu;
		uint64 FrameId = 0;
		double SimTime = 0.0;
		int32 Width = 0;
		int32 Height = 0;
		/** Unix-epoch seconds at request time, stamped into the wire meta. */
		double CaptureUnixSeconds = 0.0;
	};
	TArray<FInFlightReadback> InFlightReadbacks;
	static constexpr int32 MaxInFlightReadbacks = 3;

	/** A frame whose map and copy finished, carrying its readback back for recycling. */
	struct FCompletedReadback
	{
		TSharedPtr<FMjCameraFrame> Frame;
		TSharedPtr<FRHIGPUTextureReadback> Gpu;
		bool bCopied = false;
	};
	using FCompletedReadbackQueue = TQueue<FCompletedReadback, EQueueMode::Spsc>;

	/**
	 * Render thread produces, game thread consumes. Held by shared pointer so a
	 * render command that outlives the component on a teardown race references
	 * the queue rather than a destroyed UObject.
	 */
	TSharedPtr<FCompletedReadbackQueue, ESPMode::ThreadSafe> ResultsQueue;

	/** Map commands dispatched but not yet drained. Game thread only. */
	int32 PendingMapCommands = 0;

	/**
	 * Guards against a nested drain if a render flush ever pumped the game
	 * thread mid-drain. The async design avoids such flushes; the guard keeps
	 * the single-consumer queue contract structurally safe anyway.
	 */
	bool bDrainingResults = false;

	/**
	 * Recycle pool. FRHIGPUTextureReadback owns a staging texture, so reuse them
	 * (EnqueueCopy re-arms in place) rather than allocating one per request.
	 */
	TArray<TSharedPtr<FRHIGPUTextureReadback>> FreeReadbacks;

	// --- Frame history ring ------------------------------------------------------ //
	//
	// Retains recent frames so a client can fetch the one for a specific
	// post-step state, or the latest. HistoryLock serialises the game thread's
	// push against the bridge worker thread's fetch. Oldest first, newest last,
	// shared and const so a fetch is a refcount bump.

	mutable FCriticalSection HistoryLock;
	TArray<TSharedPtr<const FMjCameraFrame>> History;

	/**
	 * The absolute ceiling on retained frames, shared by the fixed-capacity and
	 * the time-windowed eviction paths so neither can grow past it. Must match
	 * HistoryCapacity's ClampMax.
	 */
	static constexpr int32 MaxHistoryCapacity = 64;

	/** Store a completed frame in the ring; shared, so retention is a refcount bump. */
	void PushFrameToHistoryShared(const TSharedPtr<const FMjCameraFrame>& Frame);

	/** SelectDelayedFrame without the pixel copy. */
	TSharedPtr<const FMjCameraFrame> SelectDelayedFrameShared(double NowValue, uint64 AfterSeq) const;

	// --- Latency emulation state --------------------------------------------------- //

	/** The current delay-policy clock value: wall-clock, or the applied SimTime. */
	double NowClockValue() const;

	/** Draw an effective delay from DelaySeconds plus jitter, clamped at zero. */
	double SampleDelaySeconds();

	/** The clock value delay maths uses for Frame, per bDelayUseWallClock. */
	double FrameClock(const FMjCameraFrame& Frame) const;

	/** True when latency emulation is configured at all. */
	bool IsDelayActive() const { return DelaySeconds > 0.0f || DelayJitterSeconds > 0.0f; }

	/**
	 * Push one frame onto the streaming transports and broadcast it on
	 * FMjCameraFrameBus for any out-of-core image sink, rebuilding the wire
	 * meta. Shared by the inline publish and the delayed one.
	 */
	void PublishFrameToWorkers(const FMjCameraFrame& Frame);

	/**
	 * DelayRng seeds the per-frame jitter draw; HarvestSeq tags each harvested
	 * frame and LastPublishedSeq is the last one sent, so the delayed publish
	 * emits each frame exactly once. Game thread only.
	 */
	FRandomStream DelayRng;
	uint64 HarvestSeq = 0;
	uint64 LastPublishedSeq = 0;

	// --- Capture-rate gating state --------------------------------------------------- //
	//
	// LastCapturedFrameId is the applied id at the last capture; a capture fires
	// only once it changes. LastCaptureWallSeconds backs the fps cap.
	// LastRenderedAppliedId is the applied id whose render a main-renderer
	// target currently shows, used to stamp a readback with the state its pixels
	// actually contain, because that target lags the game tick by one automatic
	// capture.

	uint64 LastCapturedFrameId = 0;
	double LastCaptureWallSeconds = 0.0;
	uint64 LastRenderedAppliedId = 0;
	double LastRenderedAppliedTime = 0.0;

	// --- Streaming state ---------------------------------------------------------------- //

	bool bStreamingEnabled = false;

	/**
	 * An FPlatformTime::Seconds() stamp of the last request, atomic because
	 * TouchRequested is callable from the bridge worker thread.
	 */
	std::atomic<double> LastRequestedSeconds{0.0};

	FCameraZmqPublisher* ZmqPublisher = nullptr;
	FRunnableThread* PublisherThread = nullptr;
	FCameraShmWriter* ShmWriter = nullptr;
};
