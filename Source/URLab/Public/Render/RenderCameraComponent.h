// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Containers/Queue.h"
#include "Engine/TextureRenderTarget2D.h"
#include "RHIGPUReadback.h"
#include "Render/RenderCameraTypes.h"
#include <atomic>
#include "RenderCameraComponent.generated.h"

class UPrimitiveComponent;

URLAB_API void RenderPumpForcedCapture(const TArray<class URenderCameraComponent*>& Cameras, uint64 TargetFrameId, int32 TimeoutMs);
URLAB_API void RenderCopyReadbackIntoFrame(const void* Mapped, int32 RowPitchPixels,
    ERenderCameraMode Mode, int32 SegmentationObjectType, FRenderCameraFrame& Frame);
URLAB_API void RenderSpearForcedCapture(const TArray<class URenderCameraComponent*>& Cameras,
    const FRenderFrameIdentity& Identity);

UCLASS(ClassGroup = (URLabRender), meta = (BlueprintSpawnableComponent))
class URLAB_API URenderCameraComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	URenderCameraComponent();

	// --- Capture configuration --------------------------------------------- //

	/**
	 * What this camera captures.
	 *
	 * Read when the render target is BUILT, because it decides that target's
	 * pixel format (BGRA8 / R32f) and the capture source, so writing this field
	 * on a camera that is already streaming changes nothing on its own. Call
	 * ApplyCaptureMode instead, which does the rebuild.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Capture")
	ERenderCameraMode CaptureMode = ERenderCameraMode::Real;

	/**
	 * Put this camera into a capture mode and make it stick, now.
	 *
	 * Setting CaptureMode alone is not enough on a live camera: SetupRenderTarget
	 * is what reads it, and SetStreamingEnabled(true) only calls that when there
	 * is no target yet. So the change is applied by cycling streaming off and on,
	 * which drops the old target (SetStreamingEnabled(false) nulls RenderTarget
	 * precisely "so the next enable rebuilds it in the current mode"), releases or
	 * updates the explicit component visibility sets for the new target format.
	 *
	 * That cycle is not free -- it reallocates GPU resources and discards its
	 * delay-ring history -- so an unchanged mode returns immediately,
	 * before any of it. A client sending the same mode on every render pays
	 * nothing; one alternating modes pays the rebuild each time.
	 *
	 * Returns true when the mode changed. Game thread only, like the streaming
	 * calls it makes.
	 */
	bool ApplyCaptureMode(ERenderCameraMode Mode);

	/** How many components one pixel of a frame in this mode carries: 4 for BGRA
	 *  colour, 1 for depth, 2 for the segmentation (objid, objtype) pair. On the
	 *  wire as `cameras[].channels`, so a client never guesses a stride. */
	static int32 ChannelsForMode(ERenderCameraMode Mode);

	/** Near clip for Depth capture, centimetres. Nearer than this reads as 0. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Capture",
		meta = (EditCondition = "CaptureMode == ERenderCameraMode::Depth", ClampMin = "0.1"))
	float DepthNearCm = 10.0f;

	/** Far clip for Depth capture, centimetres. Beyond it reads as the maximum. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Capture",
		meta = (EditCondition = "CaptureMode == ERenderCameraMode::Depth", ClampMin = "1.0"))
	float DepthFarCm = 10000.0f;

	/**
	 * Capture and read back only when the applied render frame advances.
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
	 * Manual-capture-only: the per-frame pipeline harvests completed readbacks but
	 * never issues an automatic capture. Captures come solely from an on-demand
	 * IssueSyncCapture (the forced-render path). Used by the render server's
	 * forced-render mode so the async stream does not compete with the request
	 * render for the one render thread.
	 */
	bool bManualCaptureOnly = false;

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

	/** Host-controlled capture demand. Publishers set this while a continuous stream is active. */
	bool bContinuousCapture = false;

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
	 * Measure the delay in wall-clock rather than source time. Wall-clock suits real
	 * latency emulation in live mode; source time is deterministic for stepped runs.
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

	/** Set the exact installed-scene frame these pixels will show. */
	void SetAppliedFrameIdentity(const FRenderFrameIdentity& Identity);
	FRenderFrameIdentity GetAppliedFrameIdentity() const;

	/** Apply a complete validated render-ready descriptor. */
	bool Configure(const FRenderCameraDesc& Desc, FString& OutError);
	/** Copy the complete validated pixel configuration. Game thread only. */
	bool GetConfigurationSnapshot(FRenderCameraConfigurationSnapshot& OutSnapshot,
		FString& OutError) const;
	bool ResolveCaptureConfiguration(const TOptional<FRenderCameraCaptureOverride>& Override,
		FRenderCameraEffectiveConfiguration& OutConfiguration, FString& OutError) const;
	bool BeginCaptureOverride(const FRenderCameraEffectiveConfiguration& Configuration,
		FRenderCameraOverrideToken& OutToken, FString& OutError);
	bool RestoreCaptureOverride(const FRenderCameraOverrideToken& Token, FString& OutError);
	const FRenderCameraDesc& GetDescriptor() const { return Descriptor; }
	const FRenderCameraDesc& GetRuntimeDescriptor() const { return RuntimeDescriptor; }
	const FString& GetCanonicalName() const { return Descriptor.CanonicalName; }
	int32 GetSegmentationObjectType() const { return SegmentationObjectType; }

	/** Explicit component sets supplied by the scene host; the camera performs no discovery. */
	void SetVisibilityComponents(const TArray<UPrimitiveComponent*>& ShowOnly,
		const TArray<UPrimitiveComponent*>& Hidden, int32 InSegmentationObjectType);
	/** Install immutable neutral-scene subjects selected automatically by capture mode. */
	void SetNeutralSegmentationComponents(const TArray<UPrimitiveComponent*>& Instance,
		const TArray<UPrimitiveComponent*>& Semantic);
	void SetSceneVisibility(bool bVisible);

	DECLARE_MULTICAST_DELEGATE_OneParam(FOnFrameCompleted, const FRenderCameraFrame&);
	FOnFrameCompleted OnFrameCompleted;
	DECLARE_MULTICAST_DELEGATE_OneParam(FOnSharedFrameCompleted,
		const FRenderCameraFramePtr&);
	/** Thread-safe immutable completion for nonblocking stream consumers. */
	FOnSharedFrameCompleted OnSharedFrameCompleted;

	/** Near clipping distance in cm, or <= 0 for default / CVar value. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Rendering")
	float CustomNearClipCm = 0.0f;

	/** Far clipping distance in cm, or <= 0 for default. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Camera|Rendering")
	float CustomFarClipCm = 0.0f;

	/** Recompute and apply the custom projection matrix from authored or derived intrinsics. */
	UFUNCTION(BlueprintCallable, Category = "URLab|RenderCamera")
	void SetupProjectionMatrix();

	/** Set the final render-ready near and far clipping planes in centimetres. */
	void SetClippingPlanes(float InNearClipCm, float InFarClipCm);

	/**
	 * Allocate the render target and begin capturing, or stop and give the
	 * capture budget back.
	 */
	UFUNCTION(BlueprintCallable, Category = "URLab|RenderCamera")
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
	 * Returns the dimensions from the complete validated neutral descriptor.
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
	 * whose reveal time has come. Driven by URenderCameraSubsystem once per frame
	 * rather than by a per-component tick.
	 */
	void UpdateCapturePipeline(bool bAllowCaptureIssue = true);

	/**
	 * Enqueue a non-blocking GPU-to-CPU readback for the current applied state.
	 * A no-op when streaming is off or the in-flight cap is reached. Completed
	 * readbacks land in the history ring, tagged with the state they show.
	 */
	UFUNCTION(BlueprintCallable, Category = "URLab|RenderCamera")
	void RequestReadback();

	/**
	 * Render on demand: capture the scene and enqueue a readback right now, for
	 * the current applied state. Requires streaming to be enabled already. Pair
	 * it with HarvestCompletedReadbacks to pull the finished frame into history.
	 */
	void IssueSyncCapture();

	/**
	 * Try to issue one exact non-blocking capture of the currently applied frame.
	 * Accepted receipts are never reissued. The optional neutral lease remains
	 * retained until the associated GPU map/copy writer has actually drained,
	 * which may be later than cancellation or teardown becomes terminal.
	 */
	FRenderCameraCaptureIssueResult TryIssueExactCapture(
		FRenderCameraCaptureLifetimeLeasePtr LifetimeLease = {});

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
	bool HasPendingReadbacks() const
	{
		return InFlightReadbacks.Num() > 0 || PendingMapCommands > 0;
	}

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
	bool GetFrame(uint64 MinFrameId, FRenderCameraFrame& Out) const;

	/** GetFrame without the pixel copy. Null when nothing matching is retained. */
	FRenderCameraFramePtr GetFrameShared(uint64 MinFrameId) const;

	/**
	 * The frame to answer an RPC request with.
	 *
	 * While latency emulation is active an RPC read must see what the stream is
	 * currently showing, not the undelayed newest frame, so the two agree;
	 * bIgnoreDelay opts a caller out when it wants the freshest rendered truth.
	 */
	FRenderCameraFramePtr GetFrameForRequest(uint64 MinFrameId, bool bIgnoreDelay) const;

	/**
	 * The frame to answer an RPC request with an explicit delay (in seconds).
	 * If InDelaySeconds > 0.0, selects the newest frame eligible at (NowClock - InDelaySeconds).
	 * If InDelaySeconds <= 0.0, falls back to standard request routing (camera-configured delay or fresh).
	 */
	FRenderCameraFramePtr GetFrameForRequest(uint64 MinFrameId, double InDelaySeconds) const;

	/** Clear all retained history frames (e.g. on episode reset). Thread-safe. */
	void ClearHistory();

	/** The most recent frame id retained, or 0. */
	uint64 GetLatestFrameId() const;

	/**
	 * The newest history frame eligible at NowValue whose Seq is beyond
	 * AfterSeq, on whichever clock bDelayUseWallClock selects. Thread-safe, and
	 * public so automation can drive the selection directly.
	 */
	bool SelectDelayedFrame(double NowValue, uint64 AfterSeq, FRenderCameraFrame& Out) const;

	/**
	 * Push a completed frame into the ring, evicting past the retention window.
	 * Public so automation can drive the ring with synthetic frames on a machine
	 * with no GPU.
	 */
	void PushFrameToHistory(FRenderCameraFrame&& Frame);

	/** True when this identity may be captured under the configured delay clock. */
	bool CanCaptureIdentity(const FRenderFrameIdentity& Identity) const;

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


protected:
	virtual void OnRegister() override;
	virtual void OnUnregister() override;
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	// --- Streaming setup ------------------------------------------------------- //

	void SetupRenderTarget();
	void RegisterWithStreamingManager();

	/** Turn off every renderer feature that would blend or brighten a segmentation
	 *  pixel. An id image is not a picture: a blended edge decodes to a plausible
	 *  id belonging to no object. Called from SetupRenderTarget for seg modes. */
	void ApplySegmentationShowFlags();
	void ApplyModeVisibilityComponents();


	/** Current render-frame identity. Only changed after scene/frame validation. */
	FRenderFrameIdentity AppliedIdentity;
	/** Serializes scene/epoch transitions with completion acceptance and history reads. */
	mutable FCriticalSection IdentityGateLock;
	FRenderCameraDesc Descriptor;
	FRenderCameraDesc RuntimeDescriptor;
	struct FActiveCaptureOverride
	{
		uint64 Serial = 0;
		FRenderCameraDesc SavedRuntimeDescriptor;
	};
	TOptional<FActiveCaptureOverride> ActiveCaptureOverride;
	uint64 NextOverrideSerial = 1;
	bool bCaptureOverridePoisoned = false;
	bool ApplyRuntimeCaptureConfiguration(const FRenderCameraEffectiveConfiguration& Configuration,
		FString& OutError);
	int32 SegmentationObjectType = -1;
	UPROPERTY(Transient)
	TArray<TObjectPtr<UPrimitiveComponent>> NeutralInstanceComponents;
	UPROPERTY(Transient)
	TArray<TObjectPtr<UPrimitiveComponent>> NeutralSemanticComponents;
	UPROPERTY(Transient)
	TArray<TObjectPtr<UPrimitiveComponent>> ExplicitShowOnlyComponents;
	UPROPERTY(Transient)
	TArray<TObjectPtr<UPrimitiveComponent>> ExplicitHiddenComponents;
	uint64 ConfigurationRevision = 1;
	uint64 NextCaptureSerial = 1;
	bool bEndingPlay = false;

	/** Issue a capture and readback when the fps cap and the state-change gate allow. */
	void MaybeCapture();

	/** With latency emulation on, publish the newest revealed frame, each Seq once. */
	void PublishDueDelayedFrames();

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
	bool EnqueueReadback(const FRenderFrameIdentity& ShowIdentity, bool bForceSubmit = false,
		FRenderCameraCaptureReceipt* OutExactReceipt = nullptr,
		ERenderCameraCaptureIssue* OutIssue = nullptr,
		FRenderCameraCaptureLifetimeLeasePtr LifetimeLease = {});
	void AdvanceConfigurationRevision();
	void InvalidateExactCaptures(ERenderCameraCaptureOutcome Outcome,
		const FString& Message);
	void PruneDrainedExactCaptures();

	/** Dispatch every GPU-ready readback to the render thread for map and copy, FIFO. */
	void DispatchReadyReadbacks();

	/** Drain the render thread's finished frames into history and recycle the readbacks. */
	void DrainCompletedFrames();

	struct FInFlightReadback
	{
		TSharedPtr<FRHIGPUTextureReadback, ESPMode::ThreadSafe> Gpu;
		FRenderFrameIdentity Identity;
		uint64 IdentityGeneration = 0;
		ERenderCameraMode Mode = ERenderCameraMode::Real;
		uint64 ConfigurationRevision = 0;
		uint64 CaptureSerial = 0;
		int32 SegmentationObjectType = -1;
		int32 Width = 0;
		int32 Height = 0;
		/** Unix-epoch seconds at request time, stamped into the wire meta. */
		double CaptureUnixSeconds = 0.0;
		/** Monotonic stamps for copy-out stage timing (urlab.Cam.Diag). */
		double TIssuedSeconds = 0.0;
		double TFenceReadySeconds = 0.0;
		TSharedPtr<FRenderCameraExactCaptureState, ESPMode::ThreadSafe> ExactState;
	};
	TArray<FInFlightReadback> InFlightReadbacks;
	TArray<TSharedPtr<FRenderCameraExactCaptureState, ESPMode::ThreadSafe>> ExactCaptures;
	TSharedPtr<std::atomic<int32>, ESPMode::ThreadSafe> DetachedWriterCount;
	static constexpr int32 MaxInFlightReadbacks = 3;
	void DispatchReadbackMap(FInFlightReadback&& Readback);

	// --- Copy-out stage-timing accumulators (urlab.Cam.Diag) -------------------
	// Game-thread only. Summed per harvested frame and logged every
	// DiagWindowFrames, so enabling diagnostics costs a few adds per frame and
	// one log line per window, never a per-frame spam.
	void AccumulateStageTiming(const FRenderCameraFrame& Frame);
	static constexpr int32 DiagWindowFrames = 60;
	int32 DiagFrames = 0;
	double DiagSumIssueToFence = 0.0;
	double DiagSumFenceToMap = 0.0;
	double DiagSumMapToPublish = 0.0;
	double DiagSumTotal = 0.0;
	double DiagMaxMapToPublish = 0.0;
	double DiagMaxTotal = 0.0;

	/** A frame whose map and copy finished, carrying its readback back for recycling. */
	struct FCompletedReadback
	{
		FMutableRenderCameraFramePtr Frame;
		TSharedPtr<FRHIGPUTextureReadback, ESPMode::ThreadSafe> Gpu;
		uint64 IdentityGeneration = 0;
		TSharedPtr<FRenderCameraExactCaptureState, ESPMode::ThreadSafe> ExactState;
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
	TArray<TSharedPtr<FRHIGPUTextureReadback, ESPMode::ThreadSafe>> FreeReadbacks;

	// --- Frame history ring ------------------------------------------------------ //

	/** The retained recent frames, with the lock that guards them. */
	FRenderCameraHistory History;

	/** Store a completed frame in the ring; shared, so retention is a refcount bump. */
	void PushFrameToHistoryShared(const FRenderCameraFramePtr& Frame,
		uint64 ExpectedIdentityGeneration = 0);
	bool IsIdentityGenerationCurrent(uint64 ExpectedIdentityGeneration) const;

	/** SelectDelayedFrame without the pixel copy. */
	FRenderCameraFramePtr SelectDelayedFrameShared(double NowValue, uint64 AfterSeq) const;

	// --- Latency emulation state --------------------------------------------------- //

	/** The seeded jitter RNG and the delayed-publish dedup. */
	FRenderCameraDelayModel Delay;

	/** The current delay-policy clock value: wall-clock, or the applied source time. */
	double NowClockValue() const;

	/** Draw an effective delay from DelaySeconds plus jitter, clamped at zero. */
	double SampleDelaySeconds();

	/** The clock value delay maths uses for Frame, per bDelayUseWallClock. */
	double FrameClock(const FRenderCameraFrame& Frame) const;

	/** True when latency emulation is configured at all. */
	bool IsDelayActive() const;

	/** Emit a completed or delay-revealed frame to host-side publishers. */
	void EmitFrame(const FRenderCameraFramePtr& Frame);

	/**
	 * Tags each harvested frame; incremented as frames drain from the readback
	 * pipeline. Game thread only.
	 */
	uint64 HarvestSeq = 0;

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
	bool bSceneVisible = false;

	/**
	 * An FPlatformTime::Seconds() stamp of the last request, atomic because
	 * TouchRequested is callable from the bridge worker thread.
	 */
	std::atomic<double> LastRequestedSeconds{0.0};
	/** Incremented when the installed scene/epoch changes so late GPU work is discarded. */
	std::atomic<uint64> IdentityGeneration{1};


};
