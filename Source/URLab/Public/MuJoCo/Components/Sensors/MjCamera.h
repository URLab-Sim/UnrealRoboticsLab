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

	void PushFrame(const TArray<FColor>& FrameData);
	void PushFrame(const TArray<float>& FrameData);
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
	// Two queues -- one per pixel format. Real / seg cameras drive the
	// FColor queue, depth cameras drive the float queue. The Run() loop
	// drains both and ships whatever it finds. Per-camera CaptureMode
	// never changes after streaming starts, so only one queue is ever
	// active per worker instance.
	TQueue<TArray<FColor>, EQueueMode::Spsc> FrameQueue;
	TQueue<TArray<float>, EQueueMode::Spsc> FloatFrameQueue;
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

	/** Refresh HiddenComponents from live seg pools so a late-starting seg
	 *  camera doesn't contaminate an already-streaming RGB/Depth capture. */
	void RefreshHiddenComponentsFromSegPools();

	/** Push a completed frame into the history ring under HistoryLock, evicting
	 *  the oldest beyond HistoryCapacity. Exposed (not UFUNCTION) so automation
	 *  tests can drive the ring with synthetic frames without a live GPU. */
public:
	void PushFrameToHistory(FMjCameraFrame&& Frame);

private:

	// ---- Readback state ----
	// CaptureMode picks which Pending buffer is used: BGRA8 (Real / SemSeg /
	// InstanceSeg) drives PendingPixels, Depth drives PendingFloatPixels.
	// Only one mode is in flight at a time.
	//
	// Pending* is the render-thread destination. RequestReadback Emplaces a
	// fresh TArray, SetNumUninitialized's it, and enqueues a render command
	// capturing a pointer into it. The render thread writes to that pointer.
	// Only the game thread touches Pending* (RequestReadback, gated by
	// !bReadbackPending, and TickComponent's fence-complete handler), so the
	// captured pointer stays valid until the fence completes.
	//
	// On fence completion TickComponent feeds the streaming workers a copy and
	// MOVES the frame into the History ring (below), tagged with the post-step
	// FrameId/SimTime that was applied when the readback was issued.
	TOptional<TArray<FColor>> PendingPixels;
	TOptional<TArray<float>> PendingFloatPixels;
	FRenderCommandFence ReadbackFence;
	bool bReadbackPending = false;

	// Post-step state id + sim time captured at RequestReadback time (game
	// thread), carried through to the harvested frame so consumers can ask
	// "the frame for step N".
	uint64 PendingFrameId = 0;
	double PendingSimTime = 0.0;

	// ---- Frame history ring ----
	// Retains the last HistoryCapacity frames so a client can fetch the frame
	// for a specific post-step state (by FrameId) or the latest. HistoryLock
	// serialises between the game thread (push) and the bridge worker thread
	// (GetFrame). Oldest-first; newest is Last().
	mutable FCriticalSection HistoryLock;
	TArray<FMjCameraFrame> History;

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
