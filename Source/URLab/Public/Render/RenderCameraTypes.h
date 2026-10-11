// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#pragma once

#include "CoreMinimal.h"
#include "Math/RandomStream.h"
#include "Render/RenderFrame.h"
#include "RenderCameraTypes.generated.h"

/** Pixel payload produced by a neutral render camera. Enum ordinals are stable on existing wires. */
UENUM(BlueprintType)
enum class ERenderCameraMode : uint8
{
	Real UMETA(DisplayName = "Photoreal RGB"),
	Depth UMETA(DisplayName = "Depth"),
	SemanticSegmentation UMETA(DisplayName = "Semantic Segmentation"),
	InstanceSegmentation UMETA(DisplayName = "Instance Segmentation"),
};

/** Stable neutral identity kind emitted in the second int32 segmentation lane. */
enum class ERenderSegmentationObjectKind : int32
{
	Geometry = 1,
	Body = 2,
};

inline bool RenderIsSegmentationMode(ERenderCameraMode Mode)
{
	return Mode == ERenderCameraMode::SemanticSegmentation
		|| Mode == ERenderCameraMode::InstanceSegmentation;
}

enum class ERenderCameraProjection : uint8
{
	Perspective,
	Orthographic,
};

/** Final render-ready optics. All lengths are UE centimetres or pixels as documented. */
struct FRenderCameraOptics
{
	FIntPoint Resolution = FIntPoint(640, 480);
	ERenderCameraProjection Projection = ERenderCameraProjection::Perspective;
	float VerticalFovDegrees = 45.0f;
	float OrthographicWidthCm = 0.0f;
	bool bHasIntrinsics = false;
	float FocalPixelsX = 0.0f;
	float FocalPixelsY = 0.0f;
	float PrincipalPixelsX = 320.0f;
	float PrincipalPixelsY = 240.0f;
	float NearClipCm = 2.0f;
	float FarClipCm = 10000.0f;
};

/** Author-selected runtime capture behavior, resolved before entering Render/. */
struct FRenderCameraCaptureSettings
{
	ERenderCameraMode Mode = ERenderCameraMode::Real;
	float DepthNearCm = 10.0f;
	float DepthFarCm = 10000.0f;
	bool bCaptureOnStateChange = true;
	float CaptureMaxFps = 0.0f;
	bool bManualCaptureOnly = false;
	int32 HistoryCapacity = 8;
	float RequestActiveTtlSeconds = 2.0f;
	float DelaySeconds = 0.0f;
	float DelayJitterSeconds = 0.0f;
	bool bDelayUseWallClock = false;
	int32 DelaySeed = 0;
	bool bRenderInMainRenderer = false;
	float StreamingBoost = 1.0f;
};

/** One compiled model camera. CanonicalName is sourced from compiled model metadata. */
struct FRenderCameraDesc
{
	FString CanonicalName;
	FRenderCameraOptics Optics;
	FRenderCameraCaptureSettings Capture;
};

/** One validated, allocation-neutral view of a camera's pixel configuration. */
struct URLAB_API FRenderCameraConfigurationSnapshot
{
	FString CanonicalName;
	ERenderCameraMode Mode = ERenderCameraMode::Real;
	FIntPoint Resolution = FIntPoint::ZeroValue;
	uint64 ConfigurationRevision = 0;
	int64 LogicalPixelBytes = 0;
	int64 PhysicalArrayBytes = 0;
};

/** Optional per-job changes resolved against one installed camera baseline. */
struct URLAB_API FRenderCameraCaptureOverride
{
	TOptional<ERenderCameraMode> Mode;
	TOptional<FIntPoint> Resolution;
	TOptional<FRenderCameraOptics> CompleteOptics;
	TOptional<float> DepthNearCm;
	TOptional<float> DepthFarCm;
};

struct URLAB_API FRenderCameraCaptureSelection
{
	FString CameraName;
	TOptional<FRenderCameraCaptureOverride> Override;
};

/** Complete allocation-neutral configuration pinned by an exact capture plan. */
struct URLAB_API FRenderCameraEffectiveConfiguration : FRenderCameraConfigurationSnapshot
{
	uint64 BaselineConfigurationRevision = 0;
	FRenderCameraOptics Optics;
	float DepthNearCm = 10.0f;
	float DepthFarCm = 10000.0f;
};

struct URLAB_API FRenderCameraOverrideToken
{
	bool IsValid() const { return Camera.IsValid() && Serial != 0; }
private:
	friend class URenderCameraComponent;
	TWeakObjectPtr<class URenderCameraComponent> Camera;
	uint64 Serial = 0;
};

/** Checked element and byte sizing for the neutral frame array used by Mode. */
URLAB_API bool RenderCheckedCameraBufferSizing(ERenderCameraMode Mode, FIntPoint Resolution,
	int32& OutElementCount, int64& OutLogicalBytes, int64& OutPhysicalBytes,
	FString& OutError);

/** Opaque workflow-owned capacity retained until an accepted GPU writer drains. */
class URLAB_API IRenderCameraCaptureLifetimeLease
{
public:
	virtual ~IRenderCameraCaptureLifetimeLease() = default;
};

using FRenderCameraCaptureLifetimeLeasePtr =
	TSharedPtr<const IRenderCameraCaptureLifetimeLease, ESPMode::ThreadSafe>;

/** Why an exact capture was or was not accepted on the game thread. */
enum class ERenderCameraCaptureIssue : uint8
{
	Accepted,
	RetryableNotReady,
	RetryableBusy,
	InvalidIdentity,
	InvalidConfiguration,
	ShuttingDown,
};

/** Exact capture state. Terminal outcome and GPU-writer drain are separate facts. */
enum class ERenderCameraCaptureOutcome : uint8
{
	Pending,
	Succeeded,
	MapFailed,
	IdentityChanged,
	ConfigurationChanged,
	Cancelled,
	Teardown,
};

struct FRenderCameraExactCaptureState;

struct URLAB_API FRenderCameraCaptureCompletion
{
	ERenderCameraCaptureOutcome Outcome = ERenderCameraCaptureOutcome::Pending;
	TSharedPtr<const struct FRenderCameraFrame> Frame;
	FString Message;
	bool bGpuWriterDrained = false;

	bool IsTerminal() const { return Outcome != ERenderCameraCaptureOutcome::Pending; }
	bool IsSuccessful() const { return Outcome == ERenderCameraCaptureOutcome::Succeeded; }
};

/** Immutable identity of one accepted camera-local GPU capture. */
struct URLAB_API FRenderCameraCaptureReceipt
{
	FString CameraName;
	FRenderFrameIdentity Identity;
	ERenderCameraMode Mode = ERenderCameraMode::Real;
	FIntPoint Resolution = FIntPoint::ZeroValue;
	uint64 ConfigurationRevision = 0;
	uint64 CaptureSerial = 0;

	bool IsValid() const;
	/** Thread-safe atomic drain query; does not access the GT-owned result frame. */
	bool IsGpuWriterDrained() const;
	/** Game-thread-only. The terminal result remains retained independently of history. */
	FRenderCameraCaptureCompletion Poll() const;
	/** Game-thread-only. Cancellation does not imply that the GPU writer has drained. */
	bool Cancel(const FString& Reason = TEXT("exact camera capture was cancelled")) const;

private:
	friend class URenderCameraComponent;
	TSharedPtr<FRenderCameraExactCaptureState, ESPMode::ThreadSafe> State;
};

struct URLAB_API FRenderCameraCaptureIssueResult
{
	ERenderCameraCaptureIssue Code = ERenderCameraCaptureIssue::RetryableNotReady;
	FRenderCameraCaptureReceipt Receipt;
	FString Message;

	bool IsAccepted() const { return Code == ERenderCameraCaptureIssue::Accepted; }
	bool IsRetryable() const
	{
		return Code == ERenderCameraCaptureIssue::RetryableNotReady
			|| Code == ERenderCameraCaptureIssue::RetryableBusy;
	}
};

/** One retained camera frame, tagged with the exact scene frame applied at capture. */
struct FRenderCameraFrame
{
	FRenderFrameIdentity Identity;
	ERenderCameraMode Mode = ERenderCameraMode::Real;
	uint64 CameraConfigurationRevision = 0;
	uint64 CaptureSerial = 0;
	int32 Width = 0;
	int32 Height = 0;
	TArray<FColor> Color;
	TArray<float> Depth;
	/** Two int32 values per pixel: explicit object id and producer-supplied object type. */
	TArray<int32> Seg;
	double CaptureUnixTime = 0.0;
	double RevealValue = 0.0;
	uint64 Seq = 0;
	double TIssuedSeconds = 0.0;
	double TFenceReadySeconds = 0.0;
	double TMappedSeconds = 0.0;
	double TPublishedSeconds = 0.0;
	/** Neutral capacity retained for the complete lifetime of exact-capture pixels. */
	FRenderCameraCaptureLifetimeLeasePtr PixelLifetimeLease;
};

using FRenderCameraFramePtr =
	TSharedPtr<const FRenderCameraFrame, ESPMode::ThreadSafe>;
using FMutableRenderCameraFramePtr =
	TSharedPtr<FRenderCameraFrame, ESPMode::ThreadSafe>;

struct URLAB_API FRenderCameraHistory
{
	void PushShared(const FRenderCameraFramePtr& Frame,
		int32 Capacity, bool bDelayActive, double RetainWindow, bool bUseWallClock);
	FRenderCameraFramePtr GetShared(uint64 MinFrameId) const;
	uint64 GetLatestFrameId() const;
	FRenderCameraFramePtr SelectDelayedShared(double NowValue, uint64 AfterSeq) const;
	FRenderCameraFramePtr SelectDelayedByClock(double TargetClock, bool bUseWallClock) const;
	void Clear();
	mutable FCriticalSection Lock;
	TArray<FRenderCameraFramePtr> Frames;
	static constexpr int32 MaxHistoryCapacity = 64;
};

struct URLAB_API FRenderCameraDelayModel
{
	double SampleDelaySeconds(float DelaySeconds, float JitterSeconds);
	void Configure(int32 Seed);
	static double FrameClock(const FRenderCameraFrame& Frame, bool bUseWallClock);
	static double NowClockValue(bool bUseWallClock, double AppliedSourceTime);
	static bool IsDelayActive(float DelaySeconds, float JitterSeconds);
	FRandomStream Rng;
	uint64 LastPublishedSeq = 0;
};
