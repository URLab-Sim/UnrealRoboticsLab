// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "Render/RenderCameraComponent.h"

#include "ContentStreaming.h"
#include "Containers/ContainerAllocationPolicies.h"
#include "Engine/Engine.h"
#include "Engine/PostProcessVolume.h"
#include "EngineUtils.h"
#include "HAL/IConsoleManager.h"
#include "Misc/CoreDelegates.h"
#include "RHICommandList.h"
#include "RenderingThread.h"

#include "Utils/URLabLogging.h"
#include "Render/RenderCameraSubsystem.h"

bool RenderCheckedCameraBufferSizing(ERenderCameraMode Mode, FIntPoint Resolution,
	int32& OutElementCount, int64& OutLogicalBytes, int64& OutPhysicalBytes,
	FString& OutError)
{
	OutElementCount = 0;
	OutLogicalBytes = 0;
	OutPhysicalBytes = 0;
	OutError.Reset();
	if (Resolution.X <= 0 || Resolution.Y <= 0)
	{
		OutError = TEXT("camera resolution must be positive");
		return false;
	}

	int64 ElementsPerPixel = 0;
	int64 BytesPerElement = 0;
	uint32 ElementAlignment = 0;
	switch (Mode)
	{
	case ERenderCameraMode::Real:
		ElementsPerPixel = 1;
		BytesPerElement = sizeof(FColor);
		ElementAlignment = alignof(FColor);
		break;
	case ERenderCameraMode::Depth:
		ElementsPerPixel = 1;
		BytesPerElement = sizeof(float);
		ElementAlignment = alignof(float);
		break;
	case ERenderCameraMode::SemanticSegmentation:
	case ERenderCameraMode::InstanceSegmentation:
		ElementsPerPixel = 2;
		BytesPerElement = sizeof(int32);
		ElementAlignment = alignof(int32);
		break;
	default:
		OutError = TEXT("camera mode is unknown");
		return false;
	}

	const int64 Width = Resolution.X;
	const int64 Height = Resolution.Y;
	if (Width > MAX_int64 / Height)
	{
		OutError = TEXT("camera pixel count overflows int64");
		return false;
	}
	const int64 PixelCount = Width * Height;
	if (PixelCount > MAX_int64 / ElementsPerPixel)
	{
		OutError = TEXT("camera element count overflows int64");
		return false;
	}
	const int64 ElementCount = PixelCount * ElementsPerPixel;
	if (ElementCount > MAX_int32 || ElementCount > MAX_int64 / BytesPerElement)
	{
		OutError = TEXT("camera frame does not fit a TArray");
		return false;
	}

	OutElementCount = static_cast<int32>(ElementCount);
	OutLogicalBytes = ElementCount * BytesPerElement;
	const int32 ReservedElements = DefaultCalculateSlackReserve<int32>(
		OutElementCount, static_cast<SIZE_T>(BytesPerElement), true, ElementAlignment);
	if (ReservedElements < OutElementCount
		|| static_cast<int64>(ReservedElements) > MAX_int64 / BytesPerElement)
	{
		OutElementCount = 0;
		OutLogicalBytes = 0;
		OutError = TEXT("camera frame allocator reserve is invalid");
		return false;
	}
	OutPhysicalBytes = static_cast<int64>(ReservedElements) * BytesPerElement;
	return true;
}

namespace
{
/**
 * The descriptor's fov is VERTICAL; USceneCaptureComponent2D's
 * FOVAngle is the HORIZONTAL one. Copying fovy across verbatim over-narrows the
 * view on a non-square target, which reads as "zoomed in", so convert through
 * the target's aspect and UE's derived vertical fov matches fovy exactly.
 * Identity on a square target.
 */
float HorizontalFOVFromFovy(float Fovy, FIntPoint Resolution)
{
	// A camera with no fovy produces a degenerate zero-fov frustum that renders
	// pure black. Fall back to the neutral descriptor default vertical fov.
	if (Fovy <= 0.0f)
	{
		Fovy = 45.0f;
	}
	if (Resolution.X <= 0 || Resolution.Y <= 0)
	{
		return Fovy;
	}
	const float Aspect = static_cast<float>(Resolution.X) / static_cast<float>(Resolution.Y);
	const float FovyRad = FMath::DegreesToRadians(Fovy);
	return FMath::RadiansToDegrees(2.0f * FMath::Atan(FMath::Tan(FovyRad * 0.5f) * Aspect));
}

/**
 * The vertical fov this camera should render at, degrees.
 *
 * The adapter has already resolved authored fov or intrinsics into final render
 * optics. Keep the derivation here defensive for direct neutral producers.
 */
float DeriveFovyDegrees(const URenderCameraComponent& Cam)
{
	const float Value = Cam.GetRuntimeDescriptor().Optics.VerticalFovDegrees;
	return FMath::IsFinite(Value) && Value > 0.0f ? Value : 45.0f;
}

/** What to call a camera in a log line: its descriptor name, or its component name. */
// Copy-out stage timing. When set, URenderCameraComponent decomposes each harvested frame's
// GPU-to-game-thread path (issue -> fence-ready -> mapped -> published) and logs
// a windowed summary. Off by default: a few adds per frame, no wire cost.
static TAutoConsoleVariable<int32> CVarCamDiag(
	TEXT("urlab.Cam.Diag"), 0,
	TEXT("Log camera copy-out stage timing (issue/fence/map/publish). 0=off, 1=on."),
	ECVF_Default);

// Force-submit streaming readback copies to the RHI thread so their fence signals
// in-frame instead of quantizing to the next frame boundary. Targets the dominant
// issue->fence stage of the copy-out. Possible render-thread throughput cost (one
// ImmediateFlush per capture), so it is opt-in and A/B'd. The sync path already
// force-submits; this extends it to the streaming path.
static TAutoConsoleVariable<int32> CVarCamForceSubmit(
	TEXT("urlab.Cam.ForceSubmit"), 0,
	TEXT("Force-submit streaming readback copies so the fence signals in-frame. "
		 "0=off (default), 1=on."),
	ECVF_Default);

// Near clipping plane for scene capture components (in cm, 1 UU = 1 cm).
// Default is 2.0 cm (20 mm).
static TAutoConsoleVariable<float> CVarCamNearClip(
	TEXT("urlab.Cam.NearClip"), 2.0f,
	TEXT("Near clipping plane for camera scene captures in cm (default 2.0 cm = 20 mm)."),
	ECVF_Default);

// Report what a segmentation readback's raw lanes held, per frame. An id image
// that comes back all-background has several indistinguishable causes from the
// client's side -- nothing rendered, the value was scaled out of the 0..255 lane
// range, the wrong surface was read -- and the raw range separates them.
static TAutoConsoleVariable<int32> CVarCamSegDiag(
	TEXT("urlab.Cam.SegDiag"), 0,
	TEXT("Log the raw lane range and covered-pixel count of every segmentation readback."),
	ECVF_Default);

// Which capture source a segmentation camera reads. 0 = SceneColorHDR (the one
// that carries an unlit emissive unmodified). Kept as a knob because the answer
// to "why is my id image blank" is almost always a different one of these.
static TAutoConsoleVariable<int32> CVarCamSegSource(
	TEXT("urlab.Cam.SegSource"), 0,
	TEXT("Segmentation capture source: 0=SceneColorHDR 1=FinalColorLDR 2=BaseColor ")
	TEXT("3=FinalToneCurveHDR 4=SceneColorHDRNoAlpha."),
	ECVF_Default);

// Whether a segmentation camera applies the no-blending, no-brightening show
// flags. 0 leaves the level's own flags, for isolating which of them matters.
static TAutoConsoleVariable<int32> CVarCamSegShowFlags(
	TEXT("urlab.Cam.SegShowFlags"), 1,
	TEXT("Apply the segmentation show-flag block (1, default) or leave the level's flags (0)."),
	ECVF_Default);

// Disable autoexposure (eye adaptation) for camera scene captures. 1=disabled, 0=enabled (default).
static TAutoConsoleVariable<int32> CVarCamDisableAutoExposure(
	TEXT("urlab.Cam.DisableAutoExposure"), 0,
	TEXT("Disable eye adaptation / autoexposure for camera scene captures. 1=disabled, 0=enabled (default)."),
	ECVF_Default);

FString CameraLogName(const URenderCameraComponent& Cam)
{
	return Cam.GetCanonicalName().IsEmpty() ? Cam.GetName() : Cam.GetCanonicalName();
}

} // namespace

struct FRenderCameraExactCaptureState
{
	ERenderCameraCaptureOutcome Outcome = ERenderCameraCaptureOutcome::Pending;
	FRenderCameraFramePtr Frame;
	FString Message;
	FRenderCameraCaptureLifetimeLeasePtr LifetimeLease;
	std::atomic<bool> bDiscardPixels{false};
	std::atomic<bool> bGpuWriterDrained{false};
};

namespace
{
void RetireExactCaptureWriter(
	const TSharedPtr<FRenderCameraExactCaptureState, ESPMode::ThreadSafe>& State)
{
	if (!State.IsValid()) return;
	State->LifetimeLease.Reset();
	State->bGpuWriterDrained.store(true, std::memory_order_release);
}

/**
 * Readbacks detached by component teardown. The render-thread end-frame hook
 * polls fences without blocking either thread, maps only once ready, and owns
 * every GPU object and exact lifetime lease until that writer has drained.
 */
class FDetachedCameraReadbackDrain
{
private:
	struct FEntry
	{
		TSharedPtr<FRHIGPUTextureReadback, ESPMode::ThreadSafe> Gpu;
		TSharedPtr<FRenderCameraExactCaptureState, ESPMode::ThreadSafe> ExactState;
		TSharedPtr<std::atomic<int32>, ESPMode::ThreadSafe> WriterCount;
	};

public:
	static FDetachedCameraReadbackDrain& Get()
	{
		// Process-lifetime RT owner. Deliberately never destroyed: a static
		// destructor cannot prove that an outstanding GPU fence has drained, and
		// therefore must not release its lease or report a false terminal drain.
		static FDetachedCameraReadbackDrain* Instance =
			new FDetachedCameraReadbackDrain();
		return *Instance;
	}

	void Retain(TSharedPtr<FRHIGPUTextureReadback, ESPMode::ThreadSafe> Gpu,
		TSharedPtr<FRenderCameraExactCaptureState, ESPMode::ThreadSafe> ExactState,
		TSharedPtr<std::atomic<int32>, ESPMode::ThreadSafe> WriterCount)
	{
		if (!Gpu.IsValid())
		{
			RetireExactCaptureWriter(ExactState);
			return;
		}
		if (WriterCount.IsValid())
			WriterCount->fetch_add(1, std::memory_order_relaxed);
		ENQUEUE_RENDER_COMMAND(URLabRetainDetachedCameraReadback)
		([this, Gpu = MoveTemp(Gpu), ExactState = MoveTemp(ExactState),
			WriterCount = MoveTemp(WriterCount)](FRHICommandListImmediate&) mutable
		{
			Entries.Add({MoveTemp(Gpu), MoveTemp(ExactState), MoveTemp(WriterCount)});
			if (!EndFrameHandle.IsValid())
			{
				EndFrameHandle = FCoreDelegates::OnEndFrameRT.AddRaw(
					this, &FDetachedCameraReadbackDrain::DrainReadyOnRenderThread);
			}
		});
	}

private:
	void DrainReadyOnRenderThread()
	{
		TArray<FEntry> Ready;
		for (int32 Index = Entries.Num() - 1; Index >= 0; --Index)
		{
			if (!Entries[Index].Gpu.IsValid() || Entries[Index].Gpu->IsReady())
			{
				Ready.Add(MoveTemp(Entries[Index]));
				Entries.RemoveAtSwap(Index, 1, EAllowShrinking::No);
			}
		}
		if (Entries.IsEmpty())
		{
			FCoreDelegates::OnEndFrameRT.Remove(EndFrameHandle);
			EndFrameHandle.Reset();
		}
		for (FEntry& Entry : Ready)
		{
			if (Entry.Gpu.IsValid())
			{
				int32 RowPitchPixels = 0;
				Entry.Gpu->Lock(RowPitchPixels);
				Entry.Gpu->Unlock();
			}
			RetireExactCaptureWriter(Entry.ExactState);
			if (Entry.WriterCount.IsValid())
				Entry.WriterCount->fetch_sub(1, std::memory_order_relaxed);
		}
	}

	TArray<FEntry> Entries;
	FDelegateHandle EndFrameHandle;
};
}

bool FRenderCameraCaptureReceipt::IsValid() const
{
	return State.IsValid() && !CameraName.IsEmpty() && !Identity.SceneRevision.IsEmpty()
		&& Resolution.X > 0 && Resolution.Y > 0 && ConfigurationRevision > 0
		&& CaptureSerial > 0;
}

bool FRenderCameraCaptureReceipt::IsGpuWriterDrained() const
{
	return !State.IsValid()
		|| State->bGpuWriterDrained.load(std::memory_order_acquire);
}

FRenderCameraCaptureCompletion FRenderCameraCaptureReceipt::Poll() const
{
	check(IsInGameThread());
	FRenderCameraCaptureCompletion Completion;
	if (!State.IsValid())
	{
		Completion.Outcome = ERenderCameraCaptureOutcome::Teardown;
		Completion.Message = TEXT("exact camera capture receipt is invalid");
		Completion.bGpuWriterDrained = true;
		return Completion;
	}
	Completion.Outcome = State->Outcome;
	Completion.Frame = State->Frame;
	Completion.Message = State->Message;
	Completion.bGpuWriterDrained =
		State->bGpuWriterDrained.load(std::memory_order_acquire);
	return Completion;
}

bool FRenderCameraCaptureReceipt::Cancel(const FString& Reason) const
{
	check(IsInGameThread());
	if (!State.IsValid() || State->Outcome != ERenderCameraCaptureOutcome::Pending)
		return false;
	State->Outcome = ERenderCameraCaptureOutcome::Cancelled;
	State->Message = Reason.IsEmpty()
		? TEXT("exact camera capture was cancelled") : Reason;
	State->bDiscardPixels.store(true, std::memory_order_release);
	return true;
}

// ---------------------------------------------------------------------------
// Lifetime
// ---------------------------------------------------------------------------

URenderCameraComponent::URenderCameraComponent()
{
	// The capture pipeline is driven once per frame by URenderCameraSubsystem, not by
	// a per-component tick.
	PrimaryComponentTick.bCanEverTick = false;
	PrimaryComponentTick.bStartWithTickEnabled = false;

	ResultsQueue = MakeShared<FCompletedReadbackQueue, ESPMode::ThreadSafe>();
	DetachedWriterCount = MakeShared<std::atomic<int32>, ESPMode::ThreadSafe>(0);

	CaptureComponent = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("SceneCapture"));
	if (CaptureComponent)
	{
		CaptureComponent->SetupAttachment(this);
		CaptureComponent->SetRelativeTransform(FTransform::Identity);

		// Start dormant: no capture cost until explicitly enabled.
		CaptureComponent->bCaptureEveryFrame = false;
		CaptureComponent->bCaptureOnMovement = false;
		CaptureComponent->bAlwaysPersistRenderingState = true;
		CaptureComponent->MaxViewDistanceOverride = -1.0f;

		// A scene capture does not automatically respect the scene's post-process
		// volumes, so give its own settings full weight here and copy the volume's
		// at BeginPlay to match the viewport look.
		CaptureComponent->PostProcessBlendWeight = 1.0f;
		CaptureComponent->bUseRayTracingIfEnabled = false;

		CaptureComponent->bHiddenInGame = true;
	}
}

void URenderCameraComponent::AdvanceConfigurationRevision()
{
	InvalidateExactCaptures(ERenderCameraCaptureOutcome::ConfigurationChanged,
		TEXT("camera configuration changed before exact capture completion"));
	if (ConfigurationRevision < MAX_uint64)
	{
		++ConfigurationRevision;
	}
}

void URenderCameraComponent::InvalidateExactCaptures(
	ERenderCameraCaptureOutcome Outcome, const FString& Message)
{
	check(IsInGameThread());
	for (const TSharedPtr<FRenderCameraExactCaptureState, ESPMode::ThreadSafe>& State
		: ExactCaptures)
	{
		if (State.IsValid() && State->Outcome == ERenderCameraCaptureOutcome::Pending)
		{
			State->Outcome = Outcome;
			State->Message = Message;
			State->bDiscardPixels.store(true, std::memory_order_release);
		}
	}
}

void URenderCameraComponent::PruneDrainedExactCaptures()
{
	check(IsInGameThread());
	ExactCaptures.RemoveAllSwap([](
		const TSharedPtr<FRenderCameraExactCaptureState, ESPMode::ThreadSafe>& State)
	{
		return !State.IsValid()
			|| State->bGpuWriterDrained.load(std::memory_order_acquire);
	}, EAllowShrinking::No);
}

bool URenderCameraComponent::Configure(const FRenderCameraDesc& Desc, FString& OutError)
{
	OutError.Reset();
	const FRenderCameraOptics& Optics = Desc.Optics;
	const FRenderCameraCaptureSettings& Settings = Desc.Capture;
	int32 ElementCount = 0;
	int64 LogicalBytes = 0;
	int64 PhysicalBytes = 0;
	if (!RenderCheckedCameraBufferSizing(Settings.Mode, Optics.Resolution,
		ElementCount, LogicalBytes, PhysicalBytes, OutError))
	{
		return false;
	}
	(void)ElementCount;
	(void)LogicalBytes;
	(void)PhysicalBytes;
	if (Desc.CanonicalName.IsEmpty() || Optics.Resolution.X <= 0 || Optics.Resolution.Y <= 0
		|| static_cast<uint8>(Optics.Projection)
			> static_cast<uint8>(ERenderCameraProjection::Orthographic)
		|| static_cast<uint8>(Settings.Mode)
			> static_cast<uint8>(ERenderCameraMode::InstanceSegmentation)
		|| !FMath::IsFinite(Optics.VerticalFovDegrees) || Optics.VerticalFovDegrees <= 0.0f
		|| (Optics.Projection == ERenderCameraProjection::Perspective
			&& Optics.VerticalFovDegrees >= 180.0f)
		|| (Optics.Projection == ERenderCameraProjection::Orthographic
			&& (!FMath::IsFinite(Optics.OrthographicWidthCm) || Optics.OrthographicWidthCm <= 0.0f))
		|| !FMath::IsFinite(Optics.NearClipCm)
		|| !FMath::IsFinite(Optics.FarClipCm) || Optics.NearClipCm <= 0.0f
		|| Optics.FarClipCm <= Optics.NearClipCm
		|| !FMath::IsFinite(Settings.CaptureMaxFps) || Settings.CaptureMaxFps < 0.0f
		|| !FMath::IsFinite(Settings.DepthNearCm) || !FMath::IsFinite(Settings.DepthFarCm)
		|| Settings.DepthNearCm <= 0.0f || Settings.DepthFarCm <= Settings.DepthNearCm
		|| Settings.HistoryCapacity < 1
		|| Settings.HistoryCapacity > FRenderCameraHistory::MaxHistoryCapacity
		|| !FMath::IsFinite(Settings.RequestActiveTtlSeconds)
		|| Settings.RequestActiveTtlSeconds < 0.0f
		|| !FMath::IsFinite(Settings.DelaySeconds) || Settings.DelaySeconds < 0.0f
		|| !FMath::IsFinite(Settings.DelayJitterSeconds) || Settings.DelayJitterSeconds < 0.0f
		|| !FMath::IsFinite(Settings.StreamingBoost) || Settings.StreamingBoost <= 0.0f)
	{
		OutError = TEXT("render camera descriptor contains an invalid name, optic, or capture setting");
		return false;
	}
	if (Optics.bHasIntrinsics
		&& (!FMath::IsFinite(Optics.FocalPixelsX) || !FMath::IsFinite(Optics.FocalPixelsY)
			|| !FMath::IsFinite(Optics.PrincipalPixelsX) || !FMath::IsFinite(Optics.PrincipalPixelsY)
			|| Optics.FocalPixelsX <= 0.0f || Optics.FocalPixelsY <= 0.0f))
	{
		OutError = TEXT("render camera intrinsics must be finite with positive focal lengths");
		return false;
	}
	AdvanceConfigurationRevision();
	Descriptor = Desc;
	RuntimeDescriptor = Desc;
	ActiveCaptureOverride.Reset();
	bCaptureOverridePoisoned = false;
	CaptureMode = Settings.Mode;
	DepthNearCm = Settings.DepthNearCm;
	DepthFarCm = Settings.DepthFarCm;
	bCaptureOnStateChange = Settings.bCaptureOnStateChange;
	CaptureMaxFps = Settings.CaptureMaxFps;
	bManualCaptureOnly = Settings.bManualCaptureOnly;
	HistoryCapacity = Settings.HistoryCapacity;
	RequestActiveTtlSeconds = Settings.RequestActiveTtlSeconds;
	DelaySeconds = Settings.DelaySeconds;
	DelayJitterSeconds = Settings.DelayJitterSeconds;
	bDelayUseWallClock = Settings.bDelayUseWallClock;
	bRenderInMainRenderer = Settings.bRenderInMainRenderer;
	StreamingBoost = Settings.StreamingBoost;
	CustomNearClipCm = Optics.NearClipCm;
	CustomFarClipCm = Optics.FarClipCm;
	Delay.Configure(Settings.DelaySeed != 0 ? Settings.DelaySeed
		: static_cast<int32>(GetTypeHash(Desc.CanonicalName)));
	RefreshCaptureFov();
	return true;
}

bool URenderCameraComponent::GetConfigurationSnapshot(
	FRenderCameraConfigurationSnapshot& OutSnapshot, FString& OutError) const
{
	check(IsInGameThread());
	OutSnapshot = {};
	OutError.Reset();
	if (Descriptor.CanonicalName.IsEmpty() || ConfigurationRevision == 0)
	{
		OutError = TEXT("camera configuration is not initialized");
		return false;
	}

	int32 ElementCount = 0;
	if (!RenderCheckedCameraBufferSizing(CaptureMode, Descriptor.Optics.Resolution,
		ElementCount, OutSnapshot.LogicalPixelBytes, OutSnapshot.PhysicalArrayBytes,
		OutError))
	{
		return false;
	}
	(void)ElementCount;
	OutSnapshot.CanonicalName = Descriptor.CanonicalName;
	OutSnapshot.Mode = CaptureMode;
	OutSnapshot.Resolution = Descriptor.Optics.Resolution;
	OutSnapshot.ConfigurationRevision = ConfigurationRevision;
	return true;
}

namespace
{
void NormalizeInactiveOptics(FRenderCameraOptics& Optics)
{
	if (Optics.Projection == ERenderCameraProjection::Perspective)
	{
		Optics.OrthographicWidthCm = 0.0f;
		if (!Optics.bHasIntrinsics)
		{
			Optics.FocalPixelsX = Optics.FocalPixelsY = 0.0f;
			Optics.PrincipalPixelsX = Optics.PrincipalPixelsY = 0.0f;
		}
	}
	else
	{
		Optics.VerticalFovDegrees = 0.0f;
		Optics.bHasIntrinsics = false;
		Optics.FocalPixelsX = Optics.FocalPixelsY = 0.0f;
		Optics.PrincipalPixelsX = Optics.PrincipalPixelsY = 0.0f;
	}
}

}

bool URenderCameraComponent::ResolveCaptureConfiguration(
	const TOptional<FRenderCameraCaptureOverride>& Override,
	FRenderCameraEffectiveConfiguration& Out, FString& OutError) const
{
	check(IsInGameThread());
	Out = {};
	OutError.Reset();
	if (Descriptor.CanonicalName.IsEmpty() || ConfigurationRevision == 0)
	{
		OutError = TEXT("camera configuration is not initialized");
		return false;
	}
	Out.CanonicalName = Descriptor.CanonicalName;
	Out.BaselineConfigurationRevision = ConfigurationRevision;
	Out.ConfigurationRevision = ConfigurationRevision;
	Out.Mode = Descriptor.Capture.Mode;
	Out.Optics = Descriptor.Optics;
	Out.DepthNearCm = Descriptor.Capture.DepthNearCm;
	Out.DepthFarCm = Descriptor.Capture.DepthFarCm;
	if (Override.IsSet())
	{
		const FRenderCameraCaptureOverride& Value = Override.GetValue();
		if (Value.Resolution.IsSet() && Value.CompleteOptics.IsSet())
		{
			OutError = TEXT("camera override cannot combine resolution and complete optics");
			return false;
		}
		if (Value.DepthNearCm.IsSet() != Value.DepthFarCm.IsSet())
		{
			OutError = TEXT("camera override depth bounds must be supplied together");
			return false;
		}
		if (Value.Mode.IsSet()) Out.Mode = Value.Mode.GetValue();
		if (Value.CompleteOptics.IsSet())
		{
			Out.Optics = Value.CompleteOptics.GetValue();
			if ((Out.Optics.Projection == ERenderCameraProjection::Perspective
					&& (Out.Optics.OrthographicWidthCm != 0.0f
						|| (!Out.Optics.bHasIntrinsics
							&& (Out.Optics.FocalPixelsX != 0.0f || Out.Optics.FocalPixelsY != 0.0f
								|| Out.Optics.PrincipalPixelsX != 0.0f
								|| Out.Optics.PrincipalPixelsY != 0.0f))))
				|| (Out.Optics.Projection == ERenderCameraProjection::Orthographic
					&& (Out.Optics.VerticalFovDegrees != 0.0f || Out.Optics.bHasIntrinsics
						|| Out.Optics.FocalPixelsX != 0.0f || Out.Optics.FocalPixelsY != 0.0f
						|| Out.Optics.PrincipalPixelsX != 0.0f
						|| Out.Optics.PrincipalPixelsY != 0.0f)))
			{
				OutError = TEXT("complete camera optics contains contradictory inactive fields");
				return false;
			}
		}
		else if (Value.Resolution.IsSet())
		{
			const FIntPoint Old = Out.Optics.Resolution;
			const FIntPoint New = Value.Resolution.GetValue();
			if (Old.X <= 0 || Old.Y <= 0 || New.X <= 0 || New.Y <= 0)
			{
				OutError = TEXT("camera override resolution is invalid");
				return false;
			}
			if (Out.Optics.bHasIntrinsics)
			{
				const float ScaleX = static_cast<float>(New.X) / static_cast<float>(Old.X);
				const float ScaleY = static_cast<float>(New.Y) / static_cast<float>(Old.Y);
				Out.Optics.FocalPixelsX *= ScaleX;
				Out.Optics.PrincipalPixelsX *= ScaleX;
				Out.Optics.FocalPixelsY *= ScaleY;
				Out.Optics.PrincipalPixelsY *= ScaleY;
			}
			Out.Optics.Resolution = New;
		}
		if (Value.DepthNearCm.IsSet())
		{
			if (Out.Mode != ERenderCameraMode::Depth)
			{
				OutError = TEXT("depth bounds require effective depth capture mode");
				return false;
			}
			Out.DepthNearCm = Value.DepthNearCm.GetValue();
			Out.DepthFarCm = Value.DepthFarCm.GetValue();
		}
	}
	NormalizeInactiveOptics(Out.Optics);
	Out.Resolution = Out.Optics.Resolution;
	const FRenderCameraOptics& Optics = Out.Optics;
	if (Optics.Resolution.X <= 0 || Optics.Resolution.Y <= 0
		|| static_cast<uint8>(Out.Mode) > static_cast<uint8>(ERenderCameraMode::InstanceSegmentation)
		|| static_cast<uint8>(Optics.Projection) > static_cast<uint8>(ERenderCameraProjection::Orthographic)
		|| !FMath::IsFinite(Optics.NearClipCm) || !FMath::IsFinite(Optics.FarClipCm)
		|| Optics.NearClipCm <= 0.0f || Optics.FarClipCm <= Optics.NearClipCm
		|| !FMath::IsFinite(Out.DepthNearCm) || !FMath::IsFinite(Out.DepthFarCm)
		|| Out.DepthNearCm <= 0.0f || Out.DepthFarCm <= Out.DepthNearCm
		|| (Optics.Projection == ERenderCameraProjection::Perspective
			&& (!FMath::IsFinite(Optics.VerticalFovDegrees)
				|| Optics.VerticalFovDegrees <= 0.0f || Optics.VerticalFovDegrees >= 180.0f))
		|| (Optics.Projection == ERenderCameraProjection::Orthographic
			&& (!FMath::IsFinite(Optics.OrthographicWidthCm) || Optics.OrthographicWidthCm <= 0.0f))
		|| (Optics.bHasIntrinsics && (!FMath::IsFinite(Optics.FocalPixelsX)
			|| !FMath::IsFinite(Optics.FocalPixelsY) || !FMath::IsFinite(Optics.PrincipalPixelsX)
			|| !FMath::IsFinite(Optics.PrincipalPixelsY)
			|| Optics.FocalPixelsX <= 0.0f || Optics.FocalPixelsY <= 0.0f)))
	{
		OutError = TEXT("normalized camera override contains invalid optics or depth bounds");
		return false;
	}
	int32 Elements = 0;
	return RenderCheckedCameraBufferSizing(Out.Mode, Out.Optics.Resolution, Elements,
		Out.LogicalPixelBytes, Out.PhysicalArrayBytes, OutError);
}

bool URenderCameraComponent::ApplyRuntimeCaptureConfiguration(
	const FRenderCameraEffectiveConfiguration& Configuration, FString& OutError)
{
	FRenderCameraDesc Runtime = Descriptor;
	Runtime.Optics = Configuration.Optics;
	Runtime.Capture.Mode = Configuration.Mode;
	Runtime.Capture.DepthNearCm = Configuration.DepthNearCm;
	Runtime.Capture.DepthFarCm = Configuration.DepthFarCm;
	const bool bWasStreaming = bStreamingEnabled;
	if (bWasStreaming) SetStreamingEnabled(false);
	RuntimeDescriptor = Runtime;
	CaptureMode = Runtime.Capture.Mode;
	DepthNearCm = Runtime.Capture.DepthNearCm;
	DepthFarCm = Runtime.Capture.DepthFarCm;
	CustomNearClipCm = Runtime.Optics.NearClipCm;
	CustomFarClipCm = Runtime.Optics.FarClipCm;
	RefreshCaptureFov();
	if (bWasStreaming) SetStreamingEnabled(true);
	OutError.Reset();
	return true;
}

bool URenderCameraComponent::BeginCaptureOverride(
	const FRenderCameraEffectiveConfiguration& Configuration,
	FRenderCameraOverrideToken& OutToken, FString& OutError)
{
	check(IsInGameThread());
	OutToken = {};
	FRenderCameraEffectiveConfiguration Current;
	if (bCaptureOverridePoisoned || ActiveCaptureOverride.IsSet()
		|| NextOverrideSerial == 0 || NextOverrideSerial == MAX_uint64
		|| !ResolveCaptureConfiguration({}, Current, OutError)
		|| Configuration.CanonicalName != Current.CanonicalName
		|| Configuration.BaselineConfigurationRevision != Current.BaselineConfigurationRevision)
	{
		if (OutError.IsEmpty()) OutError = TEXT("camera cannot begin the requested exact override");
		return false;
	}
	FActiveCaptureOverride State;
	State.Serial = NextOverrideSerial++;
	State.SavedRuntimeDescriptor = RuntimeDescriptor;
	if (!ApplyRuntimeCaptureConfiguration(Configuration, OutError)) return false;
	ActiveCaptureOverride = MoveTemp(State);
	OutToken.Camera = this;
	OutToken.Serial = ActiveCaptureOverride->Serial;
	return true;
}

bool URenderCameraComponent::RestoreCaptureOverride(
	const FRenderCameraOverrideToken& Token, FString& OutError)
{
	check(IsInGameThread());
	if (!ActiveCaptureOverride.IsSet() || Token.Camera.Get() != this
		|| Token.Serial != ActiveCaptureOverride->Serial)
	{
		OutError = TEXT("camera override restore token is stale or foreign");
		return false;
	}
	FRenderCameraEffectiveConfiguration Baseline;
	if (!ResolveCaptureConfiguration({}, Baseline, OutError)
		|| !ApplyRuntimeCaptureConfiguration(Baseline, OutError))
	{
		bCaptureOverridePoisoned = true;
		return false;
	}
	ActiveCaptureOverride.Reset();
	return true;
}

void URenderCameraComponent::SetVisibilityComponents(const TArray<UPrimitiveComponent*>& ShowOnly,
	const TArray<UPrimitiveComponent*>& Hidden, int32 InSegmentationObjectType)
{
	AdvanceConfigurationRevision();
	SegmentationObjectType = InSegmentationObjectType;
	if (!CaptureComponent) return;
	ExplicitShowOnlyComponents.Reset(ShowOnly.Num());
	ExplicitHiddenComponents.Reset(Hidden.Num());
	for (UPrimitiveComponent* Component : ShowOnly)
		if (Component) ExplicitShowOnlyComponents.Add(Component);
	for (UPrimitiveComponent* Component : Hidden)
		if (Component) ExplicitHiddenComponents.Add(Component);
	if (!NeutralInstanceComponents.IsEmpty() || !NeutralSemanticComponents.IsEmpty())
	{
		ApplyModeVisibilityComponents();
		return;
	}
	CaptureComponent->ShowOnlyComponents.Reset();
	CaptureComponent->HiddenComponents.Reset();
	for (UPrimitiveComponent* Component : ShowOnly)
		if (Component) CaptureComponent->ShowOnlyComponents.Add(Component);
	for (UPrimitiveComponent* Component : Hidden)
		if (Component) CaptureComponent->HiddenComponents.Add(Component);
}

void URenderCameraComponent::SetNeutralSegmentationComponents(
	const TArray<UPrimitiveComponent*>& Instance, const TArray<UPrimitiveComponent*>& Semantic)
{
	NeutralInstanceComponents.Reset(Instance.Num());
	NeutralSemanticComponents.Reset(Semantic.Num());
	for (UPrimitiveComponent* Component : Instance) if (Component) NeutralInstanceComponents.Add(Component);
	for (UPrimitiveComponent* Component : Semantic) if (Component) NeutralSemanticComponents.Add(Component);
	ApplyModeVisibilityComponents();
}

void URenderCameraComponent::ApplyModeVisibilityComponents()
{
	if (!CaptureComponent || (NeutralInstanceComponents.IsEmpty()
		&& NeutralSemanticComponents.IsEmpty())) return;
	CaptureComponent->ShowOnlyComponents.Reset();
	CaptureComponent->HiddenComponents.Reset();
	const TArray<TObjectPtr<UPrimitiveComponent>>* Selected = nullptr;
	if (CaptureMode == ERenderCameraMode::InstanceSegmentation)
	{
		Selected = &NeutralInstanceComponents;
		SegmentationObjectType = static_cast<int32>(ERenderSegmentationObjectKind::Geometry);
	}
	else if (CaptureMode == ERenderCameraMode::SemanticSegmentation)
	{
		Selected = &NeutralSemanticComponents;
		SegmentationObjectType = static_cast<int32>(ERenderSegmentationObjectKind::Body);
	}
	if (Selected)
	{
		for (UPrimitiveComponent* Component : *Selected)
			if (Component) CaptureComponent->ShowOnlyComponents.Add(Component);
	}
	else
	{
		SegmentationObjectType = -1;
		for (UPrimitiveComponent* Component : ExplicitShowOnlyComponents)
			if (Component) CaptureComponent->ShowOnlyComponents.Add(Component);
		for (UPrimitiveComponent* Component : ExplicitHiddenComponents)
			if (Component) CaptureComponent->HiddenComponents.Add(Component);
		for (UPrimitiveComponent* Component : NeutralInstanceComponents)
			if (Component) CaptureComponent->HiddenComponents.Add(Component);
		for (UPrimitiveComponent* Component : NeutralSemanticComponents)
			if (Component) CaptureComponent->HiddenComponents.Add(Component);
	}
}

void URenderCameraComponent::SetSceneVisibility(bool bVisible)
{
	if (bSceneVisible != bVisible) AdvanceConfigurationRevision();
	bSceneVisible = bVisible;
	SetVisibility(bVisible, true);
	if (CaptureComponent)
	{
		CaptureComponent->SetVisibility(bVisible && bStreamingEnabled, true);
		CaptureComponent->bHiddenInGame = !bVisible;
	}
}

void URenderCameraComponent::OnRegister()
{
	Super::OnRegister();
	bEndingPlay = false;
	// The scene-capture is a default subobject attached in the constructor. When
	// this camera is spawned as part of an actor/Blueprint, RegisterAllComponents
	// attaches + registers it for us. But when it is created at runtime via
	// NewObject + RegisterComponent at runtime, that cascade
	// does not run, leaving the capture detached at the origin -- so it must be
	// attached to this component and registered explicitly. Idempotent: a no-op
	// once it is already parented + registered.
	if (CaptureComponent)
	{
		if (CaptureComponent->GetAttachParent() != this)
		{
			CaptureComponent->AttachToComponent(this, FAttachmentTransformRules::KeepRelativeTransform);
		}
		if (!CaptureComponent->IsRegistered() && GetWorld())
		{
			CaptureComponent->RegisterComponent();
		}
	}
	RefreshCaptureFov();
	if (UWorld* World = GetWorld())
	{
		if (URenderCameraSubsystem* Subsystem = World->GetSubsystem<URenderCameraSubsystem>())
		{
			Subsystem->RegisterCamera(this);
		}
	}
}

void URenderCameraComponent::OnUnregister()
{
	bEndingPlay = true;
	InvalidateExactCaptures(ERenderCameraCaptureOutcome::Teardown,
		TEXT("camera unregistered before exact capture completion"));
	if (UWorld* World = GetWorld())
	{
		if (URenderCameraSubsystem* Subsystem = World->GetSubsystem<URenderCameraSubsystem>())
		{
			Subsystem->UnregisterCamera(this);
		}
	}
	if (bStreamingEnabled) SetStreamingEnabled(false);
	Super::OnUnregister();
}

void URenderCameraComponent::BeginPlay()
{
	Super::BeginPlay();
	if (CaptureComponent && GetWorld())
	{
		for (TActorIterator<APostProcessVolume> It(GetWorld()); It; ++It)
		{
			if (APostProcessVolume* Volume = *It; Volume && Volume->bEnabled)
			{
				CaptureComponent->PostProcessSettings = Volume->Settings;
				CaptureComponent->PostProcessBlendWeight = 1.0f;
				break;
			}
		}
		CaptureComponent->ShowFlags.SetMotionBlur(false);
		CaptureComponent->PostProcessSettings.bOverride_MotionBlurAmount = true;
		CaptureComponent->PostProcessSettings.MotionBlurAmount = 0.0f;
	}
}

void URenderCameraComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	bEndingPlay = true;
	InvalidateExactCaptures(ERenderCameraCaptureOutcome::Teardown,
		TEXT("camera ended play before exact capture completion"));
	if (UWorld* World = GetWorld())
	{
		if (URenderCameraSubsystem* Subsystem = World->GetSubsystem<URenderCameraSubsystem>())
		{
			Subsystem->UnregisterCamera(this);
		}
	}
	if (bStreamingEnabled) SetStreamingEnabled(false);
	Super::EndPlay(EndPlayReason);
}

// ---------------------------------------------------------------------------
// Resolution and field of view
// ---------------------------------------------------------------------------

FIntPoint URenderCameraComponent::CaptureResolution() const
{
	return RuntimeDescriptor.Optics.Resolution;
}

void URenderCameraComponent::SetClippingPlanes(float InNearClipCm, float InFarClipCm)
{
	if (CustomNearClipCm == InNearClipCm && CustomFarClipCm == InFarClipCm) return;
	AdvanceConfigurationRevision();
	CustomNearClipCm = InNearClipCm;
	CustomFarClipCm = InFarClipCm;
	if (CaptureComponent)
	{
		CaptureComponent->bOverride_CustomNearClippingPlane = true;
		CaptureComponent->CustomNearClippingPlane = FMath::Max(0.01f, CustomNearClipCm);
		SetupProjectionMatrix();
	}
}

void URenderCameraComponent::SetupProjectionMatrix()
{
	if (!CaptureComponent) return;
	const FRenderCameraOptics& Optics = RuntimeDescriptor.Optics;
	const float W = static_cast<float>(Optics.Resolution.X);
	const float H = static_cast<float>(Optics.Resolution.Y);
	if (W <= 0.0f || H <= 0.0f) return;

	DerivedFovy = Optics.VerticalFovDegrees;
	CaptureComponent->ProjectionType = Optics.Projection == ERenderCameraProjection::Orthographic
		? ECameraProjectionMode::Orthographic : ECameraProjectionMode::Perspective;
	CaptureComponent->OrthoWidth = Optics.OrthographicWidthCm;
	CaptureComponent->FOVAngle = HorizontalFOVFromFovy(DerivedFovy, Optics.Resolution);
	CaptureComponent->bOverride_CustomNearClippingPlane = true;
	CaptureComponent->CustomNearClippingPlane = FMath::Max(0.01f, Optics.NearClipCm);
	if (Optics.Projection == ERenderCameraProjection::Orthographic)
	{
		CaptureComponent->bUseCustomProjectionMatrix = false;
		return;
	}

	const float Fy = Optics.bHasIntrinsics ? Optics.FocalPixelsY
		: (H * 0.5f) / FMath::Tan(FMath::DegreesToRadians(DerivedFovy) * 0.5f);
	const float Fx = Optics.bHasIntrinsics ? Optics.FocalPixelsX : Fy;
	const float Cx = Optics.bHasIntrinsics ? Optics.PrincipalPixelsX : W * 0.5f;
	const float Cy = Optics.bHasIntrinsics ? Optics.PrincipalPixelsY : H * 0.5f;
	FMatrix Projection = FMatrix::Identity;
	Projection.M[0][0] = 2.0f * Fx / W; Projection.M[0][1] = 0.0f;
	Projection.M[0][2] = 0.0f; Projection.M[0][3] = 0.0f;
	Projection.M[1][0] = 0.0f; Projection.M[1][1] = 2.0f * Fy / H;
	Projection.M[1][2] = 0.0f; Projection.M[1][3] = 0.0f;
	Projection.M[2][0] = (W - 2.0f * Cx) / W;
	Projection.M[2][1] = (2.0f * Cy - H) / H;
	Projection.M[2][2] = 0.0f; Projection.M[2][3] = 1.0f;
	Projection.M[3][0] = 0.0f; Projection.M[3][1] = 0.0f;
	Projection.M[3][2] = FMath::Max(0.01f, Optics.NearClipCm); Projection.M[3][3] = 0.0f;
	CaptureComponent->bUseCustomProjectionMatrix = true;
	CaptureComponent->CustomProjectionMatrix = Projection;
}

void URenderCameraComponent::RefreshCaptureFov()
{
	DerivedFovy = DeriveFovyDegrees(*this);
	SetupProjectionMatrix();
}

// ---------------------------------------------------------------------------
// Capture pipeline
// ---------------------------------------------------------------------------

void URenderCameraComponent::UpdateCapturePipeline(bool bAllowCaptureIssue)
{
	const bool bActive = IsCaptureActive();

	// Per-camera gating: start capturing lazily when a camera becomes active, e.g.
	// on the first include_cameras request against a non-broadcast camera, and
	// pause the GPU scene capture when it goes dormant, so a scene with many
	// cameras only pays for the ones actually consumed.
	if (bAllowCaptureIssue && bActive && !bStreamingEnabled)
	{
		SetStreamingEnabled(true);
	}

	if (bStreamingEnabled && CaptureComponent && CaptureComponent->TextureTarget)
	{
		// Normal cameras are captured manually in MaybeCapture right before the
		// readback copy, so the automatic per-frame capture stays off for them: it
		// would render on the main cadence and race the copy, yielding stale or
		// black frames. Only main-renderer cameras, which must render on the main
		// cadence, keep it on while active.
		const bool bWantEveryFrame = bAllowCaptureIssue && bActive && bRenderInMainRenderer;
		if (CaptureComponent->bCaptureEveryFrame != bWantEveryFrame)
		{
			CaptureComponent->bCaptureEveryFrame = bWantEveryFrame;
		}
		if (bActive)
		{
			// Every tick, because the streaming manager decays viewpoints on a timeout.
			RegisterWithStreamingManager();
		}
	}

	HarvestCompletedReadbacks();

	// Manual-capture-only cameras (render-server forced-render mode) still harvest
	// above, but never auto-capture -- only the on-demand IssueSyncCapture drives
	// them, so the async stream cannot contend with a forced request.
	if (bStreamingEnabled && bActive && !bManualCaptureOnly)
	{
		if (bAllowCaptureIssue) MaybeCapture();
		PublishDueDelayedFrames();
	}
}

void URenderCameraComponent::HarvestCompletedReadbacks()
{
	// Two async stages, both driven from the game thread with no render-thread
	// flush: promote GPU-ready readbacks to a render-thread map and copy, then
	// drain whatever the render thread finished into the history ring.
	DispatchReadyReadbacks();
	DrainCompletedFrames();
}

void URenderCameraComponent::DispatchReadyReadbacks()
{
	// FIFO: readbacks complete in submission order, so stop at the first entry
	// whose GPU copy is not ready rather than reordering later frames ahead of it.
	while (InFlightReadbacks.Num() > 0)
	{
		FInFlightReadback& Head = InFlightReadbacks[0];
		if (!Head.Gpu.IsValid())
		{
			if (Head.ExactState.IsValid()
				&& Head.ExactState->Outcome == ERenderCameraCaptureOutcome::Pending)
			{
				Head.ExactState->Outcome = ERenderCameraCaptureOutcome::MapFailed;
				Head.ExactState->Message = TEXT("camera GPU readback was unavailable");
			}
			RetireExactCaptureWriter(Head.ExactState);
			InFlightReadbacks.RemoveAt(0);
			continue;
		}
		if (!Head.Gpu->IsReady())
		{
			break;
		}

		FInFlightReadback Front = MoveTemp(InFlightReadbacks[0]);
		InFlightReadbacks.RemoveAt(0);
		DispatchReadbackMap(MoveTemp(Front));
	}
}

void URenderCameraComponent::DispatchReadbackMap(FInFlightReadback&& Readback)
{
	// First game tick that observed the GPU fence signalled: the boundary
	// between "GPU copy done" and "we noticed", including tick quantization.
	Readback.TFenceReadySeconds = FPlatformTime::Seconds();
	FMutableRenderCameraFramePtr Frame = MakeShared<FRenderCameraFrame, ESPMode::ThreadSafe>();
	Frame->Identity = Readback.Identity;
	Frame->Mode = Readback.Mode;
	Frame->CameraConfigurationRevision = Readback.ConfigurationRevision;
	Frame->CaptureSerial = Readback.CaptureSerial;
	Frame->Width = Readback.Width;
	Frame->Height = Readback.Height;
	Frame->CaptureUnixTime = Readback.CaptureUnixSeconds;
	Frame->TIssuedSeconds = Readback.TIssuedSeconds;
	Frame->TFenceReadySeconds = Readback.TFenceReadySeconds;

	const int32 W = Readback.Width;
	const int32 H = Readback.Height;
	const ERenderCameraMode CapturedMode = Readback.Mode;
	const int32 CapturedSegmentationObjectType = Readback.SegmentationObjectType;
	const uint64 CapturedIdentityGeneration = Readback.IdentityGeneration;
	TSharedPtr<FRHIGPUTextureReadback, ESPMode::ThreadSafe> Gpu =
		MoveTemp(Readback.Gpu);
	TSharedPtr<FRenderCameraExactCaptureState, ESPMode::ThreadSafe> ExactState =
		Readback.ExactState;
	++PendingMapCommands;

	// Map and copy the ready staging buffer on the render thread. Captured
	// objects are all owned; neither the component nor mutable camera mode enters
	// the command. The exact lease is released only after this writer completes.
	const bool bSplitDiag = CVarCamDiag.GetValueOnGameThread() != 0;
	ENQUEUE_RENDER_COMMAND(MjCameraMapReadback)
	([Frame = MoveTemp(Frame), Gpu = MoveTemp(Gpu), W, H,
		CapturedMode, CapturedSegmentationObjectType,
		CapturedIdentityGeneration, ExactState,
		Results = ResultsQueue, bSplitDiag](FRHICommandListImmediate&) mutable {
		const double RTStart = FPlatformTime::Seconds();
		if (bSplitDiag)
		{
			UE_LOG(LogURLabNet, Log, TEXT("[mapsplit] enqueue->RTstart=%.2f ms"),
				(RTStart - Frame->TFenceReadySeconds) * 1000.0);
		}
		const bool bDiscard = ExactState.IsValid()
			&& ExactState->bDiscardPixels.load(std::memory_order_acquire);
		bool bCopied = false;
		int32 RowPitchPixels = 0;
		void* Data = Gpu->Lock(RowPitchPixels);
		if (!bDiscard && Data && W > 0 && H > 0)
		{
			RenderCopyReadbackIntoFrame(Data, RowPitchPixels, CapturedMode,
				CapturedSegmentationObjectType, *Frame);
			bCopied = true;
		}
		Gpu->Unlock();
		Frame->TMappedSeconds = FPlatformTime::Seconds();
		if (bSplitDiag)
		{
			UE_LOG(LogURLabNet, Log, TEXT("[mapsplit] map-work=%.2f ms"),
				(Frame->TMappedSeconds - RTStart) * 1000.0);
		}
		if (ExactState.IsValid())
		{
			if (bCopied)
				Frame->PixelLifetimeLease = MoveTemp(ExactState->LifetimeLease);
			else
				ExactState->LifetimeLease.Reset();
		}
		if (ExactState.IsValid())
			ExactState->bGpuWriterDrained.store(true, std::memory_order_release);
		FCompletedReadback Completed;
		Completed.Frame = MoveTemp(Frame);
		Completed.Gpu = MoveTemp(Gpu);
		Completed.IdentityGeneration = CapturedIdentityGeneration;
		Completed.ExactState = ExactState;
		Completed.bCopied = bCopied;
		Results->Enqueue(MoveTemp(Completed));
	});
}

void URenderCameraComponent::DrainCompletedFrames()
{
	// Single-consumer drain. The guard makes the invariant explicit: even if a
	// render flush elsewhere pumped the game thread mid-drain, the nested call
	// bails instead of racing the SPSC queue's read cursor.
	if (bDrainingResults)
	{
		return;
	}
	TGuardValue<bool> DrainGuard(bDrainingResults, true);

	FCompletedReadback Completed;
	while (ResultsQueue->Dequeue(Completed))
	{
		--PendingMapCommands;
		FScopeLock IdentityGate(&IdentityGateLock);
		const bool bIdentityCurrent =
			IsIdentityGenerationCurrent(Completed.IdentityGeneration);
		bool bPublish = Completed.bCopied && Completed.Frame.IsValid()
			&& bIdentityCurrent;
		if (Completed.ExactState.IsValid())
		{
			FRenderCameraExactCaptureState& Exact = *Completed.ExactState;
			if (Exact.Outcome == ERenderCameraCaptureOutcome::Pending)
			{
				if (!Completed.bCopied || !Completed.Frame.IsValid())
				{
					Exact.Outcome = ERenderCameraCaptureOutcome::MapFailed;
					Exact.Message = TEXT("camera GPU staging map or pixel copy failed");
				}
				else if (Completed.Frame->CameraConfigurationRevision
					!= ConfigurationRevision)
				{
					Exact.Outcome = ERenderCameraCaptureOutcome::ConfigurationChanged;
					Exact.Message = TEXT("camera configuration changed before completion");
				}
				else if (!bIdentityCurrent)
				{
					Exact.Outcome = ERenderCameraCaptureOutcome::IdentityChanged;
					Exact.Message = TEXT("camera scene or clock identity changed before completion");
				}
				else
				{
					Exact.Outcome = ERenderCameraCaptureOutcome::Succeeded;
					Exact.Frame = Completed.Frame;
				}
			}
			bPublish = bPublish
				&& Exact.Outcome == ERenderCameraCaptureOutcome::Succeeded;
		}

		const bool bExactResult = Completed.ExactState.IsValid();
		if (bPublish)
		{
			FRenderCameraFrame& Frame = *Completed.Frame;
			Frame.Seq = ++HarvestSeq;
			// Game thread now holds the mapped pixels: the copy-out is complete
			// here, before any artificial delay. This is the end of the path a
			// render-thread publish would shortcut.
			Frame.TPublishedSeconds = FPlatformTime::Seconds();
			AccumulateStageTiming(Frame);
			// Reveal time = capture clock + sampled latency. Stamped here so the
			// delayed publish can select on it; unused when no delay is configured.
			Frame.RevealValue = FrameClock(Frame) + SampleDelaySeconds();

			// The no-delay path publishes the instant the frame is harvested. With
			// latency emulation on, harvested frames go to history only and are
			// published later by the reveal-time selection, so the stream lags by
			// the configured delay.
			if (!bExactResult && !IsDelayActive())
			{
				EmitFrame(Completed.Frame);
			}
			if (!bExactResult) PushFrameToHistoryShared(Completed.Frame);
		}
		else if (!Completed.ExactState.IsValid()
			|| Completed.ExactState->Outcome == ERenderCameraCaptureOutcome::MapFailed)
		{
			// The map failed: the render target was torn down mid-flight, or the
			// target had zero size. Drop the frame, still recycle the readback.
			UE_LOG(LogURLabNet, Verbose,
				TEXT("[MjCamera] '%s' dropped a readback whose staging map failed"), *CameraLogName(*this));
		}

		if (Completed.Gpu.IsValid())
		{
			FreeReadbacks.Add(Completed.Gpu); // the staging texture is reused in place
		}
	}
	PruneDrainedExactCaptures();
}

void URenderCameraComponent::AccumulateStageTiming(const FRenderCameraFrame& Frame)
{
	if (CVarCamDiag.GetValueOnGameThread() == 0)
	{
		return;
	}
	// Only frames whose four stamps are all present decompose cleanly; a missing
	// stamp (still 0) would poison the means, so skip it.
	if (Frame.TIssuedSeconds <= 0.0 || Frame.TFenceReadySeconds <= 0.0
		|| Frame.TMappedSeconds <= 0.0 || Frame.TPublishedSeconds <= 0.0)
	{
		return;
	}

	const double IssueToFence = (Frame.TFenceReadySeconds - Frame.TIssuedSeconds) * 1000.0;
	const double FenceToMap = (Frame.TMappedSeconds - Frame.TFenceReadySeconds) * 1000.0;
	const double MapToPublish = (Frame.TPublishedSeconds - Frame.TMappedSeconds) * 1000.0;
	const double Total = (Frame.TPublishedSeconds - Frame.TIssuedSeconds) * 1000.0;

	DiagSumIssueToFence += IssueToFence;
	DiagSumFenceToMap += FenceToMap;
	DiagSumMapToPublish += MapToPublish;
	DiagSumTotal += Total;
	DiagMaxMapToPublish = FMath::Max(DiagMaxMapToPublish, MapToPublish);
	DiagMaxTotal = FMath::Max(DiagMaxTotal, Total);
	++DiagFrames;

	if (DiagFrames >= DiagWindowFrames)
	{
		const double Inv = 1.0 / static_cast<double>(DiagFrames);
		UE_LOG(LogURLabNet, Log,
			TEXT("[camstage] '%s' N=%d mean(ms): issue->fence=%.2f fence->map=%.2f "
				 "map->publish=%.2f total=%.2f | max map->publish=%.2f total=%.2f"),
			*CameraLogName(*this), DiagFrames,
			DiagSumIssueToFence * Inv, DiagSumFenceToMap * Inv,
			DiagSumMapToPublish * Inv, DiagSumTotal * Inv,
			DiagMaxMapToPublish, DiagMaxTotal);
		DiagFrames = 0;
		DiagSumIssueToFence = DiagSumFenceToMap = DiagSumMapToPublish = DiagSumTotal = 0.0;
		DiagMaxMapToPublish = DiagMaxTotal = 0.0;
	}
}

void URenderCameraComponent::WaitAndHarvestReadbacks(double TimeoutSeconds)
{
	// Poll the pipeline instead of flushing it: the render thread runs its map and
	// copy commands independently while we yield, and pushes finished frames onto
	// the results queue for us to drain. Bounded by the deadline.
	const double Deadline = FPlatformTime::Seconds() + TimeoutSeconds;
	for (;;)
	{
		HarvestCompletedReadbacks();
		if (InFlightReadbacks.Num() == 0 && PendingMapCommands == 0)
		{
			break;
		}
		if (FPlatformTime::Seconds() >= Deadline)
		{
			break;
		}
		FPlatformProcess::SleepNoStats(0.0005f);
	}
}

void URenderCameraComponent::IssueSyncCapture()
{
	TouchRequested();
	if (!CanCaptureIdentity(AppliedIdentity)) return;
	if (CaptureComponent && CaptureComponent->TextureTarget) CaptureComponent->CaptureScene();
	EnqueueReadback(AppliedIdentity, /*bForceSubmit=*/true);
}

FRenderCameraCaptureIssueResult URenderCameraComponent::TryIssueExactCapture(
	FRenderCameraCaptureLifetimeLeasePtr LifetimeLease)
{
	check(IsInGameThread());
	FRenderCameraCaptureIssueResult Result;
	if (bEndingPlay)
	{
		Result.Code = ERenderCameraCaptureIssue::ShuttingDown;
		Result.Message = TEXT("camera is ending play");
		return Result;
	}
	TouchRequested();
	if (!bStreamingEnabled)
	{
		SetStreamingEnabled(true);
	}
	if (AppliedIdentity.SceneRevision.IsEmpty()
		|| !FMath::IsFinite(AppliedIdentity.SourceTime)
		|| !CanCaptureIdentity(AppliedIdentity))
	{
		Result.Code = ERenderCameraCaptureIssue::InvalidIdentity;
		Result.Message = TEXT("camera has no complete capturable applied-frame identity");
		return Result;
	}
	if (CaptureComponent && CaptureComponent->TextureTarget)
	{
		CaptureComponent->CaptureScene();
	}
	if (!EnqueueReadback(AppliedIdentity, /*bForceSubmit=*/true, &Result.Receipt,
		&Result.Code, MoveTemp(LifetimeLease)))
	{
		if (Result.Code == ERenderCameraCaptureIssue::RetryableNotReady)
			Result.Message = TEXT("camera render target is not ready for exact capture");
		else if (Result.Code == ERenderCameraCaptureIssue::RetryableBusy)
			Result.Message = TEXT("camera readback capacity is currently full");
		else if (Result.Code == ERenderCameraCaptureIssue::InvalidConfiguration)
			Result.Message = TEXT("camera capture serial space is exhausted");
		else
			Result.Message = TEXT("camera applied-frame identity is not capturable");
	}
	return Result;
}

void RenderPumpForcedCapture(const TArray<URenderCameraComponent*>& Cams, uint64 TargetFrameId, int32 TimeoutMs)
{
	// Bound the pump under the request timeout: it must cover render + readback (and a
	// cold RT's one-time warm-up) for every requested camera, and capping it means a
	// genuinely stuck frame frees the caller rather than hanging on the deadline.
	const double PumpStart = FPlatformTime::Seconds();
	const double Deadline = PumpStart + FMath::Max(1, TimeoutMs) / 1000.0;
	const bool bDiag = CVarCamDiag.GetValueOnAnyThread() != 0;
	TArray<double> ReadyMs;
	if (bDiag)
	{
		ReadyMs.Init(-1.0, Cams.Num());
	}
	for (;;)
	{
		bool bAllReady = true;
		for (int32 i = 0; i < Cams.Num(); ++i)
		{
			URenderCameraComponent* Cam = Cams[i];
			if (!Cam)
			{
				continue;
			}
			Cam->HarvestCompletedReadbacks();
			const uint64 Have = Cam->GetLatestFrameId();
			const bool bReady = (TargetFrameId > 0) ? (Have >= TargetFrameId) : (Have > 0);
			if (!bReady)
			{
				bAllReady = false;
				// Re-issue only while nothing is outstanding, so a cold RT that could not
				// render on the first attempt retries without piling captures behind an
				// in-flight one.
				if (!Cam->HasPendingReadbacks())
				{
					Cam->IssueSyncCapture();
				}
			}
			else if (bDiag && ReadyMs.IsValidIndex(i) && ReadyMs[i] < 0.0)
			{
				ReadyMs[i] = (FPlatformTime::Seconds() - PumpStart) * 1000.0;
			}
		}
		if (bAllReady || FPlatformTime::Seconds() >= Deadline)
		{
			break;
		}
		FPlatformProcess::SleepNoStats(0.0002f);
	}
	if (bDiag)
	{
		FString Line;
		for (double Ms : ReadyMs)
		{
			Line += FString::Printf(TEXT("%.1f "), Ms);
		}
		UE_LOG(LogURLab, Log, TEXT("[pump] %d cams, per-cam ready ms (order-issued): %s"),
			Cams.Num(), *Line);
	}
}

void RenderSpearForcedCapture(const TArray<URenderCameraComponent*>& Cams, const FRenderFrameIdentity& Identity)
{
	// One synchronous readback pass over every requested camera. The caller has
	// already issued CaptureComponent->CaptureScene() for each, so those capture
	// renders are queued ahead of this command on the render thread; enqueuing the
	// copies here and submitting them with a single flush reads the freshly-rendered
	// frame. Lock() blocks the render thread on each copy's GPU fence, so there is no
	// IsReady() poll and no game-tick bounce -- the game thread waits exactly once, on
	// the FlushRenderingCommands below.
	struct FSpearItem
	{
		URenderCameraComponent* Cam = nullptr;
		FTextureRHIRef Source;
		TSharedPtr<FRHIGPUTextureReadback> Readback;
		FMutableRenderCameraFramePtr Frame;
		// The whole mode, not a bDepth bool: there are three copy-out shapes now
		// and a bool can only ever choose between two.
		ERenderCameraMode Mode = ERenderCameraMode::Real;
		int32 SegmentationObjectType = -1;
	};

	TArray<FSpearItem> Items;
	Items.Reserve(Cams.Num());
	const bool bSpearDiag = CVarCamSegDiag.GetValueOnGameThread() != 0;
	for (URenderCameraComponent* Cam : Cams)
	{
		if (!Cam || !Cam->RenderTarget || !Cam->CanCaptureIdentity(Identity))
		{
			continue;
		}
		Cam->TouchRequested();
		FTextureRenderTargetResource* Resource = Cam->RenderTarget->GameThread_GetRenderTargetResource();
		if (!Resource)
		{
			continue;
		}
		const FIntPoint Size = Resource->GetSizeXY();
		FTextureRHIRef Source = Resource->GetRenderTargetTexture();
		// A freshly enabled render target reports its size before its RHI texture
		// exists; skip a cold camera and let the caller's next request land it.
		if (Size.X <= 0 || Size.Y <= 0 || !Source.IsValid())
		{
			if (bSpearDiag)
			{
				UE_LOG(LogURLabNet, Log,
					TEXT("[segdiag] '%s' skipped COLD: size=%dx%d rhi=%d"),
					*CameraLogName(*Cam), Size.X, Size.Y, Source.IsValid() ? 1 : 0);
			}
			continue;
		}
		if (bSpearDiag)
		{
			UE_LOG(LogURLabNet, Log,
				TEXT("[segdiag] '%s' capturing mode=%s rtfmt=%d showonly=%d hidden=%d ")
				TEXT("src=%d prm=%d firstShowOnlyMat=%s"),
				*CameraLogName(*Cam), TEXT("render"),
				static_cast<int32>(Cam->RenderTarget->GetFormat()),
				Cam->CaptureComponent ? Cam->CaptureComponent->ShowOnlyComponents.Num() : -1,
				Cam->CaptureComponent ? Cam->CaptureComponent->HiddenComponents.Num() : -1,
				Cam->CaptureComponent ? static_cast<int32>(Cam->CaptureComponent->CaptureSource) : -1,
				Cam->CaptureComponent ? static_cast<int32>(Cam->CaptureComponent->PrimitiveRenderMode) : -1,
				(Cam->CaptureComponent && Cam->CaptureComponent->ShowOnlyComponents.Num() > 0
					&& Cam->CaptureComponent->ShowOnlyComponents[0].IsValid()
					&& Cam->CaptureComponent->ShowOnlyComponents[0]->GetMaterial(0))
					? *Cam->CaptureComponent->ShowOnlyComponents[0]->GetMaterial(0)->GetName()
					: TEXT("<none>"));
		}

		FSpearItem& Item = Items.AddDefaulted_GetRef();
		Item.Cam = Cam;
		Item.Source = MoveTemp(Source);
		Item.Readback = MakeShared<FRHIGPUTextureReadback>(TEXT("MjSpearForcedReadback"));
		Item.Frame = MakeShared<FRenderCameraFrame, ESPMode::ThreadSafe>();
		Item.Frame->Identity = Identity;
		Item.Frame->Width = Size.X;
		Item.Frame->Height = Size.Y;
		Item.Frame->CaptureUnixTime = (FDateTime::UtcNow() - FDateTime(1970, 1, 1)).GetTotalSeconds();
		Item.Mode = Cam->CaptureMode;
		Item.SegmentationObjectType = Cam->GetSegmentationObjectType();
	}

	if (Items.Num() == 0)
	{
		return;
	}

	// The whole readback -- copy, submit, fence-wait, memcpy -- in one render command,
	// so the render thread never yields to a game tick mid-pass.
	ENQUEUE_RENDER_COMMAND(RenderSpearForcedCapture)
	([Items](FRHICommandListImmediate& RHICmdList) {
		for (const FSpearItem& It : Items)
		{
			It.Readback->EnqueueCopy(RHICmdList, It.Source.GetReference());
		}
		// Submit every copy to the GPU at once.
		RHICmdList.ImmediateFlush(EImmediateFlushType::DispatchToRHIThread);
		for (const FSpearItem& It : Items)
		{
			// Lock blocks the render thread on this copy's GPU fence, so the staged
			// pixels are complete here without a poll. RowPitch is in PIXELS and at
			// least the width (rows are padded), so the copy is row-by-row into a
			// tightly packed buffer.
			int32 RowPitchPixels = 0;
			void* Mapped = It.Readback->Lock(RowPitchPixels);
			const int32 W = It.Frame->Width;
			const int32 H = It.Frame->Height;
			if (Mapped && W > 0 && H > 0)
			{
				RenderCopyReadbackIntoFrame(Mapped, RowPitchPixels, It.Mode, It.SegmentationObjectType, *It.Frame);
			}
			It.Readback->Unlock();
		}
	});

	// Block the game thread until the pass finished; the frames now hold the
	// freshly-rendered pixels. Retain each so the reply path (GetFrameForRequest)
	// hands it straight back.
	FlushRenderingCommands();
	for (FSpearItem& It : Items)
	{
		if (It.Cam && It.Frame.IsValid())
		{
			It.Cam->PushFrameToHistory(MoveTemp(*It.Frame));
		}
	}
}

void URenderCameraComponent::SetAppliedFrameIdentity(const FRenderFrameIdentity& Identity)
{
	FScopeLock IdentityGate(&IdentityGateLock);
	if ((!AppliedIdentity.SceneRevision.IsEmpty()
		&& AppliedIdentity.SceneRevision != Identity.SceneRevision)
		|| AppliedIdentity.ClockEpoch != Identity.ClockEpoch)
	{
		InvalidateExactCaptures(ERenderCameraCaptureOutcome::IdentityChanged,
			TEXT("camera scene or clock identity changed before completion"));
		IdentityGeneration.fetch_add(1, std::memory_order_acq_rel);
		History.Clear();
		Delay.LastPublishedSeq = 0;
		LastCapturedFrameId = 0;
	}
	AppliedIdentity = Identity;
}

FRenderFrameIdentity URenderCameraComponent::GetAppliedFrameIdentity() const
{
	FScopeLock IdentityGate(&IdentityGateLock);
	return AppliedIdentity;
}



void URenderCameraComponent::MaybeCapture()
{
	if (IsDelayActive() && !bDelayUseWallClock && !AppliedIdentity.bHasSourceTime)
	{
		return;
	}
	const uint64 AppliedId = AppliedIdentity.FrameId;
	const double NowWall = FPlatformTime::Seconds();
	const bool bFpsOk = CaptureMaxFps <= 0.0f
		|| (NowWall - LastCaptureWallSeconds) >= (1.0 / static_cast<double>(CaptureMaxFps));
	const bool bStateAdvanced = !bCaptureOnStateChange || AppliedId != LastCapturedFrameId;
	if (bFpsOk && bStateAdvanced)
	{
		if (!bRenderInMainRenderer && CaptureComponent) CaptureComponent->CaptureScene();
		const bool bIssued = EnqueueReadback(AppliedIdentity,
			/*bForceSubmit=*/CVarCamForceSubmit.GetValueOnGameThread() != 0);
		if (bIssued)
		{
			LastCapturedFrameId = AppliedId;
			LastCaptureWallSeconds = NowWall;
		}
	}
}

void URenderCameraComponent::PublishDueDelayedFrames()
{
	if (!IsDelayActive()) return;
	if (!bDelayUseWallClock && !AppliedIdentity.bHasSourceTime) return;
	const double AppliedTime = AppliedIdentity.SourceTime;
	const double NowVal = FRenderCameraDelayModel::NowClockValue(bDelayUseWallClock, AppliedTime);
	FRenderCameraFramePtr Selected = SelectDelayedFrameShared(NowVal, Delay.LastPublishedSeq);
	if (Selected.IsValid())
	{
		EmitFrame(Selected);
		Delay.LastPublishedSeq = Selected->Seq;
	}
}

// ---------------------------------------------------------------------------
// Streaming setup
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Readback copy-out, one implementation for three call sites
// ---------------------------------------------------------------------------

int32 URenderCameraComponent::ChannelsForMode(ERenderCameraMode Mode)
{
	if (Mode == ERenderCameraMode::Depth)
	{
		return 1;
	}
	return RenderIsSegmentationMode(Mode) ? 2 : 4;
}

void RenderCopyReadbackIntoFrame(const void* Mapped, int32 RowPitchPixels,
	ERenderCameraMode Mode, int32 SegmentationObjectType, FRenderCameraFrame& Frame)
{
	Frame.Mode = Mode;
	const int32 W = Frame.Width;
	const int32 H = Frame.Height;
	if (!Mapped || W <= 0 || H <= 0 || RowPitchPixels < W)
	{
		return;
	}
	int32 ElementCount = 0;
	int64 LogicalBytes = 0;
	int64 PhysicalBytes = 0;
	FString SizingError;
	if (!RenderCheckedCameraBufferSizing(Mode, FIntPoint(W, H), ElementCount,
		LogicalBytes, PhysicalBytes, SizingError))
	{
		return;
	}
	(void)LogicalBytes;
	(void)PhysicalBytes;

	if (Mode == ERenderCameraMode::Depth)
	{
		Frame.Depth.Reserve(ElementCount);
		Frame.Depth.SetNumUninitialized(ElementCount);
		const float* Src = static_cast<const float*>(Mapped);
		for (int32 y = 0; y < H; ++y)
		{
			FMemory::Memcpy(Frame.Depth.GetData() + static_cast<int64>(y) * W,
				Src + static_cast<int64>(y) * RowPitchPixels, W * sizeof(float));
		}
		return;
	}

	if (RenderIsSegmentationMode(Mode))
	{
		// The only path that DECODES rather than copies. The staging surface holds
		// PF_FloatRGBA -- the id as three byte lanes written by an unlit material
		// into scene colour -- and what leaves here is the (objid, objtype) pair a
		// client actually wants. Doing it on the render thread while the rows are
		// already mapped costs one pass over pixels that are being touched anyway,
		// and means the encoding never escapes this file.
		Frame.Seg.Reserve(ElementCount);
		Frame.Seg.SetNumUninitialized(ElementCount);
		const FFloat16Color* Src = static_cast<const FFloat16Color*>(Mapped);
		// urlab.Cam.SegDiag: what the lanes ACTUALLY held, before decoding.
		// An all-background id image has several possible causes that look
		// identical from the client -- nothing rendered, the value was scaled out
		// of the 0..255 lane range, the wrong surface was read -- and the raw
		// range separates them in one line.
		const bool bSegDiag = CVarCamSegDiag.GetValueOnAnyThread() != 0;
		float LoR = TNumericLimits<float>::Max(), HiR = TNumericLimits<float>::Lowest();
		float LoG = LoR, HiG = HiR, LoB = LoR, HiB = HiR;
		int32 Covered = 0;
		for (int32 y = 0; y < H; ++y)
		{
			const FFloat16Color* Row = Src + static_cast<int64>(y) * RowPitchPixels;
			int32* Out = Frame.Seg.GetData() + static_cast<int64>(y) * W * 2;
			for (int32 x = 0; x < W; ++x)
			{
				const float R = Row[x].R.GetFloat();
				const float G = Row[x].G.GetFloat();
				const float B = Row[x].B.GetFloat();
				const float Lanes[3] = {R, G, B};
				uint32 Encoded = 0;
				bool bValid = true;
				for (int32 Lane = 0; Lane < 3; ++Lane)
				{
					const float Rounded = FMath::RoundToFloat(Lanes[Lane]);
					if (Rounded < 0.0f || Rounded > 255.0f) { bValid = false; break; }
					Encoded |= static_cast<uint32>(Rounded) << (8 * Lane);
				}
				const int32 ObjId = bValid && Encoded != 0u ? static_cast<int32>(Encoded - 1u) : -1;
				Out[2 * x] = ObjId;
				// The type is meaningless without an object, and a caller
				// filtering on objtype must not match empty space.
				Out[2 * x + 1] = (ObjId >= 0) ? SegmentationObjectType : -1;
				if (bSegDiag)
				{
					LoR = FMath::Min(LoR, R); HiR = FMath::Max(HiR, R);
					LoG = FMath::Min(LoG, G); HiG = FMath::Max(HiG, G);
					LoB = FMath::Min(LoB, B); HiB = FMath::Max(HiB, B);
					Covered += (ObjId >= 0) ? 1 : 0;
				}
			}
		}
		if (bSegDiag)
		{
			UE_LOG(LogURLabNet, Log,
				TEXT("[segdiag] %dx%d covered=%d/%d lanes R[%.4f..%.4f] G[%.4f..%.4f] B[%.4f..%.4f]"),
				W, H, Covered, W * H, LoR, HiR, LoG, HiG, LoB, HiB);
			// A scanline across the middle, so "uniform background" and "ids in the
			// wrong place" stop looking the same in the log.
			const FFloat16Color* Mid = Src + static_cast<int64>(H / 2) * RowPitchPixels;
			FString Line;
			for (int32 i = 0; i < 8; ++i)
			{
				const int32 x = static_cast<int32>((static_cast<int64>(W) * i) / 8 + W / 16);
				Line += FString::Printf(TEXT(" (%.3f,%.3f,%.3f,%.3f)"),
					Mid[x].R.GetFloat(), Mid[x].G.GetFloat(), Mid[x].B.GetFloat(), Mid[x].A.GetFloat());
			}
			UE_LOG(LogURLabNet, Log, TEXT("[segdiag] midrow:%s"), *Line);
		}
		return;
	}

	Frame.Color.Reserve(ElementCount);
	Frame.Color.SetNumUninitialized(ElementCount);
	const FColor* Src = static_cast<const FColor*>(Mapped);
	for (int32 y = 0; y < H; ++y)
	{
		FMemory::Memcpy(Frame.Color.GetData() + static_cast<int64>(y) * W,
			Src + static_cast<int64>(y) * RowPitchPixels, W * sizeof(FColor));
	}
}

void URenderCameraComponent::ApplySegmentationShowFlags()
{
	if (!CaptureComponent)
	{
		return;
	}
	// Everything here exists to stop the renderer improving the picture.
	//
	// An id image is not a picture: a pixel is either exactly one object's id or
	// it is worthless. Any effect that MIXES neighbouring pixels invents ids that
	// belong to no object -- an anti-aliased edge between geom 3 and geom 7
	// decodes to geom 5 with total confidence -- and any effect that ADDS light
	// pushes a lane out of its 0..255 range, where the decoder gives up and
	// reports background. So the whole beautification pipeline is off, and the
	// decoder's range check (MjSegIdFromLanes) is the backstop for anything that
	// slips through.
	FEngineShowFlags& Flags = CaptureComponent->ShowFlags;

	// Blending across pixels: the ones that would fabricate ids.
	Flags.SetTemporalAA(false);
	Flags.SetAntiAliasing(false);
	Flags.SetMotionBlur(false);
	Flags.SetDepthOfField(false);

	// Adding light: the ones that would push a lane out of range.
	Flags.SetBloom(false);
	Flags.SetLocalExposure(false);
	Flags.SetEyeAdaptation(false);
	Flags.SetToneCurve(false);
	Flags.SetColorGrading(false);
	Flags.SetVignette(false);
	Flags.SetGrain(false);
	Flags.SetScreenSpaceReflections(false);
	Flags.SetLumenReflections(false);
	Flags.SetLumenGlobalIllumination(false);
	Flags.SetAmbientOcclusion(false);
	Flags.SetScreenSpaceAO(false);

	// Drawing over the siblings: fog and sky are not primitives, so ShowOnly does
	// not exclude them and they would otherwise paint the background -- and blend
	// with the ids in front of it.
	Flags.SetFog(false);
	Flags.SetVolumetricFog(false);
	Flags.SetAtmosphere(false);
	Flags.SetCloud(false);
	Flags.SetSkyLighting(false);

	// Belt and braces on the post-process side, because a post-process VOLUME in
	// the level can re-enable what a show flag turned off.
	FPostProcessSettings& Post = CaptureComponent->PostProcessSettings;
	Post.bOverride_BloomIntensity = true;
	Post.BloomIntensity = 0.0f;
	Post.bOverride_AutoExposureMethod = true;
	Post.AutoExposureMethod = EAutoExposureMethod::AEM_Manual;
	Post.bOverride_AutoExposureBias = true;
	Post.AutoExposureBias = 0.0f;
	Post.bOverride_SceneFringeIntensity = true;
	Post.SceneFringeIntensity = 0.0f;
	Post.bOverride_AmbientOcclusionIntensity = true;
	Post.AmbientOcclusionIntensity = 0.0f;
}

void URenderCameraComponent::SetupRenderTarget()
{
	const FIntPoint Res = CaptureResolution();
	int32 ElementCount = 0;
	int64 LogicalBytes = 0;
	int64 PhysicalBytes = 0;
	FString SizingError;
	if (!RenderCheckedCameraBufferSizing(CaptureMode, Res, ElementCount,
		LogicalBytes, PhysicalBytes, SizingError))
	{
		UE_LOG(LogURLabImport, Warning,
			TEXT("[MjCamera] Refusing invalid render target configuration: %s"),
			*SizingError);
		return;
	}
	(void)ElementCount;
	(void)LogicalBytes;
	(void)PhysicalBytes;
	UTextureRenderTarget2D* RT = NewObject<UTextureRenderTarget2D>(this);

	// The pixel format IS the mode. Three answers, and each one is the only format
	// that can carry what its mode means:
	//
	//   Depth -- one float of linear scene depth per pixel.
	//   Seg   -- FLOAT RGBA, because the id arrives as three byte lanes written
	//            into scene colour and an 8-bit target would saturate every lane
	//            above 1.0 to white. Half precision is exact for 0..255, which is
	//            what the lanes are and why they are lanes (see MjSegId.h).
	//   Real  -- 8-bit BGRA, already tone-mapped; anything wider is wasted bytes.
	if (CaptureMode == ERenderCameraMode::Depth)
	{
		RT->RenderTargetFormat = ETextureRenderTargetFormat::RTF_R32f;
		RT->InitCustomFormat(Res.X, Res.Y, PF_R32_FLOAT, /*bForceLinearGamma=*/true);
	}
	else if (RenderIsSegmentationMode(CaptureMode))
	{
		RT->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA16f;
		RT->InitCustomFormat(Res.X, Res.Y, PF_FloatRGBA, /*bForceLinearGamma=*/true);
	}
	else
	{
		RT->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
		RT->InitCustomFormat(Res.X, Res.Y, PF_B8G8R8A8, /*bForceLinearGamma=*/true);
	}

	RT->bGPUSharedFlag = true;
	if (GEngine)
	{
		RT->TargetGamma = GEngine->GetDisplayGamma();
	}

	CaptureComponent->TextureTarget = RT;
	CaptureComponent->bAlwaysPersistRenderingState = true;
	CaptureComponent->bRenderInMainRenderer = bRenderInMainRenderer;
	CaptureComponent->MaxViewDistanceOverride = -1.0f;

	// Explicitly disable motion blur
	CaptureComponent->ShowFlags.SetMotionBlur(false);
	CaptureComponent->PostProcessSettings.bOverride_MotionBlurAmount = true;
	CaptureComponent->PostProcessSettings.MotionBlurAmount = 0.0f;
	CaptureComponent->PostProcessSettings.bOverride_MotionBlurMax = true;
	CaptureComponent->PostProcessSettings.MotionBlurMax = 0.0f;
	CaptureComponent->PostProcessSettings.bOverride_MotionBlurPerObjectSize = true;
	CaptureComponent->PostProcessSettings.MotionBlurPerObjectSize = 0.0f;

	// -URLabScene=noexposure disables auto-exposure/eye-adaptation.
	if (CVarCamDisableAutoExposure.GetValueOnGameThread() != 0)
	{
		CaptureComponent->ShowFlags.SetEyeAdaptation(false);
		CaptureComponent->PostProcessSettings.bOverride_AutoExposureMethod = true;
		CaptureComponent->PostProcessSettings.AutoExposureMethod = EAutoExposureMethod::AEM_Manual;
		CaptureComponent->PostProcessSettings.bOverride_AutoExposureBias = true;
		CaptureComponent->PostProcessSettings.AutoExposureBias = 0.0f;
	}

	float NearClipCm = CustomNearClipCm > 0.0f ? CustomNearClipCm : CVarCamNearClip.GetValueOnGameThread();
	// -URLabScene=camnear=<cm> overrides the near clip plane.
	CaptureComponent->bOverride_CustomNearClippingPlane = true;
	CaptureComponent->CustomNearClippingPlane = FMath::Max(0.01f, NearClipCm);

	SetupProjectionMatrix();

	switch (CaptureMode)
	{
		case ERenderCameraMode::Depth:
			CaptureComponent->CaptureSource = ESceneCaptureSource::SCS_SceneDepth;
			break;

		case ERenderCameraMode::SemanticSegmentation:
		case ERenderCameraMode::InstanceSegmentation:
			// SCENE COLOUR HDR, and this is the load-bearing line of the whole
			// segmentation path. The siblings are unlit, so their emissive -- the
			// encoded id -- is written straight into scene colour in the base pass,
			// and scene colour HDR is the last point before tone mapping. Read it
			// here and the number that comes back is the number that went in.
			//
			// It used to be SCS_FinalToneCurveHDR, chosen because SCS_BaseColor
			// rendered empty against a material whose Color parameter was not wired
			// to the BaseColor G-buffer. That is true and it was the wrong
			// conclusion: the fix is a material that writes what we want to read,
			// not a source that reads after the tone curve has moved it. Every
			// segmentation pixel was tone-mapped before it reached the client.
			{
				static const ESceneCaptureSource kSegSources[] = {
					ESceneCaptureSource::SCS_SceneColorHDR,
					ESceneCaptureSource::SCS_FinalColorLDR,
					ESceneCaptureSource::SCS_BaseColor,
					ESceneCaptureSource::SCS_FinalToneCurveHDR,
					ESceneCaptureSource::SCS_SceneColorHDRNoAlpha,
				};
				const int32 Pick = FMath::Clamp(CVarCamSegSource.GetValueOnGameThread(),
					0, static_cast<int32>(UE_ARRAY_COUNT(kSegSources)) - 1);
				CaptureComponent->CaptureSource = kSegSources[Pick];
			}
			CaptureComponent->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;
			if (CVarCamSegShowFlags.GetValueOnGameThread() != 0)
			{
				ApplySegmentationShowFlags();
			}
			break;

		case ERenderCameraMode::Real:
		default:
			// LDR final colour: already tone-mapped and gamma-encoded to the same sRGB
			// the viewport shows, so the BGRA8 readback matches the editor image. The
			// readback skips TargetGamma, which is why an HDR-then-encode source
			// delivered a darker frame. It also saves a conversion and a copy.
			CaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
			CaptureComponent->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_LegacySceneCapture;
			break;
	}
	ApplyModeVisibilityComponents();

	RenderTarget = RT;
	UE_LOG(LogURLabImport, Log, TEXT("[MjCamera] '%s' RT created mode=%s (%dx%d)"),
		*CameraLogName(*this), *UEnum::GetValueAsString(CaptureMode), Res.X, Res.Y);
}



void URenderCameraComponent::RegisterWithStreamingManager()
{
	// Register as an active viewpoint, so textures stream for this camera's
	// frustum even when the player pawn is far away.
	const FIntPoint Res = CaptureResolution();
	const float HFov = CaptureComponent ? CaptureComponent->FOVAngle : DerivedFovy;
	const float Distance = (HFov > 0.0f)
							 ? Res.X / FMath::Tan(FMath::DegreesToRadians(HFov * 0.5f))
							 : 1000.0f;

	IStreamingManager::Get().AddViewInformation(
		GetComponentLocation(),
		Res.X,
		Distance,
		StreamingBoost,
		/*bOverrideLocation=*/false,
		/*Duration=*/0.0f,
		GetOwner());
}

bool URenderCameraComponent::ApplyCaptureMode(ERenderCameraMode Mode)
{
	if (CaptureMode == Mode)
	{
		// The whole point of the guard: a client that names its mode on every
		// render frame must not pay a render-target rebuild per frame.
		return false;
	}

	if (!bStreamingEnabled)
	{
		// Dormant camera: the mode is simply what the first SetupRenderTarget
		// will read.
		AdvanceConfigurationRevision();
		CaptureMode = Mode;
		Descriptor.Capture.Mode = Mode;
		return true;
	}

	// Live camera. The disable half releases the seg pool while CaptureMode still
	// says what we subscribed as, and nulls RenderTarget so the enable half
	// rebuilds it in the mode set in between.
	AdvanceConfigurationRevision();
	SetStreamingEnabled(false);
	CaptureMode = Mode;
	Descriptor.Capture.Mode = Mode;
	SetStreamingEnabled(true);

	UE_LOG(LogURLabImport, Log, TEXT("[MjCamera] '%s' capture mode -> %s (render target rebuilt)."),
		*CameraLogName(*this), TEXT("render"));
	return true;
}

void URenderCameraComponent::SetStreamingEnabled(bool bEnable)
{
	if (bEnable == bStreamingEnabled) return;
	if (bEnable)
	{
		if (!RenderTarget) SetupRenderTarget();
		if (!RenderTarget || !CaptureComponent) return;
		RefreshCaptureFov();
		CaptureComponent->SetVisibility(bSceneVisible);
		CaptureComponent->SetActive(true);
		CaptureComponent->bHiddenInGame = !bSceneVisible;
		CaptureComponent->bCaptureEveryFrame = bRenderInMainRenderer;
		CaptureComponent->bCaptureOnMovement = false;
		bStreamingEnabled = true;
		CaptureComponent->CaptureScene();
		return;
	}
	bStreamingEnabled = false;
	InvalidateExactCaptures(ERenderCameraCaptureOutcome::Teardown,
		TEXT("camera streaming stopped before exact capture completion"));
	IdentityGeneration.fetch_add(1, std::memory_order_acq_rel);
	if (CaptureComponent)
	{
		CaptureComponent->bCaptureEveryFrame = false;
		CaptureComponent->SetVisibility(false);
		CaptureComponent->TextureTarget = nullptr;
	}
	for (FInFlightReadback& Readback : InFlightReadbacks)
	{
		FDetachedCameraReadbackDrain::Get().Retain(
			MoveTemp(Readback.Gpu), MoveTemp(Readback.ExactState), DetachedWriterCount);
	}
	InFlightReadbacks.Empty();
	DrainCompletedFrames();
	FreeReadbacks.Empty();
	RenderTarget = nullptr;
}

// ---------------------------------------------------------------------------
// Async readback
// ---------------------------------------------------------------------------

void URenderCameraComponent::RequestReadback()
{
	EnqueueReadback(AppliedIdentity);
}

bool URenderCameraComponent::EnqueueReadback(const FRenderFrameIdentity& ShowIdentity,
	bool bForceSubmit, FRenderCameraCaptureReceipt* OutExactReceipt,
	ERenderCameraCaptureIssue* OutIssue,
	FRenderCameraCaptureLifetimeLeasePtr LifetimeLease)
{
	if (OutIssue) *OutIssue = ERenderCameraCaptureIssue::RetryableNotReady;
	int32 ElementCount = 0;
	int64 LogicalBytes = 0;
	int64 PhysicalBytes = 0;
	FString SizingError;
	if (!RenderCheckedCameraBufferSizing(CaptureMode, CaptureResolution(), ElementCount,
		LogicalBytes, PhysicalBytes, SizingError))
	{
		if (OutIssue) *OutIssue = ERenderCameraCaptureIssue::InvalidConfiguration;
		return false;
	}
	(void)ElementCount;
	(void)LogicalBytes;
	(void)PhysicalBytes;
	if (!RenderTarget || !CanCaptureIdentity(ShowIdentity))
	{
		if (OutIssue && !CanCaptureIdentity(ShowIdentity))
			*OutIssue = ERenderCameraCaptureIssue::InvalidIdentity;
		return false;
	}
	// Pipeline cap: keep at most MaxInFlightReadbacks async copies outstanding.
	const int32 DetachedWriters = DetachedWriterCount.IsValid()
		? DetachedWriterCount->load(std::memory_order_relaxed) : 0;
	if (InFlightReadbacks.Num() + PendingMapCommands + DetachedWriters
		>= MaxInFlightReadbacks)
	{
		if (OutIssue) *OutIssue = ERenderCameraCaptureIssue::RetryableBusy;
		return false;
	}

	FTextureRenderTargetResource* Resource = RenderTarget->GameThread_GetRenderTargetResource();
	if (!Resource)
	{
		return false;
	}

	const FIntPoint Size = Resource->GetSizeXY();
	if (Size.X <= 0 || Size.Y <= 0)
	{
		// Not sized yet: the allocation is in flight on the render thread.
		return false;
	}
	if (Size != CaptureResolution())
	{
		if (OutIssue) *OutIssue = ERenderCameraCaptureIssue::InvalidConfiguration;
		return false;
	}

	// A freshly enabled render target reports its size on the game thread before
	// its RHI texture is created on the render thread. Copying from a null texture
	// would stage nothing and yield a black frame that then latches in history, so
	// wait until the texture exists and let the caller retry next tick.
	FTextureRHIRef SourceTexture = Resource->GetRenderTargetTexture();
	if (!SourceTexture.IsValid())
	{
		return false;
	}
	if (NextCaptureSerial == MAX_uint64)
	{
		if (OutIssue) *OutIssue = ERenderCameraCaptureIssue::InvalidConfiguration;
		return false;
	}

	FInFlightReadback Entry;
	if (FreeReadbacks.Num() > 0)
	{
		Entry.Gpu = FreeReadbacks.Pop(EAllowShrinking::No);
	}
	else
	{
		Entry.Gpu = MakeShared<FRHIGPUTextureReadback, ESPMode::ThreadSafe>(
			TEXT("MjCameraReadback"));
	}
	Entry.Width = Size.X;
	Entry.Height = Size.Y;
	Entry.Mode = CaptureMode;
	Entry.ConfigurationRevision = ConfigurationRevision;
	Entry.CaptureSerial = NextCaptureSerial++;
	// Unix-epoch capture time for the wire meta, the same clock as the state
	// stream's wall_time and Python's time.time().
	Entry.CaptureUnixSeconds = (FDateTime::UtcNow() - FDateTime(1970, 1, 1)).GetTotalSeconds();
	// Monotonic stamp on the same clock as the later stage stamps, for copy-out
	// timing. Unix time above is for the wire; this is for stage deltas.
	Entry.TIssuedSeconds = FPlatformTime::Seconds();
	// The post-step state the harvested pixels show: for a manual capture that is
	// the current applied id; in every-frame mode the caller passes the id whose
	// render the target currently holds.
	Entry.Identity = ShowIdentity;
	Entry.IdentityGeneration = IdentityGeneration.load(std::memory_order_acquire);
	Entry.SegmentationObjectType = SegmentationObjectType;
	if (OutExactReceipt)
	{
		Entry.ExactState = MakeShared<FRenderCameraExactCaptureState, ESPMode::ThreadSafe>();
		Entry.ExactState->LifetimeLease = MoveTemp(LifetimeLease);
		OutExactReceipt->CameraName = CameraLogName(*this);
		OutExactReceipt->Identity = ShowIdentity;
		OutExactReceipt->Mode = Entry.Mode;
		OutExactReceipt->Resolution = Size;
		OutExactReceipt->ConfigurationRevision = Entry.ConfigurationRevision;
		OutExactReceipt->CaptureSerial = Entry.CaptureSerial;
		OutExactReceipt->State = Entry.ExactState;
		ExactCaptures.Add(Entry.ExactState);
	}

	// Schedule the GPU-to-staging copy without stalling the render thread.
	// SourceTexture is captured as a ref-counted handle, so a render target
	// reallocation between enqueue and execution cannot copy against a released
	// texture: the reference keeps the RHI texture alive until the command runs.
	TSharedPtr<FRHIGPUTextureReadback, ESPMode::ThreadSafe> Readback = Entry.Gpu;
	ENQUEUE_RENDER_COMMAND(MjCameraEnqueueReadback)
	([Readback, SourceTexture, bForceSubmit](FRHICommandListImmediate& RHICmdList) {
		Readback->EnqueueCopy(RHICmdList, SourceTexture.GetReference());
		if (bForceSubmit)
		{
			// Dispatch the copy to the RHI thread now, so the readback fence can
			// signal mid-frame rather than quantizing to the next frame boundary.
			// This is a render-thread submit, not a game-thread flush, so it neither
			// stalls the game thread nor re-enters the harvest.
			RHICmdList.ImmediateFlush(EImmediateFlushType::DispatchToRHIThread);
		}
	});

	InFlightReadbacks.Add(MoveTemp(Entry));
	if (OutIssue) *OutIssue = ERenderCameraCaptureIssue::Accepted;
	return true;
}

// ---------------------------------------------------------------------------
// Frame history
// ---------------------------------------------------------------------------

void FRenderCameraHistory::PushShared(const FRenderCameraFramePtr& Frame,
	int32 Capacity, bool bDelayActive, double RetainWindow, bool bUseWallClock)
{
	if (!Frame.IsValid())
	{
		return;
	}

	FScopeLock ScopeLock(&Lock);
	Frames.Add(Frame);

	// A hard frame ceiling regardless of mode: one frame can be megabytes and many
	// cameras share the budget, so bound worst-case retention. It is the same
	// constant that backs HistoryCapacity's ClampMax, so the two limits are one
	// source of truth and cannot disagree.
	constexpr int32 HardCap = MaxHistoryCapacity;

	if (!bDelayActive)
	{
		const int32 Cap = FMath::Clamp(Capacity, 1, HardCap);
		while (Frames.Num() > Cap)
		{
			Frames.RemoveAt(0);
		}
		return;
	}

	// Latency emulation retains enough history to cover the delay window, so the
	// reveal-time selection always has the frame it needs. A front frame is evicted
	// only once it is older than the window behind the newest, which is self-sizing
	// and independent of frame rate, and still capped by the memory ceiling.
	const double NewestClock = FRenderCameraDelayModel::FrameClock(*Frames.Last(), bUseWallClock);
	while (Frames.Num() > 1)
	{
		const bool bExpired = (NewestClock - FRenderCameraDelayModel::FrameClock(*Frames[0], bUseWallClock)) > RetainWindow;
		if (Frames.Num() > HardCap || bExpired)
		{
			Frames.RemoveAt(0);
		}
		else
		{
			break;
		}
	}
}

FRenderCameraFramePtr FRenderCameraHistory::GetShared(uint64 MinFrameId) const
{
	FScopeLock ScopeLock(&Lock);
	if (Frames.Num() == 0)
	{
		return nullptr;
	}
	if (MinFrameId == 0)
	{
		return Frames.Last();
	}
	// The oldest retained frame at or after the requested step (history is oldest
	// first), i.e. the frame that shows state at least MinFrameId.
	for (const FRenderCameraFramePtr& Frame : Frames)
	{
		if (Frame->Identity.FrameId >= MinFrameId)
		{
			return Frame;
		}
	}
	return nullptr;
}

uint64 FRenderCameraHistory::GetLatestFrameId() const
{
	FScopeLock ScopeLock(&Lock);
	return Frames.Num() > 0 ? Frames.Last()->Identity.FrameId : 0;
}

FRenderCameraFramePtr FRenderCameraHistory::SelectDelayedShared(double NowValue, uint64 AfterSeq) const
{
	FScopeLock ScopeLock(&Lock);
	for (int32 i = Frames.Num() - 1; i >= 0; --i)
	{
		if (Frames[i]->RevealValue <= NowValue)
		{
			// The newest eligible frame. Deliver it only if it is newer than the last
			// one published, so the stream never repeats or rewinds.
			if (Frames[i]->Seq > AfterSeq)
			{
				return Frames[i];
			}
			return nullptr;
		}
	}
	return nullptr;
}

FRenderCameraFramePtr FRenderCameraHistory::SelectDelayedByClock(double TargetClock, bool bUseWallClock) const
{
	FScopeLock ScopeLock(&Lock);
	if (Frames.Num() == 0)
	{
		return nullptr;
	}
	for (int32 i = Frames.Num() - 1; i >= 0; --i)
	{
		const double Clock = FRenderCameraDelayModel::FrameClock(*Frames[i], bUseWallClock);
		if (Clock <= TargetClock + 1e-6)
		{
			return Frames[i];
		}
	}
	// Every retained frame is newer than the requested observation time. Returning
	// one would make a delayed read observe the future; report unavailable until
	// history reaches the target instead.
	return nullptr;
}

void FRenderCameraHistory::Clear()
{
	FScopeLock ScopeLock(&Lock);
	Frames.Reset();
}

void URenderCameraComponent::ClearHistory()
{
	FScopeLock IdentityGate(&IdentityGateLock);
	History.Clear();
}

void URenderCameraComponent::PushFrameToHistory(FRenderCameraFrame&& Frame)
{
	FScopeLock IdentityGate(&IdentityGateLock);
	if (!CanCaptureIdentity(Frame.Identity)
		|| Frame.Identity.SceneRevision != AppliedIdentity.SceneRevision
		|| Frame.Identity.ClockEpoch != AppliedIdentity.ClockEpoch)
	{
		return;
	}
	FMutableRenderCameraFramePtr Shared = MakeShared<FRenderCameraFrame, ESPMode::ThreadSafe>(MoveTemp(Frame));
	PushFrameToHistoryShared(Shared);
}

void URenderCameraComponent::PushFrameToHistoryShared(
	const FRenderCameraFramePtr& Frame, uint64 ExpectedIdentityGeneration)
{
	if (!Frame.IsValid() || !CanCaptureIdentity(Frame->Identity)
		|| (ExpectedIdentityGeneration != 0
			&& !IsIdentityGenerationCurrent(ExpectedIdentityGeneration)))
	{
		return;
	}
	const double RetainWindow = static_cast<double>(DelaySeconds + DelayJitterSeconds) + 0.10;
	History.PushShared(Frame, HistoryCapacity, IsDelayActive(), RetainWindow, bDelayUseWallClock);
}

bool URenderCameraComponent::IsIdentityGenerationCurrent(uint64 ExpectedIdentityGeneration) const
{
	return ExpectedIdentityGeneration != 0
		&& IdentityGeneration.load(std::memory_order_acquire) == ExpectedIdentityGeneration;
}

bool URenderCameraComponent::CanCaptureIdentity(const FRenderFrameIdentity& Identity) const
{
	return !IsDelayActive() || bDelayUseWallClock || Identity.bHasSourceTime;
}

FRenderCameraFramePtr URenderCameraComponent::GetFrameShared(uint64 MinFrameId) const
{
	FScopeLock IdentityGate(&IdentityGateLock);
	return History.GetShared(MinFrameId);
}

bool URenderCameraComponent::GetFrame(uint64 MinFrameId, FRenderCameraFrame& Out) const
{
	FRenderCameraFramePtr Frame = GetFrameShared(MinFrameId);
	if (!Frame.IsValid())
	{
		return false;
	}
	Out = *Frame;
	return true;
}

FRenderCameraFramePtr URenderCameraComponent::GetFrameForRequest(uint64 MinFrameId, bool bIgnoreDelay) const
{
	FScopeLock IdentityGate(&IdentityGateLock);
	if (!bIgnoreDelay && IsDelayActive())
	{
		if (!bDelayUseWallClock && !AppliedIdentity.bHasSourceTime) return nullptr;
		const double EffectiveDelay = const_cast<URenderCameraComponent*>(this)->SampleDelaySeconds();
		if (EffectiveDelay > 0.0)
		{
			const double NowClock = NowClockValue();
			return History.SelectDelayedByClock(NowClock - EffectiveDelay, bDelayUseWallClock);
		}
	}
	return History.GetShared(MinFrameId);
}

FRenderCameraFramePtr URenderCameraComponent::GetFrameForRequest(uint64 MinFrameId, double InDelaySeconds) const
{
	FScopeLock IdentityGate(&IdentityGateLock);
	if (InDelaySeconds > 0.0)
	{
		if (!bDelayUseWallClock && !AppliedIdentity.bHasSourceTime) return nullptr;
		const double NowClock = NowClockValue();
		return History.SelectDelayedByClock(NowClock - InDelaySeconds, bDelayUseWallClock);
	}
	if (IsDelayActive())
	{
		if (!bDelayUseWallClock && !AppliedIdentity.bHasSourceTime) return nullptr;
		const double EffectiveDelay = const_cast<URenderCameraComponent*>(this)->SampleDelaySeconds();
		if (EffectiveDelay > 0.0)
		{
			return History.SelectDelayedByClock(NowClockValue() - EffectiveDelay,
				bDelayUseWallClock);
		}
	}
	return History.GetShared(MinFrameId);
}

uint64 URenderCameraComponent::GetLatestFrameId() const
{
	FScopeLock IdentityGate(&IdentityGateLock);
	return History.GetLatestFrameId();
}

FRenderCameraFramePtr URenderCameraComponent::SelectDelayedFrameShared(double NowValue, uint64 AfterSeq) const
{
	FScopeLock IdentityGate(&IdentityGateLock);
	if (IsDelayActive() && !bDelayUseWallClock && !AppliedIdentity.bHasSourceTime)
	{
		return nullptr;
	}
	return History.SelectDelayedShared(NowValue, AfterSeq);
}

bool URenderCameraComponent::SelectDelayedFrame(double NowValue, uint64 AfterSeq, FRenderCameraFrame& Out) const
{
	FRenderCameraFramePtr Frame = SelectDelayedFrameShared(NowValue, AfterSeq);
	if (!Frame.IsValid())
	{
		return false;
	}
	Out = *Frame;
	return true;
}

// ---------------------------------------------------------------------------
// Latency emulation and capture-rate control
// ---------------------------------------------------------------------------

double FRenderCameraDelayModel::FrameClock(const FRenderCameraFrame& Frame, bool bUseWallClock)
{
	return bUseWallClock ? Frame.CaptureUnixTime : (Frame.Identity.bHasSourceTime ? Frame.Identity.SourceTime : 0.0);
}

double FRenderCameraDelayModel::NowClockValue(bool bUseWallClock, double AppliedSourceTime)
{
	if (bUseWallClock)
	{
		return (FDateTime::UtcNow() - FDateTime(1970, 1, 1)).GetTotalSeconds();
	}
	return AppliedSourceTime;
}

bool FRenderCameraDelayModel::IsDelayActive(float DelaySeconds, float JitterSeconds)
{
	return DelaySeconds > 0.0f || JitterSeconds > 0.0f;
}

double FRenderCameraDelayModel::SampleDelaySeconds(float DelaySeconds, float JitterSeconds)
{
	double D = static_cast<double>(DelaySeconds);
	if (JitterSeconds > 0.0f)
	{
		// Draw from the RNG only when jitter is configured, so a fixed delay stays
		// deterministic and does not advance the stream.
		const double J = static_cast<double>(JitterSeconds);
		D += Rng.FRandRange(-J, J);
	}
	return FMath::Max(0.0, D);
}

void FRenderCameraDelayModel::Configure(int32 Seed)
{
	Rng.Initialize(Seed);
	// Re-arm the publish dedup, so the new policy re-selects cleanly.
	LastPublishedSeq = 0;
}

double URenderCameraComponent::FrameClock(const FRenderCameraFrame& Frame) const
{
	return FRenderCameraDelayModel::FrameClock(Frame, bDelayUseWallClock);
}

double URenderCameraComponent::NowClockValue() const
{
	const double AppliedTime = AppliedIdentity.bHasSourceTime ? AppliedIdentity.SourceTime : 0.0;
	return FRenderCameraDelayModel::NowClockValue(bDelayUseWallClock, AppliedTime);
}

bool URenderCameraComponent::IsDelayActive() const
{
	return FRenderCameraDelayModel::IsDelayActive(DelaySeconds, DelayJitterSeconds);
}

double URenderCameraComponent::SampleDelaySeconds()
{
	return Delay.SampleDelaySeconds(DelaySeconds, DelayJitterSeconds);
}

void URenderCameraComponent::SetCameraDelay(float InDelaySeconds, float InJitterSeconds, bool bInUseWallClock, int32 InSeed)
{
	const float NewDelaySeconds = FMath::Max(0.0f, InDelaySeconds);
	const float NewJitterSeconds = FMath::Max(0.0f, InJitterSeconds);
	const int32 DefaultSeed = static_cast<int32>(GetTypeHash(Descriptor.CanonicalName));
	const int32 Seed = (InSeed != 0) ? InSeed : DefaultSeed;
	const int32 CurrentSeed = Descriptor.Capture.DelaySeed != 0
		? Descriptor.Capture.DelaySeed : DefaultSeed;
	if (DelaySeconds == NewDelaySeconds && DelayJitterSeconds == NewJitterSeconds
		&& bDelayUseWallClock == bInUseWallClock && CurrentSeed == Seed)
	{
		return;
	}
	AdvanceConfigurationRevision();
	DelaySeconds = NewDelaySeconds;
	DelayJitterSeconds = NewJitterSeconds;
	bDelayUseWallClock = bInUseWallClock;
	Descriptor.Capture.DelaySeconds = NewDelaySeconds;
	Descriptor.Capture.DelayJitterSeconds = NewJitterSeconds;
	Descriptor.Capture.bDelayUseWallClock = bInUseWallClock;
	Descriptor.Capture.DelaySeed = InSeed;
	Delay.Configure(Seed);
}

void URenderCameraComponent::SetCaptureRate(bool bInOnStateChange, float InMaxFps)
{
	const float ClampedMaxFps = FMath::Max(0.0f, InMaxFps);
	if (bCaptureOnStateChange == bInOnStateChange && CaptureMaxFps == ClampedMaxFps)
		return;
	AdvanceConfigurationRevision();
	bCaptureOnStateChange = bInOnStateChange;
	CaptureMaxFps = ClampedMaxFps;
}

// ---------------------------------------------------------------------------
// Publishing and identity
// ---------------------------------------------------------------------------



void URenderCameraComponent::EmitFrame(const FRenderCameraFramePtr& Frame)
{
	if (!Frame) return;
	OnSharedFrameCompleted.Broadcast(Frame);
	OnFrameCompleted.Broadcast(*Frame);
}

void URenderCameraComponent::TouchRequested()
{
	LastRequestedSeconds.store(FPlatformTime::Seconds(), std::memory_order_release);
}

bool URenderCameraComponent::IsCaptureActive() const
{
	if (bContinuousCapture) return true;
	const double Last = LastRequestedSeconds.load(std::memory_order_acquire);
	return Last > 0.0 && (FPlatformTime::Seconds() - Last) <= RequestActiveTtlSeconds;
}
