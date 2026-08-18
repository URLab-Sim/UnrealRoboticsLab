// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Elements/MjCamera.h"

#include "ContentStreaming.h"
#include "Engine/Engine.h"
#include "Engine/PostProcessVolume.h"
#include "EngineUtils.h"
#include "HAL/IConsoleManager.h"
#include "Misc/CoreDelegates.h"
#include "RHICommandList.h"
#include "RenderingThread.h"

#include "Bridge/BridgeServer.h"
#include "Bridge/BridgeServerConfigUtils.h"
#include "MuJoCo/Capture/MjCameraFrameBus.h"
#include "MuJoCo/Capture/MjCameraSubsystem.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjSimClock.h"
#include "MuJoCo/Fast/MjRenderer.h"
#include "State/MjCanonicalName.h"
#include "Transport/CameraPublishTransport.h"
#include "Transport/MjExternalTransportProvider.h"
#include "Transport/NetworkManager.h"
#include "Transport/ShmCameraPublishTransport.h"
#include "Transport/ZmqCameraPublishTransport.h"
#include "Utils/URLabLogging.h"

namespace
{
bool IsSegMode(EMjCameraMode Mode)
{
	return Mode == EMjCameraMode::SemanticSegmentation
		|| Mode == EMjCameraMode::InstanceSegmentation;
}

/**
 * MuJoCo's `fovy` is the VERTICAL field of view; USceneCaptureComponent2D's
 * FOVAngle is the HORIZONTAL one. Copying fovy across verbatim over-narrows the
 * view on a non-square target, which reads as "zoomed in", so convert through
 * the target's aspect and UE's derived vertical fov matches fovy exactly.
 * Identity on a square target.
 */
float HorizontalFOVFromFovy(float Fovy, FIntPoint Resolution)
{
	// A camera with no fovy produces a degenerate zero-fov frustum that renders
	// pure black. Fall back to MuJoCo's default vertical fov.
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
 * An authored `fovy` wins. Failing that, MuJoCo's compiler derives one from
 * whichever intrinsics family the camera declares -- focal pixels against the
 * resolution, or focal length against the sensor size -- so the same derivation
 * happens here, or a camera specified by intrinsics renders at the wrong field
 * of view. The result stays a derived value: writing it back into `fovy` would
 * turn an intrinsics-specified camera into a fov-specified one and change what
 * the spec means.
 */
float DeriveFovyDegrees(const UMjCamera& Cam)
{
	if (Cam.HasFovy() && Cam.GetFovy() > 0.0)
	{
		return static_cast<float>(Cam.GetFovy());
	}

	const TArray<int32> Res = Cam.HasResolution() ? Cam.GetResolution() : TArray<int32>();
	const TArray<float> FocalPixel = Cam.GetFocalpixel();
	if (FocalPixel.Num() >= 2 && FocalPixel[1] > 0.0f && Res.Num() >= 2 && Res[1] > 0)
	{
		return 2.0f * FMath::Atan2(static_cast<float>(Res[1]), 2.0f * FocalPixel[1]) * (180.0f / PI);
	}

	const TArray<float> Focal = Cam.GetFocal();
	const TArray<float> SensorSize = Cam.GetSensorsize();
	if (Focal.Num() >= 2 && SensorSize.Num() >= 2 && Focal[1] > 0.0f)
	{
		return 2.0f * FMath::Atan2(SensorSize[1], 2.0f * Focal[1]) * (180.0f / PI);
	}

	return 45.0f;
}

// The renderer that owns the geom components a seg camera segments: the compiled
// play view at runtime, or a mirror/raw renderer that draws its own geometry. The
// segmentation sibling pool lives on it, since it owns the meshes the siblings mirror.
AMjRenderer* FindRenderer(UWorld* FallbackWorld = nullptr)
{
	if (AAMjManager* Manager = AAMjManager::GetManager())
	{
		if (AMjRenderer* View = Manager->GetCompiledRenderView())
		{
			return View;
		}
	}
	// Test and editor worlds do not dispatch BeginPlay (the singleton is unset), and a
	// mirror/raw renderer carries no compiled view, so scan the world for a renderer.
	if (FallbackWorld)
	{
		for (TActorIterator<AMjRenderer> It(FallbackWorld); It; ++It)
		{
			if (AMjRenderer* Renderer = *It)
			{
				return Renderer;
			}
		}
	}
	return nullptr;
}

/**
 * A camera's canonical <art>/<part> segments, through the single naming owner.
 * The art segment is the owning articulation's name, or the owning actor's name
 * for a manager-level global camera; the part is the MJCF name (or the
 * component name when unset) with the art prefix stripped.
 */
void ResolveCameraCanonical(const UMjCamera& Cam, FName& OutArt, FName& OutPart)
{
	const AActor* Owner = Cam.GetOwner();
	const AMjArticulation* Art = Cast<AMjArticulation>(Owner);
	OutArt = Art ? FMjCanonicalName::ArtSegment(Art)
				 : FName(*FMjCanonicalName::Sanitize(Owner ? Owner->GetName() : TEXT("unknown")));
	FString Source = Cam.MjName.Get(FString());
	if (Source.IsEmpty())
	{
		Source = Cam.GetName();
	}
	OutPart = FMjCanonicalName::PartSegment(Art, Source);
}

/** What to call a camera in a log line: its MJCF name, or its component name. */
// Copy-out stage timing. When set, UMjCamera decomposes each harvested frame's
// GPU-to-game-thread path (issue -> fence-ready -> mapped -> published) and logs
// a windowed summary. Off by default: a few adds per frame, no wire cost.
static TAutoConsoleVariable<int32> CVarCamDiag(
	TEXT("urlab.Cam.Diag"), 0,
	TEXT("Log camera copy-out stage timing (issue/fence/map/publish). 0=off, 1=on."),
	ECVF_Default);

// SPEAR-style copy-out: harvest readbacks (poll the fence, map, and publish) on
// the render thread at render-frame end, so the path never waits for a game tick.
// A/B against the default game-thread harvest.
static TAutoConsoleVariable<int32> CVarCamRTHarvest(
	TEXT("urlab.Cam.RenderThreadHarvest"), 0,
	TEXT("Harvest camera readbacks on the render thread (SPEAR-style). "
		 "0=game thread (default), 1=render thread."),
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

FString CameraLogName(const UMjCamera& Cam)
{
	return Cam.MjName.Get(Cam.GetName());
}
} // namespace

// ---------------------------------------------------------------------------
// Publish gate
// ---------------------------------------------------------------------------

std::atomic<bool> FCameraZmqWorker::bPublishersPaused{false};

// ---------------------------------------------------------------------------
// Lifetime
// ---------------------------------------------------------------------------

UMjCamera::UMjCamera()
{
	// The capture pipeline is driven once per frame by UMjCameraSubsystem, not by
	// a per-component tick.
	PrimaryComponentTick.bCanEverTick = false;
	PrimaryComponentTick.bStartWithTickEnabled = false;

	ResultsQueue = MakeShared<FCompletedReadbackQueue, ESPMode::ThreadSafe>();

	CaptureComponent = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("SceneCapture"));
	if (CaptureComponent)
	{
		CaptureComponent->SetupAttachment(this);
		CaptureComponent->SetRelativeScale3D(FVector(0.15f));

		// MuJoCo cameras look down -Z with +Y up; Unreal cameras look down +X with
		// +Z up. The MuJoCo-to-Unreal rotation negates the X and Z quaternion
		// components for the handedness flip, which mirrors the Y axis, so "up"
		// becomes -Y after conversion.
		const FVector MjForward = FVector(0.0f, 0.0f, -1.0f);
		const FVector MjUp = FVector(0.0f, -1.0f, 0.0f);
		const FRotator CorrectionRot = FRotationMatrix::MakeFromXZ(MjForward, MjUp).Rotator();
		CaptureComponent->SetRelativeRotation(CorrectionRot);

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

void UMjCamera::OnRegister()
{
	Super::OnRegister();
	// The scene-capture is a default subobject attached in the constructor. When
	// this camera is spawned as part of an actor/Blueprint, RegisterAllComponents
	// attaches + registers it for us. But when it is created at runtime via
	// NewObject + RegisterComponent (the MJB fast-path render server), that cascade
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
}

void UMjCamera::BeginPlay()
{
	Super::BeginPlay();

	// A `<default>` partial is a class, not a camera. The compiler emits no
	// camera for it and nothing can select it by name, so offering it to the
	// streaming registries would put a class in the camera list and let a client
	// ask a class for a frame.
	if (IsClassPartial())
	{
		return;
	}

	if (AAMjManager* Manager = AAMjManager::GetManager())
	{
		if (Manager->NetworkManager)
		{
			Manager->NetworkManager->RegisterCamera(this);
		}
	}

	// Match the viewport look by taking the scene's post-process settings.
	if (CaptureComponent && GetWorld())
	{
		for (TActorIterator<APostProcessVolume> It(GetWorld()); It; ++It)
		{
			APostProcessVolume* Volume = *It;
			if (Volume && Volume->bEnabled)
			{
				CaptureComponent->PostProcessSettings = Volume->Settings;
				CaptureComponent->PostProcessBlendWeight = 1.0f;
				UE_LOG(LogURLab, Log, TEXT("[MjCamera] Copied post-process settings from '%s'"), *Volume->GetName());
				break; // the first enabled volume wins
			}
		}
	}

	if (bEnableZmqBroadcast)
	{
		SetStreamingEnabled(true);
	}

	if (UWorld* World = GetWorld())
	{
		if (UMjCameraSubsystem* Subsystem = World->GetSubsystem<UMjCameraSubsystem>())
		{
			Subsystem->RegisterCamera(this);
		}
	}
}

void UMjCamera::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// Stop + drain the render-thread harvest first: it publishes through the
	// transports SetStreamingEnabled(false) below tears down.
	UnregisterRenderThreadHarvest();

	if (UWorld* World = GetWorld())
	{
		if (UMjCameraSubsystem* Subsystem = World->GetSubsystem<UMjCameraSubsystem>())
		{
			Subsystem->UnregisterCamera(this);
		}
	}

	if (AAMjManager* Manager = AAMjManager::GetManager())
	{
		if (Manager->NetworkManager)
		{
			Manager->NetworkManager->UnregisterCamera(this);
		}
	}

	// Stop rendering and tear down the image-egress transports when the actor is
	// torn down.
	if (bStreamingEnabled)
	{
		SetStreamingEnabled(false);
	}
	Super::EndPlay(EndPlayReason);
}

// ---------------------------------------------------------------------------
// Resolution and field of view
// ---------------------------------------------------------------------------

FIntPoint UMjCamera::CaptureResolution() const
{
	const TArray<int32> Authored = HasResolution() ? GetResolution() : TArray<int32>();
	const int32 W = (Authored.Num() > 0 && Authored[0] > 0) ? Authored[0] : 640;
	const int32 H = (Authored.Num() > 1 && Authored[1] > 0) ? Authored[1] : 480;
	return FIntPoint(W, H);
}

void UMjCamera::RefreshCaptureFov()
{
	DerivedFovy = DeriveFovyDegrees(*this);
	if (CaptureComponent)
	{
		CaptureComponent->FOVAngle = HorizontalFOVFromFovy(DerivedFovy, CaptureResolution());
	}
}

// ---------------------------------------------------------------------------
// Capture pipeline
// ---------------------------------------------------------------------------

void UMjCamera::UpdateCapturePipeline()
{
	const bool bActive = IsCaptureActive();

	// Per-camera gating: start capturing lazily when a camera becomes active, e.g.
	// on the first include_cameras request against a non-broadcast camera, and
	// pause the GPU scene capture when it goes dormant, so a scene with many
	// cameras only pays for the ones actually consumed.
	if (bActive && !bStreamingEnabled)
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
		const bool bWantEveryFrame = bActive && bRenderInMainRenderer;
		if (CaptureComponent->bCaptureEveryFrame != bWantEveryFrame)
		{
			CaptureComponent->bCaptureEveryFrame = bWantEveryFrame;
		}
		if (bActive)
		{
			// Every tick, because the streaming manager decays viewpoints on a timeout.
			RegisterWithStreamingManager();

			// Non-seg cameras re-sync HiddenComponents each tick, so siblings spawned
			// by a late-starting seg camera cannot leak into this capture.
			if (!IsSegMode(CaptureMode))
			{
				RefreshHiddenComponentsFromSegPools();
			}
		}
	}

	HarvestCompletedReadbacks();

	// Manual-capture-only cameras (render-server forced-render mode) still harvest
	// above, but never auto-capture -- only the on-demand IssueSyncCapture drives
	// them, so the async stream cannot contend with a forced request.
	if (bStreamingEnabled && bActive && !bManualCaptureOnly)
	{
		MaybeCapture();
		PublishDueDelayedFrames();
	}
}

void UMjCamera::HarvestCompletedReadbacks()
{
	// Two async stages, both driven from the game thread with no render-thread
	// flush: promote GPU-ready readbacks to a render-thread map and copy, then
	// drain whatever the render thread finished into the history ring.
	DispatchReadyReadbacks();
	DrainCompletedFrames();
}

void UMjCamera::DispatchReadyReadbacks()
{
	// FIFO: readbacks complete in submission order, so stop at the first entry
	// whose GPU copy is not ready rather than reordering later frames ahead of it.
	while (InFlightReadbacks.Num() > 0)
	{
		FInFlightReadback& Head = InFlightReadbacks[0];
		if (!Head.Gpu.IsValid())
		{
			InFlightReadbacks.RemoveAt(0);
			continue;
		}
		if (!Head.Gpu->IsReady())
		{
			break;
		}

		FInFlightReadback Front = MoveTemp(InFlightReadbacks[0]);
		InFlightReadbacks.RemoveAt(0);
		// First game tick that observed the GPU fence signalled: the boundary
		// between "GPU copy done" and "we noticed", including tick quantization.
		Front.TFenceReadySeconds = FPlatformTime::Seconds();

		TSharedPtr<FMjCameraFrame> Frame = MakeShared<FMjCameraFrame>();
		Frame->FrameId = Front.FrameId;
		Frame->SimTime = Front.SimTime;
		Frame->Width = Front.Width;
		Frame->Height = Front.Height;
		Frame->CaptureUnixTime = Front.CaptureUnixSeconds;
		Frame->TIssuedSeconds = Front.TIssuedSeconds;
		Frame->TFenceReadySeconds = Front.TFenceReadySeconds;

		const int32 W = Front.Width;
		const int32 H = Front.Height;
		const EMjCameraMode CapturedMode = CaptureMode;
		TSharedPtr<FRHIGPUTextureReadback> Gpu = Front.Gpu;

		++PendingMapCommands;

		// Map and copy the staging buffer on the render thread.
		// FRHIGPUTextureReadback::Lock asserts IsInRenderingThread, and the readback
		// is already ready, so this is a CPU memcpy out of mapped staging memory
		// with no GPU wait. The command captures the shared results queue rather
		// than `this`, so a teardown race that destroys the component before the
		// command runs pushes into a still-live queue instead of a freed object.
		// RowPitch is in PIXELS and at least the width (rows are padded), so the
		// copy is row-by-row into a tightly packed array.
		const bool bSplitDiag = CVarCamDiag.GetValueOnGameThread() != 0;
		ENQUEUE_RENDER_COMMAND(MjCameraMapReadback)
		([Frame, Gpu, W, H, CapturedMode, Results = ResultsQueue, bSplitDiag](FRHICommandListImmediate&) {
			const double RTStart = FPlatformTime::Seconds();
			if (bSplitDiag)
			{
				// Split fence->map into RT scheduling latency (enqueue -> this lambda
				// actually running) vs the map memcpy itself, to prove where the ~26ms
				// lives: pipeline/RT-cadence starvation vs real map cost.
				UE_LOG(LogURLabNet, Log, TEXT("[mapsplit] enqueue->RTstart=%.2f ms"),
					(RTStart - Frame->TFenceReadySeconds) * 1000.0);
			}
			bool bCopied = false;
			int32 RowPitchPixels = 0;
			void* Data = Gpu->Lock(RowPitchPixels);
			if (Data && W > 0 && H > 0)
			{
				if (CapturedMode == EMjCameraMode::Depth)
				{
					Frame->Depth.SetNumUninitialized(W * H);
					const float* Src = static_cast<const float*>(Data);
					for (int32 y = 0; y < H; ++y)
					{
						FMemory::Memcpy(&Frame->Depth[y * W], &Src[y * RowPitchPixels], W * sizeof(float));
					}
				}
				else
				{
					Frame->Color.SetNumUninitialized(W * H);
					const FColor* Src = static_cast<const FColor*>(Data);
					for (int32 y = 0; y < H; ++y)
					{
						FMemory::Memcpy(&Frame->Color[y * W], &Src[y * RowPitchPixels], W * sizeof(FColor));
					}
				}
				bCopied = true;
			}
			Gpu->Unlock();
			// Render thread, memcpy out of mapped staging complete. Same clock as
			// the game-thread publish stamp, so map->publish is the game-thread
			// bounce that a render-thread publish (SPEAR) would remove.
			Frame->TMappedSeconds = FPlatformTime::Seconds();
			if (bSplitDiag)
			{
				UE_LOG(LogURLabNet, Log, TEXT("[mapsplit] map-work=%.2f ms"),
					(Frame->TMappedSeconds - RTStart) * 1000.0);
			}
			Results->Enqueue(FCompletedReadback{Frame, Gpu, bCopied});
		});
	}
}

void UMjCamera::DrainCompletedFrames()
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

		if (Completed.bCopied && Completed.Frame.IsValid())
		{
			FMjCameraFrame& Frame = *Completed.Frame;
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
			if (!IsDelayActive())
			{
				PublishFrameToWorkers(Frame);
			}
			PushFrameToHistoryShared(Completed.Frame);
		}
		else
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
}

void UMjCamera::AccumulateStageTiming(const FMjCameraFrame& Frame)
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

void UMjCamera::EnsureRenderThreadHarvestRegistered()
{
	if (bRTHarvestRegistered)
	{
		return;
	}
	RTHarvestHandle = FCoreDelegates::OnEndFrameRT.AddUObject(this, &UMjCamera::OnEndFrameRT_Harvest);
	bRTHarvestRegistered = true;
}

void UMjCamera::UnregisterRenderThreadHarvest()
{
	if (!bRTHarvestRegistered)
	{
		return;
	}
	FCoreDelegates::OnEndFrameRT.Remove(RTHarvestHandle);
	RTHarvestHandle.Reset();
	bRTHarvestRegistered = false;
	// Drain any callback already dispatched so nothing touches the transports we
	// are about to free, then discard the unharvested readbacks.
	FlushRenderingCommands();
	FScopeLock Lock(&RTInFlightLock);
	RTInFlight.Reset();
}

void UMjCamera::OnEndFrameRT_Harvest()
{
	// Render thread, end of frame. Move the ready readbacks out under the lock,
	// then map + publish them here so the copy-out never crosses a game tick.
	// Only the render thread produces to the transports in this mode, so the SPSC
	// ZMQ queue keeps its single-producer contract; the delay path (a second,
	// game-thread producer) is not supported here -- the A/B runs at delay=0.
	TArray<FInFlightReadback> Ready;
	{
		FScopeLock Lock(&RTInFlightLock);
		while (RTInFlight.Num() > 0)
		{
			FInFlightReadback& Head = RTInFlight[0];
			if (!Head.Gpu.IsValid())
			{
				RTInFlight.RemoveAt(0);
				continue;
			}
			if (!Head.Gpu->IsReady())
			{
				break; // FIFO: submission order is completion order.
			}
			Head.TFenceReadySeconds = FPlatformTime::Seconds();
			Ready.Add(MoveTemp(RTInFlight[0]));
			RTInFlight.RemoveAt(0);
		}
	}

	const EMjCameraMode CapturedMode = CaptureMode;
	for (FInFlightReadback& R : Ready)
	{
		TSharedPtr<FMjCameraFrame> Frame = MakeShared<FMjCameraFrame>();
		Frame->FrameId = R.FrameId;
		Frame->SimTime = R.SimTime;
		Frame->Width = R.Width;
		Frame->Height = R.Height;
		Frame->CaptureUnixTime = R.CaptureUnixSeconds;
		Frame->TIssuedSeconds = R.TIssuedSeconds;
		Frame->TFenceReadySeconds = R.TFenceReadySeconds;

		const int32 W = R.Width;
		const int32 H = R.Height;
		int32 RowPitchPixels = 0;
		void* Data = R.Gpu->Lock(RowPitchPixels);
		bool bCopied = false;
		if (Data && W > 0 && H > 0)
		{
			if (CapturedMode == EMjCameraMode::Depth)
			{
				Frame->Depth.SetNumUninitialized(W * H);
				const float* Src = static_cast<const float*>(Data);
				for (int32 y = 0; y < H; ++y)
				{
					FMemory::Memcpy(&Frame->Depth[y * W], &Src[y * RowPitchPixels], W * sizeof(float));
				}
			}
			else
			{
				Frame->Color.SetNumUninitialized(W * H);
				const FColor* Src = static_cast<const FColor*>(Data);
				for (int32 y = 0; y < H; ++y)
				{
					FMemory::Memcpy(&Frame->Color[y * W], &Src[y * RowPitchPixels], W * sizeof(FColor));
				}
			}
			bCopied = true;
		}
		R.Gpu->Unlock();
		Frame->TMappedSeconds = FPlatformTime::Seconds();

		if (bCopied)
		{
			Frame->Seq = ++HarvestSeq; // sole writer in RT mode (game drain finds nothing)
			PublishFrameRenderThread(*Frame);
			Frame->TPublishedSeconds = FPlatformTime::Seconds();
			AccumulateStageTimingRT(*Frame);
			PushFrameToHistoryShared(Frame); // lock-guarded ring, safe from any thread
		}
		// R.Gpu drops here, freeing its staging texture. No cross-thread recycle
		// pool in this prototype: allocation churn is a separate axis, not the
		// copy-out latency under test.
	}
}

void UMjCamera::PublishFrameRenderThread(const FMjCameraFrame& Frame)
{
	FMjCameraWireFrame Wire;
	if (!BuildCameraWireFrame(Frame, Wire))
	{
		return;
	}
	for (UURLabCameraPublishTransport* Transport : CameraPublishers)
	{
		if (Transport)
		{
			Transport->PublishCameraFrame(Wire);
		}
	}
	// The in-proc FMjCameraFrameBus broadcast is game-thread-only; the render-thread
	// path serves the streaming transports the A/B measures and skips the bus.
}

void UMjCamera::AccumulateStageTimingRT(const FMjCameraFrame& Frame)
{
	if (CVarCamDiag.GetValueOnRenderThread() == 0)
	{
		return;
	}
	if (Frame.TIssuedSeconds <= 0.0 || Frame.TFenceReadySeconds <= 0.0
		|| Frame.TMappedSeconds <= 0.0 || Frame.TPublishedSeconds <= 0.0)
	{
		return;
	}
	const double IssueToFence = (Frame.TFenceReadySeconds - Frame.TIssuedSeconds) * 1000.0;
	const double FenceToMap = (Frame.TMappedSeconds - Frame.TFenceReadySeconds) * 1000.0;
	const double MapToPublish = (Frame.TPublishedSeconds - Frame.TMappedSeconds) * 1000.0;
	const double Total = (Frame.TPublishedSeconds - Frame.TIssuedSeconds) * 1000.0;

	RTDiagSumIssueToFence += IssueToFence;
	RTDiagSumFenceToMap += FenceToMap;
	RTDiagSumMapToPublish += MapToPublish;
	RTDiagSumTotal += Total;
	RTDiagMaxTotal = FMath::Max(RTDiagMaxTotal, Total);
	++RTDiagFrames;

	if (RTDiagFrames >= DiagWindowFrames)
	{
		const double Inv = 1.0 / static_cast<double>(RTDiagFrames);
		UE_LOG(LogURLabNet, Log,
			TEXT("[camstage-rt] '%s' N=%d mean(ms): issue->fence=%.2f fence->map=%.2f "
				 "map->publish=%.2f total=%.2f | max total=%.2f"),
			*CameraLogName(*this), RTDiagFrames,
			RTDiagSumIssueToFence * Inv, RTDiagSumFenceToMap * Inv,
			RTDiagSumMapToPublish * Inv, RTDiagSumTotal * Inv, RTDiagMaxTotal);
		RTDiagFrames = 0;
		RTDiagSumIssueToFence = RTDiagSumFenceToMap = RTDiagSumMapToPublish = RTDiagSumTotal = 0.0;
		RTDiagMaxTotal = 0.0;
	}
}

void UMjCamera::WaitAndHarvestReadbacks(double TimeoutSeconds)
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

void UMjCamera::IssueSyncCapture()
{
	// Force a render and readback for the current applied state. TouchRequested
	// keeps per-camera gating live so the per-frame pass keeps producing
	// afterwards; EnqueueReadback no-ops while the render target is still cold,
	// and MaybeCapture retries until a real frame lands. The RHI submit is forced
	// so this on-demand copy's fence signals promptly.
	TouchRequested();
	if (CaptureComponent && CaptureComponent->TextureTarget)
	{
		CaptureComponent->CaptureScene();
	}
	uint64 ShowId = 0;
	double ShowTime = 0.0;
	if (IMjSimClock* Clock = ResolveSimClock())
	{
		ShowId = Clock->GetAppliedFrameId();
		ShowTime = Clock->GetAppliedSimTime();
	}
	EnqueueReadback(ShowId, ShowTime, /*bForceSubmit=*/true);
}

void MjPumpForcedCapture(const TArray<UMjCamera*>& Cams, uint64 TargetFrameId, int32 TimeoutMs)
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
			UMjCamera* Cam = Cams[i];
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

void UMjCamera::SetSimClock(UObject* ClockObject)
{
	// Store the UObject weakly; ResolveSimClock casts it to the interface on read.
	InjectedSimClock = ClockObject;
}

IMjSimClock* UMjCamera::ResolveSimClock() const
{
	if (UObject* Obj = InjectedSimClock.Get())
	{
		if (IMjSimClock* Clock = Cast<IMjSimClock>(Obj))
		{
			return Clock;
		}
	}
	// Fallback: the manager singleton (manager paths, or before injection). Keeps
	// manager-path behavior identical while the injection seam exists for Mirror.
	return Cast<IMjSimClock>(AAMjManager::GetManager());
}

void UMjCamera::MaybeCapture()
{
	IMjSimClock* Clock = ResolveSimClock();
	const uint64 AppliedId = Clock ? Clock->GetAppliedFrameId() : 0;
	const double AppliedTime = Clock ? Clock->GetAppliedSimTime() : 0.0;
	const double NowWall = FPlatformTime::Seconds();
	const bool bFpsOk = (CaptureMaxFps <= 0.0f)
					 || (NowWall - LastCaptureWallSeconds) >= (1.0 / static_cast<double>(CaptureMaxFps));
	// The state-change gate is consumed only once a readback actually goes out, so
	// a freshly enabled camera whose render target has no RHI texture yet keeps
	// this true and retries across ticks until a real frame can be captured,
	// instead of latching a single missed capture.
	const bool bStateAdvanced = !bCaptureOnStateChange || (AppliedId != LastCapturedFrameId);

	if (bFpsOk && bStateAdvanced)
	{
		bool bIssued = false;
		if (bRenderInMainRenderer)
		{
			// This camera renders as a nested pass of the main renderer on the main
			// cadence, so the target is updated during the main render AFTER this
			// readback copy is submitted. Stamp the id whose render the target
			// currently holds, so the pixels and the frame id agree.
			bIssued = EnqueueReadback(LastRenderedAppliedId, LastRenderedAppliedTime);
		}
		else
		{
			// Render into the target immediately before enqueuing the copy, so the
			// copy captures a freshly rendered frame for the current applied state,
			// identical to the synchronous path. Relying on the component's automatic
			// every-frame capture would enqueue the copy before that frame's capture
			// rendered, producing a stale or black frame.
			if (CaptureComponent)
			{
				CaptureComponent->CaptureScene();
			}
			bIssued = EnqueueReadback(AppliedId, AppliedTime,
				/*bForceSubmit=*/CVarCamForceSubmit.GetValueOnGameThread() != 0);
		}

		if (bIssued)
		{
			LastCapturedFrameId = AppliedId;
			LastCaptureWallSeconds = NowWall;
		}
	}

	// Track the applied state whose render a main-renderer target will show next tick.
	LastRenderedAppliedId = AppliedId;
	LastRenderedAppliedTime = AppliedTime;
}

void UMjCamera::PublishDueDelayedFrames()
{
	// Publish the delayed selection, the newest frame whose reveal time has
	// passed, each Seq exactly once, so the stream lags by the configured delay.
	// The no-delay path already published inline at harvest.
	if (!IsDelayActive())
	{
		return;
	}

	// On the sim clock this is deterministic by design: while the sim is paused
	// the applied sim time is frozen, so a frame captured during the pause never
	// reaches its reveal time and delayed publishing stalls until stepping
	// resumes. That is what a sim-time delay means -- frozen time, frozen latency
	// -- and a caller wanting reveals to advance in real time while paused sets
	// bDelayUseWallClock. Any pile-up of unrevealed frames is bounded by the
	// history ring's ceiling.
	IMjSimClock* Clock = ResolveSimClock();
	const double AppliedSimTime = Clock ? Clock->GetAppliedSimTime() : 0.0;
	const double NowVal = bDelayUseWallClock
							? (FDateTime::UtcNow() - FDateTime(1970, 1, 1)).GetTotalSeconds()
							: AppliedSimTime;
	TSharedPtr<const FMjCameraFrame> Selected = SelectDelayedFrameShared(NowVal, Delay.LastPublishedSeq);
	if (Selected.IsValid())
	{
		PublishFrameToWorkers(*Selected);
		Delay.LastPublishedSeq = Selected->Seq;
	}
}

// ---------------------------------------------------------------------------
// Streaming setup
// ---------------------------------------------------------------------------

void UMjCamera::SetupRenderTarget()
{
	const FIntPoint Res = CaptureResolution();
	UTextureRenderTarget2D* RT = NewObject<UTextureRenderTarget2D>(this);

	const bool bDepthMode = (CaptureMode == EMjCameraMode::Depth);
	if (bDepthMode)
	{
		RT->RenderTargetFormat = ETextureRenderTargetFormat::RTF_R32f;
		RT->InitCustomFormat(Res.X, Res.Y, PF_R32_FLOAT, /*bForceLinearGamma=*/true);
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
	// A 1mm near clip on every capture mode: without it, robot-internal geometry
	// intrudes on the frustum and produces black-on-black frames when the camera
	// is mounted inside a body shell.
	CaptureComponent->bOverride_CustomNearClippingPlane = true;
	CaptureComponent->CustomNearClippingPlane = 0.1f;

	switch (CaptureMode)
	{
		case EMjCameraMode::Depth:
			CaptureComponent->CaptureSource = ESceneCaptureSource::SCS_SceneDepth;
			break;

		case EMjCameraMode::SemanticSegmentation:
		case EMjCameraMode::InstanceSegmentation:
			// FinalToneCurveHDR rather than SCS_BaseColor: the basic shape material's
			// Color parameter is not wired to the BaseColor G-buffer, so SCS_BaseColor
			// renders empty.
			CaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalToneCurveHDR;
			CaptureComponent->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;
			break;

		case EMjCameraMode::Real:
		default:
			// LDR final colour: already tone-mapped and gamma-encoded to the same sRGB
			// the viewport shows, so the BGRA8 readback matches the editor image. The
			// readback skips TargetGamma, which is why an HDR-then-encode source
			// delivered a darker frame. It also saves a conversion and a copy.
			CaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
			CaptureComponent->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_LegacySceneCapture;
			break;
	}

	RenderTarget = RT;
	UE_LOG(LogURLabImport, Log, TEXT("[MjCamera] '%s' RT created mode=%s (%dx%d)"),
		*CameraLogName(*this), *UEnum::GetValueAsString(CaptureMode), Res.X, Res.Y);
}

void UMjCamera::RefreshHiddenComponentsFromSegPools()
{
	if (!CaptureComponent)
	{
		return;
	}

	AMjRenderer* Renderer = FindRenderer(GetWorld());
	if (!Renderer)
	{
		return;
	}

	CaptureComponent->HiddenComponents.Reset();

	TArray<UPrimitiveComponent*> Pool;
	Renderer->GetSegPoolSiblings(EMjCameraMode::InstanceSegmentation, Pool);
	for (UPrimitiveComponent* Sibling : Pool)
	{
		CaptureComponent->HiddenComponents.Add(Sibling);
	}

	Pool.Reset();
	Renderer->GetSegPoolSiblings(EMjCameraMode::SemanticSegmentation, Pool);
	for (UPrimitiveComponent* Sibling : Pool)
	{
		CaptureComponent->HiddenComponents.Add(Sibling);
	}
}

void UMjCamera::RegisterWithStreamingManager()
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

void UMjCamera::SetStreamingEnabled(bool bEnable)
{
	if (bEnable)
	{
		if (!RenderTarget)
		{
			SetupRenderTarget();
		}

		// Seg modes subscribe to the shared sibling-mesh pool and point
		// ShowOnlyComponents at it. The pool is built lazily on the first subscriber.
		if (IsSegMode(CaptureMode))
		{
			if (AMjRenderer* Renderer = FindRenderer(GetWorld()))
			{
				TArray<UPrimitiveComponent*> Siblings;
				Renderer->AcquireSegPool(CaptureMode, this, Siblings);

				CaptureComponent->ShowOnlyComponents.Reset();
				CaptureComponent->ShowOnlyComponents.Reserve(Siblings.Num());
				for (UPrimitiveComponent* Sibling : Siblings)
				{
					CaptureComponent->ShowOnlyComponents.Add(Sibling);
				}
				UE_LOG(LogURLabImport, Log,
					TEXT("[MjCamera] '%s' seg mode: acquired %d sibling(s) into ShowOnlyComponents."),
					*CameraLogName(*this), Siblings.Num());
			}
			else
			{
				UE_LOG(LogURLabImport, Warning,
					TEXT("[MjCamera] '%s' seg mode requested but no renderer found; seg cam will show nothing."),
					*CameraLogName(*this));
			}
		}
		else
		{
			// Siblings belonging to other seg cameras are visible in scene captures
			// only, so they would otherwise appear in an RGB or depth capture. Seed
			// HiddenComponents now; the tick-time refresh keeps it in step with
			// late-starting seg cameras.
			RefreshHiddenComponentsFromSegPools();
		}

		// Build the per-camera image-egress transports from the authored broadcast
		// flags: the ZMQ backend, the SHM backend, and any external backend an
		// optional transport module installs. Each owns this camera's channel and a
		// published frame fans out to every one. Guarded so the double enable at
		// startup (RegisterCamera then BeginPlay) does not stand up a second set.
		if (CameraPublishers.Num() == 0)
		{
			if (bEnableZmqBroadcast)
			{
				// Bind inside this instance's camera port block, so several editors
				// acting as render servers never fight over one port.
				ZmqEndpoint = ResolveStreamEndpoint();
				UURLabZmqCameraPublishTransport* Zmq = NewObject<UURLabZmqCameraPublishTransport>(this);
				Zmq->Configure(ZmqEndpoint, CaptureResolution());
				CameraPublishers.Add(Zmq);
			}
			if (bEnableShmBroadcast)
			{
				UURLabShmCameraPublishTransport* Shm = NewObject<UURLabShmCameraPublishTransport>(this);
				Shm->Configure(CaptureResolution());
				CameraPublishers.Add(Shm);
			}
			// An optional transport module (gRPC / dm_env) may install a camera
			// egress backend; add it alongside the built-ins when present.
			if (FMjExternalTransportProvider::MakeCameraPublishTransport.IsBound())
			{
				if (UURLabCameraPublishTransport* Ext =
						FMjExternalTransportProvider::MakeCameraPublishTransport.Execute(this))
				{
					CameraPublishers.Add(Ext);
				}
			}

			const FString Canonical = GetCanonicalName();
			for (UURLabCameraPublishTransport* Transport : CameraPublishers)
			{
				if (Transport)
				{
					Transport->TransportInit();
					Transport->OpenCameraChannel(StreamPortIndex, Canonical, StreamPortIndex);
				}
			}
		}

		if (CaptureComponent)
		{
			RefreshCaptureFov();

			// SetVisibility(true) is what allows the component to dispatch scene
			// capture updates: bHiddenInGame alone is not enough, because the capture
			// system checks IsVisible() each frame.
			CaptureComponent->SetVisibility(true);
			CaptureComponent->SetActive(true);
			CaptureComponent->bHiddenInGame = false;
			// Normal cameras are driven manually in MaybeCapture right before each
			// readback; only main-renderer cameras render on the automatic cadence.
			CaptureComponent->bCaptureEveryFrame = bRenderInMainRenderer;
			CaptureComponent->bCaptureOnMovement = false;
		}
		bStreamingEnabled = true;
		RegisterWithStreamingManager();

		// One immediate capture, so a UI does not wait a full tick. CaptureScene
		// bypasses the visibility check and always fires.
		if (CaptureComponent)
		{
			CaptureComponent->CaptureScene();
		}

		UE_LOG(LogURLabImport, Log, TEXT("[MjCamera] '%s' streaming ENABLED."), *CameraLogName(*this));
	}
	else
	{
		bStreamingEnabled = false;

		// Stop + drain the render-thread harvest before the transports it publishes
		// through are torn down below.
		UnregisterRenderThreadHarvest();

		// Release the seg pool first, while CaptureMode still says what we subscribed as.
		if (IsSegMode(CaptureMode))
		{
			if (AMjRenderer* Renderer = FindRenderer(GetWorld()))
			{
				Renderer->ReleaseSegPool(CaptureMode, this);
			}
			if (CaptureComponent)
			{
				CaptureComponent->ShowOnlyComponents.Reset();
				CaptureComponent->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_LegacySceneCapture;
			}
		}

		if (CaptureComponent)
		{
			CaptureComponent->bCaptureEveryFrame = false;
			CaptureComponent->SetVisibility(false); // stops the capture dispatch loop
			CaptureComponent->TextureTarget = nullptr;
		}

		// Teardown is the one place a render-thread flush is correct: the in-flight
		// copy commands and any already-dispatched map commands hold shared readback
		// objects, so drain the render thread once, then release, and nothing runs
		// against freed memory. No new map commands are dispatched here; whatever
		// finished lands in the results queue and is discarded with the pipeline.
		if (InFlightReadbacks.Num() > 0 || PendingMapCommands > 0)
		{
			FlushRenderingCommands();
		}
		InFlightReadbacks.Empty();
		FCompletedReadback Discard;
		while (ResultsQueue->Dequeue(Discard))
		{
		}
		PendingMapCommands = 0;
		FreeReadbacks.Empty();

		// Drop the target, so the next enable rebuilds it in the current mode.
		RenderTarget = nullptr;

		// Close every channel and release each transport's backend handles (ZMQ
		// send threads, SHM regions, external streams).
		for (UURLabCameraPublishTransport* Transport : CameraPublishers)
		{
			if (Transport)
			{
				Transport->CloseCameraChannel(StreamPortIndex);
				Transport->TransportShutdown();
			}
		}
		CameraPublishers.Reset();

		// Tell any out-of-core image sink to release its per-camera resources.
		FMjCameraFrameBus::Get().OnStreamStopped.Broadcast(GetCanonicalName());

		UE_LOG(LogURLabImport, Log, TEXT("[MjCamera] '%s' streaming DISABLED."), *CameraLogName(*this));
	}
}

// ---------------------------------------------------------------------------
// Async readback
// ---------------------------------------------------------------------------

void UMjCamera::RequestReadback()
{
	if (IMjSimClock* Clock = ResolveSimClock())
	{
		EnqueueReadback(Clock->GetAppliedFrameId(), Clock->GetAppliedSimTime());
	}
	else
	{
		EnqueueReadback(0, 0.0);
	}
}

bool UMjCamera::EnqueueReadback(uint64 ShowFrameId, double ShowSimTime, bool bForceSubmit)
{
	if (!RenderTarget)
	{
		return false;
	}
	// Pipeline cap: keep at most MaxInFlightReadbacks async copies outstanding.
	// The active list differs by harvest mode, so the cap must count the right one
	// -- counting the empty game-thread list in RT mode removed all backpressure
	// and let readbacks pile up, inflating issue->fence.
	const bool bRTHarvest = CVarCamRTHarvest.GetValueOnGameThread() != 0;
	int32 InFlightCount;
	if (bRTHarvest)
	{
		FScopeLock Lock(&RTInFlightLock);
		InFlightCount = RTInFlight.Num();
	}
	else
	{
		InFlightCount = InFlightReadbacks.Num();
	}
	if (InFlightCount >= MaxInFlightReadbacks)
	{
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

	// A freshly enabled render target reports its size on the game thread before
	// its RHI texture is created on the render thread. Copying from a null texture
	// would stage nothing and yield a black frame that then latches in history, so
	// wait until the texture exists and let the caller retry next tick.
	FTextureRHIRef SourceTexture = Resource->GetRenderTargetTexture();
	if (!SourceTexture.IsValid())
	{
		return false;
	}

	FInFlightReadback Entry;
	if (FreeReadbacks.Num() > 0)
	{
		Entry.Gpu = FreeReadbacks.Pop(EAllowShrinking::No);
	}
	else
	{
		Entry.Gpu = MakeShared<FRHIGPUTextureReadback>(TEXT("MjCameraReadback"));
	}
	Entry.Width = Size.X;
	Entry.Height = Size.Y;
	// Unix-epoch capture time for the wire meta, the same clock as the state
	// stream's wall_time and Python's time.time().
	Entry.CaptureUnixSeconds = (FDateTime::UtcNow() - FDateTime(1970, 1, 1)).GetTotalSeconds();
	// Monotonic stamp on the same clock as the later stage stamps, for copy-out
	// timing. Unix time above is for the wire; this is for stage deltas.
	Entry.TIssuedSeconds = FPlatformTime::Seconds();
	// The post-step state the harvested pixels show: for a manual capture that is
	// the current applied id; in every-frame mode the caller passes the id whose
	// render the target currently holds.
	Entry.FrameId = ShowFrameId;
	Entry.SimTime = ShowSimTime;

	// Schedule the GPU-to-staging copy without stalling the render thread.
	// SourceTexture is captured as a ref-counted handle, so a render target
	// reallocation between enqueue and execution cannot copy against a released
	// texture: the reference keeps the RHI texture alive until the command runs.
	FRHIGPUTextureReadback* Readback = Entry.Gpu.Get();
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

	if (CVarCamRTHarvest.GetValueOnGameThread() != 0)
	{
		// Render-thread harvest: hand the in-flight readback to the RT list and
		// ensure the end-of-render-frame poller is registered. The game-thread
		// harvest (DispatchReadyReadbacks/DrainCompletedFrames) then finds nothing.
		EnsureRenderThreadHarvestRegistered();
		FScopeLock Lock(&RTInFlightLock);
		RTInFlight.Add(MoveTemp(Entry));
	}
	else
	{
		InFlightReadbacks.Add(MoveTemp(Entry));
	}
	return true;
}

// ---------------------------------------------------------------------------
// Frame history
// ---------------------------------------------------------------------------

void FMjCameraHistory::PushShared(const TSharedPtr<const FMjCameraFrame>& Frame,
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
	const double NewestClock = FMjCameraDelayModel::FrameClock(*Frames.Last(), bUseWallClock);
	while (Frames.Num() > 1)
	{
		const bool bExpired = (NewestClock - FMjCameraDelayModel::FrameClock(*Frames[0], bUseWallClock)) > RetainWindow;
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

TSharedPtr<const FMjCameraFrame> FMjCameraHistory::GetShared(uint64 MinFrameId) const
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
	for (const TSharedPtr<const FMjCameraFrame>& Frame : Frames)
	{
		if (Frame->FrameId >= MinFrameId)
		{
			return Frame;
		}
	}
	return nullptr;
}

uint64 FMjCameraHistory::GetLatestFrameId() const
{
	FScopeLock ScopeLock(&Lock);
	return Frames.Num() > 0 ? Frames.Last()->FrameId : 0;
}

TSharedPtr<const FMjCameraFrame> FMjCameraHistory::SelectDelayedShared(double NowValue, uint64 AfterSeq) const
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

void UMjCamera::PushFrameToHistory(FMjCameraFrame&& Frame)
{
	TSharedPtr<FMjCameraFrame> Shared = MakeShared<FMjCameraFrame>(MoveTemp(Frame));
	PushFrameToHistoryShared(Shared);
}

void UMjCamera::PushFrameToHistoryShared(const TSharedPtr<const FMjCameraFrame>& Frame)
{
	const double RetainWindow = static_cast<double>(DelaySeconds + DelayJitterSeconds) + 0.10;
	History.PushShared(Frame, HistoryCapacity, IsDelayActive(), RetainWindow, bDelayUseWallClock);
}

TSharedPtr<const FMjCameraFrame> UMjCamera::GetFrameShared(uint64 MinFrameId) const
{
	return History.GetShared(MinFrameId);
}

bool UMjCamera::GetFrame(uint64 MinFrameId, FMjCameraFrame& Out) const
{
	TSharedPtr<const FMjCameraFrame> Frame = GetFrameShared(MinFrameId);
	if (!Frame.IsValid())
	{
		return false;
	}
	Out = *Frame;
	return true;
}

TSharedPtr<const FMjCameraFrame> UMjCamera::GetFrameForRequest(uint64 MinFrameId, bool bIgnoreDelay) const
{
	// With latency emulation active, an RPC read must see what the stream is
	// currently publishing, the delayed past, rather than the undelayed newest
	// frame, so the two agree. bIgnoreDelay opts out for a caller that explicitly
	// wants the freshest rendered ground truth.
	if (!bIgnoreDelay && IsDelayActive())
	{
		return SelectDelayedFrameShared(NowClockValue(), 0);
	}
	return GetFrameShared(MinFrameId);
}

uint64 UMjCamera::GetLatestFrameId() const
{
	return History.GetLatestFrameId();
}

TSharedPtr<const FMjCameraFrame> UMjCamera::SelectDelayedFrameShared(double NowValue, uint64 AfterSeq) const
{
	return History.SelectDelayedShared(NowValue, AfterSeq);
}

bool UMjCamera::SelectDelayedFrame(double NowValue, uint64 AfterSeq, FMjCameraFrame& Out) const
{
	TSharedPtr<const FMjCameraFrame> Frame = SelectDelayedFrameShared(NowValue, AfterSeq);
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

double FMjCameraDelayModel::FrameClock(const FMjCameraFrame& Frame, bool bUseWallClock)
{
	return bUseWallClock ? Frame.CaptureUnixTime : Frame.SimTime;
}

double FMjCameraDelayModel::NowClockValue(bool bUseWallClock, double AppliedSimTime)
{
	if (bUseWallClock)
	{
		return (FDateTime::UtcNow() - FDateTime(1970, 1, 1)).GetTotalSeconds();
	}
	// The sim-clock "now" is the applied render-state time, supplied by the caller
	// from IMjSimClock (manager or Mirror), not read from the manager singleton.
	return AppliedSimTime;
}

bool FMjCameraDelayModel::IsDelayActive(float DelaySeconds, float JitterSeconds)
{
	return DelaySeconds > 0.0f || JitterSeconds > 0.0f;
}

double FMjCameraDelayModel::SampleDelaySeconds(float DelaySeconds, float JitterSeconds)
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

void FMjCameraDelayModel::Configure(int32 Seed)
{
	Rng.Initialize(Seed);
	// Re-arm the publish dedup, so the new policy re-selects cleanly.
	LastPublishedSeq = 0;
}

double UMjCamera::FrameClock(const FMjCameraFrame& Frame) const
{
	return FMjCameraDelayModel::FrameClock(Frame, bDelayUseWallClock);
}

double UMjCamera::NowClockValue() const
{
	IMjSimClock* Clock = ResolveSimClock();
	const double AppliedSimTime = Clock ? Clock->GetAppliedSimTime() : 0.0;
	return FMjCameraDelayModel::NowClockValue(bDelayUseWallClock, AppliedSimTime);
}

bool UMjCamera::IsDelayActive() const
{
	return FMjCameraDelayModel::IsDelayActive(DelaySeconds, DelayJitterSeconds);
}

double UMjCamera::SampleDelaySeconds()
{
	return Delay.SampleDelaySeconds(DelaySeconds, DelayJitterSeconds);
}

void UMjCamera::SetCameraDelay(float InDelaySeconds, float InJitterSeconds, bool bInUseWallClock, int32 InSeed)
{
	DelaySeconds = FMath::Max(0.0f, InDelaySeconds);
	DelayJitterSeconds = FMath::Max(0.0f, InJitterSeconds);
	bDelayUseWallClock = bInUseWallClock;
	const int32 Seed = (InSeed != 0) ? InSeed : static_cast<int32>(GetTypeHash(GetCanonicalName()));
	Delay.Configure(Seed);
}

void UMjCamera::SetCaptureRate(bool bInOnStateChange, float InMaxFps)
{
	bCaptureOnStateChange = bInOnStateChange;
	CaptureMaxFps = FMath::Max(0.0f, InMaxFps);
}

// ---------------------------------------------------------------------------
// Publishing and identity
// ---------------------------------------------------------------------------

bool UMjCamera::BuildCameraWireFrame(const FMjCameraFrame& Frame, FMjCameraWireFrame& Out) const
{
	Out.CameraIndex = StreamPortIndex;
	Out.Width = Frame.Width;
	Out.Height = Frame.Height;
	Out.FrameId = Frame.FrameId;
	Out.SimTime = Frame.SimTime;
	// The original capture time, so a delayed frame reports the moment it was
	// taken and the client's content age reflects the injected latency.
	Out.CaptureUnixSeconds = Frame.CaptureUnixTime;

	if (CaptureMode == EMjCameraMode::Depth)
	{
		if (Frame.Depth.Num() == 0)
		{
			return false;
		}
		Out.Dtype = FName(TEXT("float32"));
		Out.Bytes = reinterpret_cast<const uint8*>(Frame.Depth.GetData());
		Out.NumBytes = Frame.Depth.Num() * static_cast<int32>(sizeof(float));
	}
	else
	{
		if (Frame.Color.Num() == 0)
		{
			return false;
		}
		Out.Dtype = FName(TEXT("bgra8"));
		Out.Bytes = reinterpret_cast<const uint8*>(Frame.Color.GetData());
		Out.NumBytes = Frame.Color.Num() * static_cast<int32>(sizeof(FColor));
	}
	return true;
}

void UMjCamera::PublishFrameToWorkers(const FMjCameraFrame& Frame)
{
	FMjCameraWireFrame Wire;
	if (!BuildCameraWireFrame(Frame, Wire))
	{
		return;
	}
	for (UURLabCameraPublishTransport* Transport : CameraPublishers)
	{
		if (Transport)
		{
			Transport->PublishCameraFrame(Wire);
		}
	}

	// Broadcast the same frame on the in-proc bus for any out-of-core image sink.
	// The payload points at the frame's pixel buffer and is valid only for the
	// duration of the broadcast.
	FMjCameraFramePayload Payload;
	Payload.CanonicalName = GetCanonicalName();
	Payload.Width = Frame.Width;
	Payload.Height = Frame.Height;
	Payload.bDepth = (CaptureMode == EMjCameraMode::Depth);
	Payload.SimTime = Frame.SimTime;
	Payload.FrameId = Frame.FrameId;
	Payload.Data = Wire.Bytes;
	Payload.DataNumBytes = Wire.NumBytes;
	Payload.RowStrideBytes = Payload.bDepth
								? Frame.Width * static_cast<int32>(sizeof(float))
								: Frame.Width * static_cast<int32>(sizeof(FColor));

	FMjCameraFrameBus::Get().OnFrameReady.Broadcast(Payload);
}

void UMjCamera::TouchRequested()
{
	LastRequestedSeconds.store(FPlatformTime::Seconds(), std::memory_order_release);
}

bool UMjCamera::IsCaptureActive() const
{
	// Broadcast cameras stream continuously; the rest capture only while recently
	// requested, within the active TTL.
	if (bEnableZmqBroadcast || bEnableShmBroadcast)
	{
		return true;
	}
	const double Last = LastRequestedSeconds.load(std::memory_order_acquire);
	if (Last <= 0.0)
	{
		return false;
	}
	return (FPlatformTime::Seconds() - Last) <= static_cast<double>(RequestActiveTtlSeconds);
}

FString UMjCamera::ResolveStreamEndpoint() const
{
	if (const AAMjManager* Manager = AAMjManager::GetManager())
	{
		if (Manager->BridgeServer)
		{
			return URLabBridgeServerConfigUtils::BuildCameraEndpoint(
				Manager->BridgeServer->GetInstanceConfig(), StreamPortIndex);
		}
	}
	return ZmqEndpoint;
}

FString UMjCamera::GetActualZmqEndpoint() const
{
	for (const UURLabCameraPublishTransport* Transport : CameraPublishers)
	{
		if (const UURLabZmqCameraPublishTransport* Zmq =
				Cast<UURLabZmqCameraPublishTransport>(Transport))
		{
			const FString Bound = Zmq->GetBoundEndpoint(StreamPortIndex);
			if (!Bound.IsEmpty())
			{
				return Bound;
			}
		}
	}
	// Not streaming yet, e.g. a dormant camera advertised in the hello handshake:
	// report the endpoint this camera WILL bind from its instance's port block, so
	// discovery never advertises the stale default port.
	return ResolveStreamEndpoint();
}

FString UMjCamera::GetCanonicalName() const
{
	if (!CanonicalOverride.IsNone())
	{
		return CanonicalOverride.ToString();
	}
	FName Art, Part;
	ResolveCameraCanonical(*this, Art, Part);
	return FMjCanonicalName::Full(Art, Part);
}
