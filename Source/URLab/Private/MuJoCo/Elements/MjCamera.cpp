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
#include "HAL/FileManager.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "HAL/ThreadSafeBool.h"
#include "Misc/Paths.h"
#include "RHICommandList.h"
#include "RenderingThread.h"

#include "Bridge/BridgeServer.h"
#include "Bridge/BridgeServerConfigUtils.h"
#include "MuJoCo/Capture/CameraShmWriter.h"
#include "MuJoCo/Capture/MjCameraFrameBus.h"
#include "MuJoCo/Capture/MjCameraSubsystem.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjDebugVisualizer.h"
#include "State/MjCanonicalName.h"
#include "Transport/NetworkManager.h"
#include "Transport/ShmPublishTransport.h"
#include "Utils/URLabLogging.h"

#include "zmq.h"

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

UMjDebugVisualizer* FindDebugVisualizer(UWorld* FallbackWorld = nullptr)
{
	if (AAMjManager* Manager = AAMjManager::GetManager())
	{
		return Manager->DebugVisualizer;
	}
	// Test and editor worlds do not dispatch BeginPlay, so the singleton may be unset.
	if (FallbackWorld)
	{
		for (TActorIterator<AAMjManager> It(FallbackWorld); It; ++It)
		{
			if (AAMjManager* Manager = *It)
			{
				return Manager->DebugVisualizer;
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
FString CameraLogName(const UMjCamera& Cam)
{
	return Cam.MjName.Get(Cam.GetName());
}
} // namespace

// ---------------------------------------------------------------------------
// Publisher thread
// ---------------------------------------------------------------------------

std::atomic<bool> FCameraZmqWorker::bPublishersPaused{false};

/**
 * The background thread that publishes camera frames over ZeroMQ.
 *
 * One per streaming camera, owning its own context and PUB socket. The camera
 * pushes finished frames onto a queue; this thread drains to the freshest one
 * and sends it as [topic][meta + pixels], the same layout the SHM transport
 * writes, so both consumers parse identically.
 */
class FCameraZmqPublisher final : public FRunnable
{
public:
	FCameraZmqPublisher(const FString& InEndpoint, const FString& InTopic, FIntPoint InResolution)
		: RequestedEndpoint(InEndpoint)
		, BoundEndpoint(InEndpoint)
		, Topic(InTopic)
		, Resolution(InResolution)
	{
	}

	virtual ~FCameraZmqPublisher() override { Stop(); }

	virtual bool Init() override
	{
		ZmqContext = zmq_ctx_new();
		if (!ZmqContext)
		{
			UE_LOG(LogURLabNet, Error, TEXT("CameraZmqPublisher: zmq_ctx_new failed"));
			return false;
		}
		ZmqPublisher = zmq_socket(ZmqContext, ZMQ_PUB);
		if (!ZmqPublisher)
		{
			UE_LOG(LogURLabNet, Error, TEXT("CameraZmqPublisher: zmq_socket failed"));
			zmq_ctx_term(ZmqContext);
			ZmqContext = nullptr;
			return false;
		}

		// A live feed only cares about the FRESHEST frame, so keep the send queue
		// shallow: at HWM=1 the socket holds at most one frame in flight and a slow
		// consumer gets a near-latest frame instead of draining a backlog.
		int Hwm = 1;
		zmq_setsockopt(ZmqPublisher, ZMQ_SNDHWM, &Hwm, sizeof(Hwm));
		// LINGER=0, so a connected-but-not-reading subscriber can never block
		// zmq_ctx_term at shutdown; libzmq's default there is infinite.
		int Linger = 0;
		zmq_setsockopt(ZmqPublisher, ZMQ_LINGER, &Linger, sizeof(Linger));

		// Auto-increment the port on a bind conflict, so co-located cameras (and
		// co-located editor processes) do not fight over one port.
		FString BaseAddr = TEXT("tcp://0.0.0.0:");
		int32 Port = 5558;
		if (RequestedEndpoint.Contains(TEXT(":")))
		{
			FString Left, Right;
			RequestedEndpoint.Split(TEXT(":"), &Left, &Right, ESearchCase::IgnoreCase, ESearchDir::FromEnd);
			if (Right.IsNumeric())
			{
				Port = FCString::Atoi(*Right);
				BaseAddr = Left + TEXT(":");
			}
		}

		int Rc = -1;
		for (int32 i = 0; i < 10; ++i)
		{
			const FString TryEndpoint = FString::Printf(TEXT("%s%d"), *BaseAddr, Port + i);
			Rc = zmq_bind(ZmqPublisher, TCHAR_TO_UTF8(*TryEndpoint));
			if (Rc == 0)
			{
				BoundEndpoint = TryEndpoint;
				break;
			}
		}

		if (Rc != 0)
		{
			UE_LOG(LogURLabNet, Error,
				TEXT("CameraZmqPublisher failed to bind ZMQ after 10 retries, starting at %s"),
				*RequestedEndpoint);
			// Release the half-open socket and context, so a failed Init leaks nothing.
			zmq_close(ZmqPublisher);
			ZmqPublisher = nullptr;
			zmq_ctx_term(ZmqContext);
			ZmqContext = nullptr;
			return false;
		}

		UE_LOG(LogURLabNet, Log, TEXT("CameraZmqPublisher bound at %s [Topic: %s]"), *BoundEndpoint, *Topic);
		return true;
	}

	virtual uint32 Run() override
	{
		const int32 ExpectedPixels = Resolution.X * Resolution.Y;
		while (!bStopThread)
		{
			bool bSent = false;

			// Drain to the FRESHEST frame: if the producer outran us, skip the
			// backlog rather than send stale frames FIFO.
			FQueuedColorFrame ColorFrame;
			bool bHaveColor = false;
			while (FrameQueue.Dequeue(ColorFrame))
			{
				bHaveColor = true;
			}
			if (bHaveColor)
			{
				if (ColorFrame.Pixels.Num() == ExpectedPixels)
				{
					SendFrame(ColorFrame.Meta, ColorFrame.Pixels.GetData(),
						ColorFrame.Pixels.Num() * sizeof(FColor));
				}
				bSent = true;
			}

			FQueuedFloatFrame FloatFrame;
			bool bHaveFloat = false;
			while (FloatFrameQueue.Dequeue(FloatFrame))
			{
				bHaveFloat = true;
			}
			if (bHaveFloat)
			{
				if (FloatFrame.Pixels.Num() == ExpectedPixels)
				{
					SendFrame(FloatFrame.Meta, FloatFrame.Pixels.GetData(),
						FloatFrame.Pixels.Num() * sizeof(float));
				}
				bSent = true;
			}

			if (!bSent)
			{
				FPlatformProcess::Sleep(0.002f);
			}
		}
		return 0;
	}

	virtual void Stop() override { bStopThread = true; }

	virtual void Exit() override
	{
		if (ZmqPublisher)
		{
			zmq_close(ZmqPublisher);
			ZmqPublisher = nullptr;
		}
		if (ZmqContext)
		{
			zmq_ctx_term(ZmqContext);
			ZmqContext = nullptr;
		}
	}

	// Enqueue unconditionally: Run drains to the latest frame before sending and
	// the socket runs at HWM=1, so a backlog here collapses to the freshest frame
	// rather than streaming FIFO.
	void PushFrame(const TArray<FColor>& Pixels, const FMjCameraFrameMeta& Meta)
	{
		FrameQueue.Enqueue(FQueuedColorFrame{Meta, Pixels});
	}

	void PushFrame(const TArray<float>& Pixels, const FMjCameraFrameMeta& Meta)
	{
		FloatFrameQueue.Enqueue(FQueuedFloatFrame{Meta, Pixels});
	}

	const FString& GetBoundEndpoint() const { return BoundEndpoint; }

private:
	/**
	 * Publish one message as [topic][meta + pixels]. The metadata header is
	 * prepended to the pixel bytes in a single payload frame, so the ZMQ and SHM
	 * consumers parse an identical (meta, pixels) layout.
	 */
	void SendFrame(const FMjCameraFrameMeta& Meta, const void* Pixels, size_t PixelBytes)
	{
		if (FCameraZmqWorker::bPublishersPaused.load(std::memory_order_acquire))
		{
			return;
		}
		TArray<uint8> Payload;
		Payload.SetNumUninitialized(sizeof(FMjCameraFrameMeta) + static_cast<int32>(PixelBytes));
		FMemory::Memcpy(Payload.GetData(), &Meta, sizeof(FMjCameraFrameMeta));
		FMemory::Memcpy(Payload.GetData() + sizeof(FMjCameraFrameMeta), Pixels, PixelBytes);

		const FString TopicSpace = Topic + TEXT(" ");
		const FTCHARToUTF8 TopicUtf8(*TopicSpace);
		if (zmq_send(ZmqPublisher, TopicUtf8.Get(), TopicUtf8.Length(), ZMQ_SNDMORE) < 0)
		{
			// Topic frame dropped, e.g. at the high-water mark. Skip the body too,
			// or the multipart message desyncs.
			return;
		}
		// Best-effort feed: a body dropped under HWM=1 just means this frame is
		// skipped, and the next push carries a fresher one.
		zmq_send(ZmqPublisher, Payload.GetData(), Payload.Num(), 0);
	}

	// Each queued frame carries its metadata header, so Run can prepend it to the
	// published bytes and the client can associate the frame with the step that
	// produced it.
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

	// Two queues, one per pixel format. Real and segmentation cameras drive the
	// colour queue, depth cameras the float one. A camera's CaptureMode never
	// changes after streaming starts, so only one is ever active per publisher.
	TQueue<FQueuedColorFrame, EQueueMode::Spsc> FrameQueue;
	TQueue<FQueuedFloatFrame, EQueueMode::Spsc> FloatFrameQueue;

	FString RequestedEndpoint;
	FString BoundEndpoint;
	FString Topic;
	FIntPoint Resolution = FIntPoint::ZeroValue;

	void* ZmqContext = nullptr;
	void* ZmqPublisher = nullptr;

	FThreadSafeBool bStopThread{false};
};

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

	if (PublisherThread)
	{
		PublisherThread->Kill(true);
		delete PublisherThread;
		PublisherThread = nullptr;
	}
	if (ZmqPublisher)
	{
		delete ZmqPublisher;
		ZmqPublisher = nullptr;
	}

	// Stop rendering when the actor is torn down.
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

	if (bStreamingEnabled && bActive)
	{
		AAMjManager* Manager = AAMjManager::GetManager();
		MaybeCapture(Manager);
		PublishDueDelayedFrames(Manager);
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

		TSharedPtr<FMjCameraFrame> Frame = MakeShared<FMjCameraFrame>();
		Frame->FrameId = Front.FrameId;
		Frame->SimTime = Front.SimTime;
		Frame->Width = Front.Width;
		Frame->Height = Front.Height;
		Frame->CaptureUnixTime = Front.CaptureUnixSeconds;

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
		ENQUEUE_RENDER_COMMAND(MjCameraMapReadback)
		([Frame, Gpu, W, H, CapturedMode, Results = ResultsQueue](FRHICommandListImmediate&) {
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
	if (AAMjManager* Manager = AAMjManager::GetManager())
	{
		ShowId = Manager->GetLastAppliedFrameId();
		ShowTime = Manager->GetLastAppliedSimTime();
	}
	EnqueueReadback(ShowId, ShowTime, /*bForceSubmit=*/true);
}

void UMjCamera::MaybeCapture(AAMjManager* Mgr)
{
	const uint64 AppliedId = Mgr ? Mgr->GetLastAppliedFrameId() : 0;
	const double AppliedTime = Mgr ? Mgr->GetLastAppliedSimTime() : 0.0;
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
			bIssued = EnqueueReadback(AppliedId, AppliedTime);
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

void UMjCamera::PublishDueDelayedFrames(AAMjManager* Mgr)
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
	const double NowVal = bDelayUseWallClock
							? (FDateTime::UtcNow() - FDateTime(1970, 1, 1)).GetTotalSeconds()
							: (Mgr ? Mgr->GetLastAppliedSimTime() : 0.0);
	TSharedPtr<const FMjCameraFrame> Selected = SelectDelayedFrameShared(NowVal, LastPublishedSeq);
	if (Selected.IsValid())
	{
		PublishFrameToWorkers(*Selected);
		LastPublishedSeq = Selected->Seq;
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

	UMjDebugVisualizer* Visualizer = FindDebugVisualizer(GetWorld());
	if (!Visualizer)
	{
		return;
	}

	CaptureComponent->HiddenComponents.Reset();

	TArray<UPrimitiveComponent*> Pool;
	Visualizer->GetSegPoolSiblings(EMjCameraMode::InstanceSegmentation, Pool);
	for (UPrimitiveComponent* Sibling : Pool)
	{
		CaptureComponent->HiddenComponents.Add(Sibling);
	}

	Pool.Reset();
	Visualizer->GetSegPoolSiblings(EMjCameraMode::SemanticSegmentation, Pool);
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
			if (UMjDebugVisualizer* Visualizer = FindDebugVisualizer(GetWorld()))
			{
				TArray<UPrimitiveComponent*> Siblings;
				Visualizer->AcquireSegPool(CaptureMode, this, Siblings);

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
					TEXT("[MjCamera] '%s' seg mode requested but no DebugVisualizer found; seg cam will show nothing."),
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

		if (bEnableZmqBroadcast && !ZmqPublisher)
		{
			const FString Topic = GetCanonicalName();

			// Bind inside this instance's camera port block, so several editors
			// acting as render servers never fight over one port.
			ZmqEndpoint = ResolveStreamEndpoint();
			ZmqPublisher = new FCameraZmqPublisher(ZmqEndpoint, Topic, CaptureResolution());
			PublisherThread = FRunnableThread::Create(ZmqPublisher, TEXT("CameraZmqPublisherThread"), 0, TPri_BelowNormal);
			if (!PublisherThread)
			{
				// Drop the publisher rather than hold a runnable nothing drives.
				UE_LOG(LogURLabNet, Error,
					TEXT("[MjCamera] '%s' failed to create ZMQ publisher thread; disabling ZMQ broadcast"),
					*CameraLogName(*this));
				delete ZmqPublisher;
				ZmqPublisher = nullptr;
			}
		}

		// The SHM publisher opens an mmap'd file under the live URLab session dir.
		if (bEnableShmBroadcast && !ShmWriter)
		{
			// The "live" session segment is process-global on one host: parameterise
			// it per instance before running several editors as render servers.
			const FString Dir = UURLabShmPublishTransport::ResolveSessionDir(TEXT("live"));
			IFileManager::Get().MakeDirectory(*Dir, /*Tree=*/true);
			FName CanonArt, CanonPart;
			ResolveCameraCanonical(*this, CanonArt, CanonPart);
			const FString FileName = FString::Printf(
				TEXT("cam_%s_%s.shm"), *CanonArt.ToString(), *CanonPart.ToString());
			const FString FullPath = FPaths::Combine(Dir, FileName);

			ShmWriter = new FCameraShmWriter();
			if (!ShmWriter->Open(FullPath, CaptureResolution()))
			{
				delete ShmWriter;
				ShmWriter = nullptr;
			}
			else
			{
				UE_LOG(LogURLabNet, Log, TEXT("[MjCamera] '%s' SHM broadcast at %s"),
					*CameraLogName(*this), *FullPath);
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

		// Release the seg pool first, while CaptureMode still says what we subscribed as.
		if (IsSegMode(CaptureMode))
		{
			if (UMjDebugVisualizer* Visualizer = FindDebugVisualizer(GetWorld()))
			{
				Visualizer->ReleaseSegPool(CaptureMode, this);
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

		if (PublisherThread)
		{
			PublisherThread->Kill(true);
			delete PublisherThread;
			PublisherThread = nullptr;
		}
		if (ZmqPublisher)
		{
			delete ZmqPublisher;
			ZmqPublisher = nullptr;
		}

		if (ShmWriter)
		{
			ShmWriter->Close(/*bDeleteFile=*/true);
			delete ShmWriter;
			ShmWriter = nullptr;
		}

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
	if (AAMjManager* Manager = AAMjManager::GetManager())
	{
		EnqueueReadback(Manager->GetLastAppliedFrameId(), Manager->GetLastAppliedSimTime());
	}
	else
	{
		EnqueueReadback(0, 0.0);
	}
}

bool UMjCamera::EnqueueReadback(uint64 ShowFrameId, double ShowSimTime, bool bForceSubmit)
{
	// Pipeline cap: keep at most MaxInFlightReadbacks async copies outstanding.
	if (!RenderTarget || InFlightReadbacks.Num() >= MaxInFlightReadbacks)
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

	InFlightReadbacks.Add(MoveTemp(Entry));
	return true;
}

// ---------------------------------------------------------------------------
// Frame history
// ---------------------------------------------------------------------------

void UMjCamera::PushFrameToHistory(FMjCameraFrame&& Frame)
{
	TSharedPtr<FMjCameraFrame> Shared = MakeShared<FMjCameraFrame>(MoveTemp(Frame));
	PushFrameToHistoryShared(Shared);
}

void UMjCamera::PushFrameToHistoryShared(const TSharedPtr<const FMjCameraFrame>& Frame)
{
	if (!Frame.IsValid())
	{
		return;
	}

	FScopeLock Lock(&HistoryLock);
	History.Add(Frame);

	// A hard frame ceiling regardless of mode: one frame can be megabytes and many
	// cameras share the budget, so bound worst-case retention. It is the same
	// constant that backs HistoryCapacity's ClampMax, so the two limits are one
	// source of truth and cannot disagree.
	constexpr int32 HardCap = MaxHistoryCapacity;

	if (!IsDelayActive())
	{
		const int32 Cap = FMath::Clamp(HistoryCapacity, 1, HardCap);
		while (History.Num() > Cap)
		{
			History.RemoveAt(0);
		}
		return;
	}

	// Latency emulation retains enough history to cover the delay window, so the
	// reveal-time selection always has the frame it needs. A front frame is evicted
	// only once it is older than the window behind the newest, which is self-sizing
	// and independent of frame rate, and still capped by the memory ceiling.
	const double RetainWindow = static_cast<double>(DelaySeconds + DelayJitterSeconds) + 0.10;
	const double NewestClock = FrameClock(*History.Last());
	while (History.Num() > 1)
	{
		const bool bExpired = (NewestClock - FrameClock(*History[0])) > RetainWindow;
		if (History.Num() > HardCap || bExpired)
		{
			History.RemoveAt(0);
		}
		else
		{
			break;
		}
	}
}

TSharedPtr<const FMjCameraFrame> UMjCamera::GetFrameShared(uint64 MinFrameId) const
{
	FScopeLock Lock(&HistoryLock);
	if (History.Num() == 0)
	{
		return nullptr;
	}
	if (MinFrameId == 0)
	{
		return History.Last();
	}
	// The oldest retained frame at or after the requested step (history is oldest
	// first), i.e. the frame that shows state at least MinFrameId.
	for (const TSharedPtr<const FMjCameraFrame>& Frame : History)
	{
		if (Frame->FrameId >= MinFrameId)
		{
			return Frame;
		}
	}
	return nullptr;
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
	FScopeLock Lock(&HistoryLock);
	return History.Num() > 0 ? History.Last()->FrameId : 0;
}

TSharedPtr<const FMjCameraFrame> UMjCamera::SelectDelayedFrameShared(double NowValue, uint64 AfterSeq) const
{
	FScopeLock Lock(&HistoryLock);
	for (int32 i = History.Num() - 1; i >= 0; --i)
	{
		if (History[i]->RevealValue <= NowValue)
		{
			// The newest eligible frame. Deliver it only if it is newer than the last
			// one published, so the stream never repeats or rewinds.
			if (History[i]->Seq > AfterSeq)
			{
				return History[i];
			}
			return nullptr;
		}
	}
	return nullptr;
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

double UMjCamera::FrameClock(const FMjCameraFrame& Frame) const
{
	return bDelayUseWallClock ? Frame.CaptureUnixTime : Frame.SimTime;
}

double UMjCamera::NowClockValue() const
{
	if (bDelayUseWallClock)
	{
		return (FDateTime::UtcNow() - FDateTime(1970, 1, 1)).GetTotalSeconds();
	}
	const AAMjManager* Manager = AAMjManager::GetManager();
	return Manager ? Manager->GetLastAppliedSimTime() : 0.0;
}

double UMjCamera::SampleDelaySeconds()
{
	double D = static_cast<double>(DelaySeconds);
	if (DelayJitterSeconds > 0.0f)
	{
		// Draw from the RNG only when jitter is configured, so a fixed delay stays
		// deterministic and does not advance the stream.
		const double J = static_cast<double>(DelayJitterSeconds);
		D += DelayRng.FRandRange(-J, J);
	}
	return FMath::Max(0.0, D);
}

void UMjCamera::SetCameraDelay(float InDelaySeconds, float InJitterSeconds, bool bInUseWallClock, int32 InSeed)
{
	DelaySeconds = FMath::Max(0.0f, InDelaySeconds);
	DelayJitterSeconds = FMath::Max(0.0f, InJitterSeconds);
	bDelayUseWallClock = bInUseWallClock;
	const int32 Seed = (InSeed != 0) ? InSeed : static_cast<int32>(GetTypeHash(GetCanonicalName()));
	DelayRng.Initialize(Seed);
	// Re-arm the publish dedup, so the new policy re-selects cleanly.
	LastPublishedSeq = 0;
}

void UMjCamera::SetCaptureRate(bool bInOnStateChange, float InMaxFps)
{
	bCaptureOnStateChange = bInOnStateChange;
	CaptureMaxFps = FMath::Max(0.0f, InMaxFps);
}

// ---------------------------------------------------------------------------
// Publishing and identity
// ---------------------------------------------------------------------------

void UMjCamera::PublishFrameToWorkers(const FMjCameraFrame& Frame)
{
	FMjCameraFrameMeta Meta;
	Meta.FrameId = Frame.FrameId;
	Meta.SimTime = Frame.SimTime;
	Meta.Width = static_cast<uint32>(Frame.Width);
	Meta.Height = static_cast<uint32>(Frame.Height);
	// The original capture time, so a delayed frame reports the moment it was
	// taken and the client's content age reflects the injected latency.
	Meta.CaptureUnixTime = Frame.CaptureUnixTime;

	// The payload points at the frame's pixel buffer and is valid only for the
	// duration of the broadcast.
	FMjCameraFramePayload Payload;
	Payload.CanonicalName = GetCanonicalName();
	Payload.Width = Frame.Width;
	Payload.Height = Frame.Height;
	Payload.bDepth = (CaptureMode == EMjCameraMode::Depth);
	Payload.SimTime = Frame.SimTime;
	Payload.FrameId = Frame.FrameId;

	if (CaptureMode == EMjCameraMode::Depth)
	{
		if (Frame.Depth.Num() == 0)
		{
			return;
		}
		if (bEnableZmqBroadcast && ZmqPublisher)
		{
			ZmqPublisher->PushFrame(Frame.Depth, Meta);
		}
		if (bEnableShmBroadcast && ShmWriter)
		{
			ShmWriter->PushFrame(Frame.Depth, Meta);
		}

		Payload.Data = reinterpret_cast<const uint8*>(Frame.Depth.GetData());
		Payload.DataNumBytes = Frame.Depth.Num() * static_cast<int32>(sizeof(float));
		Payload.RowStrideBytes = Frame.Width * static_cast<int32>(sizeof(float));
	}
	else
	{
		if (Frame.Color.Num() == 0)
		{
			return;
		}
		if (bEnableZmqBroadcast && ZmqPublisher)
		{
			ZmqPublisher->PushFrame(Frame.Color, Meta);
		}
		if (bEnableShmBroadcast && ShmWriter)
		{
			ShmWriter->PushFrame(Frame.Color, Meta);
		}

		Payload.Data = reinterpret_cast<const uint8*>(Frame.Color.GetData());
		Payload.DataNumBytes = Frame.Color.Num() * static_cast<int32>(sizeof(FColor));
		Payload.RowStrideBytes = Frame.Width * static_cast<int32>(sizeof(FColor));
	}

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
	if (ZmqPublisher)
	{
		return ZmqPublisher->GetBoundEndpoint();
	}
	// Not streaming yet, e.g. a dormant camera advertised in the hello handshake:
	// report the endpoint this camera WILL bind from its instance's port block, so
	// discovery never advertises the stale default port.
	return ResolveStreamEndpoint();
}

FString UMjCamera::GetCanonicalName() const
{
	FName Art, Part;
	ResolveCameraCanonical(*this, Art, Part);
	return FMjCanonicalName::Full(Art, Part);
}
