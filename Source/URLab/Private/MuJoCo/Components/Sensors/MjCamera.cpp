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

#include "MuJoCo/Components/Sensors/MjCamera.h"
#include "MuJoCo/Components/Sensors/MjCameraSubsystem.h"
#include "MuJoCo/Components/Sensors/CameraShmWriter.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjDebugVisualizer.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/BridgeServerConfigUtils.h"
#include "Transport/NetworkManager.h"
#include "Transport/ShmPublishTransport.h" // ResolveSessionDir
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "Engine/PostProcessVolume.h"
#include "EngineUtils.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "MuJoCo/Utils/MjXmlUtils.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/Engine.h"
#include "ContentStreaming.h"
#include "XmlNode.h"
#include "RHICommandList.h"
#include "RenderingThread.h"
#include "HAL/ThreadSafeBool.h"
#include "HAL/RunnableThread.h"
#include "Utils/URLabLogging.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "State/MjCanonicalName.h"
#include "zmq.h"

namespace
{
// Resolve a camera's canonical <art>/<part> segments through the single naming
// owner. The art segment is the owning articulation's name, or the owning
// actor's name for a manager-level global camera; the part is the MJCF camera
// name (or UE component name if unset) with the art prefix stripped.
void ResolveCameraCanonical(const UMjCamera& Cam, FName& OutArt, FName& OutPart)
{
	const AActor* Owner = Cam.GetOwner();
	const AMjArticulation* Art = Cast<AMjArticulation>(Owner);
	OutArt = Art ? FMjCanonicalName::ArtSegment(Art)
				 : FName(*FMjCanonicalName::Sanitize(Owner ? Owner->GetName() : TEXT("unknown")));
	FString Source = Cam.GetMjName();
	if (Source.IsEmpty())
		Source = Cam.GetName();
	OutPart = FMjCanonicalName::PartSegment(Art, Source);
}
} // namespace

// ---------------------------------------------------------------------------
// FCameraZmqWorker
// ---------------------------------------------------------------------------

std::atomic<bool> FCameraZmqWorker::bPublishersPaused{false};

// All transport internals live here so the header carries no raw libzmq handles.
struct FCameraZmqWorker::FState
{
	FString RequestedEndpoint;
	FString BoundEndpoint;
	FString Topic;
	FIntPoint Resolution = FIntPoint::ZeroValue;

	void* ZmqContext = nullptr;
	void* ZmqPublisher = nullptr;

	FThreadSafeBool bStopThread{false};

	// Each queued frame carries its metadata header so the Run() loop can prepend
	// it to the published bytes (the client associates the streamed frame with the
	// step that produced it via Meta.FrameId).
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

	// Two queues, one per pixel format. Real / seg cameras drive the FColor queue,
	// depth cameras drive the float queue. Per-camera CaptureMode never changes
	// after streaming starts, so only one queue is ever active per worker.
	TQueue<FQueuedColorFrame, EQueueMode::Spsc> FrameQueue;
	TQueue<FQueuedFloatFrame, EQueueMode::Spsc> FloatFrameQueue;
};

FCameraZmqWorker::FCameraZmqWorker(const FString& InEndpoint, const FString& InTopic, FIntPoint InRes)
{
	State = MakePimpl<FState>();
	State->RequestedEndpoint = InEndpoint;
	State->BoundEndpoint = InEndpoint;
	State->Topic = InTopic;
	State->Resolution = InRes;
}

FCameraZmqWorker::~FCameraZmqWorker()
{
	Stop();
}

bool FCameraZmqWorker::Init()
{
	State->ZmqContext = zmq_ctx_new();
	if (!State->ZmqContext)
	{
		UE_LOG(LogURLabNet, Error, TEXT("CameraZmqWorker: zmq_ctx_new failed"));
		return false;
	}
	State->ZmqPublisher = zmq_socket(State->ZmqContext, ZMQ_PUB);
	if (!State->ZmqPublisher)
	{
		UE_LOG(LogURLabNet, Error, TEXT("CameraZmqWorker: zmq_socket failed"));
		zmq_ctx_term(State->ZmqContext);
		State->ZmqContext = nullptr;
		return false;
	}

	// A live camera feed only cares about the FRESHEST frame, so keep the PUB send
	// queue shallow. With HWM=1 the PUB holds at most one frame in flight, so a
	// slow consumer always gets a near-latest frame instead of draining a backlog.
	int hwm = 1;
	zmq_setsockopt(State->ZmqPublisher, ZMQ_SNDHWM, &hwm, sizeof(hwm));
	// LINGER=0 so a connected-but-not-reading subscriber can never block
	// zmq_ctx_term at shutdown (libzmq default is infinite).
	int linger = 0;
	zmq_setsockopt(State->ZmqPublisher, ZMQ_LINGER, &linger, sizeof(linger));

	// Auto-increment the port on bind conflict so co-located cameras (and
	// co-located editor processes) don't fight over a single port.
	FString BaseAddr = TEXT("tcp://0.0.0.0:");
	int32 Port = 5558;
	if (State->RequestedEndpoint.Contains(TEXT(":")))
	{
		FString Left, Right;
		State->RequestedEndpoint.Split(TEXT(":"), &Left, &Right, ESearchCase::IgnoreCase, ESearchDir::FromEnd);
		if (Right.IsNumeric())
		{
			Port = FCString::Atoi(*Right);
			BaseAddr = Left + TEXT(":");
		}
	}

	int rc = -1;
	for (int i = 0; i < 10; ++i)
	{
		const FString TryEndpoint = FString::Printf(TEXT("%s%d"), *BaseAddr, Port + i);
		rc = zmq_bind(State->ZmqPublisher, TCHAR_TO_UTF8(*TryEndpoint));
		if (rc == 0)
		{
			State->BoundEndpoint = TryEndpoint;
			break;
		}
	}

	if (rc != 0)
	{
		UE_LOG(LogURLabNet, Error,
			TEXT("CameraZmqWorker failed to bind ZMQ after 10 retries, starting at %s"),
			*State->RequestedEndpoint);
		// Release the half-open socket + context so a failed Init leaks nothing.
		zmq_close(State->ZmqPublisher);
		State->ZmqPublisher = nullptr;
		zmq_ctx_term(State->ZmqContext);
		State->ZmqContext = nullptr;
		return false;
	}

	UE_LOG(LogURLabNet, Log, TEXT("CameraZmqWorker bound at %s [Topic: %s]"),
		*State->BoundEndpoint, *State->Topic);
	return true;
}

uint32 FCameraZmqWorker::Run()
{
	// Publish one message as [topic][meta + pixels]. The metadata header is
	// prepended to the pixel bytes in a single payload frame so the ZMQ and SHM
	// consumers parse an identical (meta, pixels) layout.
	auto SendFrame = [this](const FMjCameraFrameMeta& Meta, const void* Pixels, size_t PixelBytes) {
		if (bPublishersPaused.load(std::memory_order_acquire))
			return;
		TArray<uint8> Payload;
		Payload.SetNumUninitialized(sizeof(FMjCameraFrameMeta) + static_cast<int32>(PixelBytes));
		FMemory::Memcpy(Payload.GetData(), &Meta, sizeof(FMjCameraFrameMeta));
		FMemory::Memcpy(Payload.GetData() + sizeof(FMjCameraFrameMeta), Pixels, PixelBytes);

		const FString TopicSpace = State->Topic + TEXT(" ");
		const FTCHARToUTF8 TopicUtf8(*TopicSpace);
		if (zmq_send(State->ZmqPublisher, TopicUtf8.Get(), TopicUtf8.Length(), ZMQ_SNDMORE) < 0)
			return; // topic frame dropped (e.g. HWM); skip the body so we don't desync the multipart message
		if (zmq_send(State->ZmqPublisher, Payload.GetData(), Payload.Num(), 0) < 0)
		{
			// Best-effort feed: a dropped body under HWM=1 just means this frame is
			// skipped; the next push carries a fresher one.
		}
	};

	const int32 ExpectedPixels = State->Resolution.X * State->Resolution.Y;
	while (!State->bStopThread)
	{
		bool bSent = false;

		// Drain to the FRESHEST frame: a live feed only wants the latest, so if the
		// producer outran us, skip the backlog rather than send stale frames FIFO.
		FState::FQueuedColorFrame ColorFrame;
		bool bHaveColor = false;
		while (State->FrameQueue.Dequeue(ColorFrame))
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

		FState::FQueuedFloatFrame FloatFrame;
		bool bHaveFloat = false;
		while (State->FloatFrameQueue.Dequeue(FloatFrame))
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

void FCameraZmqWorker::Stop()
{
	if (State)
	{
		State->bStopThread = true;
	}
}

void FCameraZmqWorker::Exit()
{
	if (!State)
		return;
	if (State->ZmqPublisher)
	{
		zmq_close(State->ZmqPublisher);
		State->ZmqPublisher = nullptr;
	}
	if (State->ZmqContext)
	{
		zmq_ctx_term(State->ZmqContext);
		State->ZmqContext = nullptr;
	}
}

void FCameraZmqWorker::PushFrame(const TArray<FColor>& FrameData, const FMjCameraFrameMeta& Meta)
{
	// Enqueue unconditionally; the worker drains this queue to the latest frame
	// before sending (and the PUB socket runs HWM=1), so a backlog here is
	// collapsed to the freshest frame rather than streamed FIFO.
	State->FrameQueue.Enqueue(FState::FQueuedColorFrame{Meta, FrameData});
}

void FCameraZmqWorker::PushFrame(const TArray<float>& FrameData, const FMjCameraFrameMeta& Meta)
{
	State->FloatFrameQueue.Enqueue(FState::FQueuedFloatFrame{Meta, FrameData});
}

FString FCameraZmqWorker::GetBoundEndpoint() const
{
	return State ? State->BoundEndpoint : FString();
}

// ---------------------------------------------------------------------------
// UMjCamera
// ---------------------------------------------------------------------------

namespace
{
bool IsSegMode(EMjCameraMode mode)
{
	return mode == EMjCameraMode::SemanticSegmentation
		|| mode == EMjCameraMode::InstanceSegmentation;
}

// MuJoCo `fovy` is the VERTICAL field of view; UE SceneCaptureComponent2D
// `FOVAngle` is the HORIZONTAL FOV. Copying fovy->FOVAngle verbatim over-narrows
// the view on non-square render targets (looks "zoomed in"). Convert via the RT
// aspect so UE's derived vertical FOV matches MuJoCo fovy exactly. Identity when
// the RT is square.
float HorizontalFOVFromFovy(float Fovy, FIntPoint Resolution)
{
	// A camera with no fovy set (0) produces a degenerate zero-FOV frustum that
	// renders pure black. Fall back to MuJoCo's default vertical fov (45 deg).
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

UMjDebugVisualizer* FindDebugVisualizer(UWorld* FallbackWorld = nullptr)
{
	if (AAMjManager* Manager = AAMjManager::GetManager())
	{
		return Manager->DebugVisualizer;
	}
	// Test/editor worlds don't dispatch BeginPlay, so the singleton may be unset.
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
} // namespace

UMjCamera::UMjCamera()
{
	// The capture pipeline is driven once per frame by UMjCameraSubsystem, not a
	// per-component tick.
	PrimaryComponentTick.bCanEverTick = false;
	PrimaryComponentTick.bStartWithTickEnabled = false;

	// Default resolution: codegen emits resolution as TArray<int32>{} (empty); seed [w,h].
	resolution = {640, 480};

	ResultsQueue = MakeShared<FCompletedReadbackQueue, ESPMode::ThreadSafe>();

	// Create the scene capture sub-component
	CaptureComponent = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("SceneCapture"));
	if (CaptureComponent)
	{
		CaptureComponent->SetupAttachment(this);
		CaptureComponent->SetRelativeScale3D(FVector(0.15f));

		// MuJoCo cameras look down -Z (forward), +Y (up).
		// Unreal cameras look down +X (forward), +Z (up).
		// MjToUERotation negates X/Z quat components (handedness flip),
		// which mirrors the Y axis, so "up" becomes -Y after conversion.
		const FVector MjForward = FVector(0.0f, 0.0f, -1.0f);
		const FVector MjUp = FVector(0.0f, -1.0f, 0.0f);
		const FRotator CorrectionRot = FRotationMatrix::MakeFromXZ(MjForward, MjUp).Rotator();
		CaptureComponent->SetRelativeRotation(CorrectionRot);

		// Start dormant: no capture cost until explicitly enabled.
		CaptureComponent->bCaptureEveryFrame = false;
		CaptureComponent->bCaptureOnMovement = false;
		CaptureComponent->bAlwaysPersistRenderingState = true;
		CaptureComponent->MaxViewDistanceOverride = -1.0f;

		// SceneCaptureComponent2D does NOT automatically respect scene Post Process
		// Volumes. Set PostProcessBlendWeight=1 so the component's own settings are
		// used; at BeginPlay we copy from the scene PPV to match the viewport look.
		CaptureComponent->PostProcessBlendWeight = 1.0f;
		CaptureComponent->bUseRayTracingIfEnabled = false;

		CaptureComponent->bHiddenInGame = true;
	}
}

void UMjCamera::OnRegister()
{
	Super::OnRegister();
	if (CaptureComponent)
	{
		CaptureComponent->FOVAngle = HorizontalFOVFromFovy(fovy, GetResolution());
	}
}

void UMjCamera::BeginPlay()
{
	Super::BeginPlay();

	if (AAMjManager* Manager = AAMjManager::GetManager())
	{
		if (Manager->NetworkManager)
			Manager->NetworkManager->RegisterCamera(this);
	}

	// Copy post-process settings from the scene's Post Process Volume(s) so the
	// capture component matches the viewport look.
	if (CaptureComponent && GetWorld())
	{
		for (TActorIterator<APostProcessVolume> It(GetWorld()); It; ++It)
		{
			APostProcessVolume* PPV = *It;
			if (PPV && PPV->bEnabled)
			{
				CaptureComponent->PostProcessSettings = PPV->Settings;
				CaptureComponent->PostProcessBlendWeight = 1.0f;
				UE_LOG(LogURLab, Log, TEXT("[MjCamera] Copied post-process settings from '%s'"), *PPV->GetName());
				break; // Use the first enabled PPV
			}
		}
	}

	if (bEnableZmqBroadcast)
	{
		SetStreamingEnabled(true);
	}

	// Register with the world subsystem that drives the per-frame capture pass.
	if (UWorld* World = GetWorld())
	{
		if (UMjCameraSubsystem* Sub = World->GetSubsystem<UMjCameraSubsystem>())
		{
			Sub->RegisterCamera(this);
		}
	}
}

void UMjCamera::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (UWorld* World = GetWorld())
	{
		if (UMjCameraSubsystem* Sub = World->GetSubsystem<UMjCameraSubsystem>())
		{
			Sub->UnregisterCamera(this);
		}
	}

	if (AAMjManager* Manager = AAMjManager::GetManager())
	{
		if (Manager->NetworkManager)
			Manager->NetworkManager->UnregisterCamera(this);
	}

	if (WorkerThread)
	{
		WorkerThread->Kill(true);
		delete WorkerThread;
		WorkerThread = nullptr;
	}
	if (ZmqWorker)
	{
		delete ZmqWorker;
		ZmqWorker = nullptr;
	}

	// Make sure we stop rendering when the actor is torn down.
	if (bStreamingEnabled)
	{
		SetStreamingEnabled(false);
	}
	Super::EndPlay(EndPlayReason);
}

FIntPoint UMjCamera::GetResolution() const
{
	const int32 W = (resolution.Num() > 0 && resolution[0] > 0) ? resolution[0] : 640;
	const int32 H = (resolution.Num() > 1 && resolution[1] > 0) ? resolution[1] : 480;
	return FIntPoint(W, H);
}

void UMjCamera::NormalizeResolution()
{
	const FIntPoint R = GetResolution();
	resolution.SetNum(2);
	resolution[0] = R.X;
	resolution[1] = R.Y;
}

void UMjCamera::UpdateCapturePipeline()
{
	const bool bActive = IsCaptureActive();

	// Per-camera capture gating: lazily start capturing when a camera becomes
	// active (e.g. first include_cameras request on a non-broadcast camera), and
	// pause the GPU scene capture when it goes dormant so a scene with many
	// cameras only pays for the ones actually consumed.
	if (bActive && !bStreamingEnabled)
	{
		SetStreamingEnabled(true);
	}

	if (bStreamingEnabled && CaptureComponent && CaptureComponent->TextureTarget)
	{
		// Normal cameras are captured manually in MaybeCapture right before the
		// readback copy, so the automatic per-frame capture stays OFF for them (it
		// would render on the main cadence and race the copy, yielding stale or
		// black frames). Only main-renderer cameras, which must render on the main
		// cadence, keep it on while active.
		const bool bWantEveryFrame = bActive && bRenderInMainRenderer;
		if (CaptureComponent->bCaptureEveryFrame != bWantEveryFrame)
		{
			CaptureComponent->bCaptureEveryFrame = bWantEveryFrame;
		}
		if (bActive)
		{
			// Register this viewpoint with the streaming manager every tick
			// (IStreamingManager uses timeout-based decay).
			RegisterWithStreamingManager();

			// Non-seg cameras re-sync HiddenComponents each tick so siblings spawned
			// by a late-starting seg camera don't leak into this capture.
			if (!IsSegMode(CaptureMode))
			{
				RefreshHiddenComponentsFromSegPools();
			}
		}
	}

	HarvestCompletedReadbacks();

	// Drive capture + readback and any delayed publish while active.
	if (bStreamingEnabled && bActive)
	{
		AAMjManager* Mgr = AAMjManager::GetManager();
		MaybeCapture(Mgr);
		PublishDueDelayedFrames(Mgr);
	}
}

void UMjCamera::HarvestCompletedReadbacks()
{
	// Two async stages, both driven from the game thread with no render-thread
	// flush: promote GPU-ready readbacks to a render-thread map/copy, then drain
	// whatever the render thread has finished into the history ring.
	DispatchReadyReadbacks();
	DrainCompletedFrames();
}

void UMjCamera::DispatchReadyReadbacks()
{
	// FIFO: a readback completes in submission order, so stop at the first entry
	// whose GPU copy isn't ready yet rather than reordering later frames ahead.
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
		const EMjCameraMode Mode = CaptureMode;
		TSharedPtr<FRHIGPUTextureReadback> Gpu = Front.Gpu;

		++PendingMapCommands;

		// Map + copy the staging buffer on the render thread. FRHIGPUTextureReadback::Lock
		// asserts IsInRenderingThread; the readback is already Ready(), so this is a
		// CPU memcpy from mapped staging memory with no GPU wait. The command captures
		// the shared results queue (not `this`), so a teardown race that destroys the
		// component before the command runs pushes into a still-live queue rather than
		// a freed object. RowPitch is in PIXELS and >= width (rows are padded), so we
		// copy row-by-row into a tightly-packed array.
		ENQUEUE_RENDER_COMMAND(MjCameraMapReadback)
		([Frame, Gpu, W, H, Mode, Results = ResultsQueue](FRHICommandListImmediate&) {
			bool bCopied = false;
			int32 RowPitchPixels = 0;
			void* Data = Gpu->Lock(RowPitchPixels);
			if (Data && W > 0 && H > 0)
			{
				if (Mode == EMjCameraMode::Depth)
				{
					Frame->Depth.SetNumUninitialized(W * H);
					const float* Src = static_cast<const float*>(Data);
					for (int32 y = 0; y < H; ++y)
						FMemory::Memcpy(&Frame->Depth[y * W], &Src[y * RowPitchPixels], W * sizeof(float));
				}
				else
				{
					Frame->Color.SetNumUninitialized(W * H);
					const FColor* Src = static_cast<const FColor*>(Data);
					for (int32 y = 0; y < H; ++y)
						FMemory::Memcpy(&Frame->Color[y * W], &Src[y * RowPitchPixels], W * sizeof(FColor));
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
	// render flush elsewhere ever pumped the game thread mid-drain, the nested
	// call bails instead of racing the SPSC queue's read cursor.
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
			// delayed publish can select by it; unused when no delay is configured.
			Frame.RevealValue = FrameClock(Frame) + SampleDelaySeconds();

			// No-delay path publishes the instant the frame is harvested. With
			// latency emulation on, harvested frames go to History only and are
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
			// The map failed (RT torn down mid-flight, or a zero-size target). Drop
			// the frame but still recycle the readback below; warn sparingly.
			UE_LOG(LogURLabNet, Verbose,
				TEXT("[MjCamera] '%s' dropped a readback whose staging map failed"), *MjName);
		}

		if (Completed.Gpu.IsValid())
		{
			FreeReadbacks.Add(Completed.Gpu); // recycle (staging texture reused in place)
		}
	}
}

void UMjCamera::WaitAndHarvestReadbacks(double TimeoutSeconds)
{
	// Poll the pipeline instead of flushing: the render thread runs the map/copy
	// commands independently while we yield, and pushes finished frames onto the
	// results queue for us to drain. Bounded by the deadline.
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
	// Force a render + readback for the current applied state. TouchRequested keeps
	// per-camera gating live so the per-frame capture pass (MaybeCapture) keeps
	// producing afterwards; EnqueueReadback no-ops while the RT is still cold, and
	// MaybeCapture retries until a real frame lands. Force the RHI submit so this
	// on-demand copy's fence signals promptly (lower render:sync latency).
	TouchRequested();
	if (CaptureComponent && CaptureComponent->TextureTarget)
	{
		CaptureComponent->CaptureScene();
	}
	uint64 ShowId = 0;
	double ShowTime = 0.0;
	if (AAMjManager* Mgr = AAMjManager::GetManager())
	{
		ShowId = Mgr->GetLastAppliedFrameId();
		ShowTime = Mgr->GetLastAppliedSimTime();
	}
	EnqueueReadback(ShowId, ShowTime, /*bForceSubmit=*/true);
}

void UMjCamera::MaybeCapture(AAMjManager* Mgr)
{
	// Resource savers (see header):
	//  - bCaptureOnStateChange: only render + read back when the applied physics
	//    state advanced (the rendered scene actually changed). No-op in live.
	//  - CaptureMaxFps: an optional wall-clock cap on top.
	const uint64 AppliedId = Mgr ? Mgr->GetLastAppliedFrameId() : 0;
	const double AppliedTime = Mgr ? Mgr->GetLastAppliedSimTime() : 0.0;
	const double NowWall = FPlatformTime::Seconds();
	const bool bFpsOk = (CaptureMaxFps <= 0.0f)
					 || (NowWall - LastCaptureWallSeconds) >= (1.0 / static_cast<double>(CaptureMaxFps));
	// The state-change gate is only "consumed" once a readback actually goes out
	// (see bIssued below), so a freshly-enabled (cold) camera whose RT has no RHI
	// texture yet keeps this true and retries across ticks until a real frame can
	// be captured, instead of latching a single missed capture.
	const bool bStateAdvanced = !bCaptureOnStateChange || (AppliedId != LastCapturedFrameId);

	if (bFpsOk && bStateAdvanced)
	{
		bool bIssued = false;
		if (bRenderInMainRenderer)
		{
			// This camera renders as a nested pass of the main renderer on the main
			// cadence (bCaptureEveryFrame stays on), so the RT is updated during the
			// main render AFTER this readback copy is submitted. Stamp the id whose
			// render the RT currently holds so pixels and frame_id agree.
			bIssued = EnqueueReadback(LastRenderedAppliedId, LastRenderedAppliedTime);
		}
		else
		{
			// Render the scene into the RT immediately before enqueuing the readback
			// copy, so the copy captures a freshly-rendered frame for the current
			// applied state (identical to the render:sync path). Relying on the
			// component's automatic every-frame capture would enqueue the copy before
			// that frame's capture renders, producing a stale or black frame.
			if (CaptureComponent)
			{
				CaptureComponent->CaptureScene();
			}
			bIssued = EnqueueReadback(AppliedId, AppliedTime);
		}

		// Only consume the state-change gate once a readback actually went out; a
		// cold RT that could not enqueue is retried next tick.
		if (bIssued)
		{
			LastCapturedFrameId = AppliedId;
			LastCaptureWallSeconds = NowWall;
		}
	}

	// Track the applied state whose render a main-renderer RT will show next tick.
	LastRenderedAppliedId = AppliedId;
	LastRenderedAppliedTime = AppliedTime;
}

void UMjCamera::PublishDueDelayedFrames(AAMjManager* Mgr)
{
	// Publish the delayed selection (newest frame whose reveal time <= now), each
	// Seq exactly once, so the stream lags by the configured delay. The no-delay
	// path already published inline at harvest.
	if (!IsDelayActive())
		return;

	// On the sim clock this is deterministic by design: while the sim is paused
	// GetLastAppliedSimTime() is frozen, so a frame captured during the pause never
	// reaches its reveal time and delayed publishing stalls until stepping resumes.
	// That is the intended behaviour for a sim-time delay (frozen time = frozen
	// latency); callers wanting reveals to advance in real time while paused set
	// bDelayUseWallClock. Any pile-up of unrevealed frames is bounded by the history
	// ring's MaxHistoryCapacity ceiling.
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
	const FIntPoint Res = GetResolution();
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
	// 1mm near clip on every capture mode: without this, robot-internal geometry
	// can intrude on the frustum and produce black-on-black frames when the camera
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
			// FinalToneCurveHDR (not SCS_BaseColor): BasicShapeMaterial's `Color` param
			// isn't wired to the BaseColor G-buffer, so SCS_BaseColor renders empty.
			CaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalToneCurveHDR;
			CaptureComponent->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;
			break;

		case EMjCameraMode::Real:
		default:
			// LDR final color: already tone-mapped and gamma-encoded to the same
			// sRGB the viewport shows, so the BGRA8 readback matches the editor
			// image (the readback skips TargetGamma, so the HDR-then-encode source
			// delivered a darker frame). It also saves one conversion/copy versus
			// SCS_FinalToneCurveHDR. Seg/Depth keep their own sources below/above.
			CaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
			CaptureComponent->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_LegacySceneCapture;
			break;
	}

	RenderTarget = RT;
	UE_LOG(LogURLabImport, Log,
		TEXT("[MjCamera] '%s' RT created mode=%s (%dx%d)"),
		*MjName,
		*UEnum::GetValueAsString(CaptureMode),
		Res.X, Res.Y);
}

void UMjCamera::RefreshHiddenComponentsFromSegPools()
{
	if (!CaptureComponent)
		return;

	UMjDebugVisualizer* Viz = FindDebugVisualizer(GetWorld());
	if (!Viz)
		return;

	CaptureComponent->HiddenComponents.Reset();

	TArray<UPrimitiveComponent*> Pool;
	Viz->GetSegPoolSiblings(EMjCameraMode::InstanceSegmentation, Pool);
	for (UPrimitiveComponent* P : Pool)
		CaptureComponent->HiddenComponents.Add(P);

	Pool.Reset();
	Viz->GetSegPoolSiblings(EMjCameraMode::SemanticSegmentation, Pool);
	for (UPrimitiveComponent* P : Pool)
		CaptureComponent->HiddenComponents.Add(P);
}

void UMjCamera::RegisterWithStreamingManager()
{
	// Register as an active viewpoint so textures stream for the camera's frustum
	// even when the player pawn is far away.
	const FIntPoint Res = GetResolution();
	const float HFov = CaptureComponent ? CaptureComponent->FOVAngle : fovy;
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
		// Every pixel-sizing path downstream reads GetResolution(), but normalise
		// the backing array too so an editor-edited malformed value is corrected
		// once here rather than defended at each read.
		NormalizeResolution();

		if (!RenderTarget)
		{
			SetupRenderTarget();
		}

		// Seg modes: subscribe to the shared sibling-mesh pool and point
		// ShowOnlyComponents at it. Pool is built lazily on first subscriber.
		if (IsSegMode(CaptureMode))
		{
			if (UMjDebugVisualizer* Viz = FindDebugVisualizer(GetWorld()))
			{
				TArray<UPrimitiveComponent*> Siblings;
				Viz->AcquireSegPool(CaptureMode, this, Siblings);

				CaptureComponent->ShowOnlyComponents.Reset();
				CaptureComponent->ShowOnlyComponents.Reserve(Siblings.Num());
				for (UPrimitiveComponent* Sib : Siblings)
				{
					CaptureComponent->ShowOnlyComponents.Add(Sib);
				}
				UE_LOG(LogURLabImport, Log,
					TEXT("[MjCamera] '%s' seg mode: acquired %d sibling(s) into ShowOnlyComponents."),
					*MjName, Siblings.Num());
			}
			else
			{
				UE_LOG(LogURLabImport, Warning,
					TEXT("[MjCamera] '%s' seg mode requested but no DebugVisualizer found; seg cam will show nothing."),
					*MjName);
			}
		}
		else
		{
			// Non-seg modes (Real, Depth): siblings from other seg cameras are
			// bVisibleInSceneCaptureOnly=true, so they'd otherwise appear in RGB
			// captures. Seed HiddenComponents now; tick-time refresh keeps it in
			// sync with late-starting seg cameras.
			RefreshHiddenComponentsFromSegPools();
		}

		if (bEnableZmqBroadcast && !ZmqWorker)
		{
			const FString Topic = GetCanonicalName();

			// Bind in this instance's camera port block (CamBasePort + index) so
			// multiple editors acting as render servers never fight over one port.
			ZmqEndpoint = ResolveStreamEndpoint();
			ZmqWorker = new FCameraZmqWorker(ZmqEndpoint, Topic, GetResolution());
			WorkerThread = FRunnableThread::Create(ZmqWorker, TEXT("CameraZmqWorkerThread"), 0, TPri_BelowNormal);
			if (!WorkerThread)
			{
				// Thread creation failed: drop the worker so we don't hold a runnable
				// that is never driven (and never publishes).
				UE_LOG(LogURLabNet, Error,
					TEXT("[MjCamera] '%s' failed to create ZMQ worker thread; disabling ZMQ broadcast"),
					*MjName);
				delete ZmqWorker;
				ZmqWorker = nullptr;
			}
		}

		// SHM publisher: opens an mmap'd file under the live URLab session dir.
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
			if (!ShmWriter->Open(FullPath, GetResolution()))
			{
				delete ShmWriter;
				ShmWriter = nullptr;
			}
			else
			{
				UE_LOG(LogURLabNet, Log,
					TEXT("[MjCamera] '%s' SHM broadcast at %s"),
					*MjName, *FullPath);
			}
		}
		if (CaptureComponent)
		{
			CaptureComponent->FOVAngle = HorizontalFOVFromFovy(fovy, GetResolution());

			// CRITICAL: SetVisibility(true) must be called to allow the component to
			// dispatch scene capture updates. bHiddenInGame alone is not sufficient:
			// the capture system checks IsVisible() each frame.
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

		// Force an immediate capture so the UI doesn't wait a full tick.
		// CaptureScene() bypasses the visibility check: it always fires.
		if (CaptureComponent)
		{
			CaptureComponent->CaptureScene();
		}

		UE_LOG(LogURLabImport, Log, TEXT("[MjCamera] '%s' streaming ENABLED."), *MjName);
	}
	else
	{
		bStreamingEnabled = false;

		// Release the seg pool first, while CaptureMode still reflects what we subscribed as.
		if (IsSegMode(CaptureMode))
		{
			if (UMjDebugVisualizer* Viz = FindDebugVisualizer(GetWorld()))
			{
				Viz->ReleaseSegPool(CaptureMode, this);
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
			CaptureComponent->SetVisibility(false); // Stop the capture dispatch loop
			CaptureComponent->TextureTarget = nullptr;
		}

		// Teardown is the one place a render-thread flush is correct: the in-flight
		// EnqueueCopy commands and any already-dispatched map/copy commands hold
		// shared readback objects, so drain the render thread once so nothing runs
		// against freed memory, then release. We do NOT dispatch new map commands
		// here; whatever finished lands in the results queue and is discarded with
		// the rest of the pipeline.
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

		// Drop the RT so the next enable rebuilds it in the current CaptureMode.
		RenderTarget = nullptr;

		if (WorkerThread)
		{
			WorkerThread->Kill(true);
			delete WorkerThread;
			WorkerThread = nullptr;
		}
		if (ZmqWorker)
		{
			delete ZmqWorker;
			ZmqWorker = nullptr;
		}

		if (ShmWriter)
		{
			ShmWriter->Close(/*bDeleteFile=*/true);
			delete ShmWriter;
			ShmWriter = nullptr;
		}

		UE_LOG(LogURLabImport, Log, TEXT("[MjCamera] '%s' streaming DISABLED."), *MjName);
	}
}

// ---------------------------------------------------------------------------
// Async readback
// ---------------------------------------------------------------------------

void UMjCamera::RequestReadback()
{
	if (AAMjManager* Mgr = AAMjManager::GetManager())
	{
		EnqueueReadback(Mgr->GetLastAppliedFrameId(), Mgr->GetLastAppliedSimTime());
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

	FTextureRenderTargetResource* Resource =
		RenderTarget->GameThread_GetRenderTargetResource();
	if (!Resource)
	{
		return false;
	}

	const FIntPoint Size = Resource->GetSizeXY();
	if (Size.X <= 0 || Size.Y <= 0)
	{
		// RT not yet sized (allocation in flight on the render thread).
		return false;
	}

	// A freshly-enabled render target reports its size on the game thread before
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
		Entry.Gpu = FreeReadbacks.Pop(EAllowShrinking::No); // recycle
	}
	else
	{
		Entry.Gpu = MakeShared<FRHIGPUTextureReadback>(TEXT("MjCameraReadback"));
	}
	Entry.Width = Size.X;
	Entry.Height = Size.Y;
	// Unix-epoch capture time for the wire meta (matches state wall_time / py time.time()).
	Entry.CaptureUnixSeconds = (FDateTime::UtcNow() - FDateTime(1970, 1, 1)).GetTotalSeconds();
	// The post-step state the harvested pixels show: for a manual capture this is
	// the current applied id; in every-frame mode the caller passes the id whose
	// render the RT currently holds.
	Entry.FrameId = ShowFrameId;
	Entry.SimTime = ShowSimTime;

	// Schedule the GPU->staging copy without stalling the render thread. SourceTexture
	// is captured as a TRefCountPtr (FTextureRHIRef) so an RT reallocation between
	// enqueue and execution cannot copy against a released texture: the reference
	// keeps the underlying RHI texture alive until the command runs.
	FRHIGPUTextureReadback* Readback = Entry.Gpu.Get();
	ENQUEUE_RENDER_COMMAND(MjCameraEnqueueReadback)
	([Readback, SourceTexture, bForceSubmit](FRHICommandListImmediate& RHICmdList) {
		Readback->EnqueueCopy(RHICmdList, SourceTexture.GetReference());
		if (bForceSubmit)
		{
			// Dispatch the copy to the RHI thread now so the readback fence can
			// signal mid-frame rather than quantizing to the next frame boundary.
			// This is a render-thread-side submit, not a game-thread flush, so it
			// neither stalls the game thread nor re-enters the harvest.
			RHICmdList.ImmediateFlush(EImmediateFlushType::DispatchToRHIThread);
		}
	});

	InFlightReadbacks.Add(MoveTemp(Entry));
	return true;
}

void UMjCamera::PushFrameToHistory(FMjCameraFrame&& Frame)
{
	TSharedPtr<FMjCameraFrame> Shared = MakeShared<FMjCameraFrame>(MoveTemp(Frame));
	PushFrameToHistoryShared(Shared);
}

void UMjCamera::PushFrameToHistoryShared(const TSharedPtr<const FMjCameraFrame>& Frame)
{
	if (!Frame.IsValid())
		return;

	FScopeLock Lock(&HistoryLock);
	History.Add(Frame);

	// Hard frame ceiling regardless of mode: a single frame can be MBs and many
	// cameras share the budget, so bound worst-case retention. Sourced from the
	// same MaxHistoryCapacity that backs HistoryCapacity's ClampMax so the two
	// limits are one source of truth and can never disagree.
	constexpr int32 HardCap = MaxHistoryCapacity;

	if (!IsDelayActive())
	{
		// Keep the last HistoryCapacity frames for by-id retrieval.
		const int32 Cap = FMath::Clamp(HistoryCapacity, 1, HardCap);
		while (History.Num() > Cap)
		{
			History.RemoveAt(0);
		}
		return;
	}

	// Latency emulation: retain enough history to cover the delay window so the
	// reveal-time selection always has the frame it needs. Evict a front frame only
	// once it is older than the window behind the newest (self-sizing and
	// fps-independent), still capped by the memory ceiling.
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
		return History.Last(); // latest available frame
	}
	// Oldest retained frame at/after the requested step (history is oldest-first),
	// i.e. the frame that shows state >= MinFrameId.
	for (const TSharedPtr<const FMjCameraFrame>& F : History)
	{
		if (F->FrameId >= MinFrameId)
		{
			return F;
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
	// currently publishing (the delayed past), not the undelayed newest frame, so
	// RPC and stream agree. bIgnoreDelay opts out for a caller that explicitly
	// wants the freshest rendered (ground-truth) frame.
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

// ---------------------------------------------------------------------------
// Camera latency emulation + capture-rate control
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
	const AAMjManager* Mgr = AAMjManager::GetManager();
	return Mgr ? Mgr->GetLastAppliedSimTime() : 0.0;
}

double UMjCamera::SampleDelaySeconds()
{
	double D = static_cast<double>(DelaySeconds);
	if (DelayJitterSeconds > 0.0f)
	{
		// Only draw from the RNG when jitter is configured, so a fixed delay stays
		// deterministic and doesn't advance the stream.
		const double J = static_cast<double>(DelayJitterSeconds);
		D += DelayRng.FRandRange(-J, J);
	}
	return FMath::Max(0.0, D);
}

void UMjCamera::PublishFrameToWorkers(const FMjCameraFrame& Frame)
{
	FMjCameraFrameMeta Meta;
	Meta.FrameId = Frame.FrameId;
	Meta.SimTime = Frame.SimTime;
	Meta.Width = static_cast<uint32>(Frame.Width);
	Meta.Height = static_cast<uint32>(Frame.Height);
	// Original capture time, so a delayed frame reports the moment it was taken and
	// the client's content-age reflects the injected latency.
	Meta.CaptureUnixTime = Frame.CaptureUnixTime;

	if (CaptureMode == EMjCameraMode::Depth)
	{
		if (Frame.Depth.Num() == 0)
			return;
		if (bEnableZmqBroadcast && ZmqWorker)
			ZmqWorker->PushFrame(Frame.Depth, Meta);
		if (bEnableShmBroadcast && ShmWriter)
			ShmWriter->PushFrame(Frame.Depth, Meta);
	}
	else
	{
		if (Frame.Color.Num() == 0)
			return;
		if (bEnableZmqBroadcast && ZmqWorker)
			ZmqWorker->PushFrame(Frame.Color, Meta);
		if (bEnableShmBroadcast && ShmWriter)
			ShmWriter->PushFrame(Frame.Color, Meta);
	}
}

TSharedPtr<const FMjCameraFrame> UMjCamera::SelectDelayedFrameShared(double NowValue, uint64 AfterSeq) const
{
	FScopeLock Lock(&HistoryLock);
	for (int32 i = History.Num() - 1; i >= 0; --i)
	{
		if (History[i]->RevealValue <= NowValue)
		{
			// Newest eligible frame. Deliver it only if it is newer than the last one
			// published (monotonic, so the stream never repeats or rewinds).
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

void UMjCamera::SetCameraDelay(float InDelaySeconds, float InJitterSeconds, bool bInUseWallClock, int32 InSeed)
{
	DelaySeconds = FMath::Max(0.0f, InDelaySeconds);
	DelayJitterSeconds = FMath::Max(0.0f, InJitterSeconds);
	bDelayUseWallClock = bInUseWallClock;
	const int32 Seed = (InSeed != 0) ? InSeed : static_cast<int32>(GetTypeHash(GetCanonicalName()));
	DelayRng.Initialize(Seed);
	// Re-arm the publish dedup so the new policy re-selects cleanly.
	LastPublishedSeq = 0;
}

void UMjCamera::SetCaptureRate(bool bInOnStateChange, float InMaxFps)
{
	bCaptureOnStateChange = bInOnStateChange;
	CaptureMaxFps = FMath::Max(0.0f, InMaxFps);
}

void UMjCamera::TouchRequested()
{
	LastRequestedSeconds.store(FPlatformTime::Seconds(), std::memory_order_release);
}

bool UMjCamera::IsCaptureActive() const
{
	// Broadcast cameras stream continuously; others capture only while recently
	// requested (within the active TTL).
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
	if (const AAMjManager* Mgr = AAMjManager::GetManager())
	{
		if (Mgr->BridgeServer)
		{
			return URLabBridgeServerConfigUtils::BuildCameraEndpoint(
				Mgr->BridgeServer->GetInstanceConfig(), StreamPortIndex);
		}
	}
	return ZmqEndpoint;
}

FString UMjCamera::GetActualZmqEndpoint() const
{
	if (ZmqWorker)
	{
		return ZmqWorker->GetBoundEndpoint();
	}
	// Not yet streaming (e.g. a dormant camera advertised in the hello handshake):
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

// ---------------------------------------------------------------------------
// ExportTo
// ---------------------------------------------------------------------------

void UMjCamera::ExportTo(mjsCamera* Element, mjsDefault* /*def*/)
{
	if (!Element)
		return;

	// --- CODEGEN_EXPORT_START ---
	if (bOverride_Pos)
	{
		double TmpPos[3];
		MjUtils::UEToMjPosition(Pos, TmpPos);
		Element->pos[0] = TmpPos[0];
		Element->pos[1] = TmpPos[1];
		Element->pos[2] = TmpPos[2];
	}
	if (bOverride_Quat)
	{
		double TmpQuat[4];
		MjUtils::UEToMjRotation(Quat, TmpQuat);
		Element->quat[0] = TmpQuat[0];
		Element->quat[1] = TmpQuat[1];
		Element->quat[2] = TmpQuat[2];
		Element->quat[3] = TmpQuat[3];
	}
	if (bOverride_TrackingMode)
	{
		switch (TrackingMode)
		{
			case EMjCameraTrackingMode::Fixed:
				Element->mode = (mjtCamLight)mjCAMLIGHT_FIXED;
				break;
			case EMjCameraTrackingMode::Track:
				Element->mode = (mjtCamLight)mjCAMLIGHT_TRACK;
				break;
			case EMjCameraTrackingMode::TrackCom:
				Element->mode = (mjtCamLight)mjCAMLIGHT_TRACKCOM;
				break;
			case EMjCameraTrackingMode::TargetBody:
				Element->mode = (mjtCamLight)mjCAMLIGHT_TARGETBODY;
				break;
			case EMjCameraTrackingMode::TargetBodyCom:
				Element->mode = (mjtCamLight)mjCAMLIGHT_TARGETBODYCOM;
				break;
			default:
				break;
		}
	}
	if (bOverride_Projection)
	{
		switch (Projection)
		{
			case EMjCameraProjection::Orthographic:
				Element->proj = (mjtProjection)mjPROJ_ORTHOGRAPHIC;
				break;
			case EMjCameraProjection::Perspective:
				Element->proj = (mjtProjection)mjPROJ_PERSPECTIVE;
				break;
			default:
				break;
		}
	}
	if (bOverride_fovy)
		Element->fovy = fovy;
	if (bOverride_ipd)
		Element->ipd = ipd;
	if (bOverride_resolution)
	{
		for (int32 i = 0; i < FMath::Min(resolution.Num(), 2); ++i)
			Element->resolution[i] = resolution[i];
	}
	if (bOverride_output)
		Element->output = output;
	if (bOverride_target)
		MjSetString(Element->targetbody, target);
	if (bOverride_focal)
	{
		for (int32 i = 0; i < FMath::Min(focal.Num(), 2); ++i)
			Element->focal_length[i] = focal[i];
	}
	if (bOverride_focalpixel)
	{
		for (int32 i = 0; i < FMath::Min(focalpixel.Num(), 2); ++i)
			Element->focal_pixel[i] = focalpixel[i];
	}
	if (bOverride_principal)
	{
		for (int32 i = 0; i < FMath::Min(principal.Num(), 2); ++i)
			Element->principal_length[i] = principal[i];
	}
	if (bOverride_principalpixel)
	{
		for (int32 i = 0; i < FMath::Min(principalpixel.Num(), 2); ++i)
			Element->principal_pixel[i] = principalpixel[i];
	}
	if (bOverride_sensorsize)
	{
		for (int32 i = 0; i < FMath::Min(sensorsize.Num(), 2); ++i)
			Element->sensor_size[i] = sensorsize[i];
	}
	// --- CODEGEN_EXPORT_END ---
}

// ---------------------------------------------------------------------------
// XML Import
// ---------------------------------------------------------------------------

void UMjCamera::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
	if (!Node)
		return;

	// --- CODEGEN_IMPORT_START ---
	MjXmlUtils::ReadAttrString(Node, TEXT("name"), MjName);
	{ // xml_enum: mode -> EMjCameraTrackingMode
		FString S = Node->GetAttribute(TEXT("mode"));
		S = S.ToLower();
		bool bMatched = false;
		if (S == TEXT("fixed"))
		{
			TrackingMode = EMjCameraTrackingMode::Fixed;
			bMatched = true;
		}
		else if (S == TEXT("track"))
		{
			TrackingMode = EMjCameraTrackingMode::Track;
			bMatched = true;
		}
		else if (S == TEXT("trackcom"))
		{
			TrackingMode = EMjCameraTrackingMode::TrackCom;
			bMatched = true;
		}
		else if (S == TEXT("targetbody"))
		{
			TrackingMode = EMjCameraTrackingMode::TargetBody;
			bMatched = true;
		}
		else if (S == TEXT("targetbodycom"))
		{
			TrackingMode = EMjCameraTrackingMode::TargetBodyCom;
			bMatched = true;
		}
		if (bMatched)
			bOverride_TrackingMode = true;
	}
	{ // xml_enum: projection -> EMjCameraProjection
		FString S = Node->GetAttribute(TEXT("projection"));
		S = S.ToLower();
		bool bMatched = false;
		if (S == TEXT("orthographic"))
		{
			Projection = EMjCameraProjection::Orthographic;
			bMatched = true;
		}
		else if (S == TEXT("perspective"))
		{
			Projection = EMjCameraProjection::Perspective;
			bMatched = true;
		}
		if (bMatched)
			bOverride_Projection = true;
	}
	MjXmlUtils::ReadAttrFloat(Node, TEXT("fovy"), fovy, bOverride_fovy);
	MjXmlUtils::ReadAttrFloat(Node, TEXT("ipd"), ipd, bOverride_ipd);
	MjXmlUtils::ReadAttrIntArray(Node, TEXT("resolution"), resolution, bOverride_resolution);
	MjXmlUtils::ReadAttrFloat(Node, TEXT("output"), output, bOverride_output);
	if (MjXmlUtils::ReadAttrString(Node, TEXT("target"), target))
		bOverride_target = true;
	MjXmlUtils::ReadAttrFloatArray(Node, TEXT("focal"), focal, bOverride_focal);
	MjXmlUtils::ReadAttrIntArray(Node, TEXT("focalpixel"), focalpixel, bOverride_focalpixel);
	MjXmlUtils::ReadAttrFloatArray(Node, TEXT("principal"), principal, bOverride_principal);
	MjXmlUtils::ReadAttrIntArray(Node, TEXT("principalpixel"), principalpixel, bOverride_principalpixel);
	MjXmlUtils::ReadAttrFloatArray(Node, TEXT("sensorsize"), sensorsize, bOverride_sensorsize);
	MjUtils::ReadVec3InMeters(Node, TEXT("pos"), Pos, bOverride_Pos);
	{ // canonicalize orientation (quat/euler/axisangle/xyaxes/zaxis)
		double TmpQuat[4] = {1.0, 0.0, 0.0, 0.0};
		if (MjOrientationUtils::OrientationToMjQuat(Node, CompilerSettings, TmpQuat))
		{
			Quat = MjUtils::MjToUERotation(TmpQuat);
			bOverride_Quat = true;
		}
	}
	if (bOverride_Pos)
		SetRelativeLocation(Pos);
	if (bOverride_Quat)
		SetRelativeRotation(Quat);
	// --- CODEGEN_IMPORT_END ---

	// Name fallback (codegen above reads MjName from the "name" attribute; here we
	// provide a sensible default if the user omitted it).
	if (MjName.IsEmpty())
		MjName = TEXT("Camera");

	// fovy: direct attribute wins; otherwise derive from MJCF intrinsics. MuJoCo's
	// compiler computes fovy from focal_pixel / focal_length when present, so we
	// mirror that here so the imported UE FOV matches what mujoco would report.
	FString FovyStr = Node->GetAttribute(TEXT("fovy"));
	if (!FovyStr.IsEmpty())
	{
		fovy = FCString::Atof(*FovyStr);
	}
	else if (bOverride_focalpixel && focalpixel.Num() >= 2 && resolution.Num() >= 2 && focalpixel[1] > 0)
	{
		fovy = 2.0f * FMath::Atan2(static_cast<float>(resolution[1]), 2.0f * static_cast<float>(focalpixel[1])) * (180.0f / PI);
	}
	else if (bOverride_focal && focal.Num() >= 2 && sensorsize.Num() >= 2 && focal[1] > 0.0f)
	{
		fovy = 2.0f * FMath::Atan2(static_cast<float>(sensorsize[1]), 2.0f * static_cast<float>(focal[1])) * (180.0f / PI);
	}
	else if (fovy <= 0.0f)
	{
		fovy = 45.0f;
	}

	// Collapse the imported resolution to a validated {width, height} pair so every
	// pixel-sizing site can index it safely (MJCF `resolution="640"` or an empty
	// array would otherwise be a single element or none).
	NormalizeResolution();

	if (CaptureComponent)
	{
		CaptureComponent->FOVAngle = HorizontalFOVFromFovy(fovy, GetResolution());
	}
}
