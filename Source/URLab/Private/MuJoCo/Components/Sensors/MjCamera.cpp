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
#include "MuJoCo/Components/Sensors/CameraShmWriter.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjDebugVisualizer.h"
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
#include "Utils/URLabLogging.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "zmq.h"

// ---------------------------------------------------------------------------
// FCameraZmqWorker
// ---------------------------------------------------------------------------

std::atomic<bool> FCameraZmqWorker::bPublishersPaused{false};

FCameraZmqWorker::FCameraZmqWorker(const FString& InEndpoint, const FString& InTopic, FIntPoint InRes)
	: RequestedEndpoint(InEndpoint), Topic(InTopic), resolution(InRes), bStopThread(false)
{
	BoundEndpoint = RequestedEndpoint;
}

FCameraZmqWorker::~FCameraZmqWorker()
{
	Stop();
}

bool FCameraZmqWorker::Init()
{
	ZmqContext = zmq_ctx_new();
	ZmqPublisher = zmq_socket(ZmqContext, ZMQ_PUB);

	// A live camera feed only cares about the FRESHEST frame, so keep the
	// PUB send queue shallow. A large HWM lets ZMQ buffer many frames per
	// subscriber and -- because PUB drops the NEWEST frames once the queue is
	// full and delivers the buffered OLD ones FIFO -- a consumer that can't
	// keep up receives a persistently stale frame (seconds of latency). With
	// HWM=1 the PUB holds at most one frame in flight, so a slow consumer
	// always gets a near-latest frame instead of draining a backlog. The
	// worker also drains its own TQueue to the latest frame before sending
	// (see Run), and the SUB drains to latest on receive, so every buffer in
	// the chain stays one frame deep.
	int hwm = 1;
	zmq_setsockopt(ZmqPublisher, ZMQ_SNDHWM, &hwm, sizeof(hwm));

	// Simple port increment logic if port is busy
	FString TryEndpoint = RequestedEndpoint;
	int32 Port = 5558;
	FString BaseAddr = TEXT("tcp://0.0.0.0:");

	// Extract port from requested if it's not the default format
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

	int rc = -1;
	for (int i = 0; i < 10; ++i)
	{
		TryEndpoint = FString::Printf(TEXT("%s%d"), *BaseAddr, Port + i);
		rc = zmq_bind(ZmqPublisher, TCHAR_TO_UTF8(*TryEndpoint));
		if (rc == 0)
		{
			BoundEndpoint = TryEndpoint;
			break;
		}
	}

	if (rc != 0)
	{
		UE_LOG(LogURLabNet, Error, TEXT("CameraZmqWorker Failed to bind ZMQ after 10 retries. Starting at %s"), *RequestedEndpoint);
		return false;
	}

	UE_LOG(LogURLabNet, Log, TEXT("CameraZmqWorker Bound at %s [Topic: %s]"), *BoundEndpoint, *Topic);
	return true;
}

uint32 FCameraZmqWorker::Run()
{
	// Publish one message as [topic][meta + pixels]. The 32-byte metadata
	// header is prepended to the pixel bytes in a single payload frame so the
	// ZMQ and SHM consumers parse an identical (meta, pixels) layout.
	auto SendFrame = [this](const FMjCameraFrameMeta& Meta, const void* Pixels, size_t PixelBytes) {
		if (bPublishersPaused.load(std::memory_order_acquire))
			return;
		TArray<uint8> Payload;
		Payload.SetNumUninitialized(sizeof(FMjCameraFrameMeta) + static_cast<int32>(PixelBytes));
		FMemory::Memcpy(Payload.GetData(), &Meta, sizeof(FMjCameraFrameMeta));
		FMemory::Memcpy(Payload.GetData() + sizeof(FMjCameraFrameMeta), Pixels, PixelBytes);

		const FString TopicSpace = Topic + TEXT(" ");
		const FTCHARToUTF8 TopicUtf8(*TopicSpace);
		zmq_send(ZmqPublisher, TopicUtf8.Get(), TopicUtf8.Length(), ZMQ_SNDMORE);
		zmq_send(ZmqPublisher, Payload.GetData(), Payload.Num(), 0);
	};

	while (!bStopThread)
	{
		const int32 ExpectedPixels = resolution.X * resolution.Y;
		bool bSent = false;

		// Drain to the FRESHEST frame: a live feed only wants the latest, so
		// if the producer outran us, skip the backlog rather than send stale
		// frames in FIFO order (which is what caused seconds of UI latency).
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

void FCameraZmqWorker::Stop()
{
	bStopThread = true;
}

void FCameraZmqWorker::Exit()
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

void FCameraZmqWorker::PushFrame(const TArray<FColor>& FrameData, const FMjCameraFrameMeta& Meta)
{
	// Enqueue unconditionally; the worker drains this queue to the latest
	// frame before sending (and the PUB socket runs HWM=1), so a backlog
	// here is collapsed to the freshest frame rather than streamed FIFO.
	FrameQueue.Enqueue(FQueuedColorFrame{Meta, FrameData});
}

void FCameraZmqWorker::PushFrame(const TArray<float>& FrameData, const FMjCameraFrameMeta& Meta)
{
	FloatFrameQueue.Enqueue(FQueuedFloatFrame{Meta, FrameData});
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
float HorizontalFOVFromFovy(float Fovy, const TArray<int32>& Resolution)
{
	if (Resolution.Num() < 2 || Resolution[0] <= 0 || Resolution[1] <= 0)
	{
		return Fovy;
	}
	const float Aspect = static_cast<float>(Resolution[0]) / static_cast<float>(Resolution[1]);
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
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;

	// Default resolution: codegen emits resolution as TArray<int32>{} (empty); seed [w,h].
	resolution = {640, 480};

	// Create the scene capture sub-component
	CaptureComponent = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("SceneCapture"));
	if (CaptureComponent)
	{
		CaptureComponent->SetupAttachment(this);
		CaptureComponent->SetRelativeScale3D(FVector(0.15f));

		// MuJoCo cameras look down -Z (forward), +Y (up).
		// Unreal cameras look down +X (forward), +Z (up).
		// MjToUERotation negates X/Z quat components (handedness flip),
		// which mirrors the Y axis — so "up" becomes -Y after conversion.
		const FVector MjForward = FVector(0.0f, 0.0f, -1.0f);
		const FVector MjUp = FVector(0.0f, -1.0f, 0.0f);
		const FRotator CorrectionRot = FRotationMatrix::MakeFromXZ(MjForward, MjUp).Rotator();
		CaptureComponent->SetRelativeRotation(CorrectionRot);

		// Start dormant — no capture cost until explicitly enabled
		CaptureComponent->bCaptureEveryFrame = false;
		CaptureComponent->bCaptureOnMovement = false;
		CaptureComponent->bAlwaysPersistRenderingState = true;
		CaptureComponent->MaxViewDistanceOverride = -1.0f;

		// SceneCaptureComponent2D does NOT automatically respect scene Post Process Volumes.
		// We set PostProcessBlendWeight=1 so the component's own PostProcessSettings are used.
		// At BeginPlay, we copy settings from the scene's PPV to match the viewport look.
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
		CaptureComponent->FOVAngle = HorizontalFOVFromFovy(fovy, resolution);
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

	// Copy post-process settings from the scene's Post Process Volume(s)
	// so the capture component matches the viewport look.
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
}

void UMjCamera::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
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

	// Make sure we stop rendering when the actor is torn down
	if (bStreamingEnabled)
	{
		SetStreamingEnabled(false);
	}
	Super::EndPlay(EndPlayReason);
}

void UMjCamera::TickComponent(float DeltaTime, ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	const bool bActive = IsCaptureActive();

	// Per-camera capture gating: lazily start capturing when a camera becomes
	// active (e.g. first include_cameras request on a non-broadcast camera),
	// and pause the GPU scene capture when it goes dormant so a scene with many
	// cameras only pays for the ones actually consumed.
	if (bActive && !bStreamingEnabled)
	{
		SetStreamingEnabled(true);
	}

	if (bStreamingEnabled && CaptureComponent && CaptureComponent->TextureTarget)
	{
		// Gate the per-frame scene capture on activeness. Idle cameras stop
		// rendering entirely (no GPU cost) until requested again.
		if (CaptureComponent->bCaptureEveryFrame != bActive)
		{
			CaptureComponent->bCaptureEveryFrame = bActive;
		}
		if (bActive)
		{
			// Register this viewpoint with the streaming manager every tick
			// (IStreamingManager uses timeout-based decay).
			RegisterWithStreamingManager();

			// Non-seg cameras re-sync HiddenComponents each tick so siblings
			// spawned by a late-starting seg camera don't leak into this capture.
			if (!IsSegMode(CaptureMode))
			{
				RefreshHiddenComponentsFromSegPools();
			}
		}
	}

	// Drain completed async readbacks in FIFO order. Non-stalling: each is a
	// GPU->staging copy that finishes a few frames after EnqueueCopy, so the
	// harvest rate tracks the render rate instead of serialising on a sync
	// ReadSurfaceData stall. Stop at the first not-ready entry to keep order.
	while (InFlightReadbacks.Num() > 0)
	{
		FInFlightReadback& Front = InFlightReadbacks[0];
		if (!Front.Gpu.IsValid() || !Front.Gpu->IsReady())
		{
			break;
		}
		// Diagnostic: readback-pipeline latency = how long this GPU copy took to
		// become ready since it was requested. If this is ~seconds, the readback
		// (not transport/display) is the camera-feed delay. Logged ~1/s.
		{
			const double NowSec = FPlatformTime::Seconds();
			const double ReadbackMs = (NowSec - Front.EnqueueSeconds) * 1000.0;
			ReadbackLatencyAccumMs += ReadbackMs;
			ReadbackLatencyMaxMs = FMath::Max(ReadbackLatencyMaxMs, ReadbackMs);
			++ReadbackLatencySamples;
			if (NowSec - ReadbackLatencyLastLogSec >= 1.0 && ReadbackLatencySamples > 0)
			{
				UE_LOG(LogURLabNet, Log,
					TEXT("[camlat] %s: readback enqueue->ready avg=%.0fms max=%.0fms inflight=%d n=%d"),
					*GetName(), ReadbackLatencyAccumMs / ReadbackLatencySamples,
					ReadbackLatencyMaxMs, InFlightReadbacks.Num(), ReadbackLatencySamples);
				ReadbackLatencyAccumMs = 0.0;
				ReadbackLatencyMaxMs = 0.0;
				ReadbackLatencySamples = 0;
				ReadbackLatencyLastLogSec = NowSec;
			}
		}
		const int32 W = Front.Width;
		const int32 H = Front.Height;

		FMjCameraFrame Frame;
		Frame.FrameId = Front.FrameId;
		Frame.SimTime = Front.SimTime;
		Frame.Width = W;
		Frame.Height = H;

		FMjCameraFrameMeta Meta;
		Meta.FrameId = Front.FrameId;
		Meta.SimTime = Front.SimTime;
		Meta.Width = static_cast<uint32>(W);
		Meta.Height = static_cast<uint32>(H);
		Meta.CaptureUnixTime = Front.CaptureUnixSeconds;

		// Lock the staging buffer. RowPitch is in PIXELS and is >= width (GPU
		// rows are padded), so copy row-by-row into a tightly-packed array.
		int32 RowPitchPixels = 0;
		void* Data = Front.Gpu->Lock(RowPitchPixels);
		if (Data && W > 0 && H > 0)
		{
			if (CaptureMode == EMjCameraMode::Depth)
			{
				// PF_R32_FLOAT: one float per pixel.
				TArray<float> Px;
				Px.SetNumUninitialized(W * H);
				const float* Src = static_cast<const float*>(Data);
				for (int32 y = 0; y < H; ++y)
					FMemory::Memcpy(&Px[y * W], &Src[y * RowPitchPixels], W * sizeof(float));
				if (bEnableZmqBroadcast && ZmqWorker)
					ZmqWorker->PushFrame(Px, Meta);
				if (bEnableShmBroadcast && ShmWriter)
					ShmWriter->PushFrame(Px, Meta);
				Frame.Depth = MoveTemp(Px);
			}
			else
			{
				// BGRA8 (FColor) for real / seg modes.
				TArray<FColor> Px;
				Px.SetNumUninitialized(W * H);
				const FColor* Src = static_cast<const FColor*>(Data);
				for (int32 y = 0; y < H; ++y)
					FMemory::Memcpy(&Px[y * W], &Src[y * RowPitchPixels], W * sizeof(FColor));
				if (bEnableZmqBroadcast && ZmqWorker)
					ZmqWorker->PushFrame(Px, Meta);
				if (bEnableShmBroadcast && ShmWriter)
					ShmWriter->PushFrame(Px, Meta);
				Frame.Color = MoveTemp(Px);
			}
			Front.Gpu->Unlock();
			PushFrameToHistory(MoveTemp(Frame));
		}
		InFlightReadbacks.RemoveAt(0);
	}

	// Keep the pipeline full while active (RequestReadback self-caps at
	// MaxInFlightReadbacks).
	if (bStreamingEnabled && bActive)
	{
		RequestReadback();
	}
}

// ---------------------------------------------------------------------------
// Streaming setup
// ---------------------------------------------------------------------------

void UMjCamera::SetupRenderTarget()
{
	UTextureRenderTarget2D* RT = NewObject<UTextureRenderTarget2D>(this);

	const bool bDepthMode = (CaptureMode == EMjCameraMode::Depth);
	if (bDepthMode)
	{
		RT->RenderTargetFormat = ETextureRenderTargetFormat::RTF_R32f;
		RT->InitCustomFormat(resolution[0], resolution[1], PF_R32_FLOAT, /*bForceLinearGamma=*/true);
	}
	else
	{
		RT->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
		RT->InitCustomFormat(resolution[0], resolution[1], PF_B8G8R8A8, /*bForceLinearGamma=*/true);
	}

	RT->bGPUSharedFlag = true;
	if (GEngine)
	{
		RT->TargetGamma = GEngine->GetDisplayGamma();
	}

	CaptureComponent->TextureTarget = RT;
	CaptureComponent->bAlwaysPersistRenderingState = true;
	CaptureComponent->MaxViewDistanceOverride = -1.0f;
	// 1mm near clip on every capture mode — without this, robot-internal
	// geometry can intrude on the frustum and produce black-on-black frames
	// when the camera is mounted inside a body shell.
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
			// Tints end up lit (not pure flat masks) until a dedicated unlit material lands.
			CaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalToneCurveHDR;
			CaptureComponent->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;
			break;

		case EMjCameraMode::Real:
		default:
			CaptureComponent->CaptureSource = ESceneCaptureSource::SCS_FinalToneCurveHDR;
			CaptureComponent->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_LegacySceneCapture;
			break;
	}

	RenderTarget = RT;
	UE_LOG(LogURLabImport, Log,
		TEXT("[MjCamera] '%s' RT created mode=%s (%dx%d)"),
		*MjName,
		*UEnum::GetValueAsString(CaptureMode),
		resolution[0], resolution[1]);
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
	const float HFov = CaptureComponent ? CaptureComponent->FOVAngle : fovy;
	const float Distance = (HFov > 0.0f)
							 ? resolution[0] / FMath::Tan(FMath::DegreesToRadians(HFov * 0.5f))
							 : 1000.0f;

	IStreamingManager::Get().AddViewInformation(
		GetComponentLocation(),
		resolution[0],
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
					TEXT("[MjCamera] '%s' seg mode requested but no DebugVisualizer found — seg cam will show nothing."),
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
			AMjArticulation* Articulation = Cast<AMjArticulation>(GetOwner());
			FString Prefix = Articulation ? Articulation->GetName() : (GetOwner() ? GetOwner()->GetName() : TEXT("unknown"));
			FString Topic = FString::Printf(TEXT("%s/camera/%s"), *Prefix, *GetCanonicalName());

			const FIntPoint Res(resolution.Num() > 0 ? resolution[0] : 0,
				resolution.Num() > 1 ? resolution[1] : 0);
			ZmqWorker = new FCameraZmqWorker(ZmqEndpoint, Topic, Res);
			WorkerThread = FRunnableThread::Create(ZmqWorker, TEXT("CameraZmqWorkerThread"), 0, TPri_BelowNormal);
		}

		// SHM publisher: opens an mmap'd file under the live URLab session
		// dir. Slot stride is `pixels * 4 bytes` -- works for BGRA8 (Real /
		// seg modes) and float32 single-channel (Depth) alike.
		if (bEnableShmBroadcast && !ShmWriter)
		{
			AMjArticulation* Articulation = Cast<AMjArticulation>(GetOwner());
			const FString Prefix = Articulation ? Articulation->GetName()
												: (GetOwner() ? GetOwner()->GetName() : TEXT("unknown"));
			const FString Dir = UURLabShmPublishTransport::ResolveSessionDir(TEXT("live"));
			IFileManager::Get().MakeDirectory(*Dir, /*Tree=*/true);
			const FString FileName = FString::Printf(
				TEXT("cam_%s_%s.shm"), *Prefix, *GetCanonicalName());
			const FString FullPath = FPaths::Combine(Dir, FileName);

			const FIntPoint ShmRes(resolution.Num() > 0 ? resolution[0] : 0,
				resolution.Num() > 1 ? resolution[1] : 0);
			ShmWriter = new FCameraShmWriter();
			if (!ShmWriter->Open(FullPath, ShmRes))
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
			CaptureComponent->FOVAngle = HorizontalFOVFromFovy(fovy, resolution);

			// CRITICAL: SetVisibility(true) must be called to allow the component
			// to dispatch scene capture updates. bHiddenInGame alone is not sufficient —
			// the capture system checks IsVisible() each frame.
			CaptureComponent->SetVisibility(true);
			CaptureComponent->SetActive(true);
			CaptureComponent->bHiddenInGame = false;
			CaptureComponent->bCaptureEveryFrame = true;
			CaptureComponent->bCaptureOnMovement = false; // We drive capture manually each tick
		}
		bStreamingEnabled = true;
		RegisterWithStreamingManager();

		// force an immediate capture so the UI doesn't wait a full tick.
		// CaptureScene() bypasses the visibility check — it always fires.
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

		// Drain in-flight async readbacks before dropping the RT. Their
		// EnqueueCopy render commands hold raw pointers into these objects, so
		// flush the render thread first, then free them -- otherwise a queued
		// copy could run against freed memory.
		if (InFlightReadbacks.Num() > 0)
		{
			FlushRenderingCommands();
			InFlightReadbacks.Empty();
		}

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
// On-demand readback
// ---------------------------------------------------------------------------

void UMjCamera::RequestReadback()
{
	// Pipeline cap: keep at most MaxInFlightReadbacks async copies outstanding.
	if (!RenderTarget || InFlightReadbacks.Num() >= MaxInFlightReadbacks)
	{
		return;
	}

	FTextureRenderTargetResource* Resource =
		RenderTarget->GameThread_GetRenderTargetResource();
	if (!Resource)
	{
		return;
	}

	const FIntPoint Size = Resource->GetSizeXY();
	if (Size.X <= 0 || Size.Y <= 0)
	{
		// RT not yet sized (allocation in flight on the render thread).
		return;
	}

	FInFlightReadback Entry;
	Entry.Gpu = MakeUnique<FRHIGPUTextureReadback>(TEXT("MjCameraReadback"));
	Entry.Width = Size.X;
	Entry.Height = Size.Y;
	Entry.EnqueueSeconds = FPlatformTime::Seconds();
	// Unix-epoch capture time for the wire meta (matches state wall_time / py time.time()).
	Entry.CaptureUnixSeconds = (FDateTime::UtcNow() - FDateTime(1970, 1, 1)).GetTotalSeconds();
	// Stamp the post-step state this readback will show: the render snapshot id
	// the game thread last applied to the actors. Carried to the harvested
	// frame so a client can ask for "the frame for step N".
	if (AAMjManager* Mgr = AAMjManager::GetManager())
	{
		Entry.FrameId = Mgr->GetLastAppliedFrameId();
		Entry.SimTime = Mgr->GetLastAppliedSimTime();
	}

	// Schedule the GPU->staging copy without stalling the render thread. The
	// copy completes asynchronously; TickComponent polls IsReady() to harvest.
	FRHIGPUTextureReadback* Readback = Entry.Gpu.Get();
	FRHITexture* SourceTexture = Resource->GetRenderTargetTexture();
	ENQUEUE_RENDER_COMMAND(MjCameraEnqueueReadback)(
		[Readback, SourceTexture](FRHICommandListImmediate& RHICmdList) {
			Readback->EnqueueCopy(RHICmdList, SourceTexture);
		});

	InFlightReadbacks.Add(MoveTemp(Entry));
}

void UMjCamera::PushFrameToHistory(FMjCameraFrame&& Frame)
{
	FScopeLock Lock(&HistoryLock);
	History.Add(MoveTemp(Frame));
	const int32 Cap = FMath::Max(1, HistoryCapacity);
	while (History.Num() > Cap)
	{
		History.RemoveAt(0);
	}
}

bool UMjCamera::GetFrame(uint64 MinFrameId, FMjCameraFrame& Out) const
{
	FScopeLock Lock(&HistoryLock);
	if (History.Num() == 0)
	{
		return false;
	}
	if (MinFrameId == 0)
	{
		// Latest available frame.
		Out = History.Last();
		return true;
	}
	// Oldest retained frame at/after the requested step (history is
	// oldest-first), i.e. the frame that shows state >= MinFrameId.
	for (const FMjCameraFrame& F : History)
	{
		if (F.FrameId >= MinFrameId)
		{
			Out = F;
			return true;
		}
	}
	return false;
}

uint64 UMjCamera::GetLatestFrameId() const
{
	FScopeLock Lock(&HistoryLock);
	return History.Num() > 0 ? History.Last().FrameId : 0;
}

void UMjCamera::TouchRequested()
{
	LastRequestedSeconds.store(FPlatformTime::Seconds(), std::memory_order_release);
}

bool UMjCamera::IsCaptureActive() const
{
	// Broadcast cameras stream continuously; others capture only while
	// recently requested (within the active TTL).
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

FString UMjCamera::GetActualZmqEndpoint() const
{
	if (ZmqWorker)
	{
		return ZmqWorker->GetBoundEndpoint();
	}
	return ZmqEndpoint;
}

FString UMjCamera::GetCanonicalName() const
{
	FString Name = GetMjName();
	if (Name.IsEmpty())
		Name = GetName();
	// '/' (MJCF body paths) and '\\' are path separators — collapse them so
	// the same identity is valid as a SHM filename, a ZMQ topic leaf, and a
	// handshake key.
	Name.ReplaceInline(TEXT("/"), TEXT("_"));
	Name.ReplaceInline(TEXT("\\"), TEXT("_"));
	return Name;
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

	// Name fallback (codegen above reads MjName from the "name" attribute;
	// here we provide a sensible default if the user omitted it).
	if (MjName.IsEmpty())
		MjName = TEXT("Camera");

	// fovy — direct attribute wins; otherwise derive from MJCF intrinsics.
	// MuJoCo's compiler computes fovy from focal_pixel / focal_length when
	// present, so we mirror that here so the imported UE FOV matches what
	// mujoco itself would report.
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

	if (CaptureComponent)
	{
		CaptureComponent->FOVAngle = HorizontalFOVFromFovy(fovy, resolution);
	}
}
