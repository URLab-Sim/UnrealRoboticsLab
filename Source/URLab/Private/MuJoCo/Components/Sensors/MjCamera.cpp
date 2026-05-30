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
#include "Transport/ShmPublishTransport.h"  // ResolveSessionDir
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
    
    // Optimize for High Bandwidth (large HWM)
    int hwm = 10;
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
    auto SendBinary = [this](const void* Data, size_t Size)
    {
        if (bPublishersPaused.load(std::memory_order_acquire)) return;
        const FString TopicSpace = Topic + TEXT(" ");
        const FTCHARToUTF8 TopicUtf8(*TopicSpace);
        zmq_send(ZmqPublisher, TopicUtf8.Get(), TopicUtf8.Length(), ZMQ_SNDMORE);
        zmq_send(ZmqPublisher, Data, Size, 0);
    };

    while (!bStopThread)
    {
        const int32 ExpectedPixels = resolution.X * resolution.Y;
        bool bSent = false;

        TArray<FColor> ColorFrame;
        if (FrameQueue.Dequeue(ColorFrame))
        {
            if (ColorFrame.Num() == ExpectedPixels)
            {
                SendBinary(ColorFrame.GetData(), ColorFrame.Num() * sizeof(FColor));
            }
            bSent = true;
        }

        TArray<float> FloatFrame;
        if (FloatFrameQueue.Dequeue(FloatFrame))
        {
            if (FloatFrame.Num() == ExpectedPixels)
            {
                SendBinary(FloatFrame.GetData(), FloatFrame.Num() * sizeof(float));
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

void FCameraZmqWorker::PushFrame(const TArray<FColor>& FrameData)
{
    // ZMQ HWM on the socket handles network-side backpressure if the
    // queue grows. TQueue lacks a cheap Count(), so don't try to drop
    // here.
    FrameQueue.Enqueue(FrameData);
}

void FCameraZmqWorker::PushFrame(const TArray<float>& FrameData)
{
    FloatFrameQueue.Enqueue(FrameData);
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
}

UMjCamera::UMjCamera()
{
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bStartWithTickEnabled = true;

    // Default resolution: codegen emits resolution as TArray<int32>{} (empty); seed [w,h].
    resolution = { 640, 480 };

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
        const FVector  MjForward    = FVector(0.0f,  0.0f, -1.0f);
        const FVector  MjUp         = FVector(0.0f, -1.0f,  0.0f);
        const FRotator CorrectionRot = FRotationMatrix::MakeFromXZ(MjForward, MjUp).Rotator();
        CaptureComponent->SetRelativeRotation(CorrectionRot);

        // Start dormant — no capture cost until explicitly enabled
        CaptureComponent->bCaptureEveryFrame    = false;
        CaptureComponent->bCaptureOnMovement    = false;
        CaptureComponent->bAlwaysPersistRenderingState = true;
        CaptureComponent->MaxViewDistanceOverride      = -1.0f;

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
        CaptureComponent->FOVAngle = fovy;
    }
}

void UMjCamera::BeginPlay()
{
    Super::BeginPlay();

    if (AAMjManager* Manager = AAMjManager::GetManager())
    {
        if (Manager->NetworkManager) Manager->NetworkManager->RegisterCamera(this);
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
        if (Manager->NetworkManager) Manager->NetworkManager->UnregisterCamera(this);
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

    if (bStreamingEnabled && CaptureComponent && CaptureComponent->TextureTarget)
    {
        // Register this viewpoint with the streaming manager every tick
        // (IStreamingManager uses timeout-based decay).
        RegisterWithStreamingManager();

        // Non-seg cameras re-sync their HiddenComponents each tick so siblings
        // spawned by a late-starting seg camera don't leak into this capture.
        if (!IsSegMode(CaptureMode))
        {
            RefreshHiddenComponentsFromSegPools();
        }

        // bCaptureEveryFrame (set by SetStreamingEnabled) drives the per-tick
        // capture. Calling CaptureScene() again here is redundant — UE warns
        // "Scene capture with bCaptureEveryFrame enabled was told to update —
        // major inefficiency", and under sustained render pressure the doubled
        // command submission can leave RHI frame breadcrumbs unbalanced.
    }

    // Check if an in-flight readback has completed. After the fence,
    // copy the buffer to the always-on workers, then move it into
    // Ready* for the bridge consumer. Pending* is left empty so the
    // next RequestReadback can Emplace fresh without disturbing data
    // the bridge is about to MoveTemp.
    if (bReadbackPending && ReadbackFence.IsFenceComplete())
    {
        bReadbackPending = false;
        if (PendingPixels.IsSet())
        {
            if (bEnableZmqBroadcast && ZmqWorker)
                ZmqWorker->PushFrame(PendingPixels.GetValue());
            if (bEnableShmBroadcast && ShmWriter)
                ShmWriter->PushFrame(PendingPixels.GetValue());
            {
                FScopeLock Lock(&FrameLock);
                ReadyPixels.Emplace(MoveTemp(PendingPixels.GetValue()));
            }
            PendingPixels.Reset();
        }
        if (PendingFloatPixels.IsSet())
        {
            if (bEnableZmqBroadcast && ZmqWorker)
                ZmqWorker->PushFrame(PendingFloatPixels.GetValue());
            if (bEnableShmBroadcast && ShmWriter)
                ShmWriter->PushFrame(PendingFloatPixels.GetValue());
            {
                FScopeLock Lock(&FrameLock);
                ReadyFloatPixels.Emplace(MoveTemp(PendingFloatPixels.GetValue()));
            }
            PendingFloatPixels.Reset();
        }
        bReadbackComplete = true;
    }

    // Always refresh PendingPixels while streaming; include_cameras consumes it
    // without enabling ZMQ/SHM broadcast (those flags only gate the workers below).
    if (bStreamingEnabled && !bReadbackPending)
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

    switch (CaptureMode)
    {
    case EMjCameraMode::Depth:
        CaptureComponent->CaptureSource = ESceneCaptureSource::SCS_SceneDepth;
        CaptureComponent->bOverride_CustomNearClippingPlane = true;
        CaptureComponent->CustomNearClippingPlane           = DepthNearCm;
        break;

    case EMjCameraMode::SemanticSegmentation:
    case EMjCameraMode::InstanceSegmentation:
        // FinalToneCurveHDR (not SCS_BaseColor): BasicShapeMaterial's `Color` param
        // isn't wired to the BaseColor G-buffer, so SCS_BaseColor renders empty.
        // Tints end up lit (not pure flat masks) until a dedicated unlit material lands.
        CaptureComponent->CaptureSource       = ESceneCaptureSource::SCS_FinalToneCurveHDR;
        CaptureComponent->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;
        break;

    case EMjCameraMode::Real:
    default:
        CaptureComponent->CaptureSource      = ESceneCaptureSource::SCS_FinalToneCurveHDR;
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
    if (!CaptureComponent) return;

    UMjDebugVisualizer* Viz = FindDebugVisualizer(GetWorld());
    if (!Viz) return;

    CaptureComponent->HiddenComponents.Reset();

    TArray<UPrimitiveComponent*> Pool;
    Viz->GetSegPoolSiblings(EMjCameraMode::InstanceSegmentation, Pool);
    for (UPrimitiveComponent* P : Pool) CaptureComponent->HiddenComponents.Add(P);

    Pool.Reset();
    Viz->GetSegPoolSiblings(EMjCameraMode::SemanticSegmentation, Pool);
    for (UPrimitiveComponent* P : Pool) CaptureComponent->HiddenComponents.Add(P);
}

void UMjCamera::RegisterWithStreamingManager()
{
    // Register as an active viewpoint so textures stream for the camera's frustum
    // even when the player pawn is far away.
    const float HFov     = CaptureComponent ? CaptureComponent->FOVAngle : fovy;
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
            FString Topic = FString::Printf(TEXT("%s/camera/%s"), *Prefix, *GetName());

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
                TEXT("cam_%s_%s.shm"), *Prefix, *GetName());
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
            CaptureComponent->FOVAngle          = fovy;

            // CRITICAL: SetVisibility(true) must be called to allow the component
            // to dispatch scene capture updates. bHiddenInGame alone is not sufficient —
            // the capture system checks IsVisible() each frame.
            CaptureComponent->SetVisibility(true);
            CaptureComponent->SetActive(true);
            CaptureComponent->bHiddenInGame      = false;
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
    if (bReadbackPending || !RenderTarget)
    {
        return;
    }

    FTextureRenderTargetResource* Resource =
        RenderTarget->GameThread_GetRenderTargetResource();
    if (!Resource)
    {
        return;
    }

    const FIntRect Rect(0, 0, Resource->GetSizeXY().X, Resource->GetSizeXY().Y);
    if (Rect.Width() <= 0 || Rect.Height() <= 0)
    {
        // RT not yet sized (allocation in flight on the render thread).
        // Skip this tick; auto-readback will retry next frame.
        return;
    }

    // No lock needed -- Pending* is touched only by the game thread
    // (this function and TickComponent's fence-complete handler), and
    // the bReadbackPending guard at the top of this function blocks
    // concurrent RequestReadback re-entry while a render command is in
    // flight. Ready* is what the bridge consumer touches; it's separate
    // and managed under FrameLock in TickComponent / ConsumePixels.
    TArray<FColor>* PixelsPtr = nullptr;
    TArray<float>*  FloatPtr  = nullptr;
    bReadbackPending = true;
    if (CaptureMode == EMjCameraMode::Depth)
    {
        PendingFloatPixels.Emplace();
        FloatPtr = &PendingFloatPixels.GetValue();
        FloatPtr->SetNumUninitialized(Rect.Width() * Rect.Height());
    }
    else
    {
        PendingPixels.Emplace();
        PixelsPtr = &PendingPixels.GetValue();
        PixelsPtr->SetNumUninitialized(Rect.Width() * Rect.Height());
    }

    if (CaptureMode == EMjCameraMode::Depth)
    {
        // PF_R32_FLOAT render target: ReadSurfaceFData lands a 4-channel
        // FLinearColor per pixel; we keep just the R channel post-fence.
        // Slightly wasteful at readback (4x temp memory) but uses the
        // stock UE async API.
        ENQUEUE_RENDER_COMMAND(MjCameraReadbackDepth)(
            [Resource, FloatPtr, Rect](FRHICommandListImmediate& RHICmdList)
            {
                TArray<FLinearColor> Scratch;
                RHICmdList.ReadSurfaceData(
                    Resource->GetRenderTargetTexture(),
                    Rect,
                    Scratch,
                    FReadSurfaceDataFlags(RCM_MinMax, CubeFace_MAX));
                if (Scratch.Num() == FloatPtr->Num())
                {
                    for (int32 i = 0; i < Scratch.Num(); ++i)
                    {
                        (*FloatPtr)[i] = Scratch[i].R;
                    }
                }
            });
    }
    else
    {
        ENQUEUE_RENDER_COMMAND(MjCameraReadback)(
            [Resource, PixelsPtr, Rect](FRHICommandListImmediate& RHICmdList)
            {
                RHICmdList.ReadSurfaceData(
                    Resource->GetRenderTargetTexture(),
                    Rect,
                    *PixelsPtr,
                    FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX));
            });
    }

    ReadbackFence.BeginFence();
}

bool UMjCamera::IsReadbackReady() const
{
    return bReadbackComplete;
}

TArray<FColor> UMjCamera::ConsumePixels()
{
    FScopeLock Lock(&FrameLock);
    if (ReadyPixels.IsSet())
    {
        TArray<FColor> Result = MoveTemp(ReadyPixels.GetValue());
        ReadyPixels.Reset();
        bReadbackComplete = ReadyFloatPixels.IsSet();
        return Result;
    }
    return TArray<FColor>();
}

TArray<float> UMjCamera::ConsumeFloatPixels()
{
    FScopeLock Lock(&FrameLock);
    if (ReadyFloatPixels.IsSet())
    {
        TArray<float> Result = MoveTemp(ReadyFloatPixels.GetValue());
        ReadyFloatPixels.Reset();
        bReadbackComplete = ReadyPixels.IsSet();
        return Result;
    }
    return TArray<float>();
}

FString UMjCamera::GetActualZmqEndpoint() const
{
    if (ZmqWorker)
    {
        return ZmqWorker->GetBoundEndpoint();
    }
    return ZmqEndpoint;
}

// ---------------------------------------------------------------------------
// ExportTo
// ---------------------------------------------------------------------------

void UMjCamera::ExportTo(mjsCamera* Element, mjsDefault* /*def*/)
{
    if (!Element) return;

    // --- CODEGEN_EXPORT_START ---
    if (bOverride_Pos)
    {
        double TmpPos[3];
        MjUtils::UEToMjPosition(Pos, TmpPos);
        Element->pos[0] = TmpPos[0]; Element->pos[1] = TmpPos[1]; Element->pos[2] = TmpPos[2];
    }
    if (bOverride_Quat)
    {
        double TmpQuat[4];
        MjUtils::UEToMjRotation(Quat, TmpQuat);
        Element->quat[0] = TmpQuat[0]; Element->quat[1] = TmpQuat[1];
        Element->quat[2] = TmpQuat[2]; Element->quat[3] = TmpQuat[3];
    }
    if (bOverride_TrackingMode)
    {
        switch (TrackingMode)
        {
            case EMjCameraTrackingMode::Fixed: Element->mode = (mjtCamLight)mjCAMLIGHT_FIXED; break;
            case EMjCameraTrackingMode::Track: Element->mode = (mjtCamLight)mjCAMLIGHT_TRACK; break;
            case EMjCameraTrackingMode::TrackCom: Element->mode = (mjtCamLight)mjCAMLIGHT_TRACKCOM; break;
            case EMjCameraTrackingMode::TargetBody: Element->mode = (mjtCamLight)mjCAMLIGHT_TARGETBODY; break;
            case EMjCameraTrackingMode::TargetBodyCom: Element->mode = (mjtCamLight)mjCAMLIGHT_TARGETBODYCOM; break;
            default: break;
        }
    }
    if (bOverride_Projection)
    {
        switch (Projection)
        {
            case EMjCameraProjection::Orthographic: Element->proj = (mjtProjection)mjPROJ_ORTHOGRAPHIC; break;
            case EMjCameraProjection::Perspective: Element->proj = (mjtProjection)mjPROJ_PERSPECTIVE; break;
            default: break;
        }
    }
    if (bOverride_fovy) Element->fovy = fovy;
    if (bOverride_ipd) Element->ipd = ipd;
    if (bOverride_resolution) { for (int32 i = 0; i < resolution.Num(); ++i) Element->resolution[i] = resolution[i]; }
    if (bOverride_output) Element->output = output;
    if (bOverride_target && !target.IsEmpty()) mjs_setString(Element->targetbody, TCHAR_TO_UTF8(*target));
    if (bOverride_focal) { for (int32 i = 0; i < focal.Num(); ++i) Element->focal_length[i] = focal[i]; }
    if (bOverride_focalpixel) { for (int32 i = 0; i < focalpixel.Num(); ++i) Element->focal_pixel[i] = focalpixel[i]; }
    if (bOverride_principal) { for (int32 i = 0; i < principal.Num(); ++i) Element->principal_length[i] = principal[i]; }
    if (bOverride_principalpixel) { for (int32 i = 0; i < principalpixel.Num(); ++i) Element->principal_pixel[i] = principalpixel[i]; }
    if (bOverride_sensorsize) { for (int32 i = 0; i < sensorsize.Num(); ++i) Element->sensor_size[i] = sensorsize[i]; }
    // --- CODEGEN_EXPORT_END ---
}

// ---------------------------------------------------------------------------
// XML Import
// ---------------------------------------------------------------------------



void UMjCamera::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
    if (!Node) return;

        // --- CODEGEN_IMPORT_START ---
    { // xml_enum: mode -> EMjCameraTrackingMode
        FString S = Node->GetAttribute(TEXT("mode"));
        S = S.ToLower();
        if      (S == TEXT("fixed")) TrackingMode = EMjCameraTrackingMode::Fixed;
        else if (S == TEXT("track")) TrackingMode = EMjCameraTrackingMode::Track;
        else if (S == TEXT("trackcom")) TrackingMode = EMjCameraTrackingMode::TrackCom;
        else if (S == TEXT("targetbody")) TrackingMode = EMjCameraTrackingMode::TargetBody;
        else if (S == TEXT("targetbodycom")) TrackingMode = EMjCameraTrackingMode::TargetBodyCom;
        if (!S.IsEmpty()) bOverride_TrackingMode = true;
    }
    { // xml_enum: projection -> EMjCameraProjection
        FString S = Node->GetAttribute(TEXT("projection"));
        S = S.ToLower();
        if      (S == TEXT("orthographic")) Projection = EMjCameraProjection::Orthographic;
        else if (S == TEXT("perspective")) Projection = EMjCameraProjection::Perspective;
        if (!S.IsEmpty()) bOverride_Projection = true;
    }
    MjXmlUtils::ReadAttrFloat(Node, TEXT("fovy"), fovy, bOverride_fovy);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("ipd"), ipd, bOverride_ipd);
    MjXmlUtils::ReadAttrIntArray(Node, TEXT("resolution"), resolution, bOverride_resolution);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("output"), output, bOverride_output);
    if (MjXmlUtils::ReadAttrString(Node, TEXT("target"), target)) bOverride_target = true;
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
    if (bOverride_Pos)  SetRelativeLocation(Pos);
    if (bOverride_Quat) SetRelativeRotation(Quat);
    // --- CODEGEN_IMPORT_END ---

    // Name
    if (!MjXmlUtils::ReadAttrString(Node, TEXT("name"), MjName))
        MjName = TEXT("Camera");

    // fovy
    FString FovyStr = Node->GetAttribute(TEXT("fovy"));
    if (!FovyStr.IsEmpty())
    {
        fovy = FCString::Atof(*FovyStr);
        if (CaptureComponent)
        {
            CaptureComponent->FOVAngle = fovy;
        }
    }

}
