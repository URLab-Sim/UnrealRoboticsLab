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

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjTestHelpers.h"
#include "MuJoCo/Elements/MjCamera.h"
#include "MuJoCo/Fast/MjRenderer.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "State/MjCanonicalName.h"
#include "Bridge/RpcDispatcher.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/TextureRenderTarget2D.h"
#include "UObject/ConstructorHelpers.h"
#include "RenderingThread.h"
#include "Misc/App.h"

namespace
{
/**
 * Author a `<camera>` on the test articulation's body with the given mode.
 *
 * The element is created through the spec factory, so the object built is
 * whatever class is registered for `<camera>` -- UMjCamera, which is what every
 * assertion here is about.
 */
UMjCamera* AddCamera(FMjUESession& Sess, const TCHAR* Name, EMjCameraMode Mode)
{
	UMjCamera* Cam = Cast<UMjCamera>(Sess.Add<UMjCameraBase>(Sess.Body, Name));
	if (Cam != nullptr)
	{
		Cam->CaptureMode = Mode;
	}
	return Cam;
}

/** As above, plus streaming enabled, for the tests that inspect the render target. */
UMjCamera* SpawnCameraAndStream(FMjUESession& Sess, EMjCameraMode Mode)
{
	UMjCamera* Cam = AddCamera(Sess, TEXT("TestCamera"), Mode);
	if (Cam != nullptr)
	{
		Cam->SetStreamingEnabled(true);
	}
	return Cam;
}

/**
 * The renderer that owns the segmentation sibling pool, as the seg cameras resolve
 * it. A bare renderer with no built geometry is enough here: its BuildSegPool walks
 * the session's authoring articulation meshes in the editor/test harness, so the
 * pool populates from the same meshes the runtime path segments.
 */
AMjRenderer* SpawnSegRenderer(FMjUESession& Sess)
{
	if (Sess.World == nullptr)
	{
		return nullptr;
	}
	FActorSpawnParameters P;
	AMjRenderer* Renderer = Sess.World->SpawnActor<AMjRenderer>(P);
	if (Renderer != nullptr)
	{
		Renderer->InitializeOverlayMaterial();
	}
	return Renderer;
}
} // namespace

// ============================================================================
// URLab.Camera.RealMode_ConfiguresFinalColorBGRA
//   Default Real mode → RT is RGBA8, CaptureSource is SCS_FinalColorLDR.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCameraRealModeConfig,
	"URLab.Camera.RealMode_ConfiguresFinalColorBGRA",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCameraRealModeConfig::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
		return false;
	}

	UMjCamera* Cam = SpawnCameraAndStream(S, EMjCameraMode::Real);
	if (!TestNotNull(TEXT("camera"), Cam))
	{
		S.Cleanup();
		return false;
	}

	if (!TestNotNull(TEXT("RT"), Cam->RenderTarget.Get()))
	{
		S.Cleanup();
		return false;
	}
	TestEqual(TEXT("RT format"), (int32)Cam->RenderTarget->RenderTargetFormat, (int32)ETextureRenderTargetFormat::RTF_RGBA8);

	if (TestNotNull(TEXT("capture component"), Cam->CaptureComponent.Get()))
	{
		TestEqual(TEXT("capture source"),
			(int32)Cam->CaptureComponent->CaptureSource,
			(int32)ESceneCaptureSource::SCS_FinalColorLDR);
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Camera.DepthMode_ConfiguresSceneDepthFloat
//   Depth mode → RT is R32f, CaptureSource is SCS_SceneDepth, near clip overridden.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCameraDepthModeConfig,
	"URLab.Camera.DepthMode_ConfiguresSceneDepthFloat",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCameraDepthModeConfig::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
		return false;
	}

	UMjCamera* Cam = SpawnCameraAndStream(S, EMjCameraMode::Depth);
	if (!TestNotNull(TEXT("camera"), Cam))
	{
		S.Cleanup();
		return false;
	}

	if (!TestNotNull(TEXT("RT"), Cam->RenderTarget.Get()))
	{
		S.Cleanup();
		return false;
	}
	TestEqual(TEXT("RT format"), (int32)Cam->RenderTarget->RenderTargetFormat, (int32)ETextureRenderTargetFormat::RTF_R32f);

	if (TestNotNull(TEXT("capture component"), Cam->CaptureComponent.Get()))
	{
		TestEqual(TEXT("capture source"),
			(int32)Cam->CaptureComponent->CaptureSource,
			(int32)ESceneCaptureSource::SCS_SceneDepth);
		TestTrue(TEXT("near clip overridden"), Cam->CaptureComponent->bOverride_CustomNearClippingPlane);
		// Frustum near clip is fixed at 0.1 cm for every capture mode to
		// stop internal robot geometry from intruding on the view. The
		// DepthNearCm property now controls only the post-process depth
		// normalisation (see MjCameraFeedEntry.cpp).
		TestEqual(TEXT("near clip value"),
			Cam->CaptureComponent->CustomNearClippingPlane, 5.0f);
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Camera.ModeCycle_RebuildsRenderTarget
//   Toggling streaming off then on after a mode change rebuilds the RT with
//   the new format. Real → Depth should switch RTF_RGBA8 → RTF_R32f.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCameraModeCycleRebuildsRT,
	"URLab.Camera.ModeCycle_RebuildsRenderTarget",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCameraModeCycleRebuildsRT::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
		return false;
	}

	UMjCamera* Cam = SpawnCameraAndStream(S, EMjCameraMode::Real);
	if (!TestNotNull(TEXT("camera"), Cam))
	{
		S.Cleanup();
		return false;
	}
	TestEqual(TEXT("initial format"), (int32)Cam->RenderTarget->RenderTargetFormat, (int32)ETextureRenderTargetFormat::RTF_RGBA8);

	Cam->SetStreamingEnabled(false);
	TestNull(TEXT("RT cleared after disable"), Cam->RenderTarget);

	Cam->CaptureMode = EMjCameraMode::Depth;
	Cam->SetStreamingEnabled(true);
	if (!TestNotNull(TEXT("RT rebuilt"), Cam->RenderTarget.Get()))
	{
		S.Cleanup();
		return false;
	}
	TestEqual(TEXT("new format"), (int32)Cam->RenderTarget->RenderTargetFormat, (int32)ETextureRenderTargetFormat::RTF_R32f);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Camera.SegPool_AcquireReleaseRefcount
//   Pool lifecycle: two cameras acquire → shared pool is built once, survives
//   one release, is destroyed on the second release.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCameraSegPoolRefcount,
	"URLab.Camera.SegPool_AcquireReleaseRefcount",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCameraSegPoolRefcount::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
		return false;
	}

	// The seg pool lives on the renderer that owns the geoms. Its OverlayParentMaterial
	// is initialized in BeginPlay; test worlds don't dispatch BeginPlay, so the helper
	// triggers initialization manually.
	AMjRenderer* Renderer = SpawnSegRenderer(S);
	if (!TestNotNull(TEXT("renderer"), Renderer))
	{
		S.Cleanup();
		return false;
	}

	// FMjUESession's base UMjGeom has no visualizer mesh. Attach a static-mesh child
	// so BuildSegPool's child walk has something to mirror into a sibling.
	UStaticMesh* CubeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (!TestNotNull(TEXT("engine cube mesh"), CubeMesh))
	{
		S.Cleanup();
		return false;
	}

	UStaticMeshComponent* ChildMesh = NewObject<UStaticMeshComponent>(S.Robot, TEXT("GeomChildMesh"));
	ChildMesh->SetStaticMesh(CubeMesh);
	ChildMesh->RegisterComponent();
	ChildMesh->AttachToComponent(S.Geom, FAttachmentTransformRules::KeepRelativeTransform);

	UMjCamera* CamA = AddCamera(S, TEXT("CamA"), EMjCameraMode::InstanceSegmentation);
	UMjCamera* CamB = AddCamera(S, TEXT("CamB"), EMjCameraMode::InstanceSegmentation);
	if (!TestNotNull(TEXT("CamA"), CamA) || !TestNotNull(TEXT("CamB"), CamB))
	{
		S.Cleanup();
		return false;
	}

	TArray<UPrimitiveComponent*> PoolA;
	Renderer->AcquireSegPool(EMjCameraMode::InstanceSegmentation, CamA, PoolA);
	TestTrue(TEXT("pool has entries after first acquire"), PoolA.Num() > 0);

	TArray<UPrimitiveComponent*> PoolB;
	Renderer->AcquireSegPool(EMjCameraMode::InstanceSegmentation, CamB, PoolB);
	TestEqual(TEXT("second acquire sees same pool size"), PoolB.Num(), PoolA.Num());
	if (PoolA.Num() > 0 && PoolB.Num() > 0)
	{
		TestEqual(TEXT("pool entries are shared"), PoolA[0], PoolB[0]);
	}

	// First release — pool should still exist because CamB still subscribed.
	Renderer->ReleaseSegPool(EMjCameraMode::InstanceSegmentation, CamA);
	TArray<UPrimitiveComponent*> Snapshot;
	Renderer->GetSegPoolSiblings(EMjCameraMode::InstanceSegmentation, Snapshot);
	TestTrue(TEXT("pool still alive after one release"), Snapshot.Num() > 0);

	// Final release — pool should be destroyed.
	Renderer->ReleaseSegPool(EMjCameraMode::InstanceSegmentation, CamB);
	Renderer->GetSegPoolSiblings(EMjCameraMode::InstanceSegmentation, Snapshot);
	TestEqual(TEXT("pool empty after last release"), Snapshot.Num(), 0);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Camera.SegMode_WiresShowOnlyListFromPool
//   A seg camera at SetStreamingEnabled(true) configures its CaptureComponent:
//   PRM_UseShowOnlyList, CaptureSource = SCS_BaseColor, ShowOnlyComponents
//   populated from the pool.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCameraSegWiresShowOnly,
	"URLab.Camera.SegMode_WiresShowOnlyListFromPool",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCameraSegWiresShowOnly::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
		return false;
	}

	if (!TestNotNull(TEXT("renderer"), SpawnSegRenderer(S)))
	{
		S.Cleanup();
		return false;
	}

	UStaticMesh* CubeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	UStaticMeshComponent* ChildMesh = NewObject<UStaticMeshComponent>(S.Robot, TEXT("GeomChildMesh"));
	ChildMesh->SetStaticMesh(CubeMesh);
	ChildMesh->RegisterComponent();
	ChildMesh->AttachToComponent(S.Geom, FAttachmentTransformRules::KeepRelativeTransform);

	UMjCamera* Cam = AddCamera(S, TEXT("SegCam"), EMjCameraMode::InstanceSegmentation);
	if (!TestNotNull(TEXT("camera"), Cam))
	{
		S.Cleanup();
		return false;
	}
	Cam->SetStreamingEnabled(true);

	if (!TestNotNull(TEXT("capture component"), Cam->CaptureComponent.Get()))
	{
		S.Cleanup();
		return false;
	}
	TestEqual(TEXT("capture source"),
		(int32)Cam->CaptureComponent->CaptureSource,
		(int32)ESceneCaptureSource::SCS_FinalToneCurveHDR);
	TestEqual(TEXT("primitive render mode"),
		(int32)Cam->CaptureComponent->PrimitiveRenderMode,
		(int32)ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList);
	TestTrue(TEXT("ShowOnlyComponents populated"),
		Cam->CaptureComponent->ShowOnlyComponents.Num() > 0);

	Cam->SetStreamingEnabled(false);
	TestEqual(TEXT("ShowOnlyComponents cleared on disable"),
		Cam->CaptureComponent->ShowOnlyComponents.Num(), 0);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Camera.NonSegMode_HidesSiblingPool
//   A Real-mode camera and an InstanceSeg camera coexist: the Real camera's
//   HiddenComponents list includes the seg pool siblings.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCameraNonSegHidesSiblings,
	"URLab.Camera.NonSegMode_HidesSiblingPool",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCameraNonSegHidesSiblings::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
		return false;
	}

	if (!TestNotNull(TEXT("renderer"), SpawnSegRenderer(S)))
	{
		S.Cleanup();
		return false;
	}

	UStaticMesh* CubeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	UStaticMeshComponent* ChildMesh = NewObject<UStaticMeshComponent>(S.Robot, TEXT("GeomChildMesh"));
	ChildMesh->SetStaticMesh(CubeMesh);
	ChildMesh->RegisterComponent();
	ChildMesh->AttachToComponent(S.Geom, FAttachmentTransformRules::KeepRelativeTransform);

	// Start the seg camera first so the pool exists when the Real camera subscribes.
	UMjCamera* Seg = AddCamera(S, TEXT("SegCam"), EMjCameraMode::InstanceSegmentation);
	UMjCamera* Real = AddCamera(S, TEXT("RealCam"), EMjCameraMode::Real);
	if (!TestNotNull(TEXT("seg camera"), Seg) || !TestNotNull(TEXT("real camera"), Real))
	{
		S.Cleanup();
		return false;
	}
	Seg->SetStreamingEnabled(true);
	Real->SetStreamingEnabled(true);

	if (!TestNotNull(TEXT("real capture component"), Real->CaptureComponent.Get()))
	{
		S.Cleanup();
		return false;
	}
	const int32 ExpectedHidden = Seg->CaptureComponent->ShowOnlyComponents.Num();
	TestEqual(TEXT("real cam hides seg siblings"),
		Real->CaptureComponent->HiddenComponents.Num(), ExpectedHidden);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Camera.RenderOnDemandSync
//   Exercises the render-on-demand path: apply a physics frame, then
//   IssueSyncCapture -> FlushRenderingCommands -> HarvestCompletedReadbacks, and
//   check a frame with pixels lands in the history ring. The GPU part runs only
//   with a real RHI; run with -RenderOffScreen to validate actual rendering.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCameraRenderOnDemandSync,
	"URLab.Camera.RenderOnDemandSync",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCameraRenderOnDemandSync::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
		return false;
	}

	UMjCamera* Cam = SpawnCameraAndStream(S, EMjCameraMode::Real);
	if (!TestNotNull(TEXT("camera"), Cam))
	{
		S.Cleanup();
		return false;
	}

	// Produce a physics frame and apply it, so the sync readback has a real
	// applied-state id to tag its frame with.
	S.Manager->PhysicsEngine->StepSync(1);
	S.Manager->ApplyLatestRenderState();
	const uint64 AppliedId = S.Manager->GetLastAppliedFrameId();
	TestTrue(TEXT("applied frame id advanced"), AppliedId > 0);

	if (!FApp::CanEverRender())
	{
		AddInfo(TEXT("No RHI: GPU capture not exercised; run with -RenderOffScreen to validate rendering"));
		S.Cleanup();
		return true;
	}

	// Warm the freshly-enabled render target (render-thread allocation) before
	// the first capture, matching RenderCamerasSync's cold-camera warmup.
	FlushRenderingCommands();

	// Synchronous render-on-demand: capture, submit, wait on the GPU fence, harvest.
	Cam->IssueSyncCapture();
	const int32 InFlight = Cam->NumInFlightReadbacks();
	FlushRenderingCommands();
	Cam->WaitAndHarvestReadbacks(1.0);
	AddInfo(FString::Printf(TEXT("RenderOnDemandSync: readbacks_enqueued=%d latest_frame=%llu"),
		InFlight, (unsigned long long)Cam->GetLatestFrameId()));

	FMjCameraFrame Frame;
	if (TestTrue(TEXT("sync capture produced a frame in history"), Cam->GetFrame(0, Frame)))
	{
		AddInfo(FString::Printf(TEXT("RenderOnDemandSync: frame_id=%llu applied=%llu size=%dx%d color_px=%d"),
			(unsigned long long)Frame.FrameId, (unsigned long long)AppliedId,
			Frame.Width, Frame.Height, Frame.Color.Num()));
		TestTrue(TEXT("frame carries rendered pixels"), Frame.Color.Num() > 0);
		TestEqual(TEXT("pixel count matches frame dimensions"),
			Frame.Color.Num(), Frame.Width * Frame.Height);
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Camera.CanonicalName_ArtSlashPart
//   A camera's canonical identity is the single "<art>/<part>" name (no
//   "camera/" infix, no raw-name aliases), and BuildCameraNameMap resolves it
//   by that name alone.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCameraCanonicalName,
	"URLab.Camera.CanonicalName_ArtSlashPart",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCameraCanonicalName::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
		return false;
	}

	UMjCamera* Cam = AddCamera(S, TEXT("WristCam"), EMjCameraMode::Real);
	if (!TestNotNull(TEXT("camera"), Cam))
	{
		S.Cleanup();
		return false;
	}

	const FString ArtSeg = FMjCanonicalName::ArtSegment(S.Robot).ToString();
	const FString Canon = Cam->GetCanonicalName();

	TestEqual(TEXT("canonical is <art>/<part>"), Canon, ArtSeg + TEXT("/WristCam"));
	TestFalse(TEXT("no camera/ infix"), Canon.Contains(TEXT("/camera/")));

	TMap<FString, UMjCamera*> ByName;
	FURLabRpcDispatcher::BuildCameraNameMap(S.Manager, ByName);
	TestEqual(TEXT("canonical name resolves to the camera"), ByName.FindRef(Canon), Cam);
	TestNull(TEXT("bare component-name alias dropped"), ByName.FindRef(TEXT("WristCam")));
	TestNull(TEXT("camera/ infix alias dropped"),
		ByName.FindRef(ArtSeg + TEXT("/camera/WristCam")));

	S.Cleanup();
	return true;
}
