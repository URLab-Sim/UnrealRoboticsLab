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
//
// Pure-logic tests for the async readback pipeline's game-thread surface: the
// shared-frame history retention (a fetch is a refcount bump, not a pixel copy),
// the reveal-aware RPC fetch that keeps include_cameras in step with the delayed
// stream, and the bounds-safe resolution accessor. These touch no GPU, so they
// run under the -NullRHI automation harness; the render-thread map/copy stage is
// exercised by the GPU RenderOnDemand test, which cannot run headless.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Render/RenderCameraComponent.h"

namespace
{
URenderCameraComponent* MakeReadbackTestCamera()
{
	// Transient, unregistered component: the history ring and fetch helpers touch
	// only plain members (no render target / world), so this is enough.
	URenderCameraComponent* Camera = NewObject<URenderCameraComponent>(GetTransientPackage());
	FRenderFrameIdentity Identity;
	Identity.SceneRevision = TEXT("camera-history-scene");
	Identity.ClockEpoch = 3;
	Identity.bHasSourceTime = true;
	Camera->SetAppliedFrameIdentity(Identity);
	return Camera;
}

FRenderCameraFrame MakeFrame(
	uint64 FrameId, double SourceTime, double RevealValue, uint64 Seq)
{
	FRenderCameraFrame F;
	F.Identity.SceneRevision = TEXT("camera-history-scene");
	F.Identity.FrameId = FrameId;
	F.Identity.SourceTime = SourceTime;
	F.Identity.ClockEpoch = 3;
	F.Identity.bHasSourceTime = true;
	F.Width = 2;
	F.Height = 1;
	F.RevealValue = RevealValue;
	F.Seq = Seq;
	F.Color.Init(FColor(static_cast<uint8>(FrameId & 0xFF), 0, 0, 255), 2);
	return F;
}
} // namespace

// ============================================================================
// URLab.CameraReadback.SharedFetchAliasesHistory
//   GetFrameShared hands back the retained frame by refcount, so two fetches of
//   the same frame resolve to the identical object (no per-fetch pixel copy).
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCameraSharedFetchAliases,
	"URLab.CameraReadback.SharedFetchAliasesHistory",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCameraSharedFetchAliases::RunTest(const FString& Parameters)
{
	URenderCameraComponent* Cam = MakeReadbackTestCamera();
	if (!TestNotNull(TEXT("camera"), Cam))
		return false;

	Cam->HistoryCapacity = 4;
	Cam->PushFrameToHistory(MakeFrame(10, 0.01, 0.0, 1));
	Cam->PushFrameToHistory(MakeFrame(20, 0.02, 0.0, 2));

	TSharedPtr<const FRenderCameraFrame> A = Cam->GetFrameShared(0);
	TSharedPtr<const FRenderCameraFrame> B = Cam->GetFrameShared(0);
	if (!TestTrue(TEXT("latest fetch valid"), A.IsValid() && B.IsValid()))
		return false;
	TestTrue(TEXT("two fetches share one frame object"), A.Get() == B.Get());
	TestEqual(TEXT("latest is frame 20"), A->Identity.FrameId, (uint64)20);

	// By-id fetch resolves the oldest frame at/after the floor.
	TSharedPtr<const FRenderCameraFrame> ById = Cam->GetFrameShared(15);
	if (TestTrue(TEXT("by-id fetch valid"), ById.IsValid()))
		TestEqual(TEXT(">=15 resolves to 20"), ById->Identity.FrameId, (uint64)20);

	// Nothing newer than 20.
	TestFalse(TEXT("fetch past newest misses"), Cam->GetFrameShared(21).IsValid());

	return true;
}

// ============================================================================
// URLab.CameraReadback.RequestFetchIsDelayAware
//   GetFrameForRequest: source-clock latency selects the newest capture at or
//   before applied source time minus delay. bIgnoreDelay or no delay falls back
//   to the by-id / latest fetch.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCameraRequestFetchDelayAware,
	"URLab.CameraReadback.RequestFetchIsDelayAware",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCameraRequestFetchDelayAware::RunTest(const FString& Parameters)
{
	URenderCameraComponent* Cam = MakeReadbackTestCamera();
	if (!TestNotNull(TEXT("camera"), Cam))
		return false;

	FRenderFrameIdentity Applied = Cam->GetAppliedFrameIdentity();
	Applied.SourceTime = 0.4;
	Cam->SetAppliedFrameIdentity(Applied);

	// At source time 0.4, a 0.5-second delay has no eligible history even after
	// the first 0.45 capture. Advancing the applied source time to 1.0 moves the
	// target to exactly 0.5; all three frames then remain inside the 0.6-second
	// retention window.
	Cam->SetCameraDelay(0.5f, 0.0f, false, 7);

	Cam->PushFrameToHistory(MakeFrame(10, 0.45, 0.45, 1));
	TestFalse(TEXT("cold source-delay read does not substitute newer frame"),
		Cam->GetFrameForRequest(0, /*bIgnoreDelay=*/false).IsValid());

	Applied.SourceTime = 1.0;
	Cam->SetAppliedFrameIdentity(Applied);
	Cam->PushFrameToHistory(MakeFrame(20, 0.50, 0.50, 2));
	Cam->PushFrameToHistory(MakeFrame(30, 1.00, 1.00, 3));

	// Delay active + honour delay returns the newest eligible capture (20), not
	// the undelayed newest capture (30).
	TSharedPtr<const FRenderCameraFrame> Revealed = Cam->GetFrameForRequest(0, /*bIgnoreDelay=*/false);
	if (TestTrue(TEXT("source-delay fetch valid"), Revealed.IsValid()))
		TestEqual(TEXT("returns newest eligible capture (20), not undelayed 30"), Revealed->Identity.FrameId, (uint64)20);

	// bIgnoreDelay bypasses the delay policy and returns the freshest rendered
	// (ground-truth) frame, which is the undelayed newest (30).
	TSharedPtr<const FRenderCameraFrame> Fresh = Cam->GetFrameForRequest(0, /*bIgnoreDelay=*/true);
	if (TestTrue(TEXT("ground-truth fetch valid"), Fresh.IsValid()))
		TestEqual(TEXT("ignore-delay returns undelayed newest (30)"), Fresh->Identity.FrameId, (uint64)30);

	// No delay configured: routing collapses to the plain by-id / latest fetch.
	Cam->SetCameraDelay(0.0f, 0.0f, false, 7);
	TSharedPtr<const FRenderCameraFrame> NoDelay = Cam->GetFrameForRequest(0, /*bIgnoreDelay=*/false);
	if (TestTrue(TEXT("no-delay fetch valid"), NoDelay.IsValid()))
		TestEqual(TEXT("no delay returns latest (30)"), NoDelay->Identity.FrameId, (uint64)30);

	return true;
}

// ============================================================================
// URLab.CameraReadback.DescriptorResolution
//   A complete validated render descriptor is the only way to configure optics.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCameraResolutionAccessor,
	"URLab.CameraReadback.DescriptorResolution",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCameraResolutionAccessor::RunTest(const FString& Parameters)
{
	URenderCameraComponent* Cam = MakeReadbackTestCamera();
	if (!TestNotNull(TEXT("camera"), Cam))
		return false;

	FRenderCameraDesc Desc;
	Desc.CanonicalName = TEXT("resolution-test");
	Desc.Optics.Resolution = FIntPoint(800, 600);
	FString Error;
	TestTrue(TEXT("valid descriptor accepted"), Cam->Configure(Desc, Error));
	TestEqual(TEXT("valid width"), Cam->CaptureResolution().X, 800);
	TestEqual(TEXT("valid height"), Cam->CaptureResolution().Y, 600);

	Desc.Optics.Resolution = FIntPoint(0, -5);
	TestFalse(TEXT("invalid dimensions rejected"), Cam->Configure(Desc, Error));
	TestEqual(TEXT("failed candidate preserves width"), Cam->CaptureResolution().X, 800);
	TestEqual(TEXT("failed candidate preserves height"), Cam->CaptureResolution().Y, 600);

	return true;
}
