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
#include "MuJoCo/Elements/MjCamera.h"

namespace
{
UMjCamera* MakeReadbackTestCamera()
{
	// Transient, unregistered component: the history ring and fetch helpers touch
	// only plain members (no render target / world), so this is enough.
	return NewObject<UMjCamera>(GetTransientPackage(), UMjCamera::StaticClass());
}

// Reveal / Seq drive the delay policy; a tiny reveal value keeps frames eligible
// under both the wall-clock and sim-clock "now" without depending on a manager.
FMjCameraFrame MakeFrame(uint64 FrameId, double RevealValue, uint64 Seq)
{
	FMjCameraFrame F;
	F.FrameId = FrameId;
	F.SimTime = 0.001 * FrameId;
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
	UMjCamera* Cam = MakeReadbackTestCamera();
	if (!TestNotNull(TEXT("camera"), Cam))
		return false;

	Cam->HistoryCapacity = 4;
	Cam->PushFrameToHistory(MakeFrame(10, 0.0, 1));
	Cam->PushFrameToHistory(MakeFrame(20, 0.0, 2));

	TSharedPtr<const FMjCameraFrame> A = Cam->GetFrameShared(0);
	TSharedPtr<const FMjCameraFrame> B = Cam->GetFrameShared(0);
	if (!TestTrue(TEXT("latest fetch valid"), A.IsValid() && B.IsValid()))
		return false;
	TestTrue(TEXT("two fetches share one frame object"), A.Get() == B.Get());
	TestEqual(TEXT("latest is frame 20"), A->FrameId, (uint64)20);

	// By-id fetch resolves the oldest frame at/after the floor.
	TSharedPtr<const FMjCameraFrame> ById = Cam->GetFrameShared(15);
	if (TestTrue(TEXT("by-id fetch valid"), ById.IsValid()))
		TestEqual(TEXT(">=15 resolves to 20"), ById->FrameId, (uint64)20);

	// Nothing newer than 20.
	TestFalse(TEXT("fetch past newest misses"), Cam->GetFrameShared(21).IsValid());

	return true;
}

// ============================================================================
// URLab.CameraReadback.RequestFetchIsDelayAware
//   GetFrameForRequest: with latency emulation active it returns the frame the
//   delayed stream currently reveals (so RPC and stream agree); bIgnoreDelay or
//   no delay falls back to the by-id / latest fetch.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCameraRequestFetchDelayAware,
	"URLab.CameraReadback.RequestFetchIsDelayAware",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCameraRequestFetchDelayAware::RunTest(const FString& Parameters)
{
	UMjCamera* Cam = MakeReadbackTestCamera();
	if (!TestNotNull(TEXT("camera"), Cam))
		return false;

	// Wall-clock delay so "now" is UtcNow epoch seconds (~1.7e9), well above the
	// small reveal values below, making the reveal gate deterministic in-test.
	Cam->DelaySeconds = 0.5f;
	Cam->bDelayUseWallClock = true;

	// Frame 30 is captured but NOT yet revealed (reveal far in the future); the
	// newest revealed frame is 20.
	Cam->PushFrameToHistory(MakeFrame(10, 1.0, 1));
	Cam->PushFrameToHistory(MakeFrame(20, 2.0, 2));
	Cam->PushFrameToHistory(MakeFrame(30, 1.0e18, 3));

	// Delay active + honour delay: the RPC fetch tracks the stream and returns the
	// newest revealed frame (20), NOT the undelayed newest (30).
	TSharedPtr<const FMjCameraFrame> Revealed = Cam->GetFrameForRequest(0, /*bIgnoreDelay=*/false);
	if (TestTrue(TEXT("reveal-aware fetch valid"), Revealed.IsValid()))
		TestEqual(TEXT("returns newest revealed (20), not undelayed 30"), Revealed->FrameId, (uint64)20);

	// bIgnoreDelay bypasses the delay policy and returns the freshest rendered
	// (ground-truth) frame, which is the undelayed newest (30).
	TSharedPtr<const FMjCameraFrame> Fresh = Cam->GetFrameForRequest(0, /*bIgnoreDelay=*/true);
	if (TestTrue(TEXT("ground-truth fetch valid"), Fresh.IsValid()))
		TestEqual(TEXT("ignore-delay returns undelayed newest (30)"), Fresh->FrameId, (uint64)30);

	// No delay configured: routing collapses to the plain by-id / latest fetch.
	Cam->DelaySeconds = 0.0f;
	TSharedPtr<const FMjCameraFrame> NoDelay = Cam->GetFrameForRequest(0, /*bIgnoreDelay=*/false);
	if (TestTrue(TEXT("no-delay fetch valid"), NoDelay.IsValid()))
		TestEqual(TEXT("no delay returns latest (30)"), NoDelay->FrameId, (uint64)30);

	return true;
}

// ============================================================================
// URLab.CameraReadback.ResolutionAccessorIsBoundsSafe
//   CaptureResolution() substitutes the 640x480 default for any missing or
//   non-positive element, so downstream pixel sizing can never index a malformed
//   resolution array out of bounds. The generated GetResolution() beside it is
//   the raw schema attribute and does no such validation.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCameraResolutionAccessor,
	"URLab.CameraReadback.ResolutionAccessorIsBoundsSafe",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCameraResolutionAccessor::RunTest(const FString& Parameters)
{
	UMjCamera* Cam = MakeReadbackTestCamera();
	if (!TestNotNull(TEXT("camera"), Cam))
		return false;

	// Well-formed pair passes through.
	Cam->SetResolution({800, 600});
	TestEqual(TEXT("valid width"), Cam->CaptureResolution().X, 800);
	TestEqual(TEXT("valid height"), Cam->CaptureResolution().Y, 600);

	// Single element (MJCF resolution="640"): height defaults.
	Cam->SetResolution({640});
	TestEqual(TEXT("single-element width kept"), Cam->CaptureResolution().X, 640);
	TestEqual(TEXT("single-element height defaults"), Cam->CaptureResolution().Y, 480);

	// Unset attribute: both default.
	Cam->ClearResolution();
	TestEqual(TEXT("empty width defaults"), Cam->CaptureResolution().X, 640);
	TestEqual(TEXT("empty height defaults"), Cam->CaptureResolution().Y, 480);

	// Non-positive entries: both default.
	Cam->SetResolution({0, -5});
	TestEqual(TEXT("non-positive width defaults"), Cam->CaptureResolution().X, 640);
	TestEqual(TEXT("non-positive height defaults"), Cam->CaptureResolution().Y, 480);

	return true;
}
