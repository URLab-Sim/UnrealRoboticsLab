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
// Pure-logic tests for the decoupled camera retrieval added with the streaming
// rework: the frame-history ring (by-id / latest fetch + eviction) and the
// per-camera capture-gating state machine. These exercise no GPU, so they run
// fine under the -NullRHI automation harness. The actual GPU pixel readback is
// validated separately (it cannot run headless).

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Render/RenderCameraComponent.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "Bridge/RpcDispatcher.h"
#include "Tests/MjTestHelpers.h"
#include "Dom/JsonObject.h"

namespace
{
URenderCameraComponent* MakeBareCamera()
{
	// Transient, unregistered component: the history ring and gating helpers
	// touch only plain members (no render target / world), so this is enough.
	URenderCameraComponent* Camera = NewObject<URenderCameraComponent>(GetTransientPackage());
	FRenderFrameIdentity Identity;
	Identity.SceneRevision = TEXT("camera-history-scene");
	Identity.ClockEpoch = 3;
	Identity.bHasSourceTime = true;
	Camera->SetAppliedFrameIdentity(Identity);
	return Camera;
}

FRenderCameraFrame MakeColorFrame(uint64 FrameId, double SimTime)
{
	FRenderCameraFrame F;
	F.Identity.SceneRevision = TEXT("camera-history-scene");
	F.Identity.FrameId = FrameId;
	F.Identity.SourceTime = SimTime;
	F.Identity.ClockEpoch = 3;
	F.Identity.bHasSourceTime = true;
	F.Width = 2;
	F.Height = 1;
	F.Color.Init(FColor(static_cast<uint8>(FrameId & 0xFF), 0, 0, 255), 2);
	return F;
}

FRenderCameraFrame MakeDelayFrame(uint64 FrameId, double SimTime, double RevealValue, uint64 Seq)
{
	FRenderCameraFrame F = MakeColorFrame(FrameId, SimTime);
	F.RevealValue = RevealValue;
	F.Seq = Seq;
	return F;
}
} // namespace

// ============================================================================
// URLab.CameraHistory.RingEvictsAndFetches
//   Ring keeps the last HistoryCapacity frames; GetFrame returns latest (id 0)
//   or the oldest retained frame >= MinFrameId; misses return false.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRenderCameraHistoryRing,
	"URLab.CameraHistory.RingEvictsAndFetches",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FRenderCameraHistoryRing::RunTest(const FString& Parameters)
{
	URenderCameraComponent* Cam = MakeBareCamera();
	if (!TestNotNull(TEXT("camera"), Cam))
		return false;

	Cam->HistoryCapacity = 3;

	FRenderCameraFrame Out;
	TestFalse(TEXT("empty history -> GetFrame(0) false"), Cam->GetFrame(0, Out));
	TestEqual(TEXT("empty history -> latest id 0"), Cam->GetLatestFrameId(), (uint64)0);

	for (uint64 Id = 1; Id <= 5; ++Id)
	{
		Cam->PushFrameToHistory(MakeColorFrame(Id, 0.01 * Id));
	}

	// Capacity 3 -> only frames 3,4,5 retained.
	TestEqual(TEXT("latest id is 5"), Cam->GetLatestFrameId(), (uint64)5);

	TestTrue(TEXT("GetFrame(0) latest"), Cam->GetFrame(0, Out));
	TestEqual(TEXT("latest frame id"), Out.Identity.FrameId, (uint64)5);

	TestTrue(TEXT("GetFrame(4)"), Cam->GetFrame(4, Out));
	TestEqual(TEXT("exact >=4 is 4"), Out.Identity.FrameId, (uint64)4);

	// Frames 1,2 were evicted; oldest retained >= 2 is 3.
	TestTrue(TEXT("GetFrame(2) -> oldest retained >=2"), Cam->GetFrame(2, Out));
	TestEqual(TEXT(">=2 resolves to 3"), Out.Identity.FrameId, (uint64)3);

	// Nothing newer than 5.
	TestFalse(TEXT("GetFrame(6) misses"), Cam->GetFrame(6, Out));

	// Payload + metadata survive the round trip.
	TestTrue(TEXT("GetFrame(5)"), Cam->GetFrame(5, Out));
	TestEqual(TEXT("width preserved"), Out.Width, 2);
	TestEqual(TEXT("pixel count preserved"), Out.Color.Num(), 2);
	TestTrue(TEXT("sim_time preserved"), FMath::IsNearlyEqual(Out.Identity.SourceTime, 0.05, 1e-9));

	return true;
}

// ============================================================================
// URLab.CameraHistory.DelayedFrameSelection
//   Latency emulation: SelectDelayedFrame returns the newest frame whose
//   RevealValue <= now, but only when its Seq advances past the last published
//   (monotonic, no repeats), and nothing when no frame is yet eligible.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCameraDelayedSelection,
	"URLab.CameraHistory.DelayedFrameSelection",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCameraDelayedSelection::RunTest(const FString& Parameters)
{
	URenderCameraComponent* Cam = MakeBareCamera();
	if (!TestNotNull(TEXT("camera"), Cam))
		return false;

	// Arm latency emulation so PushFrameToHistory uses the time-windowed
	// retention path. A large delay keeps the retain window wide; the SimTime
	// spread here is tiny so nothing is evicted and selection sees all 4 frames.
	Cam->DelaySeconds = 1.0f;
	Cam->bDelayUseWallClock = false;

	// FrameId, SimTime, RevealValue, Seq
	Cam->PushFrameToHistory(MakeDelayFrame(10, 0.00, 0.05, 1));
	Cam->PushFrameToHistory(MakeDelayFrame(20, 0.01, 0.06, 2));
	Cam->PushFrameToHistory(MakeDelayFrame(30, 0.02, 0.07, 3));
	Cam->PushFrameToHistory(MakeDelayFrame(40, 0.03, 0.08, 4));

	FRenderCameraFrame Out;

	// Nothing revealed yet at now=0.04 (earliest reveal is 0.05).
	TestFalse(TEXT("nothing eligible yet"), Cam->SelectDelayedFrame(0.04, 0, Out));

	// now=0.065 -> newest with reveal <= 0.065 is frame 20 (reveal 0.06).
	TestTrue(TEXT("selects newest eligible"), Cam->SelectDelayedFrame(0.065, 0, Out));
	TestEqual(TEXT("picked frame 20"), Out.Identity.FrameId, (uint64)20);
	TestEqual(TEXT("picked seq 2"), Out.Seq, (uint64)2);

	// Same instant, but we've already published seq 2 -> nothing new.
	TestFalse(TEXT("no repeat past AfterSeq"), Cam->SelectDelayedFrame(0.065, 2, Out));

	// Far future reveals everything; newest past seq 2 is frame 40.
	TestTrue(TEXT("advances to newest"), Cam->SelectDelayedFrame(10.0, 2, Out));
	TestEqual(TEXT("picked frame 40"), Out.Identity.FrameId, (uint64)40);

	return true;
}

// ============================================================================
// URLab.CameraHistory.DelayConfig
//   SetCameraDelay / SetCaptureRate set + clamp the latency / capture-rate
//   knobs; defaults match the resource-smart, zero-latency baseline.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCameraDelayConfig,
	"URLab.CameraHistory.DelayConfig",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCameraDelayConfig::RunTest(const FString& Parameters)
{
	URenderCameraComponent* Cam = MakeBareCamera();
	if (!TestNotNull(TEXT("camera"), Cam))
		return false;

	// Defaults: no latency, capture-on-state-change on, no fps cap.
	TestEqual(TEXT("default delay 0"), Cam->DelaySeconds, 0.0f);
	TestEqual(TEXT("default jitter 0"), Cam->DelayJitterSeconds, 0.0f);
	TestFalse(TEXT("default sim clock"), Cam->bDelayUseWallClock);
	TestTrue(TEXT("default state-change capture"), Cam->bCaptureOnStateChange);
	TestEqual(TEXT("default uncapped"), Cam->CaptureMaxFps, 0.0f);

	Cam->SetCameraDelay(0.05f, 0.01f, /*wall=*/true, /*seed=*/123);
	TestTrue(TEXT("delay set"), FMath::IsNearlyEqual(Cam->DelaySeconds, 0.05f));
	TestTrue(TEXT("jitter set"), FMath::IsNearlyEqual(Cam->DelayJitterSeconds, 0.01f));
	TestTrue(TEXT("wall clock set"), Cam->bDelayUseWallClock);

	// Negative inputs clamp to zero.
	Cam->SetCameraDelay(-1.0f, -1.0f, /*wall=*/false, /*seed=*/0);
	TestEqual(TEXT("delay clamps >=0"), Cam->DelaySeconds, 0.0f);
	TestEqual(TEXT("jitter clamps >=0"), Cam->DelayJitterSeconds, 0.0f);

	Cam->SetCaptureRate(/*on_state_change=*/false, /*max_fps=*/30.0f);
	TestFalse(TEXT("state-change off"), Cam->bCaptureOnStateChange);
	TestTrue(TEXT("fps set"), FMath::IsNearlyEqual(Cam->CaptureMaxFps, 30.0f));

	Cam->SetCaptureRate(true, -5.0f);
	TestEqual(TEXT("fps clamps >=0"), Cam->CaptureMaxFps, 0.0f);

	return true;
}

// ============================================================================
// URLab.CameraHistory.CaptureGating
//   A camera is dormant by default, active when a broadcast flag is set, and
//   active after TouchRequested within the TTL.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCameraCaptureGating,
	"URLab.CameraHistory.CaptureGating",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCameraCaptureGating::RunTest(const FString& Parameters)
{
	URenderCameraComponent* Cam = MakeBareCamera();
	if (!TestNotNull(TEXT("camera"), Cam))
		return false;

	TestFalse(TEXT("fresh camera is dormant"), Cam->IsCaptureActive());

	// Host publishers express continuous demand without entering neutral transport code.
	Cam->bContinuousCapture = true;
	TestTrue(TEXT("continuous host demand -> active"), Cam->IsCaptureActive());
	Cam->bContinuousCapture = false;
	TestFalse(TEXT("continuous demand cleared -> dormant again"), Cam->IsCaptureActive());

	// A recent request keeps a non-broadcast camera active within the TTL.
	Cam->RequestActiveTtlSeconds = 3600.0f;
	Cam->TouchRequested();
	TestTrue(TEXT("touched within TTL -> active"), Cam->IsCaptureActive());

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRenderCameraIdentityTransitionDropsOldFrames,
	"URLab.CameraHistory.IdentityTransitionDropsOldFrames",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FRenderCameraIdentityTransitionDropsOldFrames::RunTest(const FString& Parameters)
{
	URenderCameraComponent* Camera = MakeBareCamera();
	FRenderCameraFrame Old = MakeColorFrame(8, 0.08);
	FRenderFrameIdentity Next = Old.Identity;
	Next.ClockEpoch = 4;
	Next.FrameId = 1;
	Next.SourceTime = 0.01;
	Camera->SetAppliedFrameIdentity(Next);
	Camera->PushFrameToHistory(MoveTemp(Old));
	TestEqual(TEXT("old epoch completion cannot repopulate cleared history"),
		Camera->GetLatestFrameId(), uint64(0));

	FRenderCameraFrame Current = MakeColorFrame(1, 0.01);
	Current.Identity.ClockEpoch = 4;
	Camera->PushFrameToHistory(MoveTemp(Current));
	TSharedPtr<const FRenderCameraFrame> Stored = Camera->GetFrameShared(0);
	if (TestTrue(TEXT("current epoch frame accepted"), Stored.IsValid()))
	{
		TestEqual(TEXT("stored scene"), Stored->Identity.SceneRevision, Next.SceneRevision);
		TestEqual(TEXT("stored epoch"), Stored->Identity.ClockEpoch, Next.ClockEpoch);
		TestEqual(TEXT("stored frame"), Stored->Identity.FrameId, Next.FrameId);
		TestTrue(TEXT("stored source time"), FMath::IsNearlyEqual(
			Stored->Identity.SourceTime, Next.SourceTime, 1e-9));
	}
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRenderCameraSourceDelayRequiresSourceTime,
	"URLab.CameraHistory.SourceDelayRequiresSourceTime",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FRenderCameraSourceDelayRequiresSourceTime::RunTest(const FString& Parameters)
{
	URenderCameraComponent* Camera = MakeBareCamera();
	Camera->SetCameraDelay(0.1f, 0.0f, false, 4);
	FRenderFrameIdentity MissingTime = Camera->GetAppliedFrameIdentity();
	MissingTime.bHasSourceTime = false;
	MissingTime.FrameId = 2;
	Camera->SetAppliedFrameIdentity(MissingTime);
	TestFalse(TEXT("capture rejected without selected source clock"),
		Camera->CanCaptureIdentity(MissingTime));
	FRenderCameraFrame Frame = MakeColorFrame(2, 0.0);
	Frame.Identity = MissingTime;
	Camera->PushFrameToHistory(MoveTemp(Frame));
	TestFalse(TEXT("forced/direct frame is not retained"), Camera->GetFrameShared(0).IsValid());
	TestFalse(TEXT("delayed request returns no fabricated zero-time frame"),
		Camera->GetFrameForRequest(0, false).IsValid());
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRenderCameraDelayedHistoryRequiresEligibleFrame,
	"URLab.CameraHistory.DelayedHistoryRequiresEligibleFrame",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FRenderCameraDelayedHistoryRequiresEligibleFrame::RunTest(const FString& Parameters)
{
	// The history primitive accepts an exact clock boundary for both clock kinds,
	// and never substitutes its oldest frame when that frame is still too new.
	FRenderCameraHistory History;
	TSharedPtr<FRenderCameraFrame> Boundary = MakeShared<FRenderCameraFrame>(
		MakeColorFrame(10, 5.0));
	Boundary->CaptureUnixTime = 42.0;
	History.PushShared(Boundary, 4, true, 10.0, false);
	TestFalse(TEXT("source target before oldest is unavailable"),
		History.SelectDelayedByClock(4.99, false).IsValid());
	TestTrue(TEXT("source exact boundary is eligible"),
		History.SelectDelayedByClock(5.0, false).IsValid());
	TestFalse(TEXT("wall target before oldest is unavailable"),
		History.SelectDelayedByClock(41.99, true).IsValid());
	TestTrue(TEXT("wall exact boundary is eligible"),
		History.SelectDelayedByClock(42.0, true).IsValid());

	// The explicit source-delay request follows the same rule. Its applied source
	// clock is deterministic, so this also pins the exact request boundary.
	URenderCameraComponent* ExplicitSource = MakeBareCamera();
	FRenderFrameIdentity Applied = ExplicitSource->GetAppliedFrameIdentity();
	Applied.SourceTime = 10.0;
	ExplicitSource->SetAppliedFrameIdentity(Applied);
	ExplicitSource->PushFrameToHistory(MakeColorFrame(20, 8.01));
	TestFalse(TEXT("explicit source delay rejects newer-only history"),
		ExplicitSource->GetFrameForRequest(0, 2.0).IsValid());

	URenderCameraComponent* ExplicitSourceBoundary = MakeBareCamera();
	ExplicitSourceBoundary->SetAppliedFrameIdentity(Applied);
	ExplicitSourceBoundary->PushFrameToHistory(MakeColorFrame(21, 8.0));
	TSharedPtr<const FRenderCameraFrame> SourceAtBoundary =
		ExplicitSourceBoundary->GetFrameForRequest(0, 2.0);
	if (TestTrue(TEXT("explicit source delay accepts exact boundary"),
		SourceAtBoundary.IsValid()))
	{
		TestEqual(TEXT("explicit source boundary frame"),
			SourceAtBoundary->Identity.FrameId, uint64(21));
	}

	// Exercise configured and explicit wall-clock request paths without depending
	// on scheduler timing: a far-future capture cannot be eligible, while a frame
	// safely older than the delay must be.
	const double WallNow = FRenderCameraDelayModel::NowClockValue(true, 0.0);
	URenderCameraComponent* ConfiguredWall = MakeBareCamera();
	ConfiguredWall->SetCameraDelay(0.5f, 0.0f, true, 11);
	FRenderCameraFrame FutureWall = MakeColorFrame(30, 0.0);
	FutureWall.CaptureUnixTime = WallNow + 10.0;
	ConfiguredWall->PushFrameToHistory(MoveTemp(FutureWall));
	TestFalse(TEXT("configured wall delay rejects newer-only history"),
		ConfiguredWall->GetFrameForRequest(0, false).IsValid());

	URenderCameraComponent* ExplicitWall = MakeBareCamera();
	ExplicitWall->SetCameraDelay(0.0f, 0.0f, true, 12);
	FRenderCameraFrame FutureExplicitWall = MakeColorFrame(31, 0.0);
	FutureExplicitWall.CaptureUnixTime = WallNow + 10.0;
	ExplicitWall->PushFrameToHistory(MoveTemp(FutureExplicitWall));
	TestFalse(TEXT("explicit wall delay rejects newer-only history"),
		ExplicitWall->GetFrameForRequest(0, 0.5).IsValid());

	URenderCameraComponent* ConfiguredWallEligible = MakeBareCamera();
	ConfiguredWallEligible->SetCameraDelay(0.5f, 0.0f, true, 13);
	FRenderCameraFrame OldWall = MakeColorFrame(32, 0.0);
	OldWall.CaptureUnixTime = WallNow - 2.0;
	ConfiguredWallEligible->PushFrameToHistory(MoveTemp(OldWall));
	TestTrue(TEXT("configured wall delay returns eligible history"),
		ConfiguredWallEligible->GetFrameForRequest(0, false).IsValid());

	return !HasAnyErrors();
}
