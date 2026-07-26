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
#include "MuJoCo/Components/Sensors/MjCamera.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "Bridge/RpcDispatcher.h"
#include "Tests/MjTestHelpers.h"
#include "Dom/JsonObject.h"

namespace
{
UMjCamera* MakeBareCamera()
{
	// Transient, unregistered component: the history ring and gating helpers
	// touch only plain members (no render target / world), so this is enough.
	return NewObject<UMjCamera>(GetTransientPackage(), UMjCamera::StaticClass());
}

FMjCameraFrame MakeColorFrame(uint64 FrameId, double SimTime)
{
	FMjCameraFrame F;
	F.FrameId = FrameId;
	F.SimTime = SimTime;
	F.Width = 2;
	F.Height = 1;
	F.Color.Init(FColor(static_cast<uint8>(FrameId & 0xFF), 0, 0, 255), 2);
	return F;
}

FMjCameraFrame MakeDelayFrame(uint64 FrameId, double SimTime, double RevealValue, uint64 Seq)
{
	FMjCameraFrame F = MakeColorFrame(FrameId, SimTime);
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
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCameraHistoryRing,
	"URLab.CameraHistory.RingEvictsAndFetches",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCameraHistoryRing::RunTest(const FString& Parameters)
{
	UMjCamera* Cam = MakeBareCamera();
	if (!TestNotNull(TEXT("camera"), Cam))
		return false;

	Cam->HistoryCapacity = 3;

	FMjCameraFrame Out;
	TestFalse(TEXT("empty history -> GetFrame(0) false"), Cam->GetFrame(0, Out));
	TestEqual(TEXT("empty history -> latest id 0"), Cam->GetLatestFrameId(), (uint64)0);

	for (uint64 Id = 1; Id <= 5; ++Id)
	{
		Cam->PushFrameToHistory(MakeColorFrame(Id, 0.01 * Id));
	}

	// Capacity 3 -> only frames 3,4,5 retained.
	TestEqual(TEXT("latest id is 5"), Cam->GetLatestFrameId(), (uint64)5);

	TestTrue(TEXT("GetFrame(0) latest"), Cam->GetFrame(0, Out));
	TestEqual(TEXT("latest frame id"), Out.FrameId, (uint64)5);

	TestTrue(TEXT("GetFrame(4)"), Cam->GetFrame(4, Out));
	TestEqual(TEXT("exact >=4 is 4"), Out.FrameId, (uint64)4);

	// Frames 1,2 were evicted; oldest retained >= 2 is 3.
	TestTrue(TEXT("GetFrame(2) -> oldest retained >=2"), Cam->GetFrame(2, Out));
	TestEqual(TEXT(">=2 resolves to 3"), Out.FrameId, (uint64)3);

	// Nothing newer than 5.
	TestFalse(TEXT("GetFrame(6) misses"), Cam->GetFrame(6, Out));

	// Payload + metadata survive the round trip.
	TestTrue(TEXT("GetFrame(5)"), Cam->GetFrame(5, Out));
	TestEqual(TEXT("width preserved"), Out.Width, 2);
	TestEqual(TEXT("pixel count preserved"), Out.Color.Num(), 2);
	TestTrue(TEXT("sim_time preserved"), FMath::IsNearlyEqual(Out.SimTime, 0.05, 1e-9));

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
	UMjCamera* Cam = MakeBareCamera();
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

	FMjCameraFrame Out;

	// Nothing revealed yet at now=0.04 (earliest reveal is 0.05).
	TestFalse(TEXT("nothing eligible yet"), Cam->SelectDelayedFrame(0.04, 0, Out));

	// now=0.065 -> newest with reveal <= 0.065 is frame 20 (reveal 0.06).
	TestTrue(TEXT("selects newest eligible"), Cam->SelectDelayedFrame(0.065, 0, Out));
	TestEqual(TEXT("picked frame 20"), Out.FrameId, (uint64)20);
	TestEqual(TEXT("picked seq 2"), Out.Seq, (uint64)2);

	// Same instant, but we've already published seq 2 -> nothing new.
	TestFalse(TEXT("no repeat past AfterSeq"), Cam->SelectDelayedFrame(0.065, 2, Out));

	// Far future reveals everything; newest past seq 2 is frame 40.
	TestTrue(TEXT("advances to newest"), Cam->SelectDelayedFrame(10.0, 2, Out));
	TestEqual(TEXT("picked frame 40"), Out.FrameId, (uint64)40);

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
	UMjCamera* Cam = MakeBareCamera();
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
	UMjCamera* Cam = MakeBareCamera();
	if (!TestNotNull(TEXT("camera"), Cam))
		return false;

	Cam->bEnableZmqBroadcast = false;
	Cam->bEnableShmBroadcast = false;

	TestFalse(TEXT("fresh camera is dormant"), Cam->IsCaptureActive());

	// Broadcast-enabled cameras are always active.
	Cam->bEnableZmqBroadcast = true;
	TestTrue(TEXT("zmq broadcast -> active"), Cam->IsCaptureActive());
	Cam->bEnableZmqBroadcast = false;
	Cam->bEnableShmBroadcast = true;
	TestTrue(TEXT("shm broadcast -> active"), Cam->IsCaptureActive());
	Cam->bEnableShmBroadcast = false;
	TestFalse(TEXT("flags cleared -> dormant again"), Cam->IsCaptureActive());

	// A recent request keeps a non-broadcast camera active within the TTL.
	Cam->RequestActiveTtlSeconds = 3600.0f;
	Cam->TouchRequested();
	TestTrue(TEXT("touched within TTL -> active"), Cam->IsCaptureActive());

	return true;
}

// ============================================================================
// URLab.CameraHistory.StreamingApply
//   set_camera_streaming helpers: name resolution (canonical) and the
//   game-thread apply (disable path — flags cleared, keyed by canonical name,
//   unknown cameras omitted). The enable path binds real ZMQ sockets, covered
//   by the existing MjCamera streaming tests.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCameraStreamingApply,
	"URLab.CameraHistory.StreamingApply",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCameraStreamingApply::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("FMjUESession::Init failed: %s"), *S.LastError));
		return false;
	}

	UMjCamera* Cam = NewObject<UMjCamera>(S.Robot, TEXT("StreamCam"));
	if (!TestNotNull(TEXT("camera"), Cam))
		return false;
	Cam->MjName = TEXT("stream_cam");
	Cam->CaptureMode = EMjCameraMode::Real;
	Cam->RegisterComponent();
	Cam->AttachToComponent(S.Body, FAttachmentTransformRules::KeepRelativeTransform);

	// Name resolution: the canonical "<art>/<part>" name resolves to this camera.
	const FString Canon = Cam->GetCanonicalName();
	TMap<FString, UMjCamera*> ByName;
	FURLabRpcDispatcher::BuildCameraNameMap(S.Manager, ByName);
	TestTrue(TEXT("canonical name registered"), ByName.Contains(Canon));
	if (UMjCamera** F = ByName.Find(Canon))
	{
		TestEqual(TEXT("resolves to the camera"), *F, Cam);
	}

	// Pre-set the flags so the disable path has something to clear.
	Cam->bEnableZmqBroadcast = true;
	Cam->bEnableShmBroadcast = true;

	TMap<FString, TPair<bool, bool>> Reqs;
	Reqs.Add(Canon, TPair<bool, bool>(false, false));
	TSharedPtr<FJsonObject> Cams =
		FURLabRpcDispatcher::ApplyCameraStreamingGameThread(S.Manager, Reqs);
	if (!TestTrue(TEXT("reply valid"), Cams.IsValid()))
		return false;

	const TSharedPtr<FJsonObject>* CamReply = nullptr;
	TestTrue(TEXT("camera keyed by canonical name in reply"),
		Cams->TryGetObjectField(Canon, CamReply));
	bool bStreaming = true;
	if (CamReply && CamReply->IsValid())
	{
		(*CamReply)->TryGetBoolField(TEXT("streaming"), bStreaming);
	}
	TestFalse(TEXT("disabled -> not streaming"), bStreaming);
	TestFalse(TEXT("zmq flag cleared"), Cam->bEnableZmqBroadcast);
	TestFalse(TEXT("shm flag cleared"), Cam->bEnableShmBroadcast);

	// Unknown camera key is omitted from the reply.
	TMap<FString, TPair<bool, bool>> Unknown;
	Unknown.Add(TEXT("nope_not_a_camera"), TPair<bool, bool>(false, false));
	TSharedPtr<FJsonObject> Cams2 =
		FURLabRpcDispatcher::ApplyCameraStreamingGameThread(S.Manager, Unknown);
	TestEqual(TEXT("unknown camera omitted"), Cams2->Values.Num(), 0);

	return true;
}
