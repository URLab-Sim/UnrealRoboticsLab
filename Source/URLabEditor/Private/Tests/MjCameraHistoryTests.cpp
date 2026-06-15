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
