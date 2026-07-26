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

// ============================================================================
// MjRosLinkTests.cpp
//
// Link + behaviour tests for the in-process ROS 2 output:
//  - FURLabRosContext init / shutdown round-trip and idempotent double-init
//  - UURLabRosPublishTransport creates JointState publishers and publishes a
//    collected IR against real rcl without error
//  - FillJointState maps the IR to the JointState parallel arrays correctly
//
// The whole file is compiled only when ROS 2 is linked (URLAB_WITH_ROS2). The
// context-dependent tests early-return true with a log when no live ROS context
// is available, so CI stays green on machines without a running DDS. Byte-level
// rosidl fill correctness over the wire is covered by the standalone harness in
// ros/urlab_ros_ws.
// ============================================================================

#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Transport/RosContext.h"
#include "Transport/RosPublishTransport.h"
#include "State/MjStateTypes.h"

namespace
{
// A minimal collected IR: one articulation with two hinge joints and a free base,
// so the DOF flattening (1/1 hinge, 7/6 free) is exercised without a live model.
FMjStateSnapshot MakeSnapshot()
{
	FMjStateSnapshot Snap;
	Snap.StructureVersion = 1;
	Snap.Clock.SimSec = 3;
	Snap.Clock.SimNsec = 500000000;

	FMjArticulationState Art;
	Art.Name = FName(TEXT("go2"));

	FMjJointState Hip;
	Hip.Name = FName(TEXT("fl_hip"));
	Hip.Type = EMjJointType::Hinge;
	Hip.QPos = {0.10};
	Hip.QVel = {1.10};
	Art.Joints.Add(Hip);

	FMjJointState Knee;
	Knee.Name = FName(TEXT("fl_knee"));
	Knee.Type = EMjJointType::Hinge;
	Knee.QPos = {0.20};
	Knee.QVel = {1.20};
	Art.Joints.Add(Knee);

	FMjJointState Root;
	Root.Name = FName(TEXT("root"));
	Root.Type = EMjJointType::Free;
	Root.QPos = {0.0, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0};
	Root.QVel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	Art.Joints.Add(Root);

	Snap.Articulations.Add(Art);
	return Snap;
}
}  // namespace

// ---------------------------------------------------------------------------
// 1. Context round-trip + idempotent double-init
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosContextRoundTrip,
	"URLab.Ros.ContextRoundTrip",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosContextRoundTrip::RunTest(const FString& Parameters)
{
	FURLabRosContext& Ctx = FURLabRosContext::Get();
	Ctx.Initialize();
	if (!Ctx.IsAvailable())
	{
		UE_LOG(LogTemp, Display,
			TEXT("URLab.Ros.ContextRoundTrip: no live ROS context; skipping."));
		return true;
	}

	UrlabRclContext* const Handle = Ctx.GetHandle();
	TestNotNull(TEXT("context handle after init"), Handle);

	// Double-init is a no-op: the same handle, no second context.
	Ctx.Initialize();
	TestTrue(TEXT("double-init keeps the same handle"), Ctx.GetHandle() == Handle);

	// Teardown then bring it back, leaving the process context available for the
	// rest of the session.
	Ctx.Shutdown();
	TestFalse(TEXT("unavailable after shutdown"), Ctx.IsAvailable());
	Ctx.Initialize();
	TestTrue(TEXT("available again after re-init"), Ctx.IsAvailable());

	return true;
}

// ---------------------------------------------------------------------------
// 2. Publisher create + publish a collected IR against real rcl
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosPublishJointState,
	"URLab.Ros.PublishJointState",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosPublishJointState::RunTest(const FString& Parameters)
{
	FURLabRosContext::Get().Initialize();
	if (!FURLabRosContext::Get().IsAvailable())
	{
		UE_LOG(LogTemp, Display,
			TEXT("URLab.Ros.PublishJointState: no live ROS context; skipping."));
		return true;
	}

	UURLabRosPublishTransport* Transport = NewObject<UURLabRosPublishTransport>();
	TestNotNull(TEXT("transport object"), Transport);
	TestTrue(TEXT("transport init"), Transport->TransportInit());

	const FMjStateSnapshot Snap = MakeSnapshot();
	// First call builds the publisher set; second reuses it (StructureVersion
	// unchanged). Neither should crash or tear the context down.
	Transport->PublishState(Snap);
	Transport->PublishState(Snap);
	TestTrue(TEXT("context still available after publish"),
		FURLabRosContext::Get().IsAvailable());

	Transport->TransportShutdown();
	return true;
}

// ---------------------------------------------------------------------------
// 3. FillJointState: names = part segments, arrays sized to joint DOF counts
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRosFillJointState,
	"URLab.Ros.FillJointState",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRosFillJointState::RunTest(const FString& Parameters)
{
	// Pure IR -> arrays; no live ROS context needed.
	const FMjStateSnapshot Snap = MakeSnapshot();
	const FMjArticulationState& Art = Snap.Articulations[0];

	TArray<FString> Names;
	TArray<double> Positions;
	TArray<double> Velocities;
	UURLabRosPublishTransport::FillJointState(Art, Names, Positions, Velocities);

	// One name per joint, in order.
	TestEqual(TEXT("name count == joint count"), Names.Num(), 3);
	TestEqual(TEXT("name[0]"), Names[0], FString(TEXT("fl_hip")));
	TestEqual(TEXT("name[1]"), Names[1], FString(TEXT("fl_knee")));
	TestEqual(TEXT("name[2]"), Names[2], FString(TEXT("root")));

	// Positions / velocities are the joints' concatenated qpos / qvel slices:
	// 1 + 1 + 7 qpos and 1 + 1 + 6 qvel.
	TestEqual(TEXT("position length == total qpos"), Positions.Num(), 9);
	TestEqual(TEXT("velocity length == total qvel"), Velocities.Num(), 8);
	TestEqual(TEXT("position[0] is hip qpos"), Positions[0], 0.10);
	TestEqual(TEXT("velocity[1] is knee qvel"), Velocities[1], 1.20);

	return true;
}

#endif  // URLAB_WITH_ROS2
