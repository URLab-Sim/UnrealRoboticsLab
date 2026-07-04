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
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Components/Bodies/MjWorldBody.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Components/Geometry/MjGeom.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "Engine/World.h"
#include "mujoco/mujoco.h"

// ============================================================================
// URLab.Thread.PIERestart
//   Verifies the system can fully reinitialize after a shutdown cycle
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjThreadPIERestart,
	"URLab.Thread.PIERestart",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjThreadPIERestart::RunTest(const FString& Parameters)
{
	// First session
	{
		FMjUESession S;
		if (!S.Init())
		{
			AddError(FString::Printf(TEXT("First session Init() failed: %s"), *S.LastError));
			return false;
		}
		TestTrue(TEXT("First session: Manager should be initialized"), S.Manager->IsInitialized());
		S.Cleanup();
	}

	// Second session — verifies reinitalization after full shutdown
	{
		FMjUESession S2;
		if (!S2.Init())
		{
			AddError(FString::Printf(TEXT("Second session Init() failed: %s"), *S2.LastError));
			return false;
		}
		TestTrue(TEXT("Second session: Manager should be initialized after restart"),
			S2.Manager->IsInitialized());
		S2.Cleanup();
	}

	return true;
}

// ============================================================================
// URLab.Thread.ConcurrentRead
//   Smoke test: step 200 times and verify xpos values remain finite
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjThreadConcurrentRead,
	"URLab.Thread.ConcurrentRead",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjThreadConcurrentRead::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("Init() failed: %s"), *S.LastError));
		return false;
	}

	// Step 200 times from the test thread while spot-checking xpos after each batch
	const int TotalSteps = 200;
	const int BatchSize = 50;
	bool bAllFinite = true;

	for (int Batch = 0; Batch < TotalSteps / BatchSize; ++Batch)
	{
		S.Step(BatchSize);

		// Read body positions; nv >= 1 because the world body always exists
		if (S.Manager->PhysicsEngine->m_data && S.Manager->PhysicsEngine->m_model)
		{
			const int NBody = S.Manager->PhysicsEngine->m_model->nbody;
			for (int i = 0; i < NBody; ++i)
			{
				const double* XPos = &S.Manager->PhysicsEngine->m_data->xpos[i * 3];
				if (!FMath::IsFinite((float)XPos[0]) || !FMath::IsFinite((float)XPos[1]) || !FMath::IsFinite((float)XPos[2]))
				{
					bAllFinite = false;
					AddError(FString::Printf(
						TEXT("xpos for body %d is non-finite after %d steps"),
						i, (Batch + 1) * BatchSize));
					break;
				}
			}
		}

		if (!bAllFinite)
			break;
	}

	TestTrue(TEXT("All xpos values should remain finite after 200 steps"), bAllFinite);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Thread.PauseResume
//   Verify that pausing and resuming leaves the manager in a running state
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjThreadPauseResume,
	"URLab.Thread.PauseResume",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjThreadPauseResume::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("Init() failed: %s"), *S.LastError));
		return false;
	}

	// Pause
	S.Manager->PhysicsEngine->SetPaused(true);

	// Direct steps still execute; the async loop would honour the flag
	S.Step(10);

	// Resume
	S.Manager->PhysicsEngine->SetPaused(false);

	TestTrue(TEXT("Manager should be running after unpause"), S.Manager->IsRunning());
	TestTrue(TEXT("Manager should be initialized after unpause"), S.Manager->IsInitialized());

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Thread.ModelIntegrity
//   Verify model and data pointers are valid and that time advances after stepping
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjThreadModelIntegrity,
	"URLab.Thread.ModelIntegrity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjThreadModelIntegrity::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("Init() failed: %s"), *S.LastError));
		return false;
	}

	TestTrue(TEXT("Manager->PhysicsEngine->m_model should not be null"), S.Manager->PhysicsEngine->m_model != nullptr);
	TestTrue(TEXT("Manager->PhysicsEngine->m_data should not be null"), S.Manager->PhysicsEngine->m_data != nullptr);

	if (S.Manager->PhysicsEngine->m_model && S.Manager->PhysicsEngine->m_data)
	{
		TestTrue(TEXT("m_model->nq should be >= 0"), S.Manager->PhysicsEngine->m_model->nq >= 0);
		TestTrue(TEXT("m_data->time should be >= 0.0 before stepping"),
			S.Manager->PhysicsEngine->m_data->time >= 0.0);

		S.Step(10);

		TestTrue(TEXT("m_data->time should be > 0.0 after 10 steps"),
			S.Manager->PhysicsEngine->m_data->time > 0.0);
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Thread.LivePacing
//   Runs the async worker in live mode for a wall-clock window and checks that
//   sim time advances at ~real time. Guards two runtime behaviours the headless
//   suite otherwise can't see: the resolved-step-mode fix (a default Auto scene
//   used to fall through to the ~10Hz step-event timeout instead of the pacer)
//   and the hybrid-sleep pacer (must hold the rate, not overshoot into slow-mo).
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjThreadLivePacing,
	"URLab.Thread.LivePacing",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjThreadLivePacing::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("Init() failed: %s"), *S.LastError));
		return false;
	}

	UMjPhysicsEngine* Engine = S.Manager->PhysicsEngine;

	// Live mode, full speed, unpaused, worker running.
	Engine->SetResolvedStepMode(EStepMode::Live);
	Engine->SetSimSpeed(100.0f);
	Engine->SetPaused(false);
	Engine->RunMujocoAsync();

	const double SimStart = Engine->GetSimTime();
	const double WallStart = FPlatformTime::Seconds();
	FPlatformProcess::Sleep(0.5f);
	const double WallElapsed = FPlatformTime::Seconds() - WallStart;
	const double SimElapsed = Engine->GetSimTime() - SimStart;

	// Stop and join the worker before the session tears the engine down.
	Engine->bShouldStopTask = true;
	if (Engine->StepRequestEvent)
		Engine->StepRequestEvent->Trigger();
	if (Engine->AsyncPhysicsFuture.IsValid())
		Engine->AsyncPhysicsFuture.Wait();

	const double Ratio = (WallElapsed > 0.0) ? (SimElapsed / WallElapsed) : 0.0;
	AddInfo(FString::Printf(TEXT("LivePacing: sim=%.3fs wall=%.3fs ratio=%.2f"),
		SimElapsed, WallElapsed, Ratio));

	// Real-time pacing at 100%: sim advances ~= wall (ratio ~1). The old ~10Hz
	// lock gives ratio ~0.02; a pacer that oversleeps gives ratio well under 1;
	// no pacing at all gives ratio well over 1. Wide window to stay non-flaky.
	TestTrue(FString::Printf(TEXT("Live sim advances ~ real time (ratio=%.2f, want 0.5-1.5)"), Ratio),
		Ratio > 0.5 && Ratio < 1.5);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Thread.LiveSnapshotGating
//   In live mode the worker steps continuously but should publish a render
//   snapshot (bump FrameId) only when the game thread has asked for one, so the
//   full-state copy runs at consumer rate rather than physics rate.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjThreadLiveSnapshotGating,
	"URLab.Thread.LiveSnapshotGating",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjThreadLiveSnapshotGating::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(FString::Printf(TEXT("Init() failed: %s"), *S.LastError));
		return false;
	}

	UMjPhysicsEngine* Engine = S.Manager->PhysicsEngine;
	Engine->SetResolvedStepMode(EStepMode::Live);
	Engine->SetSimSpeed(100.0f);
	Engine->SetPaused(false);
	Engine->RunMujocoAsync();

	// Let the worker flush the initial pending publish (bSnapshotWanted defaults
	// true), then clear it: with no consumer asking, FrameId must hold steady
	// even though the worker keeps stepping.
	FPlatformProcess::Sleep(0.05f);
	Engine->bSnapshotWanted.store(false, std::memory_order_release);
	const uint64 IdIdle0 = Engine->GetRenderFrameId();
	FPlatformProcess::Sleep(0.1f);
	const uint64 IdIdle1 = Engine->GetRenderFrameId();

	// Ask for one; the next step should publish.
	Engine->bSnapshotWanted.store(true, std::memory_order_release);
	FPlatformProcess::Sleep(0.05f);
	const uint64 IdAfterRequest = Engine->GetRenderFrameId();

	Engine->bShouldStopTask = true;
	if (Engine->StepRequestEvent)
		Engine->StepRequestEvent->Trigger();
	if (Engine->AsyncPhysicsFuture.IsValid())
		Engine->AsyncPhysicsFuture.Wait();

	TestEqual(TEXT("FrameId holds steady while no consumer requests a snapshot"),
		(int64)IdIdle1, (int64)IdIdle0);
	TestTrue(TEXT("FrameId advances once a consumer requests a snapshot"),
		IdAfterRequest > IdIdle1);

	S.Cleanup();
	return true;
}
