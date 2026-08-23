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
// endorsed by, or sponsored by Epic Games, Inc.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

// ============================================================================
// MjRemotePerturbTests.cpp
//
// Tier-1 GAP 1.a (UE, Phase 4.1): a forwarded interactive drag INTENT
// (fastpath_perturb: {select, active, localpos, refselpos}) must ARM the mjv
// spring on a UE owner -- UMjPerturbation::ApplyRemoteDragIntent latches the
// selection + activates mjPERT_TRANSLATE, so the pre-step callback's
// mjv_applyPerturbForce drives d->xfrc_applied. Pre-4.1 the forwarded intent was
// ignored, so the spring never armed and the wrench stayed ZERO.
//
// SCOPE NOTE: the wrench magnitude itself (mass-scaled, +x, damped) is produced
// by the engine PRE-STEP callback, which only runs on the physics worker's step
// loop -- not reachable from a headless automation step, and the mjvPerturb it
// reads is private (no getter). This test therefore pins the observable contract
// the 4.1 fix restored: the intent is no longer a silent no-op -- it arms and
// releases the spring through the public API. The exact mass-scaled wrench is
// asserted on the identical code path by the Python test_perturb.py suite and by
// the Tier-2 UE-owner integration test (URLab.Fast.UeOwnerHonorsForwardedDragIntent).
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "MjTestHelpers.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Input/MjPerturbation.h"
#include <mujoco/mujoco.h>

// ---------------------------------------------------------------------------
// URLab.Perturb.RemoteDragIntentDrivesSpring
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjPerturbRemoteDragIntentDrivesSpring,
	"URLab.Perturb.RemoteDragIntentDrivesSpring",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjPerturbRemoteDragIntentDrivesSpring::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}

	if (!S.Manager->Perturbation)
	{
		AddError(TEXT("Manager has no Perturbation component"));
		S.Cleanup();
		return false;
	}

	mjModel* m = S.Manager->PhysicsEngine->GetModel();
	mjData* d = S.Manager->PhysicsEngine->GetData();
	if (!m || !d || m->nbody < 2)
	{
		AddInfo(TEXT("Skipping: model has <2 bodies"));
		S.Cleanup();
		return true;
	}
	mj_forward(m, d); // populate xpos so InitPerturbFieldsLocked has a valid frame

	// The headless harness bypasses BeginPlay; drive it so the perturbation caches
	// its manager + registers the pre-step callback exactly as at runtime. Without
	// this the component's Manager is null and ApplyRemoteDragIntent no-ops -- which
	// is the very code path (a silent no-op) the 4.1 fix replaced.
	S.Manager->Perturbation->BeginPlay();

	// A forwarded grab on the first non-world body: localpos at the body origin,
	// drag target 0.2 m in +x of the body's current world position.
	const int32 BodyId = 1;
	double LocalPos[3] = {0.0, 0.0, 0.0};
	double RefSelPos[3] = {
		d->xpos[3 * BodyId + 0] + 0.2,
		d->xpos[3 * BodyId + 1],
		d->xpos[3 * BodyId + 2],
	};

	// Pre-condition: nothing selected / dragging before any intent arrives.
	TestFalse(TEXT("no selection before intent"), S.Manager->Perturbation->HasSelection());
	TestFalse(TEXT("not dragging before intent"), S.Manager->Perturbation->IsDragging());

	// Active grab intent -> arms the spring (selection + active translate).
	S.Manager->Perturbation->ApplyRemoteDragIntent(BodyId, /*bActive=*/true, LocalPos, RefSelPos);
	TestTrue(TEXT("forwarded active intent latches selection (was a no-op pre-4.1)"),
		S.Manager->Perturbation->HasSelection());
	TestTrue(TEXT("forwarded active intent activates the drag spring"),
		S.Manager->Perturbation->IsDragging());

	// Release intent -> stops driving (spring settles); selection persists like
	// a local Ctrl-drag release.
	S.Manager->Perturbation->ApplyRemoteDragIntent(BodyId, /*bActive=*/false, LocalPos, RefSelPos);
	TestFalse(TEXT("release intent stops the drag"),
		S.Manager->Perturbation->IsDragging());

	// An invalid selection is a release too (never arms).
	S.Manager->Perturbation->ApplyRemoteDragIntent(/*Select=*/-1, /*bActive=*/true, LocalPos, RefSelPos);
	TestFalse(TEXT("invalid select never arms the drag"),
		S.Manager->Perturbation->IsDragging());

	AddInfo(TEXT("Arming/release contract asserted (the 4.1 no-op fix). The exact "
		"mass-scaled +x wrench into d->xfrc_applied is produced by the physics-worker "
		"pre-step callback (not reachable from a headless step) and is covered by "
		"Python test_perturb.py + the Tier-2 UE-owner integration test."));

	S.Cleanup();
	return true;
}
