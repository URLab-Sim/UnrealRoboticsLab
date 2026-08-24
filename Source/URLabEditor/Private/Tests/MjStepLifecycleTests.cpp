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
// MjStepLifecycleTests.cpp
//
// Step-mode lifecycle over the RPC surface, grounded in
// audit_docs/0_render_source_of_truth.md §3 (FreeRun / Stepped / StatePushed)
// and cleanup_audit_addendum.md §A4.
//
//   - set_mode round-trips every wire token (freerun / stepped / statepushed),
//     the reply echoes previous/current mode, the resolved ActiveStepMode enum
//     tracks the token, and an unknown token fails with bad_mode. The wire
//     tokens are the frozen client contract and must equal the EMjStepMode
//     names -- StepModeToString / StepModeFromString are file-static in
//     RpcHandlers_Step.cpp, so the round-trip is asserted through the public
//     dispatch surface that both feed.
//
//   - A4 (EXPECTED FAIL): after a reset in a paused stepped / state-pushed
//     session, the render snapshot must reflect the reset pose immediately.
//     HandleReset runs mj_forward under the lock but never PushRenderState, so
//     the published render frame id does not advance and cameras/render show the
//     pre-reset pose until the physics loop's idle timeout. The step path
//     (StepStatePushed / the direct handler) does push; reset must too.
//
// All tests use FMjUESession (no live ZMQ socket): Init stands the dispatcher
// up after Compile(), and the tests drive its handler entry points directly.
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "MjTestHelpers.h"
#include "Bridge/RpcDispatcher.h"
#include "Bridge/RpcErrorCodes.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Entity/MjPoseSource.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"

namespace
{
TSharedPtr<FJsonObject> LifecycleOp(const TCHAR* Op)
{
	TSharedPtr<FJsonObject> R = MakeShared<FJsonObject>();
	R->SetStringField(TEXT("op"), Op);
	R->SetStringField(TEXT("session_id"), TEXT("test-session"));
	return R;
}

FString ReplyOp(const TSharedPtr<FJsonObject>& Reply)
{
	FString Op;
	if (Reply.IsValid())
		Reply->TryGetStringField(TEXT("op"), Op);
	return Op;
}

FString ReplyStr(const TSharedPtr<FJsonObject>& Reply, const TCHAR* Field)
{
	FString V;
	if (Reply.IsValid())
		Reply->TryGetStringField(Field, V);
	return V;
}
} // namespace

// ---------------------------------------------------------------------------
// set_mode round-trips every wire token and the resolved enum tracks it.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepLifecycleSetModeRoundTrip,
	"URLab.StepLifecycle.SetModeRoundTrip",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepLifecycleSetModeRoundTrip::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}

	FURLabRpcDispatcher* Disp = S.Manager->GetStepDispatcher();
	if (!Disp)
	{
		AddError(TEXT("Manager has no StepDispatcher"));
		S.Cleanup();
		return false;
	}
	Disp->SetActiveSessionIdForTest(TEXT("test-session"));

	// Start from a known baseline so previous_mode is deterministic.
	Disp->SetActiveStepMode(EMjStepMode::FreeRun);

	// Each row: the wire token, the enum it must resolve to, and the token the
	// PREVIOUS mode is expected to echo back.
	struct FModeCase
	{
		const TCHAR* Token;
		EMjStepMode Mode;
	};
	const FModeCase Cases[] = {
		{TEXT("stepped"), EMjStepMode::Stepped},
		{TEXT("statepushed"), EMjStepMode::StatePushed},
		{TEXT("freerun"), EMjStepMode::FreeRun},
	};

	FString ExpectedPrev = TEXT("freerun");
	for (const FModeCase& C : Cases)
	{
		TSharedPtr<FJsonObject> Req = LifecycleOp(TEXT("set_mode"));
		Req->SetStringField(TEXT("mode"), C.Token);
		TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);

		TestEqual(FString::Printf(TEXT("set_mode %s -> set_mode_ok"), C.Token),
			ReplyOp(Reply), FString(TEXT("set_mode_ok")));
		TestEqual(FString::Printf(TEXT("set_mode %s echoes previous_mode"), C.Token),
			ReplyStr(Reply, TEXT("previous_mode")), ExpectedPrev);
		// enum -> wire token: current_mode is StepModeToString(NewMode).
		TestEqual(FString::Printf(TEXT("set_mode %s echoes current_mode"), C.Token),
			ReplyStr(Reply, TEXT("current_mode")), FString(C.Token));
		// wire token -> enum: StepModeFromString resolved to the right source.
		TestEqual(FString::Printf(TEXT("set_mode %s resolves ActiveStepMode"), C.Token),
			(int)Disp->GetActiveStepMode(), (int)C.Mode);

		ExpectedPrev = C.Token;
	}

	// An unknown token is rejected with bad_mode rather than silently defaulting.
	{
		TSharedPtr<FJsonObject> Req = LifecycleOp(TEXT("set_mode"));
		Req->SetStringField(TEXT("mode"), TEXT("teleport"));
		TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
		FString Code;
		Reply->TryGetStringField(TEXT("code"), Code);
		TestEqual(TEXT("unknown mode token -> bad_mode"), Code, FString(URLabError::BadMode));
		// A rejected set_mode must not move the resolved source.
		TestEqual(TEXT("rejected set_mode leaves ActiveStepMode at freerun"),
			(int)Disp->GetActiveStepMode(), (int)EMjStepMode::FreeRun);
	}

	Disp->SetActiveStepMode(EMjStepMode::FreeRun);
	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// A4 (EXPECTED FAIL): a reset in a paused stepped / state-pushed session must
// publish the reset pose to the render snapshot immediately.
//
// PushRenderState is the only writer of the render frame id (++FrameId), and it
// is what the state-pushed step and the direct handler call so cameras/render
// track the just-applied state. HandleReset runs mj_forward but omits the push,
// so the frame id does not advance across a reset and a paused session keeps
// showing the pre-reset pose until the idle timeout. The intended contract is
// that reset refreshes the snapshot exactly as a step does -- asserted here as
// "the render frame id advanced across the reset".
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjStepLifecycleResetPushesRenderState,
	"URLab.StepLifecycle.ResetPushesRenderState",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjStepLifecycleResetPushesRenderState::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}

	FURLabRpcDispatcher* Disp = S.Manager->GetStepDispatcher();
	UMjPhysicsEngine* Engine = S.Manager->PhysicsEngine;
	if (!Disp || !Engine)
	{
		AddError(TEXT("Manager missing StepDispatcher or PhysicsEngine"));
		S.Cleanup();
		return false;
	}
	Disp->SetActiveSessionIdForTest(TEXT("test-session"));

	auto ResetAdvancesFrameId = [&](EMjStepMode Mode, const TCHAR* Label) {
		Disp->SetActiveStepMode(Mode);
		// A paused client-driven session: nothing else advances the snapshot, so
		// the frame id moves iff the reset itself pushes render state.
		Engine->SetPaused(true);

		const uint64 Before = Engine->GetRenderFrameId();
		TSharedPtr<FJsonObject> Reply = Disp->Dispatch(LifecycleOp(TEXT("reset")));
		TestEqual(FString::Printf(TEXT("%s: reset -> reset_ok"), Label),
			ReplyOp(Reply), FString(TEXT("reset_ok")));

		const uint64 After = Engine->GetRenderFrameId();
		// EXPECTED FAIL (addendum §A4): HandleReset never calls PushRenderState,
		// so the render frame id does not advance and the paused session shows the
		// pre-reset pose. Should advance once, as StepStatePushed / the direct
		// handler do after applying state.
		TestTrue(FString::Printf(
					 TEXT("EXPECTED FAIL (addendum §A4): %s reset must PushRenderState (frame id %llu -> %llu)"),
					 Label, (unsigned long long)Before, (unsigned long long)After),
			After > Before);
	};

	ResetAdvancesFrameId(EMjStepMode::Stepped, TEXT("stepped"));
	ResetAdvancesFrameId(EMjStepMode::StatePushed, TEXT("statepushed"));

	Disp->SetActiveStepMode(EMjStepMode::FreeRun);
	S.Cleanup();
	return true;
}
