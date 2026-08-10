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
// MjControlOwnershipTests.cpp
//
// Per-articulation control arbitration (FMjControlOwnership + the claim /
// release RPC ops + the control-write gate in the step / twist / qpos / mocap
// handlers). TTL / heartbeat cases drive the ownership object directly with the
// deterministic clock seam; the gate cases drive the dispatcher over an
// FMjUESession manager, using two source ids over one test session.
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "MjTestHelpers.h"
#include "Bridge/ControlOwnership.h"
#include "Bridge/RpcDispatcher.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Elements/MjJointRuntime.h"
#include "State/MjStateCollector.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"

namespace
{
TSharedPtr<FJsonObject> CtrlOwnReq(const TCHAR* Op, const TCHAR* Source, const FString& Art)
{
	TSharedPtr<FJsonObject> R = MakeShared<FJsonObject>();
	R->SetStringField(TEXT("op"), Op);
	R->SetStringField(TEXT("session_id"), TEXT("test-session"));
	R->SetStringField(TEXT("source"), Source);
	R->SetStringField(TEXT("articulation"), Art);
	return R;
}

FString CtrlOwnReplyOp(const TSharedPtr<FJsonObject>& Reply)
{
	FString Op;
	if (Reply.IsValid())
		Reply->TryGetStringField(TEXT("op"), Op);
	return Op;
}

FString CtrlOwnReplyCode(const TSharedPtr<FJsonObject>& Reply)
{
	FString Code;
	if (Reply.IsValid())
		Reply->TryGetStringField(TEXT("code"), Code);
	return Code;
}

FString CtrlOwnReplyOwner(const TSharedPtr<FJsonObject>& Reply)
{
	FString Owner;
	if (Reply.IsValid())
		Reply->TryGetStringField(TEXT("owner"), Owner);
	return Owner;
}
} // namespace

// ---------------------------------------------------------------------------
// Claim / conflict / force-steal.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjControlOwnershipClaimAndSteal,
	"URLab.ControlOwnership.ClaimAndSteal",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjControlOwnershipClaimAndSteal::RunTest(const FString& Parameters)
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

	AMjArticulation* Art = S.Manager->GetAllArticulations()[0];
	const FString ArtName = Art->GetName();
	const FName Key(*ArtName);

	// Source A claims the free art.
	TSharedPtr<FJsonObject> R1 = Disp->Dispatch(CtrlOwnReq(TEXT("claim_control"), TEXT("A"), ArtName));
	TestEqual(TEXT("A claim ok"), CtrlOwnReplyOp(R1), FString(TEXT("claim_control_ok")));
	TestEqual(TEXT("A is owner"), CtrlOwnReplyOwner(R1), FString(TEXT("A")));

	// Source B is refused with the current owner.
	TSharedPtr<FJsonObject> R2 = Disp->Dispatch(CtrlOwnReq(TEXT("claim_control"), TEXT("B"), ArtName));
	TestEqual(TEXT("B refused"), CtrlOwnReplyCode(R2), FString(TEXT("control_claimed")));
	TestEqual(TEXT("conflict reports current owner"), CtrlOwnReplyOwner(R2), FString(TEXT("A")));

	// force:true steals for B.
	TSharedPtr<FJsonObject> R3 = CtrlOwnReq(TEXT("claim_control"), TEXT("B"), ArtName);
	R3->SetBoolField(TEXT("force"), true);
	TSharedPtr<FJsonObject> R3Reply = Disp->Dispatch(R3);
	TestEqual(TEXT("B force-steal ok"), CtrlOwnReplyOp(R3Reply), FString(TEXT("claim_control_ok")));
	TestEqual(TEXT("B is owner after steal"), CtrlOwnReplyOwner(R3Reply), FString(TEXT("B")));

	// The old owner's next write is rejected.
	FString CurrentOwner;
	TestTrue(TEXT("old owner A no longer owns"),
		Disp->GetControlOwnership().CheckWrite(Key, TEXT("A"), CurrentOwner)
			== FMjControlOwnership::EWriteCheck::NotOwner);
	TestEqual(TEXT("owner is now B"), CurrentOwner, FString(TEXT("B")));

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// The control-write gate: non-owner rejected, owner allowed, unclaimed rejected.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjControlOwnershipWriteGate,
	"URLab.ControlOwnership.WriteGate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjControlOwnershipWriteGate::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init([](FMjUESession& Sess) {
			Sess.Joint->SetType(EMjJointType::hinge);
		}))
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

	AMjArticulation* Art = S.Manager->GetAllArticulations()[0];
	const FString ArtName = Art->GetName();

	auto SetQpos = [Disp, &ArtName](const TCHAR* Source, double Value) {
		TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
		Req->SetStringField(TEXT("op"), TEXT("set_qpos"));
		Req->SetStringField(TEXT("session_id"), TEXT("test-session"));
		Req->SetStringField(TEXT("source"), Source);
		Req->SetStringField(TEXT("target"), ArtName);
		Req->SetStringField(TEXT("target_by"), TEXT("actor_name"));
		TArray<TSharedPtr<FJsonValue>> Q;
		Q.Add(MakeShared<FJsonValueNumber>(Value));
		Req->SetArrayField(TEXT("qpos"), Q);
		return Disp->Dispatch(Req);
	};

	// Write to an UNCLAIMED art is rejected — the safe default.
	TSharedPtr<FJsonObject> Unclaimed = SetQpos(TEXT("A"), 0.1);
	TestEqual(TEXT("unclaimed set_qpos rejected"),
		CtrlOwnReplyCode(Unclaimed), FString(TEXT("not_control_owner")));

	// A claims the art.
	TSharedPtr<FJsonObject> Claim = Disp->Dispatch(CtrlOwnReq(TEXT("claim_control"), TEXT("A"), ArtName));
	TestEqual(TEXT("A claim ok"), CtrlOwnReplyOp(Claim), FString(TEXT("claim_control_ok")));

	// Non-owner set_twist -> not_control_owner + owner field (gate precedes the
	// twist-controller check).
	{
		TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
		Req->SetStringField(TEXT("op"), TEXT("set_twist"));
		Req->SetStringField(TEXT("session_id"), TEXT("test-session"));
		Req->SetStringField(TEXT("source"), TEXT("B"));
		Req->SetStringField(TEXT("articulation"), ArtName);
		TArray<TSharedPtr<FJsonValue>> Lin;
		Lin.Add(MakeShared<FJsonValueNumber>(1.0));
		Lin.Add(MakeShared<FJsonValueNumber>(0.0));
		Req->SetArrayField(TEXT("linear"), Lin);
		TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
		TestEqual(TEXT("non-owner set_twist rejected"),
			CtrlOwnReplyCode(Reply), FString(TEXT("not_control_owner")));
		TestEqual(TEXT("set_twist rejection names owner"), CtrlOwnReplyOwner(Reply), FString(TEXT("A")));
	}

	// Non-owner set_qpos -> not_control_owner + owner field.
	TSharedPtr<FJsonObject> NonOwner = SetQpos(TEXT("B"), 0.2);
	TestEqual(TEXT("non-owner set_qpos rejected"),
		CtrlOwnReplyCode(NonOwner), FString(TEXT("not_control_owner")));
	TestEqual(TEXT("set_qpos rejection names owner"), CtrlOwnReplyOwner(NonOwner), FString(TEXT("A")));

	// Owner set_qpos succeeds.
	TSharedPtr<FJsonObject> OwnerWrite = SetQpos(TEXT("A"), 0.3);
	TestEqual(TEXT("owner set_qpos ok"), CtrlOwnReplyOp(OwnerWrite), FString(TEXT("set_qpos_ok")));

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// Step-carried control is gated; an observation-only step is not.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjControlOwnershipStepControlGate,
	"URLab.ControlOwnership.StepControlGate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjControlOwnershipStepControlGate::RunTest(const FString& Parameters)
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
	Disp->SetActiveStepMode(EStepMode::Live);

	FMjStateCollector& C = S.Manager->GetStateCollector();
	C.Init(S.Manager);
	C.RebuildProducerCacheGameThread();

	AMjArticulation* Art = S.Manager->GetAllArticulations()[0];
	const FString ArtName = Art->GetName();

	// A step carrying ctrl for an unowned art is rejected as a whole.
	{
		TSharedPtr<FJsonObject> ArtObj = MakeShared<FJsonObject>();
		TSharedPtr<FJsonObject> CtrlMap = MakeShared<FJsonObject>();
		CtrlMap->SetNumberField(TEXT("j0"), 0.5);
		ArtObj->SetObjectField(TEXT("ctrl_map"), CtrlMap);
		TSharedPtr<FJsonObject> PerArt = MakeShared<FJsonObject>();
		PerArt->SetObjectField(ArtName, ArtObj);

		TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
		Req->SetStringField(TEXT("op"), TEXT("step"));
		Req->SetStringField(TEXT("session_id"), TEXT("test-session"));
		Req->SetObjectField(TEXT("per_articulation"), PerArt);

		TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
		TestEqual(TEXT("unowned control step rejected"),
			CtrlOwnReplyCode(Reply), FString(TEXT("not_control_owner")));
	}

	// The same step without a control payload succeeds (observation is free).
	{
		TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
		Req->SetStringField(TEXT("op"), TEXT("step"));
		Req->SetStringField(TEXT("session_id"), TEXT("test-session"));
		TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
		TestEqual(TEXT("observation-only step ok"),
			CtrlOwnReplyOp(Reply), FString(TEXT("step_ok")));
	}

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// TTL frees a dropped owner.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjControlOwnershipTtlExpiry,
	"URLab.ControlOwnership.TtlExpiry",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjControlOwnershipTtlExpiry::RunTest(const FString& Parameters)
{
	FMjControlOwnership Own;
	const FName Art(TEXT("robot"));
	FString Cur;

	Own.SetClockOverrideForTest(100.0);
	TestTrue(TEXT("A claims with a 5s TTL"),
		Own.Claim(Art, TEXT("A"), 5.0, false, Cur) == FMjControlOwnership::EClaimResult::Ok);

	// Past the TTL with no owner activity, the claim is free.
	Own.SetClockOverrideForTest(107.0);
	TestTrue(TEXT("B claims the expired art"),
		Own.Claim(Art, TEXT("B"), 5.0, false, Cur) == FMjControlOwnership::EClaimResult::Ok);
	TestTrue(TEXT("B now owns"),
		Own.CheckWrite(Art, TEXT("B"), Cur) == FMjControlOwnership::EWriteCheck::Ok);

	return true;
}

// ---------------------------------------------------------------------------
// A write heartbeats the claim: activity within the TTL keeps ownership alive.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjControlOwnershipHeartbeat,
	"URLab.ControlOwnership.Heartbeat",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjControlOwnershipHeartbeat::RunTest(const FString& Parameters)
{
	FMjControlOwnership Own;
	const FName Art(TEXT("robot"));
	FString Cur;

	Own.SetClockOverrideForTest(100.0);
	Own.Claim(Art, TEXT("A"), 5.0, false, Cur);

	// A write inside the window refreshes LastActivity.
	Own.SetClockOverrideForTest(104.0);
	TestTrue(TEXT("owner write ok, refreshes activity"),
		Own.CheckWrite(Art, TEXT("A"), Cur) == FMjControlOwnership::EWriteCheck::Ok);

	// Now past the ORIGINAL TTL (100+5) but within the refreshed one (104+5):
	// ownership holds because the write reset the clock.
	Own.SetClockOverrideForTest(108.0);
	TestTrue(TEXT("ownership held past original TTL after heartbeat"),
		Own.CheckWrite(Art, TEXT("A"), Cur) == FMjControlOwnership::EWriteCheck::Ok);
	TestTrue(TEXT("a second source still cannot claim the live art"),
		Own.Claim(Art, TEXT("B"), 5.0, false, Cur) == FMjControlOwnership::EClaimResult::AlreadyOwned);

	return true;
}

// ---------------------------------------------------------------------------
// Release by a non-owner fails; OnManagerGone clears every claim.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjControlOwnershipReleaseAndReset,
	"URLab.ControlOwnership.ReleaseAndReset",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjControlOwnershipReleaseAndReset::RunTest(const FString& Parameters)
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

	AMjArticulation* Art = S.Manager->GetAllArticulations()[0];
	const FString ArtName = Art->GetName();
	const FName Key(*ArtName);

	// A claims.
	Disp->Dispatch(CtrlOwnReq(TEXT("claim_control"), TEXT("A"), ArtName));

	// Release by a non-owner fails and reports the current owner.
	TSharedPtr<FJsonObject> BadRelease =
		Disp->Dispatch(CtrlOwnReq(TEXT("release_control"), TEXT("B"), ArtName));
	TestEqual(TEXT("non-owner release rejected"),
		CtrlOwnReplyCode(BadRelease), FString(TEXT("not_control_owner")));
	TestEqual(TEXT("release rejection names owner"), CtrlOwnReplyOwner(BadRelease), FString(TEXT("A")));

	// Release by the owner frees it, and a new source can claim.
	TSharedPtr<FJsonObject> GoodRelease =
		Disp->Dispatch(CtrlOwnReq(TEXT("release_control"), TEXT("A"), ArtName));
	TestEqual(TEXT("owner release ok"), CtrlOwnReplyOp(GoodRelease), FString(TEXT("release_control_ok")));
	TSharedPtr<FJsonObject> ReClaim =
		Disp->Dispatch(CtrlOwnReq(TEXT("claim_control"), TEXT("B"), ArtName));
	TestEqual(TEXT("freed art re-claimable"), CtrlOwnReplyOp(ReClaim), FString(TEXT("claim_control_ok")));

	// OnManagerGone drops every claim (arts die with the world).
	Disp->OnManagerGone();
	FString Cur;
	TestTrue(TEXT("OnManagerGone cleared claims"),
		Disp->GetControlOwnership().CheckWrite(Key, TEXT("B"), Cur)
			== FMjControlOwnership::EWriteCheck::NotOwner);

	S.Cleanup();
	return true;
}
