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
// MjRpcFeatureTests.cpp
//
// Bridge/RPC feature guards grounded in cleanup_audit_addendum.md §A/§B and
// 0_render_source_of_truth.md §8/§10/§17. Three areas, each asserting the
// INTENDED behavior -- so the bug-sitting cases FAIL now and the fix is what
// turns them green:
//
//   1. Control-owner gating parity. A non-owner set_qpos / set_twist is
//      rejected (RejectIfNotControlOwner). set_user_channels and
//      set_geom_appearance are open, non-authoritative inputs by design and
//      stay ungated -- a non-owner writing them succeeds. This is the
//      intended model, not a bug.
//
//   2. Lease TTL activity accounting. OpRefreshesLease excludes discovery /
//      bootstrap / status ops so steady polling can't keep a held lease alive
//      forever. fastpath_hello is a renderer's discovery/model handshake and
//      must be excluded like hello (addendum §A), but it is not -- so it
//      refreshes the lease and defeats TTL auto-release. EXPECTED FAIL.
//
//   3. Reply-schema fidelity. A handler's emitted reply fields must match its
//      declared ReplyFields (the schema the `meta` op ships for client stub
//      generation). fastpath_hello and fastpath_perturb drift (addendum §B).
//      EXPECTED FAIL.
//
// All tests use FMjUESession (no live ZMQ socket): Init stands the dispatcher
// and its owning bridge up after Compile().
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "MjTestHelpers.h"
#include "Bridge/RpcDispatcher.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/OpRegistry.h"
#include "Bridge/RpcErrorCodes.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Entity/MjPoseSource.h"
#include "MuJoCo/Elements/MjJointRuntime.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"

namespace
{
const TCHAR* kSession = TEXT("test-session");

TSharedPtr<FJsonObject> Op(const TCHAR* OpName)
{
	TSharedPtr<FJsonObject> R = MakeShared<FJsonObject>();
	R->SetStringField(TEXT("op"), OpName);
	return R;
}

TSharedPtr<FJsonObject> SessionOp(const TCHAR* OpName)
{
	TSharedPtr<FJsonObject> R = Op(OpName);
	R->SetStringField(TEXT("session_id"), kSession);
	return R;
}

FString ReplyCode(const TSharedPtr<FJsonObject>& Reply)
{
	FString Code;
	if (Reply.IsValid())
		Reply->TryGetStringField(TEXT("code"), Code);
	return Code;
}

FString ReplyOp(const TSharedPtr<FJsonObject>& Reply)
{
	FString V;
	if (Reply.IsValid())
		Reply->TryGetStringField(TEXT("op"), V);
	return V;
}

TSharedPtr<FJsonValue> Num(double V) { return MakeShared<FJsonValueNumber>(V); }

// The declared reply field NAMES for an op, parsed from the "name:type?" wire
// schema in the op registry (drop the type suffix and the optional-marker).
TSet<FString> DeclaredReplyFieldNames(const FString& OpName)
{
	TSet<FString> Names;
	const TOptional<URLabOpRegistry::FOpDecl> Decl = URLabOpRegistry::FindOp(OpName);
	if (Decl.IsSet())
	{
		for (const FString& F : Decl->ReplyFields)
		{
			FString Name = F;
			int32 Colon = INDEX_NONE;
			if (Name.FindChar(TEXT(':'), Colon))
				Name = Name.Left(Colon);
			Name.RemoveFromEnd(TEXT("?"));
			Names.Add(Name);
		}
	}
	return Names;
}

// The top-level field NAMES a reply actually emitted. A msgpack binary field
// lands in the JSON DOM under a "__b64__"-suffixed key (e.g. mjb -> mjb__b64__);
// normalise it back to the schema name so the two sets are comparable.
TSet<FString> EmittedFieldNames(const TSharedPtr<FJsonObject>& Reply)
{
	TSet<FString> Names;
	if (Reply.IsValid())
	{
		for (const TPair<FString, TSharedPtr<FJsonValue>>& KV : Reply->Values)
		{
			FString Name = KV.Key;
			Name.RemoveFromEnd(TEXT("__b64__"));
			Names.Add(Name);
		}
	}
	return Names;
}

// Fields the reply emitted that the schema never declared. The contract is that
// this is empty: every field a handler emits is described by its ReplyFields.
TArray<FString> UndeclaredEmittedFields(const FString& OpName,
	const TSharedPtr<FJsonObject>& Reply)
{
	const TSet<FString> Declared = DeclaredReplyFieldNames(OpName);
	TArray<FString> Undeclared;
	for (const FString& Emitted : EmittedFieldNames(Reply))
	{
		if (!Declared.Contains(Emitted))
			Undeclared.Add(Emitted);
	}
	Undeclared.Sort();
	return Undeclared;
}
} // namespace

// ---------------------------------------------------------------------------
// Control-owner gating parity across the state-injecting ops.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRpcFeatureInputOwnershipGating,
	"URLab.RpcFeature.InputOwnershipGating",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRpcFeatureInputOwnershipGating::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init([](FMjUESession& Sess) { Sess.Joint->SetType(EMjJointType::hinge); }))
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
	Disp->SetActiveSessionIdForTest(kSession);

	AMjArticulation* Art = S.Manager->GetAllArticulations()[0];
	const FString ArtName = Art->GetName();

	// Owner A claims the articulation; B is a second control source on the same
	// session that owns nothing.
	{
		TSharedPtr<FJsonObject> Claim = SessionOp(TEXT("claim_control"));
		Claim->SetStringField(TEXT("control_owner"), TEXT("A"));
		Claim->SetStringField(TEXT("articulation"), ArtName);
		TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Claim);
		TestEqual(TEXT("A claim ok"), ReplyOp(Reply), FString(TEXT("claim_control_ok")));
	}

	// --- Baseline: the gated ops reject a non-owner today. ---

	// set_qpos (non-owner B) -> not_control_owner.
	{
		TSharedPtr<FJsonObject> Req = SessionOp(TEXT("set_qpos"));
		Req->SetStringField(TEXT("control_owner"), TEXT("B"));
		Req->SetStringField(TEXT("target"), ArtName);
		Req->SetStringField(TEXT("target_by"), TEXT("actor_name"));
		TArray<TSharedPtr<FJsonValue>> Q;
		Q.Add(Num(0.2));
		Req->SetArrayField(TEXT("qpos"), Q);
		TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
		TestEqual(TEXT("non-owner set_qpos rejected (baseline)"),
			ReplyCode(Reply), FString(TEXT("not_control_owner")));
	}

	// set_twist (non-owner B) -> not_control_owner (gate precedes the twist check).
	{
		TSharedPtr<FJsonObject> Req = SessionOp(TEXT("set_twist"));
		Req->SetStringField(TEXT("control_owner"), TEXT("B"));
		Req->SetStringField(TEXT("articulation"), ArtName);
		TArray<TSharedPtr<FJsonValue>> Lin;
		Lin.Add(Num(1.0));
		Lin.Add(Num(0.0));
		Req->SetArrayField(TEXT("linear"), Lin);
		TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
		TestEqual(TEXT("non-owner set_twist rejected (baseline)"),
			ReplyCode(Reply), FString(TEXT("not_control_owner")));
	}

	// --- Open inputs: the same non-owner succeeds via the ungated ops. ---

	// set_user_channels (non-owner B) targeting A's articulation. This is an
	// open, non-authoritative input by design -- a non-owner is expected to
	// reach the handler and get set_user_channels_ok, not not_control_owner.
	{
		TSharedPtr<FJsonObject> Channels = MakeShared<FJsonObject>();
		Channels->SetNumberField(TEXT("intruder"), 1.0);
		TSharedPtr<FJsonObject> ArtsMap = MakeShared<FJsonObject>();
		ArtsMap->SetObjectField(ArtName, Channels);

		TSharedPtr<FJsonObject> Req = SessionOp(TEXT("set_user_channels"));
		Req->SetStringField(TEXT("control_owner"), TEXT("B"));
		Req->SetObjectField(TEXT("arts"), ArtsMap);
		TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
		TestEqual(TEXT("non-owner set_user_channels succeeds (open input, not gated)"),
			ReplyOp(Reply), FString(TEXT("set_user_channels_ok")));
	}

	// set_geom_appearance (non-owner B) targeting a geom on A's entity. This is
	// an open, non-authoritative input by design -- a non-owner is expected to
	// get set_geom_appearance_ok, not not_control_owner.
	{
		TSharedPtr<FJsonObject> Req = SessionOp(TEXT("set_geom_appearance"));
		Req->SetStringField(TEXT("control_owner"), TEXT("B"));
		Req->SetStringField(TEXT("entity"), ArtName);
		Req->SetStringField(TEXT("geom"), TEXT("TestGeom"));
		TArray<TSharedPtr<FJsonValue>> Color;
		Color.Add(Num(1.0));
		Color.Add(Num(0.0));
		Color.Add(Num(0.0));
		Req->SetArrayField(TEXT("color"), Color);
		TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
		TestEqual(TEXT("non-owner set_geom_appearance succeeds (open input, not gated)"),
			ReplyOp(Reply), FString(TEXT("set_geom_appearance_ok")));
	}

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// Lease TTL: fastpath_hello must not count as owner activity.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRpcFeatureFastpathHelloLease,
	"URLab.RpcFeature.FastpathHelloDoesNotRefreshLease",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRpcFeatureFastpathHelloLease::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}

	FURLabRpcDispatcher* Disp = S.Manager->GetStepDispatcher();
	UURLabBridgeServer* Bridge = S.Manager->BridgeServer;
	if (!Disp || !Bridge)
	{
		AddError(TEXT("Manager missing StepDispatcher or BridgeServer"));
		S.Cleanup();
		return false;
	}
	Disp->SetActiveSessionIdForTest(kSession);

	// Re-arm a held 5 s lease at t=StartS, using the injectable lease clock so
	// TTL expiry is deterministic without sleeping.
	auto ArmLease = [&](double StartS) {
		Bridge->SetLeaseClockForTest(StartS);
		if (Bridge->IsLeaseHeld())
			Bridge->ReleaseLease(Bridge->GetLeaseId());
		FString NewId, Existing;
		Bridge->TryAcquireLease(TEXT("pool-client"), 5.0, NewId, Existing);
	};

	// After ArmLease(100): activity=100. Dispatch OpName at t=104 (inside the
	// TTL). Then read the lease at t=108: it survives iff the op refreshed the
	// activity stamp (108-104 <= 5); it auto-expires iff the op did not
	// (108-100 > 5).
	auto LeaseHeldAfterDispatchAt108 = [&](const TSharedPtr<FJsonObject>& Req) -> bool {
		ArmLease(100.0);
		Bridge->SetLeaseClockForTest(104.0);
		Disp->Dispatch(Req);
		Bridge->SetLeaseClockForTest(108.0);
		return Bridge->IsLeaseHeld();
	};

	// EXPECTED FAIL (addendum §A): fastpath_hello is a renderer discovery/model
	// handshake and must NOT refresh the lease (OpRefreshesLease omits it), so at
	// t=108 the idle lease should have auto-released. Today it refreshes and the
	// lease is still held -- defeating TTL auto-release.
	{
		const bool bHeld = LeaseHeldAfterDispatchAt108(Op(TEXT("fastpath_hello")));
		TestFalse(
			TEXT("EXPECTED FAIL (addendum §A): fastpath_hello must not refresh the lease (idle lease should expire)"),
			bHeld);
	}

	// Control: hello is already excluded from lease activity, so the idle lease
	// expires. (hello mints a fresh session id; re-plant the test session after.)
	{
		const bool bHeld = LeaseHeldAfterDispatchAt108(Op(TEXT("hello")));
		TestFalse(TEXT("control: hello does not refresh the lease"), bHeld);
		Disp->SetActiveSessionIdForTest(kSession);
	}

	// Control: a real state-advancing op (forward) DOES refresh, so the lease is
	// held past the original TTL -- the behavior the exclusion list is carved out
	// from.
	{
		TSharedPtr<FJsonObject> Fwd = SessionOp(TEXT("forward"));
		const bool bHeld = LeaseHeldAfterDispatchAt108(Fwd);
		TestTrue(TEXT("control: a state-advancing op refreshes the lease"), bHeld);
	}

	// Restore the wall clock for downstream tests sharing the process.
	Bridge->SetLeaseClockForTest(-1.0);
	if (Bridge->IsLeaseHeld())
		Bridge->ReleaseLease(Bridge->GetLeaseId());

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// Reply-schema fidelity: fastpath_hello emits fields it never declares.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRpcFeatureReplySchemaFastpathHello,
	"URLab.RpcFeature.ReplySchemaFastpathHello",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRpcFeatureReplySchemaFastpathHello::RunTest(const FString& Parameters)
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
	Disp->SetActiveSessionIdForTest(kSession);

	// Harness sanity / passing control: set_mode declares exactly what it emits.
	{
		Disp->SetActiveStepMode(EMjStepMode::FreeRun);
		TSharedPtr<FJsonObject> Req = SessionOp(TEXT("set_mode"));
		Req->SetStringField(TEXT("mode"), TEXT("stepped"));
		TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
		TestEqual(TEXT("set_mode -> set_mode_ok (control)"),
			ReplyOp(Reply), FString(TEXT("set_mode_ok")));
		TestEqual(TEXT("control: set_mode emits only declared reply fields"),
			UndeclaredEmittedFields(TEXT("set_mode"), Reply).Num(), 0);
		Disp->SetActiveStepMode(EMjStepMode::FreeRun);
	}

	// fastpath_hello is pre-session; it bypasses the session gate and serves the
	// live model handshake.
	TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Op(TEXT("fastpath_hello")));
	TestEqual(TEXT("fastpath_hello -> fastpath_hello_ok"),
		ReplyOp(Reply), FString(TEXT("fastpath_hello_ok")));

	const TArray<FString> Undeclared = UndeclaredEmittedFields(TEXT("fastpath_hello"), Reply);
	for (const FString& F : Undeclared)
		AddInfo(FString::Printf(TEXT("fastpath_hello emits undeclared field: %s"), *F));

	// EXPECTED FAIL (addendum §B): declares {op,mjb,bus,ngeom} but also emits
	// model_format / broadcasting (and, when a compiled scene is available,
	// xml / vfs_assets). The declared schema must cover every emitted field.
	TestEqual(
		TEXT("EXPECTED FAIL (addendum §B): fastpath_hello emits only declared reply fields"),
		Undeclared.Num(), 0);

	S.Cleanup();
	return true;
}

// ---------------------------------------------------------------------------
// Reply-schema fidelity: fastpath_perturb (drag-intent) emits an undeclared field.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRpcFeatureReplySchemaFastpathPerturb,
	"URLab.RpcFeature.ReplySchemaFastpathPerturb",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRpcFeatureReplySchemaFastpathPerturb::RunTest(const FString& Parameters)
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
	if (!S.Manager->Perturbation)
	{
		AddError(TEXT("Manager has no Perturbation to drive a drag intent"));
		S.Cleanup();
		return false;
	}
	// The manager accepts input by default (bAcceptInput=true); assert it so the
	// drag-intent path is reachable rather than short-circuiting on capability.
	TestTrue(TEXT("manager accepts input (AcceptInput capability on)"),
		S.Manager->HasCapability(EMjCapability::AcceptInput));

	// Drive the drag-INTENT shape (recognised by `active` / `localpos` /
	// `refselpos`), which replies with {op, select}.
	TSharedPtr<FJsonObject> Req = Op(TEXT("fastpath_perturb"));
	Req->SetBoolField(TEXT("active"), true);
	Req->SetNumberField(TEXT("select"), 1);
	TArray<TSharedPtr<FJsonValue>> Zero3;
	Zero3.Add(Num(0.0));
	Zero3.Add(Num(0.0));
	Zero3.Add(Num(0.0));
	Req->SetArrayField(TEXT("localpos"), Zero3);
	Req->SetArrayField(TEXT("refselpos"), Zero3);

	TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
	TestEqual(TEXT("fastpath_perturb (drag intent) -> fastpath_perturb_ok"),
		ReplyOp(Reply), FString(TEXT("fastpath_perturb_ok")));

	const TArray<FString> Undeclared = UndeclaredEmittedFields(TEXT("fastpath_perturb"), Reply);
	for (const FString& F : Undeclared)
		AddInfo(FString::Printf(TEXT("fastpath_perturb emits undeclared field: %s"), *F));

	// EXPECTED FAIL (addendum §B): declares {op,body} but the drag-intent reply
	// returns {op,select} -- `select` is undeclared.
	TestEqual(
		TEXT("EXPECTED FAIL (addendum §B): fastpath_perturb emits only declared reply fields"),
		Undeclared.Num(), 0);

	S.Cleanup();
	return true;
}
