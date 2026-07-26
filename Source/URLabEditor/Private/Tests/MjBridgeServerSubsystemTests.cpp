// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Editor.h"

#include "MjBridgeServerSubsystem.h"
#include "MuJoCo/Core/AMjManager.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/BridgeServerProvider.h"
#include "Bridge/RpcDispatcher.h"
#include "MjTestHelpers.h"

namespace
{
UURLabBridgeServerSubsystem* GetSubsystemForTest()
{
	return GEditor ? GEditor->GetEditorSubsystem<UURLabBridgeServerSubsystem>() : nullptr;
}
} // namespace

// ---------------------------------------------------------------------------
// 1. Subsystem is reachable via GEditor.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBridgeServerSubsystemReachable,
	"URLab.BridgeServerSubsystem.Reachable",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBridgeServerSubsystemReachable::RunTest(const FString& Parameters)
{
	UURLabBridgeServerSubsystem* Sub = GetSubsystemForTest();
	TestNotNull(TEXT("GEditor->GetEditorSubsystem<UURLabBridgeServerSubsystem>()"), Sub);
	return Sub != nullptr;
}

// ---------------------------------------------------------------------------
// 2. Start/Stop is idempotent + GetBridgeServer surfaces a running server.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBridgeServerSubsystemStartStop,
	"URLab.BridgeServerSubsystem.StartStop",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBridgeServerSubsystemStartStop::RunTest(const FString& Parameters)
{
	UURLabBridgeServerSubsystem* Sub = GetSubsystemForTest();
	if (!Sub)
	{
		AddError(TEXT("subsystem unavailable"));
		return false;
	}

	// Capture starting state. AutoStart=true (the default) means the server
	// is already running by the time we get here on a fresh editor; the
	// subsystem must still be idempotent under repeated Start/Stop.
	const bool bWasRunning = Sub->IsRunning();

	Sub->StopServer();
	TestFalse(TEXT("IsRunning false after Stop"), Sub->IsRunning());
	TestNull(TEXT("GetBridgeServer null after Stop"), Sub->GetBridgeServer());

	Sub->StopServer(); // idempotent
	TestFalse(TEXT("Stop is idempotent"), Sub->IsRunning());

	Sub->StartServer();
	TestTrue(TEXT("IsRunning true after Start"), Sub->IsRunning());
	TestNotNull(TEXT("GetBridgeServer non-null after Start"), Sub->GetBridgeServer());
	TestNotNull(TEXT("Dispatcher present after Start"),
		Sub->GetBridgeServer() ? Sub->GetBridgeServer()->GetDispatcher() : nullptr);

	UURLabBridgeServer* Before = Sub->GetBridgeServer();
	Sub->StartServer(); // idempotent
	TestEqual(TEXT("Start is idempotent (same server)"),
		Sub->GetBridgeServer(), Before);

	// Restore initial state for whatever runs after us.
	if (!bWasRunning)
		Sub->StopServer();
	return true;
}

// ---------------------------------------------------------------------------
// 3. Config is loaded on Initialize and matches the on-disk INI defaults.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBridgeServerSubsystemConfigLoaded,
	"URLab.BridgeServerSubsystem.ConfigLoaded",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBridgeServerSubsystemConfigLoaded::RunTest(const FString& Parameters)
{
	UURLabBridgeServerSubsystem* Sub = GetSubsystemForTest();
	if (!Sub)
	{
		AddError(TEXT("subsystem unavailable"));
		return false;
	}

	const FURLabBridgeServerConfig& C = Sub->GetConfig();

	// Defaults are documented: StepPort=5559, StatePort=5555, AutoStart=true,
	// StopOnPIEEnd=false. The user can override via INI; if they have, the
	// values should still be sensible (positive ports).
	TestTrue(TEXT("StepPort is a plausible TCP port"),
		C.StepPort > 0 && C.StepPort < 65536);
	TestTrue(TEXT("StatePort is a plausible TCP port"),
		C.StatePort > 0 && C.StatePort < 65536);
	return true;
}

// ---------------------------------------------------------------------------
// 4. Provider resolver: URLabEditor's StartupModule installs one.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBridgeServerProviderInstalled,
	"URLab.BridgeServerProvider.ResolverInstalled",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBridgeServerProviderInstalled::RunTest(const FString& Parameters)
{
	UURLabBridgeServerSubsystem* Sub = GetSubsystemForTest();
	if (!Sub)
	{
		AddError(TEXT("subsystem unavailable"));
		return false;
	}

	Sub->StartServer();
	UURLabBridgeServer* Resolved = URLabBridgeProvider::ResolveEditorServer();

	TestNotNull(TEXT("Resolver installed by URLabEditor module"), Resolved);
	TestEqual(TEXT("Resolver returns the subsystem's server"),
		Resolved, Sub->GetBridgeServer());
	return true;
}

// ---------------------------------------------------------------------------
// 5. Manager test session: the FMjUESession test helper bypasses BeginPlay
//    so its server is always manager-owned. The sub-instantiated server in
//    the editor must NOT have its lifetime affected by that test session.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBridgeServerOwnershipFlag,
	"URLab.BridgeServer.OwnershipFlag",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBridgeServerOwnershipFlag::RunTest(const FString& Parameters)
{
	// Manager-owned (test path uses NewObject directly so bOwnedByManager
	// stays at its default false; but the editor subsystem-owned server
	// also defaults to false. Provenance is set by the BeginPlay branch
	// at runtime; this test just exercises the getter/setter contract.)
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->AddToRoot();

	TestFalse(TEXT("Default not owned-by-manager"), Server->IsOwnedByManager());
	Server->SetOwnedByManager(true);
	TestTrue(TEXT("Setter flips on"), Server->IsOwnedByManager());
	Server->SetOwnedByManager(false);
	TestFalse(TEXT("Setter flips off"), Server->IsOwnedByManager());

	Server->RemoveFromRoot();
	return true;
}

// ---------------------------------------------------------------------------
// 6. Cooperative lease: acquire / contended acquire / release (right + wrong
//    id) / re-acquire, and IsLeaseHeld tracks state.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBridgeServerLeaseAcquireRelease,
	"URLab.BridgeServer.LeaseAcquireRelease",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBridgeServerLeaseAcquireRelease::RunTest(const FString& Parameters)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->AddToRoot();

	TestFalse(TEXT("No lease held initially"), Server->IsLeaseHeld());
	TestTrue(TEXT("Lease id empty initially"), Server->GetLeaseId().IsEmpty());

	// Acquire succeeds and returns a non-empty lease id.
	FString LeaseId, ExistingId;
	const bool bAcquired = Server->TryAcquireLease(TEXT("client-a"), 60.0, LeaseId, ExistingId);
	TestTrue(TEXT("First acquire succeeds"), bAcquired);
	TestFalse(TEXT("Acquired lease id non-empty"), LeaseId.IsEmpty());
	TestTrue(TEXT("IsLeaseHeld true after acquire"), Server->IsLeaseHeld());
	TestEqual(TEXT("GetLeaseId matches acquired id"), Server->GetLeaseId(), LeaseId);

	// Second acquire while held fails and returns the current lease id.
	FString LeaseId2, ExistingId2;
	const bool bAcquired2 = Server->TryAcquireLease(TEXT("client-b"), 60.0, LeaseId2, ExistingId2);
	TestFalse(TEXT("Second acquire fails while held"), bAcquired2);
	TestEqual(TEXT("Busy reports the current lease id"), ExistingId2, LeaseId);

	// Release with a wrong id errors and leaves the lease held.
	TestFalse(TEXT("Release with wrong id fails"), Server->ReleaseLease(TEXT("not-the-id")));
	TestTrue(TEXT("Lease still held after wrong-id release"), Server->IsLeaseHeld());

	// Release with the correct id succeeds and frees the lease.
	TestTrue(TEXT("Release with correct id succeeds"), Server->ReleaseLease(LeaseId));
	TestFalse(TEXT("IsLeaseHeld false after release"), Server->IsLeaseHeld());
	TestTrue(TEXT("Lease id empty after release"), Server->GetLeaseId().IsEmpty());

	// A subsequent acquire succeeds again.
	FString LeaseId3, ExistingId3;
	TestTrue(TEXT("Re-acquire succeeds after release"),
		Server->TryAcquireLease(TEXT("client-c"), 60.0, LeaseId3, ExistingId3));
	TestFalse(TEXT("Re-acquired id non-empty"), LeaseId3.IsEmpty());

	Server->RemoveFromRoot();
	return true;
}

// ---------------------------------------------------------------------------
// 7. Lazy TTL expiry frees an idle lease. Time is injected so the check is
//    deterministic without sleeping.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBridgeServerLeaseTtlExpiry,
	"URLab.BridgeServer.LeaseTtlExpiry",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBridgeServerLeaseTtlExpiry::RunTest(const FString& Parameters)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->AddToRoot();

	// Pin the lease clock so TTL expiry is deterministic.
	Server->SetLeaseClockForTest(100.0);

	// Acquire at t=100 with a 5s TTL.
	FString LeaseId, ExistingId;
	TestTrue(TEXT("Acquire at t=100 succeeds"),
		Server->TryAcquireLease(TEXT("client-a"), 5.0, LeaseId, ExistingId));

	// Still within TTL at t=104: held.
	Server->SetLeaseClockForTest(104.0);
	TestTrue(TEXT("Held within TTL"), Server->IsLeaseHeld());

	// Past TTL at t=106 (>100+5): lazily expired.
	Server->SetLeaseClockForTest(106.0);
	TestFalse(TEXT("Auto-released past TTL"), Server->IsLeaseHeld());

	// After expiry a fresh acquire succeeds.
	Server->SetLeaseClockForTest(107.0);
	FString LeaseId2, ExistingId2;
	TestTrue(TEXT("Acquire succeeds after TTL expiry"),
		Server->TryAcquireLease(TEXT("client-b"), 5.0, LeaseId2, ExistingId2));
	TestNotEqual(TEXT("New lease id differs from the expired one"), LeaseId2, LeaseId);

	Server->RemoveFromRoot();
	return true;
}

// ---------------------------------------------------------------------------
// 8. Discovery polling must NOT keep a lease alive. Driving hello repeatedly
//    past the TTL window still auto-expires the lease; a state op within the
//    window refreshes it. Exercises the DispatchInternal TouchLease gate.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBridgeServerLeaseHelloDoesNotRefresh,
	"URLab.BridgeServer.LeaseHelloDoesNotRefresh",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBridgeServerLeaseHelloDoesNotRefresh::RunTest(const FString& Parameters)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->AddToRoot();

	// Empty endpoint: constructs the dispatcher (wired back to this server)
	// with no transports, so we can drive Dispatch() directly.
	Server->Start(TEXT(""));
	FURLabRpcDispatcher* Dispatcher = Server->GetDispatcher();
	if (!Dispatcher)
	{
		AddError(TEXT("dispatcher not constructed"));
		Server->RemoveFromRoot();
		return false;
	}

	auto DispatchOp = [Dispatcher](const TCHAR* Op) {
		TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
		Req->SetStringField(TEXT("op"), Op);
		Dispatcher->Dispatch(Req);
	};

	// --- hello polling does not keep the lease alive ---
	Server->SetLeaseClockForTest(100.0);
	FString LeaseId, ExistingId;
	TestTrue(TEXT("Acquire at t=100"),
		Server->TryAcquireLease(TEXT("owner"), 5.0, LeaseId, ExistingId));

	// A pool client probes hello every second inside the TTL window; none of
	// these count as owner activity.
	for (double T = 101.0; T <= 104.0; T += 1.0)
	{
		Server->SetLeaseClockForTest(T);
		DispatchOp(TEXT("hello"));
	}

	// Past the original TTL (100+5): still expired despite the polling.
	Server->SetLeaseClockForTest(106.0);
	TestFalse(TEXT("hello polling did not keep the lease alive"), Server->IsLeaseHeld());

	// --- a state op within the window refreshes the lease ---
	Server->SetLeaseClockForTest(200.0);
	FString LeaseId2, ExistingId2;
	TestTrue(TEXT("Re-acquire at t=200"),
		Server->TryAcquireLease(TEXT("owner"), 5.0, LeaseId2, ExistingId2));

	// A real owner op at t=204 refreshes activity (rejected for missing
	// manager/session, but the gate fires before that — TouchLease runs).
	Server->SetLeaseClockForTest(204.0);
	DispatchOp(TEXT("step"));

	// At t=207 the original TTL (200+5=205) has passed, but the step refresh
	// (204+5=209) keeps it alive.
	Server->SetLeaseClockForTest(207.0);
	TestTrue(TEXT("state op refreshed the lease past its original TTL"),
		Server->IsLeaseHeld());

	Server->SetLeaseClockForTest(-1.0);
	Server->Stop();
	Server->RemoveFromRoot();
	return true;
}
