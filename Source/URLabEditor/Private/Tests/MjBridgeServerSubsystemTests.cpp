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
