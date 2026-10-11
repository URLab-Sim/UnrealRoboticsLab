// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Bridge/BridgeSubsystem.h"

#include "Bridge/BridgeServer.h"
#include "Bridge/BridgeServerConfig.h"
#include "Bridge/BridgeServerProvider.h"
#include "Engine/World.h"
#include "Utils/URLabLogging.h"

namespace
{
/** See SetUnboundForTest. */
bool GUnboundForTest = false;
} // namespace

void UURLabBridgeSubsystem::SetUnboundForTest(bool bUnbound)
{
	GUnboundForTest = bUnbound;
}

UURLabBridgeServer* UURLabBridgeSubsystem::EnsureFor(UWorld* World)
{
	if (!World)
	{
		return nullptr;
	}
	UURLabBridgeSubsystem* Sub = World->GetSubsystem<UURLabBridgeSubsystem>();
	return Sub ? Sub->EnsureServer() : nullptr;
}

UURLabBridgeServer* UURLabBridgeSubsystem::Find(UWorld* World)
{
	if (!World)
	{
		return nullptr;
	}
	UURLabBridgeSubsystem* Sub = World->GetSubsystem<UURLabBridgeSubsystem>();
	return Sub ? Sub->GetServer() : nullptr;
}

UURLabBridgeServer* UURLabBridgeSubsystem::EnsureServer()
{
	// The editor's bridge outlives PIE, so it is re-resolved every time rather than
	// cached: the editor subsystem may have stopped and replaced it between cycles,
	// and a stale pointer here would have this world talking to a dead dispatcher.
	if (!bOwnsServer && !GUnboundForTest)
	{
		if (UURLabBridgeServer* Editor = URLabBridgeProvider::ResolveEditorServer())
		{
			Server = Editor;
			return Server;
		}
	}
	if (Server)
	{
		return Server;
	}

	// Cooked / packaged: nothing else owns a bridge in this process, so the world
	// does. The bridge resolves the net config itself -- the caller never loads its
	// own copy, so the port it binds and the port it advertises cannot come from two
	// different reads.
	Server = NewObject<UURLabBridgeServer>(this, TEXT("BridgeServer"));
	bOwnsServer = true;
	const FURLabBridgeServerConfig& Config = Server->ResolveInstanceConfig();
	const FString StepEndpoint = GUnboundForTest
								 ? FString()
								 : FString::Printf(TEXT("tcp://%s:%d"),
									   *Config.BindAddress, Config.StepPort);
	Server->Start(StepEndpoint);
	UE_LOG(LogURLab, Log,
		TEXT("[BridgeSubsystem] bridge up on '%s' (no manager required)"), *StepEndpoint);
	return Server;
}

void UURLabBridgeSubsystem::Deinitialize()
{
	if (bOwnsServer && Server)
	{
		Server->Stop();
	}
	Server = nullptr;
	bOwnsServer = false;
	Super::Deinitialize();
}
