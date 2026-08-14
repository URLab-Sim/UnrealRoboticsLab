// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MjBridgeServerSubsystem.h"

#include "Bridge/BridgeServerConfigUtils.h"
#include "Bridge/InstanceRegistry.h"
#include "URLabEditorLogging.h"

void UURLabBridgeServerSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	ReloadConfig();
	UE_LOG(LogURLabEditor, Log,
		TEXT("[BridgeServer] config: AutoStart=%s StepPort=%d StatePort=%d StopOnPIEEnd=%s"),
		Config.bAutoStart ? TEXT("true") : TEXT("false"),
		Config.StepPort, Config.StatePort,
		Config.bStopOnPIEEnd ? TEXT("true") : TEXT("false"));

	if (Config.bAutoStart)
	{
		StartServer();
	}
}

void UURLabBridgeServerSubsystem::Deinitialize()
{
	StopServer();
	Super::Deinitialize();
}

void UURLabBridgeServerSubsystem::StartServer()
{
	if (Server && Server->IsRunning())
		return;

	if (!Server)
	{
		Server = NewObject<UURLabBridgeServer>(this, TEXT("EditorBridgeServer"));
	}
	Server->SetInstanceConfig(Config);
	const FString Endpoint = FString::Printf(TEXT("tcp://%s:%d"), *Config.BindAddress, Config.StepPort);
	Server->Start(Endpoint);
	Server->EnsureShmBound(Config.InstanceId); // empty id -> "live" (single editor)

	CachedUrlabVersion.Reset();
	if (const FURLabRpcDispatcher* Dispatcher = Server->GetDispatcher())
		CachedUrlabVersion = Dispatcher->URLabVersion;
	FURLabInstanceRegistry::WriteEntry(Config, CachedUrlabVersion,
		/*bManagerPresent=*/false, /*bBusy=*/false);

	// Refresh the registry entry on a ticker so its mtime stays fresh (discovery
	// treats a too-old entry as dead) and its `busy` tracks the live lease.
	if (!HeartbeatHandle.IsValid())
	{
		HeartbeatHandle = FTSTicker::GetCoreTicker().AddTicker(
			FTickerDelegate::CreateUObject(this, &UURLabBridgeServerSubsystem::RefreshRegistryHeartbeat),
			/*DelaySeconds=*/10.0f);
	}

	UE_LOG(LogURLabEditor, Log,
		TEXT("[BridgeServer] started instance='%s' index=%d bind=%s step=%d state=%d cam_base=%d "
			 "(rpc_transports=%d)"),
		Config.InstanceId.IsEmpty() ? TEXT("live") : *Config.InstanceId,
		Config.InstanceIndex, *Config.BindAddress, Config.StepPort, Config.StatePort,
		Config.CamBasePort, Server->GetRpcTransports().Num());
}

void UURLabBridgeServerSubsystem::StopServer()
{
	if (!Server)
		return;
	if (HeartbeatHandle.IsValid())
	{
		FTSTicker::GetCoreTicker().RemoveTicker(HeartbeatHandle);
		HeartbeatHandle.Reset();
	}
	Server->Stop();
	Server = nullptr;
	FURLabInstanceRegistry::RemoveEntry(Config);
	UE_LOG(LogURLabEditor, Log, TEXT("[BridgeServer] stopped"));
}

bool UURLabBridgeServerSubsystem::RefreshRegistryHeartbeat(float /*DeltaTime*/)
{
	if (!Server)
		return false; // server gone: stop ticking
	FURLabInstanceRegistry::RefreshEntry(Config, CachedUrlabVersion,
		/*bManagerPresent=*/false, /*bBusy=*/Server->IsLeaseHeld());
	return true; // keep ticking
}

bool UURLabBridgeServerSubsystem::IsRunning() const
{
	return Server && Server->IsRunning();
}

void UURLabBridgeServerSubsystem::ReloadConfig()
{
	Config = FURLabBridgeServerConfig{}; // reset to defaults
	URLabBridgeServerConfigUtils::LoadFromIni(Config);
	URLabBridgeServerConfigUtils::ApplyEnvAndCommandLineOverrides(Config);
}
