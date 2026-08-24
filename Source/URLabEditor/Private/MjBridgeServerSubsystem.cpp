// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MjBridgeServerSubsystem.h"

#include "Bridge/BridgeServerConfigUtils.h"
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

	// NOTE: the discovery registry entry (write + heartbeat + removal) is now
	// owned by UURLabBridgeServer's Start/Stop lifecycle, so both the editor
	// subsystem's server AND cooked/packaged manager-owned servers register.
	// This subsystem does not write it directly (exactly one writer).

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
	// Server->Stop() removes this instance's registry entry (bridge-owned).
	Server->Stop();
	Server = nullptr;
	UE_LOG(LogURLabEditor, Log, TEXT("[BridgeServer] stopped"));
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
