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
    if (Server && Server->IsRunning()) return;

    if (!Server)
    {
        Server = NewObject<UURLabBridgeServer>(this, TEXT("EditorBridgeServer"));
    }
    const FString Endpoint = FString::Printf(TEXT("tcp://0.0.0.0:%d"), Config.StepPort);
    Server->Start(Endpoint);
    Server->EnsureShmBound();  // open req.shm/rep.shm under "live"
    UE_LOG(LogURLabEditor, Log,
        TEXT("[BridgeServer] started on %s (rpc_transports=%d)"),
        *Endpoint, Server->GetRpcTransports().Num());
}

void UURLabBridgeServerSubsystem::StopServer()
{
    if (!Server) return;
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
    Config = FURLabBridgeServerConfig{};  // reset to defaults
    URLabBridgeServerConfigUtils::LoadFromIni(Config);
}
