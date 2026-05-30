// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "EditorSubsystem.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/BridgeServerConfig.h"
#include "MjBridgeServerSubsystem.generated.h"

/**
 * @class UURLabBridgeServerSubsystem
 * @brief Editor-time owner of a UURLabBridgeServer. Reads
 *        Plugins/UnrealRoboticsLab/Config/LocalUnrealRoboticsLab.ini at
 *        Initialize and (when AutoStart=true) brings the server online so
 *        the bridge can connect before any PIE session.
 *
 * Lifecycle: subsystem outlives PIE sessions. AAMjManager registers as
 * the active manager on BeginPlay and deregisters on EndPlay; the server
 * itself stays up across the cycle.
 *
 * Editor-only by definition (UEditorSubsystem); the matching cooked-build
 * lifecycle path is on AAMjManager itself.
 */
UCLASS()
class URLABEDITOR_API UURLabBridgeServerSubsystem : public UEditorSubsystem
{
    GENERATED_BODY()

public:
    /** Editor lifecycle. */
    virtual void Initialize(FSubsystemCollectionBase& Collection) override;
    virtual void Deinitialize() override;

    /** Bring the server online with the current config. Idempotent. */
    void StartServer();

    /** Tear the server down. Idempotent. */
    void StopServer();

    bool IsRunning() const;

    /** Active server, may be null when stopped. */
    UURLabBridgeServer* GetBridgeServer() const { return Server; }

    /** Current config (in-memory). To persist edits, write to the INI via
     *  URLabBridgeServerConfigUtils::SaveToIni and call ReloadConfig. */
    const FURLabBridgeServerConfig& GetConfig() const { return Config; }

    /** Re-read the INI. Does NOT restart the server; caller decides. */
    void ReloadConfig();

private:
    UPROPERTY()
    TObjectPtr<UURLabBridgeServer> Server;

    FURLabBridgeServerConfig Config;
};
