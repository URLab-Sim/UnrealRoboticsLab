// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "MjRendererLauncher.generated.h"

/**
 * @class UMjRendererLauncher
 * @brief Spawns and configures a fast-path AMjRenderer from command-line flags,
 *        so a UE renderer can be launched and connected with no manual actor
 *        placement -- the first step toward CLI-driven / discovered connection.
 *
 * On a game world's BeginPlay, if `-URLabFastMjb=<path>` is present it spawns an
 * AMjRenderer (optionally `-URLabFastBus=<endpoint>` to mirror a live owner) and
 * frames it with a view camera. This is a stopgap for the discovery layer:
 * eventually the endpoint + MJB arrive over the wire from an advertised owner.
 */
UCLASS()
class URLAB_API UMjRendererLauncher : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;
};
