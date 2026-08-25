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
 *        placement.
 *
 * On a game world's BeginPlay it reads the boot flags (-URLabModel, -URLabDrive,
 * -URLabCaps, -URLabSourceFind, -URLabScene) to decide whether to join a live
 * owner over gRPC, load a boot model, auto-join or browse for a discovered
 * driver, or possess a free-fly VR drone -- then spawns the resulting
 * AMjRenderer via SpawnRenderer, framing it with a view camera.
 */
UCLASS()
class URLAB_API UMjRendererLauncher : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;
};
