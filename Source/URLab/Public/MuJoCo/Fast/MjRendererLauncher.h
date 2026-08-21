// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Engine/TimerHandle.h"
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

private:
	/** Spawn + possess the free-fly drone (`-URLabVrViewer`) once a PlayerController
	 *  exists. The PC is often not ready at world BeginPlay (esp. in a packaged
	 *  boot), so this retries itself on a short timer until it succeeds or times out. */
	void TryPossessVrDrone(TWeakObjectPtr<UWorld> WeakWorld, int32 Attempt);

	FTimerHandle VrPossessTimerHandle;
};
