// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"

#include "MjSceneVisuals.generated.h"

class UMjAppearanceStore;
class UMjLightStore;

/**
 * The world's runtime-visuals stores: geom appearance and lights.
 *
 * They hang off the WORLD, not off AAMjManager, because appearance and lighting
 * are properties of the render, and a render server or a subscriber has a
 * world and a renderer but no manager at all. Hanging them off the manager made
 * the ops physics-gated for no reason -- a request render server could not recolour
 * a geom it was drawing. A world subsystem also guarantees exactly one store per
 * world: two stores over the same components would fight, and only one of them
 * would be re-driven after a rebuild.
 *
 * The manager still binds itself into the appearance store when it exists, which
 * is what lets a geom name resolve to a compiled mjModel id.
 */
UCLASS()
class URLAB_API UMjSceneVisualsSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	/** The subsystem for `World`, or null when there is no world (or none is
	 *  initialised, as in a bare automation context). */
	static UMjSceneVisualsSubsystem* Get(UWorld* World);

	/** The appearance store for this world, created on first use. */
	UMjAppearanceStore* Appearance();

	/** The light store for this world, created on first use. */
	UMjLightStore* Lights();

	/** Re-drive every held appearance and light override onto the current world.
	 *  Called after any rebuild that replaces components (render scene swap,
	 *  MJCF recompile, environment re-import). */
	static void ReapplyAll(UWorld* World);

	// UWorldSubsystem
	virtual void Deinitialize() override;

private:
	UPROPERTY(Transient)
	TObjectPtr<UMjAppearanceStore> AppearanceStore;

	UPROPERTY(Transient)
	TObjectPtr<UMjLightStore> LightStore;
};
