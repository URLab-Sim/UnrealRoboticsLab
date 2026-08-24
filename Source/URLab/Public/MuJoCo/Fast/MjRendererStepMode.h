// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Engine/TimerHandle.h"
#include "UObject/WeakObjectPtr.h"

class AMjRenderer;
class AAMjManager;
class AMjArticulation;

/**
 * @struct FMjRendererStepMode
 * @brief Direct-mode plumbing: step an AMjRenderer's own model through the shared engine.
 *
 * Direct mode installs the scene's raw mjModel/mjData into a manager's shared
 * UMjPhysicsEngine, stands up a geometry-less shadow articulation so the RPC layer
 * / a Python client can drive it, and renders the engine's thread-safe stepped
 * snapshot each frame. A plain struct owned by AMjRenderer: it holds only weak actor
 * pointers + PODs (no GC roots needed) and reaches back into the scene (its friend)
 * for the model, geom/camera components and the render origin.
 */
struct FMjRendererStepMode
{
	// The manager whose UMjPhysicsEngine steps our raw model. Get-or-spawned at
	// BeginPlay; not owned here (weak).
	TWeakObjectPtr<AAMjManager> Manager;
	// Frame id of the last render snapshot applied, so Tick skips unchanged frames.
	uint64 LastRenderFrameId = 0;
	// Polls until the manager has begun play, then installs the raw model.
	FTimerHandle InstallTimer;

	// Get-or-spawn the manager and arm the deferred install (the manager compiles its
	// scene + starts its worker in its own BeginPlay; install once that has run).
	void Begin(AMjRenderer& Scene);
	// Install the scene's raw model+data into the manager's engine and start the
	// physics worker. Retried off InstallTimer until the manager has begun play.
	void InstallIntoEngine(AMjRenderer& Scene);
	// Render the geoms + cameras from the engine's published render snapshot
	// (thread-safe; the worker steps the scene's mjData on another thread).
	void ApplyFromSnapshot(AMjRenderer& Scene);
	// Stop-join the worker + unalias the raw model, retire the shadow articulation,
	// clear the install timer, and forget the manager. Safe when Direct never started.
	void Teardown(AMjRenderer& Scene);
	// Live-swap retire: uninstall the model + shadow and reset the render de-dup
	// state, but KEEP the manager so the swap reuses the same engine + RPC context.
	void RetireForReload();
};
