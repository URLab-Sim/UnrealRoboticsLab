// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "Components/SceneComponent.h"
#include "MuJoCo/Entity/MjOverlayFlags.h"
#include "MjOverlayRenderer.generated.h"

struct mjModel_;
struct FMjRenderSnapshot;

/**
 * Draws MuJoCo's visualization overlays -- collision wireframes, joint range
 * arcs, site crosses -- for a compiled model, gated by a composable
 * FMjOverlayFlags bitmask that mirrors mjvOption. One renderer, driven by the
 * flags rather than by a mode: a viewport enables a technique by setting its flag,
 * not by switching a bundle on. The topology (geom shapes, joint ranges, site
 * sizes) comes from the model; every pose comes from the engine's published render
 * snapshot rather than live mjData, so this is a safe game-thread call while the
 * physics worker steps. The owner drives DrawOverlays once per frame under the
 * render-state lock.
 */
UCLASS(ClassGroup = (URLab), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjOverlayRenderer : public USceneComponent
{
	GENERATED_BODY()

public:
	/** Bind the model whose topology the overlays read. Borrowed; the caller owns it. */
	void SetModel(mjModel_* InModel);

	/** Which overlays to emit, and the per-group visibility masks. */
	FMjOverlayFlags Flags;

	/** Draw the site crosses this frame (our extra, mirroring the authoring toggle). */
	bool bDrawSites = false;

	/** Active perturbation to visualise (mjVIS_PERTURBFORCE / mjVIS_PERTURBOBJ),
	 *  refreshed by the owner each frame from UMjPerturbation. Body id < 0 = none. */
	int32 PerturbBodyId = -1;

	/** Applied perturb wrench on the selected body (force xyz, torque xyz), MuJoCo world. */
	double PerturbForce[6] = {0, 0, 0, 0, 0, 0};

	/** World offset applied to every drawn primitive (UE cm), matching the render
	 *  scene's own origin so the overlays land on the geometry, not the world zero. */
	FVector SceneOrigin = FVector::ZeroVector;

	/** Emit the enabled overlays for this frame from the snapshot's poses. No-op
	 *  without a bound model. */
	void DrawOverlays(const FMjRenderSnapshot& Snap) const;

private:
	void DrawCollision(const FMjRenderSnapshot& Snap) const;
	void DrawJoints(const FMjRenderSnapshot& Snap) const;
	void DrawSites(const FMjRenderSnapshot& Snap) const;
	void DrawCom(const FMjRenderSnapshot& Snap) const;
	void DrawInertia(const FMjRenderSnapshot& Snap) const;
	void DrawContacts(const FMjRenderSnapshot& Snap, bool bPoints, bool bForces, bool bSplit) const;
	void DrawPerturb(const FMjRenderSnapshot& Snap) const;
	void DrawCameras(const FMjRenderSnapshot& Snap) const;
	void DrawLights(const FMjRenderSnapshot& Snap) const;
	void DrawActuators(const FMjRenderSnapshot& Snap) const;
	void DrawTendons(const FMjRenderSnapshot& Snap) const;
	void DrawRangefinders(const FMjRenderSnapshot& Snap) const;
	void DrawConstraints(const FMjRenderSnapshot& Snap) const;
	void DrawStaticBodies(const FMjRenderSnapshot& Snap) const;
	void DrawAutoConnect(const FMjRenderSnapshot& Snap) const;

	// Borrowed; the owning scene holds the mjModel lifetime.
	mjModel_* Model = nullptr;
};
