// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "Components/SceneComponent.h"
#include "MuJoCo/Entity/MjOverlayFlags.h"
#include "MjOverlayRenderer.generated.h"

struct mjModel_;
struct mjData_;

/**
 * Draws MuJoCo's visualization overlays -- collision wireframes, joint range
 * arcs, site crosses -- for a compiled model straight from its mjModel + mjData,
 * gated by a composable FMjOverlayFlags bitmask that mirrors mjvOption. One
 * renderer, driven by the flags rather than by a mode: a viewport enables a
 * technique by setting its flag, not by switching a bundle on. The draws read the
 * live mjData pose each call (DrawDebug* has no persistence), so the owner drives
 * DrawOverlays once per frame after the model has stepped.
 */
UCLASS(ClassGroup = (URLab), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjOverlayRenderer : public USceneComponent
{
	GENERATED_BODY()

public:
	/** Bind the model + data the overlays read. Borrowed; the caller owns them. */
	void SetModel(mjModel_* InModel, mjData_* InData);

	/** Which overlays to emit, and the per-group visibility masks. */
	FMjOverlayFlags Flags;

	/** World offset applied to every drawn primitive (UE cm), matching the render
	 *  scene's own origin so the overlays land on the geometry, not the world zero. */
	FVector SceneOrigin = FVector::ZeroVector;

	/** Emit the enabled overlays for this frame. No-op without a bound model/data. */
	void DrawOverlays() const;

private:
	void DrawCollision() const;
	void DrawJoints() const;
	void DrawSites() const;

	// Borrowed; the owning scene holds the mjModel/mjData lifetimes.
	mjModel_* Model = nullptr;
	mjData_* Data = nullptr;
};
