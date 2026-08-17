// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Entity/MjGeomAssetResolver.h"

struct mjModel_;
class UMjRendererAssetBaker;

/**
 * The wire path's resolver: a geom's primitive component built straight from the
 * compiled mjModel. Non-mesh geoms map to a scaled engine primitive (capsules add
 * their two rounded caps); a mesh geom becomes a shared UStaticMesh in the editor
 * or a runtime ProceduralMeshComponent in a packaged build, and that fork is
 * hidden here so the caller is asset-type-agnostic. Materials are applied through
 * the held UMjRendererAssetBaker, whose MJB textures no imported asset set carries.
 */
class URLAB_API FMjBakedAssetResolver : public IMjGeomAssetResolver
{
public:
	FMjBakedAssetResolver(mjModel_* InModel, UMjRendererAssetBaker* InBaker);

	virtual UPrimitiveComponent* MakeGeomComponent(int32 GeomId, AActor* Body) const override;

private:
	// Borrowed; the owning scene holds the mjModel and baker lifetimes.
	mjModel_* Model = nullptr;
	UMjRendererAssetBaker* Baker = nullptr;
};
