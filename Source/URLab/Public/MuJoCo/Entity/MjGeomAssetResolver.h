// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

class UPrimitiveComponent;
class AActor;
struct FMjMeshFrameInverse;

/**
 * Resolve a geom to a ready-to-draw primitive COMPONENT + its material. COMPONENT-level (not a bare
 * UStaticMesh*) because packaged builds have no StaticMesh asset -- they build a runtime
 * ProceduralMesh. One shared renderer, two impls: baked-from-mjModel (wire) and imported UE assets
 * (authored), neither rebuilding what it already has.
 */
class URLAB_API IMjGeomAssetResolver
{
public:
	virtual ~IMjGeomAssetResolver() = default;

	/** Create + attach the geom's primitive component under Body, material applied. Null on bad id. */
	virtual UPrimitiveComponent* MakeGeomComponent(int32 GeomId, AActor* Body) const = 0;

	/**
	 * The mesh-frame correction a geom's component needs, or null when it needs none.
	 *
	 * A resolver that draws a raw asset (the imported path) returns the inverse of
	 * the pose mj_compile baked into the mesh, so the transform stream can land the
	 * asset where the compiled geom is. A resolver that draws the compiled
	 * `mesh_vert` (the baked path) needs no correction and returns null.
	 */
	virtual const FMjMeshFrameInverse* FindMeshFrameInverse(int32 GeomId) const { return nullptr; }
};
