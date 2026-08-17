// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Entity/MjGeomAssetResolver.h"
#include "MuJoCo/Entity/MjBakedAssetResolver.h"

struct mjModel_;
class UMjbAssetBaker;
class UMjGeom;
class UPrimitiveComponent;

/**
 * The authored path's resolver: a compiled geom's component built from the UE
 * assets a model import produced. A compiled scene attaches every participant
 * under a prefix, so the mjModel's names are prefixed and its meshes can be
 * double-prefixed; resolving them by name off the model is unsafe. Instead each
 * compiled geom id is mapped back to the authoring UMjGeom that bound to it, and
 * that element resolves its own `<mesh>`/`<material>` locally -- exactly as the
 * editor preview does -- through its own spec, which is prefix-free and correct
 * across multiple articulations. Anything with no authoring element behind it
 * (a scene-root floor, an inline mesh) falls back to the baked resolver, so a
 * mixed scene still draws in full.
 */
/**
 * The inverse of the pose MuJoCo bakes into a mesh at compile time, in MuJoCo's
 * own frame (wxyz quat).
 *
 * mj_compile recenters a mesh's vertices onto its inertial frame and folds the
 * shift into the geom's `pos`/`quat`, so a geom that draws the compiled
 * `mesh_vert` lands correctly at `geom_xpos`/`geom_xmat`. An imported or
 * converted geom draws the ORIGINAL UStaticMesh, whose vertices still carry the
 * asset's own origin, so it must undo that recentre to sit where the compiled
 * geom does. `Pos`/`Quat` compose onto the geom-frame world pose to do exactly
 * that; an identity value leaves the pose untouched.
 */
struct FMjMeshFrameInverse
{
	double Pos[3] = {0.0, 0.0, 0.0};
	double Quat[4] = {1.0, 0.0, 0.0, 0.0};
};

class URLAB_API FMjImportedAssetResolver : public IMjGeomAssetResolver
{
public:
	FMjImportedAssetResolver(mjModel_* InModel, UMjbAssetBaker* InBaker,
		TMap<int32, TWeakObjectPtr<UMjGeom>> InGeomIndex);

	virtual UPrimitiveComponent* MakeGeomComponent(int32 GeomId, AActor* Body) const override;

	// The mesh-frame correction for a geom that draws a raw imported/converted
	// asset, or null when the geom needs none (a primitive, or a mesh drawn from
	// the compiled `mesh_vert` through the baked fallback). Recorded when the geom's
	// component is built.
	virtual const FMjMeshFrameInverse* FindMeshFrameInverse(int32 GeomId) const override { return MeshFrameInverse.Find(GeomId); }

private:
	// Apply the geom's authored material onto Comp: resolve the `<material>` the
	// element names through its own spec and route it, effective colour and planar
	// size included, through the shared MjApplyMaterialParameters + master material.
	void ApplyImportedMaterial(UPrimitiveComponent* Comp, UMjGeom* Geom, int32 GeomId) const;

	// The authoring element that bound to a compiled geom id, or null.
	UMjGeom* OriginFor(int32 GeomId) const;

	// Record the mesh-frame correction a raw-asset mesh geom needs, from the
	// compiled mesh's baked pos/quat. A no-op for a mesh with no dataid.
	void RecordMeshFrameInverse(int32 GeomId) const;

	// Borrowed; the owning scene holds the mjModel and baker lifetimes.
	mjModel_* Model = nullptr;

	// Compiled geom id -> the authoring UMjGeom that bound to it.
	TMap<int32, TWeakObjectPtr<UMjGeom>> GeomIndex;

	// Compiled geom id -> the mesh-frame correction for the raw asset it draws.
	// Filled as components are built (MakeGeomComponent is const, so this is too).
	mutable TMap<int32, FMjMeshFrameInverse> MeshFrameInverse;

	// Draws the geoms no authoring element stands behind.
	FMjBakedAssetResolver BakedFallback;
};
