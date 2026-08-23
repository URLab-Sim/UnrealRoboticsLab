// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Drawing the soft body a <flexcomp> turns into.
//
// A flexcomp is a macro rather than a model object. MuJoCo expands it at
// compile time into a flex plus a body per unpinned vertex, and none of those
// exist in the spec tree, so the element itself compiles into no mjtObj
// family and is never given an id. The flex it produced is found by name in the
// compiled model's flex table instead, which is the case the binding's glob
// search exists for.
//
// The subclass exists for the two pieces of per-instance state that the
// rendering needs and that no function library could hold: the component the
// surface is drawn through, which has to be a UPROPERTY or nothing references
// it and the collector takes it mid-frame, and the weld map relating Unreal's
// per-face split vertices to MuJoCo's welded flex vertices.
//
// Every MJCF attribute of <flexcomp> belongs to the generated base; nothing
// here declares one. The base is checked in and depends on nothing but the
// engine, so it is included unconditionally: a UCLASS behind a directive UHT
// does not evaluate would be reflected and then absent from the translation
// unit, which is worse than not compiling.

#include "CoreMinimal.h"

#include "MuJoCo/Gen/Elements/Deformable/MjFlexcomp.gen.h"

#include "MjFlexcomp.generated.h"

class UDynamicMeshComponent;
class UMjPhysicsEngine;
class UStaticMeshComponent;

struct mjModel_;
typedef mjModel_ mjModel;

/**
 * A `<flexcomp>` element, with the deformable surface it drives.
 *
 * The surface is built from the child static mesh the first time the flex
 * resolves against a compiled model, and driven from the render snapshot every
 * tick after that. A flexcomp with no child static mesh -- a grid, a box, a
 * rope -- simulates exactly as before and simply has nothing to draw.
 */
UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjFlexcomp : public UMjFlexcompBase
{
	GENERATED_BODY()

public:
	UMjFlexcomp();

	/**
	 * The surface the deformable is drawn through.
	 *
	 * Created on the owning actor, so nothing but this reference keeps it alive
	 * through a garbage collection that lands between two ticks.
	 */
	UPROPERTY(Transient, VisibleAnywhere, Category = "MuJoCo|Flexcomp")
	TObjectPtr<UDynamicMeshComponent> DynamicMesh;

	/** Resolves the flex if it has to, then writes this frame's vertices. */
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	/** Takes the surface with it, rather than orphaning it on the actor. */
	virtual void OnComponentDestroyed(bool bDestroyingHierarchy) override;

	/**
	 * Drive the surface on a mirror (Drive=stream/push) that has no local
	 * mjData/engine, deriving this frame's flex vertices from the streamed
	 * per-body transforms + the static model -- a local replay of MuJoCo's
	 * mj_flex (engine_core_smooth.c:548-608), so zero per-vertex bytes cross
	 * the wire. Bxpos is 3*NBody, Bxquat 4*NBody (wxyz), MuJoCo world frame.
	 * Resolves + builds the surface on first call exactly like the producer
	 * path, then feeds the same UDynamicMeshComponent writeback. The producer
	 * (sim) still uses UpdateProceduralMesh's snapshot fast path.
	 */
	void UpdateFromBodyTransforms(const mjModel& Model, const double* Bxpos,
		const double* Bxquat, int32 NBody, const FVector& WorldOffset = FVector::ZeroVector);

	/**
	 * Mark this element renderer-created for a mirror (Drive=stream/push): draw
	 * compiled flex `InFlexId`, building the surface from the model's own flex
	 * topology (flex_elem / flex_shell) at the first streamed frame -- there is
	 * no authored child static mesh and no name to resolve, since the mirror
	 * booted a level with no flex content (AMjRenderer::BuildFlexcomps). Call
	 * once, after NewObject and before RegisterComponent; also disables the
	 * self-tick, because the renderer drives this element (like UMjSkincomp).
	 */
	void SetMirrorFlex(int32 InFlexId);

	/** The compiled flex a renderer-created element draws; INDEX_NONE if authored. */
	int32 GetMirrorFlexId() const { return MirrorFlexId; }

private:
	/** The child static mesh both the surface and the weld map are built from. */
	UStaticMeshComponent* FindSourceMesh() const;

	/**
	 * Point this element at the flex the compiler produced for it, building the
	 * surface the first time it lands. False while there is nothing to draw.
	 */
	bool EnsureFlex(const mjModel& Model);

	/** Fill `RawToWelded` from the source mesh. False when there is no mesh. */
	bool BuildWeldMap();

	void CreateProceduralMesh();
	void UpdateProceduralMesh(UMjPhysicsEngine& Engine);

	/**
	 * Build the surface for a renderer-created mirror element from the model's
	 * own flex topology -- elements for a 2D flex, shell fragments for a 3D one
	 * -- with this frame's reconstructed vertices (UE world space) as the
	 * initial pose, so the static shading normals are computed on real geometry.
	 * The model's topology IS the welded topology, so the weld map is the
	 * identity. The UMjSkincomp build pattern, replacing the authored path's
	 * child-static-mesh build.
	 */
	void BuildModelSurface(const mjModel& Model, const TArray<FVector>& WorldPositions);

	/**
	 * Shared writeback: take this frame's flex vertices in UE world space (one
	 * per welded MuJoCo flex vertex, indexed [0, FlexVertNum)), transform them
	 * into the mesh's local space and push them through the dynamic mesh. Both
	 * the producer snapshot path and the mirror transform path funnel here.
	 */
	void ApplyFlexWorldPositions(const TArray<FVector>& WorldPositions);

	/**
	 * Frame the mirror transform path last wrote this surface (0 = never). Its
	 * one job is to mark this element as mirror-driven: once non-zero,
	 * TickComponent keeps the surface when there is no local engine instead of
	 * releasing it, so the renderer-driven update and this component's own tick
	 * never fight over tick order.
	 */
	uint64 LastExternalDriveFrame = 0;

	/** Drop the surface and every id resolved against a model that is gone. */
	void ReleaseProceduralMesh();

	/**
	 * Raw Unreal vertex index to welded MuJoCo flex vertex index.
	 *
	 * Read every tick as `flexvert_xpos[RawToWelded[i]]`, which holds because
	 * the weld order below is the order the mesh was written to MuJoCo in: the
	 * flex's vertex n is this map's welded vertex n.
	 */
	TArray<int32> RawToWelded;

	/** Vertices in the Unreal mesh, before welding. */
	int32 NumRenderVerts = 0;

	/** The compiled flex and the extent of its block of `flexvert_xpos`. */
	int32 FlexId = INDEX_NONE;
	int32 FlexVertAdr = 0;
	int32 FlexVertNum = 0;

	/**
	 * Renderer-assigned flex index for a mirror-created element (INDEX_NONE for
	 * an authored one). Unlike FlexId it survives ReleaseProceduralMesh, so a
	 * model reload re-resolves against the same compiled slot.
	 */
	int32 MirrorFlexId = INDEX_NONE;

	/**
	 * The name `FlexId` was found under, and the check that it still means the
	 * same flex: a recompile renumbers without this element hearing about it.
	 */
	FString ResolvedFlexName;

	/**
	 * Whether the surface has been attempted for the current flex.
	 *
	 * A flexcomp with no static mesh under it would otherwise re-walk its
	 * children looking for one on every tick, forever.
	 */
	bool bSurfaceAttempted = false;
};
