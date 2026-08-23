// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Drawing a MuJoCo <skin>, bone-driven, with no per-vertex stream.
//
// A skin is linear-blend skinning: every render vertex is reconstructed each
// frame from a handful of BONE-BODY transforms (d->xpos/xquat[skin_bonebodyid])
// plus the static bind pose and weights that live in the compiled mjModel
// (mjv_updateActiveSkin, engine_vis_visualize.c:3245-3369). Because those bone
// bodies are already carried on the per-body transform bus (§8.1, Phase 2.1), a
// mirror rebuilds the deformed surface locally from transforms it already has:
// zero vertex bytes cross the wire, exactly the flex story (UMjFlexcomp).
//
// Unlike a flexcomp, a skin has no authored level component and no child static
// mesh -- its geometry (skin_vert / skin_face / skin_texcoord) lives entirely in
// the compiled model. So this component is created by AMjRenderer (one per skin),
// builds its UDynamicMeshComponent directly from those model arrays on first
// drive, and is refreshed each streamed frame through UpdateFromBodyTransforms.
// The surface is a UDynamicMeshComponent (like flex) -- NOT a runtime
// USkeletalMesh: CPU LBS writes vertex positions and the component auto-derives
// its own normals/tangents.

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"

#include "MjSkincomp.generated.h"

class UDynamicMeshComponent;

struct mjModel_;
typedef mjModel_ mjModel;

/**
 * A single MuJoCo skin, with the deformable surface it drives.
 *
 * Created and driven by AMjRenderer; there is no MJCF-authored counterpart in
 * the level (that is the authoring-only UMjSkin spec component). The surface is
 * built from the model's skin_* arrays the first time UpdateFromBodyTransforms
 * runs, and refreshed every frame after that.
 */
UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjSkincomp : public USceneComponent
{
	GENERATED_BODY()

public:
	UMjSkincomp();

	/**
	 * The surface the skin is drawn through. Owned here (a UPROPERTY) so nothing
	 * but this reference keeps it alive across a GC that lands between two ticks.
	 */
	UPROPERTY(Transient, VisibleAnywhere, Category = "MuJoCo|Skin")
	TObjectPtr<UDynamicMeshComponent> DynamicMesh;

	/** Which compiled skin this component draws. Set once, at creation. */
	void SetSkinId(int32 InSkinId) { SkinId = InSkinId; }

	/**
	 * Rebuild this frame's skinned surface from the streamed per-body transforms
	 * + the static model, replaying MuJoCo's mjv_updateActiveSkin
	 * (engine_vis_visualize.c:3245-3369): per bone, rotate/translate the bind
	 * pose into the current body frame, blend into each of its vertices by weight.
	 * Bxpos is 3*NBody, Bxquat 4*NBody (wxyz), MuJoCo world frame; SceneOrigin is
	 * the renderer's world offset (added like every other geom). Builds the
	 * surface on first call. Zero per-vertex bytes cross the wire.
	 */
	void UpdateFromBodyTransforms(const mjModel& Model, const double* Bxpos,
		const double* Bxquat, int32 NBody, const FVector& SceneOrigin);

	/** Takes the surface with it rather than orphaning it on the actor. */
	virtual void OnComponentDestroyed(bool bDestroyingHierarchy) override;

private:
	/** Point this component at its skin and build the surface once. */
	bool EnsureSkin(const mjModel& Model);

	/** Build the UDynamicMeshComponent from skin_vert / skin_face / skin_texcoord. */
	void CreateProceduralMesh(const mjModel& Model);

	/** Drop the surface and reset every id resolved against a model that is gone. */
	void ReleaseProceduralMesh();

	/** The compiled skin and the extents of its blocks in the shared skin arrays. */
	int32 SkinId = INDEX_NONE;
	int32 VertAdr = 0;
	int32 VertNum = 0;
	int32 TexAdr = INDEX_NONE;
	int32 FaceAdr = 0;
	int32 FaceNum = 0;
	int32 BoneAdr = 0;
	int32 BoneNum = 0;

	/**
	 * Whether the surface has been attempted for the current skin. Stops a skin
	 * that fails to build (no verts/faces) from re-walking the arrays every frame.
	 */
	bool bSurfaceAttempted = false;
};
