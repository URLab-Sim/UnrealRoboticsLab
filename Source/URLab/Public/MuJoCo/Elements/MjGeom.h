// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// What a <geom> looks like in the editor, and the mesh tooling that hangs off it.
//
// A geom's shape is an attribute, not a class. `Type` is a TOptional<EMjGeomType>
// on the generated base and it changes the instant someone picks a different entry
// in the dropdown, so there is one UMjGeom for all nine shapes rather than one per
// shape. Everything that used to separate a box component from a sphere component
// -- which engine primitive to preview with, how MJCF `size` maps onto a component
// scale, which scale axes the gizmo may move independently -- is one row in a table
// keyed by that attribute, and the table lives in MjGeom.cpp.
//
// Only the capsule needs more than a row. Unreal ships no capsule primitive, so its
// preview is assembled from a shaft cylinder and two sphere caps, and the caps take
// a counter-scale on Z so they stay spherical as the half-length moves away from
// the radius.
//
// `size` is authored with the scale gizmo, and the mapping between a size slot and
// a scale axis is declared once per shape. Both directions come off those rows, so
// what the handle draws and what a drag writes back cannot disagree. A shape that
// cannot hold a non-uniform scale -- a sphere has one radius -- snaps the gizmo
// back on the spot rather than silently discarding a component at write time.
//
// Nothing here asks the compiled model how big a geom is; the two things that do
// read compiled state, the world transform and the world location, index the render
// snapshot at the id this element bound to.

#include "CoreMinimal.h"

#include "MuJoCo/Gen/Elements/Geometry/MjGeom.gen.h"

#include "MjGeom.generated.h"

class UMaterialInterface;
class UStaticMeshComponent;

/**
 * A `<geom>` element, with its editor preview and its mesh tooling.
 *
 * The generated base owns every MJCF attribute; this class owns the things the
 * reflection system has to see that no attribute describes: the preview mesh
 * components, the material a user picked to see a primitive in, and the state of
 * the convex decomposition run on a mesh geom.
 */
UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjGeom : public UMjGeomBase
{
	GENERATED_BODY()

public:
	// --- Preview meshes ---------------------------------------------------- //

	// UPROPERTY so garbage collection tracks these, Transient so nothing tries to
	// save a picture. Both halves matter: editing a property from the Details panel
	// reconstructs the component, and without the UPROPERTY the raw pointers outlive
	// the objects they name, so the next cap placement dereferences freed memory.

	/** The type's primary preview mesh; the shaft cylinder for a capsule. */
	UPROPERTY(Transient)
	TObjectPtr<UStaticMeshComponent> VisualizerMesh;

	/** Capsule cap at positive local Z. Null for every other type. */
	UPROPERTY(Transient)
	TObjectPtr<UStaticMeshComponent> VisualizerCapTop;

	/** Capsule cap at negative local Z. Null for every other type. */
	UPROPERTY(Transient)
	TObjectPtr<UStaticMeshComponent> VisualizerCapBottom;

	/**
	 * The scale `VisualizerMesh` carries in its own right.
	 *
	 * One for an engine primitive: its size is the geom component's scale, and
	 * the part rides along. A mesh geom's asset is different -- the geom's
	 * `size` says nothing about it, while the `<mesh scale>` MuJoCo would have
	 * baked into the vertices and the metre-to-centimetre factor both do -- and
	 * this is where those two land.
	 */
	UPROPERTY(Transient)
	FVector VisualizerScale = FVector::OneVector;

	/**
	 * Material override for the previews.
	 *
	 * An explicit override is the user overruling the spec, so the whole
	 * spec-driven material pass stands aside when one is set.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Geom|Visual")
	TObjectPtr<UMaterialInterface> OverrideMaterial;

	/** The primary preview mesh, or null for a type that has no built-in preview. */
	UStaticMeshComponent* GetVisualizerMesh() const { return VisualizerMesh; }

	/** Show or hide this geom's preview meshes and any child static meshes. */
	void SetGeomVisibility(bool bNewVisibility);

	/** Put `InMaterial` on every built-in preview mesh this geom has. */
	void ApplyOverrideMaterial(UMaterialInterface* InMaterial);

	/**
	 * The `material` this geom names, resolved through its default-class chain.
	 *
	 * Empty when nothing in the chain names one. MuJoCo's humanoid puts
	 * `material="body"` on a default class rather than on the geom, so reading
	 * the geom's own storage answers the wrong question.
	 */
	FString EffectiveMaterialName() const;

	/**
	 * The `<mesh>` this geom names, resolved through its default-class chain.
	 *
	 * Empty when nothing in the chain names one. A menagerie model routinely
	 * writes `<geom class="visual" mesh="link0"/>` with the type on the class,
	 * and just as routinely puts the mesh on the class too.
	 */
	FString EffectiveMeshName() const;

	/**
	 * The colour MuJoCo's own visualiser would draw this geom with.
	 *
	 * Both `rgba` and `material` are defaultable, and in a menagerie model
	 * neither is usually on the geom, so both are resolved through the class
	 * chain before the precedence rule applies.
	 *
	 * That rule is Filament's (`model_renderables.cc GetDefaultMaterial`), not
	 * the classic GL renderer's: start from the geom's `rgba`, and let the
	 * material's take it back whenever the material's is anything other than
	 * (0.5, 0.5, 0.5, 1). Classic decides the opposite way round, so the two
	 * disagree when geom and material are both explicitly non-default; Unreal
	 * is a PBR renderer and Filament is MuJoCo's PBR reference, which is why
	 * this follows Filament.
	 */
	FLinearColor GetEffectiveColor() const;

	/**
	 * Redraw the component's relative scale from the effective `size`.
	 *
	 * A no-op when the type has no preview, or when `size` is too short or
	 * non-positive for the type: a geom waiting on a size it cannot resolve keeps
	 * the scale it has rather than collapsing to zero.
	 *
	 * The scale half of the preview on its own. `SyncPreviewFromSpec` on the
	 * base does this and the pose together, and is what the editor hooks call.
	 */
	void SyncEditorScaleFromSize();

	// --- Size through the scale handle -------------------------------------- //
	//
	// A geom's `size` is authored with the scale gizmo, and the mapping from a
	// size slot to a scale axis is one table in MjScalePolicy.cpp -- shared,
	// because `<site>` carries the same `type` enum and the same `size`, and a
	// sphere is one radius whichever of the two is drawing it. The base class
	// reads that table through the schema, so a geom overrides only what is a
	// geom's alone.

	/** Snap the scale onto the type's axes, then move the capsule's caps. */
	virtual void ConstrainPreviewScale() override;

	// --- Mesh decomposition ------------------------------------------------- //

	/** Resolved name of this geom's mesh asset, once one has been prepared. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Geom")
	FString MeshName;

	/** Collide against the full mesh rather than its convex hull. */
	UPROPERTY(BlueprintReadWrite, Category = "MuJoCo|Geom")
	bool bComplexMeshRequired = false;

	/** CoACD concavity threshold. Lower is more faithful and more hulls. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Geom",
		meta = (ClampMin = "0.01", ClampMax = "1.0"))
	float CoACDThreshold = 0.05f;

	/** True when this geom is one hull of another geom's decomposition. */
	UPROPERTY()
	bool bIsDecomposedHull = false;

	/** True when hull sub-geoms stand in for this geom's own collision. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Geom")
	bool bDisabledByDecomposition = false;

	/** Run CoACD on this geom's mesh and keep the hulls as sibling geoms. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Geom")
	void DecomposeMesh();

	/** Delete the hull sub-geoms and put this geom's own collision back. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Geom")
	void RemoveDecomposition();

	// --- Compiled state ----------------------------------------------------- //

	/**
	 * Move the component onto the geom's simulated world pose.
	 *
	 * Reads the render snapshot at the bound id, so it is a game-thread call and
	 * does nothing until the element has been bound to a compiled model.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
	void UpdateGlobalTransform();

	/** The geom's simulated world location, or the component's own when unbound. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
	FVector GetWorldLocation() const;

	// --- USceneComponent ---------------------------------------------------- //

	virtual void OnRegister() override;

	/** The base's pose and size preview, plus the meshes and the tint. */
	virtual void RefreshPresentation() override;

#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

private:
	/**
	 * Bring the preview in line with the effective type.
	 *
	 * Tears the old preview down and builds the new one when the type has moved,
	 * and is otherwise cheap enough to call on every property edit: it re-attaches
	 * anything a component reconstruction dropped, re-applies the material and
	 * re-places the caps.
	 */
	void RebuildVisualizer();

	/** Destroy every preview mesh this geom currently owns. */
	void DestroyVisualizer();

	/**
	 * Dress the built-in previews in the material the spec asks for.
	 *
	 * One dynamic instance of the plugin's master material per preview part,
	 * carrying the resolved colour, the shading scalars and every texture role
	 * the geom's material fills. The master declares no static switches, so one
	 * instance can express any material without a permutation existing for it.
	 */
	void ApplySpecMaterial();

	/** A preview mesh part, outered to this geom and attached when it can be. */
	UStaticMeshComponent* MakeVisualizerPart(FName PartName, const TCHAR* MeshPath);

	/**
	 * Place the capsule caps at the shaft ends and keep them spherical.
	 *
	 * The caps sit at a constant local +/-Z, which lands them on the moving shaft
	 * ends because the parent's Z scale is what moves both. Their own Z scale
	 * cancels that stretch out again, sign preserved so a mirrored geom stays
	 * mirrored.
	 */
	void UpdateCapTransforms();

	/** The type the current preview was built for; unset before the first build. */
	TOptional<EMjGeomType> BuiltType;

	/** One warning per geom for a degenerate snapshot row, not one per frame. */
	bool bWarnedDegenerateTransform = false;
};
