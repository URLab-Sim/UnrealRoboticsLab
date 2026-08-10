// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// The presentation half of <body>.
//
// Every schema attribute a body has lives on UMjBodyBase as a TOptional; nothing
// here is authoring data. What is here is the state a body needs to be seen and
// driven at runtime, which the reflection system has to know about and a
// function library therefore cannot hold: the Quick Convert pivot correction and
// the mesh offset it needs, and the two one-shot warning latches that keep a bad
// physics frame from filling the log at frame rate.
//
// The runtime accessors are members rather than library statics for the same
// reason as the state: they sit next to the state they read, and <body> is a
// single element class with no sibling variants to make a library worthwhile.
// None of them touch the spec. A compiled body's pose, velocity, applied
// wrench and sleep state are all functions of the id this element bound to and
// the model the engine holds, so they index those and nothing else.

#include "CoreMinimal.h"

#include "MuJoCo/Core/MjTypes.h"
#include "MuJoCo/Gen/Elements/Bodies/MjBody.gen.h"

#include "MjBody.generated.h"

struct FMjRenderSnapshot;

UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjBody : public UMjBodyBase
{
	GENERATED_BODY()

public:
	UMjBody();

	/**
	 * This body's Unreal mesh was authored first and converted into MJCF.
	 *
	 * A converted body's collision shape is centred on the mesh bounds rather
	 * than on the component pivot the artist placed, so the pose MuJoCo reports
	 * is the bounds centre and the component has to be set back by the offset
	 * between the two. Set by the Quick Convert component, which is the only
	 * thing that can know it.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Body")
	bool bIsQuickConverted = false;

	// --- Visual update ----------------------------------------------------- //

	/**
	 * Drive the component's world transform from one physics frame.
	 *
	 * Called by AAMjManager rather than from this component's tick, so every
	 * body in a single Unreal frame reads the same snapshot and the robot never
	 * renders half a step ahead of itself. A mocap body is excluded: it is the
	 * source of its pose, not a consumer of it.
	 */
	void ApplyRenderState(const FMjRenderSnapshot& Snap);

	/**
	 * Recompute the pivot correction from the meshes currently under this body.
	 *
	 * The offset is the local-space centre of the first child static mesh's
	 * collision bounds, so it changes only when the mesh does. Run at BeginPlay;
	 * call it again after swapping a mesh under a converted body.
	 */
	void RefreshMeshPivotOffset();

	// --- World state ------------------------------------------------------- //

	/** World position from `d->xpos`, in Unreal coordinates (cm). */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
	FVector GetWorldPosition() const;

	/** World rotation from `d->xquat`, in Unreal convention. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
	FQuat GetWorldRotation() const;

	/**
	 * Spatial velocity from `d->cvel`, in cm/s and deg/s.
	 *
	 * Read through the render snapshot rather than live data: a velocity is
	 * sampled far more often than it is acted on, and the snapshot is the one
	 * view of mjData a game-thread caller can read without racing the step.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
	FMuJoCoSpatialVelocity GetSpatialVelocity() const;

	// --- Applied force ----------------------------------------------------- //

	/**
	 * Queue an external wrench on this body for the next step.
	 *
	 * Force in newtons, torque in newton-metres, both in Unreal axes. It lands
	 * in `xfrc_applied`, which MuJoCo does not clear between steps but the
	 * engine's drain overwrites per body per frame: one call is one step's worth
	 * of push, so a sustained force is a call every tick.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
	void ApplyForce(FVector Force, FVector Torque);

	/** Zero this body's `xfrc_applied` on the next drain. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
	void ClearForce();

	// --- Sleep ------------------------------------------------------------- //

	/**
	 * Whether this body's kinematic tree is being stepped.
	 *
	 * Awake is the safe answer when there is nothing to ask: a body that never
	 * bound, or a model compiled with sleep disabled, is stepped every frame.
	 */
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Sleep")
	bool IsAwake() const;

	/** Wake this body's kinematic tree. No-op when unbound or sleep is off. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Sleep")
	void Wake();

	/**
	 * Put this body's kinematic tree to sleep immediately.
	 *
	 * The step skips the tree until something wakes it: an explicit Wake, or an
	 * impulse above the sleep tolerance.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Sleep")
	void PutToSleep();

	// --- UActorComponent --------------------------------------------------- //

	/** Pushes a mocap body's authored transform into the engine. */
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction) override;

	virtual void DescribeState(const mjModel* m, mjData* d, FMjArticulationState& Out) const override;

protected:
	virtual void BeginPlay() override;

private:
	/**
	 * Component pivot to collision-bounds centre, in this body's local frame.
	 *
	 * Only read when bIsQuickConverted; a body the reader built has its pivot
	 * where MJCF put it and needs no correction.
	 */
	FVector m_MeshPivotOffset = FVector::ZeroVector;

	// Both latches exist because ApplyRenderState runs once per body per frame:
	// an unlatched warning about a bad frame is a warning about every frame
	// after it, which buries the first one. A degenerate transform and an
	// out-of-range id are separate faults with separate fixes, so they latch
	// separately.

	bool m_bWarnedDegenerateXform = false;
	bool m_bWarnedSnapshotRange = false;
};
