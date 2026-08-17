// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "MuJoCo/Entity/MjEntity.h"
#include "MuJoCo/Entity/MjEntityApi.h"

#include "MjEntityActor.generated.h"

class UMjEntityLogicComponent;
class UMjGeom;

/** Fired once whenever the engine resets the simulation to its rest / keyframe state. */
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnMjSimulationReset);

/** Fired for a contact touching one of this entity's geoms: self geom, other geom, world contact point. */
DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FOnMjCollision, UMjGeom*, SelfGeom, UMjGeom*, OtherGeom, FVector, ContactPos);

/**
 * The runtime face of one FMjEntity: a name bound to the compiled model, and the door onto the
 * entity's parts.
 *
 * A thin actor and nothing more. It owns no meshes and no per-joint components -- the compiled model
 * already holds every id it addresses -- it only carries the entity name and answers Joint / Actuator
 * / Geom by resolving a member name against that entity's id slice. The name is the identity the
 * control lease and the Python wire key on, so a handle it hands out can route a write back to the
 * right entity without the actor being anything heavier.
 *
 * Implements the frozen IMjEntityApi (a plain C++ abstract, not a UINTERFACE), so a scene-wide caller
 * scripts against the interface while a Blueprint caller reaches the same methods through the actor.
 */
UCLASS()
class URLAB_API AMjEntity : public AActor, public IMjEntityApi
{
	GENERATED_BODY()

public:
	AMjEntity();

	/** Bind this face to an entity by its stable name (the compiled participant-prefix stem). */
	void SetEntityName(FName InName) { EntityName = InName; }

	/**
	 * The entity name a handle needs to route a control write. The frozen FMjActuator handle carries
	 * only its owning actor, so SetCtrl reads the name back from here to address the control lease.
	 */
	FName GetEntityName() const { return EntityName; }

	// IMjEntityApi: resolve a member name once to a handle carrying its id and this actor.
	virtual FMjJoint    Joint(FName Name) const override;
	virtual FMjActuator Actuator(FName Name) const override;
	virtual FMjGeom     Geom(FName Name) const override;

	// --- Authored logic hosting --------------------------------------------- //

	/**
	 * Add a logic host of the given class, stamp it with this entity's name, and register it. The
	 * build-time handoff re-instantiates each authored component this way, then copies the template's
	 * properties over the returned instance.
	 */
	UMjEntityLogicComponent* AddLogicComponent(TSubclassOf<UMjEntityLogicComponent> LogicClass);

	/** Every logic host currently riding this entity. */
	void GetLogicComponents(TArray<UMjEntityLogicComponent*>& Out) const;

	// --- Events (re-homed from AMjArticulation) ----------------------------- //

	UPROPERTY(BlueprintAssignable, Category = "MuJoCo|Events")
	FOnMjSimulationReset OnSimulationReset;

	UPROPERTY(BlueprintAssignable, Category = "MuJoCo|Events")
	FOnMjCollision OnCollision;

	// --- Per-entity debug overlay ------------------------------------------- //

	/** This entity's per-instance debug-draw intent (unioned with the manager's global toggles). */
	const FMjEntityDrawFlags& GetOverlayFlags() const { return OverlayFlags; }
	void SetOverlayFlags(const FMjEntityDrawFlags& InFlags) { OverlayFlags = InFlags; }

	bool GetDrawDebugCollision() const { return OverlayFlags.bDrawDebugCollision; }
	bool GetDrawDebugJoints() const { return OverlayFlags.bDrawDebugJoints; }
	bool GetDrawDebugSites() const { return OverlayFlags.bDrawDebugSites; }

private:
	/** Stable public name of the entity this face addresses. */
	UPROPERTY(VisibleAnywhere, Category = "MuJoCo|Entity")
	FName EntityName;

	/** Authored per-instance debug-draw intent, carried onto the partition record at handoff. */
	FMjEntityDrawFlags OverlayFlags;
};
