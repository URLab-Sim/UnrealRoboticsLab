// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Driving an actuator, for all thirteen kinds of them.
//
// The schema gives motor, position, muscle and the rest their own element
// classes with no shared base, and none of them needs one: everything a caller
// does to an actuator at runtime is either a read of the compiled model at the
// actuator's id, or a write into a staging slot the physics worker resolves on
// the next step. Neither depends on which kind it is.
//
// The staging slots are not here and they are not on the element. They live on
// the owning AMjArticulation, sized to the compiled scene's `nu`, because the
// step loop reads every one of them each step and an element that owned its own
// slot would cost the worker a UObject dereference per actuator per step. This
// library is the door onto them, so a caller still spells the operation against
// the actuator it means.
//
// Nothing here caches a model or a data pointer; both are the engine's and are
// fetched per call, because a cached mjModel* outlives its compile exactly once.

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"

#include "MjActuatorRuntime.generated.h"

class AMjArticulation;
class UMjNodeComponent;

/**
 * Runtime control of an `<actuator>` child element.
 *
 * Every entry point takes the actuator as `UMjNodeComponent*` and does nothing
 * (returning zero, or an empty result) when it is not an actuator, is unbound,
 * has no articulation to stage control on, or the simulation has no compiled
 * model. `IsActuator` is the way to tell a refusal from a genuine zero.
 */
UCLASS()
class URLAB_API UMjActuatorRuntime : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	// --- Staged control ---------------------------------------------------- //

	/**
	 * Stage a control value on the internal (UI and Blueprint) slot.
	 *
	 * Staged, not applied: the physics worker resolves the slots into `d->ctrl`
	 * at the top of the step it is about to take, so a write from the game
	 * thread never races the integrator.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Actuator",
		meta = (DefaultToSelf = "Actuator", ScriptMethod))
	static void SetControl(const UMjNodeComponent* Actuator, double Value);

	/**
	 * Stage a control value on the external (ZMQ) slot.
	 *
	 * The bridge, the ZMQ control subscriber and the ROS transports all land
	 * here. Which of the two slots reaches `d->ctrl` is the articulation's
	 * `ControlSource` and not the writer's, so both may be staged at once and
	 * ownership decided separately.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Actuator",
		meta = (DefaultToSelf = "Actuator", ScriptMethod))
	static void SetNetworkControl(const UMjNodeComponent* Actuator, double Value);

	/** Zero both slots. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Actuator",
		meta = (DefaultToSelf = "Actuator", ScriptMethod))
	static void ResetControl(const UMjNodeComponent* Actuator);

	/** The staged value the articulation's control source selects. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Actuator",
		meta = (DefaultToSelf = "Actuator", ScriptMethod))
	static double GetControl(const UMjNodeComponent* Actuator);

	/**
	 * The staged value a named control source selects, ignoring the
	 * articulation's own.
	 *
	 * What a control law wants: it is handed the source for the step it is
	 * computing rather than reading whatever the articulation is set to, so the
	 * two cannot disagree mid-step. Source 0 is the network and anything else is
	 * the UI.
	 */
	static double ResolveDesiredControl(const UMjNodeComponent* Actuator, uint8 Source);

	// --- Compiled-model reads ---------------------------------------------- //

	/** `d->ctrl` as the integrator last saw it, which is not what is staged. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Actuator",
		meta = (DefaultToSelf = "Actuator", ScriptMethod))
	static float GetAppliedControl(const UMjNodeComponent* Actuator);

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Actuator",
		meta = (DefaultToSelf = "Actuator", ScriptMethod))
	static float GetForce(const UMjNodeComponent* Actuator);

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Actuator",
		meta = (DefaultToSelf = "Actuator", ScriptMethod))
	static float GetLength(const UMjNodeComponent* Actuator);

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Actuator",
		meta = (DefaultToSelf = "Actuator", ScriptMethod))
	static float GetVelocity(const UMjNodeComponent* Actuator);

	/** [min, max] from the compiled model; zero when ctrl is unlimited. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Actuator",
		meta = (DefaultToSelf = "Actuator", ScriptMethod))
	static FVector2D GetControlRange(const UMjNodeComponent* Actuator);

	/** The activation state, or 0 for a stateless actuator. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Actuator",
		meta = (DefaultToSelf = "Actuator", ScriptMethod))
	static float GetActivation(const UMjNodeComponent* Actuator);

	/** All six gear entries from the compiled model. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Actuator",
		meta = (DefaultToSelf = "Actuator", ScriptMethod))
	static TArray<float> GetGear(const UMjNodeComponent* Actuator);

	/**
	 * Write up to six gear entries into the live model.
	 *
	 * A runtime effect only. The spec field is the authority and is not
	 * touched, so a recompile reconciles the gear back to what is authored --
	 * which is the difference from the component method this replaces, and is
	 * deliberate: authoring presence from a runtime call is how an element
	 * silently lost its default-class inheritance.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Actuator",
		meta = (DefaultToSelf = "Actuator", ScriptMethod))
	static void SetGear(const UMjNodeComponent* Actuator, const TArray<float>& Gear);

	// --- Classification ---------------------------------------------------- //

	/** True when `Node` is an `<actuator>` child element. */
	static bool IsActuator(const UMjNodeComponent* Node);

	/** The articulation whose slots `Actuator` stages control on, or null. */
	static AMjArticulation* OwningArticulation(const UMjNodeComponent* Actuator);
};
