// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"

#include "MuJoCo/Entity/MjEntityHandles.h"
#include "MuJoCo/Entity/MjEntityPickers.h"

#include "MjEntityBlueprintLibrary.generated.h"

class AMjEntity;

/**
 * The Blueprint door onto the entity API.
 *
 * The C++ face (IMjEntityApi / AMjEntity) returns the frozen plain-struct handles, which Blueprint
 * cannot see. This library mirrors the same two steps for a graph: resolve a part by name once to a
 * reflected handle, then read or drive it. A resolve that misses returns a handle with Id == -1; the
 * reads answer zero and the write does nothing, so a bad name never touches the simulation.
 */
UCLASS()
class URLAB_API UMjEntityBlueprintLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	// --- Resolve a part by name --------------------------------------------- //

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Entity",
		meta = (DefaultToSelf = "Entity", ScriptMethod))
	static FMjJointHandle Joint(const AMjEntity* Entity, FName Name);

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Entity",
		meta = (DefaultToSelf = "Entity", ScriptMethod))
	static FMjActuatorHandle Actuator(const AMjEntity* Entity, FName Name);

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Entity",
		meta = (DefaultToSelf = "Entity", ScriptMethod))
	static FMjGeomHandle Geom(const AMjEntity* Entity, FName Name);

	// --- Resolve an authored picker ----------------------------------------- //

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Entity", meta = (ScriptMethod))
	static FMjJointHandle ResolveJoint(const FMjJointPicker& Picker);

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Entity", meta = (ScriptMethod))
	static FMjActuatorHandle ResolveActuator(const FMjActuatorPicker& Picker);

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Entity", meta = (ScriptMethod))
	static FMjGeomHandle ResolveGeom(const FMjGeomPicker& Picker);

	// --- Drive a resolved handle -------------------------------------------- //

	/** The joint's position from the latest published step, or 0 for an unresolved handle. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Entity", meta = (ScriptMethod))
	static float JointPos(const FMjJointHandle& Joint);

	/** The joint's velocity from the latest published step, or 0 for an unresolved handle. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Entity", meta = (ScriptMethod))
	static float JointVel(const FMjJointHandle& Joint);

	/** Stage a control setpoint on the actuator (lands in the engine's control buffer under the UI lease). */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Entity", meta = (ScriptMethod))
	static void SetCtrl(const FMjActuatorHandle& Actuator, double Value);

	/** True when the handle resolved to a compiled id. */
	UFUNCTION(BlueprintPure, Category = "MuJoCo|Entity", meta = (ScriptMethod))
	static bool IsJointValid(const FMjJointHandle& Joint) { return Joint.Id >= 0; }

	UFUNCTION(BlueprintPure, Category = "MuJoCo|Entity", meta = (ScriptMethod))
	static bool IsActuatorValid(const FMjActuatorHandle& Actuator) { return Actuator.Id >= 0; }

	UFUNCTION(BlueprintPure, Category = "MuJoCo|Entity", meta = (ScriptMethod))
	static bool IsGeomValid(const FMjGeomHandle& Geom) { return Geom.Id >= 0; }
};
