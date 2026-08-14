// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Reading and driving a joint, and reading a tendon.
//
// Same shape as the sensor library and for the same reason: <joint> and
// <freejoint> are separate element classes, <tendon> is a section whose real
// elements are <spatial> and <fixed>, and none of them share a base that could
// carry this. What they do share is that every one of these operations is a
// function of the compiled id and the model -- qpos through jnt_qposadr, qvel
// through jnt_dofadr, tendon length through ten_length -- so none of it needs
// to live on a component.
//
// The single-slot accessors read the joint's FIRST qpos or dof slot. That is
// the whole story for a hinge or a slide and only part of it for a ball or a
// free joint, which is why the state IR carries the full width instead. These
// stay scalar because that is what a Blueprint slider and a details row want.

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"

#include "MjJointRuntime.generated.h"

class UMjNodeComponent;

/**
 * Runtime reads and writes of a `<joint>` or `<freejoint>` element.
 *
 * Reads return 0 when the node is not a joint, is unbound, or no model is
 * compiled. Writes are no-ops in the same cases; they go through the engine so
 * the write lands between steps rather than inside one.
 */
UCLASS()
class URLAB_API UMjJointRuntime : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/** The joint's first position slot, `d->qpos[jnt_qposadr]`. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Joint",
		meta = (DefaultToSelf = "Joint", ScriptMethod))
	static float GetPosition(const UMjNodeComponent* Joint);

	/** Queue a write of the joint's first position slot. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Joint",
		meta = (DefaultToSelf = "Joint", ScriptMethod))
	static void SetPosition(const UMjNodeComponent* Joint, float Position);

	/** The joint's first velocity slot, `d->qvel[jnt_dofadr]`. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Joint",
		meta = (DefaultToSelf = "Joint", ScriptMethod))
	static float GetVelocity(const UMjNodeComponent* Joint);

	/** Queue a write of the joint's first velocity slot. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Joint",
		meta = (DefaultToSelf = "Joint", ScriptMethod))
	static void SetVelocity(const UMjNodeComponent* Joint, float Velocity);

	/** The joint's first acceleration slot, `d->qacc[jnt_dofadr]`. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Joint",
		meta = (DefaultToSelf = "Joint", ScriptMethod))
	static float GetAcceleration(const UMjNodeComponent* Joint);

	/** `[min, max]` from `m->jnt_range`; zero when the joint is unlimited. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Joint",
		meta = (DefaultToSelf = "Joint", ScriptMethod))
	static FVector2D GetJointRange(const UMjNodeComponent* Joint);

	/** The joint's world anchor, `d->xanchor`, in Unreal coordinates (cm). */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Joint",
		meta = (DefaultToSelf = "Joint", ScriptMethod))
	static FVector GetWorldAnchor(const UMjNodeComponent* Joint);

	/** The joint's world axis, `d->xaxis`, as an Unreal unit vector. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Joint",
		meta = (DefaultToSelf = "Joint", ScriptMethod))
	static FVector GetWorldAxis(const UMjNodeComponent* Joint);

	/** True when `Node` is a `<joint>` or `<freejoint>` element. */
	static bool IsJoint(const UMjNodeComponent* Node);
};

/**
 * Runtime reads of a `<spatial>` or `<fixed>` tendon element.
 *
 * Separate library from the joints because a tendon is a different element
 * family, not because the mechanism differs.
 */
UCLASS()
class URLAB_API UMjTendonRuntime : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/** The tendon's current length in metres, `d->ten_length`. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Tendon",
		meta = (DefaultToSelf = "Tendon", ScriptMethod))
	static float GetLength(const UMjNodeComponent* Tendon);

	/** The tendon's current rate of change in metres per second, `d->ten_velocity`. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Tendon",
		meta = (DefaultToSelf = "Tendon", ScriptMethod))
	static float GetVelocity(const UMjNodeComponent* Tendon);

	/** True when `Node` is a `<spatial>` or `<fixed>` tendon element. */
	static bool IsTendon(const UMjNodeComponent* Node);
};
