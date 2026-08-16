// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

struct mjModel_;
typedef mjModel_ mjModel;
struct FMjEntity;

/**
 * Stateless, model-only resolution of a base twist into per-actuator setpoints. The shadowless wire
 * model has no UMjTwistController to hold state, so this replaces it: given the compiled model and an
 * entity's id slice, it derives the mapping from jnt_type / jnt_axis / actuator_trnid alone.
 */
namespace MjTwistResolve
{
	// Resolve a base twist (world/body per MuJoCo free-joint convention) into setpoints for the
	// entity's actuators. Returns a map actuatorId -> value. Model-only; no components.
	URLAB_API TArray<TPair<int32, double>> ForEntity(const mjModel* Model, const FMjEntity& Entity,
	                                                  const FVector& Linear, const FVector& Angular);
}
