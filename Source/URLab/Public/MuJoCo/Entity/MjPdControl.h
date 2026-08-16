// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

struct mjModel_;
typedef mjModel_ mjModel;
struct mjData_;
typedef mjData_ mjData;

/**
 * A PD control law's per-actuator gains. This is entity behaviour, data-ified: the gains live with
 * the entity and the drain runs the law for any actuator that has them. An actuator with no gains is
 * driven directly (its setpoint is the ctrl).
 */
struct FMjPdGains
{
	float Kp = 0.f;
	float Kv = 0.f;
	float TorqueLimit = 0.f;
};

namespace MjPdControl
{
	/**
	 * The PD law: torque = clamp(Kp*(target - pos) - Kv*vel, +/- limit), with the target first
	 * clamped to the driven joint's range. Reads the joint's qpos/qvel from the actuator's
	 * transmission, so it needs no precomputed binding. Returns the value to write to d->ctrl.
	 */
	URLAB_API double Compute(const mjModel* Model, const mjData* Data, int32 ActuatorId,
	                         double Setpoint, const FMjPdGains& Gains);
}
