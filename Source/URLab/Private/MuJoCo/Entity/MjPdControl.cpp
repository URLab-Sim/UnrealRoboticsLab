// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Entity/MjPdControl.h"

#include "mujoco/mujoco.h"

double MjPdControl::Compute(const mjModel* Model, const mjData* Data, int32 ActuatorId,
                            double Setpoint, const FMjPdGains& Gains)
{
	if (Model == nullptr || Data == nullptr || ActuatorId < 0 || ActuatorId >= Model->nu)
	{
		return Setpoint;
	}

	// The transmission's target joint carries the state the law tracks. Position/general actuators
	// on a hinge or slide joint have a single-DoF joint here; anything else has no scalar PD meaning,
	// so fall back to passing the setpoint through.
	const int32 JointId = Model->actuator_trnid[ActuatorId * 2];
	if (JointId < 0 || JointId >= Model->njnt)
	{
		return Setpoint;
	}

	double Target = Setpoint;
	if (Model->jnt_limited[JointId])
	{
		Target = FMath::Clamp(Target, Model->jnt_range[JointId * 2], Model->jnt_range[JointId * 2 + 1]);
	}

	const double Pos = Data->qpos[Model->jnt_qposadr[JointId]];
	const double Vel = Data->qvel[Model->jnt_dofadr[JointId]];
	const double Limit = Gains.TorqueLimit;
	return FMath::Clamp(Gains.Kp * (Target - Pos) - Gains.Kv * Vel, -Limit, Limit);
}
