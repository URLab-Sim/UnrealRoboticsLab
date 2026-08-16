// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Entity/MjTwistResolve.h"

#include "mujoco/mujoco.h"

#include "MuJoCo/Entity/MjEntity.h"

namespace
{
	// jnt_axis is a unit vector at compile time; renormalise defensively so a hand-authored axis
	// cannot scale the resolved rate.
	FVector JointAxis(const mjModel* Model, int JointId)
	{
		const mjtNum* Ax = &Model->jnt_axis[3 * JointId];
		return FVector(Ax[0], Ax[1], Ax[2]).GetSafeNormal();
	}
}

namespace MjTwistResolve
{
	// A base twist is realised by the actuators that drive the base degrees of freedom directly. The
	// mapping is read from the model: for each joint-transmission actuator, project the commanded
	// twist onto the joint's own axis -- the linear part for a slide DOF, the angular part for a
	// hinge DOF -- and scale by the transmission gear (actuator length = gear * joint coordinate, so
	// ctrl = gear * desired-joint-rate). This reproduces the UMjTwistController convention where
	// (vx, vy, yaw_rate) drive base slide-x / slide-y / hinge-z: with axis-aligned base joints the
	// dot products collapse back to (vx, vy, yaw_rate).
	//
	// Assumptions, all forced by "model-only" (no mjData, so no live base orientation):
	//   - The twist is expressed in the base frame; jnt_axis is expressed in the joint body's frame.
	//     These coincide at the reference configuration, which is the frame the twist convention
	//     (MuJoCo free joint: world linear, body angular) is authored against. A live-data resolver
	//     would rotate the linear part by the base orientation first; this one deliberately does not.
	//   - Only slide and hinge joint transmissions map to a single scalar setpoint. Free / ball
	//     transmissions and non-joint transmissions (tendon, site, ...) carry no single-axis twist
	//     meaning and are skipped rather than guessed.
	TArray<TPair<int32, double>> ForEntity(const mjModel* Model, const FMjEntity& Entity,
	                                        const FVector& Linear, const FVector& Angular)
	{
		TArray<TPair<int32, double>> Out;
		if (!Model)
		{
			return Out;
		}

		for (int32 ActuatorId : Entity.ActuatorIds)
		{
			if (ActuatorId < 0 || ActuatorId >= Model->nu)
			{
				continue;
			}
			if (Model->actuator_trntype[ActuatorId] != mjTRN_JOINT)
			{
				continue;
			}

			const int32 JointId = Model->actuator_trnid[2 * ActuatorId + 0];
			if (JointId < 0 || JointId >= Model->njnt)
			{
				continue;
			}

			const FVector Axis = JointAxis(Model, JointId);
			double Rate = 0.0;
			switch (Model->jnt_type[JointId])
			{
			case mjJNT_SLIDE:
				Rate = FVector::DotProduct(Linear, Axis);
				break;
			case mjJNT_HINGE:
				Rate = FVector::DotProduct(Angular, Axis);
				break;
			default:
				continue;
			}

			const double Gear = Model->actuator_gear[6 * ActuatorId + 0];
			const double Value = (Gear != 0.0) ? Gear * Rate : Rate;
			Out.Emplace(ActuatorId, Value);
		}

		return Out;
	}
}
