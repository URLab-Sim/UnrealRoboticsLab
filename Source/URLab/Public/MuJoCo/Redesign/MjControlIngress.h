// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

/**
 * Shadowless command target. Routes control writes into the one FMjControlBuffer under the write
 * lease, keyed by ENTITY NAME + id -- no AMjArticulation needed. ROS / ZMQ / UI ingress all call
 * this. Plain C++ abstract (not a UINTERFACE) so an engine object can implement it without UObject
 * reflection, matching IMjStateConsumer.
 */
class URLAB_API IMjControlIngress
{
public:
	virtual ~IMjControlIngress() = default;

	/** Write a setpoint for one actuator id of an entity, under a lease holder token. */
	virtual void WriteCtrl(FName Entity, int32 ActuatorId, double Value, const FGuid& Who) = 0;

	/**
	 * cmd_vel: resolve a body twist into the entity's joint/actuator setpoints MODEL-ONLY (the
	 * reimplementation of what UMjTwistController computed, for shadowless wire models).
	 */
	virtual void WriteTwist(FName Entity, const FVector& Linear, const FVector& Angular, const FGuid& Who) = 0;
};
