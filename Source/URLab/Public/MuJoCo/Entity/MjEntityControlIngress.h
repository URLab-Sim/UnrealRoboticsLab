// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

#include "MuJoCo/Entity/MjControlIngress.h"

struct mjModel_;
typedef mjModel_ mjModel;

struct FMjControlBuffer;
struct FMjEntity;

/**
 * Concrete shadowless command target. Routes every ingress write (ROS / ZMQ / UI) into the one
 * FMjControlBuffer, addressed by entity name -- no AMjArticulation, no shadow. A base twist is
 * resolved into per-actuator setpoints from the compiled model alone (MjTwistResolve) and then
 * funnelled through the same WriteCtrl path.
 *
 * Holds references only: the buffer and the entity partition are owned by the engine; the compiled
 * model is borrowed for twist resolution. Not a UObject.
 */
class URLAB_API FMjEntityControlIngress : public IMjControlIngress
{
public:
	FMjEntityControlIngress(const mjModel* InModel, FMjControlBuffer& InBuffer,
	                        const TArray<FMjEntity>& InPartition);

	virtual void WriteCtrl(FName Entity, int32 ActuatorId, double Value) override;

	virtual void WriteTwist(FName Entity, const FVector& Linear, const FVector& Angular) override;

private:
	const FMjEntity* FindEntity(FName Entity) const;

	const mjModel* Model = nullptr;
	FMjControlBuffer& Buffer;
	const TArray<FMjEntity>& Partition;
};
