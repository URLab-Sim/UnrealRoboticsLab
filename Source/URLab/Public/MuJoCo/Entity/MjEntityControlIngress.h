// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

#include "MuJoCo/Entity/MjControlIngress.h"

struct mjModel_;
typedef mjModel_ mjModel;

struct FMjControlBuffer;
struct FMjControlLease;
struct FMjEntity;

/**
 * Concrete shadowless command target. Routes every ingress write (ROS / ZMQ / UI) into the one
 * FMjControlBuffer under the entity write lease, addressed by entity name -- no AMjArticulation, no
 * shadow. A base twist is resolved into per-actuator setpoints from the compiled model alone
 * (MjTwistResolve) and then funnelled through the same lease-gated WriteCtrl path.
 *
 * Holds references only: the buffer, the lease and the entity partition are owned by the engine; the
 * compiled model is borrowed for twist resolution. Not a UObject.
 */
class URLAB_API FMjEntityControlIngress : public IMjControlIngress
{
public:
	FMjEntityControlIngress(const mjModel* InModel, FMjControlBuffer& InBuffer, FMjControlLease& InLease,
	                        const TArray<FMjEntity>& InPartition);

	virtual void WriteCtrl(FName Entity, int32 ActuatorId, double Value, const FGuid& Who) override;

	virtual void WriteTwist(FName Entity, const FVector& Linear, const FVector& Angular,
	                        const FGuid& Who) override;

private:
	const FMjEntity* FindEntity(FName Entity) const;

	const mjModel* Model = nullptr;
	FMjControlBuffer& Buffer;
	FMjControlLease& Lease;
	const TArray<FMjEntity>& Partition;
};
