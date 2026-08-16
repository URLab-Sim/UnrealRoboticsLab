// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Entity/MjEntityControlIngress.h"

#include "MuJoCo/Entity/MjControl.h"
#include "MuJoCo/Entity/MjEntity.h"
#include "MuJoCo/Entity/MjTwistResolve.h"

FMjEntityControlIngress::FMjEntityControlIngress(const mjModel* InModel, FMjControlBuffer& InBuffer,
                                                 FMjControlLease& InLease,
                                                 const TArray<FMjEntity>& InPartition)
	: Model(InModel)
	, Buffer(InBuffer)
	, Lease(InLease)
	, Partition(InPartition)
{
}

const FGuid& MjControlWho::Network()
{
	static const FGuid Token(0x4E455457, 0x00000000, 0x00000000, 0x00000000);
	return Token;
}

const FGuid& MjControlWho::UI()
{
	static const FGuid Token(0x55495F55, 0x49000000, 0x00000000, 0x00000000);
	return Token;
}

void FMjEntityControlIngress::WriteCtrl(FName Entity, int32 ActuatorId, double Value, const FGuid& Who)
{
	if (!Lease.CanWrite(Entity, Who))
	{
		return;
	}
	if (ActuatorId < 0 || ActuatorId >= Buffer.Setpoint.Num())
	{
		return;
	}

	Buffer.Setpoint[ActuatorId] = Value;
	Buffer.Touched[ActuatorId] = true;
}

void FMjEntityControlIngress::WriteTwist(FName Entity, const FVector& Linear, const FVector& Angular,
                                         const FGuid& Who)
{
	const FMjEntity* E = FindEntity(Entity);
	if (!E)
	{
		return;
	}

	// Model-only resolution of the base twist, then route each setpoint through the lease-gated write
	// path so the twist obeys the same lease as a direct actuator write.
	for (const TPair<int32, double>& Setpoint : MjTwistResolve::ForEntity(Model, *E, Linear, Angular))
	{
		WriteCtrl(Entity, Setpoint.Key, Setpoint.Value, Who);
	}
}

const FMjEntity* FMjEntityControlIngress::FindEntity(FName Entity) const
{
	for (const FMjEntity& Candidate : Partition)
	{
		if (Candidate.Name == Entity)
		{
			return &Candidate;
		}
	}
	return nullptr;
}
