// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

/**
 * The ONE control store (size nu). The single writer is the engine's pre-step drain. Setpoint
 * PERSISTS across substeps (Touched is NOT cleared each tick), so a set-once value holds across an
 * n-step request; only touched ids reach d->ctrl (lease-gated writers touch only their own ids).
 */
struct URLAB_API FMjControlBuffer
{
	TArray<double> Setpoint;   // size nu
	TBitArray<>    Touched;    // ids a lease holder has written

	void Init(int32 Nu)
	{
		Setpoint.Init(0.0, Nu);
		Touched.Init(false, Nu);
	}
};

/**
 * Keyframe qpos-hold injection, applied FIRST each substep (before the ctrl write pass): write held
 * qpos, ZERO held DoFs' qvel, skip free joints, and SUPPRESS the ctrl write for held entities'
 * actuators. This holds a pose by pinning qpos rather than commanding torque.
 */
struct URLAB_API FMjStateInjection
{
	TArray<double> Qpos;         // size nq (only QposMask-covered entries are meaningful)
	TArray<double> Qvel;         // size nv (zeroed for held DoFs)
	TBitArray<>    QposMask;     // per-qpos (nq): write this qpos entry from Qpos
	TBitArray<>    HoldMask;     // per-DoF (nv): zero this DoF's qvel
	TBitArray<>    SuppressCtrl; // per-actuator (nu): skip this id in the ctrl write pass
};

/**
 * Exclusive per-entity WRITE LEASE. Whoever holds an entity's lease may write its setpoints; the UI
 * grabbing control takes the lease, releasing hands it back to the network. Unclaimed means the
 * default (network) writer is allowed.
 */
struct URLAB_API FMjControlLease
{
	/** Entity name -> current holder token. Absent => unclaimed. */
	TMap<FName, FGuid> Holder;

	bool CanWrite(FName Entity, const FGuid& Who) const;
	bool Claim(FName Entity, const FGuid& Who);
	void Release(FName Entity, const FGuid& Who);
};

/**
 * Stable identity tokens for the two default control writers. The UI takes an entity's lease to
 * override the network; until it does, both are unclaimed and either is allowed to write.
 */
namespace MjControlWho
{
	URLAB_API const FGuid& Network();
	URLAB_API const FGuid& UI();
}
