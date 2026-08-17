// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

/** How a per-entity setpoint reaches d->ctrl. Resolved ONCE, at the pre-step drain (per entity). */
enum class EMjDrive : uint8
{
	Direct,     // write the setpoint straight to d->ctrl
	Controller  // transform the setpoint through the entity's control law (PD, ...)
};

/**
 * The ONE control store (size nu). The single writer is the engine's pre-step drain. Setpoint
 * PERSISTS across substeps (Touched is NOT cleared each tick), so a set-once value holds across an
 * n-step request; only touched ids reach d->ctrl.
 */
struct URLAB_API FMjControlBuffer
{
	TArray<double> Setpoint;   // size nu
	TBitArray<>    Touched;    // ids a writer has staged

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
