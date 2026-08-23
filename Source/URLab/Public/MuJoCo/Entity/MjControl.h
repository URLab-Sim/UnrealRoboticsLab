// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

/** How a per-entity setpoint reaches d->ctrl. Resolved ONCE, at the pre-step drain (per entity).
 *  Renamed from EMjDrive (Phase 0.1): that name is now the render/primary Drive axis on AMjRenderer
 *  (see MjRenderer.h). This control-drive enum has no references anywhere in the plugin, so the
 *  rename is mechanical and behavior-preserving. */
enum class EMjControlDrive : uint8
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

