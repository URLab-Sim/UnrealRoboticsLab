// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// URLab pseudo-field side of FMjOptionGenerated. The codegen emits the
// mjOption mirror struct + its ApplyToSpec / ApplyOverridesToModel
// bodies (in MjOptionGenerated.cpp) and ends each with a call to
// ApplyMjOptionExtras for the URLab-specific bits the spec/model layout
// doesn't carry (MemoryMB, MultiCCD bitflag, Sleep bitflag + tolerance).
// Edit this file directly — it is NOT codegen-managed.

#include "MuJoCo/Generated/MjOptionGenerated.h"
#include <mujoco/mujoco.h>

// Bit positions in mjOption.enableflags. Match mjENBL_MULTICCD / mjENBL_SLEEP
// from mjmodel.h; hardcoded to avoid pulling mjtEnableBit into the header.
static constexpr int MJ_ENBL_MULTICCD = 1 << 4;
static constexpr int MJ_ENBL_SLEEP = 1 << 5;

void ApplyMjOptionExtras(mjOption* Opt, const FMjOptionGenerated& Self,
	mjSpec* Spec, mjModel* /*Model*/)
{
	if (!Opt)
		return;

	if (Spec && Self.bOverride_MemoryMB)
	{
		Spec->memory = static_cast<mjtSize>(Self.MemoryMB) * 1024 * 1024;
	}

	if (Self.bEnableMultiCCD)
		Opt->enableflags |= MJ_ENBL_MULTICCD;
	else
		Opt->enableflags &= ~MJ_ENBL_MULTICCD;

	if (Self.bEnableSleep)
	{
		Opt->enableflags |= MJ_ENBL_SLEEP;
		Opt->sleep_tolerance = Self.SleepTolerance;
	}
	else
	{
		Opt->enableflags &= ~MJ_ENBL_SLEEP;
	}
}
