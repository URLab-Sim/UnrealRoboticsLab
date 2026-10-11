// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Core/MjSceneOptions.h"

#include "MuJoCo/Gen/Elements/Options/MjFlag.gen.h"
#include "MuJoCo/Gen/Elements/Options/MjOption.gen.h"

namespace
{
template <typename TDest, typename TSrc>
void Assign(TDest& Dest, const TOptional<TSrc>& Source)
{
	if (Source.IsSet())
	{
		Dest = static_cast<TDest>(*Source);
	}
}

// mjOption is MuJoCo's own struct, so the authored components go across
// verbatim -- no frame conversion, which is why this takes the spec type
// rather than an FVector.
void AssignVector(mjtNum* Dest, const TOptional<FMjDirection3>& Source)
{
	if (Source.IsSet())
	{
		Dest[0] = Source->X;
		Dest[1] = Source->Y;
		Dest[2] = Source->Z;
	}
}

void AssignSequence(mjtNum* Dest, int32 Capacity, const TOptional<TArray<double>>& Source)
{
	if (!Source.IsSet())
	{
		return;
	}
	const int32 Count = FMath::Min(Capacity, Source->Num());
	for (int32 Index = 0; Index < Count; ++Index)
	{
		Dest[Index] = (*Source)[Index];
	}
}

/** A bit MuJoCo holds in `enableflags`: set when the flag says enable. */
void AssignEnableBit(int& Flags, int Bit, const TOptional<EMjEnable>& Source)
{
	if (!Source.IsSet())
	{
		return;
	}
	if (*Source == EMjEnable::enable)
	{
		Flags |= Bit;
	}
	else
	{
		Flags &= ~Bit;
	}
}

/** A bit MuJoCo holds in `disableflags`, so the sense is inverted. */
void AssignDisableBit(int& Flags, int Bit, const TOptional<EMjEnable>& Source)
{
	if (!Source.IsSet())
	{
		return;
	}
	if (*Source == EMjEnable::enable)
	{
		Flags &= ~Bit;
	}
	else
	{
		Flags |= Bit;
	}
}
} // namespace

void MjApplyOption(const UMjOption* Option, const UMjFlag* Flags, mjOption& Out)
{
	if (Option != nullptr)
	{
		Assign(Out.timestep, Option->Timestep);
		Assign(Out.impratio, Option->Impratio);
		Assign(Out.tolerance, Option->Tolerance);
		Assign(Out.ls_tolerance, Option->LsTolerance);
		Assign(Out.noslip_tolerance, Option->NoslipTolerance);
		Assign(Out.ccd_tolerance, Option->CcdTolerance);
		Assign(Out.sleep_tolerance, Option->SleepTolerance);
		AssignVector(Out.gravity, Option->Gravity);
		AssignVector(Out.wind, Option->Wind);
		AssignVector(Out.magnetic, Option->Magnetic);
		Assign(Out.density, Option->Density);
		Assign(Out.viscosity, Option->Viscosity);
		Assign(Out.o_margin, Option->OMargin);
		AssignSequence(Out.o_solref, mjNREF, Option->OSolref);
		AssignSequence(Out.o_solimp, mjNIMP, Option->OSolimp);
		AssignSequence(Out.o_friction, 5, Option->OFriction);
		Assign(Out.integrator, Option->Integrator);
		Assign(Out.cone, Option->Cone);
		Assign(Out.jacobian, Option->Jacobian);
		Assign(Out.solver, Option->Solver);
		Assign(Out.iterations, Option->Iterations);
		Assign(Out.ls_iterations, Option->LsIterations);
		Assign(Out.noslip_iterations, Option->NoslipIterations);
		Assign(Out.ccd_iterations, Option->CcdIterations);
		Assign(Out.sdf_iterations, Option->SdfIterations);
		Assign(Out.sdf_initpoints, Option->SdfInitpoints);

		// MJCF spells this as the actuator groups to disable; MuJoCo holds one
		// bit per group.
		if (Option->Actuatorgroupdisable.IsSet())
		{
			int Mask = 0;
			for (const int32 Group : *Option->Actuatorgroupdisable)
			{
				if (Group >= 0 && Group < 31)
				{
					Mask |= 1 << Group;
				}
			}
			Out.disableactuator = Mask;
		}
	}

	if (Flags != nullptr)
	{
		AssignEnableBit(Out.enableflags, mjENBL_SLEEP, Flags->Sleep);
		AssignDisableBit(Out.disableflags, mjDSBL_MULTICCD, Flags->Multiccd);
	}
}

void MjApplyOptionToModel(const UMjOption* Option, const UMjFlag* Flags, mjModel* Model)
{
	if (Model != nullptr)
	{
		MjApplyOption(Option, Flags, Model->opt);
	}
}
