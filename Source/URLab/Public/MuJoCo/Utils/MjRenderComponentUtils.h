// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "Components/PrimitiveComponent.h"

// Fast-path render components never contribute to distance-field lighting or AO.
// The shared engine primitive meshes (the Plane especially) carry a mesh distance
// field, and a flat/non-uniformly-scaled primitive yields a degenerate DF matrix
// that spams "InverseFast: NIL/non-invertible matrix" ensures every frame in -game.
// Dropping the component from the DF scene removes the cost and the noise.
inline void DisableDistanceFields(UPrimitiveComponent* Comp)
{
	if (Comp)
	{
		Comp->SetAffectDistanceFieldLighting(false);
	}
}
