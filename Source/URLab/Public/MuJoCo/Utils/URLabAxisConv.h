// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"

// ============================================================================
// URLabAxisConv
//
// Centralised MuJoCo <-> Unreal handedness + units conversion helpers. The
// codegen and the hand-rolled import/export paths converge here so a future
// audit only has to scrub one file (instead of grepping for inlined
// expressions like ``Out.Y = -In.Y;`` and ``X / 100.0`` scattered across
// every component .cpp).
//
// Gating:
//   URLAB_USE_AXIS_CONV_HELPERS=1 routes codegen emission through these
//   helpers. Defaults to 0 (codegen stays inline) so existing serialized
//   data + compiled artifacts stay byte-identical until the migration flips.
//
// Conventions (mirroring the existing MjUtils:: shape):
//   * Position: MuJoCo metres -> UE centimetres, Y-axis sign flipped.
//   * Direction: same handedness flip but NO scale.
//   * Rotation: wxyz -> xyzw + X/Z sign flip for RHS -> LHS.
//   * Per-element apply mode (UE -> MuJoCo) inverts each rule.
// ============================================================================

#ifndef URLAB_USE_AXIS_CONV_HELPERS
#define URLAB_USE_AXIS_CONV_HELPERS 0
#endif

namespace URLabAxisConv
{
/** @brief MuJoCo position (metres, RHS) -> UE FVector (centimetres, LHS). */
FORCEINLINE FVector MjPositionToUe(const double InM[3])
{
	return FVector(
		static_cast<FVector::FReal>(InM[0] * 100.0),
		static_cast<FVector::FReal>(-InM[1] * 100.0),
		static_cast<FVector::FReal>(InM[2] * 100.0));
}

/** @brief UE FVector position (cm, LHS) -> MuJoCo metres (RHS). */
FORCEINLINE void UePositionToMj(const FVector& In, double OutM[3])
{
	OutM[0] = static_cast<double>(In.X) / 100.0;
	OutM[1] = -static_cast<double>(In.Y) / 100.0;
	OutM[2] = static_cast<double>(In.Z) / 100.0;
}

/** @brief MuJoCo direction (RHS, no scale) -> UE FVector (LHS, no scale). */
FORCEINLINE FVector MjDirectionToUe(const double In[3])
{
	return FVector(
		static_cast<FVector::FReal>(In[0]),
		static_cast<FVector::FReal>(-In[1]),
		static_cast<FVector::FReal>(In[2]));
}

/** @brief UE direction -> MuJoCo direction (LHS -> RHS, no scale). */
FORCEINLINE void UeDirectionToMj(const FVector& In, double Out[3])
{
	Out[0] = static_cast<double>(In.X);
	Out[1] = -static_cast<double>(In.Y);
	Out[2] = static_cast<double>(In.Z);
}

/** @brief MuJoCo quaternion (wxyz, RHS) -> UE FQuat (xyzw, LHS). */
FORCEINLINE FQuat MjQuatToUe(const double InWXYZ[4])
{
	// wxyz -> xyzw + X/Z negate so the resulting LHS quat matches
	// the converted MjPositionToUe vectors.
	return FQuat(
		static_cast<FQuat::FReal>(-InWXYZ[1]),
		static_cast<FQuat::FReal>(InWXYZ[2]),
		static_cast<FQuat::FReal>(-InWXYZ[3]),
		static_cast<FQuat::FReal>(InWXYZ[0]))
		.GetNormalized();
}

/** @brief UE FQuat (LHS) -> MuJoCo quaternion (wxyz, RHS). */
FORCEINLINE void UeQuatToMj(const FQuat& In, double OutWXYZ[4])
{
	OutWXYZ[0] = static_cast<double>(In.W);
	OutWXYZ[1] = -static_cast<double>(In.X);
	OutWXYZ[2] = static_cast<double>(In.Y);
	OutWXYZ[3] = -static_cast<double>(In.Z);
}

/** @brief Scalar-array Y-negate apply (in-place). Used by direction
 *  attrs like joint.axis where the codegen handles cm scaling
 *  separately (vec3_convert = "y_negate" rule path). */
FORCEINLINE void YNegate(FVector& InOut)
{
	InOut.Y = -InOut.Y;
}

/** @brief Per-element scale for fixed-size MJ array fields. The
 *  codegen array-export loop reads through this so a future units
 *  change (e.g. radians -> degrees on a specific subtype) doesn't
 *  need a per-call-site edit. */
FORCEINLINE double ScaleMToCm(double InM)
{
	return InM * 100.0;
}
FORCEINLINE double ScaleCmToM(double InCm)
{
	return InCm * 0.01;
}
} // namespace URLabAxisConv
