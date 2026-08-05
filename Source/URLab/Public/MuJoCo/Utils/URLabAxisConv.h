// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"

// ============================================================================
// URLabAxisConv
//
// The one place MuJoCo's frame and units become Unreal's, and the only place a
// handedness or unit audit has to read. Every seam that crosses the boundary --
// the editor preview, the editor write-back, the runtime render pass, the
// runtime input path -- goes through these five functions and no others.
//
// The spec itself is never converted: an MJCF attribute stays in MuJoCo's
// frame, in metres and radians, exactly as authored. Conversion happens at the
// moment a value becomes an Unreal transform, and nowhere earlier.
//
// Conventions:
//   * Position:  MuJoCo metres, RHS -> UE centimetres, LHS. Y negated, x100.
//   * Direction: RHS -> LHS. Y negated, magnitude untouched.
//   * Rotation:  MJCF [w, x, y, z] -> FQuat [x, y, z, w], X and Z negated.
// ============================================================================

namespace URLabAxisConv
{
/** MuJoCo position (metres, RHS) -> UE FVector (centimetres, LHS). */
FORCEINLINE FVector MjPositionToUe(const double InM[3])
{
	return FVector(
		static_cast<FVector::FReal>(InM[0] * 100.0),
		static_cast<FVector::FReal>(-InM[1] * 100.0),
		static_cast<FVector::FReal>(InM[2] * 100.0));
}

/** As above, for the float arrays a compiled model stores mesh vertices in. */
FORCEINLINE FVector MjPositionToUe(const float InM[3])
{
	const double AsDouble[3] = {InM[0], InM[1], InM[2]};
	return MjPositionToUe(AsDouble);
}

/** UE FVector position (cm, LHS) -> MuJoCo metres (RHS). */
FORCEINLINE void UePositionToMj(const FVector& In, double OutM[3])
{
	OutM[0] = static_cast<double>(In.X) / 100.0;
	OutM[1] = -static_cast<double>(In.Y) / 100.0;
	OutM[2] = static_cast<double>(In.Z) / 100.0;
}

/**
 * MuJoCo direction (RHS) -> UE FVector (LHS), magnitude untouched.
 *
 * Distinct from MjPositionToUe by the missing x100: a joint axis has no length
 * to convert, and a physical vector (gravity in m/s^2, wind in m/s) keeps its
 * SI magnitude, because scaling it would silently change the physics rather
 * than the frame.
 */
FORCEINLINE FVector MjDirectionToUe(const double In[3])
{
	return FVector(
		static_cast<FVector::FReal>(In[0]),
		static_cast<FVector::FReal>(-In[1]),
		static_cast<FVector::FReal>(In[2]));
}

/** UE FVector direction (LHS) -> MuJoCo (RHS), magnitude untouched. */
FORCEINLINE void UeDirectionToMj(const FVector& In, double Out[3])
{
	Out[0] = static_cast<double>(In.X);
	Out[1] = -static_cast<double>(In.Y);
	Out[2] = static_cast<double>(In.Z);
}

/** MuJoCo quaternion (wxyz, RHS) -> UE FQuat (xyzw, LHS). */
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

/** UE FQuat (LHS) -> MuJoCo quaternion (wxyz, RHS). */
FORCEINLINE void UeQuatToMj(const FQuat& In, double OutWXYZ[4])
{
	OutWXYZ[0] = static_cast<double>(In.W);
	OutWXYZ[1] = -static_cast<double>(In.X);
	OutWXYZ[2] = static_cast<double>(In.Y);
	OutWXYZ[3] = -static_cast<double>(In.Z);
}
} // namespace URLabAxisConv
