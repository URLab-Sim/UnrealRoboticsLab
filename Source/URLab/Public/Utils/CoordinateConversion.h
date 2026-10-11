// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"

// ============================================================================
// URLabCoordinateConversion
//
// The one place right-handed SI frame values become Unreal values, and the only place a
// handedness or unit audit has to read. Every seam that crosses the boundary --
// the editor preview, the editor write-back, the runtime render pass, the
// runtime input path -- goes through these functions and no others. MuJoCo is
// one producer of this convention; the bridge Cartesian codec is another.
//
// Authored or wire values remain in their source convention. Conversion happens
// at the moment a value becomes an Unreal vector or transform, and nowhere earlier.
//
// Conventions:
//   * Position:  metres, RHS -> UE centimetres, LHS. Y negated, x100.
//   * Direction: RHS -> LHS. Y negated, magnitude untouched.
//   * Rotation:  RHS [w, x, y, z] -> FQuat [x, y, z, w], X and Z negated.
// ============================================================================

namespace URLabCoordinateConversion
{
/** Right-handed position in metres -> UE FVector in centimetres, left-handed. */
FORCEINLINE FVector RightHandedMetresPositionToUnrealCentimetres(const double InM[3])
{
	return FVector(
		static_cast<FVector::FReal>(InM[0] * 100.0),
		static_cast<FVector::FReal>(-InM[1] * 100.0),
		static_cast<FVector::FReal>(InM[2] * 100.0));
}

/** As above, for the float arrays a compiled model stores mesh vertices in. */
FORCEINLINE FVector RightHandedMetresPositionToUnrealCentimetres(const float InM[3])
{
	const double AsDouble[3] = {InM[0], InM[1], InM[2]};
	return RightHandedMetresPositionToUnrealCentimetres(AsDouble);
}

/** UE FVector position in centimetres, left-handed -> right-handed metres. */
FORCEINLINE void UnrealCentimetresPositionToRightHandedMetres(const FVector& In, double OutM[3])
{
	OutM[0] = static_cast<double>(In.X) / 100.0;
	OutM[1] = -static_cast<double>(In.Y) / 100.0;
	OutM[2] = static_cast<double>(In.Z) / 100.0;
}

/**
 * Right-handed direction -> UE FVector, left-handed, magnitude untouched.
 *
 * Distinct from RightHandedMetresPositionToUnrealCentimetres by the missing x100: a joint axis has no length
 * to convert, and a physical vector (gravity in m/s^2, wind in m/s) keeps its
 * SI magnitude, because scaling it would silently change the physics rather
 * than the frame.
 */
FORCEINLINE FVector RightHandedDirectionToUnreal(const double In[3])
{
	return FVector(
		static_cast<FVector::FReal>(In[0]),
		static_cast<FVector::FReal>(-In[1]),
		static_cast<FVector::FReal>(In[2]));
}

/** UE FVector direction, left-handed -> right-handed, magnitude untouched. */
FORCEINLINE void UnrealDirectionToRightHanded(const FVector& In, double Out[3])
{
	Out[0] = static_cast<double>(In.X);
	Out[1] = -static_cast<double>(In.Y);
	Out[2] = static_cast<double>(In.Z);
}

/** Right-handed quaternion (wxyz) -> UE FQuat (xyzw), left-handed. */
FORCEINLINE FQuat RightHandedWxyzQuaternionToUnreal(const double InWXYZ[4])
{
	// wxyz -> xyzw + X/Z negate so the resulting LHS quat matches
	// the converted RightHandedMetresPositionToUnrealCentimetres vectors.
	return FQuat(
		static_cast<FQuat::FReal>(-InWXYZ[1]),
		static_cast<FQuat::FReal>(InWXYZ[2]),
		static_cast<FQuat::FReal>(-InWXYZ[3]),
		static_cast<FQuat::FReal>(InWXYZ[0]))
		.GetNormalized();
}

/**
 * Right-handed optical pose (camera looks down -Z with +Y up) to Unreal's
 * camera-component convention (+X forward, +Z up).
 */
FORCEINLINE FTransform RightHandedOpticalPoseToUnreal(
	const double PositionMetres[3], const double RotationWxyz[4])
{
	const FQuat Source = RightHandedWxyzQuaternionToUnreal(RotationWxyz);
	const FQuat OpticalCorrection = FRotationMatrix::MakeFromXZ(
		FVector(0.0, 0.0, -1.0), FVector(0.0, -1.0, 0.0)).ToQuat();
	return FTransform(Source * OpticalCorrection,
		RightHandedMetresPositionToUnrealCentimetres(PositionMetres),
		FVector::OneVector);
}

/** UE FQuat, left-handed -> right-handed quaternion (wxyz). */
FORCEINLINE void UnrealQuaternionToRightHandedWxyz(const FQuat& In, double OutWXYZ[4])
{
	OutWXYZ[0] = static_cast<double>(In.W);
	OutWXYZ[1] = -static_cast<double>(In.X);
	OutWXYZ[2] = static_cast<double>(In.Y);
	OutWXYZ[3] = -static_cast<double>(In.Z);
}
} // namespace URLabCoordinateConversion
