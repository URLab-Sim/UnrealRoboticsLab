#pragma once

#include "CoreMinimal.h"

/**
 * Converts between Unreal Engine (Left-Handed, cm, degrees)
 * and MuJoCo (Right-Handed, m, radians).
 */
struct URLAB_API FMJUnitConverter
{
	static constexpr double CmToMScale = 0.01;
	static constexpr double MToCmScale = 100.0;

	static FORCEINLINE double CmToM(double Cm) { return Cm * CmToMScale; }
	static FORCEINLINE double MToCm(double M) { return M * MToCmScale; }

	template <typename T>
	static FORCEINLINE void UEPositionToMJ(const FVector& UEPos, T* OutMJPos3)
	{
		OutMJPos3[0] = static_cast<T>(UEPos.X * CmToMScale);
		OutMJPos3[1] = static_cast<T>(-UEPos.Y * CmToMScale);
		OutMJPos3[2] = static_cast<T>(UEPos.Z * CmToMScale);
	}

	template <typename T>
	static FORCEINLINE void MJPositionToUE(const T* MJPos3, FVector& OutUEPos)
	{
		OutUEPos.X = static_cast<double>(MJPos3[0]) * MToCmScale;
		OutUEPos.Y = -static_cast<double>(MJPos3[1]) * MToCmScale;
		OutUEPos.Z = static_cast<double>(MJPos3[2]) * MToCmScale;
	}

	template <typename T>
	static FORCEINLINE void UEDirectionToMJ(const FVector& UEDir, T* OutMJDir3)
	{
		OutMJDir3[0] = static_cast<T>(UEDir.X);
		OutMJDir3[1] = static_cast<T>(-UEDir.Y);
		OutMJDir3[2] = static_cast<T>(UEDir.Z);
	}

	template <typename T>
	static FORCEINLINE void MJDirectionToUE(const T* MJDir3, FVector& OutUEDir)
	{
		OutUEDir.X = static_cast<double>(MJDir3[0]);
		OutUEDir.Y = -static_cast<double>(MJDir3[1]);
		OutUEDir.Z = static_cast<double>(MJDir3[2]);
	}

	// Algebraic involution: q_MJ(w, x, y, z) = (W, -X, Y, -Z)
	template <typename T>
	static FORCEINLINE void UEQuatToMJ(const FQuat& UEQuat, T* OutMJQuatWXYZ)
	{
		const FQuat Q = UEQuat.GetNormalized();
		OutMJQuatWXYZ[0] = static_cast<T>(Q.W);
		OutMJQuatWXYZ[1] = static_cast<T>(-Q.X);
		OutMJQuatWXYZ[2] = static_cast<T>(Q.Y);
		OutMJQuatWXYZ[3] = static_cast<T>(-Q.Z);
	}

	template <typename T>
	static FORCEINLINE void MJQuatToUE(const T* MJQuatWXYZ, FQuat& OutUEQuat)
	{
		OutUEQuat = FQuat(
			-static_cast<double>(MJQuatWXYZ[1]),
			static_cast<double>(MJQuatWXYZ[2]),
			-static_cast<double>(MJQuatWXYZ[3]),
			static_cast<double>(MJQuatWXYZ[0])
		).GetNormalized();
	}

	template <typename T>
	static FORCEINLINE void UETransformToMJ(const FTransform& UETransform, T* OutMJPos3, T* OutMJQuatWXYZ)
	{
		UEPositionToMJ(UETransform.GetLocation(), OutMJPos3);
		UEQuatToMJ(UETransform.GetRotation(), OutMJQuatWXYZ);
	}

	template <typename T>
	static FORCEINLINE void MJTransformToUE(const T* MJPos3, const T* MJQuatWXYZ, FTransform& OutUETransform)
	{
		FVector Pos;
		FQuat Quat;
		MJPositionToUE(MJPos3, Pos);
		MJQuatToUE(MJQuatWXYZ, Quat);
		OutUETransform = FTransform(Quat, Pos, FVector::OneVector);
	}
};
