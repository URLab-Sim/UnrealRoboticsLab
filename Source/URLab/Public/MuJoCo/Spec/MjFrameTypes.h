// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// MuJoCo-frame value types that Unreal must not mistake for its own.
//
// A spec attribute is verbatim MJCF: MuJoCo's frame, MuJoCo's units. The
// schema types every spatial quantity as `double[N]` and says nothing about
// what the numbers mean, so `pos` (a point, metres, right-handed), `axis` (a
// direction), `rgb1` (a colour) and `diaginertia` (kg m^2) are one declaration
// to it. They are four different things to Unreal: crossing the boundary
// negates Y and scales by 100 for a point, negates Y alone for a direction, and
// does nothing at all for the other two.
//
// Stored as `FVector` and `FQuat` they are all the same type, so every one of
// them type-checks against every transform API in the engine and three of the
// four are wrong. The quaternion is the worst of them because it has no visual
// tell: MJCF authors [w, x, y, z] right-handed, `FQuat` stores [x, y, z, w]
// left-handed, and the two differ by a component permutation AND a sign flip on
// X and Z.
//
// So the spec does not store `FVector` or `FQuat`. Each kind gets its own
// reflected type holding the authored doubles under their MJCF names, and the
// only route into Unreal space is an explicit `ToUnreal()` whose implementation
// is the one conversion library (URLabAxisConv). Handing one of these to
// `SetRelativeLocation` or `SetRelativeRotation` is a compile error, and the
// Blueprint pin will not connect to a Vector or Rotation pin either.
//
// `FMjVec3` is the deliberate hole in that scheme: it is what an MJCF triple
// with no frame and no unit stores in, and it has no `ToUnreal()` at all,
// because for a colour or an inertia diagonal no conversion is the only
// conversion that is never wrong.
//
// Which attribute is which kind is not a judgement made here: it is the
// generator's `overlay.UE_KIND` table, whose gate is exhaustive over every
// fixed `double[3]` and `double[4]` the MJCF schema declares, so a new spatial
// attribute cannot reach this file unclassified.

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"

#include "MuJoCo/Utils/URLabAxisConv.h"

#include "MjFrameTypes.generated.h"

/**
 * An MJCF position attribute: three doubles in MuJoCo's right-handed frame, in
 * metres, as authored.
 *
 * This is what the spec stores and what the writer emits. It is NOT an
 * Unreal location; `ToUnreal()` is.
 */
USTRUCT(BlueprintType)
struct FMjPosition3
{
	GENERATED_BODY()

	/** X in MuJoCo's right-handed frame, metres, as authored. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	double X = 0.0;

	/** Y in MuJoCo's right-handed frame, metres, as authored. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	double Y = 0.0;

	/** Z in MuJoCo's right-handed frame, metres, as authored. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	double Z = 0.0;

	FMjPosition3() = default;

	FMjPosition3(double InX, double InY, double InZ)
		: X(InX), Y(InY), Z(InZ)
	{
	}

	/** The same point in Unreal's frame and units: Y negated, metres to cm. */
	FVector ToUnreal() const
	{
		const double Xyz[3] = {X, Y, Z};
		return URLabAxisConv::MjPositionToUe(Xyz);
	}

	/** The MJCF spelling of an Unreal location. */
	static FMjPosition3 FromUnreal(const FVector& In)
	{
		double Xyz[3] = {0.0, 0.0, 0.0};
		URLabAxisConv::UePositionToMj(In, Xyz);
		return FMjPosition3(Xyz[0], Xyz[1], Xyz[2]);
	}

	bool operator==(const FMjPosition3& Other) const
	{
		return X == Other.X && Y == Other.Y && Z == Other.Z;
	}

	bool operator!=(const FMjPosition3& Other) const { return !(*this == Other); }
};

/**
 * An MJCF direction attribute: a joint axis, a light direction, or a physical
 * vector such as gravity. Three doubles in MuJoCo's right-handed frame.
 *
 * Crossing into Unreal negates Y and nothing else. A direction has no length to
 * rescale, and a physical vector keeps its SI magnitude, which is why this one
 * type serves both: the difference between them is what the number means, not
 * what the conversion does.
 */
USTRUCT(BlueprintType)
struct FMjDirection3
{
	GENERATED_BODY()

	/** X in MuJoCo's right-handed frame, as authored. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	double X = 0.0;

	/** Y in MuJoCo's right-handed frame, as authored. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	double Y = 0.0;

	/** Z in MuJoCo's right-handed frame, as authored. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	double Z = 0.0;

	FMjDirection3() = default;

	FMjDirection3(double InX, double InY, double InZ)
		: X(InX), Y(InY), Z(InZ)
	{
	}

	/** The same direction in Unreal's frame: Y negated, magnitude untouched. */
	FVector ToUnreal() const
	{
		const double Xyz[3] = {X, Y, Z};
		return URLabAxisConv::MjDirectionToUe(Xyz);
	}

	/** The MJCF spelling of an Unreal direction. */
	static FMjDirection3 FromUnreal(const FVector& In)
	{
		double Xyz[3] = {0.0, 0.0, 0.0};
		URLabAxisConv::UeDirectionToMj(In, Xyz);
		return FMjDirection3(Xyz[0], Xyz[1], Xyz[2]);
	}

	bool operator==(const FMjDirection3& Other) const
	{
		return X == Other.X && Y == Other.Y && Z == Other.Z;
	}

	bool operator!=(const FMjDirection3& Other) const { return !(*this == Other); }
};

/**
 * Three verbatim MJCF doubles that are not a point, a direction or a rotation:
 * a colour, an inertia diagonal, a grid count, a mesh scale, a Euler triple.
 *
 * There is deliberately no `ToUnreal()`. These quantities either have no frame
 * at all, or (Euler angles, joint ranges) have one whose unit depends on the
 * compiled model rather than on the attribute, and no static conversion could
 * be right for them.
 */
USTRUCT(BlueprintType)
struct FMjVec3
{
	GENERATED_BODY()

	/** First component, as authored. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	double X = 0.0;

	/** Second component, as authored. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	double Y = 0.0;

	/** Third component, as authored. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	double Z = 0.0;

	FMjVec3() = default;

	FMjVec3(double InX, double InY, double InZ)
		: X(InX), Y(InY), Z(InZ)
	{
	}

	bool operator==(const FMjVec3& Other) const
	{
		return X == Other.X && Y == Other.Y && Z == Other.Z;
	}

	bool operator!=(const FMjVec3& Other) const { return !(*this == Other); }
};

/**
 * An MJCF `quat` attribute: four doubles, [w, x, y, z], right-handed, verbatim.
 *
 * This is what the spec stores and what the writer emits. It is NOT an
 * Unreal rotation; `ToUnreal()` is.
 */
USTRUCT(BlueprintType)
struct FMjQuatRot
{
	GENERATED_BODY()

	/** Scalar component, as authored. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	double W = 1.0;

	/** X component in MuJoCo's right-handed frame, as authored. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	double X = 0.0;

	/** Y component in MuJoCo's right-handed frame, as authored. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	double Y = 0.0;

	/** Z component in MuJoCo's right-handed frame, as authored. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo")
	double Z = 0.0;

	FMjQuatRot() = default;

	FMjQuatRot(double InW, double InX, double InY, double InZ)
		: W(InW), X(InX), Y(InY), Z(InZ)
	{
	}

	/** The same rotation in Unreal's frame and component order. */
	FQuat ToUnreal() const
	{
		const double Wxyz[4] = {W, X, Y, Z};
		return URLabAxisConv::MjQuatToUe(Wxyz);
	}

	/** The MJCF spelling of an Unreal rotation. */
	static FMjQuatRot FromUnreal(const FQuat& In)
	{
		double Wxyz[4] = {1.0, 0.0, 0.0, 0.0};
		URLabAxisConv::UeQuatToMj(In, Wxyz);
		return FMjQuatRot(Wxyz[0], Wxyz[1], Wxyz[2], Wxyz[3]);
	}

	bool operator==(const FMjQuatRot& Other) const
	{
		return W == Other.W && X == Other.X && Y == Other.Y && Z == Other.Z;
	}

	bool operator!=(const FMjQuatRot& Other) const { return !(*this == Other); }
};

/**
 * The sanctioned bridge between a spec value and an Unreal one.
 *
 * Blueprint has no math nodes for these types, which is the point: the
 * conversion is a named call whose name carries the frame and unit decision
 * that used to be an unspoken assumption. `FMjVec3` has no entry here, because
 * there is no conversion for it that could be right.
 */
UCLASS()
class URLAB_API UMjFrameConversionLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/** An authored MJCF position, in Unreal's frame and centimetres. */
	UFUNCTION(BlueprintPure, Category = "MuJoCo|Frame",
		meta = (DisplayName = "MuJoCo Position To Unreal"))
	static FVector MjPositionToUnreal(const FMjPosition3& InPosition) { return InPosition.ToUnreal(); }

	/** An Unreal location, spelled the way MJCF authors one. */
	UFUNCTION(BlueprintPure, Category = "MuJoCo|Frame",
		meta = (DisplayName = "MuJoCo Position From Unreal"))
	static FMjPosition3 MjPositionFromUnreal(const FVector& InLocation) { return FMjPosition3::FromUnreal(InLocation); }

	/** An authored MJCF direction, in Unreal's frame. Magnitude is untouched. */
	UFUNCTION(BlueprintPure, Category = "MuJoCo|Frame",
		meta = (DisplayName = "MuJoCo Direction To Unreal"))
	static FVector MjDirectionToUnreal(const FMjDirection3& InDirection) { return InDirection.ToUnreal(); }

	/** An Unreal direction, spelled the way MJCF authors one. */
	UFUNCTION(BlueprintPure, Category = "MuJoCo|Frame",
		meta = (DisplayName = "MuJoCo Direction From Unreal"))
	static FMjDirection3 MjDirectionFromUnreal(const FVector& InDirection) { return FMjDirection3::FromUnreal(InDirection); }

	/** An authored MJCF rotation, in Unreal's frame. */
	UFUNCTION(BlueprintPure, Category = "MuJoCo|Frame",
		meta = (DisplayName = "MuJoCo Rotation To Unreal"))
	static FQuat MjRotationToUnreal(const FMjQuatRot& InRotation) { return InRotation.ToUnreal(); }

	/** An Unreal rotation, spelled the way MJCF authors one. */
	UFUNCTION(BlueprintPure, Category = "MuJoCo|Frame",
		meta = (DisplayName = "MuJoCo Rotation From Unreal"))
	static FMjQuatRot MjRotationFromUnreal(const FQuat& InRotation) { return FMjQuatRot::FromUnreal(InRotation); }
};
