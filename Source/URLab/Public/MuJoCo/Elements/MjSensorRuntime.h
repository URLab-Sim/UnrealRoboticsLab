// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Pure metadata and coordinate transforms for authored sensor kinds. Sensor
// values are accessed through the model-wide simulation service.

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"

#include "MuJoCo/Contracts/MjSensorSemantic.h"

#include "MjSensorRuntime.generated.h"

class UMjNodeComponent;

/**
 * The coordinate and unit family a sensor's output belongs to.
 *
 * MuJoCo reports SI in its own frame; Unreal wants centimetres in a left-handed
 * one. Which correction applies is a property of what the sensor measures, not
 * of how it is wired, so it is carried per sensor kind rather than derived.
 */
enum class EMjSensorValueKind : uint8
{
	/** No transform: a scalar has no frame. */
	Scalar,
	/** A position in metres: scale to centimetres, negate Y. */
	Position,
	/** A unit vector: negate Y, do not scale. */
	Direction,
	/** Velocity, acceleration, force, torque, angular rate: negate Y. */
	Vector3,
	/** MuJoCo (w,x,y,z) to Unreal (x,y,z,w), with the handedness fix. */
	Quaternion,
	/** Two concatenated positions, each treated as Position. */
	GeomFromTo,
};

/**
 * Pure classification and display metadata for a `<sensor>` child element.
 */
UCLASS()
class URLAB_API UMjSensorRuntime : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * The ROS-facing grouping of this sensor kind. Generic when not a sensor.
	 *
	 * Not reflected: EMjSensorSemantic belongs to the state IR, which is a
	 * plain header on purpose because it crosses to the physics worker. This
	 * and GetValueKind are C++ surface only.
	 */
	static EMjSensorSemantic GetSemantic(const UMjNodeComponent* Sensor);

	/** The transform family of this sensor kind. Scalar when not a sensor. */
	static EMjSensorValueKind GetValueKind(const UMjNodeComponent* Sensor);

	/** True when `Node` is a `<sensor>` child element. */
	static bool IsSensor(const UMjNodeComponent* Node);
};

/**
 * Apply the MuJoCo -> Unreal correction for `Kind` to `Values`, in place.
 *
 * Free rather than a library static because the collector applies it to values
 * it already holds, without a component to hang the call on.
 */
URLAB_API void MjTransformSensorReading(TArray<float>& Values, EMjSensorValueKind Kind);
