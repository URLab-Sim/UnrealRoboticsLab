// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Reading a sensor, for all 49 of them.
//
// The schema gives every sensor kind its own element class, and they share no
// base but UMjNodeComponent. Reading one is nevertheless the same operation
// every time: take the compiled id, index m->sensor_adr and m->sensor_dim, copy
// that many doubles out of d->sensordata, and apply the MuJoCo -> UE fixup its
// kind calls for. Nothing in that depends on which sensor it is except the
// fixup, which is a table lookup.
//
// So this is a function library rather than 49 subclasses or one base nobody
// could give them. The parameter is the shared base, so the Blueprint node
// appears when dragging off any sensor pin, and the element kind is checked at
// runtime instead of by the type system -- the one thing given up, and the
// reason every entry point fails closed on a node that is not a sensor.
//
// Nothing here caches. The model and data are the engine's, fetched per call,
// because a cached mjModel* outlives the compile that produced it exactly once.

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"

#include "State/MjStateTypes.h"

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
 * Runtime reads of a `<sensor>` child element.
 *
 * Every entry point takes the sensor as `UMjNodeComponent*` and returns an
 * empty or zero result when it is not a sensor, is unbound, or the simulation
 * has no compiled model. A caller that wants to tell those apart should ask
 * for the dimension first: it is 0 in exactly those cases.
 */
UCLASS()
class URLAB_API UMjSensorRuntime : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/**
	 * The sensor's current output, corrected into Unreal's frame and units.
	 *
	 * This is the display-facing read. The state IR carries raw MuJoCo SI
	 * instead, so anything crossing the wire should collect from there rather
	 * than undo the correction applied here.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Sensor",
		meta = (DefaultToSelf = "Sensor", ScriptMethod))
	static TArray<float> GetReading(const UMjNodeComponent* Sensor);

	/** The first component of the raw output, uncorrected. 0 when unreadable. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Sensor",
		meta = (DefaultToSelf = "Sensor", ScriptMethod))
	static float GetScalarReading(const UMjNodeComponent* Sensor);

	/**
	 * How many values this sensor produces, from the compiled model.
	 *
	 * Read from `m->sensor_dim` rather than the schema, because the variable
	 * width kinds -- rangefinder, contact, tactile, plugin, user -- only have
	 * one once the model exists.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Sensor",
		meta = (DefaultToSelf = "Sensor", ScriptMethod))
	static int32 GetDimension(const UMjNodeComponent* Sensor);

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
