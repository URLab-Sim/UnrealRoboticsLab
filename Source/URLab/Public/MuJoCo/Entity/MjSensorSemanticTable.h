// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"
#include "State/MjStateTypes.h" // EMjSensorSemantic

struct mjModel_;
typedef mjModel_ mjModel;

/**
 * Model-only sensor semantics: a table over the compiled model mapping a sensor's mjtSensor type
 * (with a name heuristic) to its EMjSensorSemantic, so typed ROS messages (Imu/Wrench/...) work
 * without any per-sensor component. Computed once at build and stored on the FMjEntity, parallel to
 * SensorIds.
 */
namespace MjSensorSemantics
{
	URLAB_API EMjSensorSemantic ForSensor(const mjModel* Model, int32 SensorId);
}
