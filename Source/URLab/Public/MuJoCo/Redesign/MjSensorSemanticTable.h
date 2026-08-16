// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"
#include "State/MjStateTypes.h" // EMjSensorSemantic

struct mjModel_;
typedef mjModel_ mjModel;

/**
 * MODEL-ONLY sensor semantics. Replaces UMjSensorRuntime::GetSemantic (which read the ProtoSpec
 * schema on a live component) with a table over the compiled model: mjtSensor type + a name
 * heuristic -> EMjSensorSemantic. Lets typed ROS messages (Imu/Wrench/...) work for a shadowless
 * wire model. Computed once at build and stored on the FMjEntity (parallel to SensorIds).
 */
namespace MjSensorSemantics
{
	URLAB_API EMjSensorSemantic ForSensor(const mjModel* Model, int32 SensorId);
}
