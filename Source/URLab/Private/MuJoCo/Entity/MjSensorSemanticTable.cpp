// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

// Model-only sensor semantics: maps a compiled sensor's mjtSensor type to the EMjSensorSemantic a
// typed ROS publisher pairs into Imu/Wrench/JointState/tf messages, with a name heuristic as a
// fallback for types MuJoCo does not distinguish (plugin/user/limit sensors carrying an IMU-style
// name). Mirrors the live UMjSensorRuntime::GetSemantic table without needing a per-sensor component.

#include "MuJoCo/Entity/MjSensorSemanticTable.h"

#include "mujoco/mujoco.h"
#include "State/MjStateTypes.h"

namespace
{
	// Keyword fallback for sensor types that carry no distinct semantic of their own (plugin, user,
	// limit and geometric-relationship sensors). Applied only when the type switch yields Generic, so
	// a well-typed sensor is never reclassified by an incidental name match.
	EMjSensorSemantic SemanticFromName(const FString& Name)
	{
		if (Name.Contains(TEXT("gyro")))
		{
			return EMjSensorSemantic::Gyro;
		}
		if (Name.Contains(TEXT("accel")))
		{
			return EMjSensorSemantic::Accel;
		}
		if (Name.Contains(TEXT("velocimeter")))
		{
			return EMjSensorSemantic::Velocity;
		}
		if (Name.Contains(TEXT("magnet")))
		{
			return EMjSensorSemantic::Magnetometer;
		}
		if (Name.Contains(TEXT("rangefinder")))
		{
			return EMjSensorSemantic::Rangefinder;
		}
		if (Name.Contains(TEXT("touch")))
		{
			return EMjSensorSemantic::Touch;
		}
		if (Name.Contains(TEXT("torque")))
		{
			return EMjSensorSemantic::Torque;
		}
		if (Name.Contains(TEXT("force")))
		{
			return EMjSensorSemantic::Force;
		}
		return EMjSensorSemantic::Generic;
	}
} // namespace

EMjSensorSemantic MjSensorSemantics::ForSensor(const mjModel* Model, int32 SensorId)
{
	if (Model == nullptr || SensorId < 0 || SensorId >= Model->nsensor)
	{
		return EMjSensorSemantic::Generic;
	}

	switch (Model->sensor_type[SensorId])
	{
		case mjSENS_GYRO:
			return EMjSensorSemantic::Gyro;
		case mjSENS_ACCELEROMETER:
			return EMjSensorSemantic::Accel;
		case mjSENS_VELOCIMETER:
			return EMjSensorSemantic::Velocity;
		case mjSENS_FORCE:
			return EMjSensorSemantic::Force;
		case mjSENS_TORQUE:
			return EMjSensorSemantic::Torque;
		case mjSENS_TOUCH:
			return EMjSensorSemantic::Touch;
		case mjSENS_RANGEFINDER:
			return EMjSensorSemantic::Rangefinder;
		case mjSENS_MAGNETOMETER:
			return EMjSensorSemantic::Magnetometer;
		case mjSENS_JOINTPOS:
			return EMjSensorSemantic::JointPos;
		case mjSENS_JOINTVEL:
			return EMjSensorSemantic::JointVel;
		case mjSENS_ACTUATORPOS:
			return EMjSensorSemantic::ActuatorPos;
		case mjSENS_ACTUATORVEL:
			return EMjSensorSemantic::ActuatorVel;
		case mjSENS_ACTUATORFRC:
			return EMjSensorSemantic::ActuatorFrc;
		case mjSENS_FRAMEPOS:
			return EMjSensorSemantic::FramePos;
		case mjSENS_FRAMEQUAT:
			return EMjSensorSemantic::FrameQuat;
		case mjSENS_FRAMEXAXIS:
		case mjSENS_FRAMEYAXIS:
		case mjSENS_FRAMEZAXIS:
			return EMjSensorSemantic::FrameAxis;
		case mjSENS_FRAMELINVEL:
			return EMjSensorSemantic::FrameLinVel;
		case mjSENS_FRAMEANGVEL:
			return EMjSensorSemantic::FrameAngVel;
		case mjSENS_FRAMELINACC:
			return EMjSensorSemantic::FrameLinAcc;
		case mjSENS_FRAMEANGACC:
			return EMjSensorSemantic::FrameAngAcc;
		case mjSENS_SUBTREECOM:
			return EMjSensorSemantic::SubtreeCom;
		case mjSENS_SUBTREELINVEL:
			return EMjSensorSemantic::SubtreeLinVel;
		case mjSENS_SUBTREEANGMOM:
			return EMjSensorSemantic::SubtreeAngMom;
		case mjSENS_CLOCK:
			return EMjSensorSemantic::Clock;
		default:
			break;
	}

	const char* RawName = mj_id2name(Model, mjOBJ_SENSOR, SensorId);
	return RawName != nullptr ? SemanticFromName(FString(UTF8_TO_TCHAR(RawName))) : EMjSensorSemantic::Generic;
}
