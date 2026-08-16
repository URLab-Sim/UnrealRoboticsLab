// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Entity/MjBodyKinematics.h"

#include "mujoco/mujoco.h"

#include "MuJoCo/Entity/MjEntity.h"

namespace MjBodyKinematics
{
	TArray<FMjBodyPose> ForEntity(const mjModel* Model, const mjData* Data, const FMjEntity& Entity)
	{
		TArray<FMjBodyPose> Out;
		if (Model == nullptr || Data == nullptr)
		{
			return Out;
		}

		Out.Reserve(Entity.BodyIds.Num());
		for (const int32 Id : Entity.BodyIds)
		{
			if (Id < 0 || Id >= Model->nbody)
			{
				continue;
			}

			FMjBodyPose& Pose = Out.AddDefaulted_GetRef();
			const char* N = mj_id2name(Model, mjOBJ_BODY, Id);
			Pose.Name = N ? FName(UTF8_TO_TCHAR(N)) : NAME_None;

			const mjtNum* Xpos = &Data->xpos[3 * Id];
			Pose.Pos = FVector(Xpos[0], Xpos[1], Xpos[2]);

			// MuJoCo stores quaternions wxyz; FQuat is (x, y, z, w).
			const mjtNum* Xquat = &Data->xquat[4 * Id];
			Pose.Rot = FQuat(Xquat[1], Xquat[2], Xquat[3], Xquat[0]);
		}

		return Out;
	}
}
