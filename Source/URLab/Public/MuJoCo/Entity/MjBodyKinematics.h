// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

struct mjModel_;
typedef mjModel_ mjModel;
struct mjData_;
typedef mjData_ mjData;
struct FMjEntity;

/**
 * Stateless, model-only forward kinematics for an entity's bodies. The shadowless wire model has no
 * live UMjBody actor to query, so this reads each body's world pose straight from mjData for the ROS
 * /tf and /odom publishers. Poses stay in MuJoCo's own frame and units (SI, right-handed); the ROS
 * layer performs any coordinate conversion.
 */
namespace MjBodyKinematics
{
	/** One body's name and world pose in MuJoCo's frame (SI, unconverted). */
	struct FMjBodyPose
	{
		FName Name;
		FVector Pos = FVector::ZeroVector;
		FQuat Rot = FQuat::Identity;
	};

	// World pose of every body in the entity's id slice, read from Data->xpos / Data->xquat. Model-only;
	// no components. Returns one entry per resolved body id, in slice order.
	URLAB_API TArray<FMjBodyPose> ForEntity(const mjModel* Model, const mjData* Data, const FMjEntity& Entity);
}
