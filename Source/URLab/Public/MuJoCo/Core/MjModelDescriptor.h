// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Contracts/MjSensorSemantic.h"

struct URLAB_API FMjModelIdentity
{
	FString ContentSha256;
	uint64 Generation = 0;
	int32 MuJoCoVersion = 0;
	int32 Nq = 0;
	int32 Nv = 0;
	int32 Nu = 0;
	int32 Na = 0;
	int32 NSensorData = 0;

	bool Matches(const FMjModelIdentity& Other) const
	{
		return Generation == Other.Generation && ContentSha256 == Other.ContentSha256
			&& MuJoCoVersion == Other.MuJoCoVersion && Nq == Other.Nq && Nv == Other.Nv
			&& Nu == Other.Nu && Na == Other.Na && NSensorData == Other.NSensorData;
	}
};

struct URLAB_API FMjJointDescriptor
{
	int32 Id = -1;
	FString Name;
	int32 Type = 0;
	int32 QposAddress = 0;
	int32 QposWidth = 0;
	int32 DofAddress = 0;
	int32 DofWidth = 0;
	bool bLimited = false;
	double RangeMin = 0.0;
	double RangeMax = 0.0;
};

struct URLAB_API FMjActuatorControlDescriptor
{
	bool bLimited = false;
	double RangeMin = 0.0;
	double RangeMax = 0.0;
};

struct URLAB_API FMjActuatorDescriptor
{
	int32 Id = -1;
	FString Name;
	int32 ControlAddress = 0;
	int32 ControlWidth = 0;
	TArray<FMjActuatorControlDescriptor> Controls;
};

struct URLAB_API FMjSensorDescriptor
{
	int32 Id = -1;
	FString Name;
	int32 Address = 0;
	int32 Dimension = 0;
	int32 Type = 0;
	EMjSensorSemantic Semantic = EMjSensorSemantic::Generic;
};

struct URLAB_API FMjNamedObjectDescriptor
{
	int32 Id = -1;
	FString Name;
};

/** Immutable after installation; owns every name and contains no MuJoCo/UObject pointer. */
struct URLAB_API FMjModelDescriptor
{
	FMjModelIdentity Identity;
	TArray<FMjJointDescriptor> Joints;
	TArray<FMjActuatorDescriptor> Actuators;
	TArray<FMjSensorDescriptor> Sensors;
	TArray<FMjNamedObjectDescriptor> Bodies;
	TArray<FMjNamedObjectDescriptor> Cameras;
};
