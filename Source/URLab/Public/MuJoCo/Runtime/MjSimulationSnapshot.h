// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Core/MjModelDescriptor.h"

enum class EMjSimulationSnapshotError : uint8
{
	None,
	NoModel,
	InvalidNativeState,
	NonFinite
};

struct URLAB_API FMjSimulationStateIdentity
{
	FMjModelIdentity Model;
	uint64 ClockEpoch = 0;
	uint64 StepSequence = 0;
	uint64 StateRevision = 0;
	double Time = 0.0;
};

/** Owned canonical MuJoCo Cartesian arrays: RHS metres and +wxyz quaternions. */
struct URLAB_API FMjSimulationCartesianSnapshot
{
	FMjSimulationStateIdentity Identity;
	TArray<double> BodyPositions;
	TArray<double> BodyQuaternions;
	TArray<double> CameraPositions;
	TArray<double> CameraQuaternions;
	/** Resolved world-space MuJoCo light positions/directions, dense by light index. */
	TArray<double> LightPositions;
	TArray<double> LightDirections;
};
