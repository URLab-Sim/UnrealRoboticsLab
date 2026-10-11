// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Core/MjModelDescriptor.h"

struct mjModel_;
struct mjData_;
typedef mjModel_ mjModel;
typedef mjData_ mjData;

enum class EMjModelStateScalar : uint8
{
	Float64
};

enum class EMjModelStateError : uint8
{
	None,
	NoModel,
	WrongData,
	InvalidMask,
	WrongModel,
	WrongMuJoCoVersion,
	WrongScalar,
	WrongLength,
	NonFinite,
	CounterExhausted
};

struct URLAB_API FMjModelObservation
{
	FMjModelIdentity Identity;
	uint64 ClockEpoch = 0;
	uint64 StepSequence = 0;
	double Time = 0.0;
	TArray<double> Qpos;
	TArray<double> Qvel;
	TArray<double> Ctrl;
	TArray<double> SensorData;
};

struct URLAB_API FMjModelState
{
	FMjModelIdentity Identity;
	uint64 ClockEpoch = 0;
	uint32 StateMask = 0;
	EMjModelStateScalar Scalar = EMjModelStateScalar::Float64;
	bool bCompleteCheckpoint = false;
	TArray<double> Values;
};

/** Pure MuJoCo adapter helpers. Returned values own all memory. */
namespace MjModelRuntime
{
	URLAB_API bool BuildDescriptor(const mjModel* Model, uint64 Generation,
		FMjModelDescriptor& OutDescriptor, FString& OutError);
	URLAB_API bool CaptureObservation(const mjModel* Model, const mjData* Data,
		const FMjModelIdentity& Identity, uint64 ClockEpoch, uint64 StepSequence,
		FMjModelObservation& OutObservation);
	URLAB_API EMjModelStateError CaptureState(const mjModel* Model, const mjData* Data,
		const FMjModelIdentity& Identity, uint64 ClockEpoch, uint32 StateMask,
		FMjModelState& OutState);
	URLAB_API EMjModelStateError ValidateState(const mjModel* Model,
		const FMjModelIdentity& Identity, const FMjModelState& State);
	/** Applies validated state only. It deliberately does not forward, step, or invoke callbacks. */
	URLAB_API EMjModelStateError ApplyState(const mjModel* Model, mjData* Data,
		const FMjModelIdentity& Identity, const FMjModelState& State);
}
