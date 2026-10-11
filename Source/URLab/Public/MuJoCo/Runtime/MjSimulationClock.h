// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Runtime/MjSimulationControl.h"

enum class EMjSimulationClockMode : uint8
{
	Stepped,
	FreeRun
};

enum class EMjSimulationClockError : uint8
{
	None,
	NoModel,
	WrongModelGeneration,
	InvalidMode,
	WrongMode,
	InvalidElapsed,
	InvalidTimeStep,
	CounterExhausted
};

struct URLAB_API FMjSimulationClockStatus
{
	EMjSimulationClockMode Mode = EMjSimulationClockMode::Stepped;
	uint64 ModelGeneration = 0;
	double TimeStepSeconds = 0.0;
	double AccumulatedDebtSeconds = 0.0;
	double DroppedTimeSeconds = 0.0;
	uint64 DroppedStepCount = 0;
	uint32 MaxStepsPerAdvance = 64;
	double MaxRetainedDebtSeconds = 0.25;
};

struct URLAB_API FMjSimulationClockChangeResult
{
	EMjSimulationClockError Error = EMjSimulationClockError::None;
	EMjSimulationControlError AuthorityError = EMjSimulationControlError::None;
	bool bCommitted = false;
	FString Message;
	FMjSimulationClockStatus Status;

	bool IsOk() const
	{
		return Error == EMjSimulationClockError::None
			&& AuthorityError == EMjSimulationControlError::None;
	}
};

struct URLAB_API FMjSimulationAdvanceResult
{
	EMjSimulationClockError Error = EMjSimulationClockError::None;
	bool bCommitted = false;
	uint32 IntegratedStepCount = 0;
	double DroppedTimeSeconds = 0.0;
	uint64 DroppedStepCount = 0;
	FString Message;
	FMjSimulationClockStatus Status;

	bool IsOk() const { return Error == EMjSimulationClockError::None; }
};
