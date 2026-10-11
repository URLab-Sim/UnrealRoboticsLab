// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Core/MjOwnedModel.h"
#include "MuJoCo/Runtime/MjSimulationPresentation.h"

class AAMjManager;

/**
 * Owned output of compiling one authored world for the simulation subsystem.
 *
 * Candidate owns its MuJoCo model/data. Presentation contains only resolved
 * values and renderer assets; neither retains an authoring actor, component,
 * spec, binding, or native pointer.
 */
struct URLAB_API FMjAuthoredSimulationPackage
{
	FMjOwnedModelCandidate Candidate;
	FMjSimulationPresentation Presentation;
	TArray<FString> Warnings;
	TArray<FString> Infos;
	FString Error;

	bool IsValid() const { return Candidate.IsValid() && Error.IsEmpty(); }
};

/** Game-thread compiler from level authoring objects to an owned runtime handoff. */
class URLAB_API FMjAuthoringCompiler
{
public:
	static FMjAuthoredSimulationPackage Compile(AAMjManager& SceneRoot);
};
