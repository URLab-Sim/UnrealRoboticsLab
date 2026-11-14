// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "MjSceneContributor.generated.h"

struct mjSpec_;
typedef struct mjSpec_ mjSpec;

UINTERFACE(MinimalAPI)
class UMjSceneContributor : public UInterface
{
	GENERATED_BODY()
};

/**
 * A level object that contributes elements directly to the scene mjSpec during compilation.
 */
class URLAB_API IMjSceneContributor
{
	GENERATED_BODY()

public:
	/**
	 * Authors this contributor's elements into SceneSpec under Prefix.
	 * Runs on the game thread immediately before mj_compile.
	 */
	virtual bool ContributeToSpec(mjSpec* SceneSpec, const FString& Prefix, FString& OutError) = 0;

	/** Unique namespace prefix for this contributor's authored elements. */
	virtual FString GetScenePrefix() const = 0;

	/** World placement for the contributor (Identity if authored in world space). */
	virtual FTransform GetScenePlacement() const { return FTransform::Identity; }

	/** Called after a compile has finished. */
	virtual void OnSceneBound() {}
};
