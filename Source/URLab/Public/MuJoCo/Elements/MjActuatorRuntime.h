// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Pure classification for authored actuator element kinds. Runtime control and
// observation use the model-wide simulation service.

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"

#include "MjActuatorRuntime.generated.h"

class UMjNodeComponent;

/**
 * Classification of authored `<actuator>` child elements.
 */
UCLASS()
class URLAB_API UMjActuatorRuntime : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	/** True when `Node` is an `<actuator>` child element. */
	static bool IsActuator(const UMjNodeComponent* Node);
};
