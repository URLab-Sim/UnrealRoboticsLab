// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Spec/MjSpecRef.h"

class UBlueprint;

/** Import and bind separate editable child Blueprints for native `<model file>` references. */
bool MjImportChildModels(UBlueprint& Blueprint, const FString& NativeSourcePath,
	const FMjDocParseOptions& Options, FString& OutError);
