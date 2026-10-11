// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include "CoreMinimal.h"

class UBlueprint;

/** Replace file-backed hfield and skin inputs with their native compiled, editable spec data. */
bool MjMaterializeNativeFileAssets(UBlueprint& Blueprint, const FString& NativeSourcePath, FString& OutError);
