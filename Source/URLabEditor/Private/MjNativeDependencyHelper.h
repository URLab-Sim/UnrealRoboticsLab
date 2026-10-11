// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include "CoreMinimal.h"

/** Run MuJoCo's recursive XML dependency scanner outside the editor process. */
bool MjDiscoverNativeDependencies(const FString& SourcePath, TArray<FString>& OutPaths, FString& OutError);
