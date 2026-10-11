// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Spec/MjNodeComponent.h"

class UMjModelAsset;

/**
 * Build the generated native MJCF/VFS closure for one `<model file>` from the
 * referenced Blueprint's current authored component tree. The first row is the
 * model's `file` row; remaining rows are dependency mounts. No source XML is read.
 */
URLAB_API bool MjBuildChildModelResources(const UMjModelAsset& ModelAsset,
	TArray<FMjPreparedAssetResource>& OutResources, FString& OutError);
