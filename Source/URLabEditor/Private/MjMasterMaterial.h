// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// The master material, as code rather than as a graph nobody can review.
//
// `M_MuJoCo_Master` is a content asset, and a content asset is a binary blob: a
// change to it is unreadable in a diff and unreproducible without opening the
// editor. This builds it instead, so the shading contract lives beside the
// resolver that writes its parameters and the two cannot drift apart silently.
//
// The one rule the material obeys is that it has NO static switch parameters.
// A static switch is a shader permutation, and neither a UMaterialInstanceDynamic
// nor a cooked build can create one, so a switch-gated texture slot is a slot
// that only an editor-time UMaterialInstanceConstant can ever fill. Every slot
// is therefore sampled unconditionally against a neutral default -- white, or a
// flat normal -- and every knob is a scalar, vector or texture parameter. The
// cost is a handful of always-on texture fetches, which is nothing against a
// robotics preview, and the gain is that one MID expresses any MuJoCo material
// in the editor and in a packaged render server alike.
//
// Run it with:  UnrealEditor-Cmd.exe <project> -run=MjBuildMasterMaterial

#include "CoreMinimal.h"
#include "Commandlets/Commandlet.h"

#include "MjMasterMaterial.generated.h"

class UMaterial;

namespace urlab::editor
{

/**
 * Build (or rebuild) `/UnrealRoboticsLab/Materials/M_MuJoCo_Master`.
 *
 * Replaces the asset's expression graph wholesale, so it is idempotent and an
 * existing master is upgraded in place, keeping every reference to it valid.
 * `bSave` writes the package; pass false to build one for inspection only.
 */
UMaterial* BuildMuJoCoMasterMaterial(bool bSave);

} // namespace urlab::editor

/** Commandlet wrapper, so the asset can be rebuilt without opening the editor. */
UCLASS()
class UMjBuildMasterMaterialCommandlet : public UCommandlet
{
	GENERATED_BODY()

public:
	virtual int32 Main(const FString& Params) override;
};
