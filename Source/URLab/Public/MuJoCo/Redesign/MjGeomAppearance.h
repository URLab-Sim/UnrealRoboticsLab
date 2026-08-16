// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Spec/MjAssetResolve.h" // EMjMaterialRole

/**
 * A name-keyed appearance override for visual domain randomization. Re-drives a geom's EXISTING
 * dynamic MID (off the one master material) -- parametric (scalar/vector params) + texture-swap.
 * OFF the mjModel entirely: DR variants live as UE content / pushed bytes, keyed by geom name, never
 * baked into the model. Only set fields override; clearing re-runs the base material pass.
 *
 * STEP 0 freezes the shape; phase 10 adds USTRUCT(BlueprintType) if BP-exposed.
 */
struct URLAB_API FMjGeomAppearance
{
	TOptional<FLinearColor> BaseColor;
	TOptional<float> Metallic;
	TOptional<float> Roughness;
	TOptional<float> Specular;
	TOptional<float> Reflectance;
	TOptional<float> Emission;
	TOptional<FVector2D> TexRepeat;

	/** role -> content-cache key / named UTexture to bind into that slot. */
	TMap<EMjMaterialRole, FName> TextureBindings;
};
