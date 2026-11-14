// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

/**
 * The texture roles MJCF declares, in `mjtTextureRole` order.
 *
 * `Count` is the array bound, not a role. `User` (role 0 in the C enum) has no
 * MJCF spelling and no slot here.
 */
enum class EMjMaterialRole : uint8
{
	Rgb = 0,
	Occlusion,
	Roughness,
	Metallic,
	Normal,
	Opacity,
	Emissive,
	Rgba,
	Orm,
	Count
};

/** The master material's texture parameter for a slot, e.g. "RgbTexture". */
URLAB_API const TCHAR* MjMaterialRoleParameter(EMjMaterialRole Role);

/**
 * A name-keyed appearance override for visual domain randomization. It re-drives a geom's existing
 * dynamic material instance (off the one master material) -- parametric scalar/vector params plus
 * texture swaps -- without touching the mjModel: variants live as UE content or pushed bytes keyed
 * by geom name. Only the set fields override; clearing one re-runs the base material pass.
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
