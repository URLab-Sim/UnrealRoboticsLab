// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Spec/MjAssetResolve.h"

struct mjModel_;

/**
 * A geom's appearance read straight off a compiled mjModel, as the same
 * FMjMaterialValues the spec path produces -- so both the baked (MJB) and the
 * imported render paths converge on MjApplyMaterialParameters + the one master
 * material instead of each writing the master's parameters their own way.
 *
 * The scalar terms keep MuJoCo's -1 "unset" sentinel (MjMetallicFor /
 * MjRoughnessFor fold it), and a role's TextureNames slot is filled with a
 * non-empty marker when the material carries that texture role, so the
 * scalar-vs-map guards inside MjApplyMaterialParameters read the model exactly
 * as UMjbAssetBaker::ApplyGeomMaterial did inline. The MJB's textures live in
 * tex_data and are bound by the baker afterward; this only carries their
 * presence, never a resolvable name.
 *
 * OutBaseColor is the rgba the geom draws with (its material's when it has one,
 * else its own) and OutGeomSize is the geom's planar extent in metres for the
 * texuniform mapping (a size-0 plane reports the finite quad's half-extent).
 */
URLAB_API FMjMaterialValues MjMaterialValuesFromModel(const mjModel_* Model, int32 GeomId,
	FLinearColor& OutBaseColor, FVector2D& OutGeomSize);
