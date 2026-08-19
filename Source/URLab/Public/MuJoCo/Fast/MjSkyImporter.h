// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

struct mjModel_;
typedef struct mjModel_ mjModel;
struct mjData_;
typedef struct mjData_ mjData;
class UWorld;

/**
 * Import a MuJoCo model's skybox into the fast-path world so camera renders match
 * MuJoCo's environment. A MuJoCo skybox is a texture with tex_type == mjtexSKYBOX:
 * six cube faces stacked vertically in tex_data (height == 6 * width). We build a
 * UTextureCube from those faces, drive the world's SkyLight (ambient + reflections),
 * and show it as the visible background sky.
 */
namespace MjSkyImporter
{
/** Import the model's ENVIRONMENT into the world: its <light> elements (positions,
 *  directions and types read from Data's resolved world transforms), the camera
 *  headlight (as an ambient fill), and its skybox (SkyLight cubemap). Replaces any
 *  previously-imported lights, so it is safe to call on every model (re)load.
 *  No-op when bBaseLevel is set (a curated level keeps its own rig). Must run on the
 *  game thread; editor-only (uses the editor texture path), a no-op when packaged. */
URLAB_API void ApplyMjEnvironment(UWorld& World, const mjModel* Model,
	const mjData* Data, bool bBaseLevel, const FVector& SceneOrigin = FVector::ZeroVector);
} // namespace MjSkyImporter
