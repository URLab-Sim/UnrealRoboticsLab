// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Render/MjRenderSceneAdapter.h"

/**
 * Fully resolved, owned presentation for one simulation model installation.
 * It contains no authoring actors, components, bindings, or native pointers.
 */
struct URLAB_API FMjSimulationPresentation
{
	FMjResolvedRenderCameraOverrides CameraOverrides;
	FRenderSceneInstallOptions SceneOptions;
	/** Negative means no camera-height clamp. */
	int32 CameraMaxHeight = -1;
	/** A standalone simulation does not invent an operator camera by default. */
	bool bIncludeUserCamera = false;
};
