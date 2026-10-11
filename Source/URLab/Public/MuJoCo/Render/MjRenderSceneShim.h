// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include "MuJoCo/Render/MjRenderScene.h"

struct mjModel_;

/** MuJoCo-owned adapter that copies a compiled model into neutral render data. */
namespace MjRenderSceneShim
{
	/** Describe Model into Out, replacing whatever Out held. A null model empties it. */
	URLAB_API void Fill(FMjRenderScene& Out, const mjModel_* Model);
}
