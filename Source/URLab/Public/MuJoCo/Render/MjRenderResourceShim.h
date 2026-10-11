#pragma once

#include "Render/RenderSceneResources.h"
struct mjModel_;

namespace MjRenderResourceShim
{
	/** Validate and copy the compiled MuJoCo render pools into neutral resources. */
	URLAB_API bool Fill(FRenderSceneResources& Out, const mjModel_* Model,
		const FString& SceneRevision, FString& OutError);
}
