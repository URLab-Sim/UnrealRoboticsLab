#pragma once

#include "Render/RenderSceneResources.h"
struct mjModel_;

namespace MjRenderResourceShim
{
	/** Validate and copy the compiled MuJoCo render pools into neutral resources. */
	URLAB_API bool Fill(FRenderSceneResources& Out, const mjModel_* Model,
		const FString& SceneRevision, FString& OutError);

	/** Build a surface binding for a flex slice with a specific MuJoCo material id (or INDEX_NONE). */
	URLAB_API FRenderSurfaceBinding MakeFlexSurfaceBinding(const mjModel_* Model,
		int32 FlexId, int32 MaterialId);
}
