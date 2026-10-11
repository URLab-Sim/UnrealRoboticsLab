#pragma once

#include "CoreMinimal.h"
#include "Render/RenderCameraTypes.h"

/** Encode one completed frame with the canonical typed camera wire semantics. */
URLAB_API bool RenderEncodeCameraFrame(
	const FRenderCameraFrame& Frame,
	const FRenderCameraEffectiveConfiguration& Configuration,
	FString& OutEncoding, int32& OutStrideBytes,
	TArray<uint8>& OutBytes, FString& OutError);
