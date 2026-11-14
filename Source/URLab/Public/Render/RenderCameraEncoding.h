#pragma once

#include "CoreMinimal.h"
#include "Render/RenderCameraTypes.h"

/**
 * Encode one completed frame with the canonical typed camera wire semantics,
 * writing exactly Configuration.LogicalPixelBytes into OutBytes.
 */
URLAB_API bool RenderEncodeCameraFrame(
	const FRenderCameraFrame& Frame,
	const FRenderCameraEffectiveConfiguration& Configuration,
	FString& OutEncoding, int32& OutStrideBytes,
	TArrayView<uint8> OutBytes, FString& OutError);

inline bool RenderEncodeCameraFrame(
	const FRenderCameraFrame& Frame,
	const FRenderCameraEffectiveConfiguration& Configuration,
	FString& OutEncoding, int32& OutStrideBytes,
	TArray<uint8>& OutBytes, FString& OutError)
{
	OutBytes.Reset();
	if (Configuration.LogicalPixelBytes > 0 && Configuration.LogicalPixelBytes <= MAX_int32)
	{
		OutBytes.SetNumUninitialized(static_cast<int32>(Configuration.LogicalPixelBytes));
	}
	if (!RenderEncodeCameraFrame(
		Frame, Configuration, OutEncoding, OutStrideBytes, MakeArrayView(OutBytes), OutError))
	{
		OutBytes.Reset();
		return false;
	}
	return true;
}
