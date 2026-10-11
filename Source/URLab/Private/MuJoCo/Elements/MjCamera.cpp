// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#include "MuJoCo/Elements/MjCamera.h"

UMjCamera::UMjCamera()
{
	PrimaryComponentTick.bCanEverTick = false;
}

FRenderCameraCaptureSettings UMjCamera::ExportCaptureSettings() const
{
	FRenderCameraCaptureSettings Out;
	Out.Mode = CaptureMode;
	Out.DepthNearCm = DepthNearCm;
	Out.DepthFarCm = DepthFarCm;
	Out.bCaptureOnStateChange = bCaptureOnStateChange;
	Out.CaptureMaxFps = CaptureMaxFps;
	Out.bManualCaptureOnly = bManualCaptureOnly;
	Out.HistoryCapacity = HistoryCapacity;
	Out.RequestActiveTtlSeconds = RequestActiveTtlSeconds;
	Out.DelaySeconds = DelaySeconds;
	Out.DelayJitterSeconds = DelayJitterSeconds;
	Out.bDelayUseWallClock = bDelayUseWallClock;
	Out.bRenderInMainRenderer = bRenderInMainRenderer;
	Out.StreamingBoost = StreamingBoost;
	return Out;
}
