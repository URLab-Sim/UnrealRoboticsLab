// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Scalability.h"
#include "UObject/WeakObjectPtr.h"

class UGameViewportClient;

/**
 * @struct FMjRenderGlobalState
 * @brief The process-wide render settings a renderer changes, and what they were before.
 *
 * Standing up a render server pushes cinematic scalability, turns off the temporally
 * accumulated post effects that make a movable-light scene read grainy, and -- in the
 * request-driven regime -- stops the primary view rendering and unclamps the frame rate
 * so a render's latency is bounded by the tick rather than by frame pacing. All of that
 * is global to the process and none of it used to be put back, so returning to the server
 * browser left the viewport blind and every subsequent session inherited settings it
 * never asked for.
 *
 * Each Push* records the previous value the first time it is called and does nothing on a
 * second call, so a model reload does not capture the state it has already changed;
 * RestoreAll puts back exactly what was captured and forgets it.
 */
struct URLAB_API FMjRenderGlobalState
{
	/** Cinematic scalability plus the grain/motion-blur suppression. bSkip records
	 *  nothing and changes nothing, for a session that asked to keep its own settings. */
	void PushQuality(bool bSkip);

	/** Stop the primary view rendering: this render server produces only scene-capture
	 *  frames, and the primary view would contend with them on the GPU queue. */
	void PushDisableWorldRendering(UGameViewportClient* Viewport);

	/** Unclamp the frame rate. Frame-rate smoothing otherwise pulls a headless server
	 *  toward a ~33 ms tick, which would dominate the latency of a tick-driven render. */
	void PushUnclampedFrameRate();

	/** Put back everything captured, in any order, and forget it. Idempotent. */
	void RestoreAll();

private:
	bool bQualityCaptured = false;
	Scalability::FQualityLevels PrevQuality;
	int32 PrevFilmGrain = 0;
	int32 PrevMotionBlurQuality = 0;
	int32 PrevDefaultMotionBlur = 0;

	bool bFrameRateCaptured = false;
	bool bPrevSmoothFrameRate = false;
	bool bPrevUseFixedFrameRate = false;

	bool bViewportCaptured = false;
	TWeakObjectPtr<UGameViewportClient> CapturedViewport;
	bool bPrevDisableWorldRendering = false;
};
