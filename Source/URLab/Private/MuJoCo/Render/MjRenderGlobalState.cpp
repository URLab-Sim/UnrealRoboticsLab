// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. This plugin incorporates
// third-party software: MuJoCo (Apache 2.0). See ThirdPartyNotices.txt.

#include "MuJoCo/Render/MjRenderGlobalState.h"

#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "HAL/IConsoleManager.h"

namespace
{
/** Read an int console variable, or Fallback when it does not exist. */
int32 GetCVarInt(const TCHAR* Name, int32 Fallback)
{
	IConsoleVariable* Var = IConsoleManager::Get().FindConsoleVariable(Name);
	return Var ? Var->GetInt() : Fallback;
}

void SetCVarInt(const TCHAR* Name, int32 Value)
{
	if (IConsoleVariable* Var = IConsoleManager::Get().FindConsoleVariable(Name))
	{
		Var->Set(Value, ECVF_SetByCode);
	}
}

constexpr const TCHAR* kFilmGrain = TEXT("r.FilmGrain");
constexpr const TCHAR* kMotionBlurQuality = TEXT("r.MotionBlurQuality");
constexpr const TCHAR* kDefaultMotionBlur = TEXT("r.DefaultFeature.MotionBlur");
} // namespace

void FMjRenderGlobalState::PushQuality(bool bSkip)
{
	if (bSkip || bQualityCaptured || !GEngine)
	{
		return;
	}
	PrevQuality = Scalability::GetQualityLevels();
	PrevFilmGrain = GetCVarInt(kFilmGrain, 1);
	PrevMotionBlurQuality = GetCVarInt(kMotionBlurQuality, 4);
	PrevDefaultMotionBlur = GetCVarInt(kDefaultMotionBlur, 1);
	bQualityCaptured = true;

	// Cinematic everywhere so the Lumen final gather converges instead of shimmering,
	// with the two temporally accumulated effects that read as grain switched off. This
	// is what makes a movable-light scene look clean rather than noisy.
	Scalability::FQualityLevels Levels = PrevQuality;
	Levels.SetFromSingleQualityLevel(4);
	Scalability::SetQualityLevels(Levels);
	SetCVarInt(kFilmGrain, 0);
	SetCVarInt(kMotionBlurQuality, 0);
	SetCVarInt(kDefaultMotionBlur, 0);
}

void FMjRenderGlobalState::PushDisableWorldRendering(UGameViewportClient* Viewport)
{
	if (bViewportCaptured || !Viewport)
	{
		return;
	}
	CapturedViewport = Viewport;
	bPrevDisableWorldRendering = Viewport->bDisableWorldRendering;
	bViewportCaptured = true;
	Viewport->bDisableWorldRendering = true;
}

void FMjRenderGlobalState::PushUnclampedFrameRate()
{
	if (bFrameRateCaptured || !GEngine)
	{
		return;
	}
	bPrevSmoothFrameRate = GEngine->bSmoothFrameRate;
	bPrevUseFixedFrameRate = GEngine->bUseFixedFrameRate;
	bFrameRateCaptured = true;

	GEngine->bSmoothFrameRate = false;
	GEngine->bUseFixedFrameRate = false;
}

void FMjRenderGlobalState::RestoreAll()
{
	if (bQualityCaptured)
	{
		Scalability::SetQualityLevels(PrevQuality);
		SetCVarInt(kFilmGrain, PrevFilmGrain);
		SetCVarInt(kMotionBlurQuality, PrevMotionBlurQuality);
		SetCVarInt(kDefaultMotionBlur, PrevDefaultMotionBlur);
		bQualityCaptured = false;
	}
	if (bFrameRateCaptured && GEngine)
	{
		GEngine->bSmoothFrameRate = bPrevSmoothFrameRate;
		GEngine->bUseFixedFrameRate = bPrevUseFixedFrameRate;
	}
	bFrameRateCaptured = false;
	if (bViewportCaptured)
	{
		if (UGameViewportClient* Viewport = CapturedViewport.Get())
		{
			Viewport->bDisableWorldRendering = bPrevDisableWorldRendering;
		}
		CapturedViewport.Reset();
		bViewportCaptured = false;
	}
}
