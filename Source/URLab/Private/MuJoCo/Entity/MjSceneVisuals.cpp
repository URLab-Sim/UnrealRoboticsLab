// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Entity/MjSceneVisuals.h"

#include "Engine/World.h"

#include "MuJoCo/Entity/MjAppearanceStore.h"
#include "MuJoCo/Entity/MjLightStore.h"

UMjSceneVisualsSubsystem* UMjSceneVisualsSubsystem::Get(UWorld* World)
{
	return World != nullptr ? World->GetSubsystem<UMjSceneVisualsSubsystem>() : nullptr;
}

UMjAppearanceStore* UMjSceneVisualsSubsystem::Appearance()
{
	if (AppearanceStore == nullptr)
	{
		AppearanceStore = NewObject<UMjAppearanceStore>(this);
		AppearanceStore->InitForWorld(GetWorld());
	}
	return AppearanceStore;
}

UMjLightStore* UMjSceneVisualsSubsystem::Lights()
{
	if (LightStore == nullptr)
	{
		LightStore = NewObject<UMjLightStore>(this);
		LightStore->InitForWorld(GetWorld());
	}
	return LightStore;
}

void UMjSceneVisualsSubsystem::ReapplyAll(UWorld* World)
{
	UMjSceneVisualsSubsystem* Visuals = Get(World);
	if (Visuals == nullptr)
	{
		return;
	}
	// Only re-drive stores that exist: creating one here to reapply nothing would
	// allocate on every rebuild of a scene no client has ever styled.
	if (Visuals->AppearanceStore != nullptr)
	{
		Visuals->AppearanceStore->ReapplyAll();
	}
	if (Visuals->LightStore != nullptr)
	{
		Visuals->LightStore->ReapplyAll();
	}
}

void UMjSceneVisualsSubsystem::Deinitialize()
{
	// Drop the stores explicitly rather than leaving them for GC: their baselines
	// and texture cache reference actors of a world that is going away.
	AppearanceStore = nullptr;
	LightStore = nullptr;
	Super::Deinitialize();
}
