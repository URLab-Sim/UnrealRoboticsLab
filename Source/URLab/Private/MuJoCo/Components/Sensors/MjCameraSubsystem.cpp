// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MuJoCo/Components/Sensors/MjCameraSubsystem.h"
#include "MuJoCo/Components/Sensors/MjCamera.h"

void UMjCameraSubsystem::RegisterCamera(UMjCamera* Camera)
{
	if (Camera)
	{
		Cameras.AddUnique(Camera);
	}
}

void UMjCameraSubsystem::UnregisterCamera(UMjCamera* Camera)
{
	Cameras.RemoveAll([Camera](const TWeakObjectPtr<UMjCamera>& C) {
		return !C.IsValid() || C.Get() == Camera;
	});
}

void UMjCameraSubsystem::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	for (int32 i = Cameras.Num() - 1; i >= 0; --i)
	{
		UMjCamera* Cam = Cameras[i].Get();
		if (!Cam)
		{
			Cameras.RemoveAtSwap(i, EAllowShrinking::No);
			continue;
		}
		Cam->UpdateCapturePipeline();
	}
}

TStatId UMjCameraSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(UMjCameraSubsystem, STATGROUP_Tickables);
}

bool UMjCameraSubsystem::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
	// Cameras run in PIE, cooked game, and the editor sim (non-PIE), so tick in
	// all of them.
	return WorldType == EWorldType::Game
		|| WorldType == EWorldType::PIE
		|| WorldType == EWorldType::Editor;
}
