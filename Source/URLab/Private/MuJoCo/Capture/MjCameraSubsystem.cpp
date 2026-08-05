// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MuJoCo/Capture/MjCameraSubsystem.h"
#include "MuJoCo/Elements/MjCamera.h"
#include "MuJoCo/Core/AMjManager.h"

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

	// Prune dead cameras and note whether any is actively capturing this tick.
	bool bAnyActive = false;
	for (int32 i = Cameras.Num() - 1; i >= 0; --i)
	{
		UMjCamera* Cam = Cameras[i].Get();
		if (!Cam)
		{
			Cameras.RemoveAtSwap(i, EAllowShrinking::No);
			continue;
		}
		if (Cam->IsCaptureActive())
		{
			bAnyActive = true;
		}
	}

	// Apply the latest physics snapshot once before any camera captures this tick,
	// so a tick-driven capture (MaybeCapture) renders the same up-to-date,
	// correctly-lit scene the synchronous render path does. Bridge-driven direct /
	// puppet sessions do not otherwise apply the snapshot on the game thread every
	// frame, which left the streamed capture rendering an unlit (dark) scene.
	// Gated on an active camera so idle worlds pay nothing.
	if (bAnyActive)
	{
		if (AAMjManager* Mgr = AAMjManager::GetManager())
		{
			Mgr->ApplyLatestRenderState();
		}
	}

	for (const TWeakObjectPtr<UMjCamera>& CamPtr : Cameras)
	{
		if (UMjCamera* Cam = CamPtr.Get())
		{
			Cam->UpdateCapturePipeline();
		}
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
