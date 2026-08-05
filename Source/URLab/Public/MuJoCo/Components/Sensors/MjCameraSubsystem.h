// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "MjCameraSubsystem.generated.h"

class UMjCamera;

/**
 * @class UMjCameraSubsystem
 * @brief World subsystem that owns the active MjCameras and drives their
 *        per-frame capture pipeline (gating, harvest, capture, publish) in one
 *        pass, instead of every camera ticking itself.
 *
 * Cameras register on BeginPlay and unregister on EndPlay. The subsystem's Tick
 * runs each registered camera's UpdateCapturePipeline once per frame.
 */
UCLASS()
class URLAB_API UMjCameraSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	void RegisterCamera(UMjCamera* Camera);
	void UnregisterCamera(UMjCamera* Camera);

	// UTickableWorldSubsystem
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;
	virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;

private:
	// Weak so a camera destroyed without a clean EndPlay can't dangle; nulls are
	// pruned on the next Tick.
	TArray<TWeakObjectPtr<UMjCamera>> Cameras;
};
