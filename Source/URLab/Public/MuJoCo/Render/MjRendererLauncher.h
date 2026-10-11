// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "MjRendererLauncher.generated.h"

/**
 * @class UMjRendererLauncher
 * @brief Starts the selected typed world workflow from command-line flags.
 *
 * On a game world's BeginPlay it takes the resolved FMjRendererBootPlan and executes it:
 * start the independent simulation service, build a request/subscriber renderer,
 * auto-join or browse for a discovered driver, or refuse an unsupported shape.
 *
 * This is the only launcher. PIE reaches it exactly as the cooked binary does, off the
 * same plan; the editor subsystem now does nothing but put the editor on a clean level
 * first, so the two shapes cannot drift apart again.
 */
UCLASS()
class URLAB_API UMjRendererLauncher : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;
	virtual void Deinitialize() override;

private:
	TWeakObjectPtr<class UURLabRenderServerWorldSubsystem> RenderWorkflow;
	TWeakObjectPtr<class UURLabViewerWorldSubsystem> ViewerWorkflow;
};
