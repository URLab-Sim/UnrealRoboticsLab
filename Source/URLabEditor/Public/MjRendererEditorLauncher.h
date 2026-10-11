// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "EditorSubsystem.h"
#include "Containers/Ticker.h"
#include "MjRendererEditorLauncher.generated.h"

/**
 * @class UMjRendererEditorLauncher
 * @brief The editor half of a flag launch: put the editor on the right level, nothing more.
 *
 * There is one launcher now. When the editor is started with the launch flags
 * (-URLabWorkflow / -URLabSimulationSource), the boot is resolved by the shared
 * FMjRendererBoot::FromCommandLine() and executed in the PLAY world by the runtime
 * UMjRendererLauncher -- the same code, in the same order, as the cooked binary. All this
 * subsystem still does is switch the editor to a clean dedicated level so pressing Play
 * boots into an empty world rather than into the project's default map, and report a
 * command line that names no buildable shape.
 *
 * Legacy discovery and browser-preview launch paths have been retired; authored models
 * continue through the manager-owned compile and simulation-world handoff.

 * The launch is deferred off a ticker until the editor world exists and the initial map has
 * settled, so switching to the fresh level is clean.
 */
UCLASS()
class URLABEDITOR_API UMjRendererEditorLauncher : public UEditorSubsystem
{
	GENERATED_BODY()

public:
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;

private:
	/** One-shot deferred launch; returns false to unregister once it has run. */
	bool TryLaunch(float DeltaTime);

	FTSTicker::FDelegateHandle TickHandle;
	float Waited = 0.0f;
};
