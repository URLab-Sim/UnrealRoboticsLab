// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "EditorSubsystem.h"
#include "Containers/Ticker.h"
#include "MjbFastPathEditorLauncher.generated.h"

/**
 * @class UMjbFastPathEditorLauncher
 * @brief Editor-time launcher for the MJB fast-path renderer.
 *
 * When the editor is started with `-URLabFastMjb=<path>` (and optionally
 * `-URLabFastBus=<endpoint>`), this stands up a persistent fast-path render
 * scene in the editor world via URLabLevelOps::LaunchFastPathSync: a clean
 * dedicated level, movable lighting, and a built + connected AMjbScene. It is
 * the CLI/automation entry point that replaces the old editor Python script;
 * the same LaunchFastPathSync op is what the bridge / server-browser UI will
 * call interactively.
 *
 * The launch is deferred off a ticker until the editor world exists and the
 * initial map has settled, so switching to the fresh level is clean.
 */
UCLASS()
class URLABEDITOR_API UMjbFastPathEditorLauncher : public UEditorSubsystem
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
