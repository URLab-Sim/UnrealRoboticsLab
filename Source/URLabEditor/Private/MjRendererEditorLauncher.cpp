// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MjRendererEditorLauncher.h"

#include "MjLevelOps.h"
#include "URLabEditorLogging.h"

#include "MuJoCo/Render/MjRendererBoot.h"
#include "Wire/UrlabWire.gen.h"

#include "Editor.h"
#include "Engine/World.h"

void UMjRendererEditorLauncher::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	// One reading of the launch surface, shared with the runtime launcher. Anything the
	// resolver classifies as a launch reaches TryLaunch, including the shapes it can only
	// refuse: reading just some of the flags is how this path and the runtime launcher came
	// to disagree about which workflow to start.
	if (FMjRendererBoot::FromCommandLine().Action == EMjBootAction::None)
	{
		return; // not a renderer launch
	}

	// Defer: the editor world does not exist yet at subsystem init, and the
	// initial map is still opening. Poll until both have settled, then launch.
	TickHandle = FTSTicker::GetCoreTicker().AddTicker(
		FTickerDelegate::CreateUObject(this, &UMjRendererEditorLauncher::TryLaunch), 0.25f);
}

void UMjRendererEditorLauncher::Deinitialize()
{
	if (TickHandle.IsValid())
	{
		FTSTicker::GetCoreTicker().RemoveTicker(TickHandle);
		TickHandle.Reset();
	}
	Super::Deinitialize();
}

bool UMjRendererEditorLauncher::TryLaunch(float DeltaTime)
{
	Waited += DeltaTime;

	const bool bWorldReady = GEditor && GEditor->GetEditorWorldContext().World() != nullptr;
	if (!bWorldReady)
	{
		return Waited < 30.0f; // keep waiting, give up after 30s
	}
	if (Waited < 0.75f)
	{
		return true; // let the initial map finish opening before we switch levels
	}

	const FMjRendererBootPlan Plan = FMjRendererBoot::FromCommandLine();
	TickHandle.Reset();

	switch (Plan.Action)
	{
		case EMjBootAction::Build:
		{
			// The canonical render service starts model-free in the play world. An explicit
			// authoring preview may still carry a model until its stage-two migration.
			if (!Plan.HasModel())
			{
				UE_LOG(LogURLabEditor, Display,
					TEXT("[URLabRender] workflow=%s has no authoring preview to prepare; "
						 "the typed render service starts in the play world"),
					MjDriveToken(Plan.Drive));
				return false;
			}
			FString LevelPath, Err;
			if (!URLabLevelOps::CreateLevelSync(TEXT("FastPathRender"), /*bForceOverwrite=*/true,
					LevelPath, Err))
			{
				UE_LOG(LogURLabEditor, Error,
					TEXT("[MjRenderer] editor launch: level setup failed: %s"), *Err);
				return false;
			}
			UE_LOG(LogURLabEditor, Log,
				TEXT("[MjRenderer] editor ready on %s; press Play to boot the renderer"), *LevelPath);
			return false;
		}

		case EMjBootAction::Simulation:
			// The play-world subsystem owns simulation boot. Keep the current authored map
			// so an authored source is visible and an explicit native/authored conflict is
			// refused by that subsystem instead of being hidden by changing levels.
			UE_LOG(LogURLabEditor, Display,
				TEXT("[URLabSimulation] editor ready; press Play to start the simulation service"));
			return false;

		case EMjBootAction::Viewer:
			UE_LOG(LogURLabEditor,Display,TEXT("[URLabViewer] editor ready; press Play to start the explicit viewer service"));
			return false;

		case EMjBootAction::Fail:
		default:
			UE_LOG(LogURLabEditor, Error, TEXT("[MjRenderer] editor launch refused: %s"), *Plan.Error);
			return false;
	}
}
