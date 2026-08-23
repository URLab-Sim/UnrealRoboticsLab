// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MjRendererEditorLauncher.h"

#include "MjLevelOps.h"
#include "URLabEditorLogging.h"

#include "MuJoCo/Fast/MjLauncherFlags.h"

#include "Editor.h"
#include "Engine/World.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

void UMjRendererEditorLauncher::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);

	// A fast-path renderer launch is requested by any of: a driver control
	// endpoint to connect to (-URLabDrive=stream:<ctrl>), a discovery flag
	// (-URLabSourceFind=discover), or a direct model file path (-URLabModel)
	// (source-of-truth §14).
	FString Ignored, IgnoredScene, IgnoredFmt;
	const bool bRequested =
		URLabLauncherFlags::DriveStreamEndpoint(Ignored) ||
		URLabLauncherFlags::SourceFindDiscover(IgnoredScene) ||
		URLabLauncherFlags::ParseModel(Ignored, IgnoredFmt);
	if (!bRequested)
	{
		return; // not a fast-path renderer launch
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

	FString Err;
	bool bOk = false;

	// A bare -URLabDrive=stream:<ctrl> (no -URLabModel) connects to a driver and pulls
	// its model over the control channel (source-of-truth §14). When -URLabModel IS
	// given, the direct model+bus path below owns it instead.
	FString ModelPathArg, ModelFmtArg;
	const bool bHasModelFlag = URLabLauncherFlags::ParseModel(ModelPathArg, ModelFmtArg);

	FString DriveStreamEp;
	FString DiscoverScene;
	if (!bHasModelFlag && URLabLauncherFlags::DriveStreamEndpoint(DriveStreamEp))
	{
		// -URLabDrive=stream:<ctrl> with no explicit model: connect to the driver.
		bOk = URLabLevelOps::LaunchFastPathFromDriverSync(DriveStreamEp, /*bFreshLevel=*/true, Err);
	}
	else if (URLabLauncherFlags::SourceFindDiscover(DiscoverScene))
	{
		// Auto-discover: connect to the first advertised Driver found.
		TArray<URLabLevelOps::FMjDriverInfo> Drivers;
		FString DiscErr;
		if (!URLabLevelOps::DiscoverFastPathDrivers(Drivers, DiscErr) || Drivers.Num() == 0)
		{
			Err = Drivers.Num() == 0 ? TEXT("no fast-path drivers advertised") : DiscErr;
		}
		else
		{
			UE_LOG(LogURLabEditor, Log, TEXT("[MjRenderer] discovered %d driver(s); connecting to '%s' (%s)"),
				Drivers.Num(), *Drivers[0].Scene, *Drivers[0].Control);
			bOk = URLabLevelOps::LaunchFastPathFromDriverSync(Drivers[0].Control, /*bFreshLevel=*/true, Err);
		}
	}
	else
	{
		// Direct model file path (Driver-less or hand-specified bus).
		// -URLabModel=<mjb> + -URLabDrive=stream:tcp://<bus> (source-of-truth §14). The
		// editor direct path takes an .mjb file; format-by-extension boot for other
		// formats is the -game launcher's job (§1.2).
		FString Mjb, Bus;
		if (bHasModelFlag && ModelFmtArg == TEXT("mjb"))
		{
			Mjb = ModelPathArg;
		}
		URLabLauncherFlags::DriveStreamTcpEndpoint(Bus);
		bOk = URLabLevelOps::LaunchFastPathSync(Mjb, Bus, /*bFreshLevel=*/true, Err);
	}

	if (bOk)
	{
		UE_LOG(LogURLabEditor, Log, TEXT("[MjRenderer] editor launch complete"));
	}
	else
	{
		UE_LOG(LogURLabEditor, Error, TEXT("[MjRenderer] editor launch failed: %s"), *Err);
	}

	TickHandle.Reset();
	return false; // one-shot
}
