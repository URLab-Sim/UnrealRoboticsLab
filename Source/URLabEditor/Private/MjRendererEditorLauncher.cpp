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
	// endpoint to connect to, a discovery flag, or a direct MJB file path.
	// Phase 1.1: the new -URLab{Drive,SourceFind,Model} flags request an editor launch too
	// (source-of-truth §14): -URLabDrive=stream:<ctrl> == -URLabFastConnect,
	// -URLabSourceFind=discover == -URLabFastDiscover, -URLabModel == -URLabFastMjb.
	const TCHAR* Cmd = FCommandLine::Get();
	FString Ignored, IgnoredScene, IgnoredFmt;
	const bool bRequested =
		(FParse::Value(Cmd, TEXT("URLabFastConnect="), Ignored) && !Ignored.IsEmpty()) ||
		FParse::Param(Cmd, TEXT("URLabFastDiscover")) ||
		(FParse::Value(Cmd, TEXT("URLabFastMjb="), Ignored) && !Ignored.IsEmpty()) ||
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

	const TCHAR* Cmd = FCommandLine::Get();
	FString Err;
	bool bOk = false;

	// Phase 1.1: a bare -URLabDrive=stream:<ctrl> (no -URLabModel) is the new spelling of
	// -URLabFastConnect (source-of-truth §14): connect to a driver and pull its model over the
	// control channel. When -URLabModel IS given, the direct model+bus path below owns it instead.
	FString ModelPathArg, ModelFmtArg;
	const bool bHasModelFlag = URLabLauncherFlags::ParseModel(ModelPathArg, ModelFmtArg);

	FString Control;
	FString DriveStreamEp;
	FString DiscoverScene;
	if (FParse::Value(Cmd, TEXT("URLabFastConnect="), Control) && !Control.IsEmpty())
	{
		// Connect to a named Driver: pull its MJB + bus over the control channel.
		bOk = URLabLevelOps::LaunchFastPathFromDriverSync(Control, /*bFreshLevel=*/true, Err);
	}
	else if (!bHasModelFlag && URLabLauncherFlags::DriveStreamEndpoint(DriveStreamEp))
	{
		// -URLabDrive=stream:<ctrl> with no explicit model: same as -URLabFastConnect.
		bOk = URLabLevelOps::LaunchFastPathFromDriverSync(DriveStreamEp, /*bFreshLevel=*/true, Err);
	}
	else if (FParse::Param(Cmd, TEXT("URLabFastDiscover")) || URLabLauncherFlags::SourceFindDiscover(DiscoverScene))
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
		// Direct MJB file path (Driver-less or hand-specified bus).
		// Phase 1.1: -URLabModel=<mjb> == -URLabFastMjb, -URLabDrive=stream:tcp://<bus> == -URLabFastBus
		// (source-of-truth §14). The editor direct path takes an .mjb file; new-flag model paths of
		// another format fall through to the legacy MJB read (format-by-extension boot is the -game
		// launcher's job, §1.2). The legacy flag wins over the new one when both are given.
		FString Mjb, Bus;
		if (!FParse::Value(Cmd, TEXT("URLabFastMjb="), Mjb) && bHasModelFlag && ModelFmtArg == TEXT("mjb"))
		{
			Mjb = ModelPathArg;
		}
		if (!FParse::Value(Cmd, TEXT("URLabFastBus="), Bus))
		{
			URLabLauncherFlags::DriveStreamTcpEndpoint(Bus);
		}
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
