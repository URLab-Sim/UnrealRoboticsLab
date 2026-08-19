// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MuJoCo/Fast/MjRendererLauncher.h"

#include "MuJoCo/Fast/MjRenderer.h"
#include "MuJoCo/Fast/MjRendererSubsystem.h"
#include "MuJoCo/Entity/MjModelSource.h"
#include "Utils/URLabLogging.h"

#include "Engine/World.h"
#include "Engine/GameInstance.h"
#include "GameFramework/Pawn.h"
#include "EngineUtils.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

namespace
{
// Parse -URLabFastOrigin=X,Y,Z (UE cm). Zero if absent/malformed.
// bShouldStopOnSeparator=false so the commas are not treated as token separators.
FVector ParseFastOrigin()
{
	FVector Origin = FVector::ZeroVector;
	FString OriginStr;
	if (FParse::Value(FCommandLine::Get(), TEXT("URLabFastOrigin="), OriginStr, false))
	{
		TArray<FString> Parts;
		OriginStr.ParseIntoArray(Parts, TEXT(","));
		if (Parts.Num() == 3)
		{
			Origin = FVector(
				FCString::Atod(*Parts[0]), FCString::Atod(*Parts[1]), FCString::Atod(*Parts[2]));
		}
		else
		{
			UE_LOG(LogURLab, Warning,
				TEXT("[MjRenderer] -URLabFastOrigin='%s' is not X,Y,Z; ignoring"), *OriginStr);
		}
	}
	return Origin;
}
} // namespace

void UMjRendererLauncher::OnWorldBeginPlay(UWorld& InWorld)
{
	Super::OnWorldBeginPlay(InWorld);
	if (!InWorld.IsGameWorld())
	{
		return;
	}

	// Headless render server: the stock GameModeBase auto-spawns a visible ADefaultPawn
	// (a ~1 m sphere) at the world origin when the level has no PlayerStart, and that
	// sphere shows up in the MuJoCo cameras (the grey "dome" over the scene). Nothing
	// here needs a player, so hide every auto-spawned pawn.
	for (TActorIterator<APawn> PawnIt(&InWorld); PawnIt; ++PawnIt)
	{
		PawnIt->SetActorHiddenInGame(true);
	}

	// The boot model may arrive as a compiled MJB (-URLabFastMjb, version-locked to
	// this libmujoco) or as source this libmujoco compiles itself: MJCF XML
	// (-URLabFastXml, assets from the file's own dir) or a .mjz archive
	// (-URLabFastMjz, unzipped in-engine to xml+assets). xml/mjz are immune to MJB
	// version skew.
	FString Mjb, FastXml, FastMjz;
	const bool bHasMjb =
		FParse::Value(FCommandLine::Get(), TEXT("URLabFastMjb="), Mjb) && !Mjb.IsEmpty();
	const bool bHasXml =
		FParse::Value(FCommandLine::Get(), TEXT("URLabFastXml="), FastXml) && !FastXml.IsEmpty();
	const bool bHasMjz =
		FParse::Value(FCommandLine::Get(), TEXT("URLabFastMjz="), FastMjz) && !FastMjz.IsEmpty();
	if (!bHasMjb && !bHasXml && !bHasMjz)
	{
		// No command-line model: this is the server-browser boot. Consume a pending
		// browser join (we just OpenLevel'd into the chosen environment), else show
		// the browser when asked (-URLabFastBrowser). Anything else is a normal map.
		if (UGameInstance* GI = InWorld.GetGameInstance())
		{
			if (UMjRendererSubsystem* Sub = GI->GetSubsystem<UMjRendererSubsystem>())
			{
				if (Sub->HasPendingJoin())
				{
					Sub->ConsumePendingJoin(&InWorld);
				}
				else
				{
					// -URLabFastAutoJoin[=scene]: headless render-farm node that joins
					// the first (or scene-matching) owner with no UI. Otherwise
					// -URLabFastBrowser shows the interactive server browser.
					FString AutoScene;
					const bool bAutoJoin =
						FParse::Value(FCommandLine::Get(), TEXT("URLabFastAutoJoin="), AutoScene)
						|| FParse::Param(FCommandLine::Get(), TEXT("URLabFastAutoJoin"));
					if (bAutoJoin)
					{
						FString Level;
						FParse::Value(FCommandLine::Get(), TEXT("URLabFastLevel="), Level);
						Sub->BeginAutoJoin(AutoScene, Level, ParseFastOrigin(),
							FParse::Param(FCommandLine::Get(), TEXT("URLabFastCameras")));
					}
					else if (FParse::Param(FCommandLine::Get(), TEXT("URLabFastBrowser")))
					{
						Sub->ShowBrowser();
					}
				}
			}
		}
		return;
	}

	// In PIE the editor-world preview (built by LaunchFastPathSync) is duplicated
	// into this world and its BeginPlay already rebuilds + streams. Don't spawn a
	// second scene on top of it. This launcher is the pure -game / packaged path,
	// where no editor preview exists.
	for (TActorIterator<AMjRenderer> It(&InWorld); It; ++It)
	{
		UE_LOG(LogURLab, Log,
			TEXT("[MjRenderer] a fast-path scene already exists in this world; launcher skipping"));
		return;
	}
	FString Bus;
	FParse::Value(FCommandLine::Get(), TEXT("URLabFastBus="), Bus);

	// Direct: step this MJB in-process through the shared engine (a full sim a
	// Python client can drive over RPC), instead of mirroring an owner's bus.
	const bool bDirect = FParse::Param(FCommandLine::Get(), TEXT("URLabFastDirect"));

	// Base-level mode: the boot map is a curated scene the operator authored (its
	// own lights, sky, floor, props), so the launcher must NOT populate its default
	// light rig on top of it. The MJB still loads into whatever map is booted.
	const bool bBaseLevel = FParse::Param(FCommandLine::Get(), TEXT("URLabFastBaseLevel"));

	const bool bCameras = FParse::Param(FCommandLine::Get(), TEXT("URLabFastCameras"));

	const FVector Origin = ParseFastOrigin();

	// Resolve the boot model to what SpawnRenderer consumes: a file path (MJB) or an
	// in-memory MJB. xml/mjz are compiled to MJB here with THIS libmujoco (same
	// normalize step fastpath_load does over the wire), then handed to the renderer's
	// existing in-memory-MJB path -- so nothing downstream needs to know the source
	// format. MJB stays a plain file path (no recompile).
	TArray<uint8> ModelBytes;
	FString ModelPath = Mjb;
	FString SourceDesc = Mjb;
	if (bHasXml || bHasMjz)
	{
		const FString SrcPath = bHasXml ? FastXml : FastMjz;
		const FString Format = bHasXml ? TEXT("xml") : TEXT("mjz");
		FString Err;
		if (!MjModelSource::CompileFileToMjb(SrcPath, Format, ModelBytes, Err))
		{
			UE_LOG(LogURLab, Error,
				TEXT("[MjRenderer] could not compile -URLabFast%s='%s': %s"),
				bHasXml ? TEXT("Xml") : TEXT("Mjz"), *SrcPath, *Err);
			return;
		}
		ModelPath.Empty();  // use the compiled bytes, not a file
		SourceDesc = FString::Printf(TEXT("%s (%s, %d KB mjb)"), *SrcPath, *Format, ModelBytes.Num() / 1024);
	}

	// One shared builder for the -game launcher and the runtime server browser.
	AMjRenderer::SpawnRenderer(&InWorld, ModelBytes, ModelPath, Bus, Origin, bDirect, bBaseLevel, bCameras);
	UE_LOG(LogURLab, Log, TEXT("[MjRenderer] launched: model=%s mode=%s bus=%s baseLevel=%d"),
		*SourceDesc, bDirect ? TEXT("direct") : TEXT("puppet"),
		Bus.IsEmpty() ? TEXT("(none)") : *Bus, bBaseLevel ? 1 : 0);
}
