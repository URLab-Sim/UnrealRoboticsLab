// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MuJoCo/Fast/MjRendererLauncher.h"

#include "MuJoCo/Fast/MjRenderer.h"
#include "MuJoCo/Fast/MjRendererSubsystem.h"
#include "MuJoCo/Fast/MjRendererDriverClient.h"
#include "MuJoCo/Fast/MjLauncherFlags.h"
#include "MuJoCo/Fast/DroneViewerPawn.h"
#include "MuJoCo/Entity/MjModelSource.h"
#include "Utils/URLabLogging.h"

#include "GameFramework/PlayerController.h"

#include "Engine/World.h"
#include "Engine/GameInstance.h"
#include "GameFramework/Pawn.h"
#include "TimerManager.h"
#include "EngineUtils.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

#include "mujoco/mujoco.h"

namespace
{
// Parse the scene origin (UE cm). Zero if absent/malformed. Phase 1.1: -URLabScene=origin=X;Y;Z
// (';' separators, since ',' delimits scene keys) writes the same value the legacy
// -URLabFastOrigin=X,Y,Z did; the new key wins when present, else the legacy flag is read.
// bShouldStopOnSeparator=false so the legacy commas are not treated as token separators.
FVector ParseFastOrigin()
{
	FVector SceneOriginVec;
	if (URLabLauncherFlags::SceneOrigin(SceneOriginVec))
	{
		return SceneOriginVec;
	}

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

	// VR / spectator viewer (-URLabVrViewer): fly a free-fly drone camera around the
	// sim the viewer renders from an owner's state bus (-URLabStateSource sets up the
	// subscription; the manager does that). Here we just spawn + possess the drone,
	// after the default pawn is hidden above. Keyboard free-fly (WASD/QE + mouse).
	// Phase 1.1: -URLabCaps=vr is the new spelling of -URLabVrViewer (source-of-truth §14); both
	// enable the drone add-on identically.
	if (FParse::Param(FCommandLine::Get(), TEXT("URLabVrViewer")) || URLabLauncherFlags::CapsWantVr())
	{
		// The PlayerController is frequently not up yet at world BeginPlay (the
		// packaged boot creates it a few frames later), so defer + retry rather
		// than silently no-op when GetFirstPlayerController() is null right now.
		TryPossessVrDrone(&InWorld, 0);
	}

	// Join a gRPC OWNER (peek/mirror): fetch its model over gRPC (fastpath_hello),
	// then spawn a Mirror that subscribes to the owner's transform stream on the
	// bus the reply advertises (grpc://, so the gRPC subscribe backend is selected).
	// Pairs with -URLabVrViewer for a free-fly drone view of the live owner sim.
	// Phase 1.1: -URLabDrive=stream:grpc://<ep> is the new spelling of -URLabFastGrpcJoin=<ep>
	// (source-of-truth §14, lean: no serve); both drive the same gRPC-join mirror.
	FString GrpcJoin;
	FString DriveGrpcEp;
	const bool bLegacyGrpcJoin =
		FParse::Value(FCommandLine::Get(), TEXT("URLabFastGrpcJoin="), GrpcJoin) && !GrpcJoin.IsEmpty();
	if (!bLegacyGrpcJoin && URLabLauncherFlags::DriveStreamGrpcEndpoint(DriveGrpcEp))
	{
		GrpcJoin = DriveGrpcEp;
	}
	if (!GrpcJoin.IsEmpty())
	{
		if (!GrpcJoin.StartsWith(TEXT("grpc://")))
		{
			GrpcJoin = TEXT("grpc://") + GrpcJoin;   // scheme selects the gRPC client backend
		}
		TArray<uint8> Mjb;
		FString Bus, Err;
		if (FMjRendererDriverClient::FetchModel(GrpcJoin, Mjb, Bus, Err) && Mjb.Num() > 0)
		{
			if (Bus.IsEmpty())
			{
				Bus = GrpcJoin;   // owner didn't advertise a bus -> use the same endpoint
			}
			AMjRenderer* Mirror = AMjRenderer::SpawnRenderer(&InWorld, Mjb,
				/*MjbFilePath=*/FString(), Bus, ParseFastOrigin(),
				/*bStepped=*/false, /*bBaseLevel=*/false, /*bCameras=*/false);
			if (Mirror)
			{
				// Point the mirror's perturb channel at the owner's gRPC control
				// endpoint so ctrl-drag forwards fastpath_perturb (the mirror has no
				// physics; the owner applies the wrench, gated on its accept_input
				// capability). Without this the drag is a no-op (guarded on an empty
				// OwnerControlEndpoint).
				Mirror->OwnerControlEndpoint = GrpcJoin;
			}
			UE_LOG(LogURLab, Display,
				TEXT("[MjRenderer] -URLabFastGrpcJoin: mirroring owner %s (bus %s), %d-byte model"),
				*GrpcJoin, *Bus, Mjb.Num());
		}
		else
		{
			UE_LOG(LogURLab, Error,
				TEXT("[MjRenderer] -URLabFastGrpcJoin: FetchModel('%s') failed: %s"), *GrpcJoin, *Err);
		}
		return;
	}

	// The boot model may arrive as a compiled MJB (-URLabFastMjb, version-locked to
	// this libmujoco) or as source this libmujoco compiles itself: MJCF XML
	// (-URLabFastXml, assets from the file's own dir) or a .mjz archive
	// (-URLabFastMjz, unzipped in-engine to xml+assets). xml/mjz are immune to MJB
	// version skew.
	FString Mjb, FastXml, FastMjz;
	bool bHasMjb =
		FParse::Value(FCommandLine::Get(), TEXT("URLabFastMjb="), Mjb) && !Mjb.IsEmpty();
	bool bHasXml =
		FParse::Value(FCommandLine::Get(), TEXT("URLabFastXml="), FastXml) && !FastXml.IsEmpty();
	bool bHasMjz =
		FParse::Value(FCommandLine::Get(), TEXT("URLabFastMjz="), FastMjz) && !FastMjz.IsEmpty();
	// Phase 1.1/1.2: -URLabModel=<path.{mjb,xml,mjz}> picks the format by extension and writes the
	// same field the matching legacy -URLabFast{Mjb,Xml,Mjz} flag would (source-of-truth §14). The
	// legacy flag wins when both are given.
	if (!bHasMjb && !bHasXml && !bHasMjz)
	{
		FString ModelPathArg, ModelFmt;
		if (URLabLauncherFlags::ParseModel(ModelPathArg, ModelFmt))
		{
			if (ModelFmt == TEXT("xml"))
			{
				FastXml = ModelPathArg;
				bHasXml = true;
			}
			else if (ModelFmt == TEXT("mjz"))
			{
				FastMjz = ModelPathArg;
				bHasMjz = true;
			}
			else
			{
				Mjb = ModelPathArg;
				bHasMjb = true;
			}
		}
	}
	if (!bHasMjb && !bHasXml && !bHasMjz)
	{
		// Headless render server with no boot model (-URLabFastServe): stand up a
		// proper Mirror renderer on a tiny placeholder model -- the exact known-good
		// path the -URLabFast<model> flags use, so the renderer's EnsureManager brings
		// up the BridgeServer + gRPC/ZMQ transports AND the renderer is camera-enabled.
		// The first client load_* then hot-swaps the real model into it. (Spawning a
		// bare manager instead would build the manager's *interactive* compiled play
		// view, which is externally-driven and has no capturing cameras.)
		// Phase 1.1: -URLabDrive=await is the new spelling of -URLabFastServe (source-of-truth §14:
		// await + serve,cameras); both stand up the placeholder render server awaiting a client load.
		if (FParse::Param(FCommandLine::Get(), TEXT("URLabFastServe")) || URLabLauncherFlags::DriveIsAwait())
		{
			static const char* kPlaceholderXml =
				"<mujoco><worldbody><geom type=\"box\" size=\"0.05 0.05 0.05\"/></worldbody></mujoco>";
			TArray<uint8> XmlBytes;
			XmlBytes.Append(reinterpret_cast<const uint8*>(kPlaceholderXml),
				FCStringAnsi::Strlen(kPlaceholderXml));
			FString CompileErr;
			mjModel* Placeholder = MjModelSource::FromBytes(
				XmlBytes, TEXT("xml"), TMap<FString, TArray<uint8>>(), CompileErr);
			if (!Placeholder)
			{
				UE_LOG(LogURLab, Error,
					TEXT("[MjRenderer] -URLabFastServe: placeholder compile failed: %s"), *CompileErr);
				return;
			}
			const int32 Sz = mj_sizeModel(Placeholder);
			TArray<uint8> PlaceholderMjb;
			PlaceholderMjb.SetNumUninitialized(Sz);
			mj_saveModel(Placeholder, nullptr, PlaceholderMjb.GetData(), Sz);
			mj_deleteModel(Placeholder);

			AMjRenderer::SpawnRenderer(&InWorld, PlaceholderMjb, /*MjbFilePath=*/FString(),
				/*BusEndpoint=*/FString(), ParseFastOrigin(),
				/*bStepped=*/false, /*bBaseLevel=*/false, /*bCameras=*/true);
			UE_LOG(LogURLab, Log,
				TEXT("[MjRenderer] -URLabFastServe: render server up on placeholder, awaiting client load"));
			return;
		}

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
					// Phase 1.1/1.3: -URLabSourceFind=discover[:scene] / =browse are the new
					// spellings (source-of-truth §14); each triggers the same finder path.
					FString AutoScene;
					const bool bAutoJoin =
						FParse::Value(FCommandLine::Get(), TEXT("URLabFastAutoJoin="), AutoScene)
						|| FParse::Param(FCommandLine::Get(), TEXT("URLabFastAutoJoin"))
						|| URLabLauncherFlags::SourceFindDiscover(AutoScene);
					if (bAutoJoin)
					{
						FString Level;
						if (!FParse::Value(FCommandLine::Get(), TEXT("URLabFastLevel="), Level))
						{
							URLabLauncherFlags::SceneLevel(Level);
						}
						const bool bCamerasWanted =
							FParse::Param(FCommandLine::Get(), TEXT("URLabFastCameras"))
							|| URLabLauncherFlags::ParseCaps().bCameras.Get(false);
						Sub->BeginAutoJoin(AutoScene, Level, ParseFastOrigin(), bCamerasWanted);
					}
					else if (FParse::Param(FCommandLine::Get(), TEXT("URLabFastBrowser"))
						|| URLabLauncherFlags::SourceFindBrowse())
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
	// Phase 1.1: -URLabDrive=stream:tcp://<ep> is the new spelling of -URLabFastBus=<ep>
	// (source-of-truth §14); both name the transform bus this scene mirrors.
	FString Bus;
	if (!FParse::Value(FCommandLine::Get(), TEXT("URLabFastBus="), Bus))
	{
		URLabLauncherFlags::DriveStreamTcpEndpoint(Bus);
	}

	// Direct: step this MJB in-process through the shared engine (a full sim a
	// Python client can drive over RPC), instead of mirroring an owner's bus.
	// Phase 1.1: -URLabDrive=sim is the new spelling of -URLabFastDirect (source-of-truth §14).
	const bool bDirect = FParse::Param(FCommandLine::Get(), TEXT("URLabFastDirect"))
		|| URLabLauncherFlags::DriveIsSim();

	// Base-level mode: the boot map is a curated scene the operator authored (its
	// own lights, sky, floor, props), so the launcher must NOT populate its default
	// light rig on top of it. The MJB still loads into whatever map is booted.
	// Phase 1.1: -URLabScene=base is the new spelling of -URLabFastBaseLevel (source-of-truth §14).
	const bool bBaseLevel = FParse::Param(FCommandLine::Get(), TEXT("URLabFastBaseLevel"))
		|| URLabLauncherFlags::SceneBaseLevel();

	// Phase 1.1: -URLabCaps=cameras is the new spelling of -URLabFastCameras (source-of-truth §14).
	const bool bCameras = FParse::Param(FCommandLine::Get(), TEXT("URLabFastCameras"))
		|| URLabLauncherFlags::ParseCaps().bCameras.Get(false);

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

void UMjRendererLauncher::TryPossessVrDrone(TWeakObjectPtr<UWorld> WeakWorld, int32 Attempt)
{
	UWorld* World = WeakWorld.Get();
	if (!World)
	{
		return;
	}

	APlayerController* PC = World->GetFirstPlayerController();
	if (!PC)
	{
		// PC not created yet -- retry shortly. Cap the retries (~5s) so a headless
		// world with no player never loops forever.
		if (Attempt < 50)
		{
			World->GetTimerManager().SetTimer(
				VrPossessTimerHandle,
				FTimerDelegate::CreateUObject(
					this, &UMjRendererLauncher::TryPossessVrDrone, WeakWorld, Attempt + 1),
				0.1f, /*bLoop=*/false);
		}
		else
		{
			UE_LOG(LogURLab, Warning,
				TEXT("[MjRenderer] -URLabVrViewer: no PlayerController after ~5s; drone not possessed"));
		}
		return;
	}

	// Idempotent: a retry (or a re-entered BeginPlay) must not spawn a second drone.
	for (TActorIterator<ADroneViewerPawn> It(World); It; ++It)
	{
		return;
	}

	// Hide whatever the PC currently possesses (the GameMode's default sphere pawn),
	// so it doesn't show up in the drone's view as a grey dome over the scene.
	if (APawn* Old = PC->GetPawn())
	{
		Old->SetActorHiddenInGame(true);
	}

	const FTransform SpawnTM(FRotator(-15.0, 0.0, 0.0), FVector(-500.0, 0.0, 250.0));
	if (ADroneViewerPawn* Drone = World->SpawnActor<ADroneViewerPawn>(
			ADroneViewerPawn::StaticClass(), SpawnTM))
	{
		PC->Possess(Drone);
		PC->SetInputMode(FInputModeGameOnly());
		PC->bShowMouseCursor = false;
		UE_LOG(LogURLab, Display,
			TEXT("[MjRenderer] -URLabVrViewer: drone free-fly camera spawned + possessed (attempt %d)"),
			Attempt);
	}
}
