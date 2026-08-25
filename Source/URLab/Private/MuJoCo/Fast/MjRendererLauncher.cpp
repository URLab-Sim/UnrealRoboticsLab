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

#include "Engine/World.h"
#include "Engine/GameInstance.h"
#include "GameFramework/Pawn.h"
#include "EngineUtils.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

#include "mujoco/mujoco.h"

namespace
{
// Parse the scene origin (UE cm). Zero if absent/malformed. -URLabScene=origin=X;Y;Z
// (';' separators, since ',' delimits scene keys).
FVector ParseFastOrigin()
{
	FVector SceneOriginVec;
	if (URLabLauncherFlags::SceneOrigin(SceneOriginVec))
	{
		return SceneOriginVec;
	}
	return FVector::ZeroVector;
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

	// VR / spectator viewer (-URLabCaps=vr): fly a free-fly drone camera around the
	// sim the viewer renders from an owner's transform bus (-URLabDrive=stream:<ep> sets
	// up the subscription; the manager does that). Here we just spawn + possess the drone,
	// after the default pawn is hidden above. Keyboard free-fly (WASD/QE + mouse).
	if (URLabLauncherFlags::CapsWantVr())
	{
		// Shared with the server browser's "VR free-fly" join option; the helper
		// defers + retries internally until the PlayerController exists.
		ADroneViewerPawn::SpawnAndPossess(&InWorld);
	}

	// Join a gRPC OWNER (peek/mirror): fetch its model over gRPC (fastpath_hello),
	// then spawn a Mirror that subscribes to the owner's transform stream on the
	// bus the reply advertises (grpc://, so the gRPC subscribe backend is selected).
	// Pairs with -URLabCaps=vr for a free-fly drone view of the live owner sim.
	// -URLabDrive=stream:grpc://<ep> (source-of-truth §14, lean: no serve).
	FString GrpcJoin;
	URLabLauncherFlags::DriveStreamGrpcEndpoint(GrpcJoin);
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
				TEXT("[MjRenderer] -URLabDrive=stream:grpc: mirroring owner %s (bus %s), %d-byte model"),
				*GrpcJoin, *Bus, Mjb.Num());
		}
		else
		{
			UE_LOG(LogURLab, Error,
				TEXT("[MjRenderer] -URLabDrive=stream:grpc: FetchModel('%s') failed: %s"), *GrpcJoin, *Err);
		}
		return;
	}

	// The boot model may arrive as a compiled MJB (version-locked to this libmujoco)
	// or as source this libmujoco compiles itself: MJCF XML (assets from the file's
	// own dir) or a .mjz archive (unzipped in-engine to xml+assets). xml/mjz are
	// immune to MJB version skew. -URLabModel=<path.{mjb,xml,mjz}> picks the format
	// by extension (source-of-truth §14, Phase 1.2).
	FString Mjb, FastXml, FastMjz;
	bool bHasMjb = false;
	bool bHasXml = false;
	bool bHasMjz = false;
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
		// Headless render server with no boot model (-URLabDrive=await): stand up a
		// proper Mirror renderer on a tiny placeholder model -- the exact known-good
		// path the -URLabModel boot uses, so the renderer's EnsureManager brings
		// up the BridgeServer + gRPC/ZMQ transports AND the renderer is camera-enabled.
		// The first client load_* then hot-swaps the real model into it. (Spawning a
		// bare manager instead would build the manager's *interactive* compiled play
		// view, which is externally-driven and has no capturing cameras.)
		// -URLabDrive=await (source-of-truth §14: await + serve,cameras) stands up the
		// placeholder render server awaiting a client load.
		if (URLabLauncherFlags::DriveIsAwait())
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
					TEXT("[MjRenderer] -URLabDrive=await: placeholder compile failed: %s"), *CompileErr);
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
				TEXT("[MjRenderer] -URLabDrive=await: render server up on placeholder, awaiting client load"));
			return;
		}

		// No command-line model: this is the server-browser boot. Consume a pending
		// browser join (we just OpenLevel'd into the chosen environment), else show
		// the browser when asked (-URLabSourceFind=browse). Anything else is a normal map.
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
					// -URLabSourceFind=discover[:scene]: headless render-farm node that
					// joins the first (or scene-matching) owner with no UI. Otherwise
					// -URLabSourceFind=browse shows the interactive server browser
					// (source-of-truth §14).
					FString AutoScene;
					if (URLabLauncherFlags::SourceFindDiscover(AutoScene))
					{
						FString Level;
						URLabLauncherFlags::SceneLevel(Level);
						// -URLabCaps threads into the autojoin: cameras/vr/input all
						// default off (lean) exactly as before; vr is also handled by
						// the CapsWantVr() possess above, so passing it here is
						// harmless (SpawnAndPossess is idempotent) but keeps the
						// autojoin path complete when consumed on the next world.
						const URLabLauncherFlags::FCaps Caps = URLabLauncherFlags::ParseCaps();
						Sub->BeginAutoJoin(AutoScene, Level, ParseFastOrigin(),
							Caps.bCameras.Get(false), Caps.bVr.Get(false), Caps.bInput.Get(false));
					}
					else if (URLabLauncherFlags::SourceFindBrowse())
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
	// -URLabDrive=stream:tcp://<ep> names the transform bus this scene mirrors
	// (source-of-truth §14).
	FString Bus;
	URLabLauncherFlags::DriveStreamTcpEndpoint(Bus);

	// -URLabDrive=sim: step this MJB in-process through the shared engine (a full sim
	// a Python client can drive over RPC), instead of mirroring an owner's bus
	// (source-of-truth §14).
	const bool bDirect = URLabLauncherFlags::DriveIsSim();

	// Base-level mode: the boot map is a curated scene the operator authored (its
	// own lights, sky, floor, props), so the launcher must NOT populate its default
	// light rig on top of it. The MJB still loads into whatever map is booted.
	// -URLabScene=base (source-of-truth §14).
	const bool bBaseLevel = URLabLauncherFlags::SceneBaseLevel();

	// -URLabCaps=cameras (source-of-truth §14).
	const bool bCameras = URLabLauncherFlags::ParseCaps().bCameras.Get(false);

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
				TEXT("[MjRenderer] could not compile -URLabModel %s='%s': %s"),
				bHasXml ? TEXT("(xml)") : TEXT("(mjz)"), *SrcPath, *Err);
			return;
		}
		ModelPath.Empty();  // use the compiled bytes, not a file
		SourceDesc = FString::Printf(TEXT("%s (%s, %d KB mjb)"), *SrcPath, *Format, ModelBytes.Num() / 1024);
	}

	// One shared builder for the -game launcher and the runtime server browser.
	AMjRenderer::SpawnRenderer(&InWorld, ModelBytes, ModelPath, Bus, Origin, bDirect, bBaseLevel, bCameras);
	UE_LOG(LogURLab, Log, TEXT("[MjRenderer] launched: model=%s mode=%s bus=%s baseLevel=%d"),
		*SourceDesc, bDirect ? TEXT("sim") : TEXT("stream"),
		Bus.IsEmpty() ? TEXT("(none)") : *Bus, bBaseLevel ? 1 : 0);
}
