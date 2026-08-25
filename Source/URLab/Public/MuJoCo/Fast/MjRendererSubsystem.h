// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "MuJoCo/Fast/MjDriverDiscovery.h"
#include "MjRendererSubsystem.generated.h"

/**
 * A choice of environment to drop a joined Renderer into. An empty Path means
 * the bare-plane option (an empty engine map plus the launcher's own light rig);
 * a set Path is a curated level the slave is dropped into with its own lighting.
 */
USTRUCT()
struct FMjRendererLevelChoice
{
	GENERATED_BODY()
	UPROPERTY()
	FString Name;
	UPROPERTY()
	FString Path; // package path, e.g. /Game/Maps/Foo; empty = bare plane
};

/**
 * @class UMjRendererSubsystem
 * @brief Packaged-game server browser + join flow for a fast-path Renderer.
 *
 * Discovers advertised fast-path Drivers from the shared registry, lists the
 * cooked levels the operator can drop a joined Renderer into, and performs the join:
 * pull the Driver's MJB over the wire (fastpath_hello), open the chosen level, and
 * spawn the Mirror Renderer there. Lives on the GameInstance so a pending join
 * survives the OpenLevel transition. The UI (SMjRendererBrowser) is a thin
 * view over this subsystem.
 */
UCLASS()
class URLAB_API UMjRendererSubsystem : public UGameInstanceSubsystem
{
	GENERATED_BODY()

public:
	// --- discovery -------------------------------------------------------- //
	/** Rescan the registry for live fast-path drivers. */
	void RefreshDrivers();
	const TArray<FMjDriverInfo>& GetDrivers() const { return Drivers; }

	// --- environments ----------------------------------------------------- //
	/** Rebuild the level list ("Bare Plane" + every cooked /Game world). */
	void RefreshLevels();
	const TArray<FMjRendererLevelChoice>& GetLevels() const { return Levels; }

	// --- join ------------------------------------------------------------- //
	/**
	 * Join a driver into the chosen environment. Fetches the driver's MJB, records
	 * the join as pending, and opens the target level (an empty engine map for the
	 * bare-plane choice, else LevelPath). The pending join is consumed on the new
	 * world by ConsumePendingJoin. Returns false + OutError if the fetch fails.
	 *
	 * Per-join capabilities (source-of-truth §5, the joining-viewer subset):
	 *  - bCameras: build + stream the model's cameras on the mirror.
	 *  - bVr: possess the free-fly drone pawn (ADroneViewerPawn) instead of the
	 *    static framing camera.
	 *  - bInput: point the mirror's perturb channel at the driver's control
	 *    endpoint so Ctrl-drag forwards fastpath_perturb intents (the owner still
	 *    gates acceptance via its own `input`/AcceptInput capability).
	 */
	bool JoinDriver(const FMjDriverInfo& Driver, const FString& LevelPath,
		const FVector& Origin, bool bCameras, bool bVr, bool bInput, FString& OutError);

	/** Spawn a pending browser-driven join into World, if one is queued. Returns
	 *  true if a slave was spawned (and clears the pending state). */
	bool ConsumePendingJoin(UWorld* World);

	bool HasPendingJoin() const { return bJoinPending; }

	/**
	 * Headless auto-join for a render-farm node: poll discovery until a driver
	 * appears (whose scene contains SceneFilter, or the first driver when empty) and
	 * join it into LevelPath at Origin. No UI. Gives up after a bounded wait.
	 * bCameras/bVr/bInput carry the same per-join capabilities as JoinDriver
	 * (the launcher fills them from -URLabCaps; all default off = lean).
	 */
	void BeginAutoJoin(const FString& SceneFilter, const FString& LevelPath,
		const FVector& Origin, bool bCameras, bool bVr, bool bInput);

	// --- browser UI ------------------------------------------------------- //
	/** Add the server-browser widget to the game viewport (idempotent). */
	void ShowBrowser();
	/** Remove the server-browser widget. */
	void HideBrowser();

	// --- in-slave HUD (shown while joined) -------------------------------- //
	/** Overlay the slave HUD (return-to-browser + live origin nudge). */
	void ShowHud();
	void HideHud();
	/** Tear down the current Renderer and go back to the server browser. */
	void ReturnToBrowser();

	// --- live origin tuning ----------------------------------------------- //
	/** Shift the active Renderer's spawn origin by Delta (UE cm), applied live. */
	void NudgeOrigin(const FVector& Delta);
	void SetOrigin(const FVector& NewOrigin);
	FVector GetOrigin() const;
	bool HasActiveRenderer() const;

private:
	TArray<FMjDriverInfo> Drivers;
	TArray<FMjRendererLevelChoice> Levels;

	// Pending join carried across the OpenLevel transition.
	bool bJoinPending = false;
	TArray<uint8> PendingMjb;
	FString PendingBus;
	// The driver's REQ/REP control endpoint, kept so bPendingInput can point the
	// spawned mirror's perturb channel back at the owner.
	FString PendingControl;
	FVector PendingOrigin = FVector::ZeroVector;
	bool bPendingBaseLevel = false;
	bool bPendingCameras = false;
	bool bPendingVr = false;
	bool bPendingInput = false;

	TSharedPtr<class SMjRendererBrowser> BrowserWidget;
	TSharedPtr<class SMjRendererHud> HudWidget;
	// The active Renderer (for live origin tuning) + the map the browser lives on
	// (so ReturnToBrowser can OpenLevel back to it).
	TWeakObjectPtr<class AMjRenderer> ActiveRenderer;
	FName HomeMap;

	// Headless auto-join state.
	FString AutoScene;
	FString AutoLevel;
	FVector AutoOrigin = FVector::ZeroVector;
	bool bAutoCameras = false;
	bool bAutoVr = false;
	bool bAutoInput = false;
	int32 AutoJoinTries = 0;
	FTimerHandle AutoJoinTimer;
	void AutoJoinPoll();
};
