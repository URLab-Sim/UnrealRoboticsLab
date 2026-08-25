// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/Views/SListView.h"
#include "MuJoCo/Fast/MjDriverDiscovery.h"

class UMjRendererSubsystem;
class STextComboBox;
class SEditableTextBox;

/**
 * Runtime (packaged-game) server browser for fast-path Renderers. Lists the
 * live drivers the UMjRendererSubsystem discovered, an environment picker (bare
 * plane or any cooked level), a spawn-origin field, and the per-join capability
 * toggles (camera feeds, VR free-fly drone, interact/perturb). Connecting hands
 * the choice back to the subsystem, which pulls the MJB and opens the level.
 * This is the runtime counterpart to the editor's SMjServerBrowser.
 */
class SMjRendererBrowser : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SMjRendererBrowser) {}
		SLATE_ARGUMENT(TWeakObjectPtr<UMjRendererSubsystem>, Subsystem)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

private:
	using FDriverPtr = TSharedPtr<FMjDriverInfo>;

	TWeakObjectPtr<UMjRendererSubsystem> Subsystem;

	TArray<FDriverPtr> Drivers;
	TSharedPtr<SListView<FDriverPtr>> DriverList;

	TArray<TSharedPtr<FString>> LevelNames;
	TSharedPtr<STextComboBox> LevelCombo;
	TSharedPtr<FString> SelectedLevelName;

	TSharedPtr<SEditableTextBox> OriginBox;
	// Per-join capabilities (the joining-viewer subset of source-of-truth §5;
	// serve/publish are owner-side and not offered here).
	bool bCameras = true;
	// Off by default: the plain flat viewer is the common case; the drone takes
	// over the whole viewport + input scheme, so it must be an explicit choice.
	bool bVr = false;
	// On by default: matches the editor and gRPC join paths, which always wire
	// the perturb channel; the gesture is a deliberate Ctrl+LMB drag and the
	// OWNER still gates acceptance via its own accept_input capability.
	bool bInput = true;
	FText StatusText;

	EActiveTimerReturnType RefreshTick(double, float);
	void Refresh();
	TSharedRef<ITableRow> OnGenerateRow(FDriverPtr Item, const TSharedRef<STableViewBase>& Table);
	FReply OnConnect(FDriverPtr Item);
	/** Package path for the selected environment ("" = bare plane). */
	FString SelectedLevelPath() const;
	/** Parse the origin box "X,Y,Z" into UE cm (zero on empty/malformed). */
	FVector ParseOrigin() const;
};
