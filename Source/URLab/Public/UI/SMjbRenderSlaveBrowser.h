// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/Views/SListView.h"
#include "MuJoCo/Fast/MjbOwnerDiscovery.h"

class UMjbRenderSlaveSubsystem;
class STextComboBox;
class SEditableTextBox;

/**
 * Runtime (packaged-game) server browser for fast-path render slaves. Lists the
 * live owners the UMjbRenderSlaveSubsystem discovered, an environment picker (bare
 * plane or any cooked level), a spawn-origin field, and a camera-feeds toggle.
 * Connecting hands the choice back to the subsystem, which pulls the MJB and opens
 * the level. This is the runtime counterpart to the editor's SMjbServerBrowser.
 */
class SMjbRenderSlaveBrowser : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SMjbRenderSlaveBrowser) {}
		SLATE_ARGUMENT(TWeakObjectPtr<UMjbRenderSlaveSubsystem>, Subsystem)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

private:
	using FOwnerPtr = TSharedPtr<FMjbOwnerInfo>;

	TWeakObjectPtr<UMjbRenderSlaveSubsystem> Subsystem;

	TArray<FOwnerPtr> Owners;
	TSharedPtr<SListView<FOwnerPtr>> OwnerList;

	TArray<TSharedPtr<FString>> LevelNames;
	TSharedPtr<STextComboBox> LevelCombo;
	TSharedPtr<FString> SelectedLevelName;

	TSharedPtr<SEditableTextBox> OriginBox;
	bool bCameras = true;
	FText StatusText;

	EActiveTimerReturnType RefreshTick(double, float);
	void Refresh();
	TSharedRef<ITableRow> OnGenerateRow(FOwnerPtr Item, const TSharedRef<STableViewBase>& Table);
	FReply OnConnect(FOwnerPtr Item);
	/** Package path for the selected environment ("" = bare plane). */
	FString SelectedLevelPath() const;
	/** Parse the origin box "X,Y,Z" into UE cm (zero on empty/malformed). */
	FVector ParseOrigin() const;
};
