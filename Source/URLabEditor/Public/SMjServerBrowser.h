// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/Views/SListView.h"
#include "MjLevelOps.h"

/**
 * @class SMjServerBrowser
 * @brief Editor panel that lists advertised fast-path owners and connects to one.
 *
 * Polls the shared registry (URLabLevelOps::DiscoverFastPathOwners) every couple
 * of seconds and shows each live owner (scene, host, geom count, endpoint). The
 * Connect button pulls the owner's MJB over the wire and stands up a fast-path
 * render scene in the editor (URLabLevelOps::LaunchFastPathFromDriverSync) -- the
 * GUI equivalent of the -URLabFastConnect / -URLabFastDiscover CLI flags.
 */
class SMjServerBrowser : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SMjServerBrowser) {}
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

private:
	using FOwnerPtr = TSharedPtr<URLabLevelOps::FMjDriverInfo>;

	TSharedRef<ITableRow> OnGenerateRow(FOwnerPtr Item, const TSharedRef<STableViewBase>& Owner);
	EActiveTimerReturnType RefreshTick(double InCurrentTime, float InDeltaTime);
	void Refresh();
	FReply OnConnectClicked(FOwnerPtr Item);

	TArray<FOwnerPtr> Owners;
	TSharedPtr<SListView<FOwnerPtr>> ListView;
	FText StatusText;
};
