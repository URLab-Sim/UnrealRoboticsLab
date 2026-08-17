// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "SMjServerBrowser.h"

#include "URLabEditorLogging.h"

#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/SBoxPanel.h"
#include "Misc/MessageDialog.h"

#define LOCTEXT_NAMESPACE "MjbServerBrowser"

void SMjServerBrowser::Construct(const FArguments& InArgs)
{
	StatusText = LOCTEXT("Idle", "Discovering fast-path drivers...");

	ChildSlot
	[
		SNew(SVerticalBox)

		// Header: title + manual refresh.
		+ SVerticalBox::Slot().AutoHeight().Padding(6)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
			[
				SNew(STextBlock)
				.Text(LOCTEXT("Title", "Fast-Path Drivers"))
			]
			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SButton)
				.Text(LOCTEXT("Refresh", "Refresh"))
				.OnClicked_Lambda([this]() { Refresh(); return FReply::Handled(); })
			]
		]

		// Driver list.
		+ SVerticalBox::Slot().FillHeight(1.0f).Padding(6, 0)
		[
			SNew(SBorder)
			[
				SAssignNew(ListView, SListView<FDriverPtr>)
				.ListItemsSource(&Drivers)
				.OnGenerateRow(this, &SMjServerBrowser::OnGenerateRow)
				.SelectionMode(ESelectionMode::Single)
			]
		]

		// Status line.
		+ SVerticalBox::Slot().AutoHeight().Padding(6)
		[
			SNew(STextBlock).Text_Lambda([this]() { return StatusText; })
		]
	];

	Refresh();
	// Auto-refresh so drivers appear/disappear without the user clicking.
	RegisterActiveTimer(2.0f,
		FWidgetActiveTimerDelegate::CreateSP(this, &SMjServerBrowser::RefreshTick));
}

EActiveTimerReturnType SMjServerBrowser::RefreshTick(double, float)
{
	Refresh();
	return EActiveTimerReturnType::Continue;
}

void SMjServerBrowser::Refresh()
{
	TArray<URLabLevelOps::FMjDriverInfo> Found;
	FString Err;
	URLabLevelOps::DiscoverFastPathDrivers(Found, Err);

	Drivers.Reset(Found.Num());
	for (const URLabLevelOps::FMjDriverInfo& D : Found)
	{
		Drivers.Add(MakeShared<URLabLevelOps::FMjDriverInfo>(D));
	}
	if (!Err.IsEmpty())
	{
		StatusText = FText::FromString(FString::Printf(TEXT("Discovery error: %s"), *Err));
	}
	else if (Drivers.Num() == 0)
	{
		StatusText = LOCTEXT("None", "No fast-path drivers advertised. Start a driver (e.g. run_fastpath_demo.py).");
	}
	else
	{
		StatusText = FText::FromString(FString::Printf(TEXT("%d driver(s) available."), Drivers.Num()));
	}
	if (ListView.IsValid())
	{
		ListView->RequestListRefresh();
	}
}

TSharedRef<ITableRow> SMjServerBrowser::OnGenerateRow(FDriverPtr Item, const TSharedRef<STableViewBase>& OwnerTable)
{
	const FString Label = FString::Printf(TEXT("%s   @ %s   (%d geoms)   %s"),
		*Item->Scene, *Item->Host, Item->Ngeom, *Item->Control);

	return SNew(STableRow<FDriverPtr>, OwnerTable)
	[
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center).Padding(4, 2)
		[
			SNew(STextBlock).Text(FText::FromString(Label))
		]
		+ SHorizontalBox::Slot().AutoWidth().Padding(4, 2)
		[
			SNew(SButton)
			.Text(LOCTEXT("Connect", "Connect"))
			.OnClicked(this, &SMjServerBrowser::OnConnectClicked, Item)
		]
	];
}

FReply SMjServerBrowser::OnConnectClicked(FDriverPtr Item)
{
	if (!Item.IsValid())
	{
		return FReply::Handled();
	}
	FString Err;
	const bool bOk = URLabLevelOps::LaunchFastPathFromDriverSync(
		Item->Control, /*bFreshLevel=*/true, Err);
	if (bOk)
	{
		StatusText = FText::FromString(FString::Printf(
			TEXT("Connected to '%s'. Press Play to stream the live puppet."), *Item->Scene));
	}
	else
	{
		StatusText = FText::FromString(FString::Printf(TEXT("Connect failed: %s"), *Err));
		UE_LOG(LogURLabEditor, Error, TEXT("[MjServerBrowser] connect failed: %s"), *Err);
	}
	return FReply::Handled();
}

#undef LOCTEXT_NAMESPACE
