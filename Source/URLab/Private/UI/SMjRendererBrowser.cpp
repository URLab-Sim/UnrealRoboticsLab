// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "UI/SMjRendererBrowser.h"

#include "MuJoCo/Fast/MjRendererSubsystem.h"
#include "Utils/URLabLogging.h"

#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SUniformGridPanel.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/STextComboBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/SBoxPanel.h"

#define LOCTEXT_NAMESPACE "MjRendererBrowser"

void SMjRendererBrowser::Construct(const FArguments& InArgs)
{
	Subsystem = InArgs._Subsystem;
	StatusText = LOCTEXT("Idle", "Discovering fast-path drivers...");

	if (Subsystem.IsValid())
	{
		Subsystem->RefreshLevels();
		for (const FMjRendererLevelChoice& L : Subsystem->GetLevels())
		{
			LevelNames.Add(MakeShared<FString>(L.Name));
		}
	}
	if (LevelNames.Num() > 0)
	{
		SelectedLevelName = LevelNames[0];
	}

	ChildSlot
	.HAlign(HAlign_Center)
	.VAlign(VAlign_Center)
	[
		SNew(SBorder)
		.Padding(16)
		[
			SNew(SBox).WidthOverride(720).HeightOverride(520)
			[
				SNew(SVerticalBox)

				// Title + refresh.
				+ SVerticalBox::Slot().AutoHeight().Padding(4)
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
					[
						SNew(STextBlock).Text(LOCTEXT("Title", "URLab Renderer  --  Fast-Path Drivers"))
					]
					+ SHorizontalBox::Slot().AutoWidth()
					[
						SNew(SButton)
						.Text(LOCTEXT("Refresh", "Refresh"))
						.OnClicked_Lambda([this]() { Refresh(); return FReply::Handled(); })
					]
				]

				// Environment row: level + origin + cameras.
				+ SVerticalBox::Slot().AutoHeight().Padding(4, 8)
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 6, 0)
					[
						SNew(STextBlock).Text(LOCTEXT("Env", "Environment:"))
					]
					+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center)
					[
						SAssignNew(LevelCombo, STextComboBox)
						.OptionsSource(&LevelNames)
						.InitiallySelectedItem(SelectedLevelName)
						.OnSelectionChanged_Lambda([this](TSharedPtr<FString> New, ESelectInfo::Type)
						{
							SelectedLevelName = New;
						})
					]
					+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(8, 0, 4, 0)
					[
						SNew(STextBlock).Text(LOCTEXT("Origin", "Origin X,Y,Z:"))
					]
					+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
					[
						SNew(SBox).WidthOverride(220)
						[
							SAssignNew(OriginBox, SEditableTextBox).Text(FText::FromString(TEXT("0,0,0")))
						]
					]
					+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(8, 0, 0, 0)
					[
						SNew(SCheckBox)
						.IsChecked(bCameras ? ECheckBoxState::Checked : ECheckBoxState::Unchecked)
						.OnCheckStateChanged_Lambda([this](ECheckBoxState S)
						{
							bCameras = (S == ECheckBoxState::Checked);
						})
						[
							SNew(STextBlock).Text(LOCTEXT("Cameras", "Camera feeds"))
						]
					]
				]

				// Driver list.
				+ SVerticalBox::Slot().FillHeight(1.0f).Padding(4)
				[
					SNew(SBorder)
					[
						SAssignNew(DriverList, SListView<FDriverPtr>)
						.ListItemsSource(&Drivers)
						.OnGenerateRow(this, &SMjRendererBrowser::OnGenerateRow)
						.SelectionMode(ESelectionMode::Single)
					]
				]

				// Status.
				+ SVerticalBox::Slot().AutoHeight().Padding(4)
				[
					SNew(STextBlock).Text_Lambda([this]() { return StatusText; })
				]
			]
		]
	];

	Refresh();
	RegisterActiveTimer(2.0f,
		FWidgetActiveTimerDelegate::CreateSP(this, &SMjRendererBrowser::RefreshTick));
}

EActiveTimerReturnType SMjRendererBrowser::RefreshTick(double, float)
{
	Refresh();
	return EActiveTimerReturnType::Continue;
}

void SMjRendererBrowser::Refresh()
{
	UMjRendererSubsystem* Sub = Subsystem.Get();
	if (!Sub)
	{
		return;
	}
	Sub->RefreshDrivers();
	Drivers.Reset();
	for (const FMjDriverInfo& D : Sub->GetDrivers())
	{
		Drivers.Add(MakeShared<FMjDriverInfo>(D));
	}
	if (Drivers.Num() == 0)
	{
		StatusText = LOCTEXT("None", "No fast-path drivers advertised. Start a driver (e.g. menagerie_swap.py).");
	}
	else
	{
		StatusText = FText::FromString(FString::Printf(TEXT("%d driver(s) available."), Drivers.Num()));
	}
	if (DriverList.IsValid())
	{
		DriverList->RequestListRefresh();
	}
}

TSharedRef<ITableRow> SMjRendererBrowser::OnGenerateRow(FDriverPtr Item, const TSharedRef<STableViewBase>& Table)
{
	const FString Label = FString::Printf(TEXT("%s   @ %s   (%d geoms)   %s"),
		*Item->Scene, *Item->Host, Item->Ngeom, *Item->Control);

	return SNew(STableRow<FDriverPtr>, Table)
	[
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().FillWidth(1.0f).VAlign(VAlign_Center).Padding(6, 3)
		[
			SNew(STextBlock).Text(FText::FromString(Label))
		]
		+ SHorizontalBox::Slot().AutoWidth().Padding(6, 3)
		[
			SNew(SButton)
			.Text(LOCTEXT("Connect", "Connect"))
			.OnClicked(this, &SMjRendererBrowser::OnConnect, Item)
		]
	];
}

FString SMjRendererBrowser::SelectedLevelPath() const
{
	UMjRendererSubsystem* Sub = Subsystem.Get();
	if (!Sub || !SelectedLevelName.IsValid())
	{
		return FString();
	}
	for (const FMjRendererLevelChoice& L : Sub->GetLevels())
	{
		if (L.Name == *SelectedLevelName)
		{
			return L.Path;
		}
	}
	return FString();
}

FVector SMjRendererBrowser::ParseOrigin() const
{
	if (!OriginBox.IsValid())
	{
		return FVector::ZeroVector;
	}
	TArray<FString> Parts;
	OriginBox->GetText().ToString().ParseIntoArray(Parts, TEXT(","));
	if (Parts.Num() != 3)
	{
		return FVector::ZeroVector;
	}
	return FVector(FCString::Atod(*Parts[0].TrimStartAndEnd()),
		FCString::Atod(*Parts[1].TrimStartAndEnd()), FCString::Atod(*Parts[2].TrimStartAndEnd()));
}

FReply SMjRendererBrowser::OnConnect(FDriverPtr Item)
{
	UMjRendererSubsystem* Sub = Subsystem.Get();
	if (!Item.IsValid() || !Sub)
	{
		return FReply::Handled();
	}
	FString Err;
	if (Sub->JoinDriver(*Item, SelectedLevelPath(), ParseOrigin(), bCameras, Err))
	{
		StatusText = FText::FromString(FString::Printf(TEXT("Joining '%s'..."), *Item->Scene));
	}
	else
	{
		StatusText = FText::FromString(FString::Printf(TEXT("Connect failed: %s"), *Err));
	}
	return FReply::Handled();
}

#undef LOCTEXT_NAMESPACE
