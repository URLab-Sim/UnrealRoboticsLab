// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "UI/SMjRendererHud.h"

#include "MuJoCo/Fast/MjRendererSubsystem.h"

#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/SBoxPanel.h"

#define LOCTEXT_NAMESPACE "MjRendererHud"

void SMjRendererHud::Construct(const FArguments& InArgs)
{
	Subsystem = InArgs._Subsystem;

	auto AxisRow = [this](const TCHAR* Label, int32 Axis) -> TSharedRef<SWidget>
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(2, 0)
			[
				SNew(SBox).WidthOverride(16)[ SNew(STextBlock).Text(FText::FromString(Label)) ]
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(2, 0)
			[
				SNew(SButton).Text(LOCTEXT("Minus", "  -  "))
				.OnClicked(this, &SMjRendererHud::Nudge, Axis, -1)
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(2, 0)
			[
				SNew(SButton).Text(LOCTEXT("Plus", "  +  "))
				.OnClicked(this, &SMjRendererHud::Nudge, Axis, 1)
			];
	};

	ChildSlot
	.HAlign(HAlign_Left)
	.VAlign(VAlign_Top)
	[
		SNew(SBorder).Padding(8)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight().Padding(2)
			[
				SNew(SButton)
				.Text(LOCTEXT("Back", "<  Server Browser"))
				.OnClicked_Lambda([this]()
				{
					if (UMjRendererSubsystem* S = Subsystem.Get())
					{
						S->ReturnToBrowser();
					}
					return FReply::Handled();
				})
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(2, 6, 2, 2)
			[
				SNew(STextBlock).Text_Lambda([this]() { return OriginText(); })
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(2)
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 4, 0)
				[
					SNew(STextBlock).Text(LOCTEXT("Step", "step (cm):"))
				]
				+ SHorizontalBox::Slot().AutoWidth()
				[
					SNew(SBox).WidthOverride(64)
					[
						SAssignNew(StepBox, SEditableTextBox).Text(FText::FromString(TEXT("25")))
					]
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(2)[ AxisRow(TEXT("X"), 0) ]
			+ SVerticalBox::Slot().AutoHeight().Padding(2)[ AxisRow(TEXT("Y"), 1) ]
			+ SVerticalBox::Slot().AutoHeight().Padding(2)[ AxisRow(TEXT("Z"), 2) ]
		]
	];
}

double SMjRendererHud::Step() const
{
	if (StepBox.IsValid())
	{
		const double V = FCString::Atod(*StepBox->GetText().ToString());
		if (!FMath::IsNearlyZero(V))
		{
			return V;
		}
	}
	return 25.0;
}

FReply SMjRendererHud::Nudge(int32 Axis, int32 Sign)
{
	if (UMjRendererSubsystem* S = Subsystem.Get())
	{
		FVector Delta = FVector::ZeroVector;
		Delta[Axis] = Sign * Step();
		S->NudgeOrigin(Delta);
	}
	return FReply::Handled();
}

FText SMjRendererHud::OriginText() const
{
	const UMjRendererSubsystem* S = Subsystem.Get();
	const FVector O = S ? S->GetOrigin() : FVector::ZeroVector;
	return FText::FromString(
		FString::Printf(TEXT("Origin  X=%.0f  Y=%.0f  Z=%.0f"), O.X, O.Y, O.Z));
}

#undef LOCTEXT_NAMESPACE
