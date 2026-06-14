// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "SMjBridgeServerToggle.h"

#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Text/STextBlock.h"
#include "Styling/AppStyle.h"
#include "Editor.h"

#include "MjBridgeServerSubsystem.h"

namespace
{
    UURLabBridgeServerSubsystem* GetSub()
    {
        return GEditor ? GEditor->GetEditorSubsystem<UURLabBridgeServerSubsystem>() : nullptr;
    }
}

void SURLabBridgeServerToggle::Construct(const FArguments& InArgs)
{
    CachedLabel = FText::FromString(TEXT("Bridge: -"));
    CachedButtonText = FText::FromString(TEXT("Start"));
    CachedColor = FLinearColor(0.35f, 0.35f, 0.35f, 1.0f);

    ChildSlot
    [
        SNew(SHorizontalBox)
        + SHorizontalBox::Slot()
        .AutoWidth()
        .VAlign(VAlign_Center)
        [
            SNew(SBorder)
            .BorderImage(FAppStyle::GetBrush("WhiteBrush"))
            .BorderBackgroundColor(this, &SURLabBridgeServerToggle::GetFillColor)
            .Padding(FMargin(6.f, 2.f))
            [
                SNew(STextBlock)
                .Text(this, &SURLabBridgeServerToggle::GetLabelText)
                .ColorAndOpacity(FLinearColor::White)
            ]
        ]
        + SHorizontalBox::Slot()
        .AutoWidth()
        .VAlign(VAlign_Center)
        .Padding(FMargin(4.f, 0.f, 0.f, 0.f))
        [
            SNew(SButton)
            .Text(this, &SURLabBridgeServerToggle::GetButtonText)
            .OnClicked(this, &SURLabBridgeServerToggle::OnButtonClicked)
        ]
    ];

    RegisterActiveTimer(0.5f,
        FWidgetActiveTimerDelegate::CreateSP(this, &SURLabBridgeServerToggle::OnPoll));
}

EActiveTimerReturnType SURLabBridgeServerToggle::OnPoll(double, float)
{
    UURLabBridgeServerSubsystem* Sub = GetSub();
    if (!Sub)
    {
        CachedLabel = FText::FromString(TEXT("Bridge: -"));
        CachedButtonText = FText::FromString(TEXT("Start"));
        CachedColor = FLinearColor(0.35f, 0.35f, 0.35f, 1.0f);
        return EActiveTimerReturnType::Continue;
    }

    if (Sub->IsRunning())
    {
        CachedLabel = FText::FromString(FString::Printf(
            TEXT("Bridge: running (port %d)"), Sub->GetConfig().StepPort));
        CachedButtonText = FText::FromString(TEXT("Stop"));
        CachedColor = FLinearColor(0.18f, 0.55f, 0.20f, 1.0f);  // green
    }
    else
    {
        CachedLabel = FText::FromString(TEXT("Bridge: stopped"));
        CachedButtonText = FText::FromString(TEXT("Start"));
        CachedColor = FLinearColor(0.40f, 0.40f, 0.40f, 1.0f);  // grey
    }
    return EActiveTimerReturnType::Continue;
}

FReply SURLabBridgeServerToggle::OnButtonClicked()
{
    if (UURLabBridgeServerSubsystem* Sub = GetSub())
    {
        if (Sub->IsRunning()) Sub->StopServer();
        else                  Sub->StartServer();
        OnPoll(0, 0);  // refresh immediately, don't wait for poll tick
    }
    return FReply::Handled();
}
