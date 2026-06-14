// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#include "SMjStepModeIndicator.h"

#include "Widgets/SBoxPanel.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Text/STextBlock.h"
#include "Styling/CoreStyle.h"
#include "Styling/AppStyle.h"
#include "MuJoCo/Core/AMjManager.h"
#include "Bridge/RpcDispatcher.h"

void SMjStepModeIndicator::Construct(const FArguments& InArgs)
{
    CachedLabel = FText::FromString(TEXT("URLab: -"));
    CachedColor = FLinearColor(0.35f, 0.35f, 0.35f, 1.0f);

    ChildSlot
    [
        SNew(SBorder)
        .BorderImage(FAppStyle::GetBrush("WhiteBrush"))
        .BorderBackgroundColor(this, &SMjStepModeIndicator::GetFillColor)
        .Padding(FMargin(6.f, 2.f))
        [
            SNew(STextBlock)
            .Text(this, &SMjStepModeIndicator::GetLabel)
            .ColorAndOpacity(FLinearColor::White)
        ]
    ];

    // Poll every 0.5s — manager is sticky, no need for per-frame work.
    RegisterActiveTimer(0.5f, FWidgetActiveTimerDelegate::CreateSP(this, &SMjStepModeIndicator::OnPoll));
}

EActiveTimerReturnType SMjStepModeIndicator::OnPoll(double, float)
{
    AAMjManager* Mgr = AAMjManager::Instance;
    if (!Mgr)
    {
        CachedLabel = FText::FromString(TEXT("URLab: -"));
        CachedColor = FLinearColor(0.35f, 0.35f, 0.35f, 1.0f);
        return EActiveTimerReturnType::Continue;
    }

    // Read the live runtime mode from the dispatcher, not the static
    // UPROPERTY default (Mgr->StepMode is the configured pin, not the
    // currently-active mode after a client set_mode promotion).
    EStepMode Mode = Mgr->StepMode;
    if (FURLabRpcDispatcher* Disp = Mgr->GetStepDispatcher())
    {
        Mode = Disp->GetActiveStepMode();
    }

    switch (Mode)
    {
    case EStepMode::Live:
        CachedLabel = FText::FromString(TEXT("URLab: live"));
        CachedColor = FLinearColor(0.18f, 0.55f, 0.20f, 1.0f);  // green
        break;
    case EStepMode::Direct:
        CachedLabel = FText::FromString(TEXT("URLab: direct"));
        CachedColor = FLinearColor(0.85f, 0.55f, 0.10f, 1.0f);  // amber
        break;
    case EStepMode::Puppet:
        CachedLabel = FText::FromString(TEXT("URLab: puppet"));
        CachedColor = FLinearColor(0.20f, 0.45f, 0.85f, 1.0f);  // blue
        break;
    case EStepMode::Auto:
        CachedLabel = FText::FromString(TEXT("URLab: auto"));
        CachedColor = FLinearColor(0.40f, 0.40f, 0.40f, 1.0f);  // grey
        break;
    }
    return EActiveTimerReturnType::Continue;
}
