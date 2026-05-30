// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/DeclarativeSyntaxSupport.h"

/**
 * @class SURLabBridgeServerToggle
 * @brief Toolbar widget: small pill + Start/Stop button for the URLab
 *        bridge server. Polls UURLabBridgeServerSubsystem every 0.5s to
 *        keep the label in sync; clicking the button forwards to
 *        Start/StopServer on the subsystem.
 *
 *  Sits next to SMjStepModeIndicator in the LevelEditor PlayToolBar.
 */
class SURLabBridgeServerToggle : public SCompoundWidget
{
public:
    SLATE_BEGIN_ARGS(SURLabBridgeServerToggle) {}
    SLATE_END_ARGS()

    void Construct(const FArguments& InArgs);

private:
    EActiveTimerReturnType OnPoll(double, float);
    FReply OnButtonClicked();

    FText GetLabelText() const { return CachedLabel; }
    FSlateColor GetFillColor() const { return CachedColor; }
    FText GetButtonText() const { return CachedButtonText; }

    FText CachedLabel;
    FText CachedButtonText;
    FLinearColor CachedColor = FLinearColor(0.4f, 0.4f, 0.4f, 1.0f);
};
