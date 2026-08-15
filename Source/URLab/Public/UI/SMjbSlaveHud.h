// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"

class UMjbRenderSlaveSubsystem;
class SEditableTextBox;

/**
 * On-screen controls shown while a render slave is joined: a button back to the
 * server browser, and live spawn-origin nudge controls (X/Y/Z +/- with a step
 * size) so the operator can tune where the MJB sits in the level in real time
 * instead of guessing the offset up front.
 */
class SMjbSlaveHud : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SMjbSlaveHud) {}
		SLATE_ARGUMENT(TWeakObjectPtr<UMjbRenderSlaveSubsystem>, Subsystem)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

private:
	TWeakObjectPtr<UMjbRenderSlaveSubsystem> Subsystem;
	TSharedPtr<SEditableTextBox> StepBox;

	double Step() const;
	FReply Nudge(int32 Axis, int32 Sign); // Axis 0/1/2 = X/Y/Z, Sign -1/+1
	FText OriginText() const;
};
