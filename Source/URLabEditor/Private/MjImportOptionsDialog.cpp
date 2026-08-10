// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MjImportOptionsDialog.h"

#include "MuJoCo/Spec/MjSpecRef.h"

#include "Framework/Application/SlateApplication.h"
#include "Misc/Paths.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SUniformGridPanel.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SWindow.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "URLabImportOptions"

bool ShowMjImportOptionsDialog(const FString& SourceXmlPath, const FText& PreparationStatus,
	FMjDocParseOptions& InOutOptions)
{
	if (!FSlateApplication::IsInitialized())
	{
		return true;
	}

	bool bAccepted = false;
	bool bAllowExternalIncludes = InOutOptions.bAllowExternalIncludes;

	const TSharedRef<SWindow> Window = SNew(SWindow)
		.Title(LOCTEXT("Title", "Import MuJoCo Model"))
		.SizingRule(ESizingRule::Autosized)
		.SupportsMinimize(false)
		.SupportsMaximize(false);

	Window->SetContent(
		SNew(SBorder)
		.Padding(16.f)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight().Padding(0.f, 0.f, 0.f, 12.f)
			[
				SNew(STextBlock).Text(FText::FromString(FPaths::GetCleanFilename(SourceXmlPath)))
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0.f, 0.f, 0.f, 12.f)
			[
				SNew(STextBlock).Text(PreparationStatus).WrapTextAt(420.f)
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0.f, 0.f, 0.f, 16.f)
			[
				SNew(SCheckBox)
				.IsChecked_Lambda([&bAllowExternalIncludes]() {
					return bAllowExternalIncludes ? ECheckBoxState::Checked : ECheckBoxState::Unchecked;
				})
				.OnCheckStateChanged_Lambda([&bAllowExternalIncludes](ECheckBoxState State) {
					bAllowExternalIncludes = (State == ECheckBoxState::Checked);
				})
				.ToolTipText(LOCTEXT("ExternalIncludesTooltip",
					"An <include> whose path escapes this model's own directory tree can name any "
					"file on this machine, and whatever it names is read into the model. Leave this "
					"off unless you trust the file you are importing."))
				[
					SNew(STextBlock).Text(LOCTEXT("ExternalIncludes",
						"Allow <include> outside the model's folder"))
				]
			]
			+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Right)
			[
				SNew(SUniformGridPanel).SlotPadding(4.f)
				+ SUniformGridPanel::Slot(0, 0)
				[
					SNew(SButton)
					.HAlign(HAlign_Center)
					.Text(LOCTEXT("Import", "Import"))
					.OnClicked_Lambda([&bAccepted, Window]() {
						bAccepted = true;
						FSlateApplication::Get().RequestDestroyWindow(Window);
						return FReply::Handled();
					})
				]
				+ SUniformGridPanel::Slot(1, 0)
				[
					SNew(SButton)
					.HAlign(HAlign_Center)
					.Text(LOCTEXT("Cancel", "Cancel"))
					.OnClicked_Lambda([Window]() {
						FSlateApplication::Get().RequestDestroyWindow(Window);
						return FReply::Handled();
					})
				]
			]
		]);

	FSlateApplication::Get().AddModalWindow(Window, nullptr);

	if (bAccepted)
	{
		InOutOptions.bAllowExternalIncludes = bAllowExternalIncludes;
	}
	return bAccepted;
}

#undef LOCTEXT_NAMESPACE
