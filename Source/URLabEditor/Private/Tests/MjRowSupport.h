// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Reading the details panel the way a user reads it.
//
// Both panel customizations -- the inherited-value rows and the array slot
// labels -- produce Slate, and what a test wants to know about them is what the
// user sees: the text on the row, and what the button on it does. That means
// running the customization through `IPropertyRowGenerator`, which is the same
// machinery a details panel runs, and then walking the widgets it hands back.
//
// One copy of that, because two suites doing it separately would drift into
// asking the property system slightly different questions while both claiming
// to test the panel.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "IDetailTreeNode.h"
#include "IPropertyRowGenerator.h"
#include "Layout/Children.h"
#include "Misc/AutomationTest.h"
#include "Modules/ModuleManager.h"
#include "PropertyEditorModule.h"
#include "PropertyHandle.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/SWidget.h"
#include "Widgets/Text/STextBlock.h"

#include "MuJoCo/Spec/MjNodeComponent.h"

namespace MjRowSupport
{

/** Every widget in the tree under `Widget`, itself included. */
inline void CollectWidgets(const TSharedRef<SWidget>& Widget, TArray<TSharedRef<SWidget>>& Out)
{
	Out.Add(Widget);
	FChildren* const Children = Widget->GetChildren();
	if (Children == nullptr)
	{
		return;
	}
	for (int32 Index = 0; Index < Children->Num(); ++Index)
	{
		CollectWidgets(Children->GetChildAt(Index), Out);
	}
}

/** Every piece of text a widget tree is showing, joined. */
inline FString TextOf(const TSharedPtr<SWidget>& Root)
{
	if (!Root.IsValid())
	{
		return FString();
	}
	TArray<TSharedRef<SWidget>> Widgets;
	CollectWidgets(Root.ToSharedRef(), Widgets);

	TArray<FString> Lines;
	for (const TSharedRef<SWidget>& Widget : Widgets)
	{
		if (Widget->GetType() == TEXT("STextBlock"))
		{
			Lines.Add(StaticCastSharedRef<STextBlock>(Widget)->GetText().ToString());
		}
	}
	return FString::Join(Lines, TEXT(" | "));
}

/** The first button in a widget tree, which is the one a customization put there. */
inline TSharedPtr<SButton> ButtonIn(const TSharedPtr<SWidget>& Root)
{
	if (!Root.IsValid())
	{
		return nullptr;
	}
	TArray<TSharedRef<SWidget>> Widgets;
	CollectWidgets(Root.ToSharedRef(), Widgets);
	for (const TSharedRef<SWidget>& Widget : Widgets)
	{
		if (Widget->GetType() == TEXT("SButton"))
		{
			return StaticCastSharedRef<SButton>(Widget);
		}
	}
	return nullptr;
}

/** The rows a details panel builds for one element. */
class FRows
{
public:
	bool Build(FAutomationTestBase& Test, UMjNodeComponent& Component)
	{
		FPropertyEditorModule& Module = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
		FPropertyRowGeneratorArgs Args;
		Args.bAllowMultipleTopLevelObjects = false;
		Generator = Module.CreatePropertyRowGenerator(Args);
		if (!Generator.IsValid())
		{
			Test.AddError(TEXT("the property editor produced no row generator"));
			return false;
		}

		TArray<UObject*> Objects;
		Objects.Add(&Component);
		Generator->SetObjects(Objects);

		Index(Generator->GetRootTreeNodes(), NAME_None);

		if (All.Num() == 0)
		{
			Test.AddError(TEXT("the row generator produced no rows at all"));
			return false;
		}
		return true;
	}

	bool Has(const TCHAR* Property) const { return ByProperty.Contains(FName(Property)); }

	/** The value column of the row for `Property`, or an invalid pointer. */
	TSharedPtr<SWidget> ValueOf(const TCHAR* Property) const
	{
		const TSharedRef<IDetailTreeNode>* const Node = ByProperty.Find(FName(Property));
		if (Node == nullptr)
		{
			return nullptr;
		}
		const FNodeWidgets Widgets = (*Node)->CreateNodeWidgets();
		return Widgets.ValueWidget.IsValid() ? Widgets.ValueWidget : Widgets.WholeRowWidget;
	}

	/**
	 * Everything every row is showing, joined.
	 *
	 * The way to find a row a customization ADDED rather than customized: it
	 * carries no property, so there is nothing to look it up by except its text.
	 */
	FString AllText() const
	{
		TArray<FString> Lines;
		for (const TSharedRef<IDetailTreeNode>& Node : All)
		{
			const FNodeWidgets Widgets = Node->CreateNodeWidgets();
			Lines.Add(TextOf(Widgets.NameWidget));
			Lines.Add(TextOf(Widgets.ValueWidget));
			Lines.Add(TextOf(Widgets.WholeRowWidget));
		}
		return FString::Join(Lines, TEXT(" | "));
	}

private:
	/**
	 * Flatten the tree, remembering one row per attribute: the one a panel shows.
	 *
	 * Two things make "the node carrying this property" ambiguous, and both are
	 * settled here rather than at each assertion.
	 *
	 * A generated attribute's category is `MuJoCo|<Element>`, so its row is built
	 * once under the plain category and once under the subcategory, and the
	 * customization -- which reaches it through `EditDefaultProperty` -- edits the
	 * subcategory's copy, which comes second. Later wins.
	 *
	 * And a container's own children carry the container's property: an optional's
	 * value, an array's elements. Those are the insides of the row, not the row,
	 * so a node whose property its ancestor already carries is not indexed. Without
	 * that, "later wins" would hand back the last element of an array.
	 */
	void Index(const TArray<TSharedRef<IDetailTreeNode>>& Nodes, FName Enclosing)
	{
		for (const TSharedRef<IDetailTreeNode>& Node : Nodes)
		{
			All.Add(Node);

			const TSharedPtr<IPropertyHandle> Handle = Node->CreatePropertyHandle();
			const FProperty* const Property = Handle.IsValid() ? Handle->GetProperty() : nullptr;
			const FName Own = Property != nullptr ? Property->GetFName() : NAME_None;
			if (Own != NAME_None && Own != Enclosing)
			{
				ByProperty.Add(Own, Node);
			}

			TArray<TSharedRef<IDetailTreeNode>> Children;
			Node->GetChildren(Children, /*bInIgnoreVisibility=*/true);
			Index(Children, Own != NAME_None ? Own : Enclosing);
		}
	}

	TSharedPtr<IPropertyRowGenerator> Generator;
	TArray<TSharedRef<IDetailTreeNode>> All;
	TMap<FName, TSharedRef<IDetailTreeNode>> ByProperty;
};

} // namespace MjRowSupport

#endif // URLAB_MJ_GEN && WITH_EDITOR
