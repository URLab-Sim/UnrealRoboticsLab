// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MjEffectiveDetails.h"

#include "MjArrayCustomizations.h"

#include "Containers/Ticker.h"
#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "Editor.h"
#include "EditorViewportClient.h"
#include "IDetailPropertyRow.h"
#include "IPropertyUtilities.h"
#include "Modules/ModuleManager.h"
#include "PropertyEditorModule.h"
#include "PropertyHandle.h"
#include "ScopedTransaction.h"
#include "Styling/SlateColor.h"
#include "UObject/PropertyOptional.h"
#include "UObject/UnrealType.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"

#include "MuJoCo/Spec/MjGenHooks.h"
#include "MuJoCo/Spec/MjNodeComponent.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Spec/MjEffective.h"

#endif  // URLAB_MJ_GEN

#define LOCTEXT_NAMESPACE "MjEffectiveDetails"

namespace
{

#if URLAB_MJ_GEN

using namespace urlab::spec;

/**
 * Attributes that look inheritable and are not.
 *
 * `MjName` is the identity attribute, and on a `<default>` partial it holds the
 * CLASS name -- so treating it as inheritable would tell every classed element
 * that its name came from its class. `Dclass` is the reference to the class
 * itself, which is what does the inheriting rather than a thing inherited.
 */
bool IsInheritable(const FProperty& Property)
{
	const FName Name = Property.GetFName();
	return Name != GET_MEMBER_NAME_CHECKED(UMjNodeComponent, MjName) && Name != FName(TEXT("Dclass"));
}

/**
 * Everything `Node` inherits, in ONE walk of its layers.
 *
 * The panel asks this question of about thirty attributes, and asking it thirty
 * times means dispatching the element's type thirty times and walking its
 * layers thirty times to read thirty fields off the same handful of objects.
 * The chain is the expensive part and it is the same chain every time, so it is
 * walked once and every attribute is read from each layer as it passes.
 *
 * Nearest layer wins, which is what the compiler's merge does: an attribute
 * already answered by a nearer layer is not overwritten by a further one. The
 * walk is `ForEachEffectiveLayer`, so the last layer offered is the schema's own
 * -- MuJoCo's value for an attribute no `<default>` in the document mentions,
 * which is most attributes of most elements, and the reason a panel that
 * consulted only the class chain showed a bare Set button on nearly every row.
 */
void CollectInherited(UMjNodeComponent& Node, const TArray<FOptionalProperty*>& Wanted,
	TMap<FName, FMjEffectiveValue>& Out)
{
	TArray<FMjEffectiveLayer> Layers;
	MjEffectiveLayersOf(Node, Layers);

	for (const FMjEffectiveLayer& Layer : Layers)
	{
		// The element itself is the first layer, and the question is what it
		// would inherit, not what it authored.
		if (Layer.Node == nullptr || Layer.Node == &Node)
		{
			continue;
		}
		for (FOptionalProperty* const Optional : Wanted)
		{
			if (Out.Contains(Optional->GetFName()))
			{
				continue;
			}
			// A hand subclass may declare properties the generated partial has
			// never heard of. None of those are spec attributes, but asking is
			// cheaper than assuming.
			if (!Layer.Node->GetClass()->IsChildOf(Optional->GetOwnerClass()))
			{
				continue;
			}
			const void* const Container = Optional->ContainerPtrToValuePtr<void>(Layer.Node);
			if (Container == nullptr || !Optional->IsSet(Container))
			{
				continue;
			}
			FMjEffectiveValue& Value = Out.Add(Optional->GetFName());
			Optional->GetValueProperty()->ExportTextItem_Direct(
				Value.Text, Optional->GetValuePointerForRead(Container), nullptr, nullptr, PPF_None);
			Value.ClassName = Layer.ClassName;
			Value.Source = Layer.bSchema ? EMjValueSource::Schema : EMjValueSource::Class;
		}
	}
}

/**
 * True while a viewport is dragging something.
 *
 * A gizmo delta refreshes every property window, and a refresh runs this
 * customization: on a model of a few hundred elements that is a whole-spec pass
 * per mouse-move, on top of the drag's own work, and it is paid to redraw rows
 * the user is not looking at because their hand is on a widget in the viewport.
 * The rows come back the moment the drag ends -- see the tick below, which is
 * what makes this a deferral rather than a loss.
 *
 * Every viewport client, not just the level editor's: the Blueprint editor has
 * its own, and the Blueprint editor is where a model is assembled.
 */
bool AnyViewportTracking()
{
	if (GEditor == nullptr)
	{
		return false;
	}
	for (const FEditorViewportClient* const Client : GEditor->GetAllViewportClients())
	{
		if (Client != nullptr && Client->IsTracking())
		{
			return true;
		}
	}
	return false;
}

/** Ask the panel to rebuild once the drag that deferred it has finished. */
void RefreshWhenTrackingEnds(TSharedPtr<IPropertyUtilities> Utilities)
{
	if (!Utilities.IsValid())
	{
		return;
	}
	FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda(
		[WeakUtilities = TWeakPtr<IPropertyUtilities>(Utilities)](float) {
			if (AnyViewportTracking())
			{
				return true;  // still dragging; ask again next tick
			}
			if (const TSharedPtr<IPropertyUtilities> Live = WeakUtilities.Pin())
			{
				Live->RequestForceRefresh();
			}
			return false;
		}));
}

/** Author `Text` onto `Node`'s `Optional`, as the user asking for it. */
void AuthorInherited(UMjNodeComponent& Node, const FOptionalProperty& Optional, const FString& Text)
{
	FScopedTransaction Transaction(LOCTEXT("AuthorInherited", "Author Inherited MuJoCo Value"));
	Node.Modify();

	void* const Container = Optional.ContainerPtrToValuePtr<void>(&Node);
	if (Container == nullptr)
	{
		return;
	}
	void* const Value = Optional.MarkSetAndGetInitializedValuePointerToReplace(Container);
	Optional.GetValueProperty()->ImportText_Direct(*Text, Value, &Node, PPF_None);

	FPropertyChangedEvent Event(const_cast<FOptionalProperty*>(&Optional), EPropertyChangeType::ValueSet);
	Node.PostEditChangeProperty(Event);
}

#endif  // URLAB_MJ_GEN

}  // namespace

TSharedRef<IDetailCustomization> FMjEffectiveDetails::MakeInstance()
{
	return MakeShareable(new FMjEffectiveDetails);
}

bool FMjEffectiveDetails::ResolveInherited(
	UMjNodeComponent& Node, const FOptionalProperty& Optional, FMjEffectiveValue& Out)
{
	Out = FMjEffectiveValue();
#if URLAB_MJ_GEN
	// The row builder's own collector, asked about one attribute. A second walk
	// written for the single-attribute question is a second answer that can
	// disagree with the rows, which is the class of bug this file keeps finding.
	TArray<FOptionalProperty*> Wanted;
	Wanted.Add(const_cast<FOptionalProperty*>(&Optional));

	TMap<FName, FMjEffectiveValue> Collected;
	CollectInherited(Node, Wanted, Collected);

	if (const FMjEffectiveValue* const Found = Collected.Find(Optional.GetFName()))
	{
		Out = *Found;
	}
#endif
	return Out.IsSet();
}

FText FMjEffectiveDetails::DescribeValue(const FMjEffectiveValue& Value)
{
	switch (Value.Source)
	{
		case EMjValueSource::Schema:
			// Named as what it is rather than as a class, because there is no
			// class to open: MuJoCo decides this one and the only way to change
			// it is to author the attribute here or on a `<default>`.
			return FText::FromString(FString::Printf(TEXT("%s  (default)"), *Value.Text));

		case EMjValueSource::Class:
			return Value.ClassName.IsEmpty()
				? FText::FromString(Value.Text)
				: FText::FromString(FString::Printf(TEXT("%s  (from %s)"), *Value.Text, *Value.ClassName));

		default:
			return FText::GetEmpty();
	}
}

void FMjEffectiveDetails::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
#if URLAB_MJ_GEN
	using namespace urlab::spec;

	TArray<TWeakObjectPtr<UObject>> Objects;
	DetailBuilder.GetObjectsBeingCustomized(Objects);
	if (Objects.Num() != 1)
	{
		// Multi-select has no single class chain to report, and a row that
		// showed one element's inheritance while editing several would be a
		// lie about the others.
		return;
	}
	UMjNodeComponent* const Node = Cast<UMjNodeComponent>(Objects[0].Get());
	if (Node == nullptr || Node->HasAnyFlags(RF_ClassDefaultObject))
	{
		return;
	}

	const TSharedPtr<IPropertyUtilities> Utilities = DetailBuilder.GetPropertyUtilities();

	// Not while the user is dragging. The engine refreshes every property
	// window per gizmo delta, and this pass reads the spec; deferring it to the
	// end of the drag is the difference between a drag that tracks the mouse and
	// one that appears to fight back. The ticker puts the rows back after.
	if (AnyViewportTracking())
	{
		RefreshWhenTrackingEnds(Utilities);
		return;
	}

	// ONE context for the whole refresh. Each attribute asks the same question
	// of the same spec, and building a context per question indexes every
	// element and every default class per question -- the quadratic the scope
	// exists to remove.
	FMjEffectiveScope Effective(*Node);

	// Inside the scope, because naming a `size` slot means knowing the shape,
	// and the shape is an effective value like any other.
	FMjArrayCustomizations::CustomizeArrays(DetailBuilder, *Node);
	FMjArrayCustomizations::AddEulerRow(DetailBuilder, *Node);

	// Every attribute the element leaves unset, asked as one question of the
	// class chain rather than as one question each.
	TArray<FOptionalProperty*> Unset;
	for (TFieldIterator<FOptionalProperty> It(Node->GetClass()); It; ++It)
	{
		FOptionalProperty* const Optional = *It;
		if (Optional == nullptr || !IsInheritable(*Optional))
		{
			continue;
		}
		const void* const Container = Optional->ContainerPtrToValuePtr<void>(Node);
		if (Container == nullptr || Optional->IsSet(Container))
		{
			continue;
		}
		Unset.Add(Optional);
	}

	TMap<FName, FMjEffectiveValue> Inherited;
	CollectInherited(*Node, Unset, Inherited);

	for (FOptionalProperty* const Optional : Unset)
	{
		const FMjEffectiveValue* const Found = Inherited.Find(Optional->GetFName());
		if (Found == nullptr)
		{
			continue;
		}

		const TSharedPtr<IPropertyHandle> Handle =
			DetailBuilder.GetProperty(Optional->GetFName(), Optional->GetOwnerClass());
		if (!Handle.IsValid() || !Handle->IsValidHandle())
		{
			continue;
		}
		IDetailPropertyRow* const Row = DetailBuilder.EditDefaultProperty(Handle);
		if (Row == nullptr)
		{
			continue;
		}

		TSharedPtr<SWidget> NameWidget;
		TSharedPtr<SWidget> ValueWidget;
		Row->GetDefaultWidgets(NameWidget, ValueWidget, /*bAddWidgetDecoration=*/true);
		if (!NameWidget.IsValid())
		{
			continue;
		}

		// The engine's own widget for an unset optional is a bare Set button that
		// value-initialises -- a geom inheriting `0.3` from its class gets a zero
		// -- and it is deliberately NOT put on these rows. Every row built here is
		// an unset attribute with a resolved value, so the only Set the user is
		// offered is the one below, which seeds what the row is showing them.
		const FText Display = FMjEffectiveDetails::DescribeValue(*Found);

		TWeakObjectPtr<UMjNodeComponent> WeakNode = Node;
		const FString Text = Found->Text;
		const bool bFromSchema = Found->Source == EMjValueSource::Schema;

		Row->CustomWidget(/*bShowChildren=*/true)
			.NameContent()[NameWidget.ToSharedRef()]
			.ValueContent()
			.MinDesiredWidth(250.0f)
			.MaxDesiredWidth(700.0f)
				[SNew(SHorizontalBox)
					+ SHorizontalBox::Slot()
						  .FillWidth(1.0f)
						  .VAlign(VAlign_Center)
						  .Padding(6.0f, 0.0f, 4.0f, 0.0f)
							  [SNew(STextBlock)
									  .Text(Display)
									  .ColorAndOpacity(FSlateColor::UseSubduedForeground())
									  .ToolTipText(bFromSchema
											  ? LOCTEXT("SchemaTip",
													"What the compiler will use: this element authors nothing and no "
													"default class mentions the attribute, so MuJoCo's own value "
													"decides it.")
											  : LOCTEXT("InheritedTip",
													"What the compiler will use: this element authors nothing, so the "
													"value comes from its default class. Editing the class changes "
													"it."))]
					+ SHorizontalBox::Slot()
						  .AutoWidth()
						  .VAlign(VAlign_Center)
							  [SNew(SButton)
									  .Text(LOCTEXT("AuthorInheritedLabel", "Set"))
									  .ToolTipText(LOCTEXT("AuthorInheritedTip",
										  "Author this value onto the element, seeded from the value shown. The "
										  "element then keeps it whatever its class does next."))
									  .OnClicked_Lambda([WeakNode, Optional, Text, Utilities]() -> FReply {
										  if (UMjNodeComponent* const Live = WeakNode.Get())
										  {
											  AuthorInherited(*Live, *Optional, Text);
											  if (Utilities.IsValid())
											  {
												  Utilities->RequestForceRefresh();
											  }
										  }
										  return FReply::Handled();
									  })]];
	}
#endif  // URLAB_MJ_GEN
}

void FMjEffectiveDetails::RegisterAll()
{
	FPropertyEditorModule& PropertyModule =
		FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
	PropertyModule.RegisterCustomClassLayout(UMjNodeComponent::StaticClass()->GetFName(),
		FOnGetDetailCustomizationInstance::CreateStatic(&FMjEffectiveDetails::MakeInstance));
	PropertyModule.NotifyCustomizationModuleChanged();
}

void FMjEffectiveDetails::UnregisterAll()
{
	if (!FModuleManager::Get().IsModuleLoaded("PropertyEditor"))
	{
		return;
	}
	FPropertyEditorModule& PropertyModule =
		FModuleManager::GetModuleChecked<FPropertyEditorModule>("PropertyEditor");
	PropertyModule.UnregisterCustomClassLayout(UMjNodeComponent::StaticClass()->GetFName());
}

#undef LOCTEXT_NAMESPACE
