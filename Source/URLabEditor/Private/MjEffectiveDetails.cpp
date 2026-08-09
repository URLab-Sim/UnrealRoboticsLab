// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MjEffectiveDetails.h"

#include "MjArrayCustomizations.h"

#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "IDetailPropertyRow.h"
#include "IPropertyUtilities.h"
#include "Modules/ModuleManager.h"
#include "PropertyEditorModule.h"
#include "PropertyHandle.h"
#include "ScopedTransaction.h"
#include "Styling/SlateColor.h"
#include <type_traits>
#include "UObject/PropertyOptional.h"
#include "UObject/UnrealType.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"

#include "MuJoCo/Spec/MjGenHooks.h"
#include "MuJoCo/Spec/MjNodeComponent.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Gen/Elements/Defaults/MjDefault.gen.h"
#include "MuJoCo/Gen/MjDispatch.gen.h"
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

/** The class a `<default>` partial belongs to, as MuJoCo names it. */
template <class P>
FString ClassNameOf(const UMjNodeComponent& Layer)
{
	UMjNodeComponent* const Parent = P::Tree::ParentOf(Layer);
	if (Parent == nullptr || Cast<UMjDefault>(Parent) == nullptr)
	{
		return FString();
	}
	// MuJoCo's own name for the root `<default>`, which authors no class name
	// because everything inherits from it.
	return Parent->MjName.Get(TEXT("main"));
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

bool FMjEffectiveDetails::ResolveInherited(UMjNodeComponent& Node, const FOptionalProperty& Optional,
	FString& OutText, FString& OutClassName)
{
	OutText.Reset();
	OutClassName.Reset();
#if URLAB_MJ_GEN
	using namespace urlab::spec;

	bool bFound = false;
	WithEffectiveDoc(Node, [&](auto& Effective) {
		using P = typename std::decay_t<decltype(Effective)>::ProfileType;
		gen::DispatchByType(Node, [&](auto& Element) {
			Effective.ForEachLayer(Element, [&](const auto& Layer) {
				const UMjNodeComponent& LayerNode = static_cast<const UMjNodeComponent&>(Layer);
				// The element itself is the first layer, and the question is
				// what it would inherit, not what it authored.
				if (&LayerNode == &Node)
				{
					return false;
				}
				// A hand subclass may declare properties the generated partial
				// has never heard of. None of those are spec attributes, but
				// asking is cheaper than assuming.
				if (!LayerNode.GetClass()->IsChildOf(Optional.GetOwnerClass()))
				{
					return false;
				}
				const void* const Container = Optional.ContainerPtrToValuePtr<void>(&LayerNode);
				if (Container == nullptr || !Optional.IsSet(Container))
				{
					return false;
				}
				Optional.GetValueProperty()->ExportTextItem_Direct(OutText,
					Optional.GetValuePointerForRead(Container), nullptr, nullptr, PPF_None);
				OutClassName = ClassNameOf<P>(LayerNode);
				bFound = true;
				return true;
			});
		});
	});
	return bFound;
#else
	return false;
#endif
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

	// ONE context for the whole refresh. Each attribute asks the same question
	// of the same spec, and building a context per question indexes every
	// element and every default class per question -- the quadratic the scope
	// exists to remove.
	FMjEffectiveScope Effective(*Node);

	const TSharedPtr<IPropertyUtilities> Utilities = DetailBuilder.GetPropertyUtilities();

	// Inside the scope, because naming a `size` slot means knowing the shape,
	// and the shape is an effective value like any other.
	FMjArrayCustomizations::CustomizeArrays(DetailBuilder, *Node);
	FMjArrayCustomizations::AddEulerRow(DetailBuilder, *Node);

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

		FString InheritedText;
		FString InheritedClass;
		if (!ResolveInherited(*Node, *Optional, InheritedText, InheritedClass))
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
		if (!NameWidget.IsValid() || !ValueWidget.IsValid())
		{
			continue;
		}

		const FText Display = InheritedClass.IsEmpty()
			? FText::FromString(InheritedText)
			: FText::FromString(FString::Printf(TEXT("%s  (from %s)"), *InheritedText, *InheritedClass));

		TWeakObjectPtr<UMjNodeComponent> WeakNode = Node;
		const FString Text = InheritedText;

		Row->CustomWidget(/*bShowChildren=*/true)
			.NameContent()[NameWidget.ToSharedRef()]
			.ValueContent()
			.MinDesiredWidth(250.0f)
			.MaxDesiredWidth(700.0f)
				[SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().AutoWidth()[ValueWidget.ToSharedRef()]
					+ SHorizontalBox::Slot()
						  .FillWidth(1.0f)
						  .VAlign(VAlign_Center)
						  .Padding(6.0f, 0.0f, 4.0f, 0.0f)
							  [SNew(STextBlock)
									  .Text(Display)
									  .ColorAndOpacity(FSlateColor::UseSubduedForeground())
									  .ToolTipText(LOCTEXT("InheritedTip",
										  "What the compiler will use: this element authors nothing, so the "
										  "value comes from its default class. Editing the class changes it."))]
					+ SHorizontalBox::Slot()
						  .AutoWidth()
						  .VAlign(VAlign_Center)
							  [SNew(SButton)
									  .Text(LOCTEXT("AuthorInheritedLabel", "Use"))
									  .ToolTipText(LOCTEXT("AuthorInheritedTip",
										  "Author this value onto the element, seeded from the class. The "
										  "element then keeps it whatever the class does next."))
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
