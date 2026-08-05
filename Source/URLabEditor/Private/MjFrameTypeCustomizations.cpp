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

#include "MjFrameTypeCustomizations.h"

#include "DetailWidgetRow.h"
#include "Modules/ModuleManager.h"
#include "PropertyEditorModule.h"
#include "PropertyHandle.h"
#include "Widgets/Input/SNumericEntryBox.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "MjFrameTypes"

namespace
{
// The struct names UE's property system keys on: the USTRUCT name without its F.
const FName NAME_MjPosition3(TEXT("MjPosition3"));
const FName NAME_MjDirection3(TEXT("MjDirection3"));
const FName NAME_MjVec3(TEXT("MjVec3"));
const FName NAME_MjQuatRot(TEXT("MjQuatRot"));

TArray<FName> Xyz()
{
	return {FName(TEXT("X")), FName(TEXT("Y")), FName(TEXT("Z"))};
}

TArray<FName> Wxyz()
{
	// MJCF's own component order, which is also the storage order. Showing it
	// as W X Y Z rather than UE's X Y Z W is the point: a reader who sees
	// "X Y Z W" will assume FQuat and read the scalar out of the wrong slot.
	return {FName(TEXT("W")), FName(TEXT("X")), FName(TEXT("Y")), FName(TEXT("Z"))};
}

} // namespace

TSharedRef<IPropertyTypeCustomization> FMjFrameTypeCustomization::MakeInstance(
	TArray<FName> InComponents, FText InUnit)
{
	return MakeShareable(new FMjFrameTypeCustomization(MoveTemp(InComponents), MoveTemp(InUnit)));
}

void FMjFrameTypeCustomization::CustomizeHeader(TSharedRef<IPropertyHandle> PropertyHandle,
	FDetailWidgetRow& HeaderRow, IPropertyTypeCustomizationUtils& CustomizationUtils)
{
	TSharedRef<SHorizontalBox> Row = SNew(SHorizontalBox);

	for (const FName& Component : Components)
	{
		TSharedPtr<IPropertyHandle> Child = PropertyHandle->GetChildHandle(Component);
		if (!Child.IsValid())
		{
			continue;
		}

		const FText Label = FText::FromName(Component);
		Row->AddSlot()
			.FillWidth(1.0f)
			.Padding(0.0f, 0.0f, 4.0f, 0.0f)
				[SNew(SNumericEntryBox<double>)
						.AllowSpin(false)
						.Font(CustomizationUtils.GetRegularFont())
						.Label()[SNew(STextBlock).Text(Label)]
						.Value_Lambda([Child]() -> TOptional<double> {
							double Value = 0.0;
							return Child->GetValue(Value) == FPropertyAccess::Success
								? TOptional<double>(Value)
								: TOptional<double>();
						})
						.OnValueCommitted_Lambda([Child](double NewValue, ETextCommit::Type) {
							Child->SetValue(NewValue);
						})];
	}

	// The unit and frame, stated on the row. The panel edits the authored MJCF
	// value and converts nothing, so this is not decoration: it is the
	// difference between typing metres and typing centimetres.
	if (!Unit.IsEmpty())
	{
		Row->AddSlot()
			.AutoWidth()
			.VAlign(VAlign_Center)
				[SNew(STextBlock)
						.Text(Unit)
						.Font(CustomizationUtils.GetRegularFont())
						.ToolTipText(LOCTEXT("UnitTip",
							"Authored MJCF value: MuJoCo's right-handed frame and MuJoCo's units. "
							"The panel does not convert; this is exactly what the writer emits."))];
	}

	HeaderRow.NameContent()[PropertyHandle->CreatePropertyNameWidget()]
		.ValueContent()
		.MinDesiredWidth(250.0f)
		.MaxDesiredWidth(700.0f)[Row];
}

void FMjFrameTypeCustomization::CustomizeChildren(TSharedRef<IPropertyHandle>,
	IDetailChildrenBuilder&, IPropertyTypeCustomizationUtils&)
{
	// The header row already shows every component; expanding would repeat it.
}

void FMjFrameTypeCustomization::RegisterAll()
{
	FPropertyEditorModule& PropertyModule =
		FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");

	auto Register = [&PropertyModule](FName StructName, TArray<FName> Components, FText Unit) {
		PropertyModule.RegisterCustomPropertyTypeLayout(
			StructName,
			FOnGetPropertyTypeCustomizationInstance::CreateStatic(
				&FMjFrameTypeCustomization::MakeInstance, MoveTemp(Components), MoveTemp(Unit)));
	};

	Register(NAME_MjPosition3, Xyz(), LOCTEXT("UnitMetres", "m (MuJoCo frame)"));
	Register(NAME_MjDirection3, Xyz(), LOCTEXT("UnitDirection", "(MuJoCo frame)"));
	Register(NAME_MjVec3, Xyz(), FText::GetEmpty());
	Register(NAME_MjQuatRot, Wxyz(), LOCTEXT("UnitQuat", "w x y z (MuJoCo frame)"));
}

void FMjFrameTypeCustomization::UnregisterAll()
{
	if (!FModuleManager::Get().IsModuleLoaded("PropertyEditor"))
	{
		return;
	}
	FPropertyEditorModule& PropertyModule =
		FModuleManager::GetModuleChecked<FPropertyEditorModule>("PropertyEditor");
	for (const FName& StructName : {NAME_MjPosition3, NAME_MjDirection3, NAME_MjVec3, NAME_MjQuatRot})
	{
		PropertyModule.UnregisterCustomPropertyTypeLayout(StructName);
	}
}

#undef LOCTEXT_NAMESPACE
