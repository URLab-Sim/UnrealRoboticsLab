// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MjEntityPickerCustomization.h"

#include "DetailWidgetRow.h"
#include "IDetailChildrenBuilder.h"
#include "Modules/ModuleManager.h"
#include "PropertyEditorModule.h"
#include "PropertyHandle.h"
#include "GameFramework/Actor.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Text/STextBlock.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Entity/MjEntityActor.h"
#include "MuJoCo/Entity/MjEntityPickers.h"

#define LOCTEXT_NAMESPACE "MjEntityPickers"

namespace
{
	// The struct names UE's property system keys on: the USTRUCT name without its F.
	const FName NAME_MjJointPicker(TEXT("MjJointPicker"));
	const FName NAME_MjActuatorPicker(TEXT("MjActuatorPicker"));
	const FName NAME_MjGeomPicker(TEXT("MjGeomPicker"));

	// The entity name a referenced actor stands for: an AMjEntity carries it directly; any other actor
	// (an articulation used as the authoring reference) names its entity by its own name.
	FName EntityNameOf(const AActor* Actor)
	{
		if (Actor == nullptr)
		{
			return NAME_None;
		}
		if (const AMjEntity* Entity = Cast<AMjEntity>(Actor))
		{
			return Entity->GetEntityName();
		}
		return FName(*Actor->GetName());
	}
}

TSharedRef<IPropertyTypeCustomization> FMjEntityPickerCustomization::MakeInstance(EMjEntityMember InFamily)
{
	return MakeShareable(new FMjEntityPickerCustomization(InFamily));
}

AActor* FMjEntityPickerCustomization::ReferencedActor() const
{
	if (!EntityHandle.IsValid())
	{
		return nullptr;
	}
	UObject* Value = nullptr;
	return EntityHandle->GetValue(Value) == FPropertyAccess::Success ? Cast<AActor>(Value) : nullptr;
}

void FMjEntityPickerCustomization::RebuildOptions()
{
	Options.Reset();
	const AActor* Actor = ReferencedActor();
	const UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(Actor);
	for (const FName& Member : MjEntityMembers::Names(Engine, EntityNameOf(Actor), Family))
	{
		Options.Add(MakeShared<FName>(Member));
	}
}

FText FMjEntityPickerCustomization::CurrentNameText() const
{
	FName Current;
	if (NameHandle.IsValid() && NameHandle->GetValue(Current) == FPropertyAccess::Success && !Current.IsNone())
	{
		return FText::FromName(Current);
	}
	return LOCTEXT("PickName", "Select...");
}

void FMjEntityPickerCustomization::CustomizeHeader(TSharedRef<IPropertyHandle> PropertyHandle,
	FDetailWidgetRow& HeaderRow, IPropertyTypeCustomizationUtils& CustomizationUtils)
{
	HeaderRow.NameContent()[PropertyHandle->CreatePropertyNameWidget()]
		.ValueContent()[SNew(STextBlock)
							.Font(CustomizationUtils.GetRegularFont())
							.Text_Lambda([this]() { return CurrentNameText(); })];
}

void FMjEntityPickerCustomization::CustomizeChildren(TSharedRef<IPropertyHandle> PropertyHandle,
	IDetailChildrenBuilder& ChildBuilder, IPropertyTypeCustomizationUtils& CustomizationUtils)
{
	EntityHandle = PropertyHandle->GetChildHandle(GET_MEMBER_NAME_CHECKED(FMjJointPicker, Entity));
	NameHandle = PropertyHandle->GetChildHandle(GET_MEMBER_NAME_CHECKED(FMjJointPicker, Name));

	if (EntityHandle.IsValid())
	{
		ChildBuilder.AddProperty(EntityHandle.ToSharedRef());
	}
	if (!NameHandle.IsValid())
	{
		return;
	}

	RebuildOptions();

	ChildBuilder.AddCustomRow(LOCTEXT("NameRow", "Name"))
		.NameContent()[NameHandle->CreatePropertyNameWidget()]
		.ValueContent()
		.MinDesiredWidth(250.0f)
			[SAssignNew(ComboBox, SComboBox<TSharedPtr<FName>>)
					.OptionsSource(&Options)
					.OnComboBoxOpening_Lambda([this]() {
						RebuildOptions();
						if (ComboBox.IsValid())
						{
							ComboBox->RefreshOptions();
						}
					})
					.OnGenerateWidget_Lambda([](TSharedPtr<FName> Item) {
						return SNew(STextBlock).Text(FText::FromName(Item.IsValid() ? *Item : NAME_None));
					})
					.OnSelectionChanged_Lambda([this](TSharedPtr<FName> Item, ESelectInfo::Type) {
						if (Item.IsValid() && NameHandle.IsValid())
						{
							NameHandle->SetValue(*Item);
						}
					})[SNew(STextBlock).Text_Lambda([this]() { return CurrentNameText(); })]];
}

void FMjEntityPickerCustomization::RegisterAll()
{
	FPropertyEditorModule& PropertyModule =
		FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");

	auto Register = [&PropertyModule](FName StructName, EMjEntityMember Family) {
		PropertyModule.RegisterCustomPropertyTypeLayout(
			StructName,
			FOnGetPropertyTypeCustomizationInstance::CreateStatic(
				&FMjEntityPickerCustomization::MakeInstance, Family));
	};

	Register(NAME_MjJointPicker, EMjEntityMember::Joint);
	Register(NAME_MjActuatorPicker, EMjEntityMember::Actuator);
	Register(NAME_MjGeomPicker, EMjEntityMember::Geom);
}

void FMjEntityPickerCustomization::UnregisterAll()
{
	if (!FModuleManager::Get().IsModuleLoaded("PropertyEditor"))
	{
		return;
	}
	FPropertyEditorModule& PropertyModule =
		FModuleManager::GetModuleChecked<FPropertyEditorModule>("PropertyEditor");
	for (const FName& StructName : {NAME_MjJointPicker, NAME_MjActuatorPicker, NAME_MjGeomPicker})
	{
		PropertyModule.UnregisterCustomPropertyTypeLayout(StructName);
	}
}

#undef LOCTEXT_NAMESPACE
