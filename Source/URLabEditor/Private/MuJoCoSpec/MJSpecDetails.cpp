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

#include "MJSpecDetails.h"

#include "DetailCategoryBuilder.h"
#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "IDetailGroup.h"
#include "Modules/ModuleManager.h"
#include "MuJoCoSpec/MJActorComponents.h"
#include "MuJoCoSpec/MJSceneComponents.h"
#include "PropertyEditorModule.h"
#include "PropertyHandle.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "MJSpecDetails"

namespace
{
TArray<FName> GetShortcutParamFields(EMJActuatorShortcut Shortcut)
{
	switch (Shortcut)
	{
	case EMJActuatorShortcut::General:
		return {};
	case EMJActuatorShortcut::Motor:
		return {TEXT("Gain"), TEXT("CtrlSpec")};
	case EMJActuatorShortcut::Position:
		return {
			TEXT("Kp"),
			TEXT("bUseKv"), TEXT("Kv"),
			TEXT("bUseDampRatio"), TEXT("DampRatio"),
			TEXT("bUseTimeConst"), TEXT("TimeConst"),
			TEXT("InheritRange")
		};
	case EMJActuatorShortcut::Velocity:
		return {TEXT("Kv")};
	case EMJActuatorShortcut::IntVelocity:
		return {
			TEXT("Kp"),
			TEXT("bUseKv"), TEXT("Kv"),
			TEXT("bUseDampRatio"), TEXT("DampRatio"),
			TEXT("InheritRange")
		};
	case EMJActuatorShortcut::Damper:
		return {TEXT("Kv")};
	case EMJActuatorShortcut::Cylinder:
		return {TEXT("TimeConst"), TEXT("Area"), TEXT("Diameter"), TEXT("Bias")};
	case EMJActuatorShortcut::Muscle:
		return {TEXT("TimeConst"), TEXT("Area"), TEXT("Bias")};
	case EMJActuatorShortcut::Adhesion:
		return {TEXT("Gain"), TEXT("CtrlSpec")};
	case EMJActuatorShortcut::PID:
		return {
			TEXT("Kp"),
			TEXT("bUseKv"), TEXT("Kv"),
			TEXT("bUseKi"), TEXT("Ki"),
			TEXT("bUseIMax"), TEXT("IMax"),
			TEXT("bUseSlewMax"), TEXT("SlewMax")
		};
	case EMJActuatorShortcut::Orientation:
		return {
			TEXT("Kp"),
			TEXT("bUseKv"), TEXT("Kv"),
			TEXT("bUseDampRatio"), TEXT("DampRatio")
		};
	case EMJActuatorShortcut::DCMotor:
		return {TEXT("Kp"), TEXT("bUseKv"), TEXT("Kv"), TEXT("TimeConst"), TEXT("Gain"), TEXT("Bias")};
	default:
		return {};
	}
}

void CustomizeFromToSpec(IDetailLayoutBuilder& DetailBuilder, FName CategoryName,
	FName SpecPropertyName, FSimpleDelegate OnChanged)
{
	TSharedRef<IPropertyHandle> SpecHandle = DetailBuilder.GetProperty(SpecPropertyName);
	TSharedPtr<IPropertyHandle> OverrideFromTo = SpecHandle->GetChildHandle(TEXT("bOverride_FromTo"));
	TSharedPtr<IPropertyHandle> UseFromTo = SpecHandle->GetChildHandle(TEXT("bUseFromTo"));
	TSharedPtr<IPropertyHandle> FromHandle = SpecHandle->GetChildHandle(TEXT("From"));
	TSharedPtr<IPropertyHandle> ToHandle = SpecHandle->GetChildHandle(TEXT("To"));

	if (!OverrideFromTo.IsValid() || !UseFromTo.IsValid() || !FromHandle.IsValid() || !ToHandle.IsValid())
	{
		return;
	}

	OverrideFromTo->SetOnPropertyValueChanged(OnChanged);
	UseFromTo->SetOnPropertyValueChanged(OnChanged);

	bool bOverride = false;
	bool bUse = false;
	OverrideFromTo->GetValue(bOverride);
	UseFromTo->GetValue(bUse);
	const bool bFromToActive = bOverride && bUse;

	OverrideFromTo->MarkHiddenByCustomization();
	UseFromTo->MarkHiddenByCustomization();
	FromHandle->MarkHiddenByCustomization();
	ToHandle->MarkHiddenByCustomization();

	IDetailCategoryBuilder& Category = DetailBuilder.EditCategory(CategoryName);
	IDetailGroup& FromToGroup = Category.AddGroup(
		TEXT("FromTo"), LOCTEXT("FromToGroupLabel", "From-To Endpoints"), false, true);
	FromToGroup.AddPropertyRow(UseFromTo.ToSharedRef());
	if (bFromToActive)
	{
		FromToGroup.AddPropertyRow(FromHandle.ToSharedRef());
		FromToGroup.AddPropertyRow(ToHandle.ToSharedRef());
	}
}
} // namespace

TSharedRef<IDetailCustomization> FMJActuatorDetails::MakeInstance()
{
	return MakeShared<FMJActuatorDetails>();
}

void FMJActuatorDetails::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
	CachedDetailBuilder = &DetailBuilder;

	TSharedRef<IPropertyHandle> ShortcutHandle = DetailBuilder.GetProperty(
		GET_MEMBER_NAME_CHECKED(UMJActuatorComponent, Shortcut));
	TSharedRef<IPropertyHandle> ShortcutParamsHandle = DetailBuilder.GetProperty(
		GET_MEMBER_NAME_CHECKED(UMJActuatorComponent, ShortcutParams));
	TSharedRef<IPropertyHandle> TargetRefHandle = DetailBuilder.GetProperty(
		GET_MEMBER_NAME_CHECKED(UMJActuatorComponent, TargetRef));
	TSharedRef<IPropertyHandle> CtrlHandle = DetailBuilder.GetProperty(
		GET_MEMBER_NAME_CHECKED(UMJActuatorComponent, Ctrl));

	ShortcutHandle->SetOnPropertyValueChanged(
		FSimpleDelegate::CreateSP(this, &FMJActuatorDetails::OnShortcutChanged));

	EMJActuatorShortcut ActiveShortcut = EMJActuatorShortcut::General;
	uint8 ShortcutRaw = 0;
	if (ShortcutHandle->GetValue(ShortcutRaw) == FPropertyAccess::Success)
	{
		ActiveShortcut = static_cast<EMJActuatorShortcut>(ShortcutRaw);
	}

	DetailBuilder.HideProperty(ShortcutHandle);
	DetailBuilder.HideProperty(ShortcutParamsHandle);
	DetailBuilder.HideProperty(TargetRefHandle);
	DetailBuilder.HideProperty(CtrlHandle);

	IDetailCategoryBuilder& Category = DetailBuilder.EditCategory(TEXT("MuJoCo|Actuator"));
	Category.AddProperty(ShortcutHandle);
	Category.AddProperty(TargetRefHandle);
	Category.AddProperty(CtrlHandle);

	const TArray<FName> ActiveFields = GetShortcutParamFields(ActiveShortcut);
	if (ActiveFields.IsEmpty())
	{
		Category.AddCustomRow(LOCTEXT("GeneralShortcutFilter", "Shortcut Parameters"))
			.WholeRowContent()
			[
				SNew(STextBlock)
				.Text(LOCTEXT("GeneralShortcutNote", "General shortcut uses Spec fields directly."))
				.Font(IDetailLayoutBuilder::GetDetailFontItalic())
			];
	}
	else
	{
		IDetailGroup& ParamsGroup = Category.AddGroup(
			TEXT("ActiveShortcutParams"),
			LOCTEXT("ActiveShortcutParamsLabel", "Active Shortcut Parameters"),
			false,
			true);
		for (const FName FieldName : ActiveFields)
		{
			TSharedPtr<IPropertyHandle> Child = ShortcutParamsHandle->GetChildHandle(FieldName);
			if (Child.IsValid())
			{
				ParamsGroup.AddPropertyRow(Child.ToSharedRef());
			}
		}
	}
}

void FMJActuatorDetails::OnShortcutChanged()
{
	if (CachedDetailBuilder)
	{
		CachedDetailBuilder->ForceRefreshDetails();
	}
}

TSharedRef<IDetailCustomization> FMJGeomDetails::MakeInstance()
{
	return MakeShared<FMJGeomDetails>();
}

void FMJGeomDetails::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
	CachedDetailBuilder = &DetailBuilder;
	CustomizeFromToSpec(
		DetailBuilder,
		TEXT("MuJoCo|Geom"),
		GET_MEMBER_NAME_CHECKED(UMJGeomComponent, Spec),
		FSimpleDelegate::CreateSP(this, &FMJGeomDetails::OnFromToChanged));
}

void FMJGeomDetails::OnFromToChanged()
{
	if (CachedDetailBuilder)
	{
		CachedDetailBuilder->ForceRefreshDetails();
	}
}

TSharedRef<IDetailCustomization> FMJSiteDetails::MakeInstance()
{
	return MakeShared<FMJSiteDetails>();
}

void FMJSiteDetails::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
	CachedDetailBuilder = &DetailBuilder;
	CustomizeFromToSpec(
		DetailBuilder,
		TEXT("MuJoCo|Site"),
		GET_MEMBER_NAME_CHECKED(UMJSiteComponent, Spec),
		FSimpleDelegate::CreateSP(this, &FMJSiteDetails::OnFromToChanged));
}

void FMJSiteDetails::OnFromToChanged()
{
	if (CachedDetailBuilder)
	{
		CachedDetailBuilder->ForceRefreshDetails();
	}
}

void MJSpecDetails::RegisterAll()
{
	FPropertyEditorModule& PropertyModule =
		FModuleManager::LoadModuleChecked<FPropertyEditorModule>(TEXT("PropertyEditor"));
	PropertyModule.RegisterCustomClassLayout(
		UMJActuatorComponent::StaticClass()->GetFName(),
		FOnGetDetailCustomizationInstance::CreateStatic(&FMJActuatorDetails::MakeInstance));
	PropertyModule.RegisterCustomClassLayout(
		UMJGeomComponent::StaticClass()->GetFName(),
		FOnGetDetailCustomizationInstance::CreateStatic(&FMJGeomDetails::MakeInstance));
	PropertyModule.RegisterCustomClassLayout(
		UMJSiteComponent::StaticClass()->GetFName(),
		FOnGetDetailCustomizationInstance::CreateStatic(&FMJSiteDetails::MakeInstance));
}

void MJSpecDetails::UnregisterAll()
{
	if (!FModuleManager::Get().IsModuleLoaded(TEXT("PropertyEditor")))
	{
		return;
	}
	FPropertyEditorModule& PropertyModule =
		FModuleManager::GetModuleChecked<FPropertyEditorModule>(TEXT("PropertyEditor"));
	PropertyModule.UnregisterCustomClassLayout(UMJActuatorComponent::StaticClass()->GetFName());
	PropertyModule.UnregisterCustomClassLayout(UMJGeomComponent::StaticClass()->GetFName());
	PropertyModule.UnregisterCustomClassLayout(UMJSiteComponent::StaticClass()->GetFName());
}

#undef LOCTEXT_NAMESPACE
