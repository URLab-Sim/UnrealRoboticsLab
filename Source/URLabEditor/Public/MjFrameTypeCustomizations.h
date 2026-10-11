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

#pragma once

#include "CoreMinimal.h"
#include "IPropertyTypeCustomization.h"

// Details-panel rows for the MuJoCo frame types (MuJoCo/Spec/MjFrameTypes.h).
//
// Without these, an FMjPosition3 renders as an expandable struct with three
// child rows -- three clicks to see a number that FVector shows inline. UE's
// own vector row is nothing more magic than a name-keyed registration
// (DetailCustomizations.cpp registers FVectorStructCustomization against
// NAME_Vector), so one registration per frame type reproduces it.
//
// What the frame types get that FVector cannot: the row states the unit and the
// frame. A spec position is metres in MuJoCo's right-handed frame, and the
// panel says so, because the panel does NOT convert -- it edits exactly what
// the writer will emit.

/**
 * A single-row editor for a fixed set of double components, with a unit label.
 *
 * One class serves every frame type; the component names and the unit come from
 * the registration, so adding a type is a registration and not a subclass.
 */
class FMjFrameTypeCustomization : public IPropertyTypeCustomization
{
public:
	/** Bound with CreateStatic payload args, one binding per registered type. */
	static TSharedRef<IPropertyTypeCustomization> MakeInstance(TArray<FName> InComponents, FText InUnit);

	// IPropertyTypeCustomization
	virtual void CustomizeHeader(TSharedRef<IPropertyHandle> PropertyHandle, FDetailWidgetRow& HeaderRow,
		IPropertyTypeCustomizationUtils& CustomizationUtils) override;
	virtual void CustomizeChildren(TSharedRef<IPropertyHandle> PropertyHandle,
		IDetailChildrenBuilder& ChildBuilder, IPropertyTypeCustomizationUtils& CustomizationUtils) override;

	/** Register (and unregister) every frame type's row. */
	static void RegisterAll();
	static void UnregisterAll();

private:
	FMjFrameTypeCustomization(TArray<FName> InComponents, FText InUnit)
		: Components(MoveTemp(InComponents)), Unit(MoveTemp(InUnit))
	{
	}

	TArray<FName> Components;
	FText Unit;
};
