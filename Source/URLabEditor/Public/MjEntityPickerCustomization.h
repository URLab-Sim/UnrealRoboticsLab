// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"
#include "IPropertyTypeCustomization.h"

#include "MuJoCo/Entity/MjEntityMembers.h"

class AActor;
template <typename T> class SComboBox;

// Details-panel row for the entity part pickers (MjEntityPickers.h). The picker is an entity ref plus
// the name of one of its parts; typed free-hand a name is a guess, so the name renders as a dropdown
// of the referenced entity's member names -- the same names the runtime resolves against. One class
// serves all three pickers; which family it lists comes from the registration.

/**
 * A joint / actuator / geom name dropdown driven by the picker's referenced entity.
 *
 * The referenced entity and the compiled model are read live: the options come from
 * MjEntityMembers::Names for the entity the picker points at, rebuilt each time the dropdown opens so
 * a fresh compile is reflected without reopening the panel. With no reachable compiled entity the list
 * is empty and the stored name still shows.
 */
class FMjEntityPickerCustomization : public IPropertyTypeCustomization
{
public:
	/** Bound with the family payload, one binding per registered picker struct. */
	static TSharedRef<IPropertyTypeCustomization> MakeInstance(EMjEntityMember InFamily);

	// IPropertyTypeCustomization
	virtual void CustomizeHeader(TSharedRef<IPropertyHandle> PropertyHandle, FDetailWidgetRow& HeaderRow,
		IPropertyTypeCustomizationUtils& CustomizationUtils) override;
	virtual void CustomizeChildren(TSharedRef<IPropertyHandle> PropertyHandle,
		IDetailChildrenBuilder& ChildBuilder, IPropertyTypeCustomizationUtils& CustomizationUtils) override;

	/** Register (and unregister) every picker struct's row. */
	static void RegisterAll();
	static void UnregisterAll();

private:
	explicit FMjEntityPickerCustomization(EMjEntityMember InFamily) : Family(InFamily) {}

	/** The actor the picker references, resolved from the Entity soft ref, or null. */
	AActor* ReferencedActor() const;

	/** Recompute the member-name options from the referenced entity's compiled model. */
	void RebuildOptions();

	/** The name currently stored on the picker, for the closed-combo label. */
	FText CurrentNameText() const;

	EMjEntityMember Family;
	TSharedPtr<IPropertyHandle> EntityHandle;
	TSharedPtr<IPropertyHandle> NameHandle;
	TArray<TSharedPtr<FName>> Options;
	TSharedPtr<SComboBox<TSharedPtr<FName>>> ComboBox;
};
