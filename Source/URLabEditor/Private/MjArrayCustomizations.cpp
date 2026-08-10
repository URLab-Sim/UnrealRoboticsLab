// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MjArrayCustomizations.h"

#include "DetailCategoryBuilder.h"
#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "IDetailPropertyRow.h"
#include "PropertyHandle.h"
#include "ScopedTransaction.h"
#include "Styling/SlateColor.h"
#include "UObject/Class.h"
#include "UObject/PropertyOptional.h"
#include "UObject/UnrealType.h"
#include "Widgets/Input/SNumericEntryBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"

#include "MjEffectiveDetails.h"
#include "MuJoCo/Spec/MjFrameTypes.h"
#include "MuJoCo/Spec/MjGenHooks.h"
#include "MuJoCo/Spec/MjNodeComponent.h"

#if URLAB_MJ_GEN
#include "MuJoCo/Gen/Elements/Geometry/MjGeom.gen.h"
#include "MuJoCo/Gen/Elements/Geometry/MjSite.gen.h"
#include "MuJoCo/Gen/MjEnums.gen.h"
#include "MuJoCo/Spec/MjElementIdentity.h"
#endif

#define LOCTEXT_NAMESPACE "MjArrayCustomizations"

namespace
{

#if URLAB_MJ_GEN

/** How many slots the schema gives `Node`'s `Xml` attribute; 0 when it has none. */
int32 SchemaArityOf(const UMjNodeComponent& Node, const char* Xml)
{
	urlab::spec::psm::ElementType Type{};
	if (!urlab::spec::MjElementTypeOfNode(Node, Type))
	{
		return 0;
	}
	return urlab::spec::MjSchemaArityOf(Type, Xml);
}

/**
 * What each `friction` slot means, for the element it is on.
 *
 * `friction` is two different attributes wearing one name. A `<geom>` takes
 * three coefficients -- one sliding, one torsional, one rolling -- and a
 * `<pair>` takes FIVE, because a contact pair names two tangential directions
 * and two rolling ones. Labelling the pair's five slots with the geom's three
 * names told the user that slot 1 was torsional friction when it is the second
 * sliding coefficient, which is a wrong number typed into a real model.
 *
 * Which form applies is read off the schema's own arity rather than from a list
 * of element types kept here, so a MuJoCo release that gives some other element
 * a five-slot friction is labelled correctly the day it is generated -- and one
 * that is neither three nor five is labelled by index rather than by a guess.
 */
TArray<FText> FrictionLabelsFor(const UMjNodeComponent& Node)
{
	switch (SchemaArityOf(Node, "friction"))
	{
		case 3:
			return {LOCTEXT("FrictionSliding", "sliding"), LOCTEXT("FrictionTorsional", "torsional"),
				LOCTEXT("FrictionRolling", "rolling")};
		case 5:
			return {LOCTEXT("FrictionSliding1", "sliding 1"), LOCTEXT("FrictionSliding2", "sliding 2"),
				LOCTEXT("FrictionTorsional5", "torsional"), LOCTEXT("FrictionRolling1", "rolling 1"),
				LOCTEXT("FrictionRolling2", "rolling 2")};
		default:
			return {};
	}
}

#endif  // URLAB_MJ_GEN

/** MuJoCo's own names for the slots, in MuJoCo's own order. */
TArray<FText> LabelsFor(const UMjNodeComponent& Node, const FName& Attribute)
{
	if (Attribute == FName(TEXT("Friction")))
	{
#if URLAB_MJ_GEN
		return FrictionLabelsFor(Node);
#else
		return {};
#endif
	}
	if (Attribute == FName(TEXT("Solref")))
	{
		return {LOCTEXT("SolrefTimeconst", "timeconst"), LOCTEXT("SolrefDampratio", "dampratio")};
	}
	if (Attribute == FName(TEXT("Solimp")))
	{
		return {LOCTEXT("SolimpDmin", "dmin"), LOCTEXT("SolimpDmax", "dmax"),
			LOCTEXT("SolimpWidth", "width"), LOCTEXT("SolimpMidpoint", "midpoint"),
			LOCTEXT("SolimpPower", "power")};
	}
	if (Attribute == FName(TEXT("Gear")))
	{
		return {LOCTEXT("GearX", "x"), LOCTEXT("GearY", "y"), LOCTEXT("GearZ", "z"),
			LOCTEXT("GearRx", "rx"), LOCTEXT("GearRy", "ry"), LOCTEXT("GearRz", "rz")};
	}
	return {};
}

#if URLAB_MJ_GEN

/**
 * The shape `size` is being read against.
 *
 * Authored first, then the default class -- a menagerie geom that says
 * `class="visual"` and nothing else is the ordinary case, and labelling its
 * three numbers as a sphere's one radius would be worse than not labelling
 * them.
 */
bool EffectiveShapeName(UMjNodeComponent& Node, EMjGeomType& OutType)
{
	const UMjGeomBase* const Geom = Cast<UMjGeomBase>(&Node);
	const UMjSite* const Site = Cast<UMjSite>(&Node);
	if (Geom == nullptr && Site == nullptr)
	{
		return false;
	}
	const TOptional<EMjGeomType>& Authored = Geom != nullptr ? Geom->Type : Site->Type;
	if (Authored.IsSet())
	{
		OutType = Authored.GetValue();
		return true;
	}

	const FOptionalProperty* const Property =
		CastField<FOptionalProperty>(Node.GetClass()->FindPropertyByName(FName(TEXT("Type"))));
	if (Property == nullptr)
	{
		return false;
	}
	FMjEffectiveValue Resolved;
	if (!FMjEffectiveDetails::ResolveInherited(Node, *Property, Resolved))
	{
		// MuJoCo's own default for both families.
		OutType = EMjGeomType::sphere;
		return true;
	}
	const UEnum* const Enum = StaticEnum<EMjGeomType>();
	const int64 Value = Enum != nullptr ? Enum->GetValueByNameString(Resolved.Text) : INDEX_NONE;
	if (Value == INDEX_NONE)
	{
		return false;
	}
	OutType = static_cast<EMjGeomType>(Value);
	return true;
}

/** What each `size` slot means for the shape it is sizing. */
TArray<FText> SizeLabelsFor(EMjGeomType Type)
{
	switch (Type)
	{
		case EMjGeomType::sphere:
			return {LOCTEXT("SizeRadius", "radius")};
		case EMjGeomType::capsule:
		case EMjGeomType::cylinder:
			return {LOCTEXT("SizeRadius2", "radius"), LOCTEXT("SizeHalfLength", "half-length")};
		case EMjGeomType::box:
		case EMjGeomType::ellipsoid:
			return {LOCTEXT("SizeHalfX", "half-x"), LOCTEXT("SizeHalfY", "half-y"),
				LOCTEXT("SizeHalfZ", "half-z")};
		case EMjGeomType::plane:
			return {LOCTEXT("SizeHalfX2", "half-x"), LOCTEXT("SizeHalfY2", "half-y"),
				LOCTEXT("SizeSpacing", "grid spacing")};
		default:
			// A mesh, an sdf or a height field takes its size from the asset,
			// so there is nothing here to name.
			return {};
	}
}

#endif  // URLAB_MJ_GEN

/**
 * The array inside a property handle, however deeply the optional wraps it.
 *
 * A `TOptional<TArray<double>>` is one node with one child in the property
 * tree; a bare `TArray<double>` is the array already. Null when the handle is
 * neither, which is the signal to leave the row alone rather than to guess.
 */
TSharedPtr<IPropertyHandle> ArrayWithin(const TSharedPtr<IPropertyHandle>& Handle)
{
	TSharedPtr<IPropertyHandle> Current = Handle;
	for (int32 Depth = 0; Depth < 2 && Current.IsValid() && Current->IsValidHandle(); ++Depth)
	{
		if (Current->AsArray().IsValid())
		{
			return Current;
		}
		uint32 NumChildren = 0;
		if (Current->GetNumChildren(NumChildren) != FPropertyAccess::Success || NumChildren != 1)
		{
			return nullptr;
		}
		Current = Current->GetChildHandle(0);
	}
	return nullptr;
}

/** The category a generated attribute declares, so an added row lands beside it. */
FName CategoryOf(const FProperty& Property)
{
	const FString Category = Property.GetMetaData(TEXT("Category"));
	return Category.IsEmpty() ? FName(TEXT("MuJoCo")) : FName(*Category);
}

// --- Euler, in MuJoCo's frame ------------------------------------------ //
//
// Done on raw doubles in MJCF's own [w, x, y, z] order rather than through
// FQuat, deliberately. The spec's quaternion is right-handed and Unreal's is
// not, and the two also disagree about which operand of a product applies
// first; borrowing the engine's type for the arithmetic would put two silent
// sign errors between what the user types and what the writer emits.

/** Hamilton product, [w, x, y, z] both sides. */
void QuatMultiply(const double A[4], const double B[4], double Out[4])
{
	Out[0] = A[0] * B[0] - A[1] * B[1] - A[2] * B[2] - A[3] * B[3];
	Out[1] = A[0] * B[1] + A[1] * B[0] + A[2] * B[3] - A[3] * B[2];
	Out[2] = A[0] * B[2] - A[1] * B[3] + A[2] * B[0] + A[3] * B[1];
	Out[3] = A[0] * B[3] + A[1] * B[2] - A[2] * B[1] + A[3] * B[0];
}

}  // namespace

void FMjArrayCustomizations::EulerDegreesToQuat(const double Degrees[3], double Out[4])
{
	double Result[4] = {1.0, 0.0, 0.0, 0.0};
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		const double Half = FMath::DegreesToRadians(Degrees[Axis]) * 0.5;
		double Term[4] = {FMath::Cos(Half), 0.0, 0.0, 0.0};
		Term[Axis + 1] = FMath::Sin(Half);

		double Composed[4];
		QuatMultiply(Result, Term, Composed);
		for (int32 Index = 0; Index < 4; ++Index)
		{
			Result[Index] = Composed[Index];
		}
	}
	for (int32 Index = 0; Index < 4; ++Index)
	{
		Out[Index] = Result[Index];
	}
}

void FMjArrayCustomizations::QuatToEulerDegrees(const double Quat[4], double OutDegrees[3])
{
	const double W = Quat[0];
	const double X = Quat[1];
	const double Y = Quat[2];
	const double Z = Quat[3];

	// Rows of R = Rx(rx) * Ry(ry) * Rz(rz), which is what "xyz" composes.
	const double R00 = 1.0 - 2.0 * (Y * Y + Z * Z);
	const double R01 = 2.0 * (X * Y - W * Z);
	const double R02 = 2.0 * (X * Z + W * Y);
	const double R12 = 2.0 * (Y * Z - W * X);
	const double R22 = 1.0 - 2.0 * (X * X + Y * Y);

	OutDegrees[1] = FMath::RadiansToDegrees(FMath::Asin(FMath::Clamp(R02, -1.0, 1.0)));
	OutDegrees[0] = FMath::RadiansToDegrees(FMath::Atan2(-R12, R22));
	OutDegrees[2] = FMath::RadiansToDegrees(FMath::Atan2(-R01, R00));
}

namespace
{

/** The element's authored `quat`, or the identity when it authors none. */
bool ReadQuat(const UMjNodeComponent& Node, const FOptionalProperty& Property, double Out[4])
{
	Out[0] = 1.0;
	Out[1] = Out[2] = Out[3] = 0.0;

	const void* const Container = Property.ContainerPtrToValuePtr<void>(&Node);
	if (Container == nullptr || !Property.IsSet(Container))
	{
		return false;
	}
	const FMjQuatRot* const Quat = static_cast<const FMjQuatRot*>(Property.GetValuePointerForRead(Container));
	if (Quat == nullptr)
	{
		return false;
	}
	Out[0] = Quat->W;
	Out[1] = Quat->X;
	Out[2] = Quat->Y;
	Out[3] = Quat->Z;
	return true;
}

/** Author `quat` from an edited euler triple. */
void WriteQuat(UMjNodeComponent& Node, const FOptionalProperty& Property, const double In[4])
{
	FScopedTransaction Transaction(LOCTEXT("EditEuler", "Edit MuJoCo Orientation"));
	Node.Modify();

	void* const Container = Property.ContainerPtrToValuePtr<void>(&Node);
	if (Container == nullptr)
	{
		return;
	}
	FMjQuatRot* const Quat =
		static_cast<FMjQuatRot*>(Property.MarkSetAndGetInitializedValuePointerToReplace(Container));
	if (Quat == nullptr)
	{
		return;
	}
	Quat->W = In[0];
	Quat->X = In[1];
	Quat->Y = In[2];
	Quat->Z = In[3];

	FPropertyChangedEvent Event(const_cast<FOptionalProperty*>(&Property), EPropertyChangeType::ValueSet);
	Node.PostEditChangeProperty(Event);
}

}  // namespace

TArray<FText> FMjArrayCustomizations::SlotLabelsFor(UMjNodeComponent& Node, const FName& Attribute)
{
	TArray<FText> Labels = LabelsFor(Node, Attribute);
#if URLAB_MJ_GEN
	if (Labels.Num() == 0 && Attribute == FName(TEXT("Size")))
	{
		EMjGeomType Shape = EMjGeomType::sphere;
		if (EffectiveShapeName(Node, Shape))
		{
			Labels = SizeLabelsFor(Shape);
		}
	}
#endif
	return Labels;
}

void FMjArrayCustomizations::CustomizeArrays(IDetailLayoutBuilder& DetailBuilder, UMjNodeComponent& Node)
{
	for (TFieldIterator<FOptionalProperty> It(Node.GetClass()); It; ++It)
	{
		FOptionalProperty* const Optional = *It;
		if (Optional == nullptr)
		{
			continue;
		}

		const TArray<FText> Labels = SlotLabelsFor(Node, Optional->GetFName());
		if (Labels.Num() == 0)
		{
			continue;
		}

		// An unset attribute has no slots to name, and its row already carries
		// the inherited value and the class that supplied it.
		const void* const Container = Optional->ContainerPtrToValuePtr<void>(&Node);
		if (Container == nullptr || !Optional->IsSet(Container))
		{
			continue;
		}

		const TSharedPtr<IPropertyHandle> Handle =
			DetailBuilder.GetProperty(Optional->GetFName(), Optional->GetOwnerClass());
		const TSharedPtr<IPropertyHandle> Array = ArrayWithin(Handle);
		const TSharedPtr<IPropertyHandleArray> Elements = Array.IsValid() ? Array->AsArray() : nullptr;
		if (!Elements.IsValid())
		{
			continue;
		}

		uint32 NumElements = 0;
		if (Elements->GetNumElements(NumElements) != FPropertyAccess::Success || NumElements == 0)
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

		TSharedRef<SHorizontalBox> Boxes = SNew(SHorizontalBox);
		for (uint32 Index = 0; Index < NumElements; ++Index)
		{
			const TSharedPtr<IPropertyHandle> Element = Elements->GetElement(static_cast<int32>(Index));
			if (!Element.IsValid() || !Element->IsValidHandle())
			{
				continue;
			}
			// A shorter array than the schema allows is legal -- MuJoCo fills
			// the rest with its defaults -- so a slot beyond the names is drawn
			// with its index rather than dropped.
			const FText Label = Labels.IsValidIndex(static_cast<int32>(Index))
				? Labels[static_cast<int32>(Index)]
				: FText::AsNumber(static_cast<int32>(Index));

			Boxes->AddSlot()
				.FillWidth(1.0f)
				.Padding(0.0f, 0.0f, 4.0f, 0.0f)
					[SNew(SNumericEntryBox<double>)
							.AllowSpin(false)
							.Label()[SNew(STextBlock).Text(Label)]
							.Value_Lambda([Element]() -> TOptional<double> {
								double Value = 0.0;
								return Element->GetValue(Value) == FPropertyAccess::Success
									? TOptional<double>(Value)
									: TOptional<double>();
							})
							.OnValueCommitted_Lambda([Element](double NewValue, ETextCommit::Type) {
								Element->SetValue(NewValue);
							})];
		}

		Row->CustomWidget(/*bShowChildren=*/false)
			.NameContent()[NameWidget.ToSharedRef()]
			.ValueContent()
			.MinDesiredWidth(250.0f)
			.MaxDesiredWidth(700.0f)[Boxes];
	}
}

void FMjArrayCustomizations::AddEulerRow(IDetailLayoutBuilder& DetailBuilder, UMjNodeComponent& Node)
{
	FOptionalProperty* const Property =
		CastField<FOptionalProperty>(Node.GetClass()->FindPropertyByName(FName(TEXT("Quat"))));
	if (Property == nullptr)
	{
		return;
	}
	const FStructProperty* const Value = CastField<FStructProperty>(Property->GetValueProperty());
	if (Value == nullptr || Value->Struct != FMjQuatRot::StaticStruct())
	{
		return;
	}

	TWeakObjectPtr<UMjNodeComponent> WeakNode = &Node;
	const auto ReadAxis = [WeakNode, Property](int32 Axis) -> TOptional<double> {
		const UMjNodeComponent* const Live = WeakNode.Get();
		if (Live == nullptr)
		{
			return TOptional<double>();
		}
		double Quat[4];
		ReadQuat(*Live, *Property, Quat);
		double Degrees[3];
		FMjArrayCustomizations::QuatToEulerDegrees(Quat, Degrees);
		return TOptional<double>(Degrees[Axis]);
	};

	const auto WriteAxis = [WeakNode, Property](double NewValue, int32 Axis) {
		UMjNodeComponent* const Live = WeakNode.Get();
		if (Live == nullptr)
		{
			return;
		}
		double Quat[4];
		ReadQuat(*Live, *Property, Quat);
		double Degrees[3];
		FMjArrayCustomizations::QuatToEulerDegrees(Quat, Degrees);
		Degrees[Axis] = NewValue;

		double Composed[4];
		FMjArrayCustomizations::EulerDegreesToQuat(Degrees, Composed);
		WriteQuat(*Live, *Property, Composed);
	};

	TSharedRef<SHorizontalBox> Row = SNew(SHorizontalBox);
	const FText Names[3] = {LOCTEXT("EulerX", "X"), LOCTEXT("EulerY", "Y"), LOCTEXT("EulerZ", "Z")};
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		Row->AddSlot()
			.FillWidth(1.0f)
			.Padding(0.0f, 0.0f, 4.0f, 0.0f)
				[SNew(SNumericEntryBox<double>)
						.AllowSpin(false)
						.Label()[SNew(STextBlock).Text(Names[Axis])]
						.Value_Lambda([ReadAxis, Axis]() { return ReadAxis(Axis); })
						.OnValueCommitted_Lambda(
							[WriteAxis, Axis](double NewValue, ETextCommit::Type) { WriteAxis(NewValue, Axis); })];
	}

	Row->AddSlot()
		.AutoWidth()
		.VAlign(VAlign_Center)
			[SNew(STextBlock)
					.Text(LOCTEXT("EulerUnit", "deg, xyz (MuJoCo frame)"))
					.ColorAndOpacity(FSlateColor::UseSubduedForeground())];

	DetailBuilder.EditCategory(CategoryOf(*Property))
		.AddCustomRow(LOCTEXT("EulerSearch", "euler orientation"))
		.NameContent()
			[SNew(STextBlock)
					.Text(LOCTEXT("EulerName", "Euler"))
					.ToolTipText(LOCTEXT("EulerTip",
						"A second way to type the element's quaternion. Nothing authors an euler "
						"attribute: a commit here writes `quat`, which is what the writer emits."))
					.Font(IDetailLayoutBuilder::GetDetailFont())]
		.ValueContent()
		.MinDesiredWidth(250.0f)
		.MaxDesiredWidth(700.0f)[Row];
}

#undef LOCTEXT_NAMESPACE
