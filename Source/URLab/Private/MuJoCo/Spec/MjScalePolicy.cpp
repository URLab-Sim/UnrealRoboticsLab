// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjScalePolicy.h"

#include "Utils/URLabLogging.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Spec/MjEffective.h"
#include "MuJoCo/Spec/MjElementIdentity.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSpecProfile.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"

#if WITH_EDITOR
#include "Logging/MessageLog.h"
#endif

THIRD_PARTY_INCLUDES_START
#include "reflect.h"
THIRD_PARTY_INCLUDES_END

#endif // URLAB_MJ_GEN

namespace urlab::spec
{
namespace
{

// The engine's basic shapes are 100 cm across and 100 cm tall, so a component
// scale of 1 is a 50 cm half-extent in every direction. MJCF sizes are metres.
constexpr double kCmPerM = 100.0;
constexpr double kBaseHalf = 50.0;
constexpr double kSizeToScale = kCmPerM / kBaseHalf;

/** Half-extent, in metres, an infinite plane previews at. */
constexpr double kInfinitePlaneHalfExtent = 10.0;

constexpr FMjSizeShape GShapes[] = {
	/* plane     */ {{{0, 0}, {1, 1}}, 2, EMjScaleLock::FlatXY, kInfinitePlaneHalfExtent, true},
	/* hfield    */
	{},
	/* sphere    */
	{{{0, 0}}, 1, EMjScaleLock::Uniform},
	/* capsule   */
	{{{0, 0}, {2, 1}}, 2, EMjScaleLock::RadialXY},
	/* ellipsoid */
	{{{0, 0}, {1, 1}, {2, 2}}, 3, EMjScaleLock::Free},
	/* cylinder  */
	{{{0, 0}, {2, 1}}, 2, EMjScaleLock::RadialXY},
	/* box       */
	{{{0, 0}, {1, 1}, {2, 2}}, 3, EMjScaleLock::Free},
	/* mesh      */
	{},
	/* sdf       */
	{},
};

static_assert(static_cast<int32>(EMjGeomType::sdf) + 1 == static_cast<int32>(UE_ARRAY_COUNT(GShapes)),
	"GShapes is indexed by EMjGeomType and must have a row for every value, in declaration order");

/**
 * The size slots a shape maps must be 0..AxisNum-1 with no gaps.
 *
 * The write-back authors the whole array for the type, so a gap would leave a
 * slot MuJoCo reads holding a zero nobody wrote.
 */
constexpr bool SizeSlotsAreContiguous()
{
	for (const FMjSizeShape& Shape : GShapes)
	{
		uint32 Seen = 0;
		for (uint8 Index = 0; Index < Shape.AxisNum; ++Index)
		{
			Seen |= 1u << Shape.Axes[Index].SizeSlot;
		}
		if (Seen != (1u << Shape.AxisNum) - 1u)
		{
			return false;
		}
	}
	return true;
}

static_assert(SizeSlotsAreContiguous(), "a shape's size slots must be 0..AxisNum-1");

/** `mjGEOMINFO` (user_objects.h:74), plus the SDF row upstream's table omits. */
constexpr int32 GSizeArity[] = {3, 0, 1, 2, 3, 2, 3, 0, 0};

static_assert(UE_ARRAY_COUNT(GSizeArity) == UE_ARRAY_COUNT(GShapes),
	"the arity table is indexed by EMjGeomType alongside the shape table");

#if URLAB_MJ_GEN

/** The field ids the policy reads, or -1 where the element has none. */
struct FShapeFieldIds
{
	int Size = -1;
	int Type = -1;
};

FShapeFieldIds ShapeFieldIdsOf(psm::ElementType Type)
{
	FShapeFieldIds Out;
	Out.Size = pssdk::internal::FieldIdByName(Type, "size");
	Out.Type = pssdk::internal::FieldIdByName(Type, "type");
	return Out;
}

#endif // URLAB_MJ_GEN

} // namespace

const FMjSizeShape& MjSizeShapeFor(EMjGeomType Type)
{
	static const FMjSizeShape NoMapping;
	const int32 Index = static_cast<int32>(Type);
	if (Index < 0 || Index >= static_cast<int32>(UE_ARRAY_COUNT(GShapes)))
	{
		return NoMapping;
	}
	return GShapes[Index];
}

int32 MjSizeArityFor(EMjGeomType Type)
{
	const int32 Index = static_cast<int32>(Type);
	if (Index < 0 || Index >= static_cast<int32>(UE_ARRAY_COUNT(GSizeArity)))
	{
		return 0;
	}
	return GSizeArity[Index];
}

void MjApplyScaleLock(EMjScaleLock Lock, FVector& Scale)
{
	switch (Lock)
	{
		case EMjScaleLock::Uniform:
			Scale.Y = Scale.Z = Scale.X;
			break;
		case EMjScaleLock::RadialXY:
			Scale.Y = Scale.X;
			break;
		case EMjScaleLock::FlatXY:
			Scale.Z = 1.0;
			break;
		case EMjScaleLock::Free:
			break;
	}
}

void MjApplyScaleLockFrom(EMjScaleLock Lock, const FVector& Previous, FVector& Scale)
{
	if (Lock == EMjScaleLock::Free)
	{
		return;
	}

	// Exactly one axis moved is a handle being dragged, and that axis is what the
	// gesture meant. More than one is a whole-vector set with no gesture to read,
	// where X stays the master: that is the documented rule and what every path
	// that sets three components at once already relies on.
	int32 Moved = 0;
	int32 Master = 0;
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		if (!FMath::IsNearlyEqual(Scale[Axis], Previous[Axis]))
		{
			++Moved;
			Master = Axis;
		}
	}
	if (Moved != 1)
	{
		Master = 0;
	}

	switch (Lock)
	{
		case EMjScaleLock::Uniform:
			Scale.X = Scale.Y = Scale.Z = Scale[Master];
			break;
		case EMjScaleLock::RadialXY:
			// Z is its own axis, so a Z gesture leaves the radius alone and a
			// radial gesture leaves the length alone.
			if (Master == 2)
			{
				Scale.Y = Scale.X;
			}
			else
			{
				Scale.X = Scale.Y = Scale[Master];
			}
			break;
		case EMjScaleLock::FlatXY:
			Scale.Z = 1.0;
			break;
		case EMjScaleLock::Free:
			break;
	}
}

FVector MjScaleFromSize(const FMjSizeShape& Shape, const TArray<double>& Size)
{
	if (Shape.AxisNum == 0)
	{
		return FVector::ZeroVector;
	}
	FVector Scale = FVector::ZeroVector;
	for (uint8 Index = 0; Index < Shape.AxisNum; ++Index)
	{
		const FMjSizeAxis& Axis = Shape.Axes[Index];
		if (Size.Num() <= static_cast<int32>(Axis.SizeSlot))
		{
			return FVector::ZeroVector;
		}
		const double Extent = Size[Axis.SizeSlot];
		Scale[Axis.ScaleAxis] = (Extent == 0.0 ? Shape.InfiniteExtent : Extent) * kSizeToScale;
	}
	MjApplyScaleLock(Shape.Lock, Scale);
	return Scale;
}

TArray<double> MjSizeFromScale(const FMjSizeShape& Shape, FVector Scale)
{
	MjApplyScaleLock(Shape.Lock, Scale);
	TArray<double> Size;
	Size.SetNumZeroed(Shape.AxisNum);
	for (uint8 Index = 0; Index < Shape.AxisNum; ++Index)
	{
		const FMjSizeAxis& Axis = Shape.Axes[Index];
		Size[Axis.SizeSlot] = Scale[Axis.ScaleAxis] / kSizeToScale;
	}
	return Size;
}

#if URLAB_MJ_GEN

EMjScalePolicy MjScalePolicyFor(psm::ElementType Type)
{
	if (MjSchemaFieldOf(Type, "size") == nullptr)
	{
		return EMjScalePolicy::Unsized;
	}
	// A `size` means something only through the `type` that says which shape it
	// describes, and only MuJoCo's GeomType enum is a shape. Asking the schema
	// which enum it is rather than which element this is means `<site>` and
	// `<geom>` are covered by the same sentence, and so is whatever the next
	// release spells the same way.
	const psm::reflect::FieldDescriptor* const TypeField = MjSchemaFieldOf(Type, "type");
	if (TypeField != nullptr && TypeField->kind == psm::reflect::FieldKind::Enum
		&& TypeField->type_name == "GeomType")
	{
		return EMjScalePolicy::GeomShaped;
	}
	return EMjScalePolicy::SizeIsNotAScale;
}

EMjScalePolicy MjScalePolicyOf(const UMjNodeComponent& Node)
{
	psm::ElementType Type{};
	if (!MjElementTypeOfNode(Node, Type))
	{
		return EMjScalePolicy::Unsized;
	}
	return MjScalePolicyFor(Type);
}

TArray<psm::ElementType> MjSizedTransformElements()
{
	TArray<psm::ElementType> Out;
	for (std::size_t Index = 0; Index < psm::reflect::ElementCount(); ++Index)
	{
		const psm::reflect::ElementDescriptor& Descriptor = psm::reflect::ElementAt(Index);
		const bool bTransform =
			MjSchemaFieldOf(Descriptor.type, "pos") != nullptr || MjSchemaFieldOf(Descriptor.type, "quat") != nullptr;
		if (bTransform && MjSchemaFieldOf(Descriptor.type, "size") != nullptr)
		{
			Out.Add(Descriptor.type);
		}
	}
	return Out;
}

bool MjEffectiveShapeOf(const UMjNodeComponent& Node, EMjGeomType& OutType, TArray<double>& OutSize)
{
	if (MjScalePolicyOf(Node) != EMjScalePolicy::GeomShaped)
	{
		return false;
	}

	OutType = EMjGeomType::sphere;
	OutSize.Reset();

	// What the element itself says, first and without touching the spec. An
	// element that authored both is answered here, and a detached one -- a
	// duplicate in the transient package, a template mid-reinstance -- is
	// answered here or not at all, because there is no document to resolve
	// through and its own values are still the truth about it.
	int32 TypeIndex = -1;
	bool bSizeSet = false;
	bool bHasSize = false;
	{
		using P = FMjInstanceProfile;
		gen::DispatchByType(const_cast<UMjNodeComponent&>(Node), [&](auto& Element) {
			using E = std::decay_t<decltype(Element)>;
			const FShapeFieldIds Ids = ShapeFieldIdsOf(gen::TMjElementType<E>::Value);
			if (Ids.Size < 0)
			{
				return;
			}
			bHasSize = true;
			if (Ids.Type >= 0)
			{
				ReadEnum<P>(Element, Ids.Type, TypeIndex);
			}
			bSizeSet = ReadSeq<P>(Element, Ids.Size, OutSize);
		});
	}
	if (!bHasSize)
	{
		return false;
	}

	// Then the default-class chain, for whichever half the element left unset.
	// A geom that says only `class="collision"` is the ordinary case in a
	// menagerie model, and reading its own storage alone would draw a sphere of
	// no size.
	if (TypeIndex < 0 || !bSizeSet)
	{
		WithEffectiveDoc(Node, [&](auto& Effective) {
			using P = typename std::decay_t<decltype(Effective)>::ProfileType;
			gen::DispatchByType(const_cast<UMjNodeComponent&>(Node), [&](auto& Element) {
				using E = std::decay_t<decltype(Element)>;
				const FShapeFieldIds Ids = ShapeFieldIdsOf(gen::TMjElementType<E>::Value);
				if (TypeIndex < 0 && Ids.Type >= 0)
				{
					Effective.ForEachLayer(Element, [&](const auto& Layer) {
						return ReadEnum<P>(Layer, Ids.Type, TypeIndex);
					});
				}
				if (!bSizeSet)
				{
					Effective.ForEachLayer(Element, [&](const auto& Layer) {
						return (bSizeSet = ReadSeq<P>(Layer, Ids.Size, OutSize));
					});
				}
			});
		});
	}

	// And the schema's own default last, the way the compiler's merge ends: a
	// site with no size anywhere is 0.005 cubed, and a geom with none has
	// nothing, which is exactly what MuJoCo makes of each.
	if (TypeIndex < 0 || !bSizeSet)
	{
		using P = FMjInstanceProfile;
		gen::DispatchByType(const_cast<UMjNodeComponent&>(Node), [&](auto& Element) {
			using E = std::decay_t<decltype(Element)>;
			const FShapeFieldIds Ids = ShapeFieldIdsOf(gen::TMjElementType<E>::Value);
			if (TypeIndex < 0 && Ids.Type >= 0)
			{
				ReadEnum<P>(P::template Defaults<E>(), Ids.Type, TypeIndex);
			}
			if (!bSizeSet)
			{
				ReadSeq<P>(P::template Defaults<E>(), Ids.Size, OutSize);
			}
		});
	}

	if (TypeIndex >= 0)
	{
		OutType = static_cast<EMjGeomType>(TypeIndex);
	}
	return true;
}

void MjReportSizeArity(const TArray<FMjSizeViolation>& Violations)
{
#if WITH_EDITOR
	FMessageLog MessageLog(TEXT("URLab"));
#endif
	for (const FMjSizeViolation& Violation : Violations)
	{
		const FString Where = Violation.File.IsEmpty()
								? FString()
								: FString::Printf(TEXT(" (%s:%d)"), *Violation.File, Violation.Line);
		UE_LOG(LogURLab, Warning, TEXT("%s: %s%s"), *Violation.Name, *Violation.Message, *Where);
#if WITH_EDITOR
		MessageLog.Warning(FText::FromString(Violation.Name + TEXT(": ") + Violation.Message + Where));
#endif
	}
}

#endif // URLAB_MJ_GEN

} // namespace urlab::spec
