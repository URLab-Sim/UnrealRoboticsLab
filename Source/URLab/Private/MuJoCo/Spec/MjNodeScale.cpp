// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjNodeScale.h"

#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjScalePolicy.h"
#include "MuJoCo/Spec/MjSpecProfile.h"

#if URLAB_MJ_GEN
#include <vector>
THIRD_PARTY_INCLUDES_START
#include "protospec/core.h"
#include "protospec/detail.h"
#include "protospec/model_core.h"
#include "protospec/profile.h"
THIRD_PARTY_INCLUDES_END
#include "MuJoCo/Gen/MjDispatch.gen.h"
#endif

#if URLAB_MJ_GEN

namespace
{
namespace pssdk = ps::sdk;

/**
 * Why a drag on this shape is not an edit of its `size`, in the terms the user
 * can act on: which element DOES own the size they were trying to change.
 *
 * Every shape here is one MuJoCo itself sizes from somewhere other than the
 * element's own `size`, so there is always somewhere to send them.
 */
FString ScaleRefusalMessage(EMjGeomType Type)
{
	switch (Type)
	{
		case EMjGeomType::mesh:
			return TEXT("the scale handle does not author a mesh shape's size: it is the <mesh> asset's own, "
						"so scale the <mesh> element rather than this one");
		case EMjGeomType::hfield:
			return TEXT("the scale handle does not author a height field's size: it is the <hfield> asset's own, "
						"so scale the <hfield> element rather than this one");
		case EMjGeomType::sdf:
			return TEXT("the scale handle does not author an sdf shape's size: it comes from the geom's plugin");
		case EMjGeomType::plane:
			return TEXT("the scale handle does not author a plane's size: two of its three values are half-extents "
						"and the third is a grid spacing, and a zero half-extent means infinite");
		default:
			return TEXT("the scale handle does not author this shape's size");
	}
}
} // namespace

namespace urlab::spec
{
bool MjNodeTryPreviewScaleFromSpec(const UMjNodeComponent& Node, FVector& OutScale)
{
	EMjGeomType Type = EMjGeomType::sphere;
	TArray<double> Size;
	if (!urlab::spec::MjEffectiveShapeOf(Node, Type, Size))
	{
		return false;
	}
	const urlab::spec::FMjSizeShape& Shape = urlab::spec::MjSizeShapeFor(Type);
	if (Shape.AxisNum == 0)
	{
		return false;
	}

	// An unresolvable size -- too short for the type, or non-positive -- leaves
	// the scale alone rather than collapsing the element to nothing.
	const FVector Scale = urlab::spec::MjScaleFromSize(Shape, Size);
	if (Scale.GetMin() <= 0.0)
	{
		return false;
	}
	OutScale = Scale;
	return true;
}

bool MjNodeHasScaleMapping(const UMjNodeComponent& Node)
{
	EMjGeomType Type = EMjGeomType::sphere;
	TArray<double> Size;
	if (!urlab::spec::MjEffectiveShapeOf(Node, Type, Size))
	{
		return false;
	}
	const urlab::spec::FMjSizeShape& Shape = urlab::spec::MjSizeShapeFor(Type);
	return Shape.AxisNum > 0 && !Shape.bSizeIsReadOnly;
}

void MjNodeConstrainPreviewScale(UMjNodeComponent& Node)
{
	const FVector Scale = Node.GetRelativeScale3D();
	FVector Locked = Scale;

	EMjGeomType Type = EMjGeomType::sphere;
	TArray<double> Size;
	const bool bShaped = urlab::spec::MjEffectiveShapeOf(Node, Type, Size);
	const urlab::spec::FMjSizeShape& Shape = urlab::spec::MjSizeShapeFor(Type);
	const bool bScaleIsTheSize = bShaped && Shape.AxisNum > 0 && !Shape.bSizeIsReadOnly;

	if (bScaleIsTheSize)
	{
		// From the baseline where there is one, so the lock resolves toward the
		// handle that moved rather than toward a fixed axis. Without a baseline
		// there is no gesture to read and the fixed rule is all there is.
		if (Node.LastPreviewTransform.IsSet())
		{
			urlab::spec::MjApplyScaleLockFrom(Shape.Lock, Node.LastPreviewTransform->GetScale3D(), Locked);
		}
		else
		{
			urlab::spec::MjApplyScaleLock(Shape.Lock, Locked);
		}
	}
	else
	{
		// The refusal. Nothing about this element's `size` -- it has none, or has
		// one no scale expresses -- so a scaled component is a picture that
		// disagrees with what will be simulated, and the honest answer is to put
		// it back rather than to keep a distortion nobody can act on.
		//
		// A shape whose `size` is not a scale is the same refusal, and it is not
		// silent about it: a mesh geom's handle can move even though the
		// write-back has nothing to author, so leaving the component stretched
		// would show a picture that would never be simulated. It snaps back to
		// what the spec implies -- which for a mesh is one, because a mesh geom's
		// own scale says nothing about its picture -- exactly as a body's does.
		if (!Node.TryPreviewScaleFromSpec(Locked))
		{
			Locked = FVector::OneVector;
		}
		if (bShaped)
		{
			// Only for a shape, because an element with no `size` at all has no
			// alternative to point the user at: the message below names the element
			// that DOES own the size, and there is one.
			Node.NotePreviewProblem(EMjPreviewProblem::ScaleNotEditable, ScaleRefusalMessage(Type));
		}
	}

	if (!Locked.Equals(Scale))
	{
		Node.SetRelativeScale3D(Locked);
	}
}

bool MjNodeWriteBackScale(UMjNodeComponent& Node, const FVector& Scale)
{
	EMjGeomType Type = EMjGeomType::sphere;
	TArray<double> Size;
	if (!urlab::spec::MjEffectiveShapeOf(Node, Type, Size))
	{
		return false;
	}
	const urlab::spec::FMjSizeShape& Shape = urlab::spec::MjSizeShapeFor(Type);
	if (Shape.AxisNum == 0 || Shape.bSizeIsReadOnly)
	{
		return false;
	}

	// Exactly the slots this type reads, which is the arity MuJoCo's own writer
	// trims a `size` to. A drag can therefore never author an over-long one.
	const TArray<double> Authored = urlab::spec::MjSizeFromScale(Shape, Scale);

	// And never a size of zero or less. MuJoCo's own `checksize` refuses one --
	// "size 0 must be positive" -- so authoring it turns a working model into one
	// that will not compile, from a gesture that looks like an ordinary drag.
	//
	// It is not a rare gesture either: the level viewport's scale grid steps in
	// 0.25 and most of a robot is smaller than that, so the first drag on a
	// centimetre-scale geom lands exactly on zero. The grid is the editor's and
	// stays as the user configured it; the check above is what keeps the spec
	// from taking the collapse.
	for (const double Value : Authored)
	{
		if (Value <= 0.0)
		{
			Node.NotePreviewProblem(EMjPreviewProblem::NonPositiveSize,
				FString::Printf(
					TEXT("a scale of %s would author a size of zero or less, so the size is unchanged: the "
						 "viewport's scale grid steps in 0.25 by default, which is larger than most of a robot"),
					*Scale.ToString()));
			return false;
		}
	}
	Node.ClearPreviewProblem(EMjPreviewProblem::NonPositiveSize);

	using P = urlab::spec::FMjInstanceProfile;
	urlab::spec::gen::DispatchByType(Node, [&Authored](auto& Element) {
		using E = std::decay_t<decltype(Element)>;
		const int FieldId =
			pssdk::internal::FieldIdByName(urlab::spec::gen::TMjElementType<E>::Value, "size");
		if (FieldId < 0)
		{
			return;
		}
		double Slots[3] = {0.0, 0.0, 0.0};
		const int32 Num = FMath::Min(Authored.Num(), 3);
		for (int32 Index = 0; Index < Num; ++Index)
		{
			Slots[Index] = Authored[Index];
		}
		pssdk::internal::SetSeqField<P>(Element, FieldId, Slots, static_cast<std::size_t>(Num));
	});
	return true;
}
} // namespace urlab::spec

#endif // URLAB_MJ_GEN
