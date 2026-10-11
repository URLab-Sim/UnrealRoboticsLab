// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Two rows the schema cannot describe on its own.
//
// MJCF types `solimp` as five doubles and stops there, so the panel offers five
// boxes numbered 0 to 4 and the user has to know that the third one is the
// constraint width. The schema is not wrong -- the meaning of a slot is prose
// in MuJoCo's documentation, not a type -- so the names live here, one row per
// attribute, and the row says `dmin` where it used to say `2`.
//
// `size` is the one that varies: a sphere's single number is a radius and a
// box's three are half-extents, so its labels come from the element's effective
// shape rather than from the attribute.
//
// The euler row is here for the same reason and not because it is an array. An
// MJCF `quat` is four numbers nobody thinks in, and while the spec must keep
// storing exactly what round-trips, nothing stops the panel offering a second
// way to type it. The row is a view: it reads the authored quaternion, and a
// commit writes the quaternion back. No euler value is ever authored, so the
// writer emits what it always did.

#include "CoreMinimal.h"

class IDetailLayoutBuilder;
class UMjNodeComponent;

/** The labelled-array rows and the euler editing row. */
class FMjArrayCustomizations
{
public:
	/** Name the slots of the numeric arrays whose slots have names. */
	static void CustomizeArrays(IDetailLayoutBuilder& DetailBuilder, UMjNodeComponent& Node);

	/**
	 * What `Attribute`'s slots are called on THIS element, in MuJoCo's order.
	 *
	 * Empty when the attribute has no named slots, which is most of them.
	 *
	 * The element matters, and that is the whole reason this is not a table keyed
	 * by attribute name: `friction` is three coefficients on a `<geom>` and five
	 * on a `<pair>`, and `size` means a radius or three half-extents depending on
	 * the shape the element resolves to. Exposed for the same reason the euler
	 * conversion below is: what a row says is checkable as text where the widget
	 * it ends up in is not, and a mislabelled friction slot is a user authoring
	 * the wrong number.
	 */
	static TArray<FText> SlotLabelsFor(UMjNodeComponent& Node, const FName& Attribute);

	/** Offer euler degrees as a second way to type the element's `quat`. */
	static void AddEulerRow(IDetailLayoutBuilder& DetailBuilder, UMjNodeComponent& Node);

	/**
	 * The conversion the euler row applies, in MJCF's own conventions.
	 *
	 * Degrees, sequence "xyz" (MuJoCo's default `eulerseq`: about x, then y,
	 * then z, each about the rotating axes), quaternion as [w, x, y, z]
	 * right-handed. Exposed because the row is only as trustworthy as this
	 * pair, and a sign error in it is invisible in a Slate widget.
	 */
	static void EulerDegreesToQuat(const double Degrees[3], double OutQuat[4]);
	static void QuatToEulerDegrees(const double Quat[4], double OutDegrees[3]);
};
