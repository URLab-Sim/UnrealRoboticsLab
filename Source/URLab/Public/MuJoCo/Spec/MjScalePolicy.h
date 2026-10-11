// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// What a component's relative scale means to the element under it.
//
// MJCF has no scale. An element that carries a shape carries a `size`, and the
// component's scale is an editor of that size and of nothing else -- so the
// question "what may a scale drag do here" has exactly three answers, and which
// one an element gets comes from the schema rather than from a list kept by
// hand:
//
//   Unsized           the element declares no `size`: a body, a frame, a light.
//                     There is nothing for a scale to be, so the scale snaps
//                     back to one. The refusal is the honest answer -- scaling
//                     a body distorts every descendant's picture while the
//                     simulation goes on using the undistorted numbers.
//
//   GeomShaped        the element declares a `size` whose meaning is decided by
//                     a `type` of MuJoCo's GeomType: `<geom>` and `<site>`. Both
//                     get the same treatment, each under its OWN type's arity
//                     and lock -- a sphere has one radius whether it is a geom
//                     or a site, and a non-uniform drag on either snaps.
//
//   SizeIsNotAScale   the element declares a `size` that is not a shape's:
//                     `<composite>` sizes a lattice, and no scale of the
//                     component expresses it. Same refusal as Unsized, for a
//                     different and stated reason.
//
// The derivation is the point. `MjSizedTransformElements()` reads the generated
// schema reflection, so a MuJoCo release that gives some new element a `pos` and
// a `size` appears in that list the day it is generated, and the classification
// gate in the tests fails by name until someone says which of the three it is.
// A hand-kept list would simply have been silently short.

#include "CoreMinimal.h"

#include "MuJoCo/Gen/MjEnums.gen.h"
#include "MuJoCo/Spec/MjGenHooks.h"
#include "MuJoCo/Spec/MjSpecRef.h"

class UMjNodeComponent;

namespace urlab::spec
{

/** Which scale axes a shape lets the gizmo move independently. */
enum class EMjScaleLock : uint8
{
	/** All three, as a box wants. */
	Free,
	/** X and Y move together, so a round cross-section stays round. */
	RadialXY,
	/** One number for all three, so a sphere stays a sphere. */
	Uniform,
	/** X and Y are free and Z is pinned to 1: a plane has no thickness. */
	FlatXY,
};

/** One component of the relative scale, and the `size` slot that decides it. */
struct FMjSizeAxis
{
	/** 0, 1 or 2: X, Y or Z of the component's relative scale. */
	uint8 ScaleAxis = 0;

	/** Index into the element's MJCF `size` array. */
	uint8 SizeSlot = 0;
};

/**
 * How one GeomType's `size` and a component's scale correspond.
 *
 * `Axes` is the mapping, declared rather than computed. Both directions are
 * derived from it -- `size` to scale for the preview, scale to `size` for the
 * write-back -- so the round trip is consistent by construction and cannot
 * drift the way a forward function and a hand-written inverse would. The schema
 * cannot supply this: it says only `size : double[1..3]`.
 */
struct FMjSizeShape
{
	FMjSizeAxis Axes[3] = {};

	/** How many of `Axes` are live. Zero means the shape has no scale mapping. */
	uint8 AxisNum = 0;

	EMjScaleLock Lock = EMjScaleLock::Free;

	/**
	 * Metres a mapped `size` of zero previews at, or 0 when zero is simply zero.
	 *
	 * Only a plane needs it: MJCF spells "infinite in this direction" as a zero
	 * half-extent, and Unreal has to draw something finite.
	 */
	double InfiniteExtent = 0.0;

	/**
	 * True when the scale gizmo must not author `size` back onto the spec.
	 *
	 * A plane's `size` is (half-x, half-y, grid-spacing): two of the three are
	 * dimensions and the third is not, and either dimension may be the zero that
	 * means infinite. No scale a drag could write preserves all of that.
	 */
	bool bSizeIsReadOnly = false;
};

/** The row for `Type`, or an empty one for a type with no mapping. */
URLAB_API const FMjSizeShape& MjSizeShapeFor(EMjGeomType Type);

/**
 * Force `Scale` onto what the shape can represent.
 *
 * X is the master, which is the rule the user sees: drag a sphere's Y handle
 * and all three snap together on the spot.
 */
URLAB_API void MjApplyScaleLock(EMjScaleLock Lock, FVector& Scale);

/**
 * Force `Scale` onto what the shape can represent, resolving toward the axis the
 * gesture actually moved.
 *
 * `Previous` is the scale before the edit. Without it the lock has to pick a
 * fixed master, and X is the only one it can pick, which silently discards a
 * drag on any other handle: a sphere's Y moved, Y was overwritten with X's
 * untouched value, and the gesture vanished with nothing to show for it.
 *
 * Only a single moved axis is read as a gesture. A change on two or three at
 * once is a whole-vector set with no handle to infer, and X stays the master
 * there, which is what `MjApplyScaleLock` has always done. Use this wherever a
 * previous scale exists; `MjApplyScaleLock` remains correct where the axes are
 * derived from a `size` and are consistent already.
 */
URLAB_API void MjApplyScaleLockFrom(EMjScaleLock Lock, const FVector& Previous, FVector& Scale);

/**
 * The relative scale `Size` implies under `Shape`, or zero when it implies none.
 *
 * A size too short for its type is unresolvable, and so is a non-positive one;
 * both come back as a zero scale, which callers read as "leave it alone" --
 * except where the shape declares that a zero size means infinite.
 */
URLAB_API FVector MjScaleFromSize(const FMjSizeShape& Shape, const TArray<double>& Size);

/** The inverse, off the same rows. Authors exactly the shape's own slots. */
URLAB_API TArray<double> MjSizeFromScale(const FMjSizeShape& Shape, FVector Scale);

/**
 * How many `size` entries MuJoCo reads for `Type`.
 *
 * `mjGEOMINFO` (user_objects.h:74), verbatim, with the one row upstream's table
 * does not reach: an SDF geom takes its shape from its plugin, never from
 * `size`. Sites share the table because they share the enum -- MuJoCo's own
 * writer trims a site's `size` by `mjGEOMINFO[site->type]`
 * (xml_native_writer.cc:486).
 */
URLAB_API int32 MjSizeArityFor(EMjGeomType Type);

#if URLAB_MJ_GEN

/** What a scale drag may do to an element of a given schema type. */
enum class EMjScalePolicy : uint8
{
	/** No `size` at all: the scale snaps back to one. */
	Unsized,
	/** A `size` a GeomType decides: the per-type lock and arity apply. */
	GeomShaped,
	/** A `size` that is not a shape's: the scale snaps back to one. */
	SizeIsNotAScale,
};

/** The policy for `Type`, derived from the schema rather than enumerated. */
URLAB_API EMjScalePolicy MjScalePolicyFor(psm::ElementType Type);

/** The same, for whatever element `Node` is. */
URLAB_API EMjScalePolicy MjScalePolicyOf(const UMjNodeComponent& Node);

/**
 * Every schema element that carries BOTH a transform and a `size`.
 *
 * The set the policy above has to cover, read off the generated reflection in
 * schema order. The classification test asserts that every member has a policy
 * that is not the unsized one, so a new sized element cannot arrive silently.
 */
URLAB_API TArray<psm::ElementType> MjSizedTransformElements();

/**
 * The `type` and `size` a GeomShaped element compiles with.
 *
 * Effective, not authored: an element that says only `class="collision"` is the
 * common case in a menagerie model, and reading its own storage alone would
 * draw a sphere of no size. False when the element is not GeomShaped at all.
 *
 * Resolves the default-class chain, so callers batch it under one
 * `FMjEffectiveScope` exactly as every other effective read is batched.
 */
URLAB_API bool MjEffectiveShapeOf(const UMjNodeComponent& Node, EMjGeomType& OutType, TArray<double>& OutSize);

/** One element whose authored `size` was longer than its type can use. */
struct FMjSizeViolation
{
	/** The element, as the model names it. */
	FString Name;

	/** Where it was authored, when the reader recorded it. */
	FString File;
	int32 Line = 0;

	/** What it authored, and what its type reads. */
	int32 Authored = 0;
	int32 Allowed = 0;

	FString Message;
};

/**
 * Say so where a user will see it: the editor's message log and the run's log.
 *
 * The spec build finds these -- one pass over the whole document, after every
 * element carries the type its class resolved to and before the compile -- and
 * hands them here. Two audiences, and the build's own diagnostic array is only
 * one of them: it is read when a build FAILS, and this one does not fail, so a
 * silently truncated size would otherwise be recorded nowhere a user looks.
 */
URLAB_API void MjReportSizeArity(const TArray<FMjSizeViolation>& Violations);

#endif // URLAB_MJ_GEN

} // namespace urlab::spec
