// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// `fromto` is a compile directive, not a stored pose, and this is where it stops
// being one.
//
// A geom or site may spell its pose as `fromto="x1 y1 z1 x2 y2 z2"`: the two
// ends of a capsule, cylinder, box or ellipsoid. MuJoCo does not keep it. Its
// compiler turns it into `pos` + `quat` + `size` (user_objects.cc, mjCGeom and
// mjCSite Compile), and its writer therefore emits pos/quat/size and never a
// `fromto` -- which the MJCF schema states outright, in the attribute's own
// facet: `fromto : double[6] (writing=custom)  # compile directive: saved as
// pos/quat/size`.
//
// So a spec that stores `fromto` verbatim has an authoring form the engine
// does not, and every consumer downstream -- the editor preview, the scale
// handle, the details panel, a Blueprint reading GetPos() -- sees a geom with no
// position and no length. Folding it at import removes the divergence at its
// source rather than teaching each consumer about it.
//
// Why here and not in the reader, which is where MJCF's other alternative
// spellings (`euler`, `axisangle`, `fullinertia`, `diameter`) fold:
//
//   Every one of those folds is a function of the element's OWN authored
//   attributes. `fromto` is not. MuJoCo's fold reads `type` to choose which
//   size slots the half-length lands in -- slot 1 for a capsule or cylinder,
//   slot 2 for a box or ellipsoid, with slot 1 taking slot 0's value -- and it
//   reads `size[0]` as an input. Both are defaultable, so either may come from
//   the element's default-class chain, and ProtoSpec's reader resolves no class
//   chain by design (lib/io/default_classes.h: classes are read verbatim and
//   NOTHING is applied). A reader-level resolver looking at one XML node cannot
//   tell a capsule from a box, and guessing writes the wrong size slots, which
//   is a wrong compiled model rather than a loud failure.
//
//   MuJoCo makes the same call: `fromto` is read straight into `mjsGeom::fromto`
//   and folded in Compile, after class merging. This pass is the equivalent
//   moment on our side -- the whole spec exists, so `MjEffective` can
//   resolve `type` and `size` through the class chain exactly as the merge does.
//
// Anything the fold cannot do exactly, it does not do: a type that does not
// admit `fromto`, a non-zero effective `pos`, or two ends closer than mjEPS all
// leave the attribute authored, so MuJoCo raises its own diagnostic at compile
// instead of this pass inventing a different one.
//
// `fromto` is defaultable, so the value being folded is the EFFECTIVE one and
// the fold has to account for the whole chain, not just the element:
//
//   A geom that says only `class="shin"` inherits its `fromto` and is as much a
//   fold site as one that spells it out -- reading only its own storage leaves
//   it previewing as a capsule of no length.
//
//   And clearing `fromto` on the element is not enough to make the fold safe,
//   because a class behind it may author one too. The element now has a `pos`,
//   the class still has a `fromto`, and MuJoCo rejects the pair -- which is
//   exactly the invalid spec this pass exists to prevent. So a class's
//   `fromto` is retired as well, once every element reading it has folded. That
//   is what MuJoCo's own writer emits: mjXWriter::OneGeom writes `fromto`
//   nowhere, in a `<default>` or on a geom, because by then every geom carries
//   the pos/quat/size the compiler derived from it.
//
//   A class read by even one element that could not fold keeps its `fromto`, so
//   the engine still reports that element rather than this pass silently
//   dropping the geometry.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN

class UMjNodeComponent;

namespace urlab::spec
{

/**
 * Replace every authored `fromto` in the tree rooted at `Root` with the
 * `pos`/`quat`/`size` MuJoCo's compiler would derive from it.
 *
 * `Adapter` is the tree adapter for the graph the nodes live in. Idempotent:
 * a second pass finds no authored `fromto` and does nothing, which is what the
 * round-trip fixpoint rests on.
 */
template <class Adapter>
void FoldFromtoTree(UMjNodeComponent& Root);

} // namespace urlab::spec

#endif // URLAB_MJ_GEN
