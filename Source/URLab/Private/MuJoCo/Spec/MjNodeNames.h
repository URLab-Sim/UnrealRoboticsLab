// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Naming the tree the way the MJCF reads.
//
// A node is created before its attributes are, so the only name available at
// construction is the element's XML tag, and `MakeUniqueObjectName` appends an
// ordinal to that unconditionally. The result is a components panel of
// `body_23`, `geom_24`, `actuator_71` -- a tree whose structure is right and
// which nobody can read. A twenty-nine joint robot is unnavigable that way, and
// the sections a user goes looking for are the worst hit, because `<actuator>`
// and `<sensor>` occur once each and still come out suffixed.
//
// This runs once the spec exists and gives every node the name its MJCF
// line already carries:
//
//   * the element's `name`, when it has one -- `torso`, `shin_left`,
//     `hip_x_right`. This is the whole of the navigability question.
//   * otherwise the CONTEXTUAL child tag, which is the tag the writer would
//     emit for that node in that parent. It matters for exactly one element:
//     `<worldbody>` is not a type, it is a `body` in the model's world slot,
//     so a node named after its element type reads `body` and the user
//     reasonably concludes the world body is missing.
//   * an ordinal only on a genuine collision. MJCF names are unique per element
//     type and not across types -- MuJoCo's humanoid has a body `torso` and a
//     geom `torso` -- and Unreal needs one namespace, so the second one asks
//     for a suffix. Only the second one.
//
// Names are sanitised to what Unreal accepts as a variable, because MJCF
// permits characters it does not: a menagerie model namespaces with `/`.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN

class UMjNodeComponent;

namespace urlab::spec
{

/**
 * Rename every node in the tree rooted at `Root` after the element it holds.
 *
 * `Adapter` is the tree adapter for the graph the nodes live in. Runs after the
 * spec is read and before anything can reference a node by name, which is
 * what makes renaming safe: on a freshly imported asset there is nothing bound
 * to the old ordinals.
 */
template <class Adapter>
void NameTreeFromSpec(UMjNodeComponent& Root);

} // namespace urlab::spec

#endif // URLAB_MJ_GEN
