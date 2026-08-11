// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// One of every top-level section, whatever the document spelled.
//
// The schema admits all seventeen children of `<mujoco>` any number of times
// (`element mujoco { compiler * / option * / ... / keyframe * }`, mjcf.schema:505),
// because a model assembles from parts: the robot in one `<worldbody>`, the floor
// in another; the meshes in one `<asset>`, the materials in another. MuJoCo reads
// each occurrence into the same place -- one world body, one asset pool, one
// option struct -- so a document with two sections describes a model with one.
//
// The reader is generic over the schema and produces a component per occurrence,
// which puts `worldbody` beside `worldbody_1` in a panel whose components ARE the
// model. The mirror of that misreading is a section the document never spelled:
// no `<option>` in the text meant no UMjOption to edit, and MuJoCo has an option
// struct either way.
//
// So the model root is normalised to exactly one of each. Duplicates fold into
// the first in document order; absences appear with every attribute unset, which
// is the storage's own word for "the document did not author this" and therefore
// writes nothing and compiles to nothing.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN

class UMjNodeComponent;
struct FSpecRef;

namespace urlab::spec
{

/**
 * True when `Node` states something: an authored attribute, or any child.
 *
 * Presence stopped answering that question the moment every top-level section
 * began to exist. A consumer asking whether a participant said anything about
 * `<option>` used to find out by whether the component was there, and now it
 * always is -- so the question has to be put to the content instead. This is
 * the writer's own condition for emitting an element, which is what keeps the
 * two agreeing about what an empty section means.
 */
bool MjAuthorsAnything(const FSpecRef& Spec, UMjNodeComponent& Node);

/**
 * Give the model at `Root` exactly one of every top-level section.
 *
 * A repeated section merges into the first in document order: its authored
 * attributes are copied over (later wins, which is the order MuJoCo's reader
 * writes them into its one struct in), then its children move across at the end
 * of their storage slot, then the emptied section is dropped. A section the
 * document never spelled is created with nothing authored.
 *
 * `Adapter` is the tree adapter for the graph the nodes live in and `Factory`
 * its construction seam. Runs before the naming pass, so a surviving or created
 * section takes its tag unsuffixed. Idempotent.
 */
template <class Adapter, class Factory>
void NormalizeModelSections(UMjNodeComponent& Root);

} // namespace urlab::spec

#endif // URLAB_MJ_GEN
