// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Where the hand seams meet the generated profile.
//
// Everything under MuJoCo/Doc is written against ProtoSpec's six-policy profile
// seam and never names a concrete element type. What it cannot avoid needing is
// the handful of entry points that DO scale with the schema, and which the
// generator therefore emits into MuJoCo/Gen (ps::ue):
//
//   Visit(E&, V&)                     the per-element field hook
//   TMjElementType<E>::Value          class -> schema ElementType
//   TMjElementOf<Type>::Type          the inverse
//   ElementTypeOfNode(node, out)      the same for a node held as its base
//   DispatchByType(node, fn)          recover a node's concrete type
//   ChildSlots(const E*, fn)          the storage slots a parent admits
//   SlotFor(parent, child)            the slot a child type occupies, or -1
//   FMjStrPolicy / FMjShape           the storage policies the emitter decides
//   FMjGeneratedProfile               the generated half of the profile tag
//   ApplyDefault(E&)                  the schema `=` defaults
//
// Ten entry points against 145 elements and 1,533 attributes. Aliasing them
// through one namespace here rather than spelling ps::ue throughout keeps the
// seam one file wide, so a rename on the generated side is one edit.
//
// URLAB_MJ_GEN is 1 when both ProtoSpec and the generated tree are present.
// Everything under MuJoCo/Doc that depends on generated types is gated on it, so
// a checkout without them builds the plugin exactly as it did before Phase 3.

#if defined(URLAB_PROTOSPEC) && URLAB_PROTOSPEC && __has_include("MuJoCo/Gen/MjProfile.gen.h")
#define URLAB_MJ_GEN 1
#else
#define URLAB_MJ_GEN 0
#endif

#if URLAB_MJ_GEN

THIRD_PARTY_INCLUDES_START
#include "MuJoCo/Gen/MjProfile.gen.h"
THIRD_PARTY_INCLUDES_END

namespace urlab::spec
{
namespace gen = ::ps::ue;
namespace psm = ::ps::mjcf;
namespace pssdk = ::ps::sdk;
} // namespace urlab::spec

#endif // URLAB_MJ_GEN
