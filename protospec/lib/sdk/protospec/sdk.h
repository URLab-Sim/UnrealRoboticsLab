// ProtoSpec SDK: the ergonomic authoring layer over the plain-data object model.
//
// The generated types are plain owned values (DR-2) with presence-tracked
// fields (DR-1) and typed references stored by name (DR-8). This SDK adds the
// convenience that makes them pleasant to build and edit, entirely on top of the
// generated Visit/reflect hooks -- it never needs regenerating when the schema
// grows. Nothing here touches MuJoCo; it is a pure tree library.
//
//   profile.h       the emission-profile seam: five policies on one parameter
//   plain_profile.h the reference profile, over the generated value types
//   builders.h   typed Add* helpers into the right child/union list
//   traversal.h  Find, ForEach*, ParentMap, path-to-element
//   refs.h       Resolve, FindReferrers, Rename, DeleteRecursive
//   classes.h    Effective / EffectiveField + FlattenDefaults / ExtractClass
//   attach.h     namespaced deep-clone splice
//
// Every verb is a template over a profile `P` defaulting to the plain profile,
// so a call site that never mentions a profile reads exactly as it always did.
#ifndef PROTOSPEC_SDK_H
#define PROTOSPEC_SDK_H

#include "protospec/attach.h"
#include "protospec/builders.h"
#include "protospec/classes.h"
#include "protospec/plain_profile.h"
#include "protospec/profile.h"
#include "protospec/refs.h"
#include "protospec/traversal.h"

#endif  // PROTOSPEC_SDK_H
