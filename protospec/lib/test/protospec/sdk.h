// ProtoSpec SDK: the ergonomic authoring layer over the plain-data object model.
//
// The generated types are plain owned values with presence-tracked
// fields and typed references stored by name. This SDK adds the
// convenience that makes them pleasant to build and edit, entirely on top of the
// generated Visit/reflect hooks -- it never needs regenerating when the schema
// grows. Nothing here touches MuJoCo; it is a pure tree library.
//
//   profile.h        the emission-profile seam: five policies on one parameter
//   plain_profile.h  the reference profile, over the generated value types
//   parents.h        ParentMap, the upward index
//   classes.h        Effective / EffectiveField
//   builders.h       typed Add* helpers into the right child/union list
//   traversal.h      Find, ForEach*, path-to-element
//   refs.h           Resolve, FindReferrers, Rename, DeleteRecursive
//   edits.h          Duplicate, Reparent
//   class_edits.h    FlattenDefaults / ExtractClass
//
// Every verb is a template over a profile `P` defaulting to the plain profile,
// so a call site that never mentions a profile reads exactly as it always did.
#ifndef PROTOSPEC_SDK_H
#define PROTOSPEC_SDK_H

#include "protospec/builders.h"
#include "protospec/class_edits.h"
#include "protospec/classes.h"
#include "protospec/edits.h"
#include "protospec/parents.h"
#include "protospec/plain_profile.h"
#include "protospec/profile.h"
#include "protospec/refs.h"
#include "protospec/traversal.h"

#endif  // PROTOSPEC_SDK_H
