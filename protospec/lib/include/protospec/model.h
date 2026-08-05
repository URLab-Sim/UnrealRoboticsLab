// ProtoSpec public API — the object model.
//
// The MuJoCo model as plain, presence-tracked, owned-value C++ (namespace
// `ps::mjcf`): every element type (Body, Geom, Joint, Material, Mesh, ...), the
// union-child wrappers, the enums and variant aliases, plus the value
// operations over them:
//
//   Clone(elem)          deep copy of an element (fresh serials)
//   ApplyDefault(elem)   seed the IDL `=` defaults on request (DR-1: never
//                        written silently)
//   ToMjcf / FromMjcf    enum <-> MJCF keyword string conversion
//
// The runtime primitives the model is built from (`ps::opt`, `ps::Ref`,
// `ps::InlineVec`, `ps::SourceLoc`) arrive via <protospec/core.h>.
//
// This is the one header a consumer includes to name and manipulate model data.
// Companion public headers:
//   <protospec/sdk.h>       ergonomic authoring (builders, traversal, refs,
//                           default classes, attach)
//   <protospec/io.h>        MJCF parse / write
//   <protospec/compile.h>   compile to a MuJoCo mjModel + Binding
//   <protospec/validate.h>  three-tier structural / referential / semantic checks
//   <protospec/reflect.h>   runtime reflection + the generic Visit hook
//   <protospec/protospec.h> all of the above at once
#ifndef PROTOSPEC_PUBLIC_MODEL_H
#define PROTOSPEC_PUBLIC_MODEL_H

#include "protospec/core.h"

// The generated object model + its value operations. These are internal
// generated headers reached through this public umbrella; a consumer names
// <protospec/model.h>, never the generated headers directly.
#include "../../generated/types.h"     // element/enum/union/variant types, Clone
#include "../../generated/keywords.h"  // ToMjcf / FromMjcf
#include "../../generated/defaults.h"  // ApplyDefault

#endif  // PROTOSPEC_PUBLIC_MODEL_H
