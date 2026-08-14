// ProtoSpec public API — runtime reflection + the generic visitor.
//
// Static schema description usable without compile-time knowledge of the
// element structs (namespace `ps::mjcf::reflect`): ElementDescriptor,
// FieldDescriptor, ChildDescriptor, UnionDescriptor, Describe / DescribeByName /
// DescribeUnion, and the per-element field-count constants. Plus the generated
// `ps::mjcf::Visit(elem, V)` hook that hands a visitor every field (by id, name,
// typed reference) and every child / union-child list.
//
// This is the contract a generic, schema-driven consumer builds on — a property
// panel, a tree view, a serializer — without a line of per-element code. Most
// authoring does not need it: prefer <protospec/sdk.h> for typed builders and
// traversal. Reach for reflection only when you are writing tooling that must
// work uniformly across every element type.
#ifndef PROTOSPEC_PUBLIC_REFLECT_H
#define PROTOSPEC_PUBLIC_REFLECT_H

#include "protospec/model.h"

#include "../../generated/reflect.h"  // ps::mjcf::reflect::*
#include "../../generated/visit.h"    // ps::mjcf::Visit

#endif  // PROTOSPEC_PUBLIC_REFLECT_H
