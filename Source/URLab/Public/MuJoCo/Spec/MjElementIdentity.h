// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// "What element is this?", from outside the runtime module.
//
// The generated dispatch answers this already, but its runtime tables are plain
// free functions with no export macro: they link inside URLab and nowhere else.
// The editor's outliner, its details customizations, its level operations and
// the ROS transports all need the answer, and the emitter is not the place to
// fix it -- an export macro on generated code would put a module's linkage
// decision inside the schema.
//
// So the seam is here, where every other hand-side entry into the generated
// half already lives, and it is four functions wide.
//
// The class form is the load-bearing one. A hand subclass of a generated
// element IS that element: dispatch walks the superclass chain to the nearest
// generated base, which is what lets a presentation class carry a render target
// without the reader, the writer or the compiler learning about it. Nothing
// asserted that until now.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN

class UMjNodeComponent;

// Declared rather than included: the generated reflection tables are a large
// header and everything below hands back a pointer into them.
namespace ps::mjcf::reflect
{
struct FieldDescriptor;
}

namespace urlab::spec
{

/**
 * The schema element `Node` is.
 *
 * False when it is not a spec element at all, which for a `UMjNodeComponent`
 * means the abstract base itself and nothing else.
 */
URLAB_API bool MjElementTypeOfNode(const UMjNodeComponent& Node, psm::ElementType& Out);

/**
 * The schema element `Class` names, or that its nearest generated base names.
 *
 * The subclass rule lives here: `UMjGeom : UMjGeomBase` reports Geom, so a
 * presentation class is never a second kind of element.
 */
URLAB_API bool MjElementTypeOfClass(const UClass* Class, psm::ElementType& Out);

/** The generated class an element type names, before any override. */
URLAB_API UClass* MjGeneratedClassOf(psm::ElementType Type);

/** The element's own MJCF tag: what it is called at the top level. */
URLAB_API const TCHAR* MjTagOf(psm::ElementType Type);

/**
 * The schema's description of the attribute `Type` spells `Xml`, or null.
 *
 * The generated reflection tables are indexed by IDL field name; a caller
 * holding an MJCF attribute name has to walk them. One walk, here, rather than
 * one per caller: the scale policy reads a field's kind off it and the details
 * panel reads its arity, and two copies of the same loop are two places for the
 * schema's spelling of an attribute to be got wrong.
 */
URLAB_API const psm::reflect::FieldDescriptor* MjSchemaFieldOf(psm::ElementType Type, const char* Xml);

/** How many values `Type` reads out of its `Xml` attribute; 0 when it has none. */
URLAB_API int32 MjSchemaArityOf(psm::ElementType Type, const char* Xml);

} // namespace urlab::spec

#endif // URLAB_MJ_GEN
