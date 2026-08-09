// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Reserved names for the unnamed, while a spec is being built.
//
// An element that reaches the compiled model without an authored name arrives
// there carrying a generated one. The spec path does not need it to bind -- it
// binds by element handle -- but the name is what leaves the engine: the bridge
// reconciles by name, so an element that used to arrive named has to keep
// arriving named.
//
// One half of the reservation is not here, and deliberately: the file-derived
// name of an unnamed asset. This path pins that one onto the ELEMENT, from the
// same derivation, at the point it rewrites `file`, so the elements a basename
// would be reserved for are exactly the ones skipped below.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN

struct FSpecRef;

class UMjNodeComponent;

namespace urlab::spec
{

/**
 * What a reserved name begins with.
 *
 * Shared with ProtoSpec's own default, and with the writer that emits the
 * scene's MJCF: a client reconciling the two by name has to see one spelling.
 */
inline const TCHAR* const MjReservedNamePrefix = TEXT("_ps:");

/**
 * Name the unnamed of one spec, and take the names off again.
 *
 * The tree is handed back exactly as it was found. Authoring the names for real
 * would put identity nobody asked for into the user's Blueprint, and it would
 * show up in the next diff of their MJCF.
 *
 * The names go on the COMPONENTS rather than on the built elements because that
 * is where the build reads them from: the walk copies them onto the spec, and
 * the macro bridge serializes a subtree back out to MJCF text, so an element
 * inside a `<replicate>` is named only if its component is.
 */
class FMjReservedNames
{
public:
	explicit FMjReservedNames(const FSpecRef& Spec);
	~FMjReservedNames();

	FMjReservedNames(const FMjReservedNames&) = delete;
	FMjReservedNames& operator=(const FMjReservedNames&) = delete;

private:
	TArray<UMjNodeComponent*> Renamed;
};

}  // namespace urlab::spec

#endif  // URLAB_MJ_GEN
