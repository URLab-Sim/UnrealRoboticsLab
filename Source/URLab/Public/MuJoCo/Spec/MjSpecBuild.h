// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Building an mjSpec from a component tree.
//
// The component tree is the authored document; an mjSpec is what MuJoCo
// compiles. This is the one-way crossing between them: components are read,
// never written, and the spec that comes back is a separate object with its
// own lifetime.
//
// Element identity is recorded during the build, while the correspondence is
// known, rather than recovered afterwards by matching names. It is kept beside
// the spec because that is exactly how long it stays meaningful.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Spec/MjSpecRef.h"

struct mjSpec_;
typedef struct mjSpec_ mjSpec;
struct mjsElement_;
typedef struct mjsElement_ mjsElement;

class UMjNodeComponent;

namespace urlab::spec
{

/**
 * An mjSpec built from a component tree, with per-component element identity.
 *
 * Owns the spec: destruction calls mj_deleteSpec. Move-only, because two
 * owners of one mjSpec is a double free waiting for a bad day.
 *
 * ElementFor holds raw mjsElement pointers into the owned spec. They are
 * meaningful only while this object is alive, and the map is never handed to
 * components: a component stores an int32 id and nothing else, so no pointer
 * into a spec can outlive the spec through one.
 */
struct URLAB_API FMjBuiltSpec
{
	FMjBuiltSpec() = default;
	FMjBuiltSpec(FMjBuiltSpec&& Other);
	FMjBuiltSpec& operator=(FMjBuiltSpec&& Other);
	~FMjBuiltSpec();
	FMjBuiltSpec(const FMjBuiltSpec&) = delete;
	FMjBuiltSpec& operator=(const FMjBuiltSpec&) = delete;

	mjSpec* Spec = nullptr;
	TMap<TObjectPtr<const UMjNodeComponent>, mjsElement*> ElementFor;
};

/**
 * Populate an mjSpec from the component tree under Root.
 *
 * Reads the components, never mutates them. On error the result's Spec is null
 * and OutDiags carries at least one diagnostic; diagnostics carry the
 * offending component's source file and line where one is implicated.
 */
URLAB_API FMjBuiltSpec BuildSpec(const FSpecRef& Root, TArray<FMjSpecDiagnostic>& OutDiags);

}  // namespace urlab::spec

#endif  // URLAB_MJ_GEN
