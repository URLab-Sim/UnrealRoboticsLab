// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Parsing an authored component tree through MJCF into a native mjSpec.
//
// The component tree is the authored document; an mjSpec is what MuJoCo
// compiles. This is the one-way crossing between them: components are read,
// never written, and the spec that comes back is a separate object with its
// own lifetime.
//
// Element identity is reconciled through compile-only reserved names and kept
// beside the spec for its lifetime.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Spec/MjAssetSink.h"
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
 * Serialize Root to MJCF and parse it with MuJoCo's native XML reader.
 *
 * VfsPrefix matches the asset mount prefix the scene assembler will use.
 * Asset paths and compiler directories are substituted in the writer only;
 * authored fields remain unchanged.
 *
 * On error the result's Spec is null and OutDiags carries at least one
 * diagnostic of Error severity. A build that succeeded can still fill OutDiags:
 * a warning is something the user needs told about a document that built, so
 * "did this work" is `Spec != nullptr`, or `MjAnyError(OutDiags)`, never whether
 * the array is empty.
 *
 * Native XML errors are reported as returned by MuJoCo's parser.
 */
URLAB_API FMjBuiltSpec BuildSpec(const FSpecRef& Root, TArray<FMjSpecDiagnostic>& OutDiags,
	const FString& VfsPrefix = FString());

/**
 * Point a built spec's asset references at the names its bytes are mounted
 * under, and clear the directories that resolved them.
 *
 * Anyone who mounts what an FMjAssetSink emitted has to call this before
 * compiling, whether or not anything is being composed. The sink's names are
 * complete, so the spec's own meshdir and texturedir would prepend a directory
 * to a name that already resolves -- and MuJoCo answers a lookup that misses by
 * falling back to a case-insensitive BASENAME match across every mount, so the
 * miss does not fail the compile. It silently returns whichever file was
 * mounted first under that basename. A model whose `visual/link.stl` and
 * `collision/link.stl` differ only by directory then compiles clean with its
 * collision geometry replaced by its visual mesh.
 *
 * Requests must be the ones emitted for THIS spec: they carry the element
 * pointers the rewrite is applied through.
 */
URLAB_API void MjNamespaceAssets(const FMjBuiltSpec& Built, const TArray<FMjAssetRequest>& Requests);

} // namespace urlab::spec

#endif // URLAB_MJ_GEN
