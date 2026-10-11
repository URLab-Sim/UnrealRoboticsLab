// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// The way back out: a UAsset becomes a file again.
//
// Import turns `file="body.obj"` into a UStaticMesh and the element keeps the
// asset. MuJoCo cannot read a UStaticMesh, so anything that compiles or writes
// the spec has to put a file back where `file` says one is. That is this
// pass, and it runs before the writer rather than inside it, because the answer
// it produces is an edit to the spec -- `file` really does now name a
// different thing -- and not a detail of one serialisation.
//
// It does nothing at all to an element nobody touched. The element remembers
// which asset its `file` stands for, so "the user swapped this mesh" is a
// comparison rather than a guess, and a model that was only looked at writes
// back byte for byte.
//
// What gets written, and where:
//
//   - A `<mesh>` goes out as an OBJ through the existing export in MeshUtils,
//     which is the one that already gets metres, handedness and winding right.
//   - A `<texture>` goes out as a PNG from the asset's source pixels.
//   - Both land in a `urlab_assets` folder inside the directory MJCF already
//     resolves that element's `file` against -- beside the model, under meshdir
//     or texturedir when the spec declares one -- so the path written back
//     is relative and the spec stays portable. It is the same place the
//     mesh-preparation script already writes its GLBs into.
//   - `<hfield>` and `<skin>` name files too and are deliberately left out:
//     neither has an imported Unreal asset to write back, so there is nothing
//     for a reference to point at and nothing for this to dump.

#include "CoreMinimal.h"

struct FSpecRef;

/** The folder a dumped asset goes into, inside the element's own base directory. */
inline const TCHAR* const MjDumpedAssetFolder = TEXT("urlab_assets");

/** Where one dumped asset goes, and what `file` says about it afterwards. */
struct URLAB_API FMjDumpTarget
{
	/** The file to write. */
	FString FullPath;

	/** The value `file` takes once it is written. */
	FString FileAttribute;
};

/**
 * Place a dump for an element whose `file` resolves against `BaseDirectory`.
 *
 * With a base directory the path written back is relative to it, so the
 * spec keeps working wherever it is copied to. Without one -- a spec
 * parsed from text that came from no file -- there is nothing to be relative
 * to, so the dump goes to the project's own scratch folder and `file` is
 * absolute, which is a spelling MuJoCo accepts.
 */
URLAB_API FMjDumpTarget MjDumpTargetFor(const FString& BaseDirectory, const FString& Name, const TCHAR* Extension);

/**
 * Reconcile every file-backed asset element with the asset it references.
 *
 * Idempotent: an element whose `file` already stands for the asset it
 * references is left untouched, which after one dump is every element again.
 */
URLAB_API void MjSyncAssetFiles(const FSpecRef& Spec);
