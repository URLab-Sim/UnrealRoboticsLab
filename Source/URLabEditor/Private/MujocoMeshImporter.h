// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// A file on disk in, a saved Unreal asset out.
//
// Separate from the pass that decides which files to import because none of
// this knows anything about MJCF. It is Unreal's own importers with the
// settings a robot mesh needs already chosen -- weighted normals, no Nanite, no
// generated collision -- plus the fallbacks that make the OBJ and STL a MuJoCo
// model ships actually load.

#include "CoreMinimal.h"

class UMaterialInstanceConstant;
class UStaticMesh;
class UTexture2D;

namespace urlab::editor
{

/**
 * Import `SourcePath` as a UStaticMesh called `AssetName` under `DestinationPath`.
 *
 * `AssetName` is the MJCF `<mesh>` name rather than the file's, because that is
 * the name every geom refers to and MJCF lets the two differ.
 *
 * Prefers an FBX, GLB or GLTF sibling of the same base name over the file the
 * spec actually named: a MuJoCo model ships OBJ and STL, and both throw away
 * smoothing information that Unreal then has to guess at. Returns the existing
 * asset untouched when one is already there, so re-importing a scene that shares
 * meshes between robots does the work once.
 */
UStaticMesh* ImportMeshAsset(const FString& SourcePath, const FString& DestinationPath, const FString& AssetName);

/**
 * Import `SourcePath` as a UTexture2D called `AssetName` under `DestinationPath`.
 *
 * `AssetName` is the MJCF `<texture>` name rather than the image's filename,
 * because that is the name every reference to it uses: a material's `<layer>`
 * names a texture, never a file. Sanitised on the way in, so a namespaced MJCF
 * name is a legal package name. `bSrgb` follows the element's `colorspace`.
 *
 * Returns the existing asset untouched when one is already there, so
 * re-importing a scene that shares textures does the work once.
 */
UTexture2D* ImportTextureAsset(
	const FString& SourcePath, const FString& DestinationPath, const FString& AssetName, bool bSrgb);

}  // namespace urlab::editor
