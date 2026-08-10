// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// What a <mesh> is once it is in Unreal: an asset, not a path.
//
// MJCF names geometry by file. Unreal cannot draw a file, so the import pass
// turns each one into a UStaticMesh -- and where that asset lands is Unreal's
// decision, not URLab's: the same OBJ goes to `Meshes/<name>` through the FBX
// factory and to `Meshes/<name>/StaticMeshes/<name>` through Interchange, which
// is what runs when the mesh-preparation script has produced a GLB beside it.
// Every consumer that re-derived the package path from the MJCF name was
// therefore guessing, and guessed wrong for most real models.
//
// So the element records the asset instead. The importer already resolves it;
// this is where that answer is kept. A user opening the Blueprint gets an asset
// picker and can drop a different mesh in, which is the other half of the point.
//
// `File` still says what the spec said, and the two are reconciled on the
// way back out -- see MjSyncAssetFiles.

#include "CoreMinimal.h"

#include "MuJoCo/Gen/Elements/Assets/MjMesh.gen.h"

#include "MjMesh.generated.h"

class UStaticMesh;

/**
 * A `<mesh>` element and the Unreal asset it stands for.
 *
 * The generated base owns every MJCF attribute, `file` included. This owns the
 * asset that file became, which no attribute can describe.
 */
UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjMesh : public UMjMeshBase
{
	GENERATED_BODY()

public:
	/**
	 * The geometry this element draws and compiles with.
	 *
	 * Set by the import pass to the asset it produced from `File`. Swapping it
	 * replaces the mesh: the next write dumps the new asset beside the model and
	 * points `File` at what it wrote.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Mesh")
	TObjectPtr<UStaticMesh> MeshAsset;

	/**
	 * The asset `File` stands for.
	 *
	 * Written by the import pass, and again by every dump. It is what makes
	 * "the user swapped this" answerable without re-deriving anything: the two
	 * agree until someone points `MeshAsset` somewhere else.
	 */
	UPROPERTY()
	FSoftObjectPath FileAsset;

	/** True when `MeshAsset` is no longer the asset `File` stands for. */
	bool IsFileStale() const;

	/**
	 * Write `MeshAsset` out as an OBJ under `BaseDirectory` and repoint `File`.
	 *
	 * `BaseDirectory` is the directory MJCF resolves this element's `file`
	 * against, so the path written back is relative to it and the spec stays
	 * portable. False when there is nothing to write or the write failed.
	 */
	bool DumpAssetToFile(const FString& BaseDirectory);
};
