// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "MjbAssetBaker.generated.h"

struct mjModel_;
class AActor;
class UPrimitiveComponent;
class UProceduralMeshComponent;
class UStaticMesh;
class UTexture2D;
class UMaterialInterface;

/**
 * @class UMjbAssetBaker
 * @brief Builds (and caches) the meshes / textures / materials for a fast-path MJB.
 *
 * Turns a compiled mjModel's mesh + texture pools into UE render assets: a shared
 * UStaticMesh per mesh id (editor) or a ProceduralMeshComponent (packaged), a
 * UTexture2D per texture id, and a per-geom dynamic material bound onto the shared
 * master. Assets are keyed by a content hash (a hash of the MJB bytes) so an
 * identical model reuses one on-disk asset set across sessions and a saved level
 * reloads its geometry.
 *
 * A UObject so the caches (Master, TextureCache, StaticMeshCache) are UPROPERTYs
 * the GC roots between build and use. Holds the model by borrowed pointer (the
 * owning AMjbScene owns its lifetime); Init rebinds it, Reset drops it.
 */
UCLASS()
class URLAB_API UMjbAssetBaker : public UObject
{
	GENERATED_BODY()

public:
	/** Bind to a freshly loaded model: store the borrowed mjModel pointer, the
	 *  content hash for the cache folder, and the force-rebuild flag; (re)load the
	 *  shared master material. Call once per model load, before any build/apply. */
	void Init(mjModel_* InModel, const FString& InContentHash, bool bInForceRebuild);

	/** Drop the cached assets, the master, the content hash and the borrowed model
	 *  pointer. Call when the owning scene retires its model. */
	void Reset();

#if WITH_EDITOR
	// Editor route: a shared UStaticMesh keyed by mesh id (cheap PIE duplication);
	// BuildFromMeshDescriptions is editor-only. Null on a bad id.
	UStaticMesh* GetOrBuildStaticMesh(int32 MeshId);
#endif

	// Packaged-game route: a ProceduralMeshComponent that builds render data at
	// runtime (no editor mesh-build modules).
	UProceduralMeshComponent* BuildMesh(int32 GeomId, AActor* Body);

	// Create + bind a per-geom dynamic material instance (colour, textures, PBR
	// terms) onto Comp from the master. No-op if the master failed to load.
	void ApplyGeomMaterial(UPrimitiveComponent* Comp, int32 GeomId);

private:
	// Build (or fetch from cache) a UTexture2D from the MJB's tex_data for the given
	// MuJoCo texture id. bSRGB selects colour vs linear sampling. Null on a bad id.
	UTexture2D* GetOrBuildTexture(int32 TexId, bool bSRGB, bool bNormal = false);

	// The rgba a geom draws with: its material's when it has one, else its own.
	// Returns a pointer to 4 floats in the model; caller must hold a valid model.
	const float* GeomRgba(int32 GeomId) const;

	// Borrowed; the owning AMjbScene owns the mjModel/mjData lifetime.
	mjModel_* Model = nullptr;

	// A content id for the loaded MJB (a hash of its bytes). Cached, persistent
	// assets live under /Game/URLabFastPath/<ContentHash>/, so an identical model
	// reuses one asset set across sessions and a saved level reloads its geometry.
	FString ContentHash;

	// Force a fresh build of the cached, content-hash-keyed assets, ignoring any
	// already on disk.
	bool bForceRebuildAssets = false;

	// Rooted so GC can't collect the loaded master material between load and the
	// (possibly much later, on the PIE-reuse path) creation of its MIDs.
	UPROPERTY(Transient)
	TObjectPtr<UMaterialInterface> Master;

	// Textures built from the MJB's tex_data, keyed by MuJoCo texture id, so a
	// texture shared across materials/geoms is built once. Transient; rebuilt on
	// each model load.
	UPROPERTY(Transient)
	TMap<int32, TObjectPtr<UTexture2D>> TextureCache;

	// Static meshes built from the MJB's mesh pool, keyed by MuJoCo mesh id, so a
	// mesh shared across geoms is built once and every geom references the same
	// asset by pointer -- which keeps the PIE world duplication cheap (no embedded
	// vertex data copied per geom, unlike a ProceduralMeshComponent). Transient.
	UPROPERTY(Transient)
	TMap<int32, TObjectPtr<UStaticMesh>> StaticMeshCache;
};
