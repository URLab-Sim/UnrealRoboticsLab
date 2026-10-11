// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Render/RenderSceneResources.h"
#include "RenderAssetBuilder.generated.h"

class AActor;
class UPrimitiveComponent;
class UProceduralMeshComponent;
class UStaticMesh;
class UTexture2D;
class UMaterialInterface;
struct FRenderResolvedMaterialOverride;

/**
 * @class URenderAssetBuilder
 * @brief Builds and caches Unreal assets from one owned neutral resource bundle.
 *
 * Turns an owned scene-revision resource bundle into UE render assets: a shared
 * UStaticMesh per mesh resource index (editor) or a ProceduralMeshComponent (packaged), a
 * UTexture2D per texture resource index, and a per-geom dynamic material bound onto the shared
 * master. A play session always builds transient assets, in the editor and in a
 * cooked build alike, so both supported targets render from the same thing.
 * BakeLoadedModelToProject is the separate, explicit editor action that writes a
 * content-hash-keyed asset set into /Game for a saved level to reference.
 *
 * A UObject so the caches (Master, TextureCache, StaticMeshCache) are UPROPERTYs
 * the GC roots between build and use. It owns every CPU-side input it consumes.
 */
UCLASS()
class URLAB_API URenderAssetBuilder : public UObject
{
	GENERATED_BODY()

public:
	/** Validate resources and required renderer content without mutating a live builder. */
	static bool ValidateResources(const FRenderSceneResources& InResources, FString& OutError);

	/** Validate a complete candidate before atomically replacing resources and caches. */
	bool InitResources(FRenderSceneResources&& InResources, bool bInForceRebuild,
		FString& OutError);

	/** Drop the cached assets, master and owned resource bundle. */
	void Reset();

	const FRenderMeshResource* FindMeshResource(int32 MeshId) const
	{
		return Resources.Meshes.IsValidIndex(MeshId) ? &Resources.Meshes[MeshId] : nullptr;
	}
	const FString& GetSceneRevision() const { return Resources.SceneRevision; }
	const FRenderSceneResources& GetResources() const { return Resources; }
#if WITH_EDITOR
	// Editor route: a shared UStaticMesh keyed by mesh resource index.
	// BuildFromMeshDescriptions is editor-only. Null on a bad index.
	UStaticMesh* GetOrBuildStaticMesh(int32 MeshId);

	/** Explicit editor action: write the loaded model's meshes and textures into
	 *  /Game/URLabRender/<hash>/ as saved assets, so a saved render level can
	 *  reference them. Returns the number of packages written.
	 *
	 *  This is the ONLY path that saves packages. It deliberately is not part of a
	 *  play session: a cooked build cannot write (or read) /Game, so baking during
	 *  play meant the editor rendered from saved, editor-built assets while the
	 *  package rendered from runtime-built ones -- two supported targets producing
	 *  different camera images -- and made the first connect stall on package saves.
	 *  Play now always builds transient assets, in both builds. */
	int32 BakeLoadedModelToProject();
#endif

	// Packaged-game route: a ProceduralMeshComponent that builds render data at
	// runtime (no editor mesh-build modules).
	UProceduralMeshComponent* BuildMesh(int32 MeshId, AActor* Body);

	// Create + bind a per-geom dynamic material instance (colour, textures, PBR
	// terms) onto Comp from the master. No-op if the master failed to load.
	void ApplyGeomMaterial(UPrimitiveComponent* Comp, int32 GeomId);

	/** Apply a compiled-authoring material whose values and textures were resolved before
	 *  scene installation. The builder retains every referenced texture for GC. */
	void ApplyResolvedMaterialOverride(UPrimitiveComponent* Comp,
		const FRenderResolvedMaterialOverride& Override);

	/** Retain a resolved imported mesh/material/texture for the installed scene lifetime. */
	void RetainResolvedObject(UObject* Object);

	// The deformable twin of ApplyGeomMaterial, for renderer-created flexible/skin
	// surfaces: its resolved material appearance, or its surface colour over neutral
	// matte defaults. No-op if the master failed to load.
	void ApplyDeformableMaterial(UPrimitiveComponent* Comp, int32 SurfaceIndex);

	/** Build (or fetch from cache) a UTexture2D from owned RGBA8 data for the
	 *  given texture index. bSRGB selects colour vs linear sampling; bNormal
	 *  marks a tangent-space normal map, which is filtered as vectors and tagged
	 *  with the normal-map codec. Null on a bad id.
	 *
	 *  Part of the build surface (not just an internal of the material bind) so a
	 *  test can assert what a play session actually hands the renderer: a
	 *  transient, fully mipped texture in both builds. */
	UTexture2D* GetOrBuildTexture(int32 TexId, bool bSRGB, bool bNormal = false);

private:
	// Bind the resource textures for every role material MatId fills onto Mid
	// (colour roles sample sRGB, data roles linear). No-op for MatId < 0. Shared
	// by ApplyGeomMaterial and ApplyFlexMaterial.
	void BindMaterialRoleTextures(class UMaterialInstanceDynamic& Mid, int32 MatId);

	// A deterministic content id for the owned resources. It names the folder
	// the explicit bake writes to, /Game/URLabRender/<ContentHash>/, so a saved
	// level reloads the geometry an identical model produced. Play does not read
	// that folder.
	FString ContentHash;

	// Force the bake to rebuild its content-hash-keyed assets, ignoring any
	// already on disk. Only BakeLoadedModelToProject consults it.
	bool bForceRebuildAssets = false;

	// True only for the duration of BakeLoadedModelToProject. Gates every
	// /Game package load and save; false (the play-time value) keeps both build
	// routes on transient assets so editor and cooked runtime installs render the same thing.
	bool bBakeToProject = false;

	// Rooted so GC cannot collect the loaded master material while components create MIDs.
	UPROPERTY(Transient)
	TObjectPtr<UMaterialInterface> Master;

	// Textures built from owned RGBA8 resources, keyed by texture resource index, so a
	// texture shared across materials/geoms is built once. Transient; rebuilt on
	// each model load.
	UPROPERTY(Transient)
	TMap<uint64, TObjectPtr<UTexture2D>> TextureCache;

	FRenderSceneResources Resources;

	// Static meshes built from the owned mesh pool, keyed by mesh resource index, so a
	// mesh shared across geoms is built once and every geom references the same
	// asset by pointer rather than copying vertex data per geom. Transient.
	UPROPERTY(Transient)
	TMap<int32, TObjectPtr<UStaticMesh>> StaticMeshCache;

	/** Strong GC-visible ownership for resolved compiled-authoring asset overrides. */
	UPROPERTY(Transient)
	TArray<TObjectPtr<UObject>> ResolvedObjects;
};
