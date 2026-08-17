// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"

class AMjRenderer;
class UMeshComponent;
class UMaterialInterface;
class UMaterialInstanceDynamic;
enum class EMjDebugShaderMode : uint8;

/**
 * @struct FMjRendererOverlay
 * @brief Debug material-tint overlay owned by the renderer that owns the geoms.
 *
 * Swaps per-geom overlay MIDs on an AMjRenderer's own geom components (and, in the
 * editor, the authoring visualizer meshes + quick-convert props), coloured by a
 * debug shader mode. Records each mesh's original slot materials so the tint can be
 * cleared and the baked appearance restored. A plain struct owned by AMjRenderer:
 * it holds only the record/MID caches (never GC-roots -- the MIDs are kept alive by
 * the mesh components they are applied to) and reaches back into the scene (its
 * friend) for the shared overlay parent material, the model, and its geom components.
 */
struct FMjRendererOverlay
{
	// Original slot-0 materials on geom components we've overridden, so we can restore.
	TMap<TWeakObjectPtr<UMeshComponent>, TObjectPtr<UMaterialInterface>> OriginalMaterials;
	// Original slot-1..N materials for multi-material meshes. Parallel to OriginalMaterials.
	TMap<TWeakObjectPtr<UMeshComponent>, TMap<int32, TObjectPtr<UMaterialInterface>>> OriginalSlotMaterials;
	// Dynamic material instances we created per mesh, reused across drives.
	TMap<TWeakObjectPtr<UMeshComponent>, TObjectPtr<UMaterialInstanceDynamic>> ActiveMIDs;

	// Load /Engine/BasicShapes/BasicShapeMaterial and record its first vector param name
	// on the scene, so the overlay MIDs and segmentation siblings have a tint parameter.
	void Initialize(AMjRenderer& Scene);
	// Swap per-geom overlay MIDs on the scene's own geom components, coloured by Mode
	// from the physics-thread island seed / awake snapshot.
	void Apply(AMjRenderer& Scene, EMjDebugShaderMode Mode, const TArray<int32>& BodyAwake,
		const TArray<int32>& BodyIslandSeed, bool bModulateBySleep,
		float SleepValueScale, float SleepSaturationScale);
	// Restore the materials recorded at apply time and clear the caches.
	void Clear();
};
