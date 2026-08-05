// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/BoxComponent.h"
#include "Components/LineBatchComponent.h"

#include "MuJoCo/Spec/MjSceneContributor.h"

#include "AMjHeightfieldActor.generated.h"

class UMjModel;

/**
 * A box-bounded sampler that turns level geometry into a MuJoCo heightfield.
 *
 * Place the actor over a landscape or any other static geometry and scale its
 * box to cover the region of interest. Every sample is a downward raycast, which
 * at useful resolutions is expensive enough that the result is cached to disk and
 * keyed on the bounds, resolution and base thickness that produced it.
 *
 * The grid drawn in the editor viewport is the sample set itself, projected onto
 * whatever the rays hit, so the preview and the data are one computation. It is
 * rebuilt from OnConstruction, which is what makes a Details-panel edit show up.
 *
 * What the sampling produces is a spec: an `<hfield>` under `<asset>` and a
 * `<geom type="hfield">` in the world body, authored onto this actor as ordinary
 * element components. The elevation is carried inline on the `<hfield>` element
 * rather than through a file, because MJCF has an attribute for exactly that and
 * a generated file would be a second place for the samples to live.
 *
 * The spec is contributed at identity, not at this actor's transform. The
 * samples are taken in world space against the axis-aligned bounds of the box,
 * so the actor's own rotation is already accounted for; attaching under its
 * transform would apply it a second time and rotate terrain that was measured
 * flat.
 */
UCLASS()
class URLAB_API AMjHeightfieldActor : public AActor, public IMjSceneContributor
{
	GENERATED_BODY()

public:
	AMjHeightfieldActor();

	// -------------------------------------------------------------------------
	// Config
	// -------------------------------------------------------------------------

	/**
	 * Sample points along each axis, giving a Resolution x Resolution grid.
	 *
	 * Detail and sampling cost both scale with the square of this, and it is the
	 * dominant term in how long a re-sample takes.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Heightfield", meta = (ClampMin = "2", ClampMax = "512"))
	int32 Resolution = 64;

	/** Name for the sampled field. Unique per level when several are placed. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Heightfield")
	FString HFieldName = TEXT("terrain");

	/**
	 * The channel the downward height rays trace against.
	 *
	 * It has to be one the landscape or terrain mesh responds to. ECC_Visibility
	 * traces the visual mesh, which follows the terrain more closely than its
	 * simplified collision does.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Heightfield")
	TEnumAsByte<ECollisionChannel> ElevationTraceChannel = ECC_Visibility;

	/** Re-sample on the next pass, ignoring any cached heightfield data. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Heightfield")
	bool bForceRecache = false;

	/**
	 * The actors height sampling is allowed to hit.
	 *
	 * Non-empty means only these count and everything else is traced through.
	 * Empty means the rays hit anything, minus the MuJoCo actors the sampler
	 * always filters out so that a simulated body cannot become terrain.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Heightfield",
		meta = (ToolTip = "If set, only these actors will be sampled for height. Leave empty to trace against everything."))
	TArray<TSoftObjectPtr<AActor>> TraceWhitelist;

	/**
	 * Thickness of the solid base under the sampled surface, as a fraction of
	 * the elevation range. MuJoCo's `base` heightfield parameter.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Heightfield", meta = (ClampMin = "0.01", ClampMax = "1.0"))
	float BaseThickness = 0.1f;

	// -------------------------------------------------------------------------
	// Visualizer
	// -------------------------------------------------------------------------

	/** Draw the grid preview in the editor viewport. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Heightfield|Visualizer")
	bool bShowGrid = true;

	/** Colour of the grid lines. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Heightfield|Visualizer")
	FLinearColor GridColor = FLinearColor(0.0f, 1.0f, 1.0f, 1.0f); // Cyan

	// -------------------------------------------------------------------------
	// Components
	// -------------------------------------------------------------------------

	/** The sampling region. Move and scale this to cover the terrain. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Heightfield")
	UBoxComponent* BoundsBox;

	/** Holds the grid preview's lines. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Heightfield|Visualizer")
	ULineBatchComponent* GridLines;

	/**
	 * The root of the spec this actor contributes to the scene.
	 *
	 * Attached under the bounding box rather than replacing it as the actor's
	 * root: the box is what the user scales, and the spec is what the
	 * compiler reads. `FSpecRef::OverActor` finds this by looking for the MuJoCo
	 * node with no MuJoCo parent, which the box being a plain scene component
	 * leaves true.
	 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Heightfield")
	TObjectPtr<UMjModel> Spec;

	// -------------------------------------------------------------------------
	// Scene contribution
	// -------------------------------------------------------------------------

	/**
	 * Raycast the terrain under the bounding box, normalise the heights, and
	 * author them as an `<hfield>` plus the world geom that references it.
	 *
	 * Idempotent: the previous pass's elements are destroyed first, so a
	 * recompile leaves one heightfield rather than a stack of them.
	 */
	virtual void AuthorSceneSpec() override;
	virtual FSpecRef GetSceneSpec() const override;
	virtual FString GetScenePrefix() const override;

	/** The sampled `<hfield>` element, once one has been authored. */
	UMjNodeComponent* GetHfieldElement() const { return HfieldElement; }

	/** The `<geom type="hfield">` that places the sampled field in the world. */
	UMjNodeComponent* GetHfieldGeomElement() const { return HfieldGeomElement; }

protected:
	virtual void OnConstruction(const FTransform& Transform) override;

private:
	/** Rebuild the preview grid against the current resolution and bounds. */
	void RebuildGridVisualizer();

	/** Trace down for the terrain height at one point, skipping MuJoCo actors. */
	float SampleHeightAt(const FVector2D& WorldXY, const FBox& Bounds) const;

	/**
	 * Sample the grid, or read back the cached samples that match this
	 * configuration.
	 *
	 * Row order is the one MJCF's `elevation` attribute wants, which is not the
	 * one MuJoCo stores: the reader copies the attribute in reverse row order so
	 * that the text reads top to bottom. Row 0 is therefore minimum Unreal Y,
	 * which is maximum MuJoCo Y, which is the last stored row.
	 */
	bool SampleElevation(TArray<float>& OutNormHeights, float& OutMinHeightCm, float& OutRangeCm) const;

	/** The elements the last authoring pass produced. Rebuilt, never patched. */
	UPROPERTY(Transient)
	TObjectPtr<UMjNodeComponent> HfieldElement;

	UPROPERTY(Transient)
	TObjectPtr<UMjNodeComponent> HfieldGeomElement;

	/** Where this heightfield's cache file lives. */
	FString GetCacheFilePath() const;

	/**
	 * What a cached sample set is valid for: this actor's bounds, resolution and
	 * base thickness. Anything else that changes leaves the samples correct.
	 */
	FString ComputeCacheKey() const;

	/** Write the sampled field and its key to the binary cache file. */
	bool SaveCache(const TArray<float>& NormHeights, float MinH, float ElevRange,
		const FBox& Bounds, const FString& CacheKey) const;

	/** Read the cache back, returning false unless its key still matches. */
	bool LoadCache(TArray<float>& OutNormHeights, float& OutMinH, float& OutElevRange,
		FBox& OutBounds, const FString& ExpectedCacheKey) const;
};
