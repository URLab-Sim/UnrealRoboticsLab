// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include "CoreMinimal.h"

#include "MuJoCo/Entity/MjLightHandle.h"
#include "MuJoCo/Entity/MjLightOverride.h"

#include "MjLightStore.generated.h"

class ULightComponentBase;
class UWorld;

/**
 * Runtime light control, built on the same shape as UMjAppearanceStore: a
 * name-keyed override that is applied to every live light carrying that name and
 * re-applied after a rebuild.
 *
 * Lights arrive one of two ways and both are addressable by name. Lights the
 * model brought in carry a UMjLightHandle stamped by MjSkyImporter with their
 * MJCF name; anything else in the level -- a hand-placed sun, a practical in an
 * art scene -- is addressed by its actor name. That second route is what makes
 * the ops useful on a render server whose level was not built from an MJCF.
 *
 * Overrides survive a model rebuild the way appearance overrides do: the
 * importer destroys and re-spawns every imported light, then ReapplyAll
 * re-drives the held overrides onto the fresh actors by name.
 *
 * Restoring is baseline-based rather than re-running an authoring pass, because
 * there is no cheap "re-import just this light" call: the first override on a
 * component snapshots what it was, and clearing puts that back. Baselines are
 * keyed weakly, so a rebuild's dead components fall out on their own and the new
 * ones snapshot afresh.
 */
UCLASS()
class URLAB_API UMjLightStore : public UObject
{
	GENERATED_BODY()

public:
	/** Bind the world whose lights this store drives. */
	void InitForWorld(UWorld* InWorld);

	/** One light as the client sees it: enough to address it and to read back the
	 *  values an absolute-intensity override would have to speak in. */
	struct FLightInfo
	{
		/** Matching key. Case-insensitive; do not put it on the wire. */
		FName Name;
		/** What the client sees. Preserves case in a cooked build, which
		 *  `Name.ToString()` does not -- see UMjLightHandle::MjNameExact. */
		FString NameExact;
		EMjLightKind Kind = EMjLightKind::Unknown;
		int32 MjId = INDEX_NONE;
		float Intensity = 0.0f;
		/** "lux" / "candelas" / "unitless" -- what Intensity is measured in. */
		FString IntensityUnits;
		FLinearColor Colour = FLinearColor::White;
		bool bEnabled = true;
		bool bCastShadow = false;
		/** True when the light came from the model (carries a UMjLightHandle). */
		bool bImported = false;
		bool bOverridden = false;
	};

	/** Every addressable light in the world. The client needs this: unlike geoms,
	 *  light names are not in the MJCF the client already holds once the level
	 *  contributes lights of its own. */
	TArray<FLightInfo> ListLights() const;

	/** What a light name resolves to. */
	struct FResolution
	{
		int32 Components = 0;
		int32 MjId = INDEX_NONE;
		EMjLightKind Kind = EMjLightKind::Unknown;

		bool IsFound() const { return Components > 0; }
	};

	FResolution ResolveLight(FName LightName) const;

	/** Store an override for `LightName` and apply it now. Returns the number of
	 *  live lights driven; zero when none is built yet (the override is still held
	 *  and lands on the next rebuild). */
	int32 SetOverride(FName LightName, const FMjLightOverride& Override);

	/** Drop the override for `LightName` and restore its baseline. Returns the
	 *  number restored, or -1 when no override was held. */
	int32 ClearOverride(FName LightName);

	/** Drop every override and restore every baseline. Returns the number restored. */
	int32 ClearAll();

	/** Re-drive every held override onto the current lights, after a rebuild. */
	void ReapplyAll();

	bool HasOverride(FName LightName) const { return Overrides.Contains(LightName); }

	const FMjLightOverride* FindOverride(FName LightName) const
	{
		return Overrides.Find(LightName);
	}

	int32 Num() const { return Overrides.Num(); }

private:
	/** One live light the store can drive, plus the identity it was found under. */
	struct FTarget
	{
		AActor* Owner = nullptr;
		ULightComponentBase* Component = nullptr;
		UMjLightHandle* Handle = nullptr;
		EMjLightKind Kind = EMjLightKind::Unknown;
		/** A sky light is driven through USkyLightComponent's own setters. */
		bool bSky = false;
	};

	/** What a component looked like before the first override touched it. */
	struct FBaseline
	{
		float Intensity = 0.0f;
		FLinearColor Colour = FLinearColor::White;
		FTransform OwnerTransform;
		bool bVisible = true;
		bool bCastShadow = false;
		/** True once an override actually moved the actor. A light that is never
		 *  posed by a client may still be moved by the scene (a light tracking a
		 *  body), and putting a stale transform back on clear would teleport it. */
		bool bPoseOverridden = false;
	};

	/** Collect the live lights named `LightName` (empty name = every light). */
	void CollectTargets(FName LightName, TArray<FTarget>& Out) const;

	/** Drive one light from `Override`, or restore its baseline when null. */
	void ApplyToTarget(const FTarget& Target, const FMjLightOverride* Override);

	int32 ApplyToWorld(FName LightName, const FMjLightOverride* Override);

	UWorld* ResolveWorld() const;

	TWeakObjectPtr<UWorld> World;

	TMap<FName, FMjLightOverride> Overrides;

	/** Pre-override state, keyed weakly so a retired component's entry lapses. */
	TMap<TWeakObjectPtr<ULightComponentBase>, FBaseline> Baselines;
};
