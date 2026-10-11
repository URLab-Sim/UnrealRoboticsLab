// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include "CoreMinimal.h"

#include "MuJoCo/Entity/MjGeomAppearance.h"

#include "MjAppearanceStore.generated.h"

class AAMjManager;
class UTexture;
class UWorld;

/**
 * The visual appearance channel: name-keyed appearance
 * overrides that re-drive a geom's live material instance.
 *
 * An override is stored by geom name and applied immediately to every live
 * component that geom has, across both render paths -- the authoring `UMjGeom`
 * previews and the render-tagged actors -- by driving each one's existing
 * dynamic material instance through `MjAppearance::Apply`. The overrides survive
 * a render rebuild: `ReapplyAll` re-drives them onto the freshly built
 * components after a render scene swap or a PIE reindex.
 *
 * Texture bindings name content, not bytes: a binding key resolves either to a
 * UE texture asset by object path or to an already-uploaded blob in the
 * content-addressed asset cache (keyed by its SHA-256), so the same upload path
 * the model manifest uses carries randomization textures with no second channel.
 */
UCLASS()
class URLAB_API UMjAppearanceStore : public UObject
{
	GENERATED_BODY()

public:
	/** Bind to the owning manager, whose world the apply walk scans. */
	void Init(AAMjManager* InManager);

	/** Bind a world directly, for the no-manager case: a render server or a stream
	 *  subscriber has a world and an AMjRenderer but no physics and no manager, and its
	 *  materials are just as overridable. Geom ids then resolve off the renderer's
	 *  own model instead of the physics engine's. */
	void InitForWorld(UWorld* InWorld);

	/** What a geom name resolves to across the live render paths. */
	struct FResolution
	{
		/** Live authoring `UMjGeom` components carrying that name. */
		int32 AuthoringComponents = 0;

		/** Reserved for neutral runtime presentation components. */
		int32 RenderComponents = 0;

		/** The compiled geom id, or INDEX_NONE when no compiled model has it. */
		int32 MjId = INDEX_NONE;

		bool IsFound() const
		{
			return AuthoringComponents > 0 || RenderComponents > 0 || MjId != INDEX_NONE;
		}
	};

	/**
	 * Resolve a geom name to a stable handle: the compiled id plus a live-component
	 * census across both render paths. Optionally scoped to `Entity` (the owning
	 * actor for the authoring path). Reads nothing off the mjModel beyond the id.
	 */
	FResolution ResolveGeom(FName GeomName, FName Entity = NAME_None) const;

	/**
	 * Store an override for `GeomName` and apply it at once. Returns the number of
	 * live material instances re-driven; zero when the geom is not yet built (the
	 * override is still stored and applies on the next rebuild).
	 */
	int32 SetOverride(FName GeomName, const FMjGeomAppearance& Appearance, FName Entity = NAME_None);

	/**
	 * Drop the override for `GeomName` and restore the authored appearance on its
	 * live components. Returns the number restored, or -1 when no override was held.
	 */
	int32 ClearOverride(FName GeomName);

	/** Drop every override and restore the authored appearance on all of them.
	 *  Returns the number of live materials restored, so a clear-everything reply
	 *  can say what it actually touched rather than reporting zero. */
	int32 ClearAll();

	bool HasOverride(FName GeomName) const { return Overrides.Contains(GeomName); }

	const FMjGeomAppearance* FindOverride(FName GeomName) const
	{
		const FStoredOverride* Found = Overrides.Find(GeomName);
		return Found != nullptr ? &Found->Appearance : nullptr;
	}

	int32 Num() const { return Overrides.Num(); }

	/** Re-drive every stored override onto the current live components. Called after
	 *  a render rebuild (render scene swap, PIE reindex). */
	void ReapplyAll();

private:
	/** One held override and the scope it was set with. */
	struct FStoredOverride
	{
		FMjGeomAppearance Appearance;
		FName Entity = NAME_None;
	};

	/** Walk the world's live geoms named `GeomName` in both render paths and drive
	 *  each one's material instance from `Override` (null restores the authored
	 *  appearance). Returns the count driven. */
	int32 ApplyToWorld(FName GeomName, FName Entity, const FMjGeomAppearance* Override);

	/** A texture-binding key to a live UTexture: a UE asset object path, else an
	 *  uploaded content-cache blob decoded to a transient texture. Null leaves the
	 *  slot at the base pass's texture. Resolved textures are cached by key. */
	UTexture* ResolveTexture(FName Key);

	/** The world the apply walk scans: the manager's, else this object's own. */
	UWorld* ResolveWorld() const;

	TWeakObjectPtr<AAMjManager> Manager;

	/** Set by InitForWorld when there is no manager to take a world from. */
	TWeakObjectPtr<UWorld> BoundWorld;

	TMap<FName, FStoredOverride> Overrides;

	/** Textures resolved from binding keys, kept alive and reused across applies. */
	UPROPERTY(Transient)
	TMap<FName, TObjectPtr<UTexture>> TextureCache;
};
