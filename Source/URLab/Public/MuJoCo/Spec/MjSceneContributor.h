// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Things that are not articulations but still put geometry in the scene.
//
// A level's MuJoCo content is not only imported robots. A heightfield actor
// samples the terrain under it; a quick-convert component turns whatever meshes
// an actor already carries into collision geometry. Neither reaches into the
// compiler to add elements to a spec while it is being built -- that would be
// a second authoring path, which is exactly what the design below avoids.
//
// They are the same thing as an import, run the other way round: MJCF authored
// from Unreal content. So they author a spec, exactly the spec an import
// of the same content would have produced, and the scene takes it as an ordinary
// participant -- one `<asset><model/>` plus `<attach prefix=.../>` pair, the same
// as every articulation. Nothing in the writer, the VFS or the binding learns
// that a participant came from Unreal geometry rather than from a file.
//
// That choice carries three properties for free, each of which would otherwise
// have had to be re-derived here: participants are ordered by prefix, so the
// cross-participant qpos layout is pinned; every asset is mounted under a
// participant-prefixed basename, so MuJoCo's case-insensitive basename fallback
// in the VFS cannot hand one contributor another's mesh; and elements bind by
// name under the same prefix, so a converted body's compiled id is recovered the
// same way a robot link's is.

#include "CoreMinimal.h"
#include "UObject/Interface.h"

#include "MuJoCo/Spec/MjSpecRef.h"

#include "MjSceneContributor.generated.h"

class UMjNodeComponent;

UINTERFACE(MinimalAPI)
class UMjSceneContributor : public UInterface
{
	GENERATED_BODY()
};

/**
 * A level object that contributes a spec to the compiled scene.
 *
 * Implemented by actors (a heightfield samples a region of the level) and by
 * components (quick convert converts the actor it sits on). The engine finds
 * both by walking the world once, so adding a third kind of contributor costs
 * an implementation and nothing else.
 */
class URLAB_API IMjSceneContributor
{
	GENERATED_BODY()

public:
	/**
	 * (Re)author this contributor's spec from the Unreal content behind it.
	 *
	 * Runs immediately before every compile, on the game thread, and is expected
	 * to be idempotent: authoring twice must leave one spec, not two. What it
	 * may cost is up to the contributor -- sampling a terrain is expensive enough
	 * to be cached on disk, converting a mesh expensive enough to be hashed --
	 * but a compile is always allowed to ask for it again.
	 */
	virtual void AuthorSceneSpec() = 0;

	/** The spec to attach into the scene. Invalid means "contribute nothing". */
	virtual FSpecRef GetSceneSpec() const = 0;

	/**
	 * The namespace every element of this contributor compiles under.
	 *
	 * Must be unique across the level: it names the participant's `<model>` asset
	 * and prefixes every compiled name and every mounted asset basename.
	 */
	virtual FString GetScenePrefix() const = 0;

	/**
	 * Where the contributed spec sits, as an Unreal world transform.
	 *
	 * Identity means the spec already carries world coordinates, which is
	 * what a contributor that sampled the level in world space wants: folding its
	 * own actor transform in a second time would move what it measured.
	 */
	virtual FTransform GetScenePlacement() const { return FTransform::Identity; }

	/** Called after a compile has told every element the id it received. */
	virtual void OnSceneBound() {}
};

/**
 * Destroy everything under `Root`, leaving `Root` itself: the elements the last
 * authoring pass created and the preview components they made for themselves.
 *
 * Re-authoring has to remove what the last pass built, and detaching is not
 * enough: a detached element is still a component of its actor, still parentless,
 * and still carries children, which is precisely the shape `FSpecRef::OverActor`
 * looks for when it decides which node is the spec root. So the old subtree
 * is destroyed rather than orphaned, previews included -- a geom's preview meshes
 * are static mesh components on the actor, and an actor whose meshes are what get
 * converted must not accumulate them.
 */
URLAB_API void MjDestroySpecChildren(UMjNodeComponent& Root);
