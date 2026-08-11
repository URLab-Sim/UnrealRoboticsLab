// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Who takes part in the scene, under which prefix, at which pose.
//
// A level holds several articulations plus a manager carrying the scene-level
// sections. This is the projection of that level, and only the projection: it
// decides membership and order, and hands the result to whoever needs it --
// the scene builder to compose and compile, the scene writer to emit text.
//
// Two properties are load-bearing and neither is incidental:
//
// ORDER. Attach order determines cross-articulation qpos layout, and
// GetAllActorsOfClass order is unpinned engine behaviour -- the same hazard class
// as sibling order. Participants are sorted by canonical prefix.
//
// ASSET NAMES. MuJoCo's VFS falls back to a case-insensitive basename match
// across every mount, so two participants that each reference their own
// `base.obj` silently share one mesh, with no warning on any version we have
// tested. Every asset is therefore mounted under a participant-prefixed
// basename; subdirectories do not help, because the fallback ignores them. The
// ship-list below is keyed by those mounted names for the same reason.

#include "CoreMinimal.h"

#include "MuJoCo/Gen/MjEnums.gen.h"
#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjGenHooks.h"

class AActor;

/**
 * The name a composed scene carries, wherever it is composed.
 *
 * The scene root's authored `model` when it made one, because that is the only
 * name the level ever gave this composition; `scene` otherwise, because a
 * composed scene has no single source document and `scene` is what the debug
 * artefact and the clients already expect to find.
 *
 * Decided here rather than at each consumer. The compiled model and the MJCF
 * the handshake ships are the same scene, and a client holding both reconciles
 * one against the other -- so a second answer to this question is not a second
 * spelling, it is two scenes as far as that client can tell.
 */
URLAB_API FString MjSceneModelName(const FSpecRef& SceneRoot);

/** One spec taking part in a scene. */
struct URLAB_API FMjSceneParticipant
{
	FSpecRef Spec;

	/** The namespace every element of this participant compiles under. */
	FString Prefix;

	/** The participant's placement in the scene, in MuJoCo metres and quaternion. */
	FVector MjPos = FVector::ZeroVector;
	FQuat MjQuat = FQuat::Identity;

	/** This participant's attach conflict policy, or unset for the scene's. */
	TOptional<EMjConflict> Conflict;
};

/**
 * One participant's assets, under the names the scene mounts them by.
 *
 * The single place those names come from. The asset pass decides them, and it
 * decides them identically for the compile, for the ship-list and for the text
 * a remote client is handed, so the three cannot drift into three conventions
 * that only agree by inspection -- which is what a text writer re-deriving a
 * name from the reference's basename did, pointing two `base.obj` at one mount.
 *
 * The bytes stay on disk: a caller that wants them runs the pass with a sink
 * that takes them.
 */
URLAB_API TArray<FMjAssetRequest> MjCollectSceneAssets(const FMjSceneParticipant& Participant);

/**
 * A scene, assembled at write time from the level.
 *
 * Nothing is owned and nothing is cached: membership is a projection of the
 * actors present, so there is no second source of truth to reconcile against the
 * level. Participants are added, the scene root supplies the manager's sections,
 * and what comes out is the membership itself: who takes part, under which
 * prefix, at which pose. Turning that into a compiled model is the scene
 * builder's job and turning it into text is the scene writer's.
 */
struct URLAB_API FSceneAssembly
{
	/** Add a participant. Order of calls does not matter; prefix order decides. */
	void Add(const FSpecRef& Spec, const FString& Prefix, const FVector& MjPos = FVector::ZeroVector,
		const FQuat& MjQuat = FQuat::Identity, TOptional<EMjConflict> Conflict = {});

	/**
	 * The spec supplying the scene's own sections: <option>, <compiler>,
	 * <size>, <statistic>, <visual>, and any world-body content.
	 */
	void SetSceneRoot(const FSpecRef& Spec);

	/** Participants, sorted by canonical prefix. */
	const TArray<FMjSceneParticipant>& GetParticipants() const;

	/** The spec supplying the scene's own sections, or an invalid handle. */
	const FSpecRef& GetSceneRoot() const { return SceneRoot; }

	/**
	 * Every asset file the scene resolved, keyed by the name it is mounted under.
	 *
	 * The bytes are left on disk, because a caller shipping a ship-list wants
	 * the names of the files and not their contents. Inline assets contribute
	 * nothing: they have no file.
	 *
	 * The key is `FMjVfsAsset::Name`, not the file's own name, and the two
	 * differ: a scene prefixes every `file=` so that two participants cannot
	 * collide in MuJoCo's flat VFS namespace. A caller that re-derives the name
	 * from the path gets one nothing resolves against.
	 */
	TMap<FString, FString> CollectAssetFiles() const;

private:
	void SortParticipants() const;

	mutable TArray<FMjSceneParticipant> Participants;
	mutable bool bSorted = true;
	FSpecRef SceneRoot;
};
