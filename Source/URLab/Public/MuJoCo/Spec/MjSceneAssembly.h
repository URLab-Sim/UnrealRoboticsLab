// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// The scene as a spec assembled from participating specs.
//
// A level holds several articulations plus a manager carrying the scene-level
// sections. MJCF expresses exactly that with <asset><model/> and <attach
// prefix=...>, so the scene spec is written, not built: each participant
// goes into the compile VFS as its own MJCF, and the scene references them.
// MuJoCo's own attach then performs the keyframe remapping, the default-class
// namespacing and the asset prefixing, which is why this is a projection rather
// than a merge.
//
// Two properties are load-bearing and neither is incidental:
//
// ORDER. Attach-row order determines cross-articulation qpos layout, and
// GetAllActorsOfClass order is unpinned engine behaviour -- the same hazard class
// as sibling order. Participants are sorted by canonical prefix.
//
// ASSET NAMES. MuJoCo's VFS falls back to a case-insensitive basename match
// across every mount, so two participants that each reference their own
// `base.obj` silently share one mesh, with no warning on any version we have
// tested. Every asset is therefore mounted under a participant-prefixed
// basename; subdirectories do not help, because the fallback ignores them.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjGenHooks.h"

class AActor;

/** One spec taking part in a scene. */
struct URLAB_API FMjSceneParticipant
{
	FSpecRef Spec;

	/** The namespace every element of this participant compiles under. */
	FString Prefix;

	/** The participant's placement in the scene, in MuJoCo metres and quaternion. */
	FVector MjPos = FVector::ZeroVector;
	FQuat MjQuat = FQuat::Identity;
};

/** An asset mounted into the compile VFS. */
struct URLAB_API FMjVfsAsset
{
	/** The prefixed basename MuJoCo will match on. */
	FString Name;
	TArray<uint8> Bytes;
};


/** A section a participant authored that the scene's own section overrides. */
struct URLAB_API FMjSectionConflict
{
	FString Prefix;
	FString Section;
	FString Detail;
};

/**
 * A scene, assembled at write time from the level.
 *
 * Nothing is owned and nothing is cached: membership is a projection of the
 * actors present, so there is no second source of truth to reconcile against the
 * level. Participants are added, the scene root supplies the manager's sections,
 * and the result is MJCF text plus the assets it needs.
 */
struct URLAB_API FSceneAssembly
{
	/** Add a participant. Order of calls does not matter; prefix order decides. */
	void Add(const FSpecRef& Spec, const FString& Prefix, const FVector& MjPos = FVector::ZeroVector,
		const FQuat& MjQuat = FQuat::Identity);

	/**
	 * The spec supplying the scene's own sections: <option>, <compiler>,
	 * <size>, <statistic>, <visual>, and any world-body content.
	 */
	void SetSceneRoot(const FSpecRef& Spec);

	/** Participants, sorted by canonical prefix. */
	const TArray<FMjSceneParticipant>& GetParticipants() const;

	/**
	 * Every asset the scene needs, mounted under participant-prefixed basenames.
	 *
	 * The same collection backs the bridge handshake's ship-list, so what the
	 * compiler sees and what a remote client is told about cannot drift apart.
	 */
	TArray<FMjVfsAsset> CollectAssets() const;

	/**
	 * Every asset file the scene resolved, keyed by the name it is mounted under.
	 *
	 * The same pass as `CollectAssets` with the bytes left on disk, because a
	 * caller shipping a ship-list wants the names of the files and not their
	 * contents. Inline assets contribute nothing: they have no file.
	 *
	 * The key is `FMjVfsAsset::Name`, not the file's own name, and the two
	 * differ: a scene prefixes every `file=` so that two participants cannot
	 * collide in MuJoCo's flat VFS namespace. A caller that re-derives the name
	 * from the path gets one nothing resolves against.
	 */
	TMap<FString, FString> CollectAssetFiles() const;

	/**
	 * Participants whose own <option>, <size> or <compiler> differs from the
	 * scene's.
	 *
	 * MuJoCo discards an attached spec's option and size wholesale rather
	 * than merging them, and on the pinned engine it does so SILENTLY: the
	 * per-field diagnostic exists only in a later release. So the editor detects
	 * this rather than relying on the engine to report it. `<compiler angle>` is
	 * the exception -- it is honoured per child and round-trips, so it is not
	 * reported.
	 */
	TArray<FMjSectionConflict> FindSectionConflicts() const;

	/**
	 * The scene MJCF: the manager's sections, the world-body content, and one
	 * <asset><model/> plus <attach prefix=.../> pair per participant.
	 *
	 * Each participant's own MJCF goes into `OutParticipantXml` keyed by the VFS
	 * name the scene references it under.
	 */
	FString WriteSceneMjcf(TMap<FString, FString>& OutParticipantXml,
		TArray<FMjSpecDiagnostic>* OutErrors = nullptr) const;

private:
	void SortParticipants() const;

	mutable TArray<FMjSceneParticipant> Participants;
	mutable bool bSorted = true;
	FSpecRef SceneRoot;
};
