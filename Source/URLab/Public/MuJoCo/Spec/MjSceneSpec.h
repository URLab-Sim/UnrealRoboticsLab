// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Composing several specs into one compiled scene.
//
// A scene is a root spec plus placed participants. Each participant is built
// on its own and then attached, so what a participant authored and where the
// scene puts it stay separate questions.
//
// The compiled scene keeps every spec it was composed from alive, not just the
// composed one: the composition may reference the participants, so releasing
// them early would leave the scene spec pointing at freed memory.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Spec/MjSpecBuild.h"

struct mjModel_;
typedef struct mjModel_ mjModel;

class UMjNodeComponent;

namespace urlab::spec
{

/** One articulation placed in the scene. */
struct URLAB_API FMjSceneSpecParticipant
{
	FSpecRef Spec;

	/** Non-empty and unique per participant, in identifier-plus-underscore form. */
	FString Prefix;

	/** MuJoCo frame, metres. */
	FVector MjPos = FVector::ZeroVector;

	/** MJCF component order is handled at the call site, not here. */
	FQuat MjQuat = FQuat::Identity;
};

/**
 * One asset the compile was given, under the name the specs reference it by.
 *
 * Kept past the compile because the debug artefact has to write the same bytes
 * out beside the XML: the VFS is gone by then, and the paths the bytes were
 * read from cannot be recovered from a spec whose references have been
 * namespaced.
 */
struct URLAB_API FMjSceneAsset
{
	FString Name;
	TArray<uint8> Bytes;
};

/**
 * The compiled scene: the model, the specs behind it, and the binding.
 *
 * Owns the model and every spec. Destruction deletes the model first, then the
 * specs, because the model must not outlive the specs it was compiled from.
 * Move-only for the same reason FMjBuiltSpec is.
 *
 * The participant specs are retained rather than dropped after composition:
 * the composed scene spec may reference them, so their lifetime is the
 * scene's. BoundIds is the engine-facing identity, and it carries ids rather
 * than element pointers so that nothing outside this object holds a pointer
 * into a spec.
 */
struct URLAB_API FMjCompiledScene
{
	FMjCompiledScene() = default;
	FMjCompiledScene(FMjCompiledScene&& Other);
	FMjCompiledScene& operator=(FMjCompiledScene&& Other);
	~FMjCompiledScene();
	FMjCompiledScene(const FMjCompiledScene&) = delete;
	FMjCompiledScene& operator=(const FMjCompiledScene&) = delete;

	mjModel* Model = nullptr;
	FMjBuiltSpec Scene;
	TArray<FMjBuiltSpec> Participants;
	TMap<TObjectPtr<const UMjNodeComponent>, int32> BoundIds;
	TArray<FMjSceneAsset> Assets;
	TArray<FMjSpecDiagnostic> Errors;
	TArray<FMjSpecDiagnostic> Warnings;

	bool IsValid() const { return Model != nullptr; }

	/**
	 * Write the compiled scene MJCF and its assets under Dir.
	 *
	 * The artefact is for a human reading what was actually compiled; it is
	 * never read back by the product.
	 */
	bool SaveDebugArtifacts(const FString& Dir, TArray<FMjSpecDiagnostic>& OutDiags) const;

private:
	/** Model then specs, shared by the destructor and move assignment. */
	void Release();
};

/**
 * Assembles a scene from a root spec and its participants.
 *
 * The builder holds only handles; nothing is built until Compile runs, and the
 * result owns everything the compile produced.
 */
class URLAB_API FMjSceneSpecBuilder
{
public:
	void SetSceneRoot(const FSpecRef& Root);
	void AddParticipant(const FMjSceneSpecParticipant& Participant);

	/** Build each spec, compose, mount assets, compile, and bind. */
	FMjCompiledScene Compile();

private:
	FSpecRef SceneRoot;
	TArray<FMjSceneSpecParticipant> Participants;
};

}  // namespace urlab::spec

#endif  // URLAB_MJ_GEN
