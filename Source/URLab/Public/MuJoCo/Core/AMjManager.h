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

#include "MuJoCo/Convert/MjQuickConvertComponent.h"
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MuJoCo/Core/MjCapabilities.h"
#include "AMjManager.generated.h"

// Forward declarations
class AMjArticulation;
class AMjHeightfieldActor;
class URenderCameraComponent;
struct FSpecRef;

class UMjCompiler;
class UMjFlag;
class UMjModel;
class UMjOption;
class UMjBody;
class UMjAppearanceStore;

/**
 * @class AAMjManager
 * @brief Authoring scene root for compiling placed URLab content.
 *
 * In a game world it performs one owned compile/data handoff to the simulation
 * world subsystem and retires. It owns no runtime, worker, transport, render
 * view, model/data pointer, or runtime service registration.
 */
UCLASS()
class URLAB_API AAMjManager : public AActor
{
	GENERATED_BODY()

public:
	AAMjManager();

	/**
	 * The scene spec's root, and the manager's transform root.
	 *
	 * A level is one MuJoCo scene, so it is one MJCF spec; the sections
	 * below are its children, and every articulation in the level is attached
	 * into it at write time rather than being merged into it.
	 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Scene")
	TObjectPtr<UMjModel> SceneSpec;

	/**
	 * The scene's `<option>`, as the spec element it is.
	 *
	 * The scene is one MuJoCo spec assembled from the level, and its
	 * top-level sections belong to the manager rather than to any articulation:
	 * MuJoCo takes the scene's option block whole and discards an attached
	 * spec's own copy, so there is exactly one authority and this is it.
	 * Fields hold MJCF's values in MJCF's units.
	 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Scene")
	TObjectPtr<UMjOption> SceneOption;

	/** The scene's `<option><flag>`. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Scene")
	TObjectPtr<UMjFlag> SceneFlags;

	/**
	 * The scene's `<compiler>`, which carries the attach conflict policy.
	 *
	 * `merge` rather than MuJoCo's `warning` default, because the attach target
	 * is a scene nobody authored: under `warning` the parent keeps every field,
	 * so an imported model's `<option>` is discarded against defaults.
	 * Per-articulation `AttachConflict` overrides it.
	 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Scene")
	TObjectPtr<UMjCompiler> SceneCompiler;

	/** A handle on the scene spec the manager's sections belong to. */
	FSpecRef GetSceneSpec() const;

	/** Editor lookup for the authored scene root. */
	static AAMjManager* Instance;

	UFUNCTION(BlueprintPure, Category = "MuJoCo|Global", meta = (DisplayName = "Get MuJoCo Manager"))
	static AAMjManager* GetManager();

	/**
	 * Every camera the RPC surface can serve, in one place: the manager's global cameras
	 * plus the compiled render view's re-homed body-fixed cameras. When the render view
	 * carries cameras it owns the model cameras, so the transitional articulation-mounted
	 * cameras are skipped -- enumerating both would collide on the shared canonical name
	 * and double the GPU capture. Falls back to the articulation cameras only while the
	 * view has none. Works with the articulations alive or gone.
	 */
	void CollectCameras(TArray<URenderCameraComponent*>& Out) const;

	/**
	 * The visual appearance channel: name-keyed overrides for live materials. Lazily created on first
	 * request and owned here (so overrides outlive any one render scene and a
	 * render-server swap can re-apply them). Never null.
	 */
	UMjAppearanceStore* GetAppearanceStore();


	// --- Capabilities (composable, orthogonal to the pose source) ---

	/**
	 * @brief Whether this instance PRODUCES camera frames (its model cameras and/or its own view).
	 *
	 * An output: absent means the frames are never produced, so nothing downstream has to
	 * check again. It used to be the same word as the camera-RPC permission below, which is
	 * why the two were one CLI token setting two booleans in two functions that could
	 * disagree -- authorising the ops on an instance that built no cameras, or building
	 * cameras nobody was allowed to configure.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Capabilities")
	bool bStreamCameraFrames = true;

	/**
	 * @brief Whether peers may CALL the camera ops on this instance.
	 *
	 * A permission: the dispatcher refuses the ops from the op's declared requirement, so
	 * nothing here has to be re-checked inside a handler.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Capabilities")
	bool bAllowCameraRpc = true;

	/**
	 * @brief Whether this instance accepts interactive input (xfrc / wrench / drag) into its sim.
	 *
	 * A permission. When off, forwarded perturbation input is rejected. Advertised in the
	 * handshake.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Capabilities")
	bool bAcceptInput = true;

	/**
	 * @brief Whether peers may swap this instance's model out from under it.
	 *
	 * A permission, and the reason it exists: the model swap was ungated, so a discovery
	 * probe could replace a running scene. The resolved workflow determines the
	 * permission policy: a simulation-owned source refuses model replacement
	 * unless an authorized simulation operation requests it.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Capabilities")
	bool bAcceptModelLoad = true;

	/** True if this instance currently has the given capability enabled. */
	UFUNCTION(BlueprintPure, Category = "MuJoCo|Capabilities")
	bool HasCapability(EMjCapability Capability) const;

	// --- Remote Stepping ---

	/**
	 * @brief The episode seed, recorded here for anything that wants to mirror it.
	 *
	 * Nothing in this plugin consumes it. Modern mjOption has no `seed` field and
	 * mj_step is deterministic -- MuJoCo's randomness comes from noise inputs the
	 * client sets, not an integrator RNG -- so there is nowhere to write it. The
	 * Retained only as authoring metadata for Blueprint integrations.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Remote Stepping")
	int32 Seed = 0;

protected:
	/** Last authored compile/handoff diagnostic. It is not runtime state. */
	FString AuthoringCompileError;

	/** Name-keyed geom appearance overrides (visual DR). Lazily created; see
	 *  GetAppearanceStore. Transient so it is GC-rooted across PIE without
	 *  serialising the overrides. */
	UPROPERTY(Transient)
	TObjectPtr<UMjAppearanceStore> AppearanceStore;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:
	void Compile();

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Status")
	FString GetLastCompileError() const;

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Authoring")
	bool CompileModel();
};
