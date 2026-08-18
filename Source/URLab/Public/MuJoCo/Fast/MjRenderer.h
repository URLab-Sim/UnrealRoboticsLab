// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MuJoCo/Fast/MjRendererStepMode.h"
#include "MuJoCo/Fast/MjRendererOverlay.h"
#include "MuJoCo/Entity/MjPoseSource.h"
#include "MuJoCo/Entity/MjGeomAssetResolver.h"
#include "MuJoCo/Core/MjSimClock.h"
#include "Templates/UniquePtr.h"
#include <atomic>
#include "MjRenderer.generated.h"

struct mjModel_;
struct mjData_;
struct FMjGeomAppearance;
class UPrimitiveComponent;
class AAMjManager;
class AMjArticulation;
class UMjGeom;
class UMjQuickConvertComponent;
class UMjRendererAssetBaker;
class UMjRendererBus;
class UTexture;
class UMjCamera;
class UMaterialInterface;
class UMaterialInstanceDynamic;
class UMeshComponent;
class UStaticMeshComponent;
enum class EMjCameraMode : uint8;
enum class EMjDebugShaderMode : uint8;

/**
 * @class AMjRenderer
 * @brief Fast-path render scene built straight from a compiled MJB.
 *
 * Loads an MJB with mj_loadModel (binary deserialize, no MJCF/ProtoSpec/
 * Blueprint) and builds ONE lightweight actor per MuJoCo body (so the renderer
 * culls per body) carrying per-geom mesh components. Its RunMode is one of two
 * pose sources:
 *
 * - Mirror (default): the scene runs NO physics. An external owner (a puppet
 *   client, or another UE instance) resolves transforms and streams them over
 *   ZMQ; the scene mirrors that per-body/per-geom transform stream.
 * - Stepped: the scene installs its own raw mjModel/mjData into the shared
 *   UMjPhysicsEngine and renders the stepped state from the engine's thread-safe
 *   snapshot, so the fast-path instance is a full sim a client can drive by RPC.
 *
 * A one-shot mj_forward runs at load to place the rest pose.
 *
 * The mesh/texture/material builders live in UMjRendererAssetBaker, the transform-bus
 * receive plumbing in UMjRendererBus, and the Direct-mode engine install +
 * snapshot render in FMjRendererStepMode; the scene owns the model/data, the body ->
 * geom scene graph, and the build orchestration.
 */
UCLASS()
class URLAB_API AMjRenderer : public AActor, public IMjSimClock
{
	GENERATED_BODY()

public:
	AMjRenderer();

	// --- IMjSimClock: the manager-less render server is its own applied-state
	// source. In Mirror mode the owner's pose broadcast carries the frame_id /
	// sim_time it resolved; the render server stores them here so its cameras'
	// capture-stamping and delay-reveal work exactly as on the manager paths. ---
	virtual uint64 GetAppliedFrameId() const override
	{
		return AppliedFrameId.load(std::memory_order_acquire);
	}
	virtual double GetAppliedSimTime() const override
	{
		return AppliedSimTime.load(std::memory_order_acquire);
	}

	/** Record the applied post-step state carried by the latest pose broadcast.
	 *  Called on the game thread as transforms are applied. */
	void SetAppliedState(uint64 FrameId, double SimTime)
	{
		AppliedFrameId.store(FrameId, std::memory_order_release);
		AppliedSimTime.store(SimTime, std::memory_order_release);
	}

	/** Absolute path to a version-matched MJB. Used only when MjbBytes is empty. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Fast")
	FString MjbFilePath;

	/** In-memory MJB received over the wire from an owner (no shared file). When
	 *  non-empty this is loaded in preference to MjbFilePath. A UPROPERTY so it
	 *  survives the editor->PIE duplication, letting the PIE copy rebuild without
	 *  a file. */
	UPROPERTY()
	TArray<uint8> MjbBytes;

	/** Mirror (draw an owner's streamed transforms) or Stepped (step this scene's
	 *  own model through the shared UMjPhysicsEngine and render it). Stepped makes
	 *  the fast-path instance a full sim a Python client can drive over the
	 *  existing RPC. Only these two pose sources are meaningful for a fast-path
	 *  scene; any other value renders as Mirror. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Fast")
	EMjPoseSource RunMode = EMjPoseSource::Mirror;

	// Eval regime (-URLabFastForcedOnly): the forced-render control REP is the one
	// pose driver -- cameras are set up but do not auto-capture, and the transform
	// bus is never connected. When false (Mirror regime) the bus is the one driver
	// and the REP is not bound, so exactly one source ever writes the rendered pose.
	bool bForcedRenderOnly = false;

	/** Owner transform bus endpoint, e.g. "tcp://127.0.0.1:5561". When set, this
	 *  scene subscribes to a per-geom transform stream and mirrors the owner. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Fast")
	FString BusEndpoint;

	/** Owner control endpoint (REQ/REP), e.g. "tcp://host:5571". Set when the
	 *  scene was connected from a discovered owner; used to push perturbations
	 *  back to the owner. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Fast")
	FString OwnerControlEndpoint;

	/** Bitmask of MuJoCo geom groups to render (bit i = group i). Default shows
	 *  groups 0-2 (visual) and hides 3+ (collision proxies), matching the common
	 *  menagerie visual/collision split. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Fast")
	int32 VisibleGroupMask = 0b0000111;

	/** Spawn the MJB's cameras and stream their rendered frames over ZMQ/SHM, so
	 *  this renderer doubles as a render server. Off by default (capture is not
	 *  free); enable per scene or via -URLabFastCameras. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Fast")
	bool bEnableCameraStreaming = false;

	/** World offset for the whole scene (UE cm). Lets a Renderer drop the MJB
	 *  at a chosen spot in a curated base level instead of the world origin; added
	 *  to every geom / camera / copycat placement. Set from -URLabFastOrigin=X,Y,Z. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Fast")
	FVector SceneOrigin = FVector::ZeroVector;

	/** Base ZMQ port for camera streams; camera i binds CameraStreamBasePort + i. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Fast")
	int32 CameraStreamBasePort = 5600;

	/** Force a fresh build of the cached, content-hash-keyed assets, ignoring any
	 *  already on disk. Owner/listener override for "reimport this scene". */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Fast")
	bool bForceRebuildAssets = false;

	/** Cap the rendered camera height (px), downscaling the MJB resolution while
	 *  keeping aspect. Each camera is a full scene capture, so this keeps many
	 *  cameras affordable. 0 = honour the MJB resolution exactly. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Fast")
	int32 CameraMaxHeight = 480;

	/** Also publish camera frames over the shared-memory ring (co-located clients). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Fast")
	bool bEnableCameraShm = false;

	/** Build from MjbFilePath and, if BusEndpoint is set, connect the transform
	 *  bus. Callable from the editor / Python so a fast-path scene can be stood
	 *  up live in the editor world without PIE. */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "URLab|Fast")
	void Launch();

	/** Build the scene from MjbFilePath. Safe to call once. Returns geom count
	 *  built, or -1 on load failure. */
	int32 LoadAndBuild();

	/** Live model swap: retire the current model, shadow articulation and geometry
	 *  (reusing the same manager + engine) and rebuild from new MJB bytes, so a
	 *  running render-server viewer can switch scenes without a relaunch. The bytes
	 *  arrive over the wire (a Driver streams them to a remote Renderer). Game
	 *  thread only. */
	void ReloadFromBytes(const TArray<uint8>& NewMjb);

	/**
	 * Spawn a fast-path Renderer into World and return it. Sets the scene up
	 * from either MjbBytes (over the wire) or MjbFilePath, connects the transform
	 * bus (Puppet) or steps it in-process (bStepped), and unless bBaseLevel is set,
	 * spawns a movable light rig + a framing camera at the scene origin so the MJB
	 * is visible on a bare map. Shared by the -game command-line launcher and the
	 * runtime server browser so both build an identical Renderer. Null on failure.
	 */
	static AMjRenderer* SpawnRenderer(UWorld* World, const TArray<uint8>& MjbBytes,
		const FString& MjbFilePath, const FString& BusEndpoint, const FVector& Origin,
		bool bStepped, bool bBaseLevel, bool bCameras);

	/** Build geometry only, at the MJB rest pose, with NO bus and NO streaming.
	 *  For the editor-world preview: a persistent, static, saveable scene that is
	 *  never animated outside a play session. Returns geom count or -1. */
	int32 BuildStaticPreview();

	/**
	 * Build the lightweight play view of an already-compiled scene. The model is
	 * BORROWED from the shared engine -- this scene never loads, steps or frees it,
	 * runs no bus and no Direct mode, and is driven externally through
	 * ApplyBodyTransforms + ApplyCameraPoses from the engine's render snapshot. Each
	 * geom resolves its component through the imported-asset resolver (the UE assets a
	 * model import produced), keyed back to the authoring UMjGeom via the participants'
	 * element indices, with the baked resolver behind it for scene-root/inline
	 * geometry. QuickProps contributes the right-click-converted props' geoms to the
	 * same index, so a converted prop draws from its own StaticMesh rather than the
	 * baked hull. When bBuildCameras is set, the model's body-fixed cameras re-home onto
	 * this view (dormant, named through the camera registry) so the RPC camera surface
	 * enumerates them independently of the articulation actors. Returns geom count built.
	 */
	int32 BuildFromCompiledModel(mjModel_* InModel, const TArray<AMjArticulation*>& Participants,
		const TArray<UMjQuickConvertComponent*>& QuickProps, bool bBuildCameras = false);

	/** Flag this scene as the manager-driven compiled view before its BeginPlay runs,
	 *  so BeginPlay does not try to load an MJB of its own. */
	void MarkExternallyDriven() { bExternallyDriven = true; }

	/** The borrowed/loaded model this scene draws, or null before a build. */
	const mjModel_* GetModelPtr() const { return Model; }

	/** The render component built for a geom id, or null (hidden/instanced/unbuilt). */
	UPrimitiveComponent* GetGeomComponent(int32 GeomId) const;

	/** The authoring UMjGeom a compiled geom id came from, or null (scene-root/baked). */
	UMjGeom* GetGeomOrigin(int32 GeomId) const;

	/** Number of geom slots (== model ngeom once built). */
	int32 NumGeoms() const { return GeomComps.Num(); }

	/** Show or hide every built geom component (the play-time visuals toggle). */
	void SetGeomsVisible(bool bVisible);

	/** Place cameras from their world pose (streamed cam transforms wxyz, 3*ncam /
	 *  4*ncam). Null pointers fall back to this process's mjData rest pose. Driven by
	 *  the manager for the compiled play view each frame from the render snapshot. */
	void ApplyCameraPoses(const double* Cxpos, const double* Cxquat);

	/** Place cameras from the render snapshot's cam_xpos (3*ncam) + cam_xmat (9*ncam,
	 *  MuJoCo 3x3), converting each 3x3 to a wxyz quaternion the same way Direct mode
	 *  does before delegating to ApplyCameraPoses. The compiled-view driver on the
	 *  manager calls this each render tick. Null pointers are a no-op. */
	void ApplyCameraPosesFromMat(const double* CamXPos, const double* CamXMat);

	/** Number of camera components built for this view (== model ncam once built). */
	int32 NumCameras() const { return CameraComps.Num(); }

	/** The camera component for a view camera index, or null (out of range/unbuilt). */
	class UMjCamera* GetCamera(int32 Index) const
	{
		return CameraComps.IsValidIndex(Index) ? CameraComps[Index].Get() : nullptr;
	}

	/**
	 * Rebuild the body->actor and geom->component maps from the tags on the
	 * already-present child actors, instead of rebuilding geometry from the MJB.
	 * For a saved scene reopened in the editor: the persistent tagged actors are
	 * re-indexed so streaming reconnects to them. Loads the model (from MjbBytes
	 * or MjbFilePath) for sizing if it isn't already. Returns the number of geoms
	 * re-indexed, or -1 if there is no model / no tagged actors were found.
	 */
	UFUNCTION(BlueprintCallable, CallInEditor, Category = "URLab|Fast")
	int32 ReindexFromLevel();

	/** Set the in-memory MJB (received over the wire). Loaded in preference to
	 *  MjbFilePath on the next build. */
	void SetMjbBytes(const TArray<uint8>& Bytes) { MjbBytes = Bytes; }

	/**
	 * Push an external force+torque on a MuJoCo body back to the owner, which
	 * applies it via xfrc_applied on its next step. Force/Torque are in UE world
	 * space (converted to MuJoCo here). No-op if OwnerControlEndpoint is unset.
	 * This is the viewer -> owner perturbation path (e.g. a drag in the renderer).
	 */
	UFUNCTION(BlueprintCallable, Category = "URLab|Fast")
	void SendPerturbation(int32 BodyId, const FVector& ForceUE, const FVector& TorqueUE);

	/**
	 * Re-drive the live material instance of every geom named `GeomName` from a
	 * domain-randomization override, without touching the loaded model. A null
	 * `Override` restores the geom's baked appearance (the base material pass).
	 * `ResolveTexture` maps a binding key to a UTexture. Returns the count driven.
	 * This is the fast-path/Mirror half of the symmetric appearance channel.
	 */
	int32 ApplyAppearanceOverride(FName GeomName, const FMjGeomAppearance* Override,
		TFunctionRef<UTexture*(FName)> ResolveTexture);

	/** Number of built geom components carrying `GeomName` in the loaded model. */
	int32 NumGeomsNamed(FName GeomName) const;

	/** Apply a per-geom world-transform stream: xpos is 3*ngeom, xquat 4*ngeom
	 *  (wxyz), in MuJoCo world frame. This is the render-time hot path (no
	 *  physics) — the wire carries quaternions. */
	void ApplyGeomTransforms(const double* Xpos, const double* Xquat);

	/** Apply a per-BODY world-transform stream: Bxpos is 3*nbody, Bxquat 4*nbody
	 *  (wxyz), MuJoCo world frame. Each geom's world pose is composed from its
	 *  body's transform and its body-relative offset (from the model), so the wire
	 *  carries nbody transforms instead of ngeom — and mocap is covered for free.
	 *  The preferred stream; ApplyGeomTransforms stays for legacy per-geom owners. */
	void ApplyBodyTransforms(const double* Bxpos, const double* Bxquat);

	/** Apply straight from this process's mjData (geom_xpos + geom_xmat), used
	 *  by the initial rest pose and the owner-less dev sweep. Converts the 3x3
	 *  orientation to a quaternion. Not used on the streamed render path. */
	void ApplyFromData();

	/** Connect the transform bus now (BusEndpoint must be set). Normally driven
	 *  by BeginPlay; exposed for tests and headless drivers. */
	void ConnectBus() { StartBus(); }
	/** Begin Direct stepping now (RunMode must be Direct). Normally driven by
	 *  BeginPlay; exposed for the -game launcher, which builds after BeginPlay. */
	void StartDirect() { Direct.Begin(*this); }
	/** True once at least one transform frame has been received off the bus. */
	bool HasReceivedFrame() const;

	/** Body actors created (one per MuJoCo body, world included). */
	int32 NumBodyActors() const { return BodyActors.Num(); }

	/** Root scene component of the body actor for MuJoCo body id, or null when out of
	 *  range. The possess camera attaches here so it follows the streamed body transform. */
	USceneComponent* GetBodyRootComponent(int32 BodyId) const;
	/** Geom render components actually built (skips hidden/mesh/unsupported). */
	int32 NumBuiltGeoms() const;

	// --- Debug overlay material tint (owned by the renderer that owns the geoms) ---

	/** Parent material for the per-geom overlay MIDs, probed from engine content. */
	UPROPERTY(Transient)
	TObjectPtr<UMaterialInterface> OverlayParentMaterial = nullptr;

	/** Vector parameter name on OverlayParentMaterial that accepts the overlay colour. */
	FName OverlayColorParam = NAME_None;

	/** Load /Engine/BasicShapes/BasicShapeMaterial and record its first vector param
	 *  name, so the overlay MIDs and segmentation siblings have a tint parameter. */
	void InitializeOverlayMaterial();

	/** Swap per-geom overlay MIDs on this renderer's own geom components, coloured by
	 *  Mode from the physics-thread island seed / awake snapshot. The visualizer only
	 *  supplies mode + data; the renderer applies it to the geometry it owns. */
	void ApplyMaterialOverlay(EMjDebugShaderMode Mode, const TArray<int32>& BodyAwake,
		const TArray<int32>& BodyIslandSeed, bool bModulateBySleep,
		float SleepValueScale, float SleepSaturationScale);

	/** Restore the materials recorded at overlay apply time and clear the caches. */
	void ClearMaterialOverlay();

	// --- Per-camera segmentation pool (siblings of this renderer's geom components) ---
	//
	// Seg-mode UMjCameras share sibling mesh components rather than each maintaining
	// their own. Pools are lazy: built on first Acquire, destroyed on last Release.
	// Only two modes are poolable — InstanceSegmentation and SemanticSegmentation.

	/**
	 * Subscribe a camera to the sibling-mesh pool for Mode, building the pool on the
	 * first subscription and returning its sibling primitives for the camera's
	 * ShowOnly list. Mode must be Semantic- or InstanceSegmentation; other values are
	 * a no-op. Camera is tracked so Release can refcount correctly.
	 */
	void AcquireSegPool(EMjCameraMode Mode, UMjCamera* Camera,
		TArray<UPrimitiveComponent*>& OutSiblings);

	/** Unsubscribe a camera. When the last subscriber leaves, the pool is destroyed. */
	void ReleaseSegPool(EMjCameraMode Mode, UMjCamera* Camera);

	/** Snapshot of a currently-live sibling pool. Used for tests and for non-seg
	 *  cameras that hide siblings via HiddenComponents. */
	void GetSegPoolSiblings(EMjCameraMode Mode, TArray<UPrimitiveComponent*>& OutSiblings) const;

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void BeginDestroy() override;
	virtual void Tick(float DeltaSeconds) override;

private:
	// Applied post-step state (IMjSimClock) carried by the latest pose broadcast in
	// Mirror mode. Read by this actor's cameras for capture-stamping / delay reveal.
	std::atomic<uint64> AppliedFrameId{0};
	std::atomic<double> AppliedSimTime{0.0};

	// Forced-render control REP socket (opaque libzmq handles; see StartRenderControl).
	void* RenderControlCtx = nullptr;
	void* RenderControlRep = nullptr;
	int32 RenderControlPort = 0;
	// Applies the state carried by a forced-render request (poses + cameras + clock).
	void ApplyForcedRenderState(const TSharedPtr<class FJsonObject>& Req);
	// Services one forced-render request end to end: apply state, force-capture, pump
	// the readback to completion (shared MjPumpForcedCapture), and reply multipart.
	// Runs on a game-thread AsyncTask off the world tick -- inline in Tick starves the
	// render thread of the frames it polls for (measured ~55 ms vs ~11 ms off-tick).
	// TReqSeconds is the request-arrival timestamp, for the latency log.
	void ServeForcedRender(const TSharedPtr<class FJsonObject>& Req, double TReqSeconds);
	// One forced render is serviced at a time (REP is strict req/rep): a request is
	// applied + captured + replied on a game-thread AsyncTask (the same context the
	// manager path uses), so PollRenderControl must not take a new request while one
	// is in flight.
	std::atomic<bool> bForcedInFlight{false};

	// FMjRendererStepMode reaches back into the scene for the model, geom/camera
	// components and render origin while it drives the shared engine.
	friend struct FMjRendererStepMode;
	// FMjRendererOverlay reaches back into the scene for the shared overlay parent
	// material and the manager it walks while it tints this renderer's geoms.
	friend struct FMjRendererOverlay;

	// Compose a geom's compiled world pose (MuJoCo frame, wxyz) with the mesh-frame
	// correction its component needs, so a raw imported/converted StaticMesh lands
	// where the compiled geom is. A no-op for a geom that needs no correction.
	void CorrectMeshFrameWorld(int32 GeomId, double* WorldPos, double* WorldQuat) const;

	// --- Mirror interactive input (viewer -> owner perturbation) ---------- //
	// Drive a Ctrl+drag pull on a body from the local player's cursor and forward
	// it to the owner via SendPerturbation. Runs only for a Mirror role with an
	// owner control endpoint (this scene has no physics of its own to apply it
	// locally). No-op in every other role. Called from Tick.
	void ProcessMirrorPerturbationInput();

	// Pick the model body under a cursor ray. The fast-path render components carry no
	// physics collision, so this ray-tests the built geoms' world bounds directly
	// instead of a channel trace, and returns the nearest hit geom's body id (via
	// geom_bodyid) with the along-ray grab distance in OutDepthCm. Returns -1 (world
	// body excluded) when the ray hits no draggable body.
	int32 PickBodyIdAlongRay(const FVector& Origin, const FVector& Dir, float& OutDepthCm) const;

	// Active Mirror drag: the body being pulled (-1 = none) and the camera-space
	// depth of the grab point, so the pull target tracks the cursor ray at the
	// distance the body was grabbed. bMirrorDragActive gates the per-tick pull.
	bool bMirrorDragActive = false;
	int32 MirrorDragBodyId = -1;
	float MirrorDragDepthCm = 0.0f;

	mjModel_* Model = nullptr;
	mjData_* Data = nullptr;

	// False when the model is borrowed from the shared engine (the compiled play
	// view): Teardown then clears the pointers WITHOUT freeing them.
	bool bOwnsModel = true;

	// True for the compiled play view: BeginPlay does not touch MjbFilePath/bytes and
	// Tick never streams, because the manager builds and drives this scene directly.
	bool bExternallyDriven = false;

	// The instanced-statics collapse shares one baked UStaticMesh across repeated
	// world-body geoms; the compiled view resolves those geoms through imported assets
	// instead, so it builds them individually and turns the collapse off.
	bool bAllowInstancedStatics = true;

	// Per-geom component builder. When set (the compiled view's imported resolver) it
	// replaces the inline baked resolver BuildGeom uses by default.
	TUniquePtr<IMjGeomAssetResolver> GeomResolver;

	// Compiled geom id -> the authoring UMjGeom it bound to, for the debug overlays
	// and segmentation pools that need each geom's originating participant.
	TMap<int32, TWeakObjectPtr<UMjGeom>> GeomOrigins;

	// Builds + caches the meshes/textures/materials for the loaded MJB (content-hash
	// keyed). A UPROPERTY so its cached assets are GC-rooted through the scene.
	UPROPERTY(Transient)
	TObjectPtr<UMjRendererAssetBaker> AssetBaker;

	// Cameras built from the MJB, indexed by MuJoCo camera id. Empty unless
	// bEnableCameraStreaming.
	UPROPERTY(Transient)
	TArray<TObjectPtr<class UMjCamera>> CameraComps;

	// Indexed by MuJoCo body id / geom id.
	UPROPERTY(Transient)
	TArray<TObjectPtr<AActor>> BodyActors;
	UPROPERTY(Transient)
	TArray<TObjectPtr<UPrimitiveComponent>> GeomComps;

	// --- Debug overlay material tint state ---------------------------------- //
	// Records the original per-geom slot materials, the created tint MIDs, and applies
	// / restores the debug overlay on this renderer's geom components. A plain struct
	// owned here (its caches never GC-root -- the MIDs live on the mesh components).
	FMjRendererOverlay Overlay;

	// --- Per-camera segmentation pool --------------------------------------- //
	// Sibling-mesh pool for InstanceSegmentation-mode cameras. Empty when no subscribers.
	UPROPERTY(Transient)
	TArray<TObjectPtr<UStaticMeshComponent>> InstanceSegSiblings;
	// Sibling-mesh pool for SemanticSegmentation-mode cameras.
	UPROPERTY(Transient)
	TArray<TObjectPtr<UStaticMeshComponent>> SemanticSegSiblings;
	TSet<TWeakObjectPtr<UMjCamera>> InstanceSegSubscribers;
	TSet<TWeakObjectPtr<UMjCamera>> SemanticSegSubscribers;

	// Mutable ref to the pool / subscriber set matching Mode, or null for non-seg modes.
	TArray<TObjectPtr<UStaticMeshComponent>>* GetSegPoolArray(EMjCameraMode Mode);
	TSet<TWeakObjectPtr<UMjCamera>>* GetSegSubscribers(EMjCameraMode Mode);
	// Build Mode's pool by walking this renderer's geom components (plus the authoring
	// meshes + quick-convert props in the editor/test harness).
	void BuildSegPool(EMjCameraMode Mode);
	// Destroy all siblings in Mode's pool and clear it.
	void DestroySegPool(EMjCameraMode Mode);
	// Spawn one sibling mesh for a given original. Returns the new component (registered).
	UStaticMeshComponent* SpawnSegSibling(UStaticMeshComponent* Original,
		int32 BodyId, uint32 GroupHash, EMjCameraMode Mode);

	// Resolve the level's manager without spawning one; null in a bare test world with
	// no manager. Used by the overlay + segmentation walks that reach the articulations.
	AAMjManager* ResolveManager() const;

	// --- transform bus (owner -> this renderer) --------------------------- //
	// Receive plumbing for the owner's "geoms" broadcast. Lazily created on the
	// first StartBus; the game thread pulls the newest raw payload each Tick.
	UPROPERTY(Transient)
	TObjectPtr<UMjRendererBus> TransportBus;

	// Ensure + connect the bus (BusEndpoint must be set); drop it. Both no-op safe.
	void StartBus();
	void StopBus();

	// --- Direct mode (in-process stepping via the shared engine) ---------- //
	// Engine install + shadow articulation + snapshot render. A plain struct owned
	// here (weak actor ptrs + PODs, no GC roots).
	FMjRendererStepMode Direct;

	// Get-or-spawn the level's manager and cache it in Direct.Manager. A render
	// server needs a manager+bridge in BOTH modes: Direct steps through it, and a
	// Mirror Renderer still needs its RPC (fastpath_load scene swaps).
	AAMjManager* EnsureManager();
	// Timer target for the deferred Direct install: delegates to Direct. A UObject
	// method so FTimerManager can hold it by weak pointer.
	void InstallIntoEngine();

	// Load the mjModel/mjData (from MjbBytes or MjbFilePath) and prime the asset
	// baker, without building any actors. No-op if already loaded. False on failure.
	bool LoadModelOnly();

	void BuildBodies();
	void BuildGeoms();
	// Collapse repeated (mesh, material) STATIC world-body geoms into one instanced
	// component each (one draw call, still per-instance culled). Instances are set
	// once from the rest pose; the geom ids handled here are added to OutHandled so
	// BuildGeoms skips them. Editor path only (needs the shared UStaticMesh).
	void BuildInstancedStatics(TSet<int32>& OutHandled);
	UPrimitiveComponent* BuildGeom(int32 GeomId);

	// Spawn a dormant UMjCamera per MJB camera (built in both the editor preview and
	// a play session, so there is one representation). No-op unless
	// bEnableCameraStreaming.
	void BuildCameras();
	// Turn the dormant cameras into a live render server (render target + ZMQ/SHM
	// bind + per-frame capture). Play session only.
	void StartCameraStreaming();

	// --- Forced render control (REQ/REP): "render this exact state, block, return
	// the fresh frame" for the manager-less render server (its only path was async
	// PUB). A client sends the state + camera list; the server applies it on the
	// game thread, force-captures, tight-polls the readback, and replies multipart
	// (msgpack header + one raw pixel frame per camera). Bound while streaming. ---
	void StartRenderControl(int32 Port);
	void StopRenderControl();
	// Non-blocking: service at most one pending forced-render request. Game thread.
	void PollRenderControl();

	// Re-home the compiled model's body-fixed cameras onto this play view: one dormant
	// UMjCamera per model camera on its body actor, named through FMjCameraRegistry so
	// the canonical "<art>/<part>" identity matches the articulation path. No ZMQ bind
	// and no capture -- the RPC path enables streaming on demand. Registers each camera
	// with the camera subsystem + network manager explicitly, because a component added
	// after the world's BeginPlay may not receive its own.
	void BuildCompiledViewCameras();

	// Copycat: drive the game viewport's view camera from an owner's free/user
	// camera (MuJoCo world eye position + forward + up), so a Renderer mirrors
	// what the operator sees in MuJoCo's own viewer. Pos/Fwd/Up are MuJoCo-frame.
	void ApplyUserCamera(const double* Pos, const double* Fwd, const double* Up);
	// The view camera the copycat drives, cached once it is locked on so a live
	// scene swap (which can change the player's default view target) does not break
	// the mirroring. Not attached to this actor, so it survives a geometry rebuild.
	UPROPERTY()
	TObjectPtr<class ACameraActor> UserCam;

	// --- small model-query helpers (shared by the build + apply paths) ----- //
	// True if geom G's group is in VisibleGroupMask (visual groups shown, collision
	// proxies hidden). Reads Model->geom_group; caller must hold a valid Model.
	bool IsGeomVisible(int32 GeomId) const;
	// The rgba a geom draws with: its material's when it has one, else its own.
	// Returns a pointer to 4 floats in the Model; caller must hold a valid Model.
	const float* GeomRgba(int32 GeomId) const;

	void Teardown();
};
