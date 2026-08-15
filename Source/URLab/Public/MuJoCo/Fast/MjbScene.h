// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "HAL/CriticalSection.h"
#include <atomic>
#include "MjbScene.generated.h"

struct mjModel_;
struct mjData_;
struct FMjRenderSnapshot;
class UPrimitiveComponent;
class UProceduralMeshComponent;
class UMaterialInterface;
class FRunnable;
class FRunnableThread;
class AAMjManager;

/** How a play-session AMjbScene sources its transforms. */
UENUM(BlueprintType)
enum class EMjbRunMode : uint8
{
	/** Mirror an owner's transform stream over ZMQ. Runs no physics here. */
	Puppet,
	/** Step this scene's own model in-process through the shared
	 *  UMjPhysicsEngine (driven by the RPC layer or free-running) and render the
	 *  stepped state. Makes the fast-path instance a full sim, not just a mirror. */
	Direct,
};

/**
 * @class AMjbScene
 * @brief Fast-path render scene built straight from a compiled MJB.
 *
 * Loads an MJB with mj_loadModel (binary deserialize, no MJCF/ProtoSpec/
 * Blueprint) and builds ONE lightweight actor per MuJoCo body (so the renderer
 * culls per body) carrying per-geom mesh components. It runs in one of two
 * RunModes:
 *
 * - Puppet (default): the scene runs NO physics. An external owner (a puppet
 *   client, or another UE instance) resolves transforms and streams them over
 *   ZMQ; the scene mirrors that per-body/per-geom transform stream.
 * - Direct: the scene installs its own raw mjModel/mjData into the shared
 *   UMjPhysicsEngine and renders the stepped state from the engine's thread-safe
 *   snapshot, so the fast-path instance is a full sim a client can drive by RPC.
 *
 * A one-shot mj_forward runs at load to place the rest pose. bTestSweep is an
 * owner-less dev fallback that animates joints locally so the builder can be
 * exercised without an owner; it is off by default and ignored in Direct mode
 * or once a bus is connected.
 */
UCLASS()
class URLAB_API AMjbScene : public AActor
{
	GENERATED_BODY()

public:
	AMjbScene();

	/** Absolute path to a version-matched MJB. Used only when MjbBytes is empty. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Fast")
	FString MjbFilePath;

	/** In-memory MJB received over the wire from an owner (no shared file). When
	 *  non-empty this is loaded in preference to MjbFilePath. A UPROPERTY so it
	 *  survives the editor->PIE duplication, letting the PIE copy rebuild without
	 *  a file. */
	UPROPERTY()
	TArray<uint8> MjbBytes;

	/** Puppet (mirror an owner) or Direct (step this scene's own model through
	 *  the shared UMjPhysicsEngine and render it). Direct makes the fast-path
	 *  instance a full sim a Python client can drive over the existing RPC. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Fast")
	EMjbRunMode RunMode = EMjbRunMode::Puppet;

	/** Animate joints locally via mj_forward so the scene moves with no owner.
	 *  Development only -- the real path applies a streamed transform set, and
	 *  Direct mode steps for real. Off by default so a placed/owner-less scene
	 *  stays static; the launcher enables it explicitly for the no-owner demo.
	 *  Ignored once a bus endpoint is connected or in Direct mode. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Fast")
	bool bTestSweep = false;

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

	/** Build geometry only, at the MJB rest pose, with NO bus and NO streaming.
	 *  For the editor-world preview: a persistent, static, saveable scene that is
	 *  never animated outside a play session. Returns geom count or -1. */
	int32 BuildStaticPreview();

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
	 * Fetch an MJB and its transform-bus endpoint from an owner over a ZMQ
	 * REQ/REP control channel. Sends a msgpack `{op:"fastpath_hello"}` and reads
	 * back the owner's `mjb` bytes plus `bus` endpoint. Synchronous with a short
	 * timeout; safe to call from the editor or a headless driver. Returns false
	 * with OutError on any failure.
	 */
	static bool FetchModelFromOwner(const FString& ControlEndpoint,
		TArray<uint8>& OutMjb, FString& OutBusEndpoint, FString& OutError);

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

	/** Worker-thread entry: receive per-geom transform frames off the bus.
	 *  Public so the bus FRunnable can drive it. */
	void RunBusLoop();
	/** Ask the bus worker to stop (called from the FRunnable's Stop). */
	void SignalBusStop() { bBusStop = true; }

	/** Connect the transform bus now (BusEndpoint must be set). Normally driven
	 *  by BeginPlay; exposed for tests and headless drivers. */
	void ConnectBus() { StartBus(); }
	/** Begin Direct stepping now (RunMode must be Direct). Normally driven by
	 *  BeginPlay; exposed for the -game launcher, which builds after BeginPlay. */
	void StartDirect() { BeginDirect(); }
	/** True once at least one transform frame has been received off the bus. */
	bool HasReceivedFrame() const { return bEverReceived.load(std::memory_order_acquire); }

	/** Body actors created (one per MuJoCo body, world included). */
	int32 NumBodyActors() const { return BodyActors.Num(); }
	/** Geom render components actually built (skips hidden/mesh/unsupported). */
	int32 NumBuiltGeoms() const;

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void BeginDestroy() override;
	virtual void Tick(float DeltaSeconds) override;

private:
	mjModel_* Model = nullptr;
	mjData_* Data = nullptr;
	// Rooted so GC can't collect the loaded master material between load and the
	// (possibly much later, on the PIE-reuse path) creation of its MIDs.
	UPROPERTY(Transient)
	TObjectPtr<UMaterialInterface> Master = nullptr;

	// A content id for the loaded MJB (a hash of its bytes). Cached, persistent
	// assets live under /Game/URLabFastPath/<ContentHash>/, so an identical model
	// reuses one asset set across sessions and a saved level reloads its geometry.
	FString ContentHash;

	// Textures built from the MJB's tex_data, keyed by MuJoCo texture id, so a
	// texture shared across materials/geoms is built once. Transient; rebuilt on
	// each LoadAndBuild.
	UPROPERTY(Transient)
	TMap<int32, TObjectPtr<class UTexture2D>> TextureCache;

	// Static meshes built from the MJB's mesh pool, keyed by MuJoCo mesh id, so a
	// mesh shared across geoms is built once and every geom references the same
	// asset by pointer -- which keeps the PIE world duplication cheap (no embedded
	// vertex data copied per geom, unlike a ProceduralMeshComponent). Transient.
	UPROPERTY(Transient)
	TMap<int32, TObjectPtr<class UStaticMesh>> StaticMeshCache;

	// Cameras built from the MJB, indexed by MuJoCo camera id. Empty unless
	// bEnableCameraStreaming.
	UPROPERTY(Transient)
	TArray<TObjectPtr<class UMjCamera>> CameraComps;

	// Indexed by MuJoCo body id / geom id.
	UPROPERTY(Transient)
	TArray<TObjectPtr<AActor>> BodyActors;
	UPROPERTY(Transient)
	TArray<TObjectPtr<UPrimitiveComponent>> GeomComps;

	double SweepTime = 0.0;

	// --- transform bus (owner -> this renderer) --------------------------- //
	void* ZmqCtx = nullptr;
	void* ZmqSub = nullptr;
	FRunnable* BusRunnable = nullptr;
	FRunnableThread* BusThread = nullptr;
	std::atomic<bool> bBusStop{false};
	// The worker only copies the newest raw payload into this preallocated buffer
	// (no UE allocation / no msgpack decode off the game thread). The game thread
	// decodes + applies it in Tick.
	FCriticalSection FrameMutex;
	uint8* RxBuf = nullptr;
	int32 RxCap = 0;
	int32 RxSize = 0;
	bool bRxPending = false; // guarded by FrameMutex
	std::atomic<bool> bEverReceived{false};

	void StartBus();
	void StopBus();

	// --- Direct mode (in-process stepping via the shared engine) ---------- //
	// The manager whose UMjPhysicsEngine steps our raw model. Get-or-spawned at
	// BeginPlay; not owned here (weak).
	TWeakObjectPtr<AAMjManager> DirectManager;
	// Direct mode: the geometry-less shadow articulation that lets the RPC layer /
	// a Python client drive the raw model (built after install, retired at teardown).
	TWeakObjectPtr<class AMjArticulation> ShadowArt;
	// Frame id of the last render snapshot applied, so Tick skips unchanged frames.
	uint64 LastRenderFrameId = 0;
	// One-shot: log the first non-finite snapshot transform (diverged physics vs
	// bad snapshot) without flooding.
	bool bDirectNanLogged = false;
	// Polls until the manager has begun play, then installs the raw model.
	FTimerHandle DirectInstallTimer;

	// Get-or-spawn the manager and arm the deferred install.
	void BeginDirect();
	// Install this scene's raw model+data into the manager's engine and start the
	// physics worker. Retried off DirectInstallTimer until the manager has begun play.
	void InstallIntoEngine();
	// Render the geoms + cameras from the engine's published render snapshot
	// (thread-safe; the worker steps our mjData on another thread).
	void ApplyFromSnapshot();

	// Build (or fetch from cache) a UTexture2D from the MJB's tex_data for the
	// given MuJoCo texture id. bSRGB selects colour vs linear sampling. Null on a
	// bad id.
	class UTexture2D* GetOrBuildTexture(int32 TexId, bool bSRGB);

	// Load the mjModel/mjData (from MjbBytes or MjbFilePath) and the master
	// material, without building any actors. No-op if already loaded. False on
	// failure.
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
	// Place cameras from their world pose. Uses the streamed cam transforms when
	// present, else this process's mjData rest pose.
	void ApplyCameraPoses(const double* Cxpos, const double* Cxquat);
	// Editor route: a shared UStaticMesh keyed by mesh id (cheap PIE duplication);
	// BuildFromMeshDescriptions is editor-only. Null on a bad id.
	class UStaticMesh* GetOrBuildStaticMesh(int32 MeshId);
	// Packaged-game route: a ProceduralMeshComponent that builds render data at
	// runtime (no editor mesh-build modules).
	class UProceduralMeshComponent* BuildMesh(int32 GeomId, AActor* Body);
	void ApplyGeomMaterial(UPrimitiveComponent* Comp, int32 GeomId);

	// --- small model-query helpers (shared by the build + apply paths) ----- //
	// True if geom G's group is in VisibleGroupMask (visual groups shown, collision
	// proxies hidden). Reads Model->geom_group; caller must hold a valid Model.
	bool IsGeomVisible(int32 GeomId) const;
	// The rgba a geom draws with: its material's when it has one, else its own.
	// Returns a pointer to 4 floats in the Model; caller must hold a valid Model.
	const float* GeomRgba(int32 GeomId) const;

	void Teardown();
};
