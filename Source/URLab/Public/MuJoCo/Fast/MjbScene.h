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
class UPrimitiveComponent;
class UProceduralMeshComponent;
class UMaterialInterface;
class FRunnable;
class FRunnableThread;

/**
 * @class AMjbScene
 * @brief Fast-path render scene built straight from a compiled MJB.
 *
 * Loads an MJB with mj_loadModel (binary deserialize, no MJCF/ProtoSpec/
 * Blueprint), builds ONE lightweight actor per MuJoCo body (so the renderer
 * culls per body) carrying per-geom mesh components, and drives them from a
 * per-geom world-transform stream. It runs NO physics: the owner (a puppet
 * client or a live/direct UE instance) resolves transforms and streams them.
 *
 * A one-shot mj_forward runs only at load to place the rest pose; the optional
 * bTestSweep animates joints locally (the owner-less "fallback" path) purely so
 * the builder can be exercised without an external owner.
 */
UCLASS()
class URLAB_API AMjbScene : public AActor
{
	GENERATED_BODY()

public:
	AMjbScene();

	/** Absolute path to a version-matched MJB. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Fast")
	FString MjbFilePath;

	/** Animate joints locally via mj_forward so the scene moves with no owner.
	 *  Development only -- the real path applies a streamed transform set.
	 *  Ignored once a bus endpoint is connected. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Fast")
	bool bTestSweep = true;

	/** Owner transform bus endpoint, e.g. "tcp://127.0.0.1:5561". When set, this
	 *  scene subscribes to a per-geom transform stream and mirrors the owner. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Fast")
	FString BusEndpoint;

	/** Bitmask of MuJoCo geom groups to render (bit i = group i). Default shows
	 *  groups 0-2 (visual) and hides 3+ (collision proxies), matching the common
	 *  menagerie visual/collision split. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Fast")
	int32 VisibleGroupMask = 0b0000111;

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

	/** Apply a per-geom world-transform stream: xpos is 3*ngeom, xquat 4*ngeom
	 *  (wxyz), in MuJoCo world frame. This is the render-time hot path (no
	 *  physics) — the wire carries quaternions. */
	void ApplyGeomTransforms(const double* Xpos, const double* Xquat);

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
	UMaterialInterface* Master = nullptr;

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

	void BuildBodies();
	void BuildGeoms();
	UPrimitiveComponent* BuildGeom(int32 GeomId);
	class UProceduralMeshComponent* BuildMesh(int32 GeomId, AActor* Body);
	void ApplyGeomMaterial(UPrimitiveComponent* Comp, int32 GeomId);
	void Teardown();
};
