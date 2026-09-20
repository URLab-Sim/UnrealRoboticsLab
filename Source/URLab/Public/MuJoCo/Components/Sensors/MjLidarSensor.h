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

#include "CoreMinimal.h"
#include "Components/LidarComponent.h"
#include <mujoco/mujoco.h>
#include <atomic>
#include "MjLidarSensor.generated.h"

/**
 * @class UMjLidarSensor
 * @brief MuJoCo adapter over the engine-agnostic ULidarComponent core: the
 * ray source is the compiled MuJoCo model.
 *
 * The core (ULidarComponent) owns every engine-agnostic part of the lidar —
 * beam grid, noise, blind zone, target aggregation, publishing — and works
 * standalone against the Unreal collision world. This adapter replaces the
 * ray source with the simulation's ground truth and keeps all MuJoCo
 * knowledge here:
 *
 *  - Rays: mj_multiRay against the compiled model, run inside the physics
 *    engine's post-step callback (physics thread, CallbackMutex held), so
 *    the lidar sees exactly what MuJoCo sees — quick-converted props,
 *    imported robots' collision geoms, the floor. A plain UE actor with no
 *    MuJoCo presence is invisible to it, by design.
 *  - Pose: the nearest UMjBody or UMjSite ancestor is the mount; the sensor
 *    frame is composed from the mount's MuJoCo pose and this component's
 *    mount-relative transform, read from mjData on the physics thread. With
 *    no MuJoCo ancestor the sensor falls back to a static world mount using
 *    the component's world pose captured at resolve time.
 *  - Scheduling: scans are rate-limited on MuJoCo simulation time (d->time)
 *    inside the callback, so ScanFrequencyHz honours pauses and sim-speed
 *    changes for free.
 *
 * Data path follows docs/concepts/architecture.md: the physics thread writes
 * each raw scan into a mutex-guarded buffer (the game thread never touches
 * mjData); TickComponent drains the newest scan, converts the sensor pose to
 * Unreal coordinates, and hands the beam results to the core's shared
 * PublishRayResults, which builds points, aggregates targets, and broadcasts
 * OnLidarScan on the game thread.
 *
 * Beam convention (sensor-local frame): azimuth sweeps around the sensor's
 * local +Z (up), elevation is positive above the local X-XY plane, and
 * azimuth/elevation of 0/0 points along the sensor's local +X.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjLidarSensor : public ULidarComponent
{
	GENERATED_BODY()

public:
	UMjLidarSensor();

	// --- Geometry (MuJoCo ray source) ---

	/**
	 * Bitmask over MuJoCo geom groups 0..5 (bit g enables group g; bit 0 has
	 * value 1, bit 5 has value 32). URLab convention: group 3 = collision-only
	 * geoms, group 2 = visual-only meshes; quick-converted simple primitives,
	 * MjPlane floors and MJCF-imported geoms without a group override default
	 * to group 0. Default 9 = groups 0 + 3: senses collision hulls plus
	 * default-group geometry (floor, primitives) while excluding visual-only
	 * group 2, so a mesh with both visual and collision geoms is never
	 * double-hit. PIE-verified: the group 0 floor becomes visible exactly
	 * when bit 0 is set.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Geometry", meta = (ClampMin = "0", ClampMax = "63"))
	int32 GeomGroupMask = 9;

	/** Include static (worldbody) geoms such as the floor. Leave enabled for environment sensing. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Geometry")
	bool bHitStaticGeoms = true;

	/** Exclude the mount body's own geoms from raycasts (avoids self-hits at the origin). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Geometry")
	bool bExcludeMountBody = true;

protected:
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	virtual bool HasExternalScanSource() const override { return true; }

private:
	// ---- Mount (game thread resolves and publishes; physics thread reads;
	// guarded by UMjPhysicsEngine::CallbackMutex, which the post-step
	// callback always holds and the game thread takes briefly while
	// publishing) ----
	struct FMountState
	{
		int32 Type = 0;      // 0 = none/unresolved, 1 = body, 2 = site
		int32 Id = -1;       // body id or site id in the compiled model
		double RelPosMj[3] = { 0, 0, 0 };  // sensor position in mount frame, MuJoCo metres
		double RelQuatMj[4] = { 1, 0, 0, 0 }; // sensor rotation in mount frame, MuJoCo (w,x,y,z)
	};
	FMountState m_Mount;

	/** Mount component the published ids were resolved from (poll reference). */
	TWeakObjectPtr<USceneComponent> m_MountComponent;
	bool m_bMountResolved = false;

	/** Last polled mount id; republishes only when it changes (or on first resolve). */
	int32 m_LastPolledCandidate = -2;

	// Set in EndPlay so the callback stops scanning even while the object is
	// still alive; also guards against destruction without EndPlay when read
	// together with the weak pointer.
	std::atomic<bool> m_bStopped{ false };

	// ---- Scan handoff (physics writes, game thread copies; m_DataMutex) ----
	struct FRawScan
	{
		double OriginMj[3] = { 0, 0, 0 };   // sensor origin, MuJoCo metres
		double SensorQuatMj[4] = { 1, 0, 0, 0 }; // sensor rotation, MuJoCo (w,x,y,z)
		TArray<float> Azimuths;   // AzN entries
		TArray<float> Elevations; // ElN entries
		TArray<double> Ranges;    // NumRays entries, -1 = miss
		TArray<int32> GeomIds;    // NumRays entries, -1 = miss
		double SimTime = 0.0;
		int64 ScanId = -1;
		int32 AzCount = 0;
	};
	FCriticalSection m_DataMutex;
	FRawScan m_SharedScan;
	FRawScan m_LocalScan;
	TArray<float> m_LocalRangesM;    // float view of m_LocalScan.Ranges for the core sink
	TArray<float> m_LocalAzimuthsUe; // azimuths mirrored into UE handedness for the core sink
	int64 m_LastConsumedScanId = -1;

	// ---- Physics-thread-only state (no locking needed) ----
	int64 m_PhysicsScanCounter = 0;
	double m_NextScanSimTime = 0.0;
	TArray<float> m_ScratchAz;
	TArray<float> m_ScratchEl;
	TArray<double> m_ScratchVec;   // 3 * NumRays
	TArray<double> m_ScratchDist;  // NumRays
	TArray<int32> m_ScratchGeomId; // NumRays
	FRandomStream m_NoiseStream;

	// ---- Game-thread-only state ----
	bool m_bCallbackRegistered = false;

	// ---- Internals ----

	/**
	 * Resolves the mount body/site id and relative pose and (re)publishes
	 * under CallbackMutex. Re-polled every tick: a recompile rebinds the
	 * mount component's id, and the next tick republishes automatically.
	 */
	void ResolveMount();

	/** Runs inside the physics engine's post-step callback (CallbackMutex held). */
	void PhysicsPostStep(mjModel* m, mjData* d);

	/** Copies out the newest scan and feeds the core's shared sink (game thread). */
	void DrainScan();
};
