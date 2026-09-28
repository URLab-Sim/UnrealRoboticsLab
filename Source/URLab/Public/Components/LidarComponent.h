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
#include "Components/SceneComponent.h"
#include "Components/LidarTypes.h"
#include "Engine/EngineTypes.h"
#include "Math/RandomStream.h"
#include "LidarComponent.generated.h"

/**
 * @class ULidarComponent
 * @brief Engine-agnostic spinning-raycast lidar: works in plain Unreal, no
 * physics-engine coupling.
 *
 * The component owns the lidar computation and, on its own, never reads
 * physics data from any source: every scan takes its cartesian pose from the
 * component's own world transform and raycasts the Unreal collision world
 * (UWorld::LineTraceSingleByChannel). Attach it under any actor or component
 * hierarchy and it senses whatever that hierarchy makes of its pose — in a
 * plain UE scene that is simply where you placed it.
 *
 * Physics-engine adapters replace the ray source instead: UMjLidarSensor
 * (MuJoCo) overrides HasExternalScanSource and produces each scan against the
 * compiled MuJoCo model on the physics thread (mj_multiRay — the simulation's
 * ground truth), streaming pose, ray directions, and ranges into the core's
 * PublishRayResults on the game thread. The core's post-processing (points,
 * targets, publishing) is shared by both ray sources.
 *
 * Data path (all game thread unless noted):
 *  1. TickComponent rate-limits scans on the scan clock (GetScanTimeSeconds);
 *     adapters with an external ray source skip this — they schedule scans
 *     themselves (UMjLidarSensor gates on MuJoCo simulation time).
 *  2. The beam grid is expanded (plus angular noise), each beam is a single
 *     LineTrace along MaxRange on TraceChannel; hits closer than MinRange
 *     after range noise fall in the blind zone and are dropped.
 *  3. Hits become FLidarPoints (world space, centimetres), optionally
 *     cluster into targets, and the completed FLidarScan broadcasts on
 *     OnLidarScan. Published scans contain hits only — misses are omitted.
 *
 * Beam convention (sensor-local frame): azimuth sweeps around the sensor's
 * local +Z (up), elevation is positive above the local X-XY plane, and
 * azimuth/elevation of 0/0 points along the sensor's local +X. The
 * full-circle azimuth grid is endpoint-exclusive; partial grids are
 * endpoint-inclusive; a single beam sits at the midpoint of its range.
 *
 * Cost note: rays are serial LineTraces on the game thread. A few thousand
 * beams per scan is comfortable; five-digit beam counts (e.g. dense Ouster
 * presets) are functional but can take milliseconds per scan.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class URLAB_API ULidarComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	ULidarComponent();

	// --- Scan pattern ---

	/** Number of azimuth beams (columns). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Scan", meta = (ClampMin = "1", ClampMax = "4096"))
	int32 AzimuthBeams = 360;

	/** Azimuth range start, degrees. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Scan", meta = (ClampMin = "-360", ClampMax = "360"))
	float AzimuthFovStart = -180.0f;

	/** Azimuth range end, degrees. A span of 360 sweeps the full circle (endpoint-exclusive). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Scan", meta = (ClampMin = "-360", ClampMax = "360"))
	float AzimuthFovEnd = 180.0f;

	/** Number of elevation beams (rows). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Scan", meta = (ClampMin = "1", ClampMax = "4096"))
	int32 ElevationBeams = 1;

	/** Elevation range start, degrees. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Scan", meta = (ClampMin = "-90", ClampMax = "90"))
	float ElevationFovStart = 0.0f;

	/** Elevation range end, degrees. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Scan", meta = (ClampMin = "-90", ClampMax = "90"))
	float ElevationFovEnd = 0.0f;

	/** Minimum range in metres; returns closer than this are discarded (blind zone). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Scan", meta = (ClampMin = "0", ClampMax = "50"))
	float MinRange = 0.1f;

	/** Maximum range in metres; the LineTrace length per beam. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Scan", meta = (ClampMin = "0.1", ClampMax = "1000"))
	float MaxRange = 100.0f;

	// --- Timing ---

	/**
	 * Scans per second on the scan clock (see GetScanTimeSeconds), so the
	 * lidar does not rescan on every tick. Values <= 0 scan on every tick.
	 * Subclasses may swap the clock: UMjLidarSensor uses MuJoCo simulation
	 * time, so pauses and sim-speed changes are honoured for free.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Timing", meta = (ClampMin = "0"))
	float ScanFrequencyHz = 10.0f;

	// --- Raycast (built-in Unreal LineTrace source) ---

	/**
	 * Collision channel the beams trace against when using the built-in
	 * Unreal ray source (a bare ULidarComponent; adapters with an external
	 * ray source ignore this). A surface is visible when its collision
	 * responses block this channel.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Raycast")
	TEnumAsByte<ECollisionChannel> TraceChannel = ECC_Visibility;

	/**
	 * Ignore the owning actor's primitives (prevents self-hits at the
	 * origin). Built-in Unreal ray source only.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Raycast")
	bool bIgnoreOwner = true;

	// --- Noise ---

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Noise")
	bool bEnableNoise = false;

	/** Range noise standard deviation, metres. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Noise", meta = (ClampMin = "0", EditCondition = "bEnableNoise"))
	float RangeNoiseStdDev = 0.01f;

	/** Azimuth noise standard deviation, degrees. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Noise", meta = (ClampMin = "0", EditCondition = "bEnableNoise"))
	float AzimuthNoiseStdDev = 0.0f;

	/** Elevation noise standard deviation, degrees. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Noise", meta = (ClampMin = "0", EditCondition = "bEnableNoise"))
	float ElevationNoiseStdDev = 0.0f;

	// --- Output ---

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Output")
	ELidarOutputMode OutputMode = ELidarOutputMode::Points;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Output")
	bool bDrawDebugPoints = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Output")
	bool bDrawDebugLines = false;

	// --- Target aggregation ---

	/**
	 * Maximum distance between two hit points (metres) for them to join the
	 * same cluster. Also gates target matching between consecutive scans.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Targeting", meta = (ClampMin = "0.01"))
	float ClusterDistanceThreshold = 0.5f;

	/**
	 * Compute per-target radial speed from centroid differencing. Kinematic
	 * approximation, not a Doppler measurement.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Targeting")
	bool bComputeVelocity = true;

	/**
	 * Number of past centroids kept per target (velocity smoothing window
	 * and coasting horizon in scans).
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Lidar|Targeting", meta = (ClampMin = "2", ClampMax = "64"))
	int32 HistorySize = 8;

	// --- Results ---

	/** Hit points of the most recently published scan (empty in Targets mode). */
	UPROPERTY(BlueprintReadOnly, Category = "Lidar")
	TArray<FLidarPoint> LastPoints;

	/** Aggregated targets of the most recently published scan (empty in Points mode). */
	UPROPERTY(BlueprintReadOnly, Category = "Lidar")
	TArray<FLidarTarget> LastTargets;

	/** Broadcast on the game thread after each scan is published. */
	UPROPERTY(BlueprintAssignable, Category = "Lidar")
	FOnLidarScan OnLidarScan;

	/** Returns the most recently published scan (points/targets per OutputMode plus metadata). */
	UFUNCTION(BlueprintCallable, Category = "Lidar")
	FLidarScan GetLastScan() const;

	// --- Pure logic helpers (static, unit-testable) ---

	/**
	 * Expands the beam angle grid. Full-circle azimuth spans (|end-start| >=
	 * 359.99) are endpoint-exclusive (periodic); partial spans and elevation
	 * are endpoint-inclusive; a single beam sits at the span midpoint.
	 */
	static void BuildBeamAngles(int32 AzBeams, float AzStart, float AzEnd,
		int32 ElBeams, float ElStart, float ElEnd,
		TArray<float>& OutAzimuths, TArray<float>& OutElevations);

	/**
	 * Unit beam direction for the given angles, expressed in the sensor's
	 * local frame (azimuth around +Z from +X, elevation positive towards
	 * +Z).
	 */
	static FVector SphericalDirection(float AzimuthDeg, float ElevationDeg);

	/**
	 * Clusters points by Euclidean distance using a spatial hash + union-find.
	 * OutClusterIds[i] is the cluster index of Points[i]; OutClusterSizes[k]
	 * is the size of cluster k.
	 */
	static void ClusterPoints(const TArray<FVector>& Points, float ThresholdCm,
		TArray<int32>& OutClusterIds, TArray<int32>& OutClusterSizes);

	/** Zero-mean Gaussian sample drawn from the stream (Box-Muller). */
	static float SampleGaussian(FRandomStream& Stream, float StdDev);

protected:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/**
	 * Scan-clock seconds used for rate limiting and published as scan
	 * SimTime. Base implementation returns world time. Subclasses override
	 * to schedule on another clock (e.g. UMjLidarSensor gates scans on
	 * MuJoCo simulation time inside its physics-thread callback).
	 */
	virtual double GetScanTimeSeconds();

	/**
	 * True when an adapter produces scans through its own ray source and
	 * calls PublishRayResults itself; the built-in tick scan (and its rate
	 * gate) is then skipped entirely.
	 */
	virtual bool HasExternalScanSource() const { return false; }

	/**
	 * Shared post-processing sink for every ray source (game thread): builds
	 * FLidarPoints from the beam angles and per-ray results around
	 * SensorPose, draws debug output, aggregates targets, updates the Last*
	 * results, and broadcasts OnLidarScan.
	 *
	 * RangesM[i] < 0 marks a miss (blind-zone returns included);
	 * SurfaceIds[i] is the hit surface's opaque id, -1 when unknown.
	 */
	void PublishRayResults(const FTransform& SensorPose,
		const TArray<float>& Azimuths, const TArray<float>& Elevations,
		const TArray<float>& RangesM, const TArray<int32>& SurfaceIds,
		int32 AzCount, double SimTime, int64 ScanId);

private:
	/** Runs one complete scan pipeline (beams, traces, filters, publish). */
	void RunScan();

	/** Clusters current hits into targets, matching against m_Tracks. */
	void BuildTargets(const TArray<FVector>& HitPositionsCm, double SimTime, const FVector& SensorPosUe, TArray<FLidarTarget>& OutTargets);

	/** Stable non-negative id for a hit component (name hash); -1 for null. */
	static int32 SurfaceIdFromComponent(const class UPrimitiveComponent* Component);

	/** Target tracking state (game thread). */
	struct FTargetTrack
	{
		int32 Id = -1;
		FVector Centroid = FVector::ZeroVector;
		double SimTime = 0.0;
		int32 Misses = 0;
		TArray<FVector> HistoryPos; // bounded by HistorySize
		TArray<double> HistoryTime;
	};
	TArray<FTargetTrack> m_Tracks;
	int32 m_NextTargetId = 1;

	int64 m_ScanCounter = 0;
	int64 m_LastScanId = -1;
	double m_LastScanSimTime = 0.0;
	double m_NextScanTime = 0.0;
	FRandomStream m_NoiseStream;

	TArray<float> m_ScratchAz;
	TArray<float> m_ScratchEl;
	TArray<int32> m_ScratchHitRays; // ray index per published hit point, reused across scans
	double m_NextTimingLogSec = 0.0; // wall-clock gate for the throttled per-scan timing log
};
