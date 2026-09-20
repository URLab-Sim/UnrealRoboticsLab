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
#include "MjLidarPointCloudViz.generated.h"

class ULidarComponent;
class UStaticMesh;
class UMaterialInterface;
class UNiagaraSystem;
class UInstancedStaticMeshComponent;
class UNiagaraComponent;

/**
 * @enum EMjLidarVizBackend
 * @brief Rendering backend used to draw the accumulated point cloud.
 */
UENUM(BlueprintType)
enum class EMjLidarVizBackend : uint8
{
	/** DrawDebugPoint calls (default fallback; strided by BatchedMaxPoints). */
	BatchedDebug UMETA(DisplayName = "Batched Debug (DrawDebugPoint)"),
	/** UInstancedStaticMeshComponent cubes, per-instance colored via Custom Data. */
	InstancedMesh UMETA(DisplayName = "Instanced Static Mesh"),
	/** UNiagaraComponent fed from user-exposed position/color array parameters. */
	Niagara UMETA(DisplayName = "Niagara Point System")
};

/**
 * @enum EMjLidarVizColorMode
 * @brief How each point of the cloud is colored.
 */
UENUM(BlueprintType)
enum class EMjLidarVizColorMode : uint8
{
	/** Fixed PointColor. */
	SingleColor UMETA(DisplayName = "Single Color"),
	/** Distance ramp over [RangeMinM..RangeMaxM]: near red, far blue. */
	ByRange UMETA(DisplayName = "By Range"),
	/** Height ramp over the current scan's Z min/max (dynamic normalization). */
	ByHeight UMETA(DisplayName = "By Height"),
	/** Elevation ramp over the current scan's elevation min/max: one color per scan ring. */
	ByElevationRing UMETA(DisplayName = "By Elevation Ring"),
	/** Discrete palette indexed by hit surface id (gray for misses). */
	BySurfaceId UMETA(DisplayName = "By Hit Surface")
};

/**
 * @struct FMjLidarVizColorContext
 * @brief Normalization bounds consumed by UMjLidarPointCloudViz::ComputePointColor.
 *
 * Plain C++ (not reflected): the ByHeight/ByElevationRing bounds are recomputed
 * from each incoming scan; buffered historical points keep the colors they
 * were assigned when their scan arrived.
 */
struct FMjLidarVizColorContext
{
	FLinearColor SingleColor = FLinearColor::Green;
	float RangeMinM = 0.0f, RangeMaxM = 100.0f;      // ByRange
	float HeightMinCm = 0.0f, HeightMaxCm = 100.0f;  // ByHeight (dynamic per scan)
	float ElevationMinDeg = -15.0f, ElevationMaxDeg = 15.0f; // ByElevationRing (dynamic per scan)
};

/**
 * @class UMjLidarPointCloudViz
 * @brief Debug visualization of the point cloud published by a ULidarComponent.
 *
 * A pure UE-side consumer (NOT a UMjComponent: nothing is registered into the
 * MuJoCo spec and mjData is never touched). BeginPlay resolves the sensor
 * (SourceSensor, or the owner's first ULidarComponent — which includes the
 * UMjLidarSensor adapter) and subscribes to OnLidarScan, which broadcasts on
 * the game thread after each scan. The component is event-driven (no Tick):
 * every scan is appended to a bounded history buffer and re-drawn through the
 * active backend.
 *
 * Backends (lazy-created on the first scan, rebuilt when the Backend property
 * changes mid-PIE): BatchedDebug points (strided), an instanced static mesh
 * colored through Per-Instance Custom Data slots 0..2, or a Niagara system fed
 * via user array parameters. Batched overlays (sensor origin axes, target
 * centroid spheres) share the backends' debug lifetime.
 *
 * The buffered cloud can be exported as an ASCII PLY file (positions in
 * metres) via ExportLastScanToPLY.
 */
UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjLidarPointCloudViz : public USceneComponent
{
	GENERATED_BODY()

public:
	UMjLidarPointCloudViz();

	// --- General ---

	/** Rendering backend. Switching it mid-PIE destroys and recreates the backend on the next scan. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Viz")
	EMjLidarVizBackend Backend = EMjLidarVizBackend::BatchedDebug;

	/** Sensor to visualize. Empty: the owner's first ULidarComponent is used (BeginPlay). */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "MuJoCo|Lidar|Viz")
	TObjectPtr<ULidarComponent> SourceSensor;

	/** Point colorization mode. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Viz")
	EMjLidarVizColorMode ColorMode = EMjLidarVizColorMode::ByRange;

	/** Fixed point color, used when ColorMode is SingleColor. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Viz", meta = (EditCondition = "ColorMode == EMjLidarVizColorMode::SingleColor", EditConditionHides))
	FLinearColor PointColor = FLinearColor(0.1f, 1.0f, 0.2f);

	/** Base point size in centimetres (cube edge length for the InstancedMesh backend). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Viz", meta = (ClampMin = "0.1"))
	float PointSizeCm = 6.0f;

	/** Scale size by hit distance: Size = PointSizeCm * clamp(RangeM / 10, 0.25, 4). */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "MuJoCo|Lidar|Viz")
	bool bScaleSizeByDistance = false;

	/** Ignore incoming scans and keep the current cloud (for close inspection). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Viz")
	bool bFreeze = false;

	/** Draw a 30 cm XYZ axis triplet at the sensor origin. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Viz")
	bool bShowSensorOrigin = true;

	/** Draw a sphere at each aggregated target's centroid. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Viz")
	bool bDrawTargetCentroids = true;

	/** Radius of the target centroid spheres. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Viz", meta = (EditCondition = "bDrawTargetCentroids", EditConditionHides))
	float TargetSphereRadiusCm = 30.0f;

	// --- History ---

	/** Scans kept in the cloud (1 = latest scan only). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Viz|History", meta = (ClampMin = "1", ClampMax = "120"))
	int32 HistoryScans = 1;

	/** Hard cap on buffered points; excess is dropped oldest-first. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Lidar|Viz|History", meta = (ClampMin = "16", ClampMax = "1048576"))
	int32 MaxPoints = 65536;

	// --- Color ranges ---

	/** ByRange lower bound in metres (resynced from the sensor's MinRange at BeginPlay). */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "MuJoCo|Lidar|Viz|Color Ranges", meta = (ClampMin = "0.01"))
	float RangeMinM = 0.1f;

	/** ByRange upper bound in metres (resynced from the sensor's MaxRange at BeginPlay). */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "MuJoCo|Lidar|Viz|Color Ranges", meta = (ClampMin = "0.01"))
	float RangeMaxM = 100.0f;

	// --- Batched Debug ---

	/** DrawDebugPoint budget; beyond it the cloud is drawn with a uniform stride. */
	UPROPERTY(EditAnywhere, Category = "MuJoCo|Lidar|Viz|Batched Debug", meta = (ClampMin = "100"))
	int32 BatchedMaxPoints = 20000;

	// --- Instanced Mesh ---

	/** Mesh drawn per point (default: /Engine/BasicShapes/Cube, 100 cm base => instance scale = SizeCm/100). */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "MuJoCo|Lidar|Viz|Instanced Mesh")
	TObjectPtr<UStaticMesh> PointMesh;

	/**
	 * Material reading Per-Instance Custom Data slots 0..2 as RGB (three
	 * Per Instance Custom Data expressions appended into RGB, wired to
	 * Base Color/Emissive). Unset: the mesh's default material renders -
	 * shape checks still work, per-point colors do not.
	 */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "MuJoCo|Lidar|Viz|Instanced Mesh")
	TObjectPtr<UMaterialInterface> PointMaterial;

	// --- Niagara ---

	/** Point system exposing the user array parameters below. Empty: falls back to BatchedDebug. */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "MuJoCo|Lidar|Viz|Niagara")
	TObjectPtr<UNiagaraSystem> PointSystem;

	/** User parameter names expected on PointSystem. */
	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "MuJoCo|Lidar|Viz|Niagara")
	FName PositionArrayName = TEXT("MjLidar.Positions");

	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "MuJoCo|Lidar|Viz|Niagara")
	FName ColorArrayName = TEXT("MjLidar.Colors");

	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "MuJoCo|Lidar|Viz|Niagara")
	FName PointCountParamName = TEXT("MjLidar.PointCount");

	UPROPERTY(EditAnywhere, AdvancedDisplay, Category = "MuJoCo|Lidar|Viz|Niagara")
	FName PointSizeParamName = TEXT("MjLidar.PointSize");

	// --- API ---

	/**
	 * Exports the current cloud (including history) as ASCII PLY to
	 * <Project>/Saved/URLab/LidarScans/. Positions are written in metres.
	 * Returns true and fills OutFilePath on success.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Lidar|Viz")
	bool ExportLastScanToPLY(FString& OutFilePath);

	/** Clears the cloud, scan counts, and backend instances. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Lidar|Viz")
	void ClearCloud();

	/** Number of points currently buffered (drawn in full by the ISM/Niagara backends). */
	UFUNCTION(BlueprintPure, Category = "MuJoCo|Lidar|Viz")
	int32 GetRenderedPointCount() const;

	// --- Pure logic helpers (static, unit-testable) ---

	/** Red -> yellow -> green -> blue ramp; T01 clamped to [0,1] (near = red, far = blue). */
	static FLinearColor RangeRampColor(float T01);

	/** Fixed 12-color palette indexed by geom id modulo 12; negative ids (misses) map to gray. */
	static FLinearColor SurfaceIdColor(int32 SurfaceId);

	/** Deterministic hash -> hue color for target ids. */
	static FLinearColor TargetIdColor(int32 TargetId);

	/** Colors one point according to Mode using Ctx's normalization bounds. */
	static FLinearColor ComputePointColor(EMjLidarVizColorMode Mode, const FLidarPoint& Point, const FMjLidarVizColorContext& Ctx);

	/** 1 while TotalPoints <= MaxDrawn, otherwise ceil(TotalPoints / MaxDrawn). */
	static int32 SubsampleStride(int32 TotalPoints, int32 MaxDrawn);

	/** ASCII PLY text ("x y z r g b", positions in metres, uchar colors) for the given cloud. */
	static FString BuildPlyText(const TArray<FVector>& PositionsCm, const TArray<FLinearColor>& Colors);

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	/** OnLidarScan handler (broadcast on the game thread; shared payload, treated as read-only). */
	UFUNCTION()
	void HandleLidarScan(const FLidarScan& Scan);

	/** One buffered visualization point. */
	struct FVizPoint
	{
		FVector Pos = FVector::ZeroVector;
		FLinearColor Color = FLinearColor::Green;
		float SizeCm = 6.0f;
	};

	/** Appends Scan's points (colors/sizes applied) to the cloud, then trims history and cap. */
	void AppendScan(const FLidarScan& Scan);

	/** Creates the backend component on first use; honors the Niagara fallback. */
	void EnsureBackendCreated();

	/** Destroys the backend components created by this component. */
	void DestroyBackend();

	/** Refreshes the backend from m_Cloud, then draws the origin/target overlays. */
	void RebuildBackend(const FLidarScan& Scan);

	/** DrawDebugPoint pass, strided by BatchedMaxPoints. */
	void RebuildBatchedDebug();

	/** ClearInstances + AddInstances (world space) + per-instance Custom Data pass. */
	void RebuildInstancedMesh();

	/** Pushes the cloud into the Niagara user array parameters. */
	void RebuildNiagara();

	/** Debug-drawn overlays shared by all backends: sensor origin axes and target spheres. */
	void DrawOverlays(const FLidarScan& Scan);

	/** HistoryScans x scan period, clamped to [0.1, 12] s (0.5 s period when unknown). */
	float ResolveDebugLifetime() const;

	/** Color context for one scan: fixed bounds plus dynamic height/elevation min/max. */
	FMjLidarVizColorContext BuildColorContext(const TArray<FLidarPoint>& Points) const;

	/** Copies m_Cloud into parallel position/color arrays (PLY export and the Niagara push). */
	void GatherPositionsAndColors(TArray<FVector>& OutPositions, TArray<FLinearColor>& OutColors) const;

	// ---- Bound sensor (weak: the sensor may be destroyed first) ----
	TWeakObjectPtr<ULidarComponent> m_ResolvedSensor;

	// ---- Cloud state (game thread only) ----
	TArray<FVizPoint> m_Cloud;
	TArray<int32> m_ScanCounts; // per-scan point counts, oldest first
	int64 m_LastScanId = -1;

	// ---- Backend state ----
	bool m_bBackendCreated = false;
	EMjLidarVizBackend m_ActiveBackend = EMjLidarVizBackend::BatchedDebug;
	TObjectPtr<UInstancedStaticMeshComponent> m_IsmComponent;
	TObjectPtr<UNiagaraComponent> m_NiagaraComponent;
	bool m_bWarnedNoNiagaraSystem = false;
	bool m_bWarnedNoPointMesh = false;
};
