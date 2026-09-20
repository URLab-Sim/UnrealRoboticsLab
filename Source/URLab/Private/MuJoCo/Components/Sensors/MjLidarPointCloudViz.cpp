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

#include "MuJoCo/Components/Sensors/MjLidarPointCloudViz.h"

#include "Components/LidarComponent.h"
#include "Utils/URLabLogging.h"

#include "Components/InstancedStaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/PlatformFileManager.h"
#include "Materials/MaterialInterface.h"
#include "Misc/DateTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "NiagaraComponent.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "NiagaraSystem.h"
#include "UObject/ConstructorHelpers.h"

namespace
{
	// Linear map of Value from [Min, Max] to [0, 1]; a degenerate (flat)
	// range maps to the ramp midpoint so a constant field still colors.
	float Normalize01(float Value, float Min, float Max)
	{
		const float Span = Max - Min;
		if (Span > -KINDA_SMALL_NUMBER && Span < KINDA_SMALL_NUMBER)
		{
			return 0.5f;
		}
		return FMath::Clamp((Value - Min) / Span, 0.0f, 1.0f);
	}

	// 32-bit FNV-1a over the id's bytes plus a finalize mix; deterministic
	// on every platform and compiler (no libc rand, no float seeding).
	uint32 HashInt32(int32 Value)
	{
		uint32 Hash = 2166136261u;
		for (int32 Byte = 0; Byte < 4; ++Byte)
		{
			Hash ^= static_cast<uint32>((Value >> (8 * Byte)) & 0xFF);
			Hash *= 16777619u;
		}
		Hash ^= Hash >> 15;
		Hash *= 2246822519u;
		Hash ^= Hash >> 13;
		return Hash;
	}

	float Hash01(int32 Value)
	{
		return static_cast<float>(HashInt32(Value) & 0x00FFFFFFu) / static_cast<float>(0x01000000u);
	}

	// Small HSL -> linear RGB conversion (Hue/Saturation/Lightness in [0, 1]).
	FLinearColor HslToLinear(float Hue, float Saturation, float Lightness)
	{
		const float H = FMath::Frac(Hue) * 6.0f;
		const int32 Sector = FMath::Clamp(FMath::TruncToInt32(H), 0, 5);
		const float Frac = H - static_cast<float>(Sector);
		const float Chroma = (1.0f - FMath::Abs(2.0f * Lightness - 1.0f)) * Saturation;
		const float ChromaX = Chroma * ((Sector % 2 == 0) ? Frac : 1.0f - Frac);
		const float Base = Lightness - Chroma * 0.5f;
		switch (Sector)
		{
		case 0: return FLinearColor(Chroma + Base, ChromaX + Base, Base);
		case 1: return FLinearColor(ChromaX + Base, Chroma + Base, Base);
		case 2: return FLinearColor(Base, Chroma + Base, ChromaX + Base);
		case 3: return FLinearColor(Base, ChromaX + Base, Chroma + Base);
		case 4: return FLinearColor(ChromaX + Base, Base, Chroma + Base);
		default: return FLinearColor(Chroma + Base, Base, ChromaX + Base);
		}
	}

	// Common setup shared by the lazily-created backend components.
	template <typename TComponent>
	TComponent* CreateBackendComponent(AActor* Owner, USceneComponent* Parent)
	{
		TComponent* Comp = NewObject<TComponent>(Owner);
		Comp->SetupAttachment(Parent);
		Comp->SetMobility(EComponentMobility::Movable);
		Comp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Comp->SetCastShadow(false);
		return Comp;
	}
} // namespace

UMjLidarPointCloudViz::UMjLidarPointCloudViz()
{
	// Event-driven: the sensor's OnLidarScan broadcast drives everything.
	PrimaryComponentTick.bCanEverTick = false;

	// Default point mesh: the engine's 100 cm cube (instance scale = SizeCm / 100).
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeFinder(TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (CubeFinder.Succeeded())
	{
		PointMesh = CubeFinder.Object;
	}
}

void UMjLidarPointCloudViz::BeginPlay()
{
	Super::BeginPlay();

	ULidarComponent* Sensor = SourceSensor;
	if (!Sensor && GetOwner())
	{
		Sensor = GetOwner()->FindComponentByClass<ULidarComponent>();
	}
	if (!Sensor)
	{
		UE_LOG(LogURLab, Warning,
			TEXT("MjLidarPointCloudViz on '%s': no ULidarComponent found (SourceSensor unset and the owner has none); the visualization stays idle."),
			*GetNameSafe(GetOwner()));
		return;
	}

	m_ResolvedSensor = Sensor;

	// Sync the ByRange normalization bounds with the sensor's actual range.
	RangeMinM = Sensor->MinRange;
	RangeMaxM = Sensor->MaxRange;

	Sensor->OnLidarScan.AddDynamic(this, &UMjLidarPointCloudViz::HandleLidarScan);
}

void UMjLidarPointCloudViz::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (ULidarComponent* Sensor = m_ResolvedSensor.Get())
	{
		Sensor->OnLidarScan.RemoveDynamic(this, &UMjLidarPointCloudViz::HandleLidarScan);
	}
	m_ResolvedSensor = nullptr;

	DestroyBackend();
	m_Cloud.Reset();
	m_ScanCounts.Reset();

	Super::EndPlay(EndPlayReason);
}

void UMjLidarPointCloudViz::HandleLidarScan(const FLidarScan& Scan)
{
	if (bFreeze)
	{
		return;
	}

	if (m_bBackendCreated && m_ActiveBackend != Backend)
	{
		DestroyBackend(); // support switching backends mid-PIE
	}

	AppendScan(Scan);
	RebuildBackend(Scan);
}

void UMjLidarPointCloudViz::AppendScan(const FLidarScan& Scan)
{
	if (Scan.ScanId >= 0 && Scan.ScanId == m_LastScanId)
	{
		return; // duplicate broadcast of an already-buffered scan
	}
	m_LastScanId = Scan.ScanId;

	// Dynamic normalization (ByHeight / ByElevationRing) uses THIS scan's
	// extents; historical points keep the colors computed at their arrival.
	const FMjLidarVizColorContext Ctx = BuildColorContext(Scan.Points);

	const int32 NewCount = Scan.Points.Num();
	m_Cloud.Reserve(m_Cloud.Num() + NewCount);
	for (const FLidarPoint& Point : Scan.Points)
	{
		FVizPoint& VizPoint = m_Cloud.AddDefaulted_GetRef();
		VizPoint.Pos = Point.WorldPos;
		VizPoint.Color = ComputePointColor(ColorMode, Point, Ctx);
		VizPoint.SizeCm = PointSizeCm;
		if (bScaleSizeByDistance)
		{
			VizPoint.SizeCm *= FMath::Clamp(Point.RangeM / 10.0f, 0.25f, 4.0f);
		}
	}
	m_ScanCounts.Add(NewCount);

	// Whole scans beyond HistoryScans expire oldest-first (empty scans count
	// too, so stale points from Targets mode / all-miss frames age out).
	while (m_ScanCounts.Num() > FMath::Max(1, HistoryScans))
	{
		m_Cloud.RemoveAt(0, m_ScanCounts[0]);
		m_ScanCounts.RemoveAt(0);
	}

	// Enforce the total cap: drop from the head and decrement the oldest
	// scan's count; remove a zeroed count while further scans remain.
	while (m_Cloud.Num() > MaxPoints)
	{
		const int32 Excess = m_Cloud.Num() - MaxPoints;
		const int32 Take = FMath::Min(Excess, m_ScanCounts[0]);
		m_Cloud.RemoveAt(0, Take);
		m_ScanCounts[0] -= Take;
		if (m_ScanCounts[0] <= 0 && m_ScanCounts.Num() > 1)
		{
			m_ScanCounts.RemoveAt(0);
		}
	}
}

FMjLidarVizColorContext UMjLidarPointCloudViz::BuildColorContext(const TArray<FLidarPoint>& Points) const
{
	FMjLidarVizColorContext Ctx;
	Ctx.SingleColor = PointColor;
	Ctx.RangeMinM = RangeMinM;
	Ctx.RangeMaxM = RangeMaxM;

	if (Points.Num() > 0)
	{
		float MinZ = Points[0].WorldPos.Z;
		float MaxZ = MinZ;
		float MinEl = Points[0].ElevationDeg;
		float MaxEl = MinEl;
		for (const FLidarPoint& Point : Points)
		{
			MinZ = FMath::Min(MinZ, Point.WorldPos.Z);
			MaxZ = FMath::Max(MaxZ, Point.WorldPos.Z);
			MinEl = FMath::Min(MinEl, Point.ElevationDeg);
			MaxEl = FMath::Max(MaxEl, Point.ElevationDeg);
		}
		Ctx.HeightMinCm = MinZ;
		Ctx.HeightMaxCm = MaxZ;
		Ctx.ElevationMinDeg = MinEl;
		Ctx.ElevationMaxDeg = MaxEl;
	}

	return Ctx;
}

void UMjLidarPointCloudViz::GatherPositionsAndColors(TArray<FVector>& OutPositions, TArray<FLinearColor>& OutColors) const
{
	OutPositions.Reset();
	OutColors.Reset();
	OutPositions.Reserve(m_Cloud.Num());
	OutColors.Reserve(m_Cloud.Num());
	for (const FVizPoint& VizPoint : m_Cloud)
	{
		OutPositions.Add(VizPoint.Pos);
		OutColors.Add(VizPoint.Color);
	}
}

// ---------------------------------------------------------------------------
// Static pure helpers
// ---------------------------------------------------------------------------

FLinearColor UMjLidarPointCloudViz::RangeRampColor(float T01)
{
	const float T = FMath::Clamp(T01, 0.0f, 1.0f);

	// Four stops: red -> yellow -> green -> blue (near = red, far = blue).
	static const FLinearColor Stops[4] =
	{
		FLinearColor(1.0f, 0.0f, 0.0f),
		FLinearColor(1.0f, 1.0f, 0.0f),
		FLinearColor(0.0f, 1.0f, 0.0f),
		FLinearColor(0.0f, 0.0f, 1.0f)
	};

	const float Seg = T * 3.0f;
	const int32 Index = FMath::Clamp(FMath::FloorToInt32(Seg), 0, 2);
	const float Frac = Seg - static_cast<float>(Index);
	return FMath::Lerp(Stops[Index], Stops[Index + 1], Frac);
}

FLinearColor UMjLidarPointCloudViz::SurfaceIdColor(int32 SurfaceId)
{
	if (SurfaceId < 0)
	{
		return FLinearColor::Gray; // miss / blind zone
	}

	static const FLinearColor Palette[12] =
	{
		FLinearColor::Red,
		FLinearColor::Green,
		FLinearColor::Blue,
		FLinearColor::Yellow,
		FLinearColor(0.0f, 1.0f, 1.0f),
		FLinearColor(1.0f, 0.0f, 1.0f),
		FLinearColor(1.0f, 0.5f, 0.0f),
		FLinearColor(0.5f, 0.0f, 1.0f),
		FLinearColor(0.0f, 0.5f, 0.5f),
		FLinearColor(1.0f, 0.4f, 0.7f),
		FLinearColor(0.5f, 1.0f, 0.0f),
		FLinearColor(0.55f, 0.35f, 0.15f)
	};
	return Palette[SurfaceId % 12];
}

FLinearColor UMjLidarPointCloudViz::TargetIdColor(int32 TargetId)
{
	// Deterministic hash-derived hue (HSL); distinct ids spread across the wheel.
	const float Hue = Hash01(TargetId);
	return HslToLinear(Hue, 0.85f, 0.5f);
}

FLinearColor UMjLidarPointCloudViz::ComputePointColor(EMjLidarVizColorMode Mode, const FLidarPoint& Point, const FMjLidarVizColorContext& Ctx)
{
	switch (Mode)
	{
	case EMjLidarVizColorMode::SingleColor:
		return Ctx.SingleColor;
	case EMjLidarVizColorMode::ByRange:
		return RangeRampColor(Normalize01(Point.RangeM, Ctx.RangeMinM, Ctx.RangeMaxM));
	case EMjLidarVizColorMode::ByHeight:
		return RangeRampColor(Normalize01(Point.WorldPos.Z, Ctx.HeightMinCm, Ctx.HeightMaxCm));
	case EMjLidarVizColorMode::ByElevationRing:
		return RangeRampColor(Normalize01(Point.ElevationDeg, Ctx.ElevationMinDeg, Ctx.ElevationMaxDeg));
	case EMjLidarVizColorMode::BySurfaceId:
	default:
		return SurfaceIdColor(Point.HitSurfaceId);
	}
}

int32 UMjLidarPointCloudViz::SubsampleStride(int32 TotalPoints, int32 MaxDrawn)
{
	if (TotalPoints <= 0)
	{
		return 1;
	}
	if (MaxDrawn <= 0)
	{
		MaxDrawn = 1; // guard against a zero/negative budget
	}
	if (TotalPoints <= MaxDrawn)
	{
		return 1;
	}
	return FMath::DivideAndRoundUp(TotalPoints, MaxDrawn);
}

FString UMjLidarPointCloudViz::BuildPlyText(const TArray<FVector>& PositionsCm, const TArray<FLinearColor>& Colors)
{
	// PLY stores metres (lidar convention): positions are divided by 100 here.
	const int32 Count = FMath::Min(PositionsCm.Num(), Colors.Num());

	FString Text;
	Text.Reserve(Count * 40 + 160);
	Text += TEXT("ply\n");
	Text += TEXT("format ascii 1.0\n");
	Text += FString::Printf(TEXT("element vertex %d\n"), Count);
	Text += TEXT("property float x\n");
	Text += TEXT("property float y\n");
	Text += TEXT("property float z\n");
	Text += TEXT("property uchar red\n");
	Text += TEXT("property uchar green\n");
	Text += TEXT("property uchar blue\n");
	Text += TEXT("end_header\n");

	for (int32 i = 0; i < Count; ++i)
	{
		const FVector& P = PositionsCm[i];
		const FColor ByteColor = Colors[i].ToFColor(/*sRGB*/ false);
		Text += FString::Printf(TEXT("%f %f %f %d %d %d\n"),
			P.X / 100.0f, P.Y / 100.0f, P.Z / 100.0f, ByteColor.R, ByteColor.G, ByteColor.B);
	}
	return Text;
}

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

bool UMjLidarPointCloudViz::ExportLastScanToPLY(FString& OutFilePath)
{
	OutFilePath.Reset();

	if (m_Cloud.Num() == 0)
	{
		UE_LOG(LogURLab, Warning, TEXT("MjLidarPointCloudViz: ExportLastScanToPLY called with an empty cloud; nothing written."));
		return false;
	}

	TArray<FVector> PositionsCm;
	TArray<FLinearColor> Colors;
	GatherPositionsAndColors(PositionsCm, Colors);

	const FString Directory = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLab"), TEXT("LidarScans"));
	if (!FPlatformFileManager::Get().GetPlatformFile().CreateDirectoryTree(*Directory))
	{
		UE_LOG(LogURLab, Error, TEXT("MjLidarPointCloudViz: could not create directory '%s'."), *Directory);
		return false;
	}

	const FDateTime Now = FDateTime::Now();
	const FString FilePath = FPaths::Combine(Directory,
		FString::Printf(TEXT("LidarScan_%lld_%02d%02d%02d.ply"),
			m_LastScanId, Now.GetHour(), Now.GetMinute(), Now.GetSecond()));

	if (!FFileHelper::SaveStringToFile(BuildPlyText(PositionsCm, Colors), *FilePath))
	{
		UE_LOG(LogURLab, Error, TEXT("MjLidarPointCloudViz: failed to write '%s'."), *FilePath);
		return false;
	}

	UE_LOG(LogURLab, Log, TEXT("MjLidarPointCloudViz: exported %d points to '%s'."), m_Cloud.Num(), *FilePath);
	OutFilePath = FilePath;
	return true;
}

void UMjLidarPointCloudViz::ClearCloud()
{
	m_Cloud.Reset();
	m_ScanCounts.Reset();
	m_LastScanId = -1;

	if (m_IsmComponent)
	{
		m_IsmComponent->ClearInstances();
		m_IsmComponent->MarkRenderStateDirty();
	}
	if (m_NiagaraComponent)
	{
		UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(m_NiagaraComponent, PositionArrayName, TArray<FVector>());
		UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(m_NiagaraComponent, ColorArrayName, TArray<FLinearColor>());
		m_NiagaraComponent->SetVariableInt(PointCountParamName, 0);
	}
}

int32 UMjLidarPointCloudViz::GetRenderedPointCount() const
{
	return m_Cloud.Num();
}

// ---------------------------------------------------------------------------
// Backends
// ---------------------------------------------------------------------------

void UMjLidarPointCloudViz::EnsureBackendCreated()
{
	if (m_bBackendCreated)
	{
		return;
	}

	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}
	USceneComponent* Parent = Owner->GetRootComponent();
	if (!Parent)
	{
		Parent = this; // rare: owner without a root; attach to the viz component instead
	}

	if (Backend == EMjLidarVizBackend::InstancedMesh)
	{
		if (!PointMesh && !m_bWarnedNoPointMesh)
		{
			UE_LOG(LogURLab, Warning, TEXT("MjLidarPointCloudViz on '%s': InstancedMesh backend has no PointMesh; the backend will render nothing."),
				*GetNameSafe(Owner));
			m_bWarnedNoPointMesh = true;
		}

		UInstancedStaticMeshComponent* Ism = CreateBackendComponent<UInstancedStaticMeshComponent>(Owner, Parent);
		Ism->SetStaticMesh(PointMesh);
		if (PointMaterial)
		{
			Ism->SetMaterial(0, PointMaterial);
		}
		// Per-instance RGB custom data must be declared before registration.
		Ism->SetNumCustomDataFloats(3);
		Ism->RegisterComponent();
		m_IsmComponent = Ism;
	}
	else if (Backend == EMjLidarVizBackend::Niagara)
	{
		if (!PointSystem)
		{
			if (!m_bWarnedNoNiagaraSystem)
			{
				UE_LOG(LogURLab, Warning, TEXT("MjLidarPointCloudViz on '%s': Niagara backend selected but PointSystem is empty; falling back to BatchedDebug."),
					*GetNameSafe(Owner));
				m_bWarnedNoNiagaraSystem = true;
			}
		}
		else
		{
			UNiagaraComponent* Niagara = CreateBackendComponent<UNiagaraComponent>(Owner, Parent);
			Niagara->SetAsset(PointSystem);
			Niagara->RegisterComponent();
			Niagara->Activate();
			m_NiagaraComponent = Niagara;
		}
	}

	m_ActiveBackend = m_NiagaraComponent
		? EMjLidarVizBackend::Niagara
		: (m_IsmComponent ? EMjLidarVizBackend::InstancedMesh : EMjLidarVizBackend::BatchedDebug);
	m_bBackendCreated = true;
}

void UMjLidarPointCloudViz::DestroyBackend()
{
	if (m_IsmComponent)
	{
		m_IsmComponent->DestroyComponent();
		m_IsmComponent = nullptr;
	}
	if (m_NiagaraComponent)
	{
		m_NiagaraComponent->DestroyComponent();
		m_NiagaraComponent = nullptr;
	}
	m_bBackendCreated = false;
}

void UMjLidarPointCloudViz::RebuildBackend(const FLidarScan& Scan)
{
	EnsureBackendCreated();
	if (!m_bBackendCreated)
	{
		return;
	}

	switch (m_ActiveBackend)
	{
	case EMjLidarVizBackend::InstancedMesh:
		RebuildInstancedMesh();
		break;
	case EMjLidarVizBackend::Niagara:
		RebuildNiagara();
		break;
	case EMjLidarVizBackend::BatchedDebug:
	default:
		RebuildBatchedDebug();
		break;
	}

	DrawOverlays(Scan);
}

void UMjLidarPointCloudViz::RebuildBatchedDebug()
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	const float Lifetime = ResolveDebugLifetime();
	const int32 Stride = SubsampleStride(m_Cloud.Num(), BatchedMaxPoints);
	for (int32 i = 0; i < m_Cloud.Num(); i += Stride)
	{
		const FVizPoint& VizPoint = m_Cloud[i];
		DrawDebugPoint(World, VizPoint.Pos, FMath::Max(VizPoint.SizeCm, 1.0f),
			VizPoint.Color.ToFColor(/*sRGB*/ false), false, Lifetime);
	}
}

void UMjLidarPointCloudViz::RebuildInstancedMesh()
{
	if (!m_IsmComponent)
	{
		return;
	}

	m_IsmComponent->ClearInstances();

	TArray<FTransform> Transforms;
	Transforms.Reserve(m_Cloud.Num());
	for (const FVizPoint& VizPoint : m_Cloud)
	{
		// The default cube mesh is 100 cm on a side: scale = SizeCm / 100.
		const float UniformScale = FMath::Max(VizPoint.SizeCm, KINDA_SMALL_NUMBER) / 100.0f;
		Transforms.Add(FTransform(FQuat::Identity, VizPoint.Pos, FVector(UniformScale)));
	}
	m_IsmComponent->AddInstances(Transforms, /*bShouldReturnIndices*/ false, /*bWorldSpace*/ true, /*bUpdateNavigation*/ false);

	// Per-instance color via Custom Data slots 0..2 (RGB). The batched range
	// setter's semantics are unverified for this engine build (no Private
	// sources shipped), so write point by point and dirty the render state
	// once at the end.
	for (int32 i = 0; i < m_Cloud.Num(); ++i)
	{
		const FLinearColor& C = m_Cloud[i].Color;
		const float CustomData[3] = { C.R, C.G, C.B };
		m_IsmComponent->SetCustomData(i, TArrayView<const float>(CustomData, 3), /*bMarkRenderStateDirty*/ false);
	}
	m_IsmComponent->MarkRenderStateDirty();
}

void UMjLidarPointCloudViz::RebuildNiagara()
{
	if (!m_NiagaraComponent)
	{
		return;
	}

	TArray<FVector> Positions;
	TArray<FLinearColor> Colors;
	GatherPositionsAndColors(Positions, Colors);

	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayPosition(m_NiagaraComponent, PositionArrayName, Positions);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(m_NiagaraComponent, ColorArrayName, Colors);
	m_NiagaraComponent->SetVariableInt(PointCountParamName, Positions.Num());
	m_NiagaraComponent->SetVariableFloat(PointSizeParamName, PointSizeCm);
}

// ---------------------------------------------------------------------------
// Overlays
// ---------------------------------------------------------------------------

void UMjLidarPointCloudViz::DrawOverlays(const FLidarScan& Scan)
{
	UWorld* World = GetWorld();
	if (!World || (!bShowSensorOrigin && !bDrawTargetCentroids))
	{
		return;
	}

	const float Lifetime = ResolveDebugLifetime();

	if (bShowSensorOrigin && m_ResolvedSensor.IsValid())
	{
		// FLidarScan carries no sensor origin; read it from the sensor component.
		const ULidarComponent* Sensor = m_ResolvedSensor.Get();
		const FVector Origin = Sensor->GetComponentLocation();
		const float Axis = 30.0f;
		DrawDebugLine(World, Origin, Origin + FVector(Axis, 0.0f, 0.0f), FColor::Red, false, Lifetime, 0, 0.3f);
		DrawDebugLine(World, Origin, Origin + FVector(0.0f, Axis, 0.0f), FColor::Green, false, Lifetime, 0, 0.3f);
		DrawDebugLine(World, Origin, Origin + FVector(0.0f, 0.0f, Axis), FColor::Blue, false, Lifetime, 0, 0.3f);
	}

	if (bDrawTargetCentroids)
	{
		for (const FLidarTarget& Target : Scan.Targets)
		{
			DrawDebugSphere(World, Target.CentroidWorldPos, TargetSphereRadiusCm, 10,
				TargetIdColor(Target.TargetId).ToFColor(/*sRGB*/ false), false, Lifetime, 0, 0.5f);
		}
	}
}

float UMjLidarPointCloudViz::ResolveDebugLifetime() const
{
	// One scan period by default; unknown/unset rates fall back to 0.5 s.
	float Period = 0.5f;
	if (const ULidarComponent* Sensor = m_ResolvedSensor.Get())
	{
		if (Sensor->ScanFrequencyHz > 0.0f)
		{
			Period = 1.0f / Sensor->ScanFrequencyHz;
		}
	}
	return FMath::Clamp(Period * FMath::Max(1, HistoryScans), 0.1f, 12.0f);
}
