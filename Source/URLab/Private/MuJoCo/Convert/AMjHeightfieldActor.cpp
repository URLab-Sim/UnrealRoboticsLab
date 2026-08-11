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

#include "MuJoCo/Convert/AMjHeightfieldActor.h"

#include "DrawDebugHelpers.h"
#include "MuJoCo/Convert/MjQuickConvertComponent.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Spec/MjFrameTypes.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Gen/Elements/Assets/MjAsset.gen.h"
#include "MuJoCo/Gen/Elements/Bodies/MjBody.gen.h"
#include "MuJoCo/Gen/Elements/Assets/MjHfield.gen.h"
#include "MuJoCo/Gen/Elements/MjModel.gen.h"
#include "Serialization/BufferArchive.h"
#include "Utils/URLabLogging.h"

#if URLAB_MJ_GEN
#include "MuJoCo/Spec/MjNodeFactories.h"
#endif

AMjHeightfieldActor::AMjHeightfieldActor()
{
	PrimaryActorTick.bCanEverTick = false;

	// Root bounding box — user scales/moves this to cover the terrain region
	BoundsBox = CreateDefaultSubobject<UBoxComponent>(TEXT("BoundsBox"));
	BoundsBox->SetBoxExtent(FVector(1000.f, 1000.f, 500.f));
	BoundsBox->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
	BoundsBox->SetCollisionResponseToAllChannels(ECR_Overlap);
	BoundsBox->SetLineThickness(2.0f);
	SetRootComponent(BoundsBox);

	// Line batch for the editor grid visualizer — no collision, no tick
	GridLines = CreateDefaultSubobject<ULineBatchComponent>(TEXT("GridLines"));
	GridLines->SetupAttachment(BoundsBox);
	GridLines->bCalculateAccurateBounds = false;

	// The spec's root. Under the box, not in place of it: the box is the
	// authored sampling region and has to stay the thing the gizmo moves.
	Spec = CreateDefaultSubobject<UMjModel>(TEXT("Spec"));
	Spec->SetupAttachment(BoundsBox);
}

// -----------------------------------------------------------------------------
// Helper: Raycast
// -----------------------------------------------------------------------------

float AMjHeightfieldActor::SampleHeightAt(const FVector2D& WorldXY, const FBox& Bounds) const
{
	const float RayStartZ = Bounds.Max.Z;
	const float RayEndZ = Bounds.Min.Z;

	// Resolve whitelist once (soft ptrs → raw ptrs)
	TSet<AActor*> WhitelistSet;
	bool bUseWhitelist = false;
	for (const auto& SoftPtr : TraceWhitelist)
	{
		if (AActor* Actor = SoftPtr.Get())
		{
			WhitelistSet.Add(Actor);
			bUseWhitelist = true;
		}
	}

	FCollisionQueryParams CollisionParams;
	CollisionParams.AddIgnoredActor(this); // Always ignore self
	CollisionParams.bTraceComplex = true;  // Trace against actual mesh geometry, not simplified collision

	FHitResult Hit;
	bool bHit = GetWorld()->LineTraceSingleByChannel(
		Hit,
		FVector(WorldXY.X, WorldXY.Y, RayStartZ),
		FVector(WorldXY.X, WorldXY.Y, RayEndZ),
		ElevationTraceChannel,
		CollisionParams);

	// Iteratively trace until we hit something valid, or miss entirely
	while (bHit)
	{
		AActor* HitActor = Hit.GetActor();
		if (HitActor)
		{
			bool bIgnore = false;

			if (bUseWhitelist)
			{
				// Whitelist mode: only accept hits on whitelisted actors
				bIgnore = !WhitelistSet.Contains(HitActor);
			}
			else
			{
				// Default mode: ignore MuJoCo actors
				bIgnore = HitActor->FindComponentByClass<UMjQuickConvertComponent>() || HitActor->IsA<AMjArticulation>() || HitActor->IsA<AMjHeightfieldActor>();
			}

			if (bIgnore)
			{
				CollisionParams.AddIgnoredActor(HitActor);
				bHit = GetWorld()->LineTraceSingleByChannel(
					Hit,
					Hit.ImpactPoint - FVector(0, 0, 1.0f),
					FVector(WorldXY.X, WorldXY.Y, RayEndZ),
					ElevationTraceChannel,
					CollisionParams);
				continue;
			}
		}
		// Valid hit!
		break;
	}

	return bHit ? Hit.ImpactPoint.Z : Bounds.Min.Z;
}

// -----------------------------------------------------------------------------
// Editor Visualizer
// -----------------------------------------------------------------------------

void AMjHeightfieldActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	RebuildGridVisualizer();
}

void AMjHeightfieldActor::RebuildGridVisualizer()
{
	if (!GridLines)
		return;

	// Clear old lines
	GridLines->Flush();

	if (!bShowGrid || Resolution < 2)
		return;

	if (!GetWorld())
		return; // Required for raycasting

	// Safety cap: don't draw an enormous number of lines and tank editor perf
	const int32 DrawResolution = FMath::Min(Resolution, 128);

	// Get world-space bounds from the box component
	FBoxSphereBounds BoxBounds = BoundsBox->CalcBounds(BoundsBox->GetComponentTransform());
	FBox Bounds = BoxBounds.GetBox();

	const float StepX = (Bounds.Max.X - Bounds.Min.X) / FMath::Max(DrawResolution - 1, 1);
	const float StepY = (Bounds.Max.Y - Bounds.Min.Y) / FMath::Max(DrawResolution - 1, 1);

	const FColor LineColorSRGB = GridColor.ToFColorSRGB();

	// 1. Pre-sample the heights into a 2D array
	TArray<FVector> Points;
	Points.SetNum(DrawResolution * DrawResolution);

	for (int32 Row = 0; Row < DrawResolution; ++Row)
	{
		const float Y = Bounds.Min.Y + Row * StepY;
		for (int32 Col = 0; Col < DrawResolution; ++Col)
		{
			const float X = Bounds.Min.X + Col * StepX;
			const float Z = SampleHeightAt(FVector2D(X, Y), Bounds);
			Points[Row * DrawResolution + Col] = FVector(X, Y, Z);
		}
	}

	// 2. Draw lines connecting adjacent points to form a projected grid
	for (int32 Row = 0; Row < DrawResolution; ++Row)
	{
		for (int32 Col = 0; Col < DrawResolution; ++Col)
		{
			const int32 CurrIdx = Row * DrawResolution + Col;
			const FVector& CurrPt = Points[CurrIdx];

			// Draw line to the right neighbour
			if (Col + 1 < DrawResolution)
			{
				const FVector& RightPt = Points[Row * DrawResolution + (Col + 1)];
				GridLines->DrawLine(CurrPt, RightPt, LineColorSRGB, 0, 1.0f, -1.0f);
			}

			// Draw line to the bottom neighbour
			if (Row + 1 < DrawResolution)
			{
				const FVector& DownPt = Points[(Row + 1) * DrawResolution + Col];
				GridLines->DrawLine(CurrPt, DownPt, LineColorSRGB, 0, 1.0f, -1.0f);
			}
		}
	}

	// If the resolution was capped, draw a warning cross in the centre
	if (Resolution > 128)
	{
		float Z = Bounds.Max.Z + 10.f;
		FVector Centre(
			(Bounds.Min.X + Bounds.Max.X) * 0.5f,
			(Bounds.Min.Y + Bounds.Max.Y) * 0.5f,
			Z);
		GridLines->DrawLine(Centre + FVector(-200, 0, 0), Centre + FVector(200, 0, 0), FColor::Red, 0, 4.0f, -1.0f);
		GridLines->DrawLine(Centre + FVector(0, -200, 0), Centre + FVector(0, 200, 0), FColor::Red, 0, 4.0f, -1.0f);
	}
}

// -----------------------------------------------------------------------------
// Sampling
// -----------------------------------------------------------------------------

bool AMjHeightfieldActor::SampleElevation(TArray<float>& OutNormHeights, float& OutMinHeightCm,
	float& OutRangeCm) const
{
	OutNormHeights.Reset();
	OutMinHeightCm = 0.0f;
	OutRangeCm = 1.0f;

	if (!GetWorld())
	{
		UE_LOG(LogURLabGenerator, Warning, TEXT("[MjHeightfieldActor] '%s': no world to sample against."), *GetName());
		return false;
	}

	const FBox Bounds = BoundsBox->CalcBounds(BoundsBox->GetComponentTransform()).GetBox();
	const float BoundsWidth = Bounds.Max.X - Bounds.Min.X;  // UE X (cm)
	const float BoundsHeight = Bounds.Max.Y - Bounds.Min.Y; // UE Y (cm)

	if (BoundsWidth <= 0.f || BoundsHeight <= 0.f)
	{
		UE_LOG(LogURLabGenerator, Warning,
			TEXT("[MjHeightfieldActor] '%s': Bounding box has zero or negative size — skipping."), *GetName());
		return false;
	}

	const int32 NRows = Resolution;
	const int32 NCols = Resolution;
	const FString CacheKey = ComputeCacheKey();

	if (!bForceRecache)
	{
		FBox CachedBounds;
		if (LoadCache(OutNormHeights, OutMinHeightCm, OutRangeCm, CachedBounds, CacheKey))
		{
			UE_LOG(LogURLabGenerator, Log, TEXT("[MjHeightfieldActor] '%s': Using cached heightfield (%dx%d)."),
				*GetName(), NCols, NRows);
			return true;
		}
	}

	const float StepX = BoundsWidth / FMath::Max(NCols - 1, 1);
	const float StepY = BoundsHeight / FMath::Max(NRows - 1, 1);

	TArray<float> RawHeights;
	RawHeights.Reserve(NRows * NCols);

	float MaxH = -FLT_MAX;
	float MinH = FLT_MAX;

	// Row 0 is minimum Unreal Y. MJCF's `elevation` is read top-to-bottom and
	// copied into storage in reverse row order, and storage row 0 is minimum
	// MuJoCo Y, which is maximum Unreal Y -- so the attribute's first row is the
	// one at minimum Unreal Y and the loop runs forwards.
	for (int32 Row = 0; Row < NRows; ++Row)
	{
		for (int32 Col = 0; Col < NCols; ++Col)
		{
			const float WorldX = Bounds.Min.X + Col * StepX;
			const float WorldY = Bounds.Min.Y + Row * StepY;

			const float H = SampleHeightAt(FVector2D(WorldX, WorldY), Bounds);

			MaxH = FMath::Max(MaxH, H);
			MinH = FMath::Min(MinH, H);
			RawHeights.Add(H);
		}
	}

	UE_LOG(LogURLabGenerator, Log, TEXT("[MjHeightfieldActor] '%s': Sampled %dx%d grid. MinH=%.1f MaxH=%.1f (UE cm)"),
		*GetName(), NCols, NRows, MinH, MaxH);

	// MuJoCo rejects a heightfield whose elevation extent is not positive, and a
	// perfectly flat sample set is a legitimate result, so the floor is a unit
	// rather than a diagnostic.
	OutMinHeightCm = MinH;
	OutRangeCm = FMath::Max(MaxH - MinH, 1.0f);

	OutNormHeights.Reserve(RawHeights.Num());
	for (float H : RawHeights)
	{
		OutNormHeights.Add((H - OutMinHeightCm) / OutRangeCm);
	}

	SaveCache(OutNormHeights, OutMinHeightCm, OutRangeCm, Bounds, CacheKey);
	return true;
}

FSpecRef AMjHeightfieldActor::GetSceneSpec() const
{
	// Nothing sampled means nothing to attach. An empty participant would still
	// compile; it would just put an `<attach>` of an empty model in the scene for
	// a heightfield that has no data, which reads as a working terrain.
	if (HfieldElement == nullptr)
	{
		return FSpecRef();
	}
	return FSpecRef::OverActor(const_cast<AMjHeightfieldActor&>(*this));
}

FString AMjHeightfieldActor::GetScenePrefix() const
{
	return GetName() + TEXT("_");
}

void AMjHeightfieldActor::AuthorSceneSpec()
{
	HfieldElement = nullptr;
	HfieldGeomElement = nullptr;

#if URLAB_MJ_GEN
	if (Spec == nullptr)
	{
		return;
	}
	MjDestroySpecChildren(*Spec);

	TArray<float> NormHeights;
	float MinHeightCm = 0.0f;
	float RangeCm = 1.0f;
	if (!SampleElevation(NormHeights, MinHeightCm, RangeCm) || NormHeights.Num() != Resolution * Resolution)
	{
		return;
	}
	bForceRecache = false;

	const FBox Bounds = BoundsBox->CalcBounds(BoundsBox->GetComponentTransform()).GetBox();
	const double BoundsWidth = Bounds.Max.X - Bounds.Min.X;
	const double BoundsHeight = Bounds.Max.Y - Bounds.Min.Y;

	urlab::spec::FMjInstanceScope Scope(*this);

	UMjAsset& Asset = urlab::spec::FInstanceNodeFactory::Create<UMjAsset>(*Spec);
	UMjHfield& Hfield = urlab::spec::FInstanceNodeFactory::Create<UMjHfield>(Asset);
	Hfield.MjName = HFieldName;
	Hfield.Nrow = Resolution;
	Hfield.Ncol = Resolution;

	// size = { half x, half y, elevation range, base thickness }, all metres and
	// all required to be positive. The elevation the compiler stores is
	// normalised to [0,1], so this third entry is what gives it a scale.
	const double HalfXMetres = BoundsWidth / 200.0;
	const double HalfYMetres = BoundsHeight / 200.0;
	const double RangeMetres = RangeCm / 100.0;
	Hfield.Size = {HalfXMetres, HalfYMetres, RangeMetres, RangeMetres * BaseThickness};

	TArray<double> Elevation;
	Elevation.Reserve(NormHeights.Num());
	for (float Height : NormHeights)
	{
		Elevation.Add(static_cast<double>(Height));
	}
	Hfield.Elevation = MoveTemp(Elevation);

	UMjBodyBase& WorldBody = urlab::spec::FInstanceNodeFactory::Create<UMjBodyBase>(*Spec);
	UMjGeomBase& Geom = urlab::spec::FInstanceNodeFactory::Create<UMjGeomBase>(WorldBody);
	Geom.MjName = HFieldName + TEXT("_geom");
	Geom.Type = EMjGeomType::hfield;
	Geom.Hfield = HFieldName;

	// The samples were taken in world space, so the geom carries world
	// coordinates and the participant attaches at identity. The field's origin
	// is the centre of the sampled region at the lowest height measured, which
	// is where the normalised elevation reads zero.
	Geom.Pos = FMjPosition3::FromUnreal(FVector(
		(Bounds.Min.X + Bounds.Max.X) * 0.5, (Bounds.Min.Y + Bounds.Max.Y) * 0.5, MinHeightCm));

	HfieldElement = &Hfield;
	HfieldGeomElement = &Geom;

	UE_LOG(LogURLabGenerator, Log,
		TEXT("[MjHeightfieldActor] '%s': authored hfield '%s' %dx%d size=[%.3f %.3f %.3f %.3f]"), *GetName(),
		*HFieldName, Resolution, Resolution, HalfXMetres, HalfYMetres, RangeMetres, RangeMetres * BaseThickness);
#endif
}

// -----------------------------------------------------------------------------
// Heightfield Cache
// -----------------------------------------------------------------------------

FString AMjHeightfieldActor::GetCacheFilePath() const
{
	return FPaths::ConvertRelativePathToFull(
		FString::Printf(TEXT("%s/URLab/HeightfieldCache/%s_%s.hfcache"),
			*FPaths::ProjectSavedDir(), *HFieldName, *GetName()));
}

FString AMjHeightfieldActor::ComputeCacheKey() const
{
	FBoxSphereBounds BoxBounds = BoundsBox->CalcBounds(BoundsBox->GetComponentTransform());
	FBox Bounds = BoxBounds.GetBox();

	return FString::Printf(TEXT("res=%d pos=%.1f,%.1f,%.1f,%.1f,%.1f,%.1f base=%.3f"),
		Resolution,
		Bounds.Min.X, Bounds.Min.Y, Bounds.Min.Z,
		Bounds.Max.X, Bounds.Max.Y, Bounds.Max.Z,
		BaseThickness);
}

bool AMjHeightfieldActor::SaveCache(const TArray<float>& NormHeights, float MinH, float ElevRange,
	const FBox& Bounds, const FString& CacheKey) const
{
	FString Path = GetCacheFilePath();
	FString Dir = FPaths::GetPath(Path);
	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	PlatformFile.CreateDirectoryTree(*Dir);

	FBufferArchive Archive;
	// Header. Version 2 is the row order MJCF's `elevation` attribute wants;
	// version 1 held the reverse, so an old cache is rejected rather than read
	// upside down.
	int32 Version = 2;
	int32 NRows = Resolution;
	int32 NCols = Resolution;
	Archive << Version << NRows << NCols;
	Archive << MinH << ElevRange;

	// Cache key for invalidation
	FString Key = CacheKey;
	Archive << Key;

	// Bounds
	FVector BMin = Bounds.Min;
	FVector BMax = Bounds.Max;
	Archive << BMin << BMax;

	// Height data
	int32 Count = NormHeights.Num();
	Archive << Count;
	Archive.Serialize(const_cast<float*>(NormHeights.GetData()), Count * sizeof(float));

	bool bOk = FFileHelper::SaveArrayToFile(Archive, *Path);
	if (bOk)
	{
		UE_LOG(LogURLabGenerator, Log, TEXT("[MjHeightfieldActor] Saved cache: %s (%d heights)"), *Path, Count);
	}
	return bOk;
}

bool AMjHeightfieldActor::LoadCache(TArray<float>& OutNormHeights, float& OutMinH, float& OutElevRange,
	FBox& OutBounds, const FString& ExpectedCacheKey) const
{
	FString Path = GetCacheFilePath();
	if (!FPaths::FileExists(Path))
		return false;

	TArray<uint8> RawData;
	if (!FFileHelper::LoadFileToArray(RawData, *Path))
		return false;

	FMemoryReader Archive(RawData, true);

	int32 Version, NRows, NCols;
	Archive << Version << NRows << NCols;
	if (Version != 2 || NRows != Resolution || NCols != Resolution)
		return false;

	Archive << OutMinH << OutElevRange;

	FString StoredKey;
	Archive << StoredKey;
	if (StoredKey != ExpectedCacheKey)
		return false;

	FVector BMin, BMax;
	Archive << BMin << BMax;
	OutBounds = FBox(BMin, BMax);

	int32 Count;
	Archive << Count;
	if (Count != NRows * NCols)
		return false;

	OutNormHeights.SetNumUninitialized(Count);
	Archive.Serialize(OutNormHeights.GetData(), Count * sizeof(float));

	UE_LOG(LogURLabGenerator, Log, TEXT("[MjHeightfieldActor] Loaded cache: %s (%d heights)"), *Path, Count);
	return true;
}
