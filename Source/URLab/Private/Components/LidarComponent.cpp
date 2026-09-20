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

#include "Components/LidarComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "DrawDebugHelpers.h"

ULidarComponent::ULidarComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
}

void ULidarComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// An adapter with its own ray source produces scans and calls
	// PublishRayResults itself; the built-in tick scan stays off.
	if (HasExternalScanSource())
	{
		return;
	}

	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	// Rate-limit on the scan clock (not necessarily wall time), so adapters
	// gating on simulation time honour sim-speed changes and pauses for free.
	const float Hz = ScanFrequencyHz;
	if (Hz > 0.0f)
	{
		const double Now = GetScanTimeSeconds();
		if (Now + 1e-9 < m_NextScanTime)
		{
			return;
		}
		// Clock jumped backwards (e.g. simulation reset): resync.
		if (m_NextScanTime > Now + 1.0)
		{
			m_NextScanTime = Now;
		}
		m_NextScanTime = FMath::Max(m_NextScanTime + 1.0 / (double)Hz, Now);
	}

	RunScan();
}

double ULidarComponent::GetScanTimeSeconds()
{
	return GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0;
}

void ULidarComponent::RunScan()
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	// Snapshot the configuration first so one scan is internally consistent
	// even if the game thread edits properties concurrently.
	const int32 AzN = FMath::Clamp(AzimuthBeams, 1, 4096);
	const int32 ElN = FMath::Clamp(ElevationBeams, 1, 4096);
	const int64 NumRays64 = static_cast<int64>(AzN) * ElN;
	if (NumRays64 <= 0 || NumRays64 > 16 * 1024 * 1024)
	{
		return;
	}
	const float MinR = FMath::Max(MinRange, 0.0f);
	const float MaxR = FMath::Max(MaxRange, MinR + 0.01f);

	const FTransform Pose = GetComponentTransform();
	const double SimTime = GetScanTimeSeconds();

	// Beam angles (+ angular noise).
	BuildBeamAngles(AzN, AzimuthFovStart, AzimuthFovEnd, ElN, ElevationFovStart, ElevationFovEnd, m_ScratchAz, m_ScratchEl);
	const bool bNoise = bEnableNoise;
	const float AzNoise = bNoise ? FMath::Max(AzimuthNoiseStdDev, 0.0f) : 0.0f;
	const float ElNoise = bNoise ? FMath::Max(ElevationNoiseStdDev, 0.0f) : 0.0f;
	const float RangeNoise = bNoise ? FMath::Max(RangeNoiseStdDev, 0.0f) : 0.0f;
	if (AzNoise > 0.0f)
	{
		for (int32 a = 0; a < AzN; ++a)
		{
			m_ScratchAz[a] += SampleGaussian(m_NoiseStream, AzNoise);
		}
	}
	if (ElNoise > 0.0f)
	{
		for (int32 e = 0; e < ElN; ++e)
		{
			m_ScratchEl[e] += SampleGaussian(m_NoiseStream, ElNoise);
		}
	}

	FCollisionQueryParams TraceParams(FName(TEXT("LidarComponentScan")), /*bTraceComplex=*/ false);
	if (bIgnoreOwner)
	{
		if (AActor* OwnerToIgnore = GetOwner())
		{
			TraceParams.AddIgnoredActor(OwnerToIgnore);
		}
	}

	TArray<float> RangesM;
	TArray<int32> SurfaceIds;
	RangesM.Init(-1.0f, static_cast<int32>(NumRays64));
	SurfaceIds.Init(-1, static_cast<int32>(NumRays64));

	const float MaxRangeCm = MaxR * 100.0f;
	const FVector Origin = Pose.GetLocation();
	int32 Idx = 0;
	for (int32 e = 0; e < ElN; ++e)
	{
		for (int32 a = 0; a < AzN; ++a)
		{
			const FVector DirWorld = Pose.GetRotation().RotateVector(SphericalDirection(m_ScratchAz[a], m_ScratchEl[e]));
			const FVector End = Origin + DirWorld * MaxRangeCm;

			FHitResult Hit;
			const bool bHit = World->LineTraceSingleByChannel(Hit, Origin, End, TraceChannel, TraceParams);

			// Post-filter: range noise, then the blind zone. The blind-zone
			// check runs on the noised return so the published value honours
			// MinRange's contract ("returns closer than this are discarded").
			if (bHit)
			{
				float RangeM = Hit.Distance * 0.01f;
				if (RangeNoise > 0.0f)
				{
					RangeM = FMath::Max(RangeM + SampleGaussian(m_NoiseStream, RangeNoise), 0.0f);
				}
				if (RangeM >= MinR)
				{
					RangesM[Idx] = RangeM;
					SurfaceIds[Idx] = SurfaceIdFromComponent(Hit.GetComponent());
				}
			}
			++Idx;
		}
	}

	PublishRayResults(Pose, m_ScratchAz, m_ScratchEl, RangesM, SurfaceIds, AzN, SimTime, ++m_ScanCounter);
}

void ULidarComponent::PublishRayResults(const FTransform& SensorPose,
	const TArray<float>& Azimuths, const TArray<float>& Elevations,
	const TArray<float>& RangesM, const TArray<int32>& SurfaceIds,
	int32 AzCount, double SimTime, int64 ScanId)
{
	const FQuat SensorRot = SensorPose.GetRotation();
	const FVector Origin = SensorPose.GetLocation();

	const int32 NumRays = RangesM.Num();
	const int32 ClampedAz = FMath::Max(AzCount, 1);
	const bool bWantPoints = OutputMode == ELidarOutputMode::Points || OutputMode == ELidarOutputMode::Both;
	const bool bWantTargets = OutputMode == ELidarOutputMode::Targets || OutputMode == ELidarOutputMode::Both;

	// HitPositionsCm only feeds the debug draw and the target aggregator; the
	// common Points-only path (no debug switches) skips the array entirely.
	const bool bNeedHitPositions = bWantTargets || bDrawDebugPoints || bDrawDebugLines;
	TArray<FLidarPoint> Points;
	TArray<FVector> HitPositionsCm;
	Points.Reserve(bWantPoints ? NumRays : 0);
	HitPositionsCm.Reserve(bNeedHitPositions ? NumRays : 0);

	for (int32 i = 0; i < NumRays; ++i)
	{
		const float R = RangesM[i];
		if (R < 0.0f)
		{
			continue; // miss (or blind zone)
		}
		const int32 a = i % ClampedAz;
		const int32 e = i / ClampedAz;
		const float Az = Azimuths.IsValidIndex(a) ? Azimuths[a] : 0.0f;
		const float El = Elevations.IsValidIndex(e) ? Elevations[e] : 0.0f;

		// Hit point from origin + direction * range keeps WorldPos and
		// RangeM exactly coherent.
		const FVector DirWorld = SensorRot.RotateVector(SphericalDirection(Az, El));
		const FVector HitPosCm = Origin + DirWorld * (R * 100.0f);
		if (bNeedHitPositions)
		{
			HitPositionsCm.Add(HitPosCm);
		}

		if (bWantPoints)
		{
			FLidarPoint& P = Points.AddDefaulted_GetRef();
			P.WorldPos = HitPosCm;
			P.AzimuthDeg = Az;
			P.ElevationDeg = El;
			P.RangeM = R;
			P.HitSurfaceId = SurfaceIds.IsValidIndex(i) ? SurfaceIds[i] : -1;
			P.FrameId = ScanId;
		}
	}

	// Debug drawing (game thread, world coordinates).
	if (bDrawDebugPoints || bDrawDebugLines)
	{
		UWorld* World = GetWorld();
		if (World)
		{
			const float Lifetime = FMath::Clamp(ScanFrequencyHz > 0.0f ? 1.0f / ScanFrequencyHz : 0.05f, 0.05f, 0.5f);
			for (int32 h = 0; h < HitPositionsCm.Num(); ++h)
			{
				if (bDrawDebugPoints)
				{
					DrawDebugPoint(World, HitPositionsCm[h], 4.0f, FColor::Green, false, Lifetime);
				}
				if (bDrawDebugLines)
				{
					DrawDebugLine(World, Origin, HitPositionsCm[h], FColor::Silver, false, Lifetime, 0, 0.15f);
				}
			}
		}
	}

	LastPoints = MoveTemp(Points);
	TArray<FLidarTarget> Targets;
	if (bWantTargets)
	{
		BuildTargets(HitPositionsCm, SimTime, Origin, Targets);
	}
	LastTargets = MoveTemp(Targets);

	m_LastScanId = ScanId;
	m_LastScanSimTime = SimTime;

	// Publish the completed scan.
	FLidarScan Scan;
	Scan.Points = LastPoints;
	Scan.Targets = LastTargets;
	Scan.ScanId = ScanId;
	Scan.SimTime = SimTime;
	OnLidarScan.Broadcast(Scan);
}

void ULidarComponent::BuildTargets(const TArray<FVector>& HitPositionsCm, double SimTime, const FVector& SensorPosUe, TArray<FLidarTarget>& OutTargets)
{
	OutTargets.Reset();

	TArray<int32> ClusterIds;
	TArray<int32> ClusterSizes;
	ClusterPoints(HitPositionsCm, ClusterDistanceThreshold * 100.0f, ClusterIds, ClusterSizes);

	const int32 NumClusters = ClusterSizes.Num();
	if (NumClusters == 0)
	{
		return;
	}

	// Centroids.
	TArray<FVector> Centroids;
	Centroids.SetNum(NumClusters);
	{
		TArray<int32> Counts;
		Counts.SetNum(NumClusters);
		for (int32 i = 0; i < Centroids.Num(); ++i)
		{
			Centroids[i] = FVector::ZeroVector;
			Counts[i] = 0;
		}
		for (int32 i = 0; i < HitPositionsCm.Num(); ++i)
		{
			const int32 C = ClusterIds[i];
			if (C >= 0 && C < NumClusters)
			{
				Centroids[C] += HitPositionsCm[i];
				++Counts[C];
			}
		}
		for (int32 c = 0; c < NumClusters; ++c)
		{
			if (Counts[c] > 0)
			{
				Centroids[c] /= (float)Counts[c];
			}
		}
	}

	// Match clusters to existing tracks (greedy nearest within the gate).
	// Relative positions use the caller's SensorPosUe (already computed in
	// RunScan from the same scan origin).
	const float GateCm = FMath::Max(ClusterDistanceThreshold * 300.0f, 1.0f);
	const int32 TrackCount = m_Tracks.Num();
	TArray<int32> TrackOfCluster;
	TrackOfCluster.Init(-1, NumClusters);
	TArray<bool> TrackTaken;
	TrackTaken.Init(false, TrackCount);

	for (int32 Pass = 0; Pass < 2 && TrackCount > 0; ++Pass)
	{
		// Pass 0: pick best (closest) track per cluster; Pass 1: allow the
		// remaining clusters to claim any still-free track (first fit).
		for (int32 c = 0; c < NumClusters; ++c)
		{
			if (TrackOfCluster[c] >= 0)
			{
				continue;
			}
			int32 Best = -1;
			float BestDist = FLT_MAX;
			for (int32 t = 0; t < TrackCount; ++t)
			{
				if (TrackTaken[t])
				{
					continue;
				}
				const float Dist = FVector::Dist(Centroids[c], m_Tracks[t].Centroid);
				if (Dist <= GateCm && Dist < BestDist)
				{
					Best = t;
					BestDist = Dist;
				}
			}
			if (Best >= 0)
			{
				TrackOfCluster[c] = Best;
				TrackTaken[Best] = true;
			}
		}
	}

	const int32 HistCap = FMath::Max(HistorySize, 2);
	for (int32 c = 0; c < NumClusters; ++c)
	{
		int32 TrackIndex = TrackOfCluster[c];
		if (TrackIndex < 0)
		{
			// New track.
			FTargetTrack NewTrack;
			NewTrack.Id = m_NextTargetId++;
			NewTrack.Centroid = Centroids[c];
			NewTrack.SimTime = SimTime;
			NewTrack.Misses = 0;
			NewTrack.HistoryPos.Add(Centroids[c]);
			NewTrack.HistoryTime.Add(SimTime);
			TrackIndex = m_Tracks.Add(MoveTemp(NewTrack));
		}
		else
		{
			FTargetTrack& Track = m_Tracks[TrackIndex];
			Track.Centroid = Centroids[c];
			Track.SimTime = SimTime;
			Track.Misses = 0;
			Track.HistoryPos.Add(Centroids[c]);
			Track.HistoryTime.Add(SimTime);
			while (Track.HistoryPos.Num() > HistCap)
			{
				Track.HistoryPos.RemoveAt(0);
				Track.HistoryTime.RemoveAt(0);
			}
		}

		const FTargetTrack& Track = m_Tracks[TrackIndex];

		FLidarTarget& T = OutTargets.AddDefaulted_GetRef();
		T.TargetId = Track.Id;
		T.NumPoints = ClusterSizes[c];
		T.CentroidWorldPos = Track.Centroid;
		T.RelativePos = Track.Centroid - SensorPosUe;

		if (bComputeVelocity && Track.HistoryPos.Num() >= 2)
		{
			const int32 Last = Track.HistoryPos.Num() - 1;
			const double Dt = Track.HistoryTime[Last] - Track.HistoryTime[Last - 1];
			if (Dt > 1e-6)
			{
				// Kinematic approximation: centroid differencing, NOT Doppler.
				const FVector VelCmS = (Track.HistoryPos[Last] - Track.HistoryPos[Last - 1]) / (float)Dt;
				const FVector LoS = T.RelativePos.GetSafeNormal();
				T.RadialSpeed = (float)(FVector::DotProduct(VelCmS, LoS) / 100.0); // cm/s -> m/s
			}
		}
	}

	// Age and prune unmatched tracks. TrackTaken covers the tracks that
	// existed at match time; tracks appended during this scan (indices >=
	// TrackCount) were just matched by construction.
	for (int32 t = m_Tracks.Num() - 1; t >= 0; --t)
	{
		const bool bMatchedThisScan = (t < TrackCount) && TrackTaken[t];
		if (!bMatchedThisScan)
		{
			FTargetTrack& Track = m_Tracks[t];
			++Track.Misses;
			if (Track.Misses > HistCap)
			{
				m_Tracks.RemoveAt(t);
			}
		}
	}
}

int32 ULidarComponent::SurfaceIdFromComponent(const UPrimitiveComponent* Component)
{
	if (!Component)
	{
		return -1;
	}
	// Name hash: stable across sessions (unlike pointers), deterministic for
	// the viz's palette coloring, and non-negative so -1 stays a miss-only
	// sentinel.
	const uint32 Hash = GetTypeHash(Component->GetFName());
	return static_cast<int32>(Hash & 0x7FFFFFFEu);
}

FLidarScan ULidarComponent::GetLastScan() const
{
	FLidarScan Scan;
	Scan.Points = LastPoints;
	Scan.Targets = LastTargets;
	Scan.ScanId = m_LastScanId;
	Scan.SimTime = m_LastScanSimTime;
	return Scan;
}

// ---------------------------------------------------------------------------
// Pure logic helpers (static; exercised by URLabEditor automation tests)
// ---------------------------------------------------------------------------

void ULidarComponent::BuildBeamAngles(int32 AzBeams, float AzStart, float AzEnd,
	int32 ElBeams, float ElStart, float ElEnd,
	TArray<float>& OutAzimuths, TArray<float>& OutElevations)
{
	const int32 AzN = FMath::Max(AzBeams, 1);
	const int32 ElN = FMath::Max(ElBeams, 1);
	OutAzimuths.SetNum(AzN);
	OutElevations.SetNum(ElN);

	const float AzSpan = AzEnd - AzStart;
	// A full circle wraps onto itself, so it must be endpoint-exclusive;
	// partial spans include both endpoints.
	const bool bPeriodicAz = FMath::Abs(AzSpan) >= 359.99f;
	for (int32 a = 0; a < AzN; ++a)
	{
		if (AzN == 1)
		{
			OutAzimuths[a] = 0.5f * (AzStart + AzEnd);
		}
		else if (bPeriodicAz)
		{
			OutAzimuths[a] = AzStart + AzSpan * ((float)a / (float)AzN);
		}
		else
		{
			OutAzimuths[a] = AzStart + AzSpan * ((float)a / (float)(AzN - 1));
		}
	}

	const float ElSpan = ElEnd - ElStart;
	for (int32 e = 0; e < ElN; ++e)
	{
		if (ElN == 1)
		{
			OutElevations[e] = 0.5f * (ElStart + ElEnd);
		}
		else
		{
			OutElevations[e] = ElStart + ElSpan * ((float)e / (float)(ElN - 1));
		}
	}
}

FVector ULidarComponent::SphericalDirection(float AzimuthDeg, float ElevationDeg)
{
	const float Az = FMath::DegreesToRadians(AzimuthDeg);
	const float El = FMath::DegreesToRadians(ElevationDeg);
	const float Ce = FMath::Cos(El);
	return FVector(Ce * FMath::Cos(Az), Ce * FMath::Sin(Az), FMath::Sin(El));
}

void ULidarComponent::ClusterPoints(const TArray<FVector>& Points, float ThresholdCm,
	TArray<int32>& OutClusterIds, TArray<int32>& OutClusterSizes)
{
	const int32 N = Points.Num();
	OutClusterIds.Reset();
	OutClusterSizes.Reset();
	if (N == 0)
	{
		return;
	}

	if (ThresholdCm <= 0.0f)
	{
		// Degenerate threshold: every point is its own cluster.
		OutClusterIds.SetNum(N);
		OutClusterSizes.Init(1, N);
		for (int32 i = 0; i < N; ++i)
		{
			OutClusterIds[i] = i;
		}
		return;
	}

	// Union-find with path compression.
	TArray<int32> Parent;
	Parent.SetNum(N);
	for (int32 i = 0; i < N; ++i)
	{
		Parent[i] = i;
	}
	auto Find = [&Parent](int32 X) {
		while (Parent[X] != X)
		{
			Parent[X] = Parent[Parent[X]];
			X = Parent[X];
		}
		return X;
	};
	auto Union = [&Parent, &Find](int32 A, int32 B) {
		const int32 Ra = Find(A);
		const int32 Rb = Find(B);
		if (Ra != Rb)
		{
			Parent[Rb] = Ra;
		}
	};

	// Spatial hash: cell edge = threshold, so any within-threshold pair must
	// land in the 3x3x3 neighbourhood.
	const float Cell = ThresholdCm;
	auto CellKey = [&Cell](const FVector& P)
	{
		return FIntVector(
			FMath::FloorToInt(P.X / Cell),
			FMath::FloorToInt(P.Y / Cell),
			FMath::FloorToInt(P.Z / Cell));
	};
	TMap<FIntVector, TArray<int32>> Grid;
	Grid.Reserve(N);
	for (int32 i = 0; i < N; ++i)
	{
		Grid.FindOrAdd(CellKey(Points[i])).Add(i);
	}

	const float ThrSq = ThresholdCm * ThresholdCm;
	for (int32 i = 0; i < N; ++i)
	{
		const FVector& P = Points[i];
		const FIntVector Key = CellKey(P);
		for (int32 Dx = -1; Dx <= 1; ++Dx)
		{
			for (int32 Dy = -1; Dy <= 1; ++Dy)
			{
				for (int32 Dz = -1; Dz <= 1; ++Dz)
				{
					const TArray<int32>* Bucket = Grid.Find(Key + FIntVector(Dx, Dy, Dz));
					if (!Bucket)
					{
						continue;
					}
					for (const int32 J : *Bucket)
					{
						if (J == i)
						{
							continue;
						}
						if (FVector::DistSquared(P, Points[J]) <= ThrSq)
						{
							Union(i, J);
						}
					}
				}
			}
		}
	}

	// Compact roots into cluster indices.
	OutClusterIds.SetNum(N);
	TMap<int32, int32> RootToCluster;
	RootToCluster.Reserve(N);
	int32 NumClusters = 0;
	for (int32 i = 0; i < N; ++i)
	{
		const int32 Root = Find(i);
		int32 Cluster = RootToCluster.FindRef(Root, -1);
		if (Cluster < 0)
		{
			Cluster = NumClusters++;
			RootToCluster.Add(Root, Cluster);
		}
		OutClusterIds[i] = Cluster;
	}

	OutClusterSizes.Init(0, NumClusters);
	for (int32 i = 0; i < N; ++i)
	{
		++OutClusterSizes[OutClusterIds[i]];
	}
}

float ULidarComponent::SampleGaussian(FRandomStream& Stream, float StdDev)
{
	if (StdDev <= 0.0f)
	{
		return 0.0f;
	}
	// Box-Muller. FRandomStream::FRand() is [0, 1).
	float U1 = Stream.FRand();
	const float U2 = Stream.FRand();
	U1 = FMath::Max(U1, 1e-6f);
	return StdDev * FMath::Sqrt(-2.0f * FMath::Loge(U1)) * FMath::Cos(2.0f * PI * U2);
}
