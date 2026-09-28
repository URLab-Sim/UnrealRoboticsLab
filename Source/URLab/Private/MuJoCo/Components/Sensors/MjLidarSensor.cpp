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

#include "MuJoCo/Components/Sensors/MjLidarSensor.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Components/Geometry/MjSite.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "Utils/URLabLogging.h"
#include "HAL/PlatformTime.h"

UMjLidarSensor::UMjLidarSensor()
{
}

void UMjLidarSensor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	m_bStopped.store(true, std::memory_order_release);

	// Take the callback mutex briefly so any in-flight physics callback is
	// finished (and none can start) when we return from EndPlay.
	if (AAMjManager* Manager = AAMjManager::GetManager())
	{
		if (Manager->PhysicsEngine)
		{
			FScopeLock Lock(&Manager->PhysicsEngine->CallbackMutex);
		}
	}

	SetComponentTickEnabled(false);
	Super::EndPlay(EndPlayReason);
}

void UMjLidarSensor::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	AAMjManager* Manager = AAMjManager::GetManager();
	if (!Manager || !Manager->PhysicsEngine)
	{
		return;
	}

	// 1. Resolve (and re-poll) the mount once per tick; republishes under
	//    CallbackMutex whenever the mount component's bound id changes.
	ResolveMount();

	// 2. Register the physics-thread callback once (weak guard: it becomes a
	//    no-op once this component is gone, matching the manager's own pattern).
	if (!m_bCallbackRegistered)
	{
		TWeakObjectPtr<UMjLidarSensor> WeakThis(this);
		Manager->PhysicsEngine->RegisterPostStepCallback(
			[WeakThis](mjModel* m, mjData* d) {
				UMjLidarSensor* Self = WeakThis.Get();
				if (!Self || Self->m_bStopped.load(std::memory_order_acquire))
				{
					return;
				}
				Self->PhysicsPostStep(m, d);
			});
		m_bCallbackRegistered = true;
	}

	// 3. Drain the newest scan into the core's shared sink.
	DrainScan();
}

void UMjLidarSensor::ResolveMount()
{
	// Walk the attachment chain to the nearest MuJoCo pose source.
	USceneComponent* MountComponent = nullptr;
	{
		USceneComponent* Parent = GetAttachParent();
		while (Parent)
		{
			if (Parent->IsA<UMjSite>() || Parent->IsA<UMjBody>())
			{
				MountComponent = Parent;
				break;
			}
			Parent = Parent->GetAttachParent();
		}
	}

	UMjComponent* MountMjComp = Cast<UMjComponent>(MountComponent);
	int32 Candidate = -1;
	if (MountMjComp)
	{
		Candidate = MountMjComp->GetMjID();
		if (Candidate < 0)
		{
			if (mjsElement* Elem = MountMjComp->GetSpecElementForDiagnostics())
			{
				Candidate = mjs_getId(Elem);
			}
		}
		// Bounds are validated against the live model inside the callback;
		// a stale id simply yields no scans until the next poll republishes.
	}

	// Re-poll guard: already published against this component and polled id
	// (an unbound mount keeps Candidate at -1 without republishing or
	// re-logging; the bind after a compile changes the id and republishes).
	if (m_bMountResolved && m_MountComponent.Get() == MountComponent
		&& Candidate == m_LastPolledCandidate)
	{
		return;
	}

	// Compose the mount-relative transform of this sensor, walking from this
	// component up to (but excluding) the mount component. World_child =
	// World_parent * Rel_child, so Rel_mount_sensor = Rel(...*Rel(this)) with
	// each parent's relative transform pre-multiplied.
	FTransform Rel = GetRelativeTransform();
	if (MountComponent)
	{
		USceneComponent* Ancestor = GetAttachParent();
		while (Ancestor && Ancestor != MountComponent)
		{
			Rel = Ancestor->GetRelativeTransform() * Rel;
			Ancestor = Ancestor->GetAttachParent();
		}
	}

	int32 MountType = 0;
	int32 MountId = -1;

	if (MountMjComp && Candidate >= 0)
	{
		if (Cast<UMjBody>(MountMjComp))
		{
			MountType = 1;
		}
		else
		{
			MountType = 2;
		}
		MountId = Candidate;
	}
	else if (MountMjComp)
	{
		UE_LOG(LogURLab, Warning,
			TEXT("[MjLidarSensor] '%s': mount component '%s' has no valid MuJoCo id yet; falling back to a static world mount until it binds."),
			*GetName(), *MountComponent->GetName());
	}

	if (MountType == 0)
	{
		// No usable MuJoCo ancestor: static world mount. The sensor pose is
		// captured once (worldbody never moves).
		Rel = GetComponentTransform();
		MountType = 1;
		MountId = 0; // worldbody
		UE_LOG(LogURLab, Log,
			TEXT("[MjLidarSensor] '%s': no UMjBody/UMjSite ancestor; using static world mount. Attach under a body to follow the robot."),
			*GetName());
	}

	// Convert the relative pose into MuJoCo coordinates (metres, w-first quat).
	double RelPosMj[3];
	double RelQuatMj[4];
	MjUtils::UEToMjPosition(Rel.GetLocation(), RelPosMj);
	MjUtils::UEToMjRotation(Rel.GetRotation(), RelQuatMj);

	// Publish under CallbackMutex so the physics thread (which reads m_Mount
	// while holding it) never observes a torn state.
	if (AAMjManager* Manager = AAMjManager::GetManager())
	{
		if (Manager->PhysicsEngine)
		{
			FScopeLock Lock(&Manager->PhysicsEngine->CallbackMutex);
			m_Mount.Type = MountType;
			m_Mount.Id = MountId;
			for (int32 i = 0; i < 3; ++i)
			{
				m_Mount.RelPosMj[i] = RelPosMj[i];
			}
			for (int32 i = 0; i < 4; ++i)
			{
				m_Mount.RelQuatMj[i] = RelQuatMj[i];
			}
			m_MountComponent = MountComponent;
			m_LastPolledCandidate = Candidate;
			m_bMountResolved = true;
		}
		// Manager went away mid-resolve: the poll retries next tick.
	}
}

void UMjLidarSensor::PhysicsPostStep(mjModel* m, mjData* d)
{
	if (!m || !d)
	{
		return;
	}

	// Snapshot the configuration first so one scan is internally consistent
	// even if the game thread edits properties concurrently (single-word
	// reads are atomic on our target platforms).
	const int32 AzN = FMath::Clamp(AzimuthBeams, 1, 4096);
	const int32 ElN = FMath::Clamp(ElevationBeams, 1, 4096);
	const int64 NumRays64 = static_cast<int64>(AzN) * ElN;
	if (NumRays64 <= 0 || NumRays64 > 16 * 1024 * 1024)
	{
		return;
	}
	const int32 NumRays = static_cast<int32>(NumRays64);
	const float AzStart = AzimuthFovStart;
	const float AzEnd = AzimuthFovEnd;
	const float ElStart = ElevationFovStart;
	const float ElEnd = ElevationFovEnd;
	const float MinR = FMath::Max(MinRange, 0.0f);
	const float MaxR = FMath::Max(MaxRange, MinR + 0.01f);
	const float Hz = ScanFrequencyHz;

	// Rate-limit on simulation time (not wall clock), so the sensor honours
	// sim-speed changes and pauses for free.
	if (Hz > 0.0f)
	{
		if (d->time + 1e-9 < m_NextScanSimTime)
		{
			return;
		}
		// Simulation time jumped backwards (reset): resync.
		if (m_NextScanSimTime > d->time + 1.0)
		{
			m_NextScanSimTime = d->time;
		}
		m_NextScanSimTime = FMath::Max(m_NextScanSimTime + 1.0 / (double)Hz, d->time);
	}

	// Mount must be resolved.
	if (m_Mount.Type == 0)
	{
		return;
	}

	const double* MountXpos = nullptr;
	const double* MountXmat = nullptr;
	const double* MountQuat = nullptr;
	double SiteQuatMj[4] = { 1, 0, 0, 0 };
	int32 BodyExclude = -1;
	if (m_Mount.Type == 1)
	{
		if (m_Mount.Id < 0 || m_Mount.Id >= m->nbody)
		{
			return;
		}
		MountXpos = d->xpos + 3 * m_Mount.Id;
		MountXmat = d->xmat + 9 * m_Mount.Id;
		MountQuat = d->xquat + 4 * m_Mount.Id;
		// Never exclude the worldbody (id 0): it owns the floor and other
		// static environment, which a lidar must see.
		if (bExcludeMountBody && m_Mount.Id > 0)
		{
			BodyExclude = m_Mount.Id;
		}
	}
	else
	{
		if (m_Mount.Id < 0 || m_Mount.Id >= m->nsite)
		{
			return;
		}
		MountXpos = d->site_xpos + 3 * m_Mount.Id;
		MountXmat = d->site_xmat + 9 * m_Mount.Id;
		mju_mat2Quat(SiteQuatMj, MountXmat);
		MountQuat = SiteQuatMj;
		if (bExcludeMountBody)
		{
			const int32 SiteBody = m->site_bodyid[m_Mount.Id];
			if (SiteBody > 0)
			{
				BodyExclude = SiteBody;
			}
		}
	}

	// Sensor frame in MuJoCo world coordinates: R_sensor = R_mount * R_rel,
	// p_sensor = p_mount + R_mount * p_rel.
	double RelMat[9];
	mju_quat2Mat(RelMat, m_Mount.RelQuatMj);
	double SensorMat[9];
	mju_mulMatMat(SensorMat, MountXmat, RelMat, 3, 3, 3);
	double SensorPos[3];
	double RelPosWorld[3];
	mju_mulMatVec3(RelPosWorld, MountXmat, m_Mount.RelPosMj);
	SensorPos[0] = MountXpos[0] + RelPosWorld[0];
	SensorPos[1] = MountXpos[1] + RelPosWorld[1];
	SensorPos[2] = MountXpos[2] + RelPosWorld[2];
	double SensorQuat[4];
	mju_mulQuat(SensorQuat, MountQuat, m_Mount.RelQuatMj);

	// Beam angles (+ angular noise).
	BuildBeamAngles(AzN, AzStart, AzEnd, ElN, ElStart, ElEnd, m_ScratchAz, m_ScratchEl);
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

	// Direction fan: ray index = e * AzN + a (elevation-major).
	const uint64 FanStartCycles = FPlatformTime::Cycles();
	m_ScratchVec.SetNumUninitialized(3 * NumRays);
	{
		int32 Idx = 0;
		for (int32 e = 0; e < ElN; ++e)
		{
			for (int32 a = 0; a < AzN; ++a)
			{
				const FVector Dir = SphericalDirection(m_ScratchAz[a], m_ScratchEl[e]);
				const double V[3] = { (double)Dir.X, (double)Dir.Y, (double)Dir.Z };
				mju_mulMatVec3(m_ScratchVec.GetData() + 3 * Idx, SensorMat, V);
				++Idx;
			}
		}
	}
	const uint64 FanCycles = FPlatformTime::Cycles() - FanStartCycles;

	// Geom group filter: bit g of the mask enables group g. All-groups-set is
	// equivalent to no filter at all (NULL), which skips group exclusion.
	mjtByte GroupBytes[mjNGROUP];
	bool bAllGroups = true;
	const int32 Mask = GeomGroupMask & 0x3F;
	for (int32 g = 0; g < mjNGROUP; ++g)
	{
		GroupBytes[g] = ((Mask >> g) & 1) ? 1 : 0;
		bAllGroups = bAllGroups && GroupBytes[g] != 0;
	}

	// Cast all rays from the single sensor origin against the MuJoCo model.
	m_ScratchDist.SetNumUninitialized(NumRays);
	m_ScratchGeomId.SetNumUninitialized(NumRays);
	const uint64 RayStartCycles = FPlatformTime::Cycles();
	mj_multiRay(m, d, SensorPos, m_ScratchVec.GetData(),
		bAllGroups ? nullptr : GroupBytes,
		bHitStaticGeoms ? 1 : 0,
		BodyExclude,
		m_ScratchGeomId.GetData(), m_ScratchDist.GetData(), nullptr,
		NumRays, (mjtNum)MaxR);
	const uint64 RayCycles = FPlatformTime::Cycles() - RayStartCycles;

	// Post-filter: range noise, then the blind zone. The blind-zone check
	// runs on the noised return so the published value honours MinRange's
	// contract ("returns closer than this are discarded").
	const uint64 FilterStartCycles = FPlatformTime::Cycles();
	for (int32 i = 0; i < NumRays; ++i)
	{
		double& R = m_ScratchDist[i];
		if (R >= 0.0)
		{
			if (RangeNoise > 0.0f)
			{
				R = FMath::Max(R + (double)SampleGaussian(m_NoiseStream, RangeNoise), 0.0);
			}
			if (R < (double)MinR)
			{
				R = -1.0; // inside the blind zone: treat as a miss
				m_ScratchGeomId[i] = -1;
			}
		}
		else
		{
			m_ScratchGeomId[i] = -1;
		}
	}
	const uint64 FilterCycles = FPlatformTime::Cycles() - FilterStartCycles;

	// Publish the raw scan for the game thread.
	{
		FScopeLock Lock(&m_DataMutex);
		m_SharedScan.ScanId = ++m_PhysicsScanCounter;
		m_SharedScan.SimTime = d->time;
		m_SharedScan.AzCount = AzN;
		for (int32 i = 0; i < 3; ++i)
		{
			m_SharedScan.OriginMj[i] = SensorPos[i];
		}
		for (int32 i = 0; i < 4; ++i)
		{
			m_SharedScan.SensorQuatMj[i] = SensorQuat[i];
		}
		m_SharedScan.Azimuths = m_ScratchAz;
		m_SharedScan.Elevations = m_ScratchEl;
		m_SharedScan.Ranges = m_ScratchDist;
		m_SharedScan.GeomIds = m_ScratchGeomId;
	}

	// Throttled per-scan timing, one line per sensor per ~5 s wall time (fan
	// = direction grid build, multiray = mj_multiRay against the compiled
	// model, filter = range noise + blind zone). These numbers size the
	// remaining serial costs on the physics thread.
	const double NowSec = FPlatformTime::Seconds();
	if (NowSec >= m_NextTimingLogSec)
	{
		m_NextTimingLogSec = NowSec + 5.0;
		UE_LOG(LogURLab, Log,
			TEXT("[MjLidarSensor] '%s' scan: rays=%d fan=%.3fms multiray=%.3fms filter=%.3fms"),
			*GetName(), NumRays,
			FPlatformTime::ToMilliseconds64(FanCycles),
			FPlatformTime::ToMilliseconds64(RayCycles),
			FPlatformTime::ToMilliseconds64(FilterCycles));
	}
}

void UMjLidarSensor::DrainScan()
{
	{
		FScopeLock Lock(&m_DataMutex);
		if (m_SharedScan.ScanId < 0 || m_SharedScan.ScanId == m_LastConsumedScanId)
		{
			return;
		}
		// Copy out (keeps the shared buffers' capacity stable so the physics
		// thread never re-allocates under the lock either).
		m_LocalScan = m_SharedScan;
		m_LastConsumedScanId = m_SharedScan.ScanId;
	}

	const int32 AzCount = m_LocalScan.AzCount > 0 ? m_LocalScan.AzCount : 1;
	const int32 NumRays = m_LocalScan.Ranges.Num();
	m_LocalRangesM.SetNumUninitialized(NumRays);
	for (int32 i = 0; i < NumRays; ++i)
	{
		m_LocalRangesM[i] = (float)m_LocalScan.Ranges[i];
	}

	// Handedness: the pose conversion mirrors Y (MjToUERotation negates the
	// quat's X/Z, MjToUEPosition negates Y), so the beam directions must be
	// mirrored along with it: M(R_mj * dir(az, el)) == MjToUERotation(R_mj) *
	// dir(-az, el), because mirroring SphericalDirection(az, el) is exactly
	// SphericalDirection(-az, el). Negating the stored azimuths therefore
	// makes the core's sink rebuild the same world rays that were cast.
	const int32 AzN = m_LocalScan.Azimuths.Num();
	m_LocalAzimuthsUe.SetNumUninitialized(AzN);
	for (int32 a = 0; a < AzN; ++a)
	{
		m_LocalAzimuthsUe[a] = -m_LocalScan.Azimuths[a];
	}

	// Sensor pose in Unreal world coordinates; the core's sink rebuilds the
	// hit points around it (equivalent to converting each MuJoCo hit).
	const FQuat RotUe = MjUtils::MjToUERotation(m_LocalScan.SensorQuatMj);
	const FVector PosUe = MjUtils::MjToUEPosition(m_LocalScan.OriginMj);
	const FTransform PoseUe(RotUe, PosUe);

	PublishRayResults(PoseUe, m_LocalAzimuthsUe, m_LocalScan.Elevations,
		m_LocalRangesM, m_LocalScan.GeomIds, AzCount,
		m_LocalScan.SimTime, m_LocalScan.ScanId);
}
