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

#include "State/MjStateCollector.h"
#include "State/MjCanonicalName.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Components/MjComponent.h"
#include "MuJoCo/Input/MjTwistController.h"
#include "Async/Async.h"
#include "Misc/ScopeExit.h"
#include "mujoco/mujoco.h"

void FMjStateCollector::Init(AAMjManager* InManager)
{
	Manager = InManager;
	bCacheValid.store(false, std::memory_order_release);
}

void FMjStateCollector::MarkProducerCacheDirty()
{
	bCacheValid.store(false, std::memory_order_release);
	RequestGameThreadRebuild();
}

void FMjStateCollector::RequestGameThreadRebuild()
{
	bool bExpected = false;
	if (!bRebuildScheduled.compare_exchange_strong(bExpected, true))
		return; // a rebuild is already queued

	TWeakObjectPtr<AAMjManager> WeakMgr = Manager;
	AsyncTask(ENamedThreads::GameThread, [this, WeakMgr]() {
		if (WeakMgr.IsValid())
			RebuildProducerCacheGameThread();
		else
			bRebuildScheduled.store(false, std::memory_order_release);
	});
}

void FMjStateCollector::RebuildProducerCacheGameThread()
{
	ON_SCOPE_EXIT
	{
		bRebuildScheduled.store(false, std::memory_order_release);
	};

	AAMjManager* Mgr = Manager.Get();
	if (!Mgr)
		return;

	// Assemble into a local with no lock held; the physics thread keeps reading
	// the previous cache meanwhile. Only the swap below is guarded.
	TArray<FCachedArticulation> NewCache;
	const TArray<AMjArticulation*>& Arts = Mgr->GetAllArticulations();
	NewCache.Reserve(Arts.Num());
	for (AMjArticulation* Art : Arts)
	{
		if (!Art)
			continue;

		FCachedArticulation Rec;
		Rec.Art = Art;
		Rec.ArtSegment = FMjCanonicalName::ArtSegment(Art);

		TArray<UMjComponent*> Components;
		Art->GetComponents<UMjComponent>(Components);
		Rec.Producers.Reserve(Components.Num());
		for (UMjComponent* Comp : Components)
		{
			if (!Comp || Comp->bIsDefault)
				continue;
			Rec.Producers.Add(Comp);
		}

		Rec.TwistCtrl = Art->FindComponentByClass<UMjTwistController>();
		NewCache.Add(MoveTemp(Rec));
	}

	{
		FScopeLock Lock(&CacheMutex);
		Cache = MoveTemp(NewCache);
	}
	++StructureVersion;
	bCacheValid.store(true, std::memory_order_release);
}

const FMjStateSnapshot& FMjStateCollector::Collect(mjModel* m, mjData* d, int64 StepIdx)
{
	Snapshot.Reset();
	if (!m || !d)
		return Snapshot;

	AAMjManager* Mgr = Manager.Get();

	Snapshot.Time = d->time;
	Snapshot.Step = StepIdx;
	Snapshot.StructureVersion = StructureVersion;

	// Clock: ROS builtin_interfaces/Time sec/nsec pairs, matching AppendClockFields.
	const int32 SimSec = static_cast<int32>(d->time);
	Snapshot.Clock.SimSec = SimSec;
	Snapshot.Clock.SimNsec = static_cast<int32>((d->time - SimSec) * 1.0e9);
	const FTimespan Delta = FDateTime::UtcNow() - FDateTime(1970, 1, 1);
	Snapshot.Clock.WallSec = Delta.GetTotalSeconds();
	Snapshot.Clock.WallNsec = (Delta.GetTicks() % ETimespan::TicksPerSecond) * 100;

	if (!bCacheValid.load(std::memory_order_acquire))
		RequestGameThreadRebuild();

	bool bStaleRef = false;
	{
		FScopeLock Lock(&CacheMutex);
		Snapshot.Articulations.Reserve(Cache.Num());
		for (const FCachedArticulation& Rec : Cache)
		{
			AMjArticulation* Art = Rec.Art.Get();
			if (!Art)
			{
				bStaleRef = true;
				continue;
			}
			FMjArticulationState& ArtState = Snapshot.Articulations.AddDefaulted_GetRef();
			ArtState.Name = Rec.ArtSegment;
			for (const TWeakObjectPtr<UMjComponent>& WeakComp : Rec.Producers)
			{
				if (UMjComponent* Comp = WeakComp.Get())
					Comp->DescribeState(ArtState);
			}
			if (UMjTwistController* Twist = Rec.TwistCtrl.Get())
				Twist->DescribeState(ArtState);
		}
	}

	// Non-articulation entities: raw MjIds with no component, read straight from
	// mjData. Prefer the manager's entity cache (built at PostCompile).
	if (Mgr)
	{
		const TArray<FMjEntityRecord>& Entities = Mgr->GetEntities();
		Snapshot.Entities.Reserve(Entities.Num());
		for (const FMjEntityRecord& Ent : Entities)
		{
			if (Ent.MjId < 0 || Ent.MjId >= m->nbody)
				continue;
			FMjEntityState& E = Snapshot.Entities.AddDefaulted_GetRef();
			E.Name = FName(*Ent.Name);
			E.bFreeBase = Ent.bHasFreeBase;
			for (int i = 0; i < 3; ++i)
				E.Xpos[i] = d->xpos[Ent.MjId * 3 + i];
			for (int i = 0; i < 4; ++i)
				E.Xquat[i] = d->xquat[Ent.MjId * 4 + i];
			if (Ent.bHasFreeBase && m->body_jntnum && m->body_jntadr)
			{
				const int FirstJnt = m->body_jntadr[Ent.MjId];
				if (FirstJnt >= 0 && FirstJnt < m->njnt && m->jnt_type[FirstJnt] == mjJNT_FREE)
				{
					const int QAddr = m->jnt_qposadr[FirstJnt];
					const int VAddr = m->jnt_dofadr[FirstJnt];
					E.QPos.SetNumUninitialized(7);
					E.QVel.SetNumUninitialized(6);
					for (int i = 0; i < 7; ++i)
						E.QPos[i] = d->qpos[QAddr + i];
					for (int i = 0; i < 6; ++i)
						E.QVel[i] = d->qvel[VAddr + i];
				}
			}
		}
	}

	if (bStaleRef)
		MarkProducerCacheDirty();

	return Snapshot;
}
