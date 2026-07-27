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
#include "State/MjStateProducer.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Components/MjComponent.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Input/MjTwistController.h"
#include "Components/ActorComponent.h"
#include "GameFramework/Actor.h"
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

	// Registered IMjStateProducers the art walk cannot discover (user channel
	// components, scene-level actors). Scope is resolved here on the game thread:
	// a producer owned by an articulation caches under that art; everything else
	// is a scene producer. The physics-thread step never does scope logic.
	TArray<TWeakObjectPtr<UObject>> NewSceneProducers;
	{
		TArray<TWeakObjectPtr<UObject>> Registered;
		Mgr->GetStateProducers(Registered);
		for (const TWeakObjectPtr<UObject>& WeakProducer : Registered)
		{
			UObject* Obj = WeakProducer.Get();
			if (!Obj)
				continue;

			AActor* OwnerActor = Cast<AActor>(Obj);
			if (!OwnerActor)
			{
				if (UActorComponent* Comp = Cast<UActorComponent>(Obj))
					OwnerActor = Comp->GetOwner();
			}

			AMjArticulation* OwningArt = Cast<AMjArticulation>(OwnerActor);
			FCachedArticulation* Rec = OwningArt
				? NewCache.FindByPredicate([OwningArt](const FCachedArticulation& R) {
					  return R.Art.Get() == OwningArt;
				  })
				: nullptr;
			if (Rec)
				Rec->InterfaceProducers.Add(Obj);
			else
				NewSceneProducers.Add(Obj);
		}
	}

	// World geometry cache: every geom not on a robot body, resolved once here
	// (shapes are static) so the per-step physics-thread build only reads the
	// parent body's live pose. Mesh geoms collapse to their AABB box; very large
	// geoms (the ground / environment shell) and planes are skipped.
	TArray<FCachedWorldGeom> NewWorldGeoms;
	if (Mgr->PhysicsEngine)
	{
		if (const mjModel* m = Mgr->PhysicsEngine->GetModel())
		{
			// Robot bodies are compiled with the articulation's raw-name prefix; match
			// by name (the actor name is stable on the game thread) rather than mj ids,
			// which may not be bound yet when the cache first rebuilds.
			TArray<FString> RobotPrefixes;
			RobotPrefixes.Reserve(Arts.Num());
			for (AMjArticulation* Art : Arts)
			{
				if (Art)
					RobotPrefixes.Add(Art->GetName() + TEXT("_"));
			}

			// Skip geoms whose (half-extent) box is large enough to be environment
			// shell rather than a discrete object: as an AABB it would engulf the
			// robot and flag every start state in collision. Discrete graspable /
			// avoidable objects are well under this. (A future mesh path can carry
			// the real concave geometry instead of a box.)
			const double MaxWorldExtent = 1.0;
			for (int g = 0; g < m->ngeom; ++g)
			{
				const int b = m->geom_bodyid[g];
				if (b <= 0)
					continue;
				const char* BodyRaw = mj_id2name(const_cast<mjModel*>(m), mjOBJ_BODY, b);
				const FString BodyName = BodyRaw ? FString(UTF8_TO_TCHAR(BodyRaw)) : FString();
				bool bRobot = false;
				for (const FString& Prefix : RobotPrefixes)
				{
					if (BodyName.StartsWith(Prefix))
					{
						bRobot = true;
						break;
					}
				}
				if (bRobot)
					continue;

				FCachedWorldGeom W;
				W.BodyId = b;
				const char* Raw = mj_id2name(const_cast<mjModel*>(m), mjOBJ_GEOM, g);
				W.Name = Raw ? FName(UTF8_TO_TCHAR(Raw))
							 : FName(*FString::Printf(TEXT("geom%d"), g));
				const mjtNum* Gp = &m->geom_pos[3 * g];
				const mjtNum* Gq = &m->geom_quat[4 * g];
				for (int k = 0; k < 3; ++k)
					W.LocalPos[k] = Gp[k];
				for (int k = 0; k < 4; ++k)
					W.LocalQuat[k] = Gq[k];
				W.bStatic = (m->body_dofnum[b] == 0);

				const int gt = m->geom_type[g];
				const mjtNum* Sz = &m->geom_size[3 * g];
				if (gt == mjGEOM_SPHERE)
				{
					W.Shape = EMjWorldGeomShape::Sphere;
					W.Size[0] = Sz[0];
				}
				else if (gt == mjGEOM_CYLINDER || gt == mjGEOM_CAPSULE)
				{
					W.Shape = EMjWorldGeomShape::Cylinder;
					W.Size[0] = Sz[0];
					W.Size[1] = Sz[1];
				}
				else if (gt == mjGEOM_BOX)
				{
					W.Shape = EMjWorldGeomShape::Box;
					W.Size[0] = Sz[0];
					W.Size[1] = Sz[1];
					W.Size[2] = Sz[2];
				}
				else if (gt == mjGEOM_MESH)
				{
					// AABB of the (centered) mesh verts -> box half-extents.
					W.Shape = EMjWorldGeomShape::Box;
					const int did = m->geom_dataid[g];
					if (did < 0)
						continue;
					const int va = m->mesh_vertadr[did];
					const int vn = m->mesh_vertnum[did];
					double mx[3] = {0.0, 0.0, 0.0};
					for (int v = 0; v < vn; ++v)
					{
						const float* P = &m->mesh_vert[3 * (va + v)];
						for (int k = 0; k < 3; ++k)
							mx[k] = FMath::Max(mx[k], (double)FMath::Abs(P[k]));
					}
					W.Size[0] = mx[0];
					W.Size[1] = mx[1];
					W.Size[2] = mx[2];
				}
				else
				{
					continue;  // plane / hfield / ellipsoid: unsupported for now
				}

				if (FMath::Max3(W.Size[0], W.Size[1], W.Size[2]) > MaxWorldExtent)
					continue;  // ground / environment shell
				NewWorldGeoms.Add(W);
			}
		}
	}

	{
		FScopeLock Lock(&CacheMutex);
		Cache = MoveTemp(NewCache);
		SceneProducers = MoveTemp(NewSceneProducers);
		WorldGeomCache = MoveTemp(NewWorldGeoms);
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
			for (const TWeakObjectPtr<UObject>& WeakProducer : Rec.InterfaceProducers)
			{
				if (IMjStateProducer* Producer = Cast<IMjStateProducer>(WeakProducer.Get()))
					Producer->DescribeState(ArtState);
			}
		}

		// Scene-scoped producers fill the snapshot's own blocks (e.g. scene
		// user channels), after the art loop but still under the cache lock.
		for (const TWeakObjectPtr<UObject>& WeakProducer : SceneProducers)
		{
			if (IMjStateProducer* Producer = Cast<IMjStateProducer>(WeakProducer.Get()))
				Producer->DescribeSceneState(Snapshot);
		}

		// World geometry: cached (static) shapes composed with the parent body's
		// live world pose. World pose = body pose * geom-local offset.
		Snapshot.WorldGeoms.Reserve(WorldGeomCache.Num());
		for (const FCachedWorldGeom& W : WorldGeomCache)
		{
			if (W.BodyId < 0 || W.BodyId >= m->nbody)
				continue;
			FMjWorldGeom G;
			G.Name = W.Name;
			G.Shape = W.Shape;
			G.Size[0] = W.Size[0];
			G.Size[1] = W.Size[1];
			G.Size[2] = W.Size[2];
			G.bStatic = W.bStatic;
			const mjtNum* Bp = &d->xpos[3 * W.BodyId];
			const mjtNum* Bq = &d->xquat[4 * W.BodyId];
			mjtNum Rotated[3];
			mju_rotVecQuat(Rotated, W.LocalPos, Bq);
			G.Xpos[0] = Bp[0] + Rotated[0];
			G.Xpos[1] = Bp[1] + Rotated[1];
			G.Xpos[2] = Bp[2] + Rotated[2];
			mju_mulQuat(G.Xquat, Bq, W.LocalQuat);
			Snapshot.WorldGeoms.Add(MoveTemp(G));
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
