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
#include "MuJoCo/Entity/MjEntityPawn.h"
#include "MuJoCo/Input/MjTwistController.h"
#include "Components/ActorComponent.h"
#include "GameFramework/Actor.h"
#include "EngineUtils.h"
#include "Async/Async.h"
#include "Misc/ScopeExit.h"
#include "mujoco/mujoco.h"

// Element state production lives here rather than on the elements.
//
// A joint, a sensor and an actuator carry no per-instance state and so have no
// hand class to hold an override, and one virtual per leaf would mean 47 sensor
// overrides of one identical body. Each of these is a pure function of the
// element's compiled id and the model, so each is a free function keyed on the id
// the entity owns.

namespace
{
/** The IR name of an element: its compiled mj name with the entity's "<Name>_"
 *  prefix stripped, then sanitized. The one place that strip happens on the id path,
 *  matching FMjCanonicalName::PartSegment on the component path. */
FName LocalName(const mjModel* m, int32 ObjType, int32 Id, FName EntityName)
{
	const char* Raw = mj_id2name(m, ObjType, Id);
	FString Local = Raw ? FString(UTF8_TO_TCHAR(Raw)) : FString();
	const FString Prefix = EntityName.ToString() + TEXT("_");
	if (Local.StartsWith(Prefix))
		Local = Local.Mid(Prefix.Len());
	return FName(*FMjCanonicalName::Sanitize(Local));
}

void DescribeJoint(FName EntityName, int32 Id, const mjModel* m, mjData* d,
	FMjArticulationState& Out)
{
	if (Id < 0 || Id >= m->njnt)
	{
		return;
	}
	const int32 QposAdr = m->jnt_qposadr[Id];
	const int32 DofAdr = m->jnt_dofadr[Id];
	if (QposAdr < 0 || DofAdr < 0)
	{
		return;
	}

	// Slot widths follow the joint type -- free 7/6, ball 4/3, hinge and slide
	// 1/1 -- counted from the joint's first qpos and dof slot.
	int32 QSize = 1;
	int32 VSize = 1;
	EMjJointType JType = EMjJointType::hinge;
	switch (m->jnt_type[Id])
	{
		case mjJNT_FREE:
			QSize = 7;
			VSize = 6;
			JType = EMjJointType::free;
			break;
		case mjJNT_BALL:
			QSize = 4;
			VSize = 3;
			JType = EMjJointType::ball;
			break;
		case mjJNT_SLIDE:
			JType = EMjJointType::slide;
			break;
		default:
			JType = EMjJointType::hinge;
			break;
	}

	FMjJointState& J = Out.Joints.AddDefaulted_GetRef();
	J.Name = LocalName(m, mjOBJ_JOINT, Id, EntityName);
	J.Type = JType;
	J.QPos.SetNumUninitialized(QSize);
	J.QVel.SetNumUninitialized(VSize);
	for (int32 i = 0; i < QSize; ++i)
	{
		J.QPos[i] = d->qpos[QposAdr + i];
	}
	for (int32 i = 0; i < VSize; ++i)
	{
		J.QVel[i] = d->qvel[DofAdr + i];
	}

	// The reference slice, for the 1-DOF joints a URDF exposes, so the ROS
	// /joint_states shift can emit qpos - qpos0: URDF q=0 is MuJoCo qpos0. Free
	// and ball joints are not URDF joints, so they record no shift.
	if (JType == EMjJointType::hinge || JType == EMjJointType::slide)
	{
		J.RefPos.SetNumUninitialized(QSize);
		for (int32 i = 0; i < QSize; ++i)
		{
			J.RefPos[i] = m->qpos0[QposAdr + i];
		}
	}
}

void DescribeSensor(FName EntityName, int32 Id, EMjSensorSemantic Semantic, const mjModel* m,
	mjData* d, FMjArticulationState& Out)
{
	if (Id < 0 || Id >= m->nsensor)
	{
		return;
	}
	const int32 Dim = m->sensor_dim[Id];
	const int32 Adr = m->sensor_adr[Id];
	if (Dim <= 0 || Adr < 0 || Adr >= m->nsensordata)
	{
		return;
	}

	// The IR carries raw MuJoCo SI values, in the MuJoCo frame, as joints and
	// bodies do. The coordinate and unit fixup belongs to the display-facing
	// reader and not to the serialisation path.
	FMjSensorState& S = Out.Sensors.AddDefaulted_GetRef();
	S.Name = LocalName(m, mjOBJ_SENSOR, Id, EntityName);
	S.Semantic = Semantic;
	S.Values.SetNumUninitialized(Dim);
	for (int32 i = 0; i < Dim; ++i)
	{
		S.Values[i] = d->sensordata[Adr + i];
	}
}

void DescribeActuator(FName EntityName, int32 Id, const mjModel* m, mjData* d,
	FMjArticulationState& Out)
{
	if (Id < 0 || Id >= m->nu)
	{
		return;
	}
	FMjActuatorState& A = Out.Actuators.AddDefaulted_GetRef();
	A.Name = LocalName(m, mjOBJ_ACTUATOR, Id, EntityName);

	// The transmission target comes out of the compiled model rather than the
	// spec, so an actuator that reached its joint through a default class is
	// reported the same as one that named it outright.
	const int32 TrnType = m->actuator_trntype[Id];
	if (TrnType == mjTRN_JOINT || TrnType == mjTRN_JOINTINPARENT)
	{
		const int32 JointId = m->actuator_trnid[Id * 2];
		if (JointId >= 0 && JointId < m->njnt)
		{
			A.TargetJoint = LocalName(m, mjOBJ_JOINT, JointId, EntityName);
		}
	}

	const int32 ActAdr = m->actuator_actadr[Id]; // negative for a stateless actuator
	A.Ctrl = d->ctrl[Id];
	A.Act = (ActAdr >= 0) ? d->act[ActAdr] : 0.0;
	A.Force = d->actuator_force[Id];
}
} // namespace

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

	if (!Mgr->PhysicsEngine)
		return;

	// Assemble into a local with no lock held; the physics thread keeps reading
	// the previous cache meanwhile. Only the swap below is guarded.
	const TArray<FMjEntity>& Partition = Mgr->PhysicsEngine->GetEntityPartition();
	TArray<FCachedEntity> NewCache;
	NewCache.Reserve(Partition.Num());

	// A possessed entity's twist controller lives on its AMjEntityPawn, spawned by the
	// possess handoff after the articulations are retired, so it resolves by the entity name
	// the pawn possesses rather than through the destroyed articulation. Gathered once here;
	// an unpossessed entity has no pawn and so publishes no twist.
	TMap<FName, TWeakObjectPtr<UMjTwistController>> PawnTwistByEntity;
	if (UWorld* World = Mgr->GetWorld())
	{
		for (TActorIterator<AMjEntityPawn> It(World); It; ++It)
		{
			if (It->OwnerEntityName.IsNone())
				continue;
			if (UMjTwistController* TC = It->FindComponentByClass<UMjTwistController>())
				PawnTwistByEntity.Add(It->OwnerEntityName, TC);
		}
	}

	// The scope a side-channel producer attaches under, keyed by the entity name (== the
	// owning actor's GetName()); an entity with no actor behind it (a raw prop) simply has no
	// channels. Maps entity name -> cache index for scoping the registered producers below.
	TMap<FName, int32> NameToRec;
	for (const FMjEntity& E : Partition)
	{
		FCachedEntity Rec;
		Rec.Name = E.Name;
		Rec.PublicName = E.PublicName;
		Rec.BodyIds = E.BodyIds;
		Rec.JointIds = E.JointIds;
		Rec.ActuatorIds = E.ActuatorIds;
		Rec.SensorIds = E.SensorIds;
		Rec.SensorSemantics = E.SensorSemantics;

		if (const TWeakObjectPtr<UMjTwistController>* Found = PawnTwistByEntity.Find(E.Name))
			Rec.TwistCtrl = *Found;
		NameToRec.Add(E.Name, NewCache.Num());
		NewCache.Add(MoveTemp(Rec));
	}

	// Registered IMjStateProducers the partition cannot discover (user channel
	// components, scene-level actors). Scope is resolved here on the game thread:
	// a producer owned by an articulation caches under that entity; everything else
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
			const int32* RecIdx = OwningArt ? NameToRec.Find(OwningArt->GetFName()) : nullptr;
			if (RecIdx)
				NewCache[*RecIdx].InterfaceProducers.Add(Obj);
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
			// Robot bodies are compiled with the entity's raw-name prefix; match by
			// name rather than mj ids, which may not be bound yet when the cache first
			// rebuilds.
			TArray<FString> RobotPrefixes;
			RobotPrefixes.Reserve(Partition.Num());
			for (const FMjEntity& E : Partition)
			{
				RobotPrefixes.Add(E.Name.ToString() + TEXT("_"));
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
					// Real triangle geometry (same mesh_vert / mesh_face tables the
					// URDF export uses), so concave obstacles reach MoveIt faithfully
					// instead of an engulfing AABB.
					const int did = m->geom_dataid[g];
					if (did < 0)
						continue;
					const int va = m->mesh_vertadr[did];
					const int vn = m->mesh_vertnum[did];
					const int fa = m->mesh_faceadr[did];
					const int fn = m->mesh_facenum[did];
					if (vn <= 0 || fn <= 0)
						continue;
					TSharedPtr<FMjWorldMesh> Mesh = MakeShared<FMjWorldMesh>();
					Mesh->Verts.Reserve(vn);
					for (int v = 0; v < vn; ++v)
					{
						const float* P = &m->mesh_vert[3 * (va + v)];
						Mesh->Verts.Emplace(P[0], P[1], P[2]);
					}
					Mesh->Tris.Reserve(fn * 3);
					for (int f = 0; f < fn; ++f)
					{
						const int* T = &m->mesh_face[3 * (fa + f)];
						Mesh->Tris.Add(T[0]);
						Mesh->Tris.Add(T[1]);
						Mesh->Tris.Add(T[2]);
					}
					W.Shape = EMjWorldGeomShape::Mesh;
					W.Mesh = Mesh;
				}
				else
				{
					continue; // plane / hfield / ellipsoid: unsupported for now
				}

				// Primitive extent cap skips the ground / environment shell, whose AABB
				// would engulf the robot. Meshes carry real geometry, so they pass through.
				if (W.Shape != EMjWorldGeomShape::Mesh && FMath::Max3(W.Size[0], W.Size[1], W.Size[2]) > MaxWorldExtent)
					continue;
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

	{
		FScopeLock Lock(&CacheMutex);
		Snapshot.Articulations.Reserve(Cache.Num());
		for (const FCachedEntity& Rec : Cache)
		{
			FMjArticulationState& ArtState = Snapshot.Articulations.AddDefaulted_GetRef();
			ArtState.Name = Rec.PublicName;

			// Bodies: the transform the body component's DescribeState produced, read
			// straight from mjData by id (same fields, same axis handling).
			for (int32 BodyId : Rec.BodyIds)
			{
				if (BodyId < 0 || BodyId >= m->nbody)
					continue;
				FMjBodyState& B = ArtState.Bodies.AddDefaulted_GetRef();
				B.Name = LocalName(m, mjOBJ_BODY, BodyId, Rec.Name);
				for (int32 i = 0; i < 3; ++i)
					B.Xpos[i] = d->xpos[BodyId * 3 + i];
				for (int32 i = 0; i < 4; ++i)
					B.Xquat[i] = d->xquat[BodyId * 4 + i];
			}
			for (int32 JointId : Rec.JointIds)
				DescribeJoint(Rec.Name, JointId, m, d, ArtState);
			for (int32 ActuatorId : Rec.ActuatorIds)
				DescribeActuator(Rec.Name, ActuatorId, m, d, ArtState);
			for (int32 k = 0; k < Rec.SensorIds.Num(); ++k)
			{
				const EMjSensorSemantic Semantic = Rec.SensorSemantics.IsValidIndex(k)
													 ? Rec.SensorSemantics[k]
													 : EMjSensorSemantic::Generic;
				DescribeSensor(Rec.Name, Rec.SensorIds[k], Semantic, m, d, ArtState);
			}

			if (UMjTwistController* Twist = Rec.TwistCtrl.Get())
				Twist->DescribeState(m, d, ArtState);
			for (const TWeakObjectPtr<UObject>& WeakProducer : Rec.InterfaceProducers)
			{
				if (IMjStateProducer* Producer = Cast<IMjStateProducer>(WeakProducer.Get()))
					Producer->DescribeState(m, d, ArtState);
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
			G.Mesh = W.Mesh;
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
	// mjData. Prefer the manager's entity cache, which the compile rebuilt.
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

	return Snapshot;
}
