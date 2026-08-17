// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

// Definitions for the Entity addressing, control, and render contract types declared under
// MuJoCo/Entity/.

#include "mujoco/mujoco.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Entity/MjEntityActor.h"
#include "MuJoCo/Entity/MjEntity.h"
#include "MuJoCo/Entity/MjControl.h"
#include "MuJoCo/Entity/MjPoseSource.h"
#include "MuJoCo/Entity/MjControlIngress.h"
#include "MuJoCo/Entity/MjGeomAssetResolver.h"
#include "MuJoCo/Entity/MjSensorSemanticTable.h"
#include "MuJoCo/Entity/MjCameraRegistry.h"
#include "MuJoCo/Entity/MjEntityApi.h"
#include "MuJoCo/Entity/MjGeomAppearance.h"
#include "MuJoCo/Entity/MjOverlayFlags.h"

// --- MjEntity ---------------------------------------------------------------------------------- //
namespace
{
	FString MjNameOf(const mjModel* Model, int ObjType, int Id)
	{
		const char* N = mj_id2name(Model, ObjType, Id);
		return N ? FString(UTF8_TO_TCHAR(N)) : FString();
	}

	// A participant's compiled prefix (AMjArticulation::GetCompiledPrefix) already ends with '_', so
	// its elements are named "<prefix><element>" and match by StartsWith.
	bool BelongsToPrefix(const FString& Name, const FString& Prefix)
	{
		return !Prefix.IsEmpty() && Name.StartsWith(Prefix);
	}

	// Root of an entity = the highest-in-tree (smallest id) body it owns; free base if that body
	// carries an mjJNT_FREE joint.
	void ResolveRoot(const mjModel* Model, FMjEntity& E)
	{
		if (E.BodyIds.Num() == 0)
		{
			return;
		}
		E.RootBodyId = E.BodyIds[0];
		for (int32 B : E.BodyIds)
		{
			E.RootBodyId = FMath::Min(E.RootBodyId, B);
		}
		const int Adr = Model->body_jntadr[E.RootBodyId];
		const int Num = Model->body_jntnum[E.RootBodyId];
		for (int K = 0; K < Num; ++K)
		{
			if (Model->jnt_type[Adr + K] == mjJNT_FREE)
			{
				E.bFreeBase = true;
				break;
			}
		}
	}
}

TArray<FMjEntity> MjEntityBuilder::Build(const mjModel* Model, const FMjEntityPartition& How)
{
	TArray<FMjEntity> Entities;
	if (Model == nullptr)
	{
		return Entities;
	}

	// Single root entity over the whole model (raw path with no split / no prefixes).
	if (How.Prefixes.Num() == 0 && !How.bBodySubtreeSplit)
	{
		FMjEntity& E = Entities.AddDefaulted_GetRef();
		for (int B = 1; B < Model->nbody; ++B) { E.BodyIds.Add(B); } // skip world body 0
		for (int J = 0; J < Model->njnt; ++J) { E.JointIds.Add(J); }
		for (int A = 0; A < Model->nu; ++A) { E.ActuatorIds.Add(A); }
		for (int S = 0; S < Model->nsensor; ++S)
		{
			E.SensorIds.Add(S);
			E.SensorSemantics.Add(MjSensorSemantics::ForSensor(Model, S));
		}
		ResolveRoot(Model, E);
		return Entities;
	}

	// Prefix partition (compiled path: one entity per participant prefix). Build all entities up
	// front so the index map stays valid, then bucket every element by prefix.
	TMap<FString, int32> PrefixToIndex;
	for (int32 i = 0; i < How.Prefixes.Num(); ++i)
	{
		const FString& P = How.Prefixes[i];
		const int32 Idx = Entities.Num();
		FMjEntity& E = Entities.AddDefaulted_GetRef();
		E.Name = FName(*(P.EndsWith(TEXT("_")) ? P.LeftChop(1) : P));
		E.PublicName = (How.PublicNames.IsValidIndex(i) && !How.PublicNames[i].IsNone())
			? How.PublicNames[i]
			: E.Name;
		if (How.ActorIds.IsValidIndex(i))
		{
			E.ActorId = How.ActorIds[i];
		}
		PrefixToIndex.Add(P, Idx);
	}

	auto EntityForName = [&](const FString& Name) -> FMjEntity*
	{
		for (const FString& P : How.Prefixes)
		{
			if (BelongsToPrefix(Name, P))
			{
				return &Entities[PrefixToIndex[P]];
			}
		}
		return nullptr;
	};

	for (int B = 1; B < Model->nbody; ++B)
	{
		if (FMjEntity* E = EntityForName(MjNameOf(Model, mjOBJ_BODY, B))) { E->BodyIds.Add(B); }
	}
	for (int J = 0; J < Model->njnt; ++J)
	{
		if (FMjEntity* E = EntityForName(MjNameOf(Model, mjOBJ_JOINT, J))) { E->JointIds.Add(J); }
	}
	for (int A = 0; A < Model->nu; ++A)
	{
		if (FMjEntity* E = EntityForName(MjNameOf(Model, mjOBJ_ACTUATOR, A))) { E->ActuatorIds.Add(A); }
	}
	for (int S = 0; S < Model->nsensor; ++S)
	{
		if (FMjEntity* E = EntityForName(MjNameOf(Model, mjOBJ_SENSOR, S)))
		{
			E->SensorIds.Add(S);
			E->SensorSemantics.Add(MjSensorSemantics::ForSensor(Model, S));
		}
	}

	for (FMjEntity& E : Entities)
	{
		ResolveRoot(Model, E);
	}
	return Entities;
}

// --- MjControlLease ---------------------------------------------------------------------------- //
bool FMjControlLease::CanWrite(FName Entity, const FGuid& Who) const
{
	const FGuid* Held = Holder.Find(Entity);
	return Held == nullptr || *Held == Who; // unclaimed => default writer allowed
}

bool FMjControlLease::Claim(FName Entity, const FGuid& Who)
{
	const FGuid* Held = Holder.Find(Entity);
	if (Held && *Held != Who)
	{
		return false;
	}
	Holder.Add(Entity, Who);
	return true;
}

void FMjControlLease::Release(FName Entity, const FGuid& Who)
{
	const FGuid* Held = Holder.Find(Entity);
	if (Held && *Held == Who)
	{
		Holder.Remove(Entity);
	}
}

// --- MjEntityApi handles ----------------------------------------------------------------------- //
// Each handle carries its resolved id and a weak ref to the AMjEntity that produced it. The reads
// index the engine's published snapshot at the joint's qpos/dof address; the write routes through the
// engine's control ingress. SetCtrl needs the entity NAME (the lease key) and the frozen struct holds
// only the actor, so it reads the name back from the AMjEntity the handle points at.
namespace
{
	const UMjPhysicsEngine* HandleEngine(const TWeakObjectPtr<AActor>& Entity)
	{
		return AAMjManager::ResolveEngine(Entity.Get());
	}
}

float FMjJoint::Pos() const
{
	const UMjPhysicsEngine* Engine = HandleEngine(Entity);
	const mjModel* Model = Engine ? Engine->GetModel() : nullptr;
	if (Model == nullptr || Id < 0 || Id >= Model->njnt)
	{
		return 0.f;
	}
	const int32 Adr = Model->jnt_qposadr[Id];
	if (Adr < 0 || Adr >= Model->nq)
	{
		return 0.f;
	}
	return static_cast<float>(MjSnapshotValue(*Engine, Adr,
		[](const FMjRenderSnapshot& S) -> const TArray<mjtNum>& { return S.QPos; }));
}

float FMjJoint::Vel() const
{
	const UMjPhysicsEngine* Engine = HandleEngine(Entity);
	const mjModel* Model = Engine ? Engine->GetModel() : nullptr;
	if (Model == nullptr || Id < 0 || Id >= Model->njnt)
	{
		return 0.f;
	}
	const int32 Adr = Model->jnt_dofadr[Id];
	if (Adr < 0 || Adr >= Model->nv)
	{
		return 0.f;
	}
	return static_cast<float>(MjSnapshotValue(*Engine, Adr,
		[](const FMjRenderSnapshot& S) -> const TArray<mjtNum>& { return S.QVel; }));
}

void FMjActuator::SetCtrl(double Value) const
{
	UMjPhysicsEngine* Engine = AAMjManager::ResolveEngine(Entity.Get());
	const AMjEntity* Self = Cast<AMjEntity>(Entity.Get());
	if (Engine == nullptr || Self == nullptr || Id < 0)
	{
		return;
	}
	if (IMjControlIngress* Ingress = Engine->GetControlIngress())
	{
		Ingress->WriteCtrl(Self->GetEntityName(), Id, Value, MjControlWho::UI());
	}
}
