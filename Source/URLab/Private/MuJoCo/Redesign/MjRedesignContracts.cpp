// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

// STEP 0 of the Entity redesign (docs/core_redesign_target.md sec 16.7): the frozen contract
// headers, plus trivial stub definitions so they compile AND link. Phases fill in the real logic;
// the SHAPES are frozen here so the concurrent tracks can build against them.

#include "MuJoCo/Redesign/MjEntity.h"
#include "MuJoCo/Redesign/MjControl.h"
#include "MuJoCo/Redesign/MjPoseSource.h"
#include "MuJoCo/Redesign/MjControlIngress.h"
#include "MuJoCo/Redesign/MjGeomAssetResolver.h"
#include "MuJoCo/Redesign/MjEnrichment.h"
#include "MuJoCo/Redesign/MjSensorSemanticTable.h"
#include "MuJoCo/Redesign/MjCameraRegistry.h"
#include "MuJoCo/Redesign/MjEntityApi.h"
#include "MuJoCo/Redesign/MjGeomAppearance.h"
#include "MuJoCo/Redesign/MjOverlayFlags.h"

// --- MjEntity ---------------------------------------------------------------------------------- //
TArray<FMjEntity> MjEntityBuilder::Build(const mjModel* /*Model*/, const FMjEntityPartition& /*How*/)
{
	// TODO(phase 1): partition the live model by prefix into entities.
	return {};
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
float FMjJoint::Pos() const { return 0.f; }            // TODO(phase 9): read d->qpos via the entity
float FMjJoint::Vel() const { return 0.f; }            // TODO(phase 9): read d->qvel
void  FMjActuator::SetCtrl(double /*Value*/) const {}  // TODO(phase 9): -> control buffer + lease

// --- MjCameraRegistry -------------------------------------------------------------------------- //
void FMjCameraRegistry::Build(const mjModel* /*Model*/, const TArray<FMjEntity>& /*Entities*/)
{
	// TODO(phase 4N-c): enumerate model cameras, compute canonical names from entity prefixes.
}

const FMjCameraInfo* FMjCameraRegistry::Find(FName Canonical) const
{
	return Cameras.FindByPredicate([Canonical](const FMjCameraInfo& C) { return C.CanonicalName == Canonical; });
}

// --- MjSensorSemantics ------------------------------------------------------------------------- //
EMjSensorSemantic MjSensorSemantics::ForSensor(const mjModel* /*Model*/, int32 /*SensorId*/)
{
	// TODO(phase 4N-b): map mjtSensor type (+ name heuristic) to the ROS semantic.
	return EMjSensorSemantic::Generic;
}
