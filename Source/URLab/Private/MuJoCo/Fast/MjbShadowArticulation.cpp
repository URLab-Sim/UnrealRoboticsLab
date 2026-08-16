// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MuJoCo/Fast/MjbShadowArticulation.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Gen/Elements/Joints/MjJoint.gen.h"
#include "MuJoCo/Gen/Elements/Joints/MjFreeJoint.gen.h"
#include "MuJoCo/Gen/Elements/Actuators/MjMotor.gen.h"
#include "State/MjStateCollector.h"
#include "Utils/URLabLogging.h"
#include "Components/SceneComponent.h"
#include "Engine/World.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace URLabFastShadow
{
namespace
{
// Create one element node of the given generated class, name it from the raw
// model, bind it to its raw-model id, and index it on the articulation.
UMjNodeComponent* MakeNode(AMjArticulation& Art, UClass* Class, const char* Name, int32 ObjType, int32 Id)
{
	UMjNodeComponent* Node = NewObject<UMjNodeComponent>(&Art, Class);
	if (!Node)
	{
		return nullptr;
	}
	if (Name && *Name)
	{
		Node->MjName = FString(ANSI_TO_TCHAR(Name));
	}
	if (USceneComponent* Root = Art.GetRootComponent())
	{
		Node->SetupAttachment(Root);
	}
	Node->RegisterComponent();
	Node->BindTo(Id);
	Art.IndexBoundElement(*Node, ObjType, Id);
	return Node;
}
} // namespace

AMjArticulation* Build(AAMjManager* Mgr, mjModel_* Model, const FString& ArtId)
{
	if (!Mgr || !Mgr->PhysicsEngine || !Model)
	{
		return nullptr;
	}
	UWorld* World = Mgr->GetWorld();
	if (!World)
	{
		return nullptr;
	}
	mjModel* m = Model;

	FActorSpawnParameters Params;
	Params.Owner = Mgr;
	Params.ObjectFlags |= RF_Transient;
	AMjArticulation* Art = World->SpawnActor<AMjArticulation>(AMjArticulation::StaticClass(), Params);
	if (!Art)
	{
		return nullptr;
	}
	Art->ActorId = ArtId;
	Art->bRawShadow = true;

	// Joints -> qpos/qvel observation. UMjJoint covers 1-DOF (hinge/slide) and
	// ball joints; a free joint is its own class. DescribeJoint reads the actual
	// jnt_type from the model, so the class only has to classify as a joint.
	for (int32 J = 0; J < m->njnt; ++J)
	{
		UClass* Cls = (m->jnt_type[J] == mjJNT_FREE) ? UMjFreeJoint::StaticClass() : UMjJoint::StaticClass();
		MakeNode(*Art, Cls, mj_id2name(m, mjOBJ_JOINT, J), mjOBJ_JOINT, J);
	}

	// Actuators -> ctrl control. One generic motor kind for all: the state read-back
	// is kind-independent, so the class only has to classify as an actuator.
	for (int32 A = 0; A < m->nu; ++A)
	{
		MakeNode(*Art, UMjMotor::StaticClass(), mj_id2name(m, mjOBJ_ACTUATOR, A), mjOBJ_ACTUATOR, A);
	}

	// Register so the handshake, control ingress and state collector all resolve the art.
	Mgr->PhysicsEngine->RegisterArticulation(Art);
	Mgr->GetStateCollector().MarkProducerCacheDirty();

	UE_LOG(LogURLab, Log,
		TEXT("[MjbShadow] built '%s': %d joints, %d actuators name-bound to the raw model"),
		*ArtId, (int)m->njnt, (int)m->nu);
	return Art;
}

void Teardown(AAMjManager* Mgr, AMjArticulation* Art)
{
	if (!Art)
	{
		return;
	}
	if (Mgr && Mgr->PhysicsEngine)
	{
		Mgr->PhysicsEngine->UnregisterArticulation(Art);
	}
	Art->ClearElementIndex();
	if (Mgr)
	{
		Mgr->GetStateCollector().MarkProducerCacheDirty();
	}
	Art->Destroy();
}
} // namespace URLabFastShadow
