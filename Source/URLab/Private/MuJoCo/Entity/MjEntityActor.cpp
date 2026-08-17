// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Entity/MjEntityActor.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Entity/MjEntityLogicComponent.h"
#include "MuJoCo/Entity/MjEntityMembers.h"

AMjEntity::AMjEntity()
{
	PrimaryActorTick.bCanEverTick = false;
}

namespace
{
	// Resolve a member name to an id and pack it with this actor. The handle stores the resolved id
	// and a weak ref back to the face, so every later call is id-based with no string lookup.
	template <typename FHandle>
	FHandle MakeHandle(const AMjEntity* Self, FName EntityName, EMjEntityMember Family, FName Member)
	{
		FHandle Handle;
		Handle.Entity = const_cast<AMjEntity*>(Self);
		Handle.Id = MjEntityMembers::ResolveId(
			AAMjManager::ResolveEngine(Self), EntityName, Family, Member);
		return Handle;
	}
}

FMjJoint AMjEntity::Joint(FName Name) const
{
	return MakeHandle<FMjJoint>(this, EntityName, EMjEntityMember::Joint, Name);
}

FMjActuator AMjEntity::Actuator(FName Name) const
{
	return MakeHandle<FMjActuator>(this, EntityName, EMjEntityMember::Actuator, Name);
}

FMjGeom AMjEntity::Geom(FName Name) const
{
	return MakeHandle<FMjGeom>(this, EntityName, EMjEntityMember::Geom, Name);
}

UMjEntityLogicComponent* AMjEntity::AddLogicComponent(TSubclassOf<UMjEntityLogicComponent> LogicClass)
{
	if (LogicClass == nullptr)
	{
		return nullptr;
	}

	UMjEntityLogicComponent* Logic = NewObject<UMjEntityLogicComponent>(this, LogicClass);
	if (Logic == nullptr)
	{
		return nullptr;
	}

	Logic->OwnerEntityName = EntityName;
	Logic->RegisterComponent();
	return Logic;
}

void AMjEntity::GetLogicComponents(TArray<UMjEntityLogicComponent*>& Out) const
{
	GetComponents<UMjEntityLogicComponent>(Out);
}
