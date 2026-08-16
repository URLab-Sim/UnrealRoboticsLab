// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Entity/MjEntityLogicComponent.h"

#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Entity/MjEntityActor.h"

UMjEntityLogicComponent::UMjEntityLogicComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UMjEntityLogicComponent::BeginPlay()
{
	Super::BeginPlay();

	if (AAMjManager* Manager = AAMjManager::GetManager())
	{
		CachedEntity = Manager->GetEntity(OwnerEntityName);
	}

	if (AMjEntity* Entity = CachedEntity.Get())
	{
		NativeOnEntityReady(Entity);
		ReceiveEntityReady(Entity);
	}
}

void UMjEntityLogicComponent::TickComponent(
	float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	AMjEntity* Entity = CachedEntity.Get();
	if (Entity == nullptr)
	{
		return;
	}

	NativeOnEntityTick(Entity, DeltaTime);
	ReceiveEntityTick(Entity, DeltaTime);
}
