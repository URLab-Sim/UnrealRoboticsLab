// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCoSpec/MJModelComponent.h"
#include "MuJoCoSpec/MJSceneComponents.h"
#include "MuJoCoSpec/MJActorComponents.h"

AMjArticulation::AMjArticulation()
{
	PrimaryActorTick.bCanEverTick = false;

	DefaultSceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("DefaultSceneRoot"));
	RootComponent = DefaultSceneRoot;

}

FString AMjArticulation::GetCompiledPrefix() const
{
	return ActorId.IsEmpty() ? FString() : ActorId + TEXT("/");
}

void AMjArticulation::BeginPlay()
{
	Super::BeginPlay();
	UpdateGroup3Visibility();
}

TArray<UMJActuatorComponent*> AMjArticulation::GetActuators() const
{
	TArray<UMJActuatorComponent*> Out;
	GetComponents<UMJActuatorComponent>(Out);
	return Out;
}

TArray<UMJJointComponent*> AMjArticulation::GetJoints() const
{
	TArray<UMJJointComponent*> Out;
	GetComponents<UMJJointComponent>(Out);
	return Out;
}

TArray<UMJSensorComponent*> AMjArticulation::GetSensors() const
{
	TArray<UMJSensorComponent*> Out;
	GetComponents<UMJSensorComponent>(Out);
	return Out;
}

TArray<UMJTendonComponent*> AMjArticulation::GetTendons() const
{
	TArray<UMJTendonComponent*> Out;
	GetComponents<UMJTendonComponent>(Out);
	return Out;
}

TArray<UMJBodyComponent*> AMjArticulation::GetBodies() const
{
	TArray<UMJBodyComponent*> Out;
	GetComponents<UMJBodyComponent>(Out);
	return Out;
}

TArray<UMJGeomComponent*> AMjArticulation::GetGeoms() const
{
	TArray<UMJGeomComponent*> Out;
	GetComponents<UMJGeomComponent>(Out);
	return Out;
}

void AMjArticulation::ToggleGroup3Visibility()
{
	bShowGroup3 = !bShowGroup3;
	UpdateGroup3Visibility();
}

void AMjArticulation::UpdateGroup3Visibility()
{
	for (UMJGeomComponent* Geom : GetGeoms())
	{
		if (Geom && Geom->Spec.Group == 3)
		{
			Geom->SetVisibility(bShowGroup3, false);
		}
	}
}

#if WITH_EDITOR
void AMjArticulation::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
	if (PropertyChangedEvent.GetPropertyName() == GET_MEMBER_NAME_CHECKED(AMjArticulation, bShowGroup3))
	{
		UpdateGroup3Visibility();
	}
}

void AMjArticulation::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	UpdateGroup3Visibility();
}
#endif
