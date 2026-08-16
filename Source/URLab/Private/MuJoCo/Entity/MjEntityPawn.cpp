// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Entity/MjEntityPawn.h"

AMjEntityPawn::AMjEntityPawn()
{
	PrimaryActorTick.bCanEverTick = false;
}

void AMjEntityPawn::PossessedBy(AController* NewController)
{
	Super::PossessedBy(NewController);
}

void AMjEntityPawn::UnPossessed()
{
	Super::UnPossessed();
}

void AMjEntityPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
}
