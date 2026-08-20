// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MuJoCo/Fast/DroneViewerPawn.h"

#include "Camera/CameraComponent.h"
#include "GameFramework/FloatingPawnMovement.h"
#include "GameFramework/PlayerController.h"
#include "Components/InputComponent.h"

ADroneViewerPawn::ADroneViewerPawn()
{
	PrimaryActorTick.bCanEverTick = true;

	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	SetRootComponent(Camera);

	Movement = CreateDefaultSubobject<UFloatingPawnMovement>(TEXT("Movement"));
	Movement->UpdatedComponent = Camera;
	Movement->MaxSpeed = 4000.0f;   // ceiling; per-tick input scales the request
	Movement->Acceleration = 16000.0f;
	Movement->Deceleration = 16000.0f;

	// The pawn faces where the controller looks; movement is relative to that.
	bUseControllerRotationPitch = true;
	bUseControllerRotationYaw = true;
	bUseControllerRotationRoll = false;
	AutoPossessPlayer = EAutoReceiveInput::Disabled;  // possessed explicitly by the launcher
}

void ADroneViewerPawn::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	// Poll movement keys off the controller -- no input mappings needed, so this
	// works in any booted map. Directions are relative to the current view.
	APlayerController* PC = Cast<APlayerController>(GetController());
	if (!PC)
	{
		return;
	}
	FVector Dir = FVector::ZeroVector;
	if (PC->IsInputKeyDown(EKeys::W)) Dir += GetActorForwardVector();
	if (PC->IsInputKeyDown(EKeys::S)) Dir -= GetActorForwardVector();
	if (PC->IsInputKeyDown(EKeys::D)) Dir += GetActorRightVector();
	if (PC->IsInputKeyDown(EKeys::A)) Dir -= GetActorRightVector();
	if (PC->IsInputKeyDown(EKeys::E) || PC->IsInputKeyDown(EKeys::SpaceBar)) Dir += FVector::UpVector;
	if (PC->IsInputKeyDown(EKeys::Q) || PC->IsInputKeyDown(EKeys::LeftControl)) Dir -= FVector::UpVector;

	if (!Dir.IsNearlyZero())
	{
		const float Speed = MoveSpeed *
			(PC->IsInputKeyDown(EKeys::LeftShift) ? BoostMultiplier : 1.0f);
		// Scale the normalized direction so MaxSpeed doesn't clamp us below MoveSpeed.
		AddMovementInput(Dir.GetSafeNormal(), Speed / FMath::Max(Movement->MaxSpeed, 1.0f));
	}
}

void ADroneViewerPawn::SetupPlayerInputComponent(UInputComponent* InInputComponent)
{
	Super::SetupPlayerInputComponent(InInputComponent);
	if (!InInputComponent)
	{
		return;
	}
	// Bind the mouse axes directly (BindAxisKey needs no project input config).
	InInputComponent->BindAxisKey(EKeys::MouseX, this, &ADroneViewerPawn::Turn);
	InInputComponent->BindAxisKey(EKeys::MouseY, this, &ADroneViewerPawn::LookUp);
}

void ADroneViewerPawn::Turn(float Value)
{
	if (Value != 0.0f)
	{
		AddControllerYawInput(Value * LookSensitivity);
	}
}

void ADroneViewerPawn::LookUp(float Value)
{
	if (Value != 0.0f)
	{
		AddControllerPitchInput(-Value * LookSensitivity);  // screen-space (up = look up)
	}
}
