// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Entity/MjEntityPawn.h"

#include "Camera/CameraComponent.h"
#include "Components/SceneComponent.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "GameFramework/SpringArmComponent.h"
#include "InputAction.h"
#include "InputMappingContext.h"

#include "MuJoCo/Input/MjTwistController.h"

AMjEntityPawn::AMjEntityPawn()
{
	PrimaryActorTick.bCanEverTick = false;

	DefaultSceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("EntityPawnRoot"));
	RootComponent = DefaultSceneRoot;
}

void AMjEntityPawn::PostInitializeComponents()
{
	Super::PostInitializeComponents();

	if (FindComponentByClass<UMjTwistController>() != nullptr)
	{
		return;
	}

	TwistController = NewObject<UMjTwistController>(this, TEXT("TwistController"));

	static UInputMappingContext* DefaultIMC = LoadObject<UInputMappingContext>(
		nullptr, TEXT("/UnrealRoboticsLab/Input/IMC_TwistControl.IMC_TwistControl"));
	static UInputAction* DefaultMove = LoadObject<UInputAction>(
		nullptr, TEXT("/UnrealRoboticsLab/Input/IA_TwistMove.IA_TwistMove"));
	static UInputAction* DefaultTurn = LoadObject<UInputAction>(
		nullptr, TEXT("/UnrealRoboticsLab/Input/IA_TwistTurn.IA_TwistTurn"));

	if (DefaultIMC != nullptr)
	{
		TwistController->TwistMappingContext = DefaultIMC;
	}
	if (DefaultMove != nullptr)
	{
		TwistController->MoveAction = DefaultMove;
	}
	if (DefaultTurn != nullptr)
	{
		TwistController->TurnAction = DefaultTurn;
	}

	TwistController->RegisterComponent();
}

void AMjEntityPawn::SetTrackedComponent(USceneComponent* Body)
{
	TrackedComponent = Body;
	if (PossessCameraArm != nullptr)
	{
		AttachArmToTracked();
	}
}

void AMjEntityPawn::AttachArmToTracked()
{
	if (PossessCameraArm == nullptr)
	{
		return;
	}

	USceneComponent* Target = TrackedComponent.IsValid() ? TrackedComponent.Get() : GetRootComponent();
	PossessCameraArm->AttachToComponent(Target, FAttachmentTransformRules::KeepRelativeTransform);
	PossessCameraArm->SetRelativeRotation(FRotator(PossessCameraPitch, 0.0f, 0.0f));
}

void AMjEntityPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	UEnhancedInputComponent* EIC = Cast<UEnhancedInputComponent>(PlayerInputComponent);
	if (EIC == nullptr)
	{
		return;
	}
	if (UMjTwistController* TwistCtrl = FindComponentByClass<UMjTwistController>())
	{
		TwistCtrl->BindInput(EIC);
	}
}

void AMjEntityPawn::PossessedBy(AController* NewController)
{
	Super::PossessedBy(NewController);

	APlayerController* PC = Cast<APlayerController>(NewController);
	if (PC == nullptr)
	{
		return;
	}

	UMjTwistController* TwistCtrl = FindComponentByClass<UMjTwistController>();
	if (TwistCtrl != nullptr && TwistCtrl->TwistMappingContext != nullptr)
	{
		if (ULocalPlayer* LP = PC->GetLocalPlayer())
		{
			if (UEnhancedInputLocalPlayerSubsystem* Subsystem = LP->GetSubsystem<UEnhancedInputLocalPlayerSubsystem>())
			{
				Subsystem->AddMappingContext(TwistCtrl->TwistMappingContext, 1);
			}
		}
	}

	USpringArmComponent* Arm = NewObject<USpringArmComponent>(this, TEXT("PossessCameraArm"));
	Arm->SetupAttachment(RootComponent);
	Arm->TargetArmLength = PossessCameraDistance;
	Arm->SetRelativeRotation(FRotator(PossessCameraPitch, 0.0f, 0.0f));
	Arm->bDoCollisionTest = false;
	Arm->bUsePawnControlRotation = false;
	Arm->bEnableCameraLag = true;
	Arm->CameraLagSpeed = PossessCameraLagSpeed;
	Arm->CameraLagMaxDistance = 100.0f;
	Arm->bEnableCameraRotationLag = true;
	Arm->CameraRotationLagSpeed = PossessCameraRotationLagSpeed;
	Arm->SocketOffset = PossessCameraOffset;
	Arm->RegisterComponent();

	UCameraComponent* Cam = NewObject<UCameraComponent>(this, TEXT("PossessCamera"));
	Cam->SetupAttachment(Arm);
	Cam->RegisterComponent();

	Arm->ComponentTags.Add(TEXT("PossessCamera"));
	Cam->ComponentTags.Add(TEXT("PossessCamera"));

	PossessCameraArm = Arm;
	PossessCamera = Cam;

	// The camera follows the tracked body rather than the pawn, so it tracks the physics rather than
	// the placement the pawn was spawned at.
	AttachArmToTracked();
}

void AMjEntityPawn::UnPossessed()
{
	APlayerController* PC = Cast<APlayerController>(GetController());

	if (UMjTwistController* TwistCtrl = FindComponentByClass<UMjTwistController>())
	{
		TwistCtrl->ResetTwist();
		if (PC != nullptr && TwistCtrl->TwistMappingContext != nullptr)
		{
			if (ULocalPlayer* LP = PC->GetLocalPlayer())
			{
				if (UEnhancedInputLocalPlayerSubsystem* Subsystem = LP->GetSubsystem<UEnhancedInputLocalPlayerSubsystem>())
				{
					Subsystem->RemoveMappingContext(TwistCtrl->TwistMappingContext);
				}
			}
		}
	}

	TArray<UActorComponent*> ToRemove;
	for (UActorComponent* Comp : GetComponents())
	{
		if (Comp != nullptr && Comp->ComponentTags.Contains(TEXT("PossessCamera")))
		{
			ToRemove.Add(Comp);
		}
	}
	for (UActorComponent* Comp : ToRemove)
	{
		Comp->DestroyComponent();
	}

	PossessCameraArm = nullptr;
	PossessCamera = nullptr;

	Super::UnPossessed();
}
