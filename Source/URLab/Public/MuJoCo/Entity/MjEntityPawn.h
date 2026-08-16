// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

#include "MjEntityPawn.generated.h"

class AMjEntity;
class UCameraComponent;
class USpringArmComponent;
class UMjTwistController;

/**
 * Opt-in possession for one entity: the spring-arm chase camera and twist controller re-homed off
 * the articulation. Not every entity gets one -- an entity is possessable only when a pawn is spawned
 * for it. The spring-arm attaches to the render view's body component so the camera follows the
 * physics, and PossessedBy / UnPossessed wire the twist mapping context exactly as the articulation
 * did.
 */
UCLASS(config = Game)
class URLAB_API AMjEntityPawn : public APawn
{
	GENERATED_BODY()

public:
	AMjEntityPawn();

	/** The entity this pawn possesses, by its stable model-derived name. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Entity")
	FName OwnerEntityName;

	// --- Possess camera (re-homed from AMjArticulation) --------------------- //

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera", meta = (ClampMin = "50.0", ClampMax = "1000.0"))
	float PossessCameraDistance = 300.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera", meta = (ClampMin = "-80.0", ClampMax = "0.0"))
	float PossessCameraPitch = -20.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera", meta = (ClampMin = "0.5", ClampMax = "20.0"))
	float PossessCameraLagSpeed = 3.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera", meta = (ClampMin = "0.5", ClampMax = "20.0"))
	float PossessCameraRotationLagSpeed = 3.0f;

	/** Lifts the camera above the body centre, which takes the bounce out of a gait. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Camera")
	FVector PossessCameraOffset = FVector(0.0f, 0.0f, 30.0f);

	virtual void PossessedBy(AController* NewController) override;
	virtual void UnPossessed() override;

protected:
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Camera")
	TObjectPtr<USpringArmComponent> PossessCameraArm;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Camera")
	TObjectPtr<UCameraComponent> PossessCamera;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Input")
	TObjectPtr<UMjTwistController> TwistController;

private:
	/** The entity this pawn follows, resolved when possession is set up. */
	TWeakObjectPtr<AMjEntity> Entity;
};
