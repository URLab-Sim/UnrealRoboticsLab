// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

#include "MjEntityPawn.generated.h"

class UCameraComponent;
class USceneComponent;
class USpringArmComponent;
class UMjTwistController;

/**
 * Opt-in possession for one entity: the spring-arm chase camera and twist controller re-homed off
 * the articulation. Not every entity gets one -- an entity is possessable only when a pawn is spawned
 * for it. The spring-arm follows the render view's body component so the camera tracks the physics,
 * and PossessedBy / UnPossessed wire the twist mapping context exactly as the articulation did.
 *
 * The pawn is not the physics body, so it cannot host the spring-arm off its own root the way the
 * articulation hung it off RootBody. The integrator calls SetTrackedComponent with the render view's
 * body component for this entity, and the spring-arm attaches there.
 */
UCLASS(config = Game, BlueprintType)
class URLAB_API AMjEntityPawn : public APawn
{
	GENERATED_BODY()

public:
	AMjEntityPawn();

	/** The entity this pawn possesses, by its stable model-derived name. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Entity")
	FName OwnerEntityName;

	/**
	 * Point the possess-camera at the render view's body component for this entity.
	 *
	 * The pawn owns no physics body, so the spring-arm has nothing of its own to follow. The integrator
	 * hands it the body component the render view drives from the snapshot; the camera then tracks the
	 * robot rather than the fixed placement the pawn was spawned at. Safe to call before or after
	 * possession -- if the arm already exists it re-attaches immediately.
	 */
	void SetTrackedComponent(USceneComponent* Body);

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
	/**
	 * Unreal fires this for every actor before any BeginPlay starts, which is why the twist controller
	 * is attached here: consumers built during a sibling actor's BeginPlay see it as soon as they look.
	 */
	virtual void PostInitializeComponents() override;

	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Entity")
	TObjectPtr<USceneComponent> DefaultSceneRoot;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Camera")
	TObjectPtr<USpringArmComponent> PossessCameraArm;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Camera")
	TObjectPtr<UCameraComponent> PossessCamera;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Input")
	TObjectPtr<UMjTwistController> TwistController;

private:
	/** Attach the spring-arm to the tracked body, or fall back to the pawn root when none is set. */
	void AttachArmToTracked();

	/** The render view's body component the possess-camera follows. */
	TWeakObjectPtr<USceneComponent> TrackedComponent;
};
