// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "DroneViewerPawn.generated.h"

/**
 * A free-fly "drone" camera pawn for the VR/spectator viewer: WASD moves in the
 * view plane, Q/E (and Space/Ctrl) move up/down, the mouse looks. Movement keys
 * are polled from the PlayerController each tick (so no project input mappings are
 * required), and mouse look is bound directly to the mouse axes via BindAxisKey.
 *
 * It renders nothing itself (camera only) -- the sim it flies around is drawn by
 * the transform-mirror renderer (AMjRenderer, Drive=stream). Spawned + possessed
 * by the launcher on -URLabVrViewer.
 */
UCLASS()
class URLAB_API ADroneViewerPawn : public APawn
{
	GENERATED_BODY()

public:
	ADroneViewerPawn();

	virtual void Tick(float DeltaSeconds) override;
	virtual void SetupPlayerInputComponent(class UInputComponent* InInputComponent) override;

	/** cm/s at full stick. Shift multiplies it (boost). */
	UPROPERTY(EditAnywhere, Category = "Drone")
	float MoveSpeed = 500.0f;

	UPROPERTY(EditAnywhere, Category = "Drone")
	float BoostMultiplier = 4.0f;

	UPROPERTY(EditAnywhere, Category = "Drone")
	float LookSensitivity = 1.5f;

private:
	UPROPERTY()
	TObjectPtr<class UCameraComponent> Camera;

	UPROPERTY()
	TObjectPtr<class UFloatingPawnMovement> Movement;

	void Turn(float Value);
	void LookUp(float Value);

	/** True while Ctrl is held: the cursor is shown and fly/look are suspended so
	 *  the mouse can drive the mirror's Ctrl+LMB drag-perturb (which deprojects the
	 *  cursor). Cleared on release, restoring free-fly. */
	bool bGrabMode = false;
};
