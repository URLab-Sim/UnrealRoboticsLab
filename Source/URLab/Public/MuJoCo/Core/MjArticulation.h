// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "MjArticulation.generated.h"

class UMJModelComponent;
class UMJBodyComponent;
class UMJGeomComponent;
class UMJJointComponent;
class UMJActuatorComponent;
class UMJSensorComponent;
class UMJTendonComponent;
class USceneComponent;

/**
 * One MuJoCo articulation/model actor in a level, backed by MuJoCoSpec components.
 */
UCLASS()
class URLAB_API AMjArticulation : public AActor
{
	GENERATED_BODY()

public:
	AMjArticulation();

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Model")
	TObjectPtr<USceneComponent> DefaultSceneRoot;

	/**
	 * Namespace prefix applied when attaching this articulation into the scene spec.
	 * Empty means no prefix.
	 */
	FString GetCompiledPrefix() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Runtime")
	TArray<UMJActuatorComponent*> GetActuators() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Runtime")
	TArray<UMJJointComponent*> GetJoints() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Runtime")
	TArray<UMJSensorComponent*> GetSensors() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Runtime")
	TArray<UMJTendonComponent*> GetTendons() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Runtime")
	TArray<UMJBodyComponent*> GetBodies() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Runtime")
	TArray<UMJGeomComponent*> GetGeoms() const;

	/** Show or hide group-3 geoms (collision meshes). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Visuals")
	bool bShowGroup3 = false;

	UFUNCTION(CallInEditor, Category = "MuJoCo|Visuals")
	void ToggleGroup3Visibility();

	void UpdateGroup3Visibility();

	/** Bridge-owned identifier echoed in the handshake. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Identity")
	FString ActorId;

protected:
	virtual void BeginPlay() override;

public:
#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
	virtual void OnConstruction(const FTransform& Transform) override;
#endif
};
