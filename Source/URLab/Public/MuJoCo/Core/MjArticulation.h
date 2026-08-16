// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// One MuJoCo spec, spawned into a level.
//
// The spec is the component tree: `Spec` is its root element and every
// body, geom, joint and sensor under it is an element of it. The actor adds
// what a spec alone cannot carry -- a world placement, the staged control
// the physics worker reads, and the index that turns a compiled id back into
// the element that received it.
//
// It does not compile anything. A scene is one model with each participant
// attached into it under a prefix, so compiling is the engine's job and the
// articulation is a participant in it.

#include "CoreMinimal.h"

#include <atomic>

#include "GameFramework/Pawn.h"

#include "MuJoCo/Gen/MjEnums.gen.h"
#include "Templates/UniquePtr.h"

#include "MjArticulation.generated.h"

class UMjBody;
class UMjGeom;
class UMjNodeComponent;
class UMjModel;
struct FSpecRef;
struct FMjRenderSnapshot;
struct mjModel_;
struct mjData_;
typedef struct mjModel_ mjModel;
typedef struct mjData_ mjData;

/**
 * One compiled family's elements, by id and by every name they answer to.
 *
 * A USTRUCT with UPROPERTY members so the engine can see the components it
 * holds: a Blueprint recompile reinstances every one of them, and a plain
 * TObjectPtr the reflection system cannot reach is left pointing at the
 * objects that were replaced.
 */
USTRUCT()
struct FMjElementFamily
{
	GENERATED_BODY()

	UPROPERTY(Transient)
	TMap<int32, TObjectPtr<UMjNodeComponent>> ById;

	UPROPERTY(Transient)
	TMap<FString, TObjectPtr<UMjNodeComponent>> ByName;
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnMjSimulationReset);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(FOnMjCollision, UMjGeom*, SelfGeom, UMjGeom*, OtherGeom, FVector, ContactPos);

UCLASS(config = Game)
class URLAB_API AMjArticulation : public APawn
{
	GENERATED_BODY()

public:
	AMjArticulation();

	/** Ticks in editor viewports only while something wants to be drawn. */
	virtual bool ShouldTickIfViewportsOnly() const override;

	// --- Spec ----------------------------------------------------------- //

	/**
	 * This articulation's MuJoCo spec, as the component tree it is.
	 *
	 * There is no second artifact: what the importer read, what the details panel
	 * edits and what the writer emits are all this tree.
	 */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Spec")
	TObjectPtr<UMjModel> Spec;

	/** A handle on this articulation's spec, for the writer and the compiler. */
	FSpecRef GetSpec() const;

	/** Optional path to an MJCF this articulation was imported from. */
	UPROPERTY(EditAnywhere, Category = "MuJoCo Import")
	FFilePath MuJoCoXMLFile;

	/**
	 * What happens when this robot's model-wide settings disagree with the
	 * scene's, or unset to accept whatever the scene does.
	 *
	 * A scene has one `<option>`, one `<size>` and one `<compiler>`, and a robot
	 * that authored its own brings them along. MuJoCo settles that when the robot
	 * is attached: `warning` keeps the scene's value and says so, `merge` takes
	 * the safer of the two per field, and `error` refuses the attach.
	 *
	 * Set per robot rather than per scene because MuJoCo asks the question at
	 * each attach, so one robot can refuse to have its timestep replaced while
	 * the rest of the scene carries on warning.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Scene")
	TOptional<EMjConflict> AttachConflict;

	/**
	 * The prefix every compiled name of this participant carries.
	 *
	 * Derived rather than stored: the scene assembly attaches each participant
	 * under its actor name, so this is that name and nothing is free to disagree
	 * with it.
	 */
	FString GetCompiledPrefix() const;

	// --- Element index ------------------------------------------------------ //
	//
	// Which of this articulation's elements received which compiled id. Filled
	// by the engine's single pass over the compile binding, because that pass is
	// the only place both facts are known at once: the spec says what
	// element a node is, and the binding says what model family and id it got.

	/** Record that `Node` bound to `Id` in the compiled family `ObjType`. */
	void IndexBoundElement(UMjNodeComponent& Node, int32 ObjType, int32 Id);

	/** Forget every indexed element. */
	void ClearElementIndex();

	/** The element of family `ObjType` that bound to `Id`, or null. */
	UMjNodeComponent* GetComponentByMjId(int32 ObjType, int32 Id) const;

	/**
	 * The element of family `ObjType` known by `Name`, or null.
	 *
	 * Both the authored MJCF name and the Unreal component name resolve, because
	 * a Blueprint caller has the variable in front of it and a bridge caller has
	 * the name out of the model.
	 */
	UMjNodeComponent* GetComponentByName(int32 ObjType, const FString& Name) const;

	/** Every indexed element of family `ObjType`, in no particular order. */
	TArray<UMjNodeComponent*> GetComponentsOfFamily(int32 ObjType) const;

	// --- Runtime discovery -------------------------------------------------- //
	//
	// Every accessor answers from the element index, so it answers for the
	// compiled scene and only for elements that survived the compile. There is
	// no hand class for a joint, a sensor, an actuator or a tendon -- they carry
	// no per-instance state -- so they come back as the element base and the
	// runtime libraries (UMjJointRuntime, UMjSensorRuntime, UMjActuatorRuntime,
	// UMjTendonRuntime) are what reads them.

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Runtime")
	TArray<UMjNodeComponent*> GetActuators() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Runtime")
	TArray<UMjNodeComponent*> GetJoints() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Runtime")
	TArray<UMjNodeComponent*> GetSensors() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Runtime")
	TArray<UMjNodeComponent*> GetTendons() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Runtime")
	TArray<UMjBody*> GetBodies() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Runtime")
	TArray<UMjGeom*> GetGeoms() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Runtime")
	TArray<FString> GetActuatorNames() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Runtime")
	TArray<FString> GetJointNames() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Runtime")
	TArray<FString> GetSensorNames() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Runtime")
	TArray<FString> GetBodyNames() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Runtime")
	TArray<FString> GetTendonNames() const;

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
	UMjNodeComponent* GetActuator(const FString& Name) const;

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
	UMjNodeComponent* GetJoint(const FString& Name) const;

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
	UMjNodeComponent* GetSensor(const FString& Name) const;

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
	UMjNodeComponent* GetTendon(const FString& Name) const;

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
	UMjBody* GetBody(const FString& Name) const;

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
	UMjBody* GetBodyByMjId(int32 Id) const;

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime")
	UMjGeom* GetGeomByMjId(int32 Id) const;

	/**
	 * Every spec element on this actor of the schema element `TagName` names.
	 *
	 * The way to reach elements that never compile to an mjModel object and so
	 * are absent from the index -- `<frame>`, `<default>`, `<key>` -- and the
	 * only accessor that answers before a compile.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Discovery")
	TArray<UMjNodeComponent*> GetElementsByTag(const FString& TagName) const;

	// --- Sleep -------------------------------------------------------------- //

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Sleep")
	void WakeAll();

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Sleep")
	void SleepAll();

	// --- Keyframes ---------------------------------------------------------- //

	/** Every `<key>` element authored on this articulation. */
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Keyframes")
	TArray<UMjNodeComponent*> GetKeyframes() const;

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "MuJoCo|Keyframes")
	TArray<FString> GetKeyframeNames() const;

	/**
	 * Teleport to a named keyframe's joint angles, once.
	 *
	 * Free joints are left alone: `mj_resetDataKeyframe` would set the world
	 * position too, which throws the robot across the scene when all the caller
	 * wanted was a pose.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Keyframes")
	bool ResetToKeyframe(const FString& KeyframeName = TEXT(""));

	/**
	 * Drive continuously to a keyframe until released.
	 *
	 * Prefers the keyframe's `ctrl`, which holds the pose through the actuators.
	 * Falls back to injecting its `qpos` directly, which holds it kinematically.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Keyframes")
	bool HoldKeyframe(const FString& KeyframeName = TEXT(""));

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Keyframes")
	void StopHoldKeyframe();

	UFUNCTION(BlueprintPure, Category = "MuJoCo|Keyframes")
	bool IsHoldingKeyframe() const { return bHoldingKeyframe; }

	// --- Convenience one-liners --------------------------------------------- //

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime|Actuators")
	bool SetActuatorControl(const FString& ActuatorName, float Value);

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime|Actuators")
	FVector2D GetActuatorRange(const FString& ActuatorName) const;

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime|Joints")
	float GetJointAngle(const FString& JointName) const;

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime|Sensors")
	float GetSensorScalar(const FString& SensorName) const;

	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Runtime|Sensors")
	TArray<float> GetSensorReading(const FString& SensorName) const;

	// --- Presentation ------------------------------------------------------- //

	/** Forwards one engine snapshot to every body, so they all observe one frame. */
	void ApplyRenderState(const FMjRenderSnapshot& Snap);

	virtual void Tick(float DeltaTime) override;

	/** Show or hide group-3 geoms, which is where collision meshes conventionally live. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Visuals")
	bool bShowGroup3 = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Interp, Category = "MuJoCo|Debug")
	bool bDrawDebugCollision = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Interp, Category = "MuJoCo|Debug")
	bool bDrawDebugJoints = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Interp, Category = "MuJoCo|Debug")
	bool bDrawDebugSites = false;

	UFUNCTION(CallInEditor, Category = "MuJoCo|Visuals")
	void ToggleGroup3Visibility();

	void UpdateGroup3Visibility();

	void DrawDebugCollision();
	void DrawDebugJoints();
	void DrawDebugSites();

	// --- Possess camera ----------------------------------------------------- //

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

	// --- Identity ----------------------------------------------------------- //

	/**
	 * Bridge-owned identifier echoed in the handshake.
	 *
	 * Set at spawn time so policy code has a handle that survives Unreal's
	 * volatile actor naming. Empty when unset; the bridge falls back to the
	 * actor name.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Identity")
	FString ActorId;

	UPROPERTY(BlueprintAssignable, Category = "MuJoCo|Events")
	FOnMjSimulationReset OnSimulationReset;

	UPROPERTY(BlueprintAssignable, Category = "MuJoCo|Events")
	FOnMjCollision OnCollision;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MjArticulation")
	TObjectPtr<USceneComponent> DefaultSceneRoot;

protected:
	virtual void BeginPlay() override;

	/**
	 * Unreal fires this for every actor before any `BeginPlay` starts, which is
	 * why the twist controller is attached here: consumers built during a
	 * sibling actor's `BeginPlay` see it as soon as they look.
	 */
	virtual void PostInitializeComponents() override;

	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

public:
	virtual void PossessedBy(AController* NewController) override;
	virtual void UnPossessed() override;

#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void PostEditMove(bool bFinished) override;
#endif

private:
	/** Keyed by `mjtObj`. Sparse: a family with no elements has no entry. */
	UPROPERTY(Transient)
	TMap<int32, FMjElementFamily> ElementIndex;

	/** Set while this articulation drives a keyframe hold on the engine, for the UI toggle. */
	bool bHoldingKeyframe = false;
};
