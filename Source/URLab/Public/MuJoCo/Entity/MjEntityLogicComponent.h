// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"

#include "MjEntityLogicComponent.generated.h"

class AMjEntity;

/**
 * The host for an entity's authored task logic (the microwave button->door rule, and its kind).
 *
 * It is PART OF THE ASSET: authored in the editor on the articulation and saved with it, then
 * re-instantiated onto the thin runtime AMjEntity at build by MjEntityHandoff::TransferAuthoredLogic.
 * The heavy mesh/component tree stays editor-only; only the logic rides the runtime entity.
 *
 * Logic runs against the entity's IMjEntity handle surface: resolve parts once when the entity is
 * ready (OnConstruct shape), then act each tick (`if Button.Pos() > 0.02: Motor.SetCtrl(5.0)`).
 * Blueprint subclasses override the BlueprintImplementableEvent hooks; C++ subclasses override the
 * Native* virtuals. A headless / Mirror entity with no authored logic hosts none of these.
 */
UCLASS(ClassGroup = (MuJoCo), meta = (BlueprintSpawnableComponent))
class URLAB_API UMjEntityLogicComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UMjEntityLogicComponent();

	/**
	 * The entity this logic scripts against, by its stable model-derived name. Set on the authoring
	 * articulation and re-stamped by the handoff so the component resolves the runtime entity it was
	 * transferred onto.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Entity")
	FName OwnerEntityName;

	/** The resolved runtime entity, cached at BeginPlay. Null until the entity is found. */
	UFUNCTION(BlueprintPure, Category = "MuJoCo|Entity")
	AMjEntity* GetEntity() const { return CachedEntity.Get(); }

	// --- Authored logic hooks (the microwave shape) ------------------------- //

	/** Resolve part handles here (OnConstruct shape): Button = Joint("button"); Motor = Actuator(...). */
	UFUNCTION(BlueprintImplementableEvent, Category = "MuJoCo|Entity", meta = (DisplayName = "On Entity Ready"))
	void ReceiveEntityReady(AMjEntity* Entity);

	/** Per-tick behaviour (Tick shape): if Button.Pos() > 0.02: Motor.SetCtrl(5.0). */
	UFUNCTION(BlueprintImplementableEvent, Category = "MuJoCo|Entity", meta = (DisplayName = "On Entity Tick"))
	void ReceiveEntityTick(AMjEntity* Entity, float DeltaSeconds);

	/** C++ counterpart to ReceiveEntityReady; native logic subclasses override this. */
	virtual void NativeOnEntityReady(AMjEntity* Entity) {}

	/** C++ counterpart to ReceiveEntityTick; native logic subclasses override this. */
	virtual void NativeOnEntityTick(AMjEntity* Entity, float DeltaSeconds) {}

protected:
	virtual void BeginPlay() override;

	virtual void TickComponent(
		float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	/** Resolved once at BeginPlay via AAMjManager::GetEntity(OwnerEntityName). */
	TWeakObjectPtr<AMjEntity> CachedEntity;
};
