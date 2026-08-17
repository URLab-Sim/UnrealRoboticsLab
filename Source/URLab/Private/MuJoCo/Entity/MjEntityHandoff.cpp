// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Entity/MjEntityHandoff.h"

#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EngineUtils.h"

#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Entity/MjEntity.h"
#include "MuJoCo/Entity/MjEntityActor.h"
#include "MuJoCo/Entity/MjEntityLogicComponent.h"
#include "MuJoCo/Entity/MjEntityPawn.h"
#include "MuJoCo/Input/MjTwistController.h"

namespace MjEntityHandoff
{
	void TransferAuthoredLogic(AMjArticulation* From, AMjEntity* To)
	{
		if (From == nullptr || To == nullptr)
		{
			return;
		}

		TArray<UMjEntityLogicComponent*> Authored;
		From->GetComponents<UMjEntityLogicComponent>(Authored);

		for (UMjEntityLogicComponent* Template : Authored)
		{
			if (Template == nullptr)
			{
				continue;
			}

			UMjEntityLogicComponent* Copy = To->AddLogicComponent(Template->GetClass());
			if (Copy == nullptr)
			{
				continue;
			}

			UEngine::CopyPropertiesForUnrelatedObjects(Template, Copy);
			Copy->OwnerEntityName = To->GetEntityName();
		}
	}

	void TransferDebugFlags(const AMjArticulation* From, FMjEntity& Entity)
	{
		if (From == nullptr)
		{
			return;
		}

		Entity.Overlay.bDrawDebugCollision = From->bDrawDebugCollision;
		Entity.Overlay.bDrawDebugJoints = From->bDrawDebugJoints;
		Entity.Overlay.bDrawDebugSites = From->bDrawDebugSites;
	}

	void TransferPossessConfig(const AMjArticulation* From, AMjEntity* To)
	{
		if (From == nullptr || To == nullptr)
		{
			return;
		}

		UWorld* World = To->GetWorld();
		if (World == nullptr)
		{
			return;
		}

		const FName EntityName = To->GetEntityName();

		AMjEntityPawn* Pawn = nullptr;
		for (TActorIterator<AMjEntityPawn> It(World); It; ++It)
		{
			if (It->OwnerEntityName == EntityName)
			{
				Pawn = *It;
				break;
			}
		}
		if (Pawn == nullptr)
		{
			FActorSpawnParameters SpawnParams;
			SpawnParams.ObjectFlags |= RF_Transient;
			Pawn = World->SpawnActor<AMjEntityPawn>(SpawnParams);
			if (Pawn == nullptr)
			{
				return;
			}
		}

		Pawn->OwnerEntityName = EntityName;
		Pawn->PossessCameraDistance = From->PossessCameraDistance;
		Pawn->PossessCameraPitch = From->PossessCameraPitch;
		Pawn->PossessCameraLagSpeed = From->PossessCameraLagSpeed;
		Pawn->PossessCameraRotationLagSpeed = From->PossessCameraRotationLagSpeed;
		Pawn->PossessCameraOffset = From->PossessCameraOffset;

		const UMjTwistController* Src = From->FindComponentByClass<UMjTwistController>();
		UMjTwistController* Dst = Pawn->FindComponentByClass<UMjTwistController>();
		if (Src != nullptr && Dst != nullptr)
		{
			Dst->MaxVx = Src->MaxVx;
			Dst->MaxVy = Src->MaxVy;
			Dst->MaxYawRate = Src->MaxYawRate;
			Dst->TwistMappingContext = Src->TwistMappingContext;
			Dst->MoveAction = Src->MoveAction;
			Dst->TurnAction = Src->TurnAction;
			Dst->ActionKeys = Src->ActionKeys;
		}
	}
}
