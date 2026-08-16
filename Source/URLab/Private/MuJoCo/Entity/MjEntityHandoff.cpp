// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Entity/MjEntityHandoff.h"

#include "Engine/Engine.h"

#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Entity/MjEntity.h"
#include "MuJoCo/Entity/MjEntityActor.h"
#include "MuJoCo/Entity/MjEntityLogicComponent.h"

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
	}
}
