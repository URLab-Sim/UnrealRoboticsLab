// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

class AMjArticulation;
class AMjEntity;
struct FMjEntity;

/**
 * The build-time bridge that demotes an authoring articulation to a thin runtime entity.
 *
 * These are free functions, not AMjManager members, so the runtime split (an editor-time scene
 * compiler vs a lean runtime session) stays clean: the manager orchestrates, the Entity module owns
 * the transfer. Called once per articulation at the manager's post-compile handoff, before the
 * articulation is retired.
 */
namespace MjEntityHandoff
{
	/**
	 * Re-instantiate each authored UMjEntityLogicComponent from the articulation onto the entity:
	 * copy the template with CopyPropertiesForUnrelatedObjects, stamp OwnerEntityName, and
	 * RegisterComponent so the logic runs on the runtime host. An articulation with no logic
	 * components adds nothing.
	 */
	URLAB_API void TransferAuthoredLogic(AMjArticulation* From, AMjEntity* To);

	/** Copy the articulation's per-instance debug-draw intent into the entity's partition record. */
	URLAB_API void TransferDebugFlags(const AMjArticulation* From, FMjEntity& Entity);

	/** Re-home the possess spring-arm / camera / twist configuration onto the runtime entity. */
	URLAB_API void TransferPossessConfig(const AMjArticulation* From, FName EntityName);
}
