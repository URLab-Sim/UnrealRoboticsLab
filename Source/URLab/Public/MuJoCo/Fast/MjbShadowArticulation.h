// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"

struct mjModel_;
class AAMjManager;
class AMjArticulation;

/**
 * Build a geometry-less "shadow" articulation over a raw mjModel.
 *
 * The fast-path Direct instance installs a raw mjModel/mjData into the shared
 * UMjPhysicsEngine (no mjSpec, no import). The control/observation RPC layer and
 * the client handshake, however, address everything by articulation. This stands
 * up ONE lightweight AMjArticulation whose joint + actuator element components are
 * name-bound (mj_id2name / BindTo) to the installed raw model, so the existing
 * client drives it unchanged (set ctrl, read qpos/qvel/actuator state). It carries
 * no meshes/materials -- AMjbScene already renders.
 */
namespace URLabFastShadow
{
	/** Spawn + wire the shadow articulation into the manager's engine. Returns the
	 *  world-owned actor, or null on failure. ArtId becomes the public prefix the
	 *  client addresses (the handshake's ArtSegment). */
	AMjArticulation* Build(AAMjManager* Mgr, mjModel_* Model, const FString& ArtId);

	/** Retire a shadow articulation: unregister from the engine, unbind + clear its
	 *  elements, and destroy the actor. Safe with a null Art. */
	void Teardown(AAMjManager* Mgr, AMjArticulation* Art);
}
