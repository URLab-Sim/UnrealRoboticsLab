#pragma once

#include "CoreMinimal.h"

/**
 * @struct FMuJoCoDebugData
 * @brief Thread-safe buffer for debug visualization data.
 */
struct FMuJoCoDebugData
{
    TArray<FVector> ContactPoints;
    TArray<FVector> ContactNormals;
    TArray<float> ContactForces;

    /** Per-body awake state snapshot (`mjData.body_awake`), size nbody. 0 = asleep. */
    TArray<int32> BodyAwake;

    /**
     * Halton colour seed per body. `island_dofadr[island]` when in an active
     * constraint island, else `tree_dofadr[tree]` (with `mj_sleepCycle` when
     * asleep). -1 means "don't colour" (e.g. worldbody). Size nbody.
     */
    TArray<int32> BodyIslandSeed;
};
