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
};
