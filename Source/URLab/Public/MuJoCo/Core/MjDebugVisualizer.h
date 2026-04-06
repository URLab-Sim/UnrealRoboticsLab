// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "MuJoCo/Core/MjDebugTypes.h"
#include "MjDebugVisualizer.generated.h"

// Forward declarations
class AAMjManager;
struct FMuJoCoDebugData;

/**
 * @class UMjDebugVisualizer
 * @brief Handles debug visualization for the MuJoCo simulation (contact forces, collision wireframes, etc.).
 */
UCLASS(ClassGroup=(MuJoCo), meta=(BlueprintSpawnableComponent))
class URLAB_API UMjDebugVisualizer : public UActorComponent
{
    GENERATED_BODY()

public:
    UMjDebugVisualizer();

    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

    // --- Debug rendering params ---

    /** @brief If true, draws contact force visualization. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Interp, Category = "MuJoCo|Debug")
    bool bShowDebug = false;

    /** @brief Scaling factor for contact force visualization. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Interp, Category = "MuJoCo|Debug")
    float DebugForceScale = 0.1f;

    /** @brief Maximum force value for clamping visual size. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Interp, Category = "MuJoCo|Debug")
    float DebugMaxForce = 1000.0f;

    /** @brief Size of the drawn contact point. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Interp, Category = "MuJoCo|Debug")
    float DebugContactPointSize = 5.0f;

    /** @brief Base thickness of the contact force arrow. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Interp, Category = "MuJoCo|Debug")
    float DebugContactArrowThickness = 1.0f;

    // --- Global toggle flags ---

    /** @brief Toggles debug collision drawing globally for all articulations. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Interp, Category = "MuJoCo|Debug")
    bool bGlobalDrawDebugCollision = false;

    /** @brief Toggles debug joint axis/range drawing globally for all articulations. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Interp, Category = "MuJoCo|Debug")
    bool bGlobalDrawDebugJoints = false;

    /** @brief Toggles debug Group 3 drawing globally for all articulations. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Interp, Category = "MuJoCo|Debug")
    bool bGlobalShowGroup3 = false;

    /** @brief Toggles debug collision drawing globally for all QuickConvert components. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Interp, Category = "MuJoCo|Debug")
    bool bGlobalQuickConvertCollision = false;

    // --- Thread-safe debug data ---
    FMuJoCoDebugData DebugData;
    FCriticalSection DebugMutex;

    // --- Methods ---

    /** @brief Captures debug info from m_data to DebugData. Called on Physics Thread. */
    void CaptureDebugData();

    /** @brief Applies global debug visibility settings to all active articulations. */
    void UpdateAllGlobalVisibility();

    /** @brief Toggle contact force visualization (key: 1). */
    void ToggleDebugContacts();

    /** @brief Toggle collision wireframes on articulations (key: 3). */
    void ToggleArticulationCollisions();

    /** @brief Toggle collision wireframes on Quick Convert objects (key: 5). */
    void ToggleQuickConvertCollisions();

    /** @brief Toggle joint axes on all articulations (key: 4). */
    void ToggleDebugJoints();

    /** @brief Toggle visual mesh visibility on all articulations (key: 2). */
    void ToggleVisuals();

private:
    bool bVisualsHidden = false;
};
