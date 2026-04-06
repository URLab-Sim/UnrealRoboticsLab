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

#include "MuJoCo/Core/MjDebugVisualizer.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Components/QuickConvert/MjQuickConvertComponent.h"
#include "DrawDebugHelpers.h"
#include "Utils/URLabLogging.h"

UMjDebugVisualizer::UMjDebugVisualizer()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UMjDebugVisualizer::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (!bShowDebug) return;

    FMuJoCoDebugData LocalDebugData;
    {
        FScopeLock Lock(&DebugMutex);
        LocalDebugData = DebugData;
    }

    UWorld* World = GetWorld();
    if (!World) return;

    for (int i = 0; i < LocalDebugData.ContactPoints.Num(); ++i)
    {
        float Force = 0.0f;
        if (i < LocalDebugData.ContactForces.Num()) Force = LocalDebugData.ContactForces[i];

        float ClampedForce = FMath::Min(Force, DebugMaxForce);
        float VisualLength = ClampedForce * DebugForceScale;

        DrawDebugPoint(World, LocalDebugData.ContactPoints[i], DebugContactPointSize, FColor::Red, false, -1.0f);

        if (i < LocalDebugData.ContactNormals.Num())
        {
            float ArrowHeadSize = FMath::Clamp(VisualLength * 0.2f, 2.0f, 15.0f);

            DrawDebugDirectionalArrow(World,
                LocalDebugData.ContactPoints[i],
                LocalDebugData.ContactPoints[i] + LocalDebugData.ContactNormals[i] * VisualLength,
                ArrowHeadSize, FColor::Yellow, false, -1.0f, 0, DebugContactArrowThickness);
        }
    }
}

void UMjDebugVisualizer::CaptureDebugData()
{
    AAMjManager* Manager = Cast<AAMjManager>(GetOwner());
    if (!Manager || !Manager->PhysicsEngine || !Manager->PhysicsEngine->m_model || !Manager->PhysicsEngine->m_data) return;

    mjModel* Model = Manager->PhysicsEngine->m_model;
    mjData* Data = Manager->PhysicsEngine->m_data;

    FScopeLock Lock(&DebugMutex);

    DebugData.ContactPoints.Reset();
    DebugData.ContactNormals.Reset();
    DebugData.ContactForces.Reset();

    for (int i = 0; i < Data->ncon; ++i)
    {
        FVector Pos = MjUtils::MjToUEPosition(Data->contact[i].pos);
        DebugData.ContactPoints.Add(Pos);

        // Normal: first row of contact frame, Y-flipped for UE coordinate convention
        double* f = Data->contact[i].frame;
        FVector NormalArg(f[0], -f[1], f[2]);
        DebugData.ContactNormals.Add(NormalArg);

        mjtNum cforce[6];
        mj_contactForce(Model, Data, i, cforce);
        DebugData.ContactForces.Add((float)cforce[0]);
    }
}

void UMjDebugVisualizer::UpdateAllGlobalVisibility()
{
    AAMjManager* Manager = Cast<AAMjManager>(GetOwner());
    if (!Manager) return;

    for (AMjArticulation* Art : Manager->GetAllArticulations())
    {
        if (Art)
        {
            Art->bDrawDebugCollision = bGlobalDrawDebugCollision;
            Art->bDrawDebugJoints = bGlobalDrawDebugJoints;
            Art->bShowGroup3 = bGlobalShowGroup3;
            Art->UpdateGroup3Visibility();
        }
    }

    for (UMjQuickConvertComponent* QC : Manager->GetAllQuickComponents())
    {
        if (QC)
        {
            QC->m_debug_meshes = bGlobalQuickConvertCollision;
        }
    }
}

void UMjDebugVisualizer::ToggleDebugContacts()
{
    bShowDebug = !bShowDebug;
    UE_LOG(LogURLab, Log, TEXT("Debug contacts: %s"), bShowDebug ? TEXT("ON") : TEXT("OFF"));
}

void UMjDebugVisualizer::ToggleArticulationCollisions()
{
    bGlobalDrawDebugCollision = !bGlobalDrawDebugCollision;

    AAMjManager* Manager = Cast<AAMjManager>(GetOwner());
    if (!Manager) return;

    for (AMjArticulation* Art : Manager->GetAllArticulations())
    {
        if (Art)
        {
            Art->bDrawDebugCollision = bGlobalDrawDebugCollision;
        }
    }
    UE_LOG(LogURLab, Log, TEXT("Articulation collisions: %s"), bGlobalDrawDebugCollision ? TEXT("ON") : TEXT("OFF"));
}

void UMjDebugVisualizer::ToggleQuickConvertCollisions()
{
    bGlobalQuickConvertCollision = !bGlobalQuickConvertCollision;

    AAMjManager* Manager = Cast<AAMjManager>(GetOwner());
    if (!Manager) return;

    for (UMjQuickConvertComponent* QC : Manager->GetAllQuickComponents())
    {
        if (QC)
        {
            QC->m_debug_meshes = bGlobalQuickConvertCollision;
        }
    }
    UE_LOG(LogURLab, Log, TEXT("QuickConvert collisions: %s"), bGlobalQuickConvertCollision ? TEXT("ON") : TEXT("OFF"));
}

void UMjDebugVisualizer::ToggleDebugJoints()
{
    bGlobalDrawDebugJoints = !bGlobalDrawDebugJoints;

    AAMjManager* Manager = Cast<AAMjManager>(GetOwner());
    if (!Manager) return;

    for (AMjArticulation* Art : Manager->GetAllArticulations())
    {
        if (Art)
        {
            Art->bDrawDebugJoints = bGlobalDrawDebugJoints;
        }
    }
    UE_LOG(LogURLab, Log, TEXT("Debug joints: %s"), bGlobalDrawDebugJoints ? TEXT("ON") : TEXT("OFF"));
}

void UMjDebugVisualizer::ToggleVisuals()
{
    bVisualsHidden = !bVisualsHidden;

    AAMjManager* Manager = Cast<AAMjManager>(GetOwner());
    if (!Manager) return;

    for (AMjArticulation* Art : Manager->GetAllArticulations())
    {
        if (!Art) continue;
        TArray<UStaticMeshComponent*> MeshComps;
        Art->GetComponents<UStaticMeshComponent>(MeshComps);
        for (UStaticMeshComponent* SMC : MeshComps)
        {
            SMC->SetVisibility(!bVisualsHidden);
        }
    }
    UE_LOG(LogURLab, Log, TEXT("Visuals: %s"), bVisualsHidden ? TEXT("HIDDEN") : TEXT("VISIBLE"));
}
