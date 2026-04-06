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

#include "MuJoCo/Input/MjInputHandler.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjDebugVisualizer.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "Kismet/GameplayStatics.h"
#include "Cinematics/MjOrbitCameraActor.h"
#include "Cinematics/MjKeyframeCameraActor.h"
#include "MuJoCo/Components/Forces/MjImpulseLauncher.h"
#include "Utils/URLabLogging.h"

UMjInputHandler::UMjInputHandler()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UMjInputHandler::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    UWorld* World = GetWorld();
    if (!World) return;

    APlayerController* PC = World->GetFirstPlayerController();
    if (PC)
    {
        ProcessHotkeys(PC);
    }
}

void UMjInputHandler::ProcessHotkeys(APlayerController* PC)
{
    if (!PC) return;

    AAMjManager* Manager = Cast<AAMjManager>(GetOwner());
    if (!Manager) return;

    // Keys 1-5: Debug toggles (dispatched to DebugVisualizer)
    if (PC->WasInputKeyJustPressed(EKeys::One))
    {
        if (Manager->DebugVisualizer) Manager->DebugVisualizer->ToggleDebugContacts();
    }
    if (PC->WasInputKeyJustPressed(EKeys::Two))
    {
        if (Manager->DebugVisualizer) Manager->DebugVisualizer->ToggleVisuals();
    }
    if (PC->WasInputKeyJustPressed(EKeys::Three))
    {
        if (Manager->DebugVisualizer) Manager->DebugVisualizer->ToggleArticulationCollisions();
    }
    if (PC->WasInputKeyJustPressed(EKeys::Four))
    {
        if (Manager->DebugVisualizer) Manager->DebugVisualizer->ToggleDebugJoints();
    }
    if (PC->WasInputKeyJustPressed(EKeys::Five))
    {
        if (Manager->DebugVisualizer) Manager->DebugVisualizer->ToggleQuickConvertCollisions();
    }

    // P: Pause/Resume
    if (PC->WasInputKeyJustPressed(EKeys::P))
    {
        if (Manager->PhysicsEngine)
        {
            Manager->PhysicsEngine->SetPaused(!Manager->PhysicsEngine->bIsPaused);
            UE_LOG(LogURLab, Log, TEXT("Simulation: %s"), Manager->PhysicsEngine->bIsPaused ? TEXT("PAUSED") : TEXT("PLAYING"));
        }
    }

    // R: Reset
    if (PC->WasInputKeyJustPressed(EKeys::R))
    {
        Manager->ResetSimulation();
        UE_LOG(LogURLab, Log, TEXT("Simulation: RESET"));
    }

    // O: Toggle orbit/keyframe cameras
    if (PC->WasInputKeyJustPressed(EKeys::O))
    {
        UWorld* World = GetWorld();

        TArray<AActor*> OrbitCams;
        UGameplayStatics::GetAllActorsOfClass(World, AMjOrbitCameraActor::StaticClass(), OrbitCams);
        for (AActor* A : OrbitCams)
        {
            AMjOrbitCameraActor* Orbit = Cast<AMjOrbitCameraActor>(A);
            if (Orbit)
            {
                Orbit->ToggleOrbit();
                UE_LOG(LogURLab, Log, TEXT("Orbit camera toggled"));
            }
        }

        TArray<AActor*> KfCams;
        UGameplayStatics::GetAllActorsOfClass(World, AMjKeyframeCameraActor::StaticClass(), KfCams);
        for (AActor* A : KfCams)
        {
            AMjKeyframeCameraActor* KfCam = Cast<AMjKeyframeCameraActor>(A);
            if (KfCam)
            {
                KfCam->TogglePlayPause();
                UE_LOG(LogURLab, Log, TEXT("Keyframe camera: %s"), KfCam->IsPlaying() ? TEXT("PLAYING") : TEXT("PAUSED"));
            }
        }
    }

    // F: Fire impulse launchers
    if (PC->WasInputKeyJustPressed(EKeys::F))
    {
        UWorld* World = GetWorld();

        TArray<AActor*> Launchers;
        UGameplayStatics::GetAllActorsOfClass(World, AMjImpulseLauncher::StaticClass(), Launchers);
        for (AActor* A : Launchers)
        {
            AMjImpulseLauncher* Launcher = Cast<AMjImpulseLauncher>(A);
            if (Launcher)
            {
                Launcher->ResetAndFire();
                UE_LOG(LogURLab, Log, TEXT("Fired impulse launcher: %s"), *Launcher->GetName());
            }
        }
    }
}
