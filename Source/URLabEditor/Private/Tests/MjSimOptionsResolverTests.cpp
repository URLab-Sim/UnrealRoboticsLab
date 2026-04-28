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

// Covers FMuJoCoOptions::Resolve across five conflict scenarios.
// Verifies the compiled mjModel->opt value; warning text is spot-checked
// manually in the test log.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjTestHelpers.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Components/Bodies/MjWorldBody.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Components/Geometry/MjGeom.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "Engine/World.h"
#include "mujoco/mujoco.h"

namespace
{
    AMjArticulation* SpawnSecondArticulation(FMjUESession& S, const FString& Name)
    {
        if (!S.World) return nullptr;

        FActorSpawnParameters P;
        P.Name = *Name;
        AMjArticulation* Bot = S.World->SpawnActor<AMjArticulation>(P);
        if (!Bot) return nullptr;

        UMjWorldBody* WB = NewObject<UMjWorldBody>(Bot, TEXT("WorldBody"));
        Bot->SetRootComponent(WB);
        WB->RegisterComponent();

        UMjBody* Body = NewObject<UMjBody>(Bot, TEXT("Body"));
        Body->RegisterComponent();
        Body->AttachToComponent(WB, FAttachmentTransformRules::KeepRelativeTransform);

        UMjGeom* Geom = NewObject<UMjGeom>(Bot, TEXT("Geom"));
        Geom->Size = FVector(0.1f, 0.1f, 0.1f);
        Geom->bOverride_Size = true;
        Geom->RegisterComponent();
        Geom->AttachToComponent(Body, FAttachmentTransformRules::KeepRelativeTransform);

        UMjJoint* Joint = NewObject<UMjJoint>(Bot, TEXT("Joint"));
        Joint->RegisterComponent();
        Joint->AttachToComponent(Body, FAttachmentTransformRules::KeepRelativeTransform);

        return Bot;
    }
}

// ============================================================================
// URLab.SimOptions.SingleArticulation
//   One articulation sets timestep; compiled model reflects that value.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSimOptionsSingleArticulation,
    "URLab.SimOptions.SingleArticulation",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjSimOptionsSingleArticulation::RunTest(const FString&)
{
    FMjUESession S;
    if (!S.Init([](FMjUESession& Sess)
    {
        Sess.Robot->SimOptions.bOverride_Timestep = true;
        Sess.Robot->SimOptions.Timestep = 0.003f;
    }))
    {
        AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
        S.Cleanup();
        return false;
    }

    mjModel* M = S.Manager->PhysicsEngine->m_model;
    TestNotNull(TEXT("Model"), M);
    if (M)
    {
        TestTrue(TEXT("Single-articulation timestep applied"),
            FMath::IsNearlyEqual((float)M->opt.timestep, 0.003f, 1e-6f));
    }

    S.Cleanup();
    return true;
}

// ============================================================================
// URLab.SimOptions.TwoArticulationsAgreeOnValue
//   Both articulations set timestep to the same value - silent apply,
//   value reaches the compiled model.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSimOptionsTwoArticulationsAgreeOnValue,
    "URLab.SimOptions.TwoArticulationsAgreeOnValue",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjSimOptionsTwoArticulationsAgreeOnValue::RunTest(const FString&)
{
    FMjUESession S;
    if (!S.Init([](FMjUESession& Sess)
    {
        Sess.Robot->SimOptions.bOverride_Timestep = true;
        Sess.Robot->SimOptions.Timestep = 0.0025f;

        AMjArticulation* Second = SpawnSecondArticulation(Sess, TEXT("SecondBot_Agree"));
        if (Second)
        {
            Second->SimOptions.bOverride_Timestep = true;
            Second->SimOptions.Timestep = 0.0025f;
        }
    }))
    {
        AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
        S.Cleanup();
        return false;
    }

    mjModel* M = S.Manager->PhysicsEngine->m_model;
    TestNotNull(TEXT("Model"), M);
    if (M)
    {
        TestTrue(TEXT("Agreed-value timestep applied to compiled model"),
            FMath::IsNearlyEqual((float)M->opt.timestep, 0.0025f, 1e-6f));
    }

    S.Cleanup();
    return true;
}

// ============================================================================
// URLab.SimOptions.TwoArticulationsDiffValues_HigherPriorityWins
//   ArmA(pri=5, timestep=0.002) vs ArmB(pri=2, timestep=0.005).
//   Compiled model must have 0.002. A [SimOptions] 'Timestep' conflict
//   WARNING is logged (verify in test run log).
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSimOptionsDiffValuesHigherPriorityWins,
    "URLab.SimOptions.TwoArticulationsDiffValues_HigherPriorityWins",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjSimOptionsDiffValuesHigherPriorityWins::RunTest(const FString&)
{
    FMjUESession S;
    if (!S.Init([](FMjUESession& Sess)
    {
        Sess.Robot->SimOptions.bOverride_Timestep = true;
        Sess.Robot->SimOptions.Timestep = 0.002f;
        Sess.Robot->SimOptionsPriority = 5;

        AMjArticulation* Second = SpawnSecondArticulation(Sess, TEXT("SecondBot_LowerPri"));
        if (Second)
        {
            Second->SimOptions.bOverride_Timestep = true;
            Second->SimOptions.Timestep = 0.005f;
            Second->SimOptionsPriority = 2;
        }
    }))
    {
        AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
        S.Cleanup();
        return false;
    }

    mjModel* M = S.Manager->PhysicsEngine->m_model;
    TestNotNull(TEXT("Model"), M);
    if (M)
    {
        TestTrue(TEXT("Higher-priority articulation's timestep wins (expected 0.002)"),
            FMath::IsNearlyEqual((float)M->opt.timestep, 0.002f, 1e-6f));
    }

    S.Cleanup();
    return true;
}

// ============================================================================
// URLab.SimOptions.TwoArticulationsSamePriority_ActorOrderWins
//   ArmA(pri=0, val=0.002) vs ArmB(pri=0, val=0.005). First wins by actor
//   iteration order. Two WARNINGs expected (conflict + ambiguous priority tie).
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSimOptionsSamePriorityActorOrderWins,
    "URLab.SimOptions.TwoArticulationsSamePriority_ActorOrderWins",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjSimOptionsSamePriorityActorOrderWins::RunTest(const FString&)
{
    FMjUESession S;
    if (!S.Init([](FMjUESession& Sess)
    {
        Sess.Robot->SimOptions.bOverride_Timestep = true;
        Sess.Robot->SimOptions.Timestep = 0.002f;
        Sess.Robot->SimOptionsPriority = 0;

        AMjArticulation* Second = SpawnSecondArticulation(Sess, TEXT("SecondBot_SamePri"));
        if (Second)
        {
            Second->SimOptions.bOverride_Timestep = true;
            Second->SimOptions.Timestep = 0.005f;
            Second->SimOptionsPriority = 0;
        }
    }))
    {
        AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
        S.Cleanup();
        return false;
    }

    mjModel* M = S.Manager->PhysicsEngine->m_model;
    TestNotNull(TEXT("Model"), M);
    if (M)
    {
        // Actor iteration order isn't formally guaranteed, so accept either
        // value as "tie resolved deterministically on this platform".
        const bool bRobotWon  = FMath::IsNearlyEqual((float)M->opt.timestep, 0.002f, 1e-6f);
        const bool bSecondWon = FMath::IsNearlyEqual((float)M->opt.timestep, 0.005f, 1e-6f);
        TestTrue(TEXT("Same-priority tie resolved to one of the two values"),
            bRobotWon || bSecondWon);

        if (!bRobotWon)
        {
            AddInfo(TEXT("Tie broke to second articulation; platform iteration "
                         "order differs from spawn order. Not a failure."));
        }
    }

    S.Cleanup();
    return true;
}

// ============================================================================
// URLab.SimOptions.ManagerOverrideTrumpsResolver
//   Manager sets timestep=0.010. Two articulations disagree (0.002 vs 0.005).
//   Compiled model should have 0.010 (Manager always wins). The resolver
//   still logs its conflict WARNING even though Manager will overwrite.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSimOptionsManagerOverrideTrumpsResolver,
    "URLab.SimOptions.ManagerOverrideTrumpsResolver",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjSimOptionsManagerOverrideTrumpsResolver::RunTest(const FString&)
{
    FMjUESession S;
    if (!S.Init([](FMjUESession& Sess)
    {
        Sess.Robot->SimOptions.bOverride_Timestep = true;
        Sess.Robot->SimOptions.Timestep = 0.002f;
        Sess.Robot->SimOptionsPriority = 1;

        AMjArticulation* Second = SpawnSecondArticulation(Sess, TEXT("SecondBot_MgrOverride"));
        if (Second)
        {
            Second->SimOptions.bOverride_Timestep = true;
            Second->SimOptions.Timestep = 0.005f;
            Second->SimOptionsPriority = 0;
        }

        Sess.Manager->PhysicsEngine->Options.bOverride_Timestep = true;
        Sess.Manager->PhysicsEngine->Options.Timestep = 0.010f;
    }))
    {
        AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
        S.Cleanup();
        return false;
    }

    mjModel* M = S.Manager->PhysicsEngine->m_model;
    TestNotNull(TEXT("Model"), M);
    if (M)
    {
        TestTrue(TEXT("Manager's timestep override wins over articulation resolution (expected 0.010)"),
            FMath::IsNearlyEqual((float)M->opt.timestep, 0.010f, 1e-6f));
    }

    S.Cleanup();
    return true;
}
