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

// Empirical probe: does Path 1 (mjs_getId from m_SpecElement) work in
// current MuJoCo, or does the v0.1-alpha-era comment in MjComponent.h
// ("mjs_getId can return stale/garbage IDs after mjs_attach merges")
// still hold?
//
// For each component, after Compile + PostSetup:
//   - Path 1 id   = mjs_getId(m_SpecElement)
//   - Path 2 id   = mj_name2id(model, type, m_prefix + name)
//   - Bind result = component's view.id
// Test logs all three via UE_LOG(Display) and decides Path 1 vs Path 2
// success per component. Test always passes — its job is to report,
// not to gate.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/AutomationEditorCommon.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Components/Bodies/MjWorldBody.h"
#include "MuJoCo/Components/Geometry/MjGeom.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/MjArticulation.h"
#include <mujoco/mujoco.h>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FMjBindingPathProbeTest,
    "URLab.MuJoCo.Binding.PathProbe",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

namespace
{
    enum class EPathResult : uint8 { Success, Stale, Failed, Skipped };

    struct FProbeResult
    {
        FString Component;
        FString MjType;
        int Path1Id;
        int Path2Id;
        int BindId;
        EPathResult Path1Outcome;
        EPathResult Path2Outcome;
    };

    const TCHAR* PathResultName(EPathResult R)
    {
        switch (R)
        {
        case EPathResult::Success: return TEXT("SUCCESS");
        case EPathResult::Stale:   return TEXT("STALE_ID");
        case EPathResult::Failed:  return TEXT("FAILED");
        case EPathResult::Skipped: return TEXT("SKIPPED");
        }
        return TEXT("?");
    }

    EPathResult ClassifyPath1(int Id, int MaxCount, void* SpecElement)
    {
        if (!SpecElement) return EPathResult::Skipped;
        if (Id < 0) return EPathResult::Failed;
        if (Id >= MaxCount) return EPathResult::Stale;
        return EPathResult::Success;
    }

    EPathResult ClassifyPath2(int Id)
    {
        return Id >= 0 ? EPathResult::Success : EPathResult::Failed;
    }

    int GetMjCountFor(const mjModel* m, mjtObj Type)
    {
        switch (Type)
        {
        case mjOBJ_BODY:     return m->nbody;
        case mjOBJ_JOINT:    return m->njnt;
        case mjOBJ_GEOM:     return m->ngeom;
        case mjOBJ_SITE:     return m->nsite;
        case mjOBJ_CAMERA:   return m->ncam;
        case mjOBJ_LIGHT:    return m->nlight;
        case mjOBJ_MESH:     return m->nmesh;
        case mjOBJ_HFIELD:   return m->nhfield;
        case mjOBJ_TEXTURE:  return m->ntex;
        case mjOBJ_MATERIAL: return m->nmat;
        case mjOBJ_PAIR:     return m->npair;
        case mjOBJ_EXCLUDE:  return m->nexclude;
        case mjOBJ_EQUALITY: return m->neq;
        case mjOBJ_TENDON:   return m->ntendon;
        case mjOBJ_ACTUATOR: return m->nu;
        case mjOBJ_SENSOR:   return m->nsensor;
        case mjOBJ_NUMERIC:  return m->nnumeric;
        case mjOBJ_TEXT:     return m->ntext;
        case mjOBJ_TUPLE:    return m->ntuple;
        case mjOBJ_KEY:      return m->nkey;
        case mjOBJ_PLUGIN:   return m->nplugin;
        default:             return 0;
        }
    }

    FProbeResult Probe(
        const TCHAR* Label,
        mjtObj Type,
        UMjComponent* Comp,
        const FString& Prefix,
        const mjModel* Model,
        int BindId)
    {
        FProbeResult R;
        R.Component = Label;
        R.MjType = FString::Printf(TEXT("%d"), (int)Type);
        R.BindId = BindId;

        const int MaxCount = GetMjCountFor(Model, Type);
        mjsElement* SpecEl = Comp->GetSpecElementForDiagnostics();
        R.Path1Id = SpecEl ? mjs_getId(SpecEl) : -1;
        R.Path1Outcome = ClassifyPath1(R.Path1Id, MaxCount, SpecEl);

        const FString NameToLookup = Comp->MjName.IsEmpty() ? Comp->GetName() : Comp->MjName;
        const FString PrefixedName = Prefix + NameToLookup;
        R.Path2Id = mj_name2id(Model, Type, TCHAR_TO_UTF8(*PrefixedName));
        R.Path2Outcome = ClassifyPath2(R.Path2Id);

        return R;
    }
}

bool FMjBindingPathProbeTest::RunTest(const FString& Parameters)
{
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false);
    if (!World) { AddError(TEXT("Failed to create temporary world.")); return false; }
    FWorldContext& WorldContext = GEngine->CreateNewWorldContext(EWorldType::Game);
    WorldContext.SetCurrentWorld(World);

    FActorSpawnParameters SpawnParams;
    AMjArticulation* Robot = World->SpawnActor<AMjArticulation>(SpawnParams);
    if (!Robot) { AddError(TEXT("Failed to spawn AMjArticulation.")); World->DestroyWorld(false); GEngine->DestroyWorldContext(World); return false; }

    UMjWorldBody* WorldBody = NewObject<UMjWorldBody>(Robot, TEXT("WorldBody"));
    Robot->SetRootComponent(WorldBody);
    WorldBody->RegisterComponent();

    UMjBody* RootBody = NewObject<UMjBody>(Robot, TEXT("RootBody"));
    RootBody->RegisterComponent();
    RootBody->AttachToComponent(WorldBody, FAttachmentTransformRules::KeepRelativeTransform);

    UMjGeom* Geom = NewObject<UMjGeom>(Robot, TEXT("TestGeom"));
    Geom->size = { 0.1f, 0.1f, 0.1f };
    Geom->bOverride_size = true;
    Geom->RegisterComponent();
    Geom->AttachToComponent(RootBody, FAttachmentTransformRules::KeepRelativeTransform);

    UMjJoint* Joint = NewObject<UMjJoint>(Robot, TEXT("TestJoint"));
    Joint->RegisterComponent();
    Joint->AttachToComponent(RootBody, FAttachmentTransformRules::KeepRelativeTransform);

    AAMjManager* Manager = World->SpawnActor<AAMjManager>(SpawnParams);
    if (!Manager) { AddError(TEXT("Failed to spawn AAMjManager.")); World->DestroyWorld(false); GEngine->DestroyWorldContext(World); return false; }

    // mj_compile fires inside Manager->Compile() (PhysicsEngine::Compile line 221).
    // mjs_getId on a spec element is only meaningful after that point.
    Manager->Compile();

    if (!Manager->IsInitialized())
    {
        AddError(TEXT("Manager->IsInitialized() returned false after Compile — mj_compile did not produce a valid model. Cannot probe."));
        World->DestroyWorld(false); GEngine->DestroyWorldContext(World); return false;
    }

    const mjModel* Model = Manager->PhysicsEngine ? Manager->PhysicsEngine->GetModel() : nullptr;
    if (!Model)
    {
        AddError(TEXT("PhysicsEngine->GetModel() returned null after Compile."));
        World->DestroyWorld(false); GEngine->DestroyWorldContext(World); return false;
    }
    if (Robot->bAttachFailed)
    {
        AddError(TEXT("Articulation reports bAttachFailed — mjs_attach did not move child spec into root. Probe would be meaningless."));
        World->DestroyWorld(false); GEngine->DestroyWorldContext(World); return false;
    }
    if (Model->nbody < 2 || Model->ngeom < 1 || Model->njnt < 1)
    {
        AddError(FString::Printf(
            TEXT("Compiled model is missing expected entities (nbody=%d, ngeom=%d, njnt=%d). Expected nbody>=2, ngeom>=1, njnt>=1. Probe aborted."),
            Model->nbody, Model->ngeom, Model->njnt));
        World->DestroyWorld(false); GEngine->DestroyWorldContext(World); return false;
    }

    const FString Prefix = Robot->GetPrefixForDiagnostics();
    UE_LOG(LogTemp, Display, TEXT("[PathProbe] Articulation prefix: '%s'"), *Prefix);
    UE_LOG(LogTemp, Display, TEXT("[PathProbe] Model counts: nbody=%d  ngeom=%d  njnt=%d  (mj_compile completed)"),
        Model->nbody, Model->ngeom, Model->njnt);

    TArray<FProbeResult> Results;
    Results.Add(Probe(TEXT("RootBody"), mjOBJ_BODY,  RootBody, Prefix, Model, RootBody->GetMjID()));
    Results.Add(Probe(TEXT("TestGeom"), mjOBJ_GEOM,  Geom,     Prefix, Model, Geom->GetMjID()));
    Results.Add(Probe(TEXT("TestJoint"),mjOBJ_JOINT, Joint,    Prefix, Model, Joint->GetMjID()));

    int Path1Successes = 0, Path1Stales = 0, Path1Failures = 0, Path1Skipped = 0;
    int Path2Successes = 0, Path2Failures = 0;
    int Path1MatchesBind = 0;
    int Path2MatchesBind = 0;

    for (const FProbeResult& R : Results)
    {
        UE_LOG(LogTemp, Display,
            TEXT("[PathProbe] [%s] BindId=%d  Path1Id=%d (%s)  Path2Id=%d (%s)  Path1==Bind:%d Path2==Bind:%d"),
            *R.Component, R.BindId, R.Path1Id, PathResultName(R.Path1Outcome),
            R.Path2Id, PathResultName(R.Path2Outcome),
            (int)(R.Path1Id == R.BindId && R.BindId >= 0),
            (int)(R.Path2Id == R.BindId && R.BindId >= 0));

        switch (R.Path1Outcome)
        {
        case EPathResult::Success: ++Path1Successes; break;
        case EPathResult::Stale:   ++Path1Stales; break;
        case EPathResult::Failed:  ++Path1Failures; break;
        case EPathResult::Skipped: ++Path1Skipped; break;
        }
        switch (R.Path2Outcome)
        {
        case EPathResult::Success: ++Path2Successes; break;
        default: ++Path2Failures; break;
        }
        if (R.Path1Outcome == EPathResult::Success && R.Path1Id == R.BindId) ++Path1MatchesBind;
        if (R.Path2Outcome == EPathResult::Success && R.Path2Id == R.BindId) ++Path2MatchesBind;
    }

    UE_LOG(LogTemp, Display, TEXT("[PathProbe] === SUMMARY (n=%d) ==="), Results.Num());
    UE_LOG(LogTemp, Display, TEXT("[PathProbe] Path1: success=%d  stale=%d  failed=%d  skipped=%d"),
        Path1Successes, Path1Stales, Path1Failures, Path1Skipped);
    UE_LOG(LogTemp, Display, TEXT("[PathProbe] Path2: success=%d  failed=%d"),
        Path2Successes, Path2Failures);
    UE_LOG(LogTemp, Display, TEXT("[PathProbe] Path1==Bind: %d/%d   Path2==Bind: %d/%d"),
        Path1MatchesBind, Results.Num(), Path2MatchesBind, Results.Num());

    if (Path1Stales == 0 && Path1Failures == 0 && Path1Successes == Results.Num())
    {
        UE_LOG(LogTemp, Display, TEXT("[PathProbe] VERDICT: Path 1 is reliable in this scenario. The 'stale IDs after mjs_attach' comment may be obsolete."));
    }
    else
    {
        UE_LOG(LogTemp, Display, TEXT("[PathProbe] VERDICT: Path 1 failed on %d/%d components. The fallback IS load-bearing."),
            Path1Stales + Path1Failures, Results.Num());
    }

    Manager->PhysicsEngine->bShouldStopTask = true;
    World->DestroyWorld(false);
    GEngine->DestroyWorldContext(World);
    return true;
}
