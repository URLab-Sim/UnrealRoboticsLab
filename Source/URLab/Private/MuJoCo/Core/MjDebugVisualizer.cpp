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
#include "MuJoCo/Utils/MjColor.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Components/Geometry/MjGeom.h"
#include "MuJoCo/Components/QuickConvert/MjQuickConvertComponent.h"
#include "DrawDebugHelpers.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Utils/URLabLogging.h"

UMjDebugVisualizer::UMjDebugVisualizer()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UMjDebugVisualizer::BeginPlay()
{
    Super::BeginPlay();
    InitializeOverlayMaterial();
}

void UMjDebugVisualizer::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    UpdateBodyOverlays();

    if (bGlobalDrawTendons)
    {
        DrawTendonLines();
    }

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

    DebugData.BodyAwake.SetNumUninitialized(Model->nbody);
    for (int32 i = 0; i < Model->nbody; ++i)
    {
        DebugData.BodyAwake[i] = Data->body_awake ? Data->body_awake[i] : 1;
    }

    // Per-body Halton seed, matching MuJoCo's native islandColor algorithm in
    // engine_vis_visualize.c: use the active constraint island's dof-address if
    // available, otherwise fall back to the kinematic tree's dof-address so
    // sleeping bodies keep a stable colour. When asleep, mj_sleepCycle collapses
    // connected sleep-cycle trees to their smallest index so a group of bodies
    // resting on each other share one colour.
    DebugData.BodyIslandSeed.Init(-1, Model->nbody);
    const bool bSleepEnabled = (Model->opt.enableflags & mjENBL_SLEEP) != 0;
    for (int32 b = 0; b < Model->nbody; ++b)
    {
        const int32 WeldId = Model->body_weldid ? Model->body_weldid[b] : b;
        if (WeldId < 0 || WeldId >= Model->nbody) continue;
        if (!Model->body_dofnum || Model->body_dofnum[WeldId] == 0) continue;

        const int32 Dof = Model->body_dofadr ? Model->body_dofadr[WeldId] : -1;
        if (Dof < 0 || Dof >= Model->nv) continue;

        const int32 Island = (Data->nisland > 0 && Data->dof_island) ? Data->dof_island[Dof] : -1;
        int32 H = (Island >= 0 && Data->island_dofadr) ? Data->island_dofadr[Island] : -1;

        if (H == -1 && bSleepEnabled && Model->dof_treeid && Model->tree_dofadr)
        {
            int32 Tree = Model->dof_treeid[Dof];
            const bool bBodyAwake = Data->body_awake ? (Data->body_awake[b] != 0) : true;
            if (!bBodyAwake && Data->tree_asleep && Model->ntree > 0 &&
                Tree >= 0 && Tree < Model->ntree)
            {
                // Reimplementation of MuJoCo's mj_sleepCycle (engine_sleep.c).
                int32 Smallest = Tree;
                int32 Current = Tree;
                for (int32 Count = 0; Count <= Model->ntree; ++Count)
                {
                    const int32 Next = Data->tree_asleep[Current];
                    if (Next < 0 || Next >= Model->ntree) { Smallest = -1; break; }
                    if (Next < Smallest) Smallest = Next;
                    Current = Next;
                    if (Current == Tree) break;
                }
                Tree = Smallest;
            }
            if (Tree >= 0 && Tree < Model->ntree)
            {
                H = Model->tree_dofadr[Tree];
            }
        }

        DebugData.BodyIslandSeed[b] = H;
    }

    // Tendon wrap points — mirror MuJoCo's own renderer (engine_vis_visualize.c
    // addSpatialTendonGeoms). It iterates wrap-point index j from ten_wrapadr[t]
    // to ...+ten_wrapnum[t]-1 and connects wrap_xpos[3*j] -> wrap_xpos[3*j+3],
    // skipping pulleys (wrap_obj == -2). Declared layout is `nwrap x 6` so we
    // snapshot 2*nwrap 3D points.
    const int32 NumWrapPoints = 2 * Model->nwrap;
    DebugData.WrapPointsFlat.SetNumUninitialized(NumWrapPoints);
    if (Data->wrap_xpos && Model->nwrap > 0)
    {
        for (int32 p = 0; p < NumWrapPoints; ++p)
        {
            DebugData.WrapPointsFlat[p] = MjUtils::MjToUEPosition(Data->wrap_xpos + p * 3);
        }
    }

    const int32 NumWrapObj = 2 * Model->nwrap;
    DebugData.WrapObj.SetNumUninitialized(NumWrapObj);
    if (Data->wrap_obj && Model->nwrap > 0)
    {
        for (int32 i = 0; i < NumWrapObj; ++i) DebugData.WrapObj[i] = Data->wrap_obj[i];
    }

    DebugData.TendonWrapAdr.SetNumUninitialized(Model->ntendon);
    DebugData.TendonWrapNum.SetNumUninitialized(Model->ntendon);
    DebugData.TendonLength.SetNumUninitialized(Model->ntendon);
    DebugData.TendonLimited.SetNumUninitialized(Model->ntendon);
    DebugData.TendonRangeLo.SetNumUninitialized(Model->ntendon);
    DebugData.TendonRangeHi.SetNumUninitialized(Model->ntendon);
    for (int32 t = 0; t < Model->ntendon; ++t)
    {
        DebugData.TendonWrapAdr[t]  = Data->ten_wrapadr ? Data->ten_wrapadr[t] : 0;
        DebugData.TendonWrapNum[t]  = Data->ten_wrapnum ? Data->ten_wrapnum[t] : 0;
        DebugData.TendonLength[t]   = Data->ten_length ? (float)Data->ten_length[t] : 0.0f;
        DebugData.TendonLimited[t]  = Model->tendon_limited ? Model->tendon_limited[t] : 0;
        DebugData.TendonRangeLo[t]  = Model->tendon_range ? (float)Model->tendon_range[t * 2 + 0] : 0.0f;
        DebugData.TendonRangeHi[t]  = Model->tendon_range ? (float)Model->tendon_range[t * 2 + 1] : 0.0f;
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

void UMjDebugVisualizer::CycleDebugShaderMode()
{
    const uint8 Count = static_cast<uint8>(EMjDebugShaderMode::SemanticSegmentation) + 1;
    DebugShaderMode = static_cast<EMjDebugShaderMode>((static_cast<uint8>(DebugShaderMode) + 1) % Count);

    const TCHAR* Label = TEXT("Off");
    switch (DebugShaderMode)
    {
        case EMjDebugShaderMode::Island:                Label = TEXT("Island"); break;
        case EMjDebugShaderMode::InstanceSegmentation:  Label = TEXT("Instance Segmentation"); break;
        case EMjDebugShaderMode::SemanticSegmentation:  Label = TEXT("Semantic Segmentation"); break;
        default: break;
    }
    UE_LOG(LogURLab, Log, TEXT("Debug shader mode: %s"), Label);
}

void UMjDebugVisualizer::ToggleTendons()
{
    bGlobalDrawTendons = !bGlobalDrawTendons;
    UE_LOG(LogURLab, Log, TEXT("Tendon rendering: %s"), bGlobalDrawTendons ? TEXT("ON") : TEXT("OFF"));
}

void UMjDebugVisualizer::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    ClearBodyOverlays();
    Super::EndPlay(EndPlayReason);
}

void UMjDebugVisualizer::ClearBodyOverlays()
{
    for (auto& Pair : OriginalMaterials)
    {
        UStaticMeshComponent* Mesh = Pair.Key.Get();
        if (!Mesh) continue;
        Mesh->SetMaterial(0, Pair.Value);

        if (const TMap<int32, UMaterialInterface*>* Extra = OriginalSlotMaterials.Find(Pair.Key))
        {
            for (const auto& SlotPair : *Extra)
            {
                Mesh->SetMaterial(SlotPair.Key, SlotPair.Value);
            }
        }
    }
    OriginalMaterials.Reset();
    OriginalSlotMaterials.Reset();
    ActiveMIDs.Reset();
}

void UMjDebugVisualizer::UpdateBodyOverlays()
{
    if (DebugShaderMode == EMjDebugShaderMode::Off)
    {
        if (OriginalMaterials.Num() > 0) ClearBodyOverlays();
        return;
    }

    if (!OverlayParentMaterial || OverlayColorParam.IsNone()) return;

    AAMjManager* Manager = Cast<AAMjManager>(GetOwner());
    if (!Manager) return;

    TArray<int32> LocalAwake;
    TArray<int32> LocalIslandSeed;
    {
        FScopeLock Lock(&DebugMutex);
        LocalAwake = DebugData.BodyAwake;
        LocalIslandSeed = DebugData.BodyIslandSeed;
    }

    auto ApplyToMesh = [&](UStaticMeshComponent* Mesh, int32 BodyId, uint32 GroupHash)
    {
        if (!Mesh) return;

        const bool bAwake =
            (LocalAwake.IsValidIndex(BodyId) ? LocalAwake[BodyId] != 0 : true);
        const int32 Seed =
            (LocalIslandSeed.IsValidIndex(BodyId) ? LocalIslandSeed[BodyId] : -1);

        TWeakObjectPtr<UStaticMeshComponent> WeakMesh(Mesh);
        const int32 NumSlots = FMath::Max(1, Mesh->GetNumMaterials());

        if (!OriginalMaterials.Contains(WeakMesh))
        {
            OriginalMaterials.Add(WeakMesh, Mesh->GetMaterial(0));
            for (int32 SlotIdx = 1; SlotIdx < NumSlots; ++SlotIdx)
            {
                OriginalSlotMaterials.FindOrAdd(WeakMesh).Add(SlotIdx, Mesh->GetMaterial(SlotIdx));
            }
        }

        const bool bColourAsAwake = bAwake || !bModulateBySleep;
        FLinearColor Color;
        switch (DebugShaderMode)
        {
            case EMjDebugShaderMode::Island:
                Color = MjColor::IslandColor(Seed, bColourAsAwake,
                    SleepValueScale, SleepSaturationScale);
                break;
            case EMjDebugShaderMode::InstanceSegmentation:
                Color = MjColor::InstanceSegmentationColor(GroupHash, BodyId, bColourAsAwake,
                    SleepValueScale, SleepSaturationScale);
                break;
            case EMjDebugShaderMode::SemanticSegmentation:
                Color = MjColor::SemanticSegmentationColor(GroupHash, bColourAsAwake,
                    SleepValueScale, SleepSaturationScale);
                break;
            default:
                return;
        }

        UMaterialInstanceDynamic* MID = nullptr;
        if (UMaterialInstanceDynamic** Existing = ActiveMIDs.Find(WeakMesh))
        {
            MID = *Existing;
        }
        if (!MID)
        {
            MID = UMaterialInstanceDynamic::Create(OverlayParentMaterial, this);
            ActiveMIDs.Add(WeakMesh, MID);
        }

        for (int32 SlotIdx = 0; SlotIdx < NumSlots; ++SlotIdx)
        {
            if (Mesh->GetMaterial(SlotIdx) != MID)
            {
                Mesh->SetMaterial(SlotIdx, MID);
            }
        }

        MID->SetVectorParameterValue(OverlayColorParam, Color);
    };

    for (AMjArticulation* Art : Manager->GetAllArticulations())
    {
        if (!Art) continue;

        // Semantic grouping hashes the Blueprint class so two instances share colour.
        const uint32 ArtHash = GetTypeHash(Art->GetClass()->GetFName());

        TArray<UMjGeom*> Geoms;
        Art->GetRuntimeComponents<UMjGeom>(Geoms);
        for (UMjGeom* Geom : Geoms)
        {
            if (!Geom || !Geom->IsBound()) continue;
            const int32 BodyId = Geom->GetMj().body_id;

            ApplyToMesh(Geom->GetVisualizerMesh(), BodyId, ArtHash);

            TArray<USceneComponent*> Children;
            Geom->GetChildrenComponents(true, Children);
            for (USceneComponent* Child : Children)
            {
                if (UStaticMeshComponent* SMC = Cast<UStaticMeshComponent>(Child))
                {
                    ApplyToMesh(SMC, BodyId, ArtHash);
                }
            }
        }
    }

    for (UMjQuickConvertComponent* QC : Manager->GetAllQuickComponents())
    {
        if (!QC) continue;
        const int32 BodyId = QC->GetMjBodyId();
        if (BodyId < 0) continue;

        AActor* Owner = QC->GetOwner();
        if (!Owner) continue;

        TArray<UStaticMeshComponent*> MeshComps;
        Owner->GetComponents<UStaticMeshComponent>(MeshComps);

        // Semantic grouping hashes the first static mesh so props sharing a mesh read as one "type".
        uint32 GroupHash = GetTypeHash(Owner->GetClass()->GetFName());
        for (UStaticMeshComponent* SMC : MeshComps)
        {
            if (SMC && SMC->GetStaticMesh())
            {
                GroupHash = GetTypeHash(SMC->GetStaticMesh()->GetFName());
                break;
            }
        }

        for (UStaticMeshComponent* SMC : MeshComps)
        {
            ApplyToMesh(SMC, BodyId, GroupHash);
        }
    }
}

void UMjDebugVisualizer::DrawTendonLines()
{
    UWorld* World = GetWorld();
    if (!World) return;

    TArray<FVector> LocalPoints;
    TArray<int32> LocalWrapObj, LocalAdr, LocalNum;
    TArray<float> LocalLengths, LocalLo, LocalHi;
    TArray<uint8> LocalLimited;
    {
        FScopeLock Lock(&DebugMutex);
        LocalPoints = DebugData.WrapPointsFlat;
        LocalWrapObj = DebugData.WrapObj;
        LocalAdr = DebugData.TendonWrapAdr;
        LocalNum = DebugData.TendonWrapNum;
        LocalLengths = DebugData.TendonLength;
        LocalLimited = DebugData.TendonLimited;
        LocalLo = DebugData.TendonRangeLo;
        LocalHi = DebugData.TendonRangeHi;
    }

    const int32 NumTendons = LocalAdr.Num();
    for (int32 t = 0; t < NumTendons; ++t)
    {
        float Stretch = 0.5f;
        if (LocalLimited.IsValidIndex(t) && LocalLimited[t])
        {
            const float Len = LocalLengths[t];
            const float Lo = LocalLo[t];
            const float Hi = LocalHi[t];
            if (Hi > Lo + KINDA_SMALL_NUMBER)
            {
                Stretch = FMath::Clamp((Len - Lo) / (Hi - Lo), 0.0f, 1.0f);
            }
        }
        const FColor LineColor = FLinearColor::LerpUsingHSV(
            FLinearColor::Blue, FLinearColor::Red, Stretch).ToFColor(false);

        const int32 Adr = LocalAdr[t];
        const int32 Num = LocalNum[t];
        // Mirror MuJoCo's engine_vis_visualize.c: for each consecutive wrap
        // index pair (j, j+1), skip pulley endpoints (wrap_obj == -2), then
        // connect wrap_xpos[3*j] -> wrap_xpos[3*j + 3].
        for (int32 j = Adr; j + 1 < Adr + Num; ++j)
        {
            if (LocalWrapObj.IsValidIndex(j)     && LocalWrapObj[j]     == -2) continue;
            if (LocalWrapObj.IsValidIndex(j + 1) && LocalWrapObj[j + 1] == -2) continue;
            if (!LocalPoints.IsValidIndex(j + 1)) continue;
            const FVector& A = LocalPoints[j];
            const FVector& B = LocalPoints[j + 1];
            DrawDebugLine(World, A, B, LineColor, false, -1.0f, 0, TendonLineThickness);
        }
    }
}

void UMjDebugVisualizer::InitializeOverlayMaterial()
{
    if (OverlayParentMaterial) return;

    UMaterialInterface* Parent = LoadObject<UMaterialInterface>(
        nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));

    if (!Parent)
    {
        UE_LOG(LogURLab, Warning,
            TEXT("Debug viz: failed to load /Engine/BasicShapes/BasicShapeMaterial — overlays disabled"));
        return;
    }

    TArray<FMaterialParameterInfo> VecInfos;
    TArray<FGuid> Guids;
    Parent->GetAllVectorParameterInfo(VecInfos, Guids);

    if (VecInfos.Num() == 0)
    {
        UE_LOG(LogURLab, Warning,
            TEXT("Debug viz: BasicShapeMaterial exposes no vector params — overlays disabled"));
        return;
    }

    // Prefer well-known colour-param names so we don't accidentally drive an emissive tint etc.
    static const FName PreferredNames[] = {
        TEXT("Color"), TEXT("BaseColor"), TEXT("Tint"), TEXT("TintColor"), TEXT("DiffuseColor")
    };
    FName Chosen = NAME_None;
    for (const FName& Pref : PreferredNames)
    {
        for (const FMaterialParameterInfo& Info : VecInfos)
        {
            if (Info.Name == Pref) { Chosen = Pref; break; }
        }
        if (!Chosen.IsNone()) break;
    }
    if (Chosen.IsNone()) Chosen = VecInfos[0].Name;

    OverlayParentMaterial = Parent;
    OverlayColorParam = Chosen;
}
