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

#include "MuJoCo/Components/Geometry/Primitives/MjBox.h"

#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "Utils/URLabLogging.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "UObject/ConstructorHelpers.h"

UMjBox::UMjBox()
{
	Type = EMjGeomType::Box;
	bOverride_Type = true;
}

void UMjBox::EnsureVisualizerMesh()
{
    if (VisualizerMesh) return;

    VisualizerMesh = NewObject<UStaticMeshComponent>(this, TEXT("VisualizerMesh"));
    if (VisualizerMesh)
    {
        VisualizerMesh->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
        VisualizerMesh->SetCollisionResponseToAllChannels(ECR_Overlap);

        UStaticMesh* LoadedMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
        if (LoadedMesh) VisualizerMesh->SetStaticMesh(LoadedMesh);

        if (IsRegistered())
        {
            VisualizerMesh->SetupAttachment(this);
            VisualizerMesh->RegisterComponent();
        }
    }
}

void UMjBox::OnRegister()
{
    Super::OnRegister();
    EnsureVisualizerMesh();

    if (VisualizerMesh && !VisualizerMesh->IsRegistered())
    {
        VisualizerMesh->SetupAttachment(this);
        VisualizerMesh->RegisterComponent();
    }

    // Apply override material after mesh is created and registered
    if (VisualizerMesh && OverrideMaterial && IsValid(OverrideMaterial) && GetOwner())
    {
        VisualizerMesh->SetMaterial(0, OverrideMaterial);
    }
}


void UMjBox::SetGeomVisibility(bool bNewVisibility)
{
    if (VisualizerMesh)
    {
        VisualizerMesh->SetVisibility(bNewVisibility, false);
        VisualizerMesh->bHiddenInGame = !bNewVisibility;
        
#if WITH_EDITOR
        VisualizerMesh->Modify();
        VisualizerMesh->MarkRenderStateDirty();
        VisualizerMesh->RecreateRenderState_Concurrent();
#endif
    }
}



void UMjBox::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
        // --- CODEGEN_IMPORT_START ---

    // --- CODEGEN_IMPORT_END ---

	Super::ImportFromXml(Node, CompilerSettings);
	// MuJoCo box size is 3 half-extents in metres.
	Extents = FVector(
		size.Num() > 0 ? size[0] : 0.0f,
		size.Num() > 1 ? size[1] : 0.0f,
		size.Num() > 2 ? size[2] : 0.0f);

    // Sync Unreal scale immediately on import so the editor visual matches the data
    const float BaseSize = 50.0f;
    const float UnitScale = 100.0f;
    FVector NewScale = (Extents * UnitScale) / BaseSize;
    SetRelativeScale3D(NewScale);
}

void UMjBox::ExportTo(mjsGeom* Element, mjsDefault* def)
{
    // See UMjSphere::ExportTo for the guard rationale: derive from scale
    // when user-authored or when imported with an explicit size attr; skip
    // when imported with inherited size so MuJoCo's class default applies.
    if (!bWasImported || bOverride_size)
    {
        const FVector scale = GetRelativeScale3D();
        size = { (float)scale.X * 0.5f, (float)scale.Y * 0.5f, (float)scale.Z * 0.5f };
        bOverride_size = true;
    }

	Super::ExportTo(Element, def);

        // --- CODEGEN_EXPORT_START ---

    // --- CODEGEN_EXPORT_END ---
}

void UMjBox::ApplyOverrideMaterial(UMaterialInterface* Material)
{
    EnsureVisualizerMesh();
    if (VisualizerMesh && Material && IsValid(Material))
    {
        VisualizerMesh->SetMaterial(0, Material);
    }
}

void UMjBox::SyncUnrealTransformFromMj()
{
    if (m_GeomView.id == -1) return;

    // If we are NOT an override, sync our Unreal scale from the resolved MuJoCo size (including defaults)
    if (!bOverride_size)
    {
        // MuJoCo box size is half-extents. Unreal Cube is 100 units.
        // Size 0.5 -> scale 1.0
        float r[3];
        r[0] = m_GeomView.size[0];
        r[1] = m_GeomView.size[1];
        r[2] = m_GeomView.size[2];

        FVector NewScale = FVector(r[0], r[1], r[2]) * 2.0f;
        SetRelativeScale3D(NewScale);

        UE_LOG(LogURLabBind, Log, TEXT("[MjBox] Syncing scale for '%s' from MuJoCo size: %f %f %f -> NewScale: %s"), 
            *GetName(), r[0], r[1], r[2], *NewScale.ToString());
    }
}

