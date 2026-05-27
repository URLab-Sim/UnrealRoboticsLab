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

#include "MuJoCo/Components/Geometry/Primitives/MjSphere.h"

#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "Utils/URLabLogging.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "UObject/ConstructorHelpers.h"

UMjSphere::UMjSphere()
{
	Type = EMjGeomType::Sphere;
	bOverride_Type = true;
}

void UMjSphere::EnsureVisualizerMesh()
{
    if (VisualizerMesh) return;

    VisualizerMesh = NewObject<UStaticMeshComponent>(this, TEXT("VisualizerMesh"));
    if (VisualizerMesh)
    {
        VisualizerMesh->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
        VisualizerMesh->SetCollisionResponseToAllChannels(ECR_Overlap);

        UStaticMesh* LoadedMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere"));
        if (LoadedMesh) VisualizerMesh->SetStaticMesh(LoadedMesh);

        if (IsRegistered())
        {
            VisualizerMesh->SetupAttachment(this);
            VisualizerMesh->RegisterComponent();
        }
    }
}

void UMjSphere::OnRegister()
{
    Super::OnRegister();
    EnsureVisualizerMesh();

    if (VisualizerMesh && !VisualizerMesh->IsRegistered())
    {
        VisualizerMesh->SetupAttachment(this);
        VisualizerMesh->RegisterComponent();
    }

    if (VisualizerMesh && OverrideMaterial && IsValid(OverrideMaterial) && GetOwner())
    {
        VisualizerMesh->SetMaterial(0, OverrideMaterial);
    }
}




void UMjSphere::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
        // --- CODEGEN_IMPORT_START ---

    // --- CODEGEN_IMPORT_END ---

	Super::ImportFromXml(Node, CompilerSettings);
	Radius = size.Num() > 0 ? size[0] : 0.0f;

    // Sync Unreal scale immediately on import so the editor visual matches the data
    const float BaseSize = 50.0f;
    const float UnitScale = 100.0f;
    FVector NewScale = FVector((Radius * UnitScale) / BaseSize);
    SetRelativeScale3D(NewScale);
}

void UMjSphere::ExportTo(mjsGeom* Element, mjsDefault* def)
{
    // Derive size from RelativeScale3D when (a) user-authored, or
    // (b) imported with an explicit size attr. The second condition lets
    // a user rescale an imported sphere in the viewport and have the
    // change reach the physics geom. Imported-with-inherited-size geoms
    // (bOverride_size=false) skip this so MuJoCo's class-default
    // inheritance still applies on compile.
    if (!bWasImported || bOverride_size)
    {
        const FVector scale = GetRelativeScale3D();
        size = { (float)scale.X * 0.5f };
        bOverride_size = true;
    }

	Super::ExportTo(Element, def);

        // --- CODEGEN_EXPORT_START ---

    // --- CODEGEN_EXPORT_END ---
}

void UMjSphere::ApplyOverrideMaterial(UMaterialInterface* Material)
{
    EnsureVisualizerMesh();
    if (VisualizerMesh && Material && IsValid(Material)) VisualizerMesh->SetMaterial(0, Material);
}

void UMjSphere::SyncUnrealTransformFromMj()
{
    if (m_GeomView.id == -1) return;

    if (!bOverride_size)
    {
        // MuJoCo sphere size is radius. Unreal Sphere is 100 units diameter.
        float RadiusVal = m_GeomView.size[0];
        FVector NewScale = FVector(RadiusVal * 2.0f);
        SetRelativeScale3D(NewScale);

        UE_LOG(LogURLabBind, Log, TEXT("[MjSphere] Syncing scale for '%s' from MuJoCo radius: %f -> NewScale: %s"), 
            *GetName(), RadiusVal, *NewScale.ToString());
    }
}

#if WITH_EDITOR
void UMjSphere::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

    FName PropertyName = (PropertyChangedEvent.Property != nullptr) ? PropertyChangedEvent.Property->GetFName() : NAME_None;
    FName MemberPropertyName = (PropertyChangedEvent.MemberProperty != nullptr) ? PropertyChangedEvent.MemberProperty->GetFName() : NAME_None;

    // Enforce uniform scaling for Sphere
    if (PropertyName == FName(TEXT("RelativeScale3D")) || MemberPropertyName == FName(TEXT("RelativeScale3D")))
    {
        FVector scale = GetRelativeScale3D();
        if (!scale.AllComponentsEqual())
        {
            // force Y and Z to match X
            scale.Y = scale.Z = scale.X;
            SetRelativeScale3D(scale);
        }
    }
}
#endif

void UMjSphere::SetGeomVisibility(bool bNewVisibility)
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
