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

#include "MuJoCo/Components/Geometry/MjGeom.h"
#include "Components/StaticMeshComponent.h"
#include "MuJoCo/Utils/MjXmlUtils.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "XmlNode.h"
#include "Utils/URLabLogging.h"
#include "MuJoCo/Components/Defaults/MjDefault.h"
#include "MuJoCo/Components/Defaults/MujocoDefaults.h"
#include "MuJoCo/Core/Spec/MjSpecWrapper.h"
#include "Utils/IO.h"
#include "Utils/MeshUtils.h"
#include "Misc/ScopedSlowTask.h"
#include "Chaos/TriangleMeshImplicitObject.h"
#include "PhysicsEngine/BodySetup.h"

#if WITH_EDITOR
#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "Kismet2/BlueprintEditorUtils.h"
#include "AssetToolsModule.h"
#include "IAssetTools.h"
#include "AutomatedAssetImportData.h"
#endif


UMjGeom::UMjGeom()
{
	PrimaryComponentTick.bCanEverTick = false;
    
    // Default collision settings (matching MuJoCo defaults)
    contype = MujocoDefaults::Geom::Contype;
    conaffinity = MujocoDefaults::Geom::Conaffinity;
    bOverride_Pos = false;
    bOverride_Quat = false;
}










void UMjGeom::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
    if (!Node) return;

    // size + type are both codegen-owned now. The xml_enum block sets Type
    // first inside CODEGEN_IMPORT so the fromto canonicalisation can branch
    // on Type to pick size[1] vs size[2] for the half-length write.

        // --- CODEGEN_IMPORT_START ---
    MjXmlUtils::ReadAttrString(Node, TEXT("class"), MjClassName);
    { // xml_enum: type -> EMjGeomType
        FString S = Node->GetAttribute(TEXT("type"));
        S = S.ToLower();
        bool bMatched = false;
        if      (S == TEXT("plane")) { Type = EMjGeomType::Plane; bMatched = true; }
        else if (S == TEXT("hfield")) { Type = EMjGeomType::Hfield; bMatched = true; }
        else if (S == TEXT("sphere")) { Type = EMjGeomType::Sphere; bMatched = true; }
        else if (S == TEXT("capsule")) { Type = EMjGeomType::Capsule; bMatched = true; }
        else if (S == TEXT("ellipsoid")) { Type = EMjGeomType::Ellipsoid; bMatched = true; }
        else if (S == TEXT("cylinder")) { Type = EMjGeomType::Cylinder; bMatched = true; }
        else if (S == TEXT("box")) { Type = EMjGeomType::Box; bMatched = true; }
        else if (S == TEXT("mesh")) { Type = EMjGeomType::Mesh; bMatched = true; }
        else if (S == TEXT("sdf")) { Type = EMjGeomType::SDF; bMatched = true; }
        if (bMatched) bOverride_Type = true;
    }
    { // xml_enum: shellinertia -> EMjGeomInertia
        FString S = Node->GetAttribute(TEXT("shellinertia"));
        S = S.ToLower();
        bool bMatched = false;
        if      (S == TEXT("false")) { ShellInertia = EMjGeomInertia::Volume; bMatched = true; }
        else if (S == TEXT("true")) { ShellInertia = EMjGeomInertia::Shell; bMatched = true; }
        if (bMatched) bOverride_ShellInertia = true;
    }
    { // xml_enum: fluidshape -> EMjFluidShape
        FString S = Node->GetAttribute(TEXT("fluidshape"));
        S = S.ToLower();
        bool bMatched = false;
        if      (S == TEXT("none")) { FluidShape = EMjFluidShape::None; bMatched = true; }
        else if (S == TEXT("ellipsoid")) { FluidShape = EMjFluidShape::Ellipsoid; bMatched = true; }
        if (bMatched) bOverride_FluidShape = true;
    }
    MjXmlUtils::ReadAttrInt(Node, TEXT("contype"), contype, bOverride_contype);
    MjXmlUtils::ReadAttrInt(Node, TEXT("conaffinity"), conaffinity, bOverride_conaffinity);
    MjXmlUtils::ReadAttrInt(Node, TEXT("condim"), condim, bOverride_condim);
    MjXmlUtils::ReadAttrInt(Node, TEXT("group"), group, bOverride_group);
    MjXmlUtils::ReadAttrInt(Node, TEXT("priority"), priority, bOverride_priority);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("size"), size, bOverride_size);
    if (MjXmlUtils::ReadAttrString(Node, TEXT("material"), material)) bOverride_material = true;
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("friction"), friction, bOverride_friction);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("mass"), mass, bOverride_mass);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("density"), density, bOverride_density);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("solmix"), solmix, bOverride_solmix);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("solref"), solref, bOverride_solref);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("solimp"), solimp, bOverride_solimp);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("margin"), margin, bOverride_margin);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("gap"), gap, bOverride_gap);
    if (MjXmlUtils::ReadAttrString(Node, TEXT("hfield"), hfield)) bOverride_hfield = true;
    if (MjXmlUtils::ReadAttrString(Node, TEXT("mesh"), mesh)) bOverride_mesh = true;
    MjXmlUtils::ReadAttrDouble(Node, TEXT("fitscale"), fitscale, bOverride_fitscale);
    MjXmlUtils::ReadAttrColor(Node, TEXT("rgba"), rgba, bOverride_rgba);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("fluidcoef"), fluidcoef, bOverride_fluidcoef);
    MjUtils::ReadVec3InMeters(Node, TEXT("pos"), Pos, bOverride_Pos);
    { // canonicalize orientation (quat/euler/axisangle/xyaxes/zaxis)
        double TmpQuat[4] = {1.0, 0.0, 0.0, 0.0};
        if (MjOrientationUtils::OrientationToMjQuat(Node, CompilerSettings, TmpQuat))
        {
            Quat = MjUtils::MjToUERotation(TmpQuat);
            bOverride_Quat = true;
        }
    }
    { // canonicalize fromto -> Pos/Quat/size half-length
        FVector FTPos; FQuat FTQuat; float FTHalf = 0.f;
        if (MjUtils::DecomposeFromTo(Node, FTPos, FTQuat, FTHalf))
        {
            Pos = FTPos; bOverride_Pos = true;
            Quat = FTQuat; bOverride_Quat = true;
            const bool bFTZSlot = (Type == EMjGeomType::Box || Type == EMjGeomType::Ellipsoid);
            const int32 FTSlot = bFTZSlot ? 2 : 1;
            while (size.Num() <= FTSlot) { size.Add(-1.0f); }
            size[FTSlot] = FTHalf;
            bOverride_size = true;
        }
    }
    if (bOverride_Pos)  SetRelativeLocation(Pos);
    if (bOverride_Quat) SetRelativeRotation(Quat);
        // --- CODEGEN_IMPORT_END ---

    // Implicit mesh-type detection (mesh attr present but type wasn't set)
    if (!bOverride_Type && !mesh.IsEmpty())
    {
        Type = EMjGeomType::Mesh;
        bOverride_Type = true;
    }
    MeshName = mesh;

    bWasImported = true;
}

void UMjGeom::ExportTo(mjsGeom* Element, mjsDefault* Default)
{
    if (!Element) return;


    if (!bOverride_Type && !MeshName.IsEmpty())
    {
        Type = EMjGeomType::Mesh;
        bOverride_Type = true;
    }

    // Determine the FINAL effective type for the mesh-name decision. When
    // Type isn't overridden locally, fall back to the default's geom type.
    // (The codegen-owned xml_enum export below writes Element->type only when
    // bOverride_Type is true — exactly the desired behavior.)
    // --- CODEGEN_GEOM_FINAL_TYPE_START ---
    int FinalType = bOverride_Type
        ? mjGEOM_PLANE  /* placeholder; remapped below */
        : (Default ? Default->geom->type : mjGEOM_MESH);
    if (bOverride_Type)
    {
        switch (Type)
        {
            case EMjGeomType::Plane: FinalType = mjGEOM_PLANE; break;
            case EMjGeomType::Hfield: FinalType = mjGEOM_HFIELD; break;
            case EMjGeomType::Sphere: FinalType = mjGEOM_SPHERE; break;
            case EMjGeomType::Capsule: FinalType = mjGEOM_CAPSULE; break;
            case EMjGeomType::Ellipsoid: FinalType = mjGEOM_ELLIPSOID; break;
            case EMjGeomType::Cylinder: FinalType = mjGEOM_CYLINDER; break;
            case EMjGeomType::Box: FinalType = mjGEOM_BOX; break;
            case EMjGeomType::Mesh: FinalType = mjGEOM_MESH; break;
            case EMjGeomType::SDF: FinalType = mjGEOM_SDF; break;
        }
    }
    if (FinalType == mjGEOM_MESH && !MeshName.IsEmpty())
    {
        MjSetStringRaw(Element->meshname, MeshName);
    }
    // --- CODEGEN_GEOM_FINAL_TYPE_END ---

    // size: codegen-owned TArray<float>; the default per-attr export writes
    // Element->size[i] = size[i] up to size.Num(). Clamped to 3 by mjsGeom's
    // fixed-size array. Slots beyond size.Num() inherit defaults.

        // --- CODEGEN_EXPORT_START ---
    if (bOverride_Pos)
    {
        double TmpPos[3];
        MjUtils::UEToMjPosition(Pos, TmpPos);
        Element->pos[0] = TmpPos[0]; Element->pos[1] = TmpPos[1]; Element->pos[2] = TmpPos[2];
    }
    if (bOverride_Quat)
    {
        double TmpQuat[4];
        MjUtils::UEToMjRotation(Quat, TmpQuat);
        Element->quat[0] = TmpQuat[0]; Element->quat[1] = TmpQuat[1];
        Element->quat[2] = TmpQuat[2]; Element->quat[3] = TmpQuat[3];
    }
    if (bOverride_Type)
    {
        switch (Type)
        {
            case EMjGeomType::Plane: Element->type = (mjtGeom)mjGEOM_PLANE; break;
            case EMjGeomType::Hfield: Element->type = (mjtGeom)mjGEOM_HFIELD; break;
            case EMjGeomType::Sphere: Element->type = (mjtGeom)mjGEOM_SPHERE; break;
            case EMjGeomType::Capsule: Element->type = (mjtGeom)mjGEOM_CAPSULE; break;
            case EMjGeomType::Ellipsoid: Element->type = (mjtGeom)mjGEOM_ELLIPSOID; break;
            case EMjGeomType::Cylinder: Element->type = (mjtGeom)mjGEOM_CYLINDER; break;
            case EMjGeomType::Box: Element->type = (mjtGeom)mjGEOM_BOX; break;
            case EMjGeomType::Mesh: Element->type = (mjtGeom)mjGEOM_MESH; break;
            case EMjGeomType::SDF: Element->type = (mjtGeom)mjGEOM_SDF; break;
            default: break;
        }
    }
    if (bOverride_ShellInertia)
    {
        switch (ShellInertia)
        {
            case EMjGeomInertia::Volume: Element->typeinertia = (mjtGeomInertia)mjINERTIA_VOLUME; break;
            case EMjGeomInertia::Shell: Element->typeinertia = (mjtGeomInertia)mjINERTIA_SHELL; break;
            default: break;
        }
    }
    if (bOverride_FluidShape)
    {
        switch (FluidShape)
        {
            case EMjFluidShape::None: Element->fluid_ellipsoid = (mjtNum)0.0; break;
            case EMjFluidShape::Ellipsoid: Element->fluid_ellipsoid = (mjtNum)1.0; break;
            default: break;
        }
    }
    if (bOverride_contype) Element->contype = contype;
    if (bOverride_conaffinity) Element->conaffinity = conaffinity;
    if (bOverride_condim) Element->condim = condim;
    if (bOverride_group) Element->group = group;
    if (bOverride_priority) Element->priority = priority;
    if (bOverride_size) { for (int32 i = 0; i < FMath::Min(size.Num(), 3); ++i) { if (size[i] != -1.0f) Element->size[i] = size[i]; } }
    if (bOverride_friction) { for (int32 i = 0; i < FMath::Min(friction.Num(), 3); ++i) Element->friction[i] = friction[i]; }
    if (bOverride_mass) Element->mass = mass;
    if (bOverride_density) Element->density = density;
    if (bOverride_solmix) Element->solmix = solmix;
    if (bOverride_solref) { for (int32 i = 0; i < FMath::Min(solref.Num(), 2); ++i) Element->solref[i] = solref[i]; }
    if (bOverride_solimp) { for (int32 i = 0; i < FMath::Min(solimp.Num(), 5); ++i) Element->solimp[i] = solimp[i]; }
    if (bOverride_margin) Element->margin = margin;
    if (bOverride_gap) Element->gap = gap;
    if (bOverride_mesh) MjSetString(Element->meshname, mesh);
    if (bOverride_fitscale) Element->fitscale = fitscale;
    if (bOverride_rgba) { Element->rgba[0] = rgba.R; Element->rgba[1] = rgba.G; Element->rgba[2] = rgba.B; Element->rgba[3] = rgba.A; }
    if (bOverride_fluidcoef) { for (int32 i = 0; i < FMath::Min(fluidcoef.Num(), 5); ++i) Element->fluid_coefs[i] = fluidcoef[i]; }
        // --- CODEGEN_EXPORT_END ---
}

void UMjGeom::Bind(mjModel* Model, mjData* Data, const FString& Prefix)
{
    Super::Bind(Model, Data, Prefix);
    BindAndCacheView(m_GeomView, Prefix);
    if (m_GeomView.id != -1)
    {
        SyncUnrealTransformFromMj();
    }
}

void UMjGeom::UpdateGlobalTransform()
{
    if (m_GeomView._m && m_GeomView._d && m_GeomView.id >= 0)
    {
        // Get world position from MuJoCo
        FVector WorldPos = MjUtils::MjToUEPosition(m_GeomView.geom_xpos);
        FQuat WorldRot;
        mjtNum quat[4];
        mju_mat2Quat(quat, m_GeomView.geom_xmat);
        WorldRot = MjUtils::MjToUERotation(quat);
        
        SetWorldLocation(WorldPos);
        SetWorldRotation(WorldRot);
    }
}

FVector UMjGeom::GetWorldLocation() const
{
    if (m_GeomView._m && m_GeomView._d && m_GeomView.id >= 0)
    {
        return MjUtils::MjToUEPosition(m_GeomView.geom_xpos);
    }
    return GetComponentLocation();
}
void UMjGeom::SetFriction(float NewFriction)
{
    if (friction.Num() == 0) friction.Add(NewFriction);
    else friction[0] = NewFriction;
    bOverride_friction = true;
    
    // Update runtime model if bound
    if (m_GeomView._m && m_GeomView._d && m_GeomView.id >= 0 && m_GeomView.geom_friction)
    {
        m_GeomView.geom_friction[0] = NewFriction;
    }
}

void UMjGeom::SyncUnrealTransformFromMj()
{
    // Base implementation does nothing, specialized in primitive subtypes.
}

FString UMjGeom::GetResolvedMaterialName() const
{
    // Explicit material on this geom wins
    if (bOverride_material && !material.IsEmpty()) return material;

    // Walk the default class chain
    if (UMjDefault* Def = FindEditorDefault())
    {
        TSet<FString> Visited;
        UMjDefault* Cur = Def;
        while (Cur && !Visited.Contains(Cur->ClassName))
        {
            Visited.Add(Cur->ClassName);
            if (UMjGeom* G = Cur->FindChildOfType<UMjGeom>())
            {
                if (G->bOverride_material && !G->material.IsEmpty()) return G->material;
            }
            Cur = Cur->ParentClassName.IsEmpty() ? nullptr
                : FindDefaultByClassName(GetOwner(), Cur->ParentClassName);
        }
    }

    return FString(); // No material found
}

#if WITH_EDITOR
void UMjGeom::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);

    FName PropertyName = (PropertyChangedEvent.Property != nullptr) ? PropertyChangedEvent.Property->GetFName() : NAME_None;
    FName MemberPropertyName = (PropertyChangedEvent.MemberProperty != nullptr) ? PropertyChangedEvent.MemberProperty->GetFName() : NAME_None;

    // If the user manually changes the scale in the editor, mark it as an override.
    if (PropertyName == FName(TEXT("RelativeScale3D")) || MemberPropertyName == FName(TEXT("RelativeScale3D")))
    {
        // Don't auto-override if the scale hasn't actually changed meaningfully
        if (!GetRelativeScale3D().Equals(FVector(1.0f)))
        {
            bOverride_size = true;
            UE_LOG(LogURLab, Log, TEXT("[MjGeom] User manually changed scale for '%s'. Marking bOverride_size = true."), *GetName());
        }
    }

    // Sync MjClassName when DefaultClass changes
    if (PropertyName == GET_MEMBER_NAME_CHECKED(UMjGeom, DefaultClass))
    {
        if (DefaultClass)
            MjClassName = DefaultClass->ClassName;
        else
            MjClassName.Empty();
    }

    // Apply OverrideMaterial to the visual mesh
    if (PropertyName == GET_MEMBER_NAME_CHECKED(UMjGeom, OverrideMaterial) ||
        MemberPropertyName == GET_MEMBER_NAME_CHECKED(UMjGeom, OverrideMaterial))
    {
        ApplyOverrideMaterial(OverrideMaterial);
    }
}
#endif

void UMjGeom::ApplyOverrideMaterial(UMaterialInterface* Material)
{
    // Base implementation is a no-op. mesh geoms should not have their imported
    // materials overwritten — they already have materials from the import pipeline.
    // Primitive subclasses (Box, Sphere, Cylinder) override this with direct
    // VisualizerMesh access.
}

void UMjGeom::RegisterToSpec(FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody)
{
    if (!ParentBody) return;
    if (bDisabledByDecomposition) return; // Decomposed source geom — hull sub-geoms register instead

    // When MjClassName is empty, pass nullptr so MuJoCo applies the parent body's
    // childclass default automatically (MuJoCo 3.x spec API behaviour).
    // Only resolve an explicit default when a class name was explicitly set.
    mjsDefault* SpecDef = nullptr;
    if (!MjClassName.IsEmpty())
    {
        SpecDef = mjs_findDefault(Wrapper.Spec, TCHAR_TO_UTF8(*MjClassName));
        if (!SpecDef) SpecDef = mjs_getSpecDefault(Wrapper.Spec);
    }

    mjsGeom* geom = mjs_addGeom(ParentBody, SpecDef);
    m_SpecElement = geom->element;

    FString NameToRegister = MjName.IsEmpty() ? GetName() : MjName;
    FString UniqueName = Wrapper.GetUniqueName(NameToRegister, mjOBJ_GEOM, GetOwner());
    mjs_setName(geom->element, TCHAR_TO_UTF8(*UniqueName));
    MjName = UniqueName;

    TArray<FString> ExtraMeshNames;

    if (Type == EMjGeomType::Mesh)
    {
        if (bIsDecomposedHull && !MeshName.IsEmpty())
        {
            // Hull sub-geom: OBJ files already exist from editor decomposition.
            // MeshName format: "{AssetName}_{index}" → file: "Complex_{AssetName}_sub_{index}.obj"
            int32 LastUnderscore;
            FString BaseAssetName = MeshName;
            FString IndexStr = TEXT("0");
            if (MeshName.FindLastChar('_', LastUnderscore))
            {
                BaseAssetName = MeshName.Left(LastUnderscore);
                IndexStr = MeshName.Mid(LastUnderscore + 1);
            }

            FString SubDir = Wrapper.MeshCacheSubDir.IsEmpty() ? TEXT("Shared") : Wrapper.MeshCacheSubDir;
            FString ObjPath = FString::Printf(TEXT("%s/URLab/ConvertedMeshes/%s/Complex_%s_sub_%s.obj"),
                *FPaths::ProjectSavedDir(), *SubDir, *BaseAssetName, *IndexStr);
            FString ObjFullPath = FPaths::ConvertRelativePathToFull(ObjPath);

            if (FPaths::FileExists(ObjFullPath))
            {
                Wrapper.AddMeshAsset(MeshName, ObjFullPath, FVector::OneVector);
            }
            else
            {
                // Fallback: export the child visualization SMC as a simple convex mesh.
                // This re-exports the already-decomposed hull geometry, NOT re-running CoACD.
                UE_LOG(LogURLab, Warning, TEXT("[MjGeom] Hull OBJ not found at '%s'. Re-exporting from child SMC."), *ObjFullPath);
                TArray<USceneComponent*> FallbackChildren;
                GetChildrenComponents(true, FallbackChildren);
                for (USceneComponent* Child : FallbackChildren)
                {
                    if (UStaticMeshComponent* SMC = Cast<UStaticMeshComponent>(Child))
                    {
                        TArray<FString> Names = Wrapper.PrepareMeshForMuJoCo(SMC, false);
                        if (Names.Num() > 0)
                        {
                            MeshName = Names[0];
                        }
                        break;
                    }
                }
            }
        }
        else
        {
            // Normal geom: find child SMC and prepare mesh (may run CoACD for complex)
            TArray<USceneComponent*> Children;
            GetChildrenComponents(true, Children);

            for (USceneComponent* Child : Children)
            {
                if (UStaticMeshComponent* SMC = Cast<UStaticMeshComponent>(Child))
                {
                    TArray<FString> AssetNames = Wrapper.PrepareMeshForMuJoCo(SMC, bComplexMeshRequired);
                    if (AssetNames.Num() > 0)
                    {
                        MeshName = AssetNames[0];
                        for (int32 i = 1; i < AssetNames.Num(); ++i) ExtraMeshNames.Add(AssetNames[i]);
                        break;
                    }
                }
            }
        }
    }

    // PrepareMeshForMuJoCo / AddMeshAsset may rewrite MeshName to the
    // *resolved* bare asset name (e.g. XML "scene/white_table/left_foot_001"
    // -> "left_foot_001"). Sync the codegen-owned `mesh` UPROPERTY before
    // ExportTo so the codegen-emitted meshname write matches the asset
    // actually registered in the spec.
    if (Type == EMjGeomType::Mesh && !MeshName.IsEmpty())
    {
        mesh = MeshName;
        bOverride_mesh = true;
    }

    ExportTo(geom, SpecDef);

    // Extra mesh names from complex decomposition — hull sub-geoms should already
    // exist as persistent components (created by DecomposeMesh editor action).
    // They register themselves via their own RegisterToSpec calls.
    // Log a warning if we have extra meshes but no hull sub-geoms.
    if (ExtraMeshNames.Num() > 0)
    {
        UE_LOG(LogURLab, Warning, TEXT("[MjGeom] '%s' has %d extra mesh parts from complex decomposition. "
            "These should be persistent hull sub-geom components. Use 'Decompose mesh' in the editor."),
            *GetName(), ExtraMeshNames.Num());
    }
}

void UMjGeom::SetGeomVisibility(bool bNewVisibility)
{
    TArray<USceneComponent*> Children;
    GetChildrenComponents(true, Children);
    
    for (USceneComponent* Child : Children)
    {
        if (UStaticMeshComponent* MeshComp = Cast<UStaticMeshComponent>(Child))
        {
            MeshComp->SetVisibility(bNewVisibility, false);
            MeshComp->bHiddenInGame = !bNewVisibility;
            
#if WITH_EDITOR
            MeshComp->Modify();
            MeshComp->MarkRenderStateDirty();
            MeshComp->RecreateRenderState_Concurrent();
#endif
        }
    }
}

#if WITH_EDITOR
void UMjGeom::DecomposeMesh()
{
    UE_LOG(LogURLab, Log, TEXT("[MjGeom] DecomposeMesh called on '%s'. Type=%d, bOverride_Type=%d"),
        *GetName(), (int)Type, (int)bOverride_Type);

    if (Type != EMjGeomType::Mesh)
    {
        UE_LOG(LogURLab, Warning, TEXT("[MjGeom] DecomposeMesh: '%s' is not a mesh geom (Type=%d)."), *GetName(), (int)Type);
        return;
    }

    // Find the child StaticMeshComponent.
    // In the Blueprint editor, GetChildrenComponents() doesn't work on SCS templates —
    // we must walk the SCS node tree instead.
    UStaticMeshComponent* SMC = nullptr;

    // Try runtime attachment hierarchy first (works on placed instances)
    TArray<USceneComponent*> Children;
    GetChildrenComponents(true, Children);
    for (USceneComponent* Child : Children)
    {
        if (UStaticMeshComponent* Found = Cast<UStaticMeshComponent>(Child))
        {
            SMC = Found;
            break;
        }
    }

    // If not found, try SCS node tree (Blueprint editor context).
    // SCS templates aren't attached via the runtime hierarchy, so we walk
    // the outer chain to find the Blueprint and its SCS node tree.
    if (!SMC)
    {
        UBlueprint* BP = nullptr;

        // Walk outer chain to find the Blueprint
        for (UObject* Outer = GetOuter(); Outer; Outer = Outer->GetOuter())
        {
            if (UBlueprint* Found = Cast<UBlueprint>(Outer))
            {
                BP = Found;
                break;
            }
            if (UBlueprintGeneratedClass* BPGC = Cast<UBlueprintGeneratedClass>(Outer))
            {
                BP = Cast<UBlueprint>(BPGC->ClassGeneratedBy);
                break;
            }
        }

        if (BP && BP->SimpleConstructionScript)
        {
            USCS_Node* MyNode = nullptr;
            TArray<USCS_Node*> AllNodes = BP->SimpleConstructionScript->GetAllNodes();
            for (USCS_Node* Node : AllNodes)
            {
                if (Node->ComponentTemplate == this)
                {
                    MyNode = Node;
                    break;
                }
            }
            if (MyNode)
            {
                for (USCS_Node* ChildNode : MyNode->ChildNodes)
                {
                    if (UStaticMeshComponent* Found = Cast<UStaticMeshComponent>(ChildNode->ComponentTemplate))
                    {
                        SMC = Found;
                        break;
                    }
                }
                UE_LOG(LogURLab, Log, TEXT("[MjGeom] DecomposeMesh: SCS node '%s' has %d children. SMC found: %s"),
                    *MyNode->GetVariableName().ToString(), MyNode->ChildNodes.Num(),
                    SMC ? *SMC->GetName() : TEXT("none"));
            }
            else
            {
                UE_LOG(LogURLab, Warning, TEXT("[MjGeom] DecomposeMesh: Could not find SCS node for this component."));
            }
        }
        else
        {
            UE_LOG(LogURLab, Warning, TEXT("[MjGeom] DecomposeMesh: Could not find Blueprint. BP=%p"), BP);
        }
    }

    if (!SMC || !SMC->GetStaticMesh())
    {
        UE_LOG(LogURLab, Warning, TEXT("[MjGeom] DecomposeMesh: '%s' has no StaticMesh child."), *GetName());
        return;
    }

    UStaticMesh* LocalMesh = SMC->GetStaticMesh();
    UBodySetup* BodySetup = LocalMesh->GetBodySetup();
    if (!BodySetup || BodySetup->TriMeshGeometries.Num() == 0)
    {
        UE_LOG(LogURLab, Warning, TEXT("[MjGeom] DecomposeMesh: '%s' has no collision geometry."), *GetName());
        return;
    }

    // Remove any existing decomposition first
    RemoveDecomposition();

    FScopedSlowTask SlowTask(2.f, NSLOCTEXT("URLab", "DecomposingMesh", "Running CoACD mesh decomposition..."));
    SlowTask.MakeDialog(/*bShowCancelButton=*/false);

    SlowTask.EnterProgressFrame(1.f, NSLOCTEXT("URLab", "DecompStep1", "Decomposing mesh with CoACD..."));

    // Export and decompose
    FString AssetName = LocalMesh->GetName();
    FString OwnerDir = GetOwner() ? GetOwner()->GetClass()->GetName() : TEXT("Shared");
    FString FilePath = FString::Printf(TEXT("%s/URLab/ConvertedMeshes/%s/Complex_%s.obj"),
        *FPaths::ProjectSavedDir(), *OwnerDir, *AssetName);
    FString FullFilePath = FPaths::ConvertRelativePathToFull(FilePath);

    const int32 GeometryIndex = 0;
    auto& TriGeom = BodySetup->TriMeshGeometries[GeometryIndex];
    auto& Vertices = TriGeom.GetReference()->Particles().X();

    int MeshCount = 0;
    IO::DeleteMeshCache(FullFilePath, true);

    if (TriGeom.GetReference()->Elements().RequiresLargeIndices())
    {
        const auto& Indices = TriGeom.GetReference()->Elements().GetLargeIndexBuffer();
        MeshCount = MeshUtils::SaveMesh(FullFilePath, Vertices, Indices, true, CoACDThreshold);
        { FString Hash = IO::ComputeMeshHash(Vertices, Indices) + FString::Printf(TEXT("_complex_%.4f"), CoACDThreshold);
        IO::SaveMeshHash(FullFilePath, Hash); }
    }
    else
    {
        const auto& Indices = TriGeom.GetReference()->Elements().GetSmallIndexBuffer();
        MeshCount = MeshUtils::SaveMesh(FullFilePath, Vertices, Indices, true, CoACDThreshold);
        { FString Hash = IO::ComputeMeshHash(Vertices, Indices) + FString::Printf(TEXT("_complex_%.4f"), CoACDThreshold);
        IO::SaveMeshHash(FullFilePath, Hash); }
    }

    if (MeshCount == 0)
    {
        UE_LOG(LogURLab, Error, TEXT("[MjGeom] DecomposeMesh: CoACD produced 0 hulls for '%s'."), *GetName());
        return;
    }

    SlowTask.EnterProgressFrame(1.f, FText::Format(
        NSLOCTEXT("URLab", "DecompStep2", "Creating {0} hull components..."),
        FText::AsNumber(MeshCount)));

    // Create hull sub-geom components.
    // In the BP editor we must create SCS nodes; at runtime we use instance components.
    UBlueprint* BP = nullptr;
    USCS_Node* MyNode = nullptr;
    for (UObject* Outer = GetOuter(); Outer; Outer = Outer->GetOuter())
    {
        if (UBlueprint* Found = Cast<UBlueprint>(Outer)) { BP = Found; break; }
        if (UBlueprintGeneratedClass* BPGC = Cast<UBlueprintGeneratedClass>(Outer))
        { BP = Cast<UBlueprint>(BPGC->ClassGeneratedBy); break; }
    }
    if (BP && BP->SimpleConstructionScript)
    {
        for (USCS_Node* Node : BP->SimpleConstructionScript->GetAllNodes())
        {
            if (Node->ComponentTemplate == this) { MyNode = Node; break; }
        }
    }

    // Find the parent node (hull siblings go under the same parent as this geom)
    USCS_Node* ParentNode = nullptr;
    if (MyNode && BP)
    {
        for (USCS_Node* Node : BP->SimpleConstructionScript->GetAllNodes())
        {
            if (Node->ChildNodes.Contains(MyNode)) { ParentNode = Node; break; }
        }
    }

    // Runtime fallback
    USceneComponent* ParentComp = GetAttachParent();
    bool bIsSCSContext = (MyNode != nullptr && ParentNode != nullptr);

    if (!bIsSCSContext && !ParentComp)
    {
        UE_LOG(LogURLab, Error, TEXT("[MjGeom] DecomposeMesh: '%s' has no parent to attach hulls to."), *GetName());
        return;
    }

    if (BP) BP->Modify(); // UE undo for Blueprint

    for (int32 i = 0; i < MeshCount; ++i)
    {
        FString HullNameStr = FString::Printf(TEXT("%s_hull_%d"), *GetName(), i);

        UMjGeom* Hull = nullptr;

        if (bIsSCSContext)
        {
            // Blueprint editor: create an SCS node
            USCS_Node* HullNode = BP->SimpleConstructionScript->CreateNode(UMjGeom::StaticClass(), *HullNameStr);
            Hull = Cast<UMjGeom>(HullNode->ComponentTemplate);
            ParentNode->AddChildNode(HullNode);
        }
        else
        {
            // Runtime: create instance component
            Hull = NewObject<UMjGeom>(GetOwner(), UMjGeom::StaticClass(),
                MakeUniqueObjectName(GetOwner(), UMjGeom::StaticClass(), *HullNameStr));
            Hull->CreationMethod = EComponentCreationMethod::Instance;
            GetOwner()->AddInstanceComponent(Hull);
            Hull->RegisterComponent();
            Hull->AttachToComponent(ParentComp, FAttachmentTransformRules::KeepRelativeTransform);
        }

        if (!Hull) continue;

        Hull->bIsDecomposedHull = true;
        Hull->bOverride_Type = true;
        Hull->Type = EMjGeomType::Mesh;
        Hull->MeshName = FString::Printf(TEXT("%s_%d"), *AssetName, i);
        Hull->MjName = FString::Printf(TEXT("%s_hull_%d"), *(MjName.IsEmpty() ? GetName() : MjName), i);

        // Copy physics properties from source geom
        Hull->friction = friction;           Hull->bOverride_friction = bOverride_friction;
        Hull->solref = solref;               Hull->bOverride_solref = bOverride_solref;
        Hull->solimp = solimp;               Hull->bOverride_solimp = bOverride_solimp;
        Hull->density = density;             Hull->bOverride_density = bOverride_density;
        Hull->mass = mass;                   Hull->bOverride_mass = bOverride_mass;
        Hull->margin = margin;               Hull->bOverride_margin = bOverride_margin;
        Hull->gap = gap;                     Hull->bOverride_gap = bOverride_gap;
        Hull->condim = condim;               Hull->bOverride_condim = bOverride_condim;
        Hull->contype = contype;             Hull->bOverride_contype = bOverride_contype;
        Hull->conaffinity = conaffinity;     Hull->bOverride_conaffinity = bOverride_conaffinity;
        Hull->priority = priority;           Hull->bOverride_priority = bOverride_priority;
        Hull->group = 3;                     Hull->bOverride_group = true;
        Hull->rgba = rgba;                   Hull->bOverride_rgba = bOverride_rgba;
        Hull->MjClassName = MjClassName;

        // Import the OBJ file as a UStaticMesh asset for editor visualization
        FString SubObjPath = FString::Printf(TEXT("%s/URLab/ConvertedMeshes/%s/Complex_%s_sub_%d.obj"),
            *FPaths::ProjectSavedDir(), *OwnerDir, *AssetName, i);
        FString SubObjFullPath = FPaths::ConvertRelativePathToFull(SubObjPath);

        if (FPaths::FileExists(SubObjFullPath))
        {
            // Use articulation name as subfolder for organization
            FString ArticName = GetOwner() ? GetOwner()->GetName() : TEXT("Unknown");
            FString DestPath = FString::Printf(TEXT("/Game/URLab/DecomposedMeshes/%s"), *ArticName);

            UAutomatedAssetImportData* ImportData = NewObject<UAutomatedAssetImportData>();
            ImportData->Filenames.Add(SubObjFullPath);
            ImportData->DestinationPath = DestPath;
            ImportData->bReplaceExisting = true;
            ImportData->bSkipReadOnly = true;

            IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools").Get();
            TArray<UObject*> ImportedAssets = AssetTools.ImportAssetsAutomated(ImportData);

            UStaticMesh* ImportedMesh = nullptr;
            for (UObject* Asset : ImportedAssets)
            {
                if (UStaticMesh* SM = Cast<UStaticMesh>(Asset))
                {
                    ImportedMesh = SM;
                    break;
                }
            }

            if (ImportedMesh)
            {
                // Create a child StaticMeshComponent on the hull for visualization
                FString SMCName = FString::Printf(TEXT("%s_vis"), *HullNameStr);

                if (bIsSCSContext && BP)
                {
                    USCS_Node* HullNode = nullptr;
                    // Find the hull's SCS node we just created
                    for (USCS_Node* Node : BP->SimpleConstructionScript->GetAllNodes())
                    {
                        if (Node->ComponentTemplate == Hull) { HullNode = Node; break; }
                    }
                    if (HullNode)
                    {
                        USCS_Node* SMCNode = BP->SimpleConstructionScript->CreateNode(UStaticMeshComponent::StaticClass(), *SMCName);
                        UStaticMeshComponent* VisMesh = Cast<UStaticMeshComponent>(SMCNode->ComponentTemplate);
                        if (VisMesh)
                        {
                            VisMesh->SetStaticMesh(ImportedMesh);
                            VisMesh->SetRelativeScale3D(FVector(100.0f)); // OBJ is in meters, UE in cm
                        }
                        HullNode->AddChildNode(SMCNode);
                    }
                }
                else
                {
                    UStaticMeshComponent* VisMesh = NewObject<UStaticMeshComponent>(GetOwner(), *SMCName);
                    VisMesh->SetStaticMesh(ImportedMesh);
                    VisMesh->SetRelativeScale3D(FVector(100.0f));
                    VisMesh->CreationMethod = EComponentCreationMethod::Instance;
                    GetOwner()->AddInstanceComponent(VisMesh);
                    VisMesh->RegisterComponent();
                    VisMesh->AttachToComponent(Hull, FAttachmentTransformRules::KeepRelativeTransform);
                }

                UE_LOG(LogURLab, Log, TEXT("[MjGeom] Imported hull mesh '%s' for visualization"), *ImportedMesh->GetName());
            }
        }

        UE_LOG(LogURLab, Log, TEXT("[MjGeom] Created hull '%s' (mesh: %s)"), *Hull->GetName(), *Hull->MeshName);
    }

    if (BP)
    {
        FBlueprintEditorUtils::MarkBlueprintAsStructurallyModified(BP);
    }

    // Disable source geom
    bDisabledByDecomposition = true;

    UE_LOG(LogURLab, Log, TEXT("[MjGeom] Decomposed '%s' into %d hull sub-geoms."), *GetName(), MeshCount);
}

void UMjGeom::RemoveDecomposition()
{
    FString SourceName = MjName.IsEmpty() ? GetName() : MjName;
    int32 Removed = 0;

    // Try SCS path first (Blueprint editor)
    UBlueprint* BP = nullptr;
    for (UObject* Outer = GetOuter(); Outer; Outer = Outer->GetOuter())
    {
        if (UBlueprint* Found = Cast<UBlueprint>(Outer)) { BP = Found; break; }
        if (UBlueprintGeneratedClass* BPGC = Cast<UBlueprintGeneratedClass>(Outer))
        { BP = Cast<UBlueprint>(BPGC->ClassGeneratedBy); break; }
    }

    if (BP && BP->SimpleConstructionScript)
    {
        BP->Modify();
        TArray<USCS_Node*> AllNodes = BP->SimpleConstructionScript->GetAllNodes();
        TArray<USCS_Node*> NodesToRemove;

        for (USCS_Node* Node : AllNodes)
        {
            UMjGeom* Geom = Cast<UMjGeom>(Node->ComponentTemplate);
            if (Geom && Geom != this && Geom->bIsDecomposedHull)
            {
                if (Geom->MjName.StartsWith(SourceName + TEXT("_hull_")))
                {
                    NodesToRemove.Add(Node);
                }
            }
        }

        for (USCS_Node* Node : NodesToRemove)
        {
            BP->SimpleConstructionScript->RemoveNode(Node);
            Removed++;
        }

        if (Removed > 0)
        {
            FBlueprintEditorUtils::MarkBlueprintAsStructurallyModified(BP);
        }
    }
    else if (GetOwner())
    {
        // Runtime path: destroy instance components
        TArray<UMjGeom*> AllGeoms;
        GetOwner()->GetComponents<UMjGeom>(AllGeoms);

        for (UMjGeom* Geom : AllGeoms)
        {
            if (Geom == this) continue;
            if (!Geom->bIsDecomposedHull) continue;
            if (Geom->MjName.StartsWith(SourceName + TEXT("_hull_")))
            {
                Geom->DestroyComponent();
                Removed++;
            }
        }
    }

    bDisabledByDecomposition = false;
    UE_LOG(LogURLab, Log, TEXT("[MjGeom] Removed %d hull sub-geoms for '%s'. Source geom re-enabled."), Removed, *GetName());
}
#else
void UMjGeom::DecomposeMesh() {}
void UMjGeom::RemoveDecomposition() {}
#endif

#if WITH_EDITOR
// --- CODEGEN_EDITOR_OPTIONS_START ---
TArray<FString> UMjGeom::GetDefaultClassOptions() const { return UMjComponent::GetSiblingComponentOptions(this, UMjDefault::StaticClass(), true); }
// --- CODEGEN_EDITOR_OPTIONS_END ---
#endif
