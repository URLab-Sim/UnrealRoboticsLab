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

#include "MuJoCo/Components/Geometry/MjSite.h"
#include "mujoco/mujoco.h"
#include "XmlFile.h"

#include "MuJoCo/Utils/MjUtils.h"
#include "MuJoCo/Utils/MjXmlUtils.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "MuJoCo/Core/Spec/MjSpecWrapper.h"
#include "Utils/URLabLogging.h"

UMjSite::UMjSite()
{
	PrimaryComponentTick.bCanEverTick = false;

    Type = EMjSiteType::Sphere;
    size = { 0.01f, 0.01f, 0.01f };
    bOverride_size = true;
    rgba = FLinearColor(0.5f, 0.5f, 0.5f, 1.0f);
    group = 0;
}

void UMjSite::ExportTo(mjsSite* Element, mjsDefault* def)
{
    if (!Element) return;
    UE_LOG(LogURLabExport, Log, TEXT("Exporting Element %s"), *this->GetName());

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
    switch (Type)
    {
        case EMjSiteType::Sphere: Element->type = (mjtGeom)mjGEOM_SPHERE; break;
        case EMjSiteType::Capsule: Element->type = (mjtGeom)mjGEOM_CAPSULE; break;
        case EMjSiteType::Ellipsoid: Element->type = (mjtGeom)mjGEOM_ELLIPSOID; break;
        case EMjSiteType::Cylinder: Element->type = (mjtGeom)mjGEOM_CYLINDER; break;
        case EMjSiteType::Box: Element->type = (mjtGeom)mjGEOM_BOX; break;
        case EMjSiteType::Mesh: Element->type = (mjtGeom)mjGEOM_MESH; break;
        case EMjSiteType::Hfield: Element->type = (mjtGeom)mjGEOM_HFIELD; break;
        default: break;
    }
    if (bOverride_group) Element->group = group;
    if (bOverride_size) { for (int32 i = 0; i < FMath::Min(size.Num(), 3); ++i) { if (size[i] != -1.0f) Element->size[i] = size[i]; } }
    if (bOverride_rgba) { Element->rgba[0] = rgba.R; Element->rgba[1] = rgba.G; Element->rgba[2] = rgba.B; Element->rgba[3] = rgba.A; }
    // --- CODEGEN_EXPORT_END ---
}



void UMjSite::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
    if (!Node) return;

    // size + type are codegen-owned now. The xml_enum block sets Type first
    // inside CODEGEN_IMPORT so the fromto canon can branch on Type when
    // picking which size[] slot to write.

        // --- CODEGEN_IMPORT_START ---
    { // xml_enum: type -> EMjSiteType
        FString S = Node->GetAttribute(TEXT("type"));
        S = S.ToLower();
        if      (S == TEXT("sphere")) Type = EMjSiteType::Sphere;
        else if (S == TEXT("capsule")) Type = EMjSiteType::Capsule;
        else if (S == TEXT("ellipsoid")) Type = EMjSiteType::Ellipsoid;
        else if (S == TEXT("cylinder")) Type = EMjSiteType::Cylinder;
        else if (S == TEXT("box")) Type = EMjSiteType::Box;
        else if (S == TEXT("mesh")) Type = EMjSiteType::Mesh;
        else if (S == TEXT("hfield")) Type = EMjSiteType::Hfield;
    }
    MjXmlUtils::ReadAttrInt(Node, TEXT("group"), group, bOverride_group);
    if (MjXmlUtils::ReadAttrString(Node, TEXT("material"), material)) bOverride_material = true;
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("size"), size, bOverride_size);
    MjXmlUtils::ReadAttrColor(Node, TEXT("rgba"), rgba, bOverride_rgba);
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
            const bool bFTZSlot = (Type == EMjSiteType::Box || Type == EMjSiteType::Ellipsoid);
            const int32 FTSlot = bFTZSlot ? 2 : 1;
            while (size.Num() <= FTSlot) { size.Add(-1.0f); }
            size[FTSlot] = FTHalf;
            bOverride_size = true;
        }
    }
    if (bOverride_Pos)  SetRelativeLocation(Pos);
    if (bOverride_Quat) SetRelativeRotation(Quat);
        // --- CODEGEN_IMPORT_END ---
}

void UMjSite::RegisterToSpec(FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody)
{
    if (!ParentBody) return;

    mjsDefault* effectiveDefault = ResolveDefault(Wrapper.Spec, MjClassName);

    mjsSite* site = mjs_addSite(ParentBody, effectiveDefault);
    m_SpecElement = site->element;
    SetSpecElementName(Wrapper, site->element, mjOBJ_SITE);

    ExportTo(site, effectiveDefault);
}

void UMjSite::Bind(mjModel* model, mjData* data, const FString& Prefix)
{
    Super::Bind(model, data, Prefix);
    BindAndCacheView(m_SiteView, Prefix);
}

#if WITH_EDITOR
// --- CODEGEN_EDITOR_OPTIONS_START ---
TArray<FString> UMjSite::GetDefaultClassOptions() const { return UMjComponent::GetSiblingComponentOptions(this, UMjDefault::StaticClass(), true); }
// --- CODEGEN_EDITOR_OPTIONS_END ---
#endif

// --- Multi-UCLASS subclass constructors --------------------------------------
// All shape state is on the base UMjSite (codegen-owned ``size`` TArray +
// rgba/group/material/Pos/Quat plus the hand-declared EMjSiteType ``Type``).
// Each subclass just pins Type so the Blueprint "Add Component" picker can
// distinguish them.
UMjBoxSite::UMjBoxSite()       { Type = EMjSiteType::Box; }
UMjSphereSite::UMjSphereSite() { Type = EMjSiteType::Sphere; }
UMjCapsuleSite::UMjCapsuleSite()   { Type = EMjSiteType::Capsule; }
UMjCylinderSite::UMjCylinderSite() { Type = EMjSiteType::Cylinder; }
UMjEllipsoidSite::UMjEllipsoidSite() { Type = EMjSiteType::Ellipsoid; }
