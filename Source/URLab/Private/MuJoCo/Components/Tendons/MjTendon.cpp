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

#include "MuJoCo/Components/Tendons/MjTendon.h"
#include "mujoco/mujoco.h"
#include "mujoco/mjspec.h"
#include "MuJoCo/Components/Defaults/MujocoDefaults.h"
#include "MuJoCo/Core/Spec/MjSpecWrapper.h"
#include "XmlNode.h"
#include "Utils/URLabLogging.h"
#include "MuJoCo/Utils/MjXmlUtils.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"


UMjTendon::UMjTendon()
{
    PrimaryComponentTick.bCanEverTick = false;
}

void UMjTendon::BeginPlay()
{
    Super::BeginPlay();
}

void UMjTendon::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
    if (!Node) return;

        // --- CODEGEN_IMPORT_START ---
    MjXmlUtils::ReadAttrInt(Node, TEXT("group"), group, bOverride_group);
    MjXmlUtils::ReadAttrBool(Node, TEXT("limited"), limited, bOverride_limited);
    MjXmlUtils::ReadAttrBool(Node, TEXT("actuatorfrclimited"), actuatorfrclimited, bOverride_actuatorfrclimited);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("range"), range, bOverride_range);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("actuatorfrcrange"), actuatorfrcrange, bOverride_actuatorfrcrange);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("solreflimit"), solreflimit, bOverride_solreflimit);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("solimplimit"), solimplimit, bOverride_solimplimit);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("solreffriction"), solreffriction, bOverride_solreffriction);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("solimpfriction"), solimpfriction, bOverride_solimpfriction);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("frictionloss"), frictionloss, bOverride_frictionloss);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("springlength"), springlength, bOverride_springlength);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("width"), width, bOverride_width);
    if (MjXmlUtils::ReadAttrString(Node, TEXT("material"), material)) bOverride_material = true;
    MjXmlUtils::ReadAttrFloat(Node, TEXT("margin"), margin, bOverride_margin);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("stiffness"), stiffness, bOverride_stiffness);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("damping"), damping, bOverride_damping);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("armature"), armature, bOverride_armature);
    MjXmlUtils::ReadAttrColor(Node, TEXT("rgba"), rgba, bOverride_rgba);
    // --- CODEGEN_IMPORT_END ---

    // Class name
    MjXmlUtils::ReadAttrString(Node, TEXT("class"), MjClassName);

    // --- Physics ---

    // --- Limits ---
    MjXmlUtils::ReadAttrBool(Node, TEXT("actfrclimited"), bActFrcLimited, bOverride_ActFrcLimited);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("actfrcrange"), ActFrcRange, bOverride_ActFrcRange);

    // --- Solver ---

    // --- Visual ---
    TArray<float> RgbaParts;

    // --- Wrap entries (child nodes of the fixed/spatial element) ---
    Wraps.Empty();
    for (const FXmlNode* Child : Node->GetChildrenNodes())
    {
        if (!Child) continue;
        FString ChildTag = Child->GetTag();

        FMjTendonWrap Wrap;

        if (ChildTag.Equals(TEXT("joint"), ESearchCase::IgnoreCase))
        {
            Wrap.Type = EMjTendonWrapType::Joint;
            Wrap.TargetName = Child->GetAttribute(TEXT("joint"));
            FString CoefStr = Child->GetAttribute(TEXT("coef"));
            if (!CoefStr.IsEmpty()) Wrap.Coef = FCString::Atof(*CoefStr);
        }
        else if (ChildTag.Equals(TEXT("site"), ESearchCase::IgnoreCase))
        {
            Wrap.Type = EMjTendonWrapType::Site;
            Wrap.TargetName = Child->GetAttribute(TEXT("site"));
        }
        else if (ChildTag.Equals(TEXT("geom"), ESearchCase::IgnoreCase))
        {
            Wrap.Type = EMjTendonWrapType::Geom;
            Wrap.TargetName = Child->GetAttribute(TEXT("geom"));
            Wrap.SideSite = Child->GetAttribute(TEXT("sidesite"));
        }
        else if (ChildTag.Equals(TEXT("pulley"), ESearchCase::IgnoreCase))
        {
            Wrap.Type = EMjTendonWrapType::Pulley;
            FString DivisorStr = Child->GetAttribute(TEXT("divisor"));
            if (!DivisorStr.IsEmpty()) Wrap.Divisor = FCString::Atof(*DivisorStr);
        }
        else
        {
            UE_LOG(LogURLabImport, Warning, TEXT("[MjTendon XML Import] Unknown wrap tag: '%s'"), *ChildTag);
            continue;
        }

        Wraps.Add(Wrap);
    }

    UE_LOG(LogURLabImport, Log, TEXT("[MjTendon XML Import] '%s' -> %d wrap entries"), *GetName(), Wraps.Num());
}

void UMjTendon::ExportTo(mjsTendon* Element, mjsDefault* def)
{
    if (!Element) return;

    // --- Physics properties ---

    // --- Limits ---

    if (bOverride_ActFrcLimited) Element->actfrclimited = bActFrcLimited ? mjLIMITED_TRUE : mjLIMITED_FALSE;
    if (bOverride_ActFrcRange)
    {
        for (int i = 0; i < ActFrcRange.Num() && i < 2; ++i)
            Element->actfrcrange[i] = ActFrcRange[i];
    }

    // --- Solver ---

    // --- Visuals ---

        // --- CODEGEN_EXPORT_START ---
    if (bOverride_group) Element->group = group;
    if (bOverride_limited) Element->limited = limited ? 1 : 0;
    if (bOverride_actuatorfrclimited) Element->actfrclimited = actuatorfrclimited ? 1 : 0;
    if (bOverride_range) { for (int32 i = 0; i < range.Num(); ++i) Element->range[i] = range[i]; }
    if (bOverride_actuatorfrcrange) { for (int32 i = 0; i < actuatorfrcrange.Num(); ++i) Element->actfrcrange[i] = actuatorfrcrange[i]; }
    if (bOverride_solreflimit) { for (int32 i = 0; i < solreflimit.Num(); ++i) Element->solref_limit[i] = solreflimit[i]; }
    if (bOverride_solimplimit) { for (int32 i = 0; i < solimplimit.Num(); ++i) Element->solimp_limit[i] = solimplimit[i]; }
    if (bOverride_solreffriction) { for (int32 i = 0; i < solreffriction.Num(); ++i) Element->solref_friction[i] = solreffriction[i]; }
    if (bOverride_solimpfriction) { for (int32 i = 0; i < solimpfriction.Num(); ++i) Element->solimp_friction[i] = solimpfriction[i]; }
    if (bOverride_frictionloss) Element->frictionloss = frictionloss;
    if (bOverride_springlength) { for (int32 i = 0; i < springlength.Num(); ++i) Element->springlength[i] = springlength[i]; }
    if (bOverride_width) Element->width = width;
    if (bOverride_margin) Element->margin = margin;
    if (bOverride_stiffness) { for (int32 i = 0; i < stiffness.Num(); ++i) Element->stiffness[i] = stiffness[i]; }
    if (bOverride_damping) { for (int32 i = 0; i < damping.Num(); ++i) Element->damping[i] = damping[i]; }
    if (bOverride_armature) Element->armature = armature;
    if (bOverride_rgba) { Element->rgba[0] = rgba.R; Element->rgba[1] = rgba.G; Element->rgba[2] = rgba.B; Element->rgba[3] = rgba.A; }
    // --- CODEGEN_EXPORT_END ---
}

void UMjTendon::RegisterToSpec(FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody)
{
    mjsDefault* effectiveDefault = ResolveDefault(Wrapper.Spec, MjClassName);

    // Create tendon in the spec
    mjsTendon* Tendon = mjs_addTendon(Wrapper.Spec, effectiveDefault);
    if (!Tendon)
    {
        UE_LOG(LogURLabImport, Error, TEXT("[MjTendon] mjs_addTendon failed for '%s'"), *GetName());
        return;
    }

    m_SpecElement = Tendon->element;

    // Set name
    FString TName = MjName.IsEmpty() ? GetName() : MjName;
    mjs_setName(Tendon->element, TCHAR_TO_UTF8(*TName));

    // Export properties
    ExportTo(Tendon, effectiveDefault);

    // --- Wrap entries ---
    for (const FMjTendonWrap& Wrap : Wraps)
    {
        switch (Wrap.Type)
        {
        case EMjTendonWrapType::Joint:
            {
                mjsWrap* W = mjs_wrapJoint(Tendon, TCHAR_TO_UTF8(*Wrap.TargetName), (double)Wrap.Coef);
                if (!W)
                {
                    UE_LOG(LogURLabImport, Warning, TEXT("[MjTendon] mjs_wrapJoint failed for joint '%s' in tendon '%s'"),
                        *Wrap.TargetName, *TName);
                }
            }
            break;

        case EMjTendonWrapType::Site:
            {
                mjsWrap* W = mjs_wrapSite(Tendon, TCHAR_TO_UTF8(*Wrap.TargetName));
                if (!W)
                {
                    UE_LOG(LogURLabImport, Warning, TEXT("[MjTendon] mjs_wrapSite failed for site '%s' in tendon '%s'"),
                        *Wrap.TargetName, *TName);
                }
            }
            break;

        case EMjTendonWrapType::Geom:
            {
                // FTCHARToUTF8 keeps the converted string alive for the
                // duration of the wrapGeom call. Storing TCHAR_TO_UTF8 macro
                // output in a const char* would dangle on Linux (see
                // MjSpecWrapper::AddDefault for the same fix).
                FTCHARToUTF8 SideSiteConv(*Wrap.SideSite);
                const char* SideSiteStr = Wrap.SideSite.IsEmpty() ? "" : SideSiteConv.Get();
                mjsWrap* W = mjs_wrapGeom(Tendon, TCHAR_TO_UTF8(*Wrap.TargetName), SideSiteStr);
                if (!W)
                {
                    UE_LOG(LogURLabImport, Warning, TEXT("[MjTendon] mjs_wrapGeom failed for geom '%s' in tendon '%s'"),
                        *Wrap.TargetName, *TName);
                }
            }
            break;

        case EMjTendonWrapType::Pulley:
            {
                mjsWrap* W = mjs_wrapPulley(Tendon, (double)Wrap.Divisor);
                if (!W)
                {
                    UE_LOG(LogURLabImport, Warning, TEXT("[MjTendon] mjs_wrapPulley failed in tendon '%s'"), *TName);
                }
            }
            break;
        }
    }

    UE_LOG(LogURLabImport, Log, TEXT("[MjTendon] Registered '%s' with %d wraps"), *TName, Wraps.Num());
}

void UMjTendon::Bind(mjModel* model, mjData* data, const FString& Prefix)
{
    Super::Bind(model, data, Prefix);
    m_TendonView = BindToView<TendonView>(Prefix);
    if (m_TendonView.id != -1)
    {
        m_ID = m_TendonView.id;
    }
    else
    {
        UE_LOG(LogURLabImport, Warning, TEXT("[MjTendon] Failed to bind tendon '%s' (prefix='%s')"),
            *GetName(), *Prefix);
    }
}

float UMjTendon::GetLength() const
{
    return m_TendonView.GetLength();
}

float UMjTendon::GetVelocity() const
{
    return m_TendonView.GetVelocity();
}

FString UMjTendon::GetMjName() const
{
    if (m_TendonView.id < 0 || !m_TendonView.name) return FString();
    return MjUtils::MjToString(m_TendonView.name);
}

#if WITH_EDITOR
TArray<FString> UMjTendon::GetDefaultClassOptions() const
{
    return GetSiblingComponentOptions(this, UMjDefault::StaticClass(), true);
}
#endif

// --- Multi-UCLASS subclass constructors --------------------------------------
// Tendon spec API doesn't distinguish spatial vs fixed; that's encoded by the
// wrap entries the user adds. These subclasses are pure Blueprint UX hints.
UMjSpatialTendon::UMjSpatialTendon() {}
UMjFixedTendon::UMjFixedTendon() {}
