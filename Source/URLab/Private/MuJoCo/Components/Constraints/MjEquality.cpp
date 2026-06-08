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

#include "MuJoCo/Components/Constraints/MjEquality.h"
#include "MuJoCo/Core/Spec/MjSpecWrapper.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "XmlNode.h"
#include "MuJoCo/Utils/MjXmlUtils.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "MuJoCo/Components/Tendons/MjTendon.h"
#include "Utils/URLabLogging.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"

UMjEquality::UMjEquality()
{
    PrimaryComponentTick.bCanEverTick = false;
}

void UMjEquality::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
    if (!Node) return;

    // Resolve EqualityType from the XML tag FIRST so the codegen-emitted
    // import block (below) sees the correct enum value. The
    // unit_conversion / mjs_data_packed_attrs branches for anchor / relpose
    // / polycoef / torquescale are all gated on EqualityType — without
    // this dispatch up front, they'd evaluate against the UPROPERTY default
    // (Weld) and silently misbehave for joint/tendon/flex equalities.
    {
        const FString Tag = Node->GetTag().ToLower();
        if      (Tag == TEXT("connect"))     EqualityType = EMjEqualityType::Connect;
        else if (Tag == TEXT("weld"))        EqualityType = EMjEqualityType::Weld;
        else if (Tag == TEXT("joint"))       EqualityType = EMjEqualityType::Joint;
        else if (Tag == TEXT("tendon"))      EqualityType = EMjEqualityType::Tendon;
        else if (Tag == TEXT("flex"))        EqualityType = EMjEqualityType::Flex;
        else if (Tag == TEXT("flexvert"))    EqualityType = EMjEqualityType::FlexVert;
        else if (Tag == TEXT("flexstrain"))  EqualityType = EMjEqualityType::FlexStrain;
    }

    // --- CODEGEN_IMPORT_START ---
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("relpose"), relpose, bOverride_relpose);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("anchor"), anchor, bOverride_anchor);
    if (MjXmlUtils::ReadAttrString(Node, TEXT("site1"), site1)) bOverride_site1 = true;
    if (MjXmlUtils::ReadAttrString(Node, TEXT("site2"), site2)) bOverride_site2 = true;
    MjXmlUtils::ReadAttrFloat(Node, TEXT("active"), active, bOverride_active);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("solref"), solref, bOverride_solref);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("solimp"), solimp, bOverride_solimp);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("torquescale"), torquescale, bOverride_torquescale);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("polycoef"), polycoef, bOverride_polycoef);
    // target_collation: -> Obj1
    Obj1 = Node->GetAttribute(TEXT("body1"));
    if (Obj1.IsEmpty()) Obj1 = Node->GetAttribute(TEXT("site1"));
    if (Obj1.IsEmpty()) Obj1 = Node->GetAttribute(TEXT("joint1"));
    if (Obj1.IsEmpty()) Obj1 = Node->GetAttribute(TEXT("tendon1"));
    if (Obj1.IsEmpty()) Obj1 = Node->GetAttribute(TEXT("flex"));
    // target_collation: -> Obj2
    Obj2 = Node->GetAttribute(TEXT("body2"));
    if (Obj2.IsEmpty()) Obj2 = Node->GetAttribute(TEXT("site2"));
    if (Obj2.IsEmpty()) Obj2 = Node->GetAttribute(TEXT("joint2"));
    if (Obj2.IsEmpty()) Obj2 = Node->GetAttribute(TEXT("tendon2"));
    if (bOverride_anchor)
    {
        if      ((EqualityType == EMjEqualityType::Connect) || (EqualityType == EMjEqualityType::Weld)) { for (float& V : anchor) { V *= 100.0f; } }
    }
    // --- CODEGEN_IMPORT_END ---

    UE_LOG(LogURLabImport, Log, TEXT("[MjEquality XML Import] '%s' (%d) | Obj1: %s, Obj2: %s"),
        *GetName(), (int)EqualityType, *Obj1, *Obj2);
}

void UMjEquality::ExportTo(mjsEquality* Element)
{
    if (!Element) return;

    Element->type = (mjtEq)EqualityType;

    // mjsEquality.objtype tells MuJoCo what kind of object name1/name2 refer
    // to (body / site / joint / tendon / flex). Without this set, mjs_attach's
    // CopyList step can't resolve the references and silently drops the
    // equality.
    //
    // Connect / Weld are dual-mode: either body-to-body (with optional
    // anchor) or site-to-site. URLab's parser collates site1/site2 into
    // Obj1/Obj2 alongside body1/body2; we use the non-empty site1
    // UPROPERTY as the discriminator for "this was a site-mode equality".
    // --- CODEGEN_OBJTYPE_DISPATCH_START ---
    switch (EqualityType)
    {
        case EMjEqualityType::Connect:
        case EMjEqualityType::Weld:
            Element->objtype = !site1.IsEmpty() ? mjOBJ_SITE : mjOBJ_BODY;
            break;
        case EMjEqualityType::Joint:
            Element->objtype = mjOBJ_JOINT;
            break;
        case EMjEqualityType::Tendon:
            Element->objtype = mjOBJ_TENDON;
            break;
        case EMjEqualityType::Flex:
        case EMjEqualityType::FlexVert:
        case EMjEqualityType::FlexStrain:
            Element->objtype = mjOBJ_FLEX;
            break;
        default:
            Element->objtype = mjOBJ_UNKNOWN;
            break;
    }
    // --- CODEGEN_OBJTYPE_DISPATCH_END ---

    // --- CODEGEN_EXPORT_START ---
    if (bOverride_active) Element->active = active;
    if (bOverride_solref) { for (int32 i = 0; i < solref.Num(); ++i) Element->solref[i] = solref[i]; }
    if (bOverride_solimp) { for (int32 i = 0; i < solimp.Num(); ++i) Element->solimp[i] = solimp[i]; }
    if (((EqualityType == EMjEqualityType::Connect) || (EqualityType == EMjEqualityType::Weld)) && bOverride_anchor)
    {
        for (int32 i = 0; i < anchor.Num() && i < 3; ++i)
        {
            float V = anchor[i];
            V *= 0.01f;
            Element->data[0 + i] = (mjtNum)V;
        }
    }
    if ((EqualityType == EMjEqualityType::Weld) && bOverride_relpose)
    {
        for (int32 i = 0; i < relpose.Num() && i < 7; ++i)
            Element->data[3 + i] = (mjtNum)relpose[i];
    }
    if (((EqualityType == EMjEqualityType::Joint) || (EqualityType == EMjEqualityType::Tendon)) && bOverride_polycoef)
    {
        for (int32 i = 0; i < polycoef.Num() && i < 5; ++i)
            Element->data[0 + i] = (mjtNum)polycoef[i];
    }
    if ((EqualityType == EMjEqualityType::Weld) && bOverride_torquescale)
    {
        Element->data[10] = (mjtNum)torquescale;
    }
    MjSetString(Element->name1, Obj1);
    MjSetString(Element->name2, Obj2);
    // --- CODEGEN_EXPORT_END ---
}

void UMjEquality::RegisterToSpec(FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody)
{
    mjsEquality* Eq = mjs_addEquality(Wrapper.Spec, nullptr);
    if (!Eq) return;

    m_SpecElement = Eq->element;

    FString TName = MjName.IsEmpty() ? GetName() : MjName;
    mjs_setName(Eq->element, TCHAR_TO_UTF8(*TName));

    ExportTo(Eq);
}

#if WITH_EDITOR
TArray<FString> UMjEquality::GetObjOptions() const
{
    UClass* FilterClass = nullptr;
    switch (EqualityType)
    {
        case EMjEqualityType::Connect:
        case EMjEqualityType::Weld:
            FilterClass = UMjBody::StaticClass();
            break;
        case EMjEqualityType::Joint:
            FilterClass = UMjJoint::StaticClass();
            break;
        case EMjEqualityType::Tendon:
            FilterClass = UMjTendon::StaticClass();
            break;
    }
    if (!FilterClass) return {TEXT("")};
    return UMjComponent::GetSiblingComponentOptions(this, FilterClass);
}
#endif

// --- Multi-UCLASS subclass constructors --------------------------------------
// Each subclass pins EqualityType. State is on the base; mjs_data_packed_attrs
// in codegen_rules.json branches on EqualityType for polycoef/torquescale.
UMjConnectEquality::UMjConnectEquality()       { EqualityType = EMjEqualityType::Connect; }
UMjWeldEquality::UMjWeldEquality()             { EqualityType = EMjEqualityType::Weld; }
UMjJointEquality::UMjJointEquality()           { EqualityType = EMjEqualityType::Joint; }
UMjTendonEquality::UMjTendonEquality()         { EqualityType = EMjEqualityType::Tendon; }
UMjFlexEquality::UMjFlexEquality()             { EqualityType = EMjEqualityType::Flex; }
UMjFlexVertEquality::UMjFlexVertEquality()     { EqualityType = EMjEqualityType::FlexVert; }
UMjFlexStrainEquality::UMjFlexStrainEquality() { EqualityType = EMjEqualityType::FlexStrain; }
