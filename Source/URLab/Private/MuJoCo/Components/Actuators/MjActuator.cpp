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

#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "MuJoCo/Utils/MjXmlUtils.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/Spec/MjSpecWrapper.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "MuJoCo/Components/Geometry/MjSite.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Components/Tendons/MjTendon.h"
#include "Utils/URLabLogging.h"

UMjActuator::UMjActuator()
{
	PrimaryComponentTick.bCanEverTick = false;

    Type = EMjActuatorType::Motor;
    TransmissionType = EMjActuatorTrnType::Joint;
    InternalValue.store(0.0f);
    NetworkValue.store(0.0f);
}

void UMjActuator::BeginPlay()
{
	Super::BeginPlay();
}

void UMjActuator::ExportTo(mjsActuator* Element, mjsDefault* Default)
{
    if (!Element) return;

    // --- CODEGEN_EXPORT_START ---
    MjSetString(Element->target, TargetName);
    switch (TransmissionType)
    {
        case EMjActuatorTrnType::Joint:          Element->trntype = mjTRN_JOINT;         break;
        case EMjActuatorTrnType::JointInParent:  Element->trntype = mjTRN_JOINTINPARENT; break;
        case EMjActuatorTrnType::SliderCrank:    Element->trntype = mjTRN_SLIDERCRANK;   break;
        case EMjActuatorTrnType::Tendon:         Element->trntype = mjTRN_TENDON;        break;
        case EMjActuatorTrnType::Site:           Element->trntype = mjTRN_SITE;          break;
        case EMjActuatorTrnType::Body:           Element->trntype = mjTRN_BODY;          break;
        default:                                 Element->trntype = mjTRN_UNDEFINED;     break;
    }
    if (TransmissionType == EMjActuatorTrnType::SliderCrank && !SliderSite.IsEmpty())
    {
        MjSetStringRaw(Element->slidersite, SliderSite);
    }
    if (TransmissionType == EMjActuatorTrnType::Site && !RefSite.IsEmpty())
    {
        MjSetStringRaw(Element->refsite, RefSite);
    }
    if (bOverride_GainType)
    {
        switch (GainType)
        {
            case EMjGainType::Fixed: Element->gaintype = (mjtGain)mjGAIN_FIXED; break;
            case EMjGainType::Affine: Element->gaintype = (mjtGain)mjGAIN_AFFINE; break;
            case EMjGainType::Muscle: Element->gaintype = (mjtGain)mjGAIN_MUSCLE; break;
            case EMjGainType::User: Element->gaintype = (mjtGain)mjGAIN_USER; break;
            default: break;
        }
    }
    if (bOverride_BiasType)
    {
        switch (BiasType)
        {
            case EMjBiasType::None: Element->biastype = (mjtBias)mjBIAS_NONE; break;
            case EMjBiasType::Affine: Element->biastype = (mjtBias)mjBIAS_AFFINE; break;
            case EMjBiasType::Muscle: Element->biastype = (mjtBias)mjBIAS_MUSCLE; break;
            case EMjBiasType::User: Element->biastype = (mjtBias)mjBIAS_USER; break;
            default: break;
        }
    }
    if (bOverride_DynType)
    {
        switch (DynType)
        {
            case EMjDynType::None: Element->dyntype = (mjtDyn)mjDYN_NONE; break;
            case EMjDynType::Integrator: Element->dyntype = (mjtDyn)mjDYN_INTEGRATOR; break;
            case EMjDynType::Filter: Element->dyntype = (mjtDyn)mjDYN_FILTER; break;
            case EMjDynType::FilterExact: Element->dyntype = (mjtDyn)mjDYN_FILTEREXACT; break;
            case EMjDynType::Muscle: Element->dyntype = (mjtDyn)mjDYN_MUSCLE; break;
            case EMjDynType::User: Element->dyntype = (mjtDyn)mjDYN_USER; break;
            default: break;
        }
    }
    if (bOverride_group) Element->group = group;
    if (bOverride_nsample) Element->nsample = nsample;
    if (bOverride_interp) Element->interp = interp;
    if (bOverride_delay) Element->delay = delay;
    if (bOverride_ctrllimited) Element->ctrllimited = ctrllimited ? 1 : 0;
    if (bOverride_forcelimited) Element->forcelimited = forcelimited ? 1 : 0;
    if (bOverride_actlimited) Element->actlimited = actlimited ? 1 : 0;
    if (bOverride_ctrlrange) { for (int32 i = 0; i < ctrlrange.Num(); ++i) Element->ctrlrange[i] = ctrlrange[i]; }
    if (bOverride_forcerange) { for (int32 i = 0; i < forcerange.Num(); ++i) Element->forcerange[i] = forcerange[i]; }
    if (bOverride_actrange) { for (int32 i = 0; i < actrange.Num(); ++i) Element->actrange[i] = actrange[i]; }
    if (bOverride_lengthrange) { for (int32 i = 0; i < lengthrange.Num(); ++i) Element->lengthrange[i] = lengthrange[i]; }
    if (bOverride_gear) { for (int32 i = 0; i < gear.Num(); ++i) Element->gear[i] = gear[i]; }
    if (bOverride_damping) { for (int32 i = 0; i < damping.Num(); ++i) Element->damping[i] = damping[i]; }
    if (bOverride_armature) Element->armature = armature;
    if (bOverride_cranklength) Element->cranklength = cranklength;
    if (bOverride_gainprm) { for (int32 i = 0; i < gainprm.Num(); ++i) Element->gainprm[i] = gainprm[i]; }
    if (bOverride_biasprm) { for (int32 i = 0; i < biasprm.Num(); ++i) Element->biasprm[i] = biasprm[i]; }
    if (bOverride_dynprm) { for (int32 i = 0; i < dynprm.Num(); ++i) Element->dynprm[i] = dynprm[i]; }
    if (bOverride_actdim) Element->actdim = actdim;
    // --- CODEGEN_EXPORT_END ---
}


void UMjActuator::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
    if (!Node) return;

    // --- CODEGEN_IMPORT_START ---
    MjXmlUtils::ReadAttrString(Node, TEXT("class"), MjClassName);
    { // xml_enum: gaintype -> EMjGainType
        FString S = Node->GetAttribute(TEXT("gaintype"));
        S = S.ToLower();
        if      (S == TEXT("fixed")) GainType = EMjGainType::Fixed;
        else if (S == TEXT("affine")) GainType = EMjGainType::Affine;
        else if (S == TEXT("muscle")) GainType = EMjGainType::Muscle;
        else if (S == TEXT("user")) GainType = EMjGainType::User;
        if (!S.IsEmpty()) bOverride_GainType = true;
    }
    { // xml_enum: biastype -> EMjBiasType
        FString S = Node->GetAttribute(TEXT("biastype"));
        S = S.ToLower();
        if      (S == TEXT("none")) BiasType = EMjBiasType::None;
        else if (S == TEXT("affine")) BiasType = EMjBiasType::Affine;
        else if (S == TEXT("muscle")) BiasType = EMjBiasType::Muscle;
        else if (S == TEXT("user")) BiasType = EMjBiasType::User;
        if (!S.IsEmpty()) bOverride_BiasType = true;
    }
    { // xml_enum: dyntype -> EMjDynType
        FString S = Node->GetAttribute(TEXT("dyntype"));
        S = S.ToLower();
        if      (S == TEXT("none")) DynType = EMjDynType::None;
        else if (S == TEXT("integrator")) DynType = EMjDynType::Integrator;
        else if (S == TEXT("filter")) DynType = EMjDynType::Filter;
        else if (S == TEXT("filterexact")) DynType = EMjDynType::FilterExact;
        else if (S == TEXT("muscle")) DynType = EMjDynType::Muscle;
        else if (S == TEXT("user")) DynType = EMjDynType::User;
        if (!S.IsEmpty()) bOverride_DynType = true;
    }
    MjXmlUtils::ReadAttrInt(Node, TEXT("group"), group, bOverride_group);
    MjXmlUtils::ReadAttrInt(Node, TEXT("nsample"), nsample, bOverride_nsample);
    MjXmlUtils::ReadAttrInt(Node, TEXT("interp"), interp, bOverride_interp);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("delay"), delay, bOverride_delay);
    MjXmlUtils::ReadAttrBool(Node, TEXT("ctrllimited"), ctrllimited, bOverride_ctrllimited);
    MjXmlUtils::ReadAttrBool(Node, TEXT("forcelimited"), forcelimited, bOverride_forcelimited);
    MjXmlUtils::ReadAttrBool(Node, TEXT("actlimited"), actlimited, bOverride_actlimited);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("ctrlrange"), ctrlrange, bOverride_ctrlrange);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("forcerange"), forcerange, bOverride_forcerange);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("actrange"), actrange, bOverride_actrange);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("lengthrange"), lengthrange, bOverride_lengthrange);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("gear"), gear, bOverride_gear);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("damping"), damping, bOverride_damping);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("armature"), armature, bOverride_armature);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("cranklength"), cranklength, bOverride_cranklength);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("gainprm"), gainprm, bOverride_gainprm);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("biasprm"), biasprm, bOverride_biasprm);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("dynprm"), dynprm, bOverride_dynprm);
    MjXmlUtils::ReadAttrInt(Node, TEXT("actdim"), actdim, bOverride_actdim);
    { // canonicalize actuator transmission attrs
        FString TrnTarget;
        if (MjXmlUtils::ReadAttrString(Node, TEXT("joint"), TrnTarget))
        {
            TransmissionType = EMjActuatorTrnType::Joint;
            TargetName = TrnTarget;
        }
        if (MjXmlUtils::ReadAttrString(Node, TEXT("jointinparent"), TrnTarget))
        {
            TransmissionType = EMjActuatorTrnType::JointInParent;
            TargetName = TrnTarget;
        }
        if (MjXmlUtils::ReadAttrString(Node, TEXT("tendon"), TrnTarget))
        {
            TransmissionType = EMjActuatorTrnType::Tendon;
            TargetName = TrnTarget;
        }
        if (MjXmlUtils::ReadAttrString(Node, TEXT("slidersite"), SliderSite))
        {
            TransmissionType = EMjActuatorTrnType::SliderCrank;
        }
        if (MjXmlUtils::ReadAttrString(Node, TEXT("site"), TrnTarget))
        {
            TransmissionType = EMjActuatorTrnType::Site;
            TargetName = TrnTarget;
        }
        MjXmlUtils::ReadAttrString(Node, TEXT("refsite"), RefSite);
        if (MjXmlUtils::ReadAttrString(Node, TEXT("body"), TrnTarget))
        {
            TransmissionType = EMjActuatorTrnType::Body;
            TargetName = TrnTarget;
        }
    }
    // --- CODEGEN_IMPORT_END ---
}

void UMjActuator::Bind(mjModel* Model, mjData* Data, const FString& Prefix)
{
    Super::Bind(Model, Data, Prefix);
    BindAndCacheView(m_ActuatorView, Prefix);
}

// ----------------------------------------------------------------------------------
// Blueprint Runtime API — Setup & Control
// ----------------------------------------------------------------------------------

void UMjActuator::SetControl(float Value)
{
    InternalValue.store(Value);
}

void UMjActuator::SetNetworkControl(float Value)
{
    NetworkValue.store(Value);
}

void UMjActuator::ResetControl()
{
    InternalValue.store(0.0f);
    NetworkValue.store(0.0f);
}

float UMjActuator::GetControl() const
{
    if (AMjArticulation* Art = Cast<AMjArticulation>(GetOwner()))
    {
        return ResolveDesiredControl(Art->ControlSource);
    }

    if (AAMjManager* Mgr = AAMjManager::GetManager())
    {
        if (Mgr->PhysicsEngine)
            return ResolveDesiredControl((uint8)Mgr->PhysicsEngine->ControlSource);
    }
    return InternalValue.load();
}

float UMjActuator::GetMjControl() const
{
    return (m_ActuatorView.ctrl) ? (float)m_ActuatorView.ctrl[0] : 0.0f;
}

float UMjActuator::ResolveDesiredControl(uint8 Source) const
{
    // Source: 0 = ZMQ, 1 = UI (matching EControlSource)
    if (Source == 0) 
    {
        return NetworkValue.load();
    }
    return InternalValue.load();
}

float UMjActuator::GetForce() const
{
    if (m_ActuatorView.id != -1 && m_ActuatorView.actuator_force)
    {
        return (float)m_ActuatorView.actuator_force[0];
    }
    return 0.0f;
}

float UMjActuator::GetLength() const
{
    if (m_ActuatorView.id != -1 && m_ActuatorView.actuator_length)
    {
        return (float)m_ActuatorView.actuator_length[0];
    }
    return 0.0f;
}

float UMjActuator::GetVelocity() const
{
    if (m_ActuatorView.id != -1 && m_ActuatorView.actuator_velocity)
    {
        return (float)m_ActuatorView.actuator_velocity[0];
    }
    return 0.0f;
}

void UMjActuator::SetGear(const TArray<float>& NewGear)
{
    if (m_ActuatorView.id != -1 && m_ActuatorView.actuator_gear)
    {
        for(int i=0; i<NewGear.Num() && i<6; ++i)
        {
             m_ActuatorView.actuator_gear[i] = (mjtNum)NewGear[i];
        }
    }
}

TArray<float> UMjActuator::GetGear() const
{
    TArray<float> Res;
    if (m_ActuatorView.id != -1 && m_ActuatorView.actuator_gear)
    {
        for(int i=0; i<6; ++i) Res.Add((float)m_ActuatorView.actuator_gear[i]);
    }
    return Res;
}

FVector2D UMjActuator::GetControlRange() const
{
    if (m_ActuatorView.id < 0 || !m_ActuatorView.actuator_ctrlrange) return FVector2D::ZeroVector;
    return FVector2D((float)m_ActuatorView.actuator_ctrlrange[0], (float)m_ActuatorView.actuator_ctrlrange[1]);
}

float UMjActuator::GetActivation() const
{
    if (m_ActuatorView.id < 0 || !m_ActuatorView.act) return 0.0f;
    return (float)m_ActuatorView.act[0];
}

FString UMjActuator::GetMjName() const
{
    if (m_ActuatorView.id < 0 || !m_ActuatorView.name) return FString();
    return MjUtils::MjToString(m_ActuatorView.name);
}

// ----------------------------------------------------------------------------------
// Base Export (Common Properties & Virtual Dispatch)
// ----------------------------------------------------------------------------------

void UMjActuator::RegisterToSpec(FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody)
{
    if (m_SpecElement) return;

    mjsDefault* effectiveDefault = ResolveDefault(Wrapper.Spec, MjClassName);

    const char* defName = effectiveDefault ? mjs_getString(mjs_getName(effectiveDefault->element)) : "NULL";
    UE_LOG(LogURLabBind, Verbose, TEXT("[MjActuator::RegisterToSpec] '%s' class='%s' -> resolved default='%s'"),
        *GetName(), *MjClassName, UTF8_TO_TCHAR(defName ? defName : "unnamed"));

    mjsActuator* act = mjs_addActuator(Wrapper.Spec, effectiveDefault);
    m_SpecElement = act->element;
    SetSpecElementName(Wrapper, act->element, mjOBJ_ACTUATOR);

    UE_LOG(LogURLabBind, Verbose, TEXT("[MjActuator::RegisterToSpec] '%s' after mjs_addActuator: ctrlrange=[%.3f, %.3f], ctrllimited=%d"),
        *GetName(), act->ctrlrange[0], act->ctrlrange[1], act->ctrllimited);

    ExportTo(act, effectiveDefault);
}

#if WITH_EDITOR
TArray<FString> UMjActuator::GetTargetNameOptions() const
{
    UClass* FilterClass = nullptr;
    switch (TransmissionType)
    {
        case EMjActuatorTrnType::Joint:
        case EMjActuatorTrnType::JointInParent:
            FilterClass = UMjJoint::StaticClass();
            break;
        case EMjActuatorTrnType::Tendon:
            FilterClass = UMjTendon::StaticClass();
            break;
        case EMjActuatorTrnType::Site:
        case EMjActuatorTrnType::SliderCrank:
            FilterClass = UMjSite::StaticClass();
            break;
        case EMjActuatorTrnType::Body:
            FilterClass = UMjBody::StaticClass();
            break;
        default:
            FilterClass = UMjJoint::StaticClass();
            break;
    }
    return GetSiblingComponentOptions(this, FilterClass);
}

// --- CODEGEN_EDITOR_OPTIONS_START ---
TArray<FString> UMjActuator::GetSliderSiteOptions() const { return UMjComponent::GetSiblingComponentOptions(this, UMjSite::StaticClass()); }
TArray<FString> UMjActuator::GetRefSiteOptions() const { return UMjComponent::GetSiblingComponentOptions(this, UMjSite::StaticClass()); }
TArray<FString> UMjActuator::GetDefaultClassOptions() const { return UMjComponent::GetSiblingComponentOptions(this, UMjDefault::StaticClass(), true); }
    // --- CODEGEN_EDITOR_OPTIONS_END ---
#endif

