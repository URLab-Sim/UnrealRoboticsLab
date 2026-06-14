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

#include "MuJoCo/Components/Keyframes/MjKeyframe.h"
#include "MuJoCo/Core/Spec/MjSpecWrapper.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "MuJoCo/Components/Joints/MjFreeJoint.h"
#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "MuJoCo/Utils/MjXmlUtils.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "XmlNode.h"
#include "Utils/URLabLogging.h"

UMjKeyframe::UMjKeyframe()
{
    PrimaryComponentTick.bCanEverTick = false;
}

void UMjKeyframe::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
    if (!Node) return;

    // --- CODEGEN_IMPORT_START ---
    MjXmlUtils::ReadAttrFloat(Node, TEXT("time"), Time, bOverride_Time);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("qpos"), Qpos, bOverride_Qpos);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("qvel"), Qvel, bOverride_Qvel);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("act"), Act, bOverride_Act);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("mpos"), Mpos, bOverride_Mpos);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("mquat"), Mquat, bOverride_Mquat);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("ctrl"), Ctrl, bOverride_Ctrl);
    // --- CODEGEN_IMPORT_END ---

    UE_LOG(LogURLabImport, Log, TEXT("[MjKeyframe XML Import] '%s' | Time: %f, Qpos: %d, Qvel: %d"),
        *GetName(), Time, Qpos.Num(), Qvel.Num());
}

void UMjKeyframe::ExportTo(mjsKey* Element, mjsDefault* Default)
{
    if (!Element) return;

    // --- CODEGEN_EXPORT_START ---
    if (bOverride_Time) Element->time = Time;
    if (bOverride_Qpos) MjSetDoubleVec(Element->qpos, Qpos);
    if (bOverride_Qvel) MjSetDoubleVec(Element->qvel, Qvel);
    if (bOverride_Act) MjSetDoubleVec(Element->act, Act);
    if (bOverride_Ctrl) MjSetDoubleVec(Element->ctrl, Ctrl);
    if (bOverride_Mpos) MjSetDoubleVec(Element->mpos, Mpos);
    if (bOverride_Mquat) MjSetDoubleVec(Element->mquat, Mquat);
    // --- CODEGEN_EXPORT_END ---
}

// Counts expected DOF dimensions from the owning actor's joint components.
struct FKeyframeDimensions
{
    int32 NumFreeJoints = 0;
    int32 NqJointsOnly = 0;   // hinge=1, slide=1, ball=4
    int32 NvJointsOnly = 0;   // hinge=1, slide=1, ball=3
    int32 NqWithFree = 0;     // NqJointsOnly + NumFreeJoints * 7
    int32 NvWithFree = 0;     // NvJointsOnly + NumFreeJoints * 6
    int32 NumActuators = 0;
};

static FKeyframeDimensions CountDimensions(AActor* Owner)
{
    FKeyframeDimensions D;
    if (!Owner) return D;

    TArray<UActorComponent*> Components;
    Owner->GetComponents(Components);

    for (UActorComponent* Comp : Components)
    {
        if (Cast<UMjFreeJoint>(Comp))
        {
            D.NumFreeJoints++;
            continue;
        }
        if (UMjJoint* Joint = Cast<UMjJoint>(Comp))
        {
            if (Joint->bIsDefault) continue;
            if (Joint->Type == EMjJointType::Free)
            {
                D.NumFreeJoints++;
            }
            else if (Joint->Type == EMjJointType::Hinge)
            {
                D.NqJointsOnly += 1;
                D.NvJointsOnly += 1;
            }
            else if (Joint->Type == EMjJointType::Slide)
            {
                D.NqJointsOnly += 1;
                D.NvJointsOnly += 1;
            }
            else if (Joint->Type == EMjJointType::Ball)
            {
                D.NqJointsOnly += 4;
                D.NvJointsOnly += 3;
            }
        }
        if (Cast<UMjActuator>(Comp))
        {
            UMjComponent* MjComp = Cast<UMjComponent>(Comp);
            if (MjComp && !MjComp->bIsDefault)
                D.NumActuators++;
        }
    }

    D.NqWithFree = D.NqJointsOnly + D.NumFreeJoints * 7;
    D.NvWithFree = D.NvJointsOnly + D.NumFreeJoints * 6;
    return D;
}

// SetVec is a thin alias for MjUtils::MjSetDoubleVec; kept here so the
// freejoint padding paths below read cleanly.

void UMjKeyframe::RegisterToSpec(FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody)
{
    mjsKey* Key = mjs_addKey(Wrapper.Spec);
    if (!Key)
    {
        UE_LOG(LogURLabImport, Error, TEXT("[MjKeyframe] mjs_addKey failed for '%s'"), *GetName());
        return;
    }

    m_SpecElement = Key->element;

    FString TName = MjName.IsEmpty() ? GetName() : MjName;
    mjs_setName(Key->element, TCHAR_TO_UTF8(*TName));

    // 1) Codegen-owned ExportTo: writes Time + verbatim qpos/qvel/act/ctrl/
    //    mpos/mquat. May be overwritten below for freejoint-aware qpos/qvel
    //    padding.
    ExportTo(Key);

    // 2) Freejoint-aware padding: when the model has free joints, MJCF
    //    keyframe XML often omits the freejoint qpos/qvel slots (the user
    //    only types the hinge/slide/ball values). Detect this by length and
    //    pad in front with identity-pose (qpos: 0,0,0, 1,0,0,0) / zero-vel
    //    (qvel: 0,0,0, 0,0,0) per freejoint.
    FKeyframeDimensions Dims = CountDimensions(GetOwner());

    if (bOverride_Qpos && Dims.NumFreeJoints > 0 && Qpos.Num() == Dims.NqJointsOnly)
    {
        TArray<float> Padded;
        Padded.Reserve(Dims.NqWithFree);
        for (int32 i = 0; i < Dims.NumFreeJoints; i++)
        {
            Padded.Append({0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f});
        }
        Padded.Append(Qpos);
        MjSetDoubleVec(Key->qpos, Padded);
        UE_LOG(LogURLabImport, Log, TEXT("[MjKeyframe] '%s': padded qpos %d -> %d (%d free joints)"),
            *TName, Qpos.Num(), Padded.Num(), Dims.NumFreeJoints);
    }
    else if (bOverride_Qpos && Dims.NumFreeJoints > 0 && Qpos.Num() != Dims.NqWithFree)
    {
        UE_LOG(LogURLabImport, Warning, TEXT("[MjKeyframe] '%s': qpos has %d values, expected %d (with free) or %d (joints only)"),
            *TName, Qpos.Num(), Dims.NqWithFree, Dims.NqJointsOnly);
    }

    if (bOverride_Qvel && Dims.NumFreeJoints > 0 && Qvel.Num() == Dims.NvJointsOnly)
    {
        TArray<float> Padded;
        Padded.Reserve(Dims.NvWithFree);
        for (int32 i = 0; i < Dims.NumFreeJoints; i++)
        {
            Padded.Append({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
        }
        Padded.Append(Qvel);
        MjSetDoubleVec(Key->qvel, Padded);
        UE_LOG(LogURLabImport, Log, TEXT("[MjKeyframe] '%s': padded qvel %d -> %d"),
            *TName, Qvel.Num(), Padded.Num());
    }
    else if (bOverride_Qvel && Dims.NumFreeJoints > 0 && Qvel.Num() != Dims.NvWithFree)
    {
        UE_LOG(LogURLabImport, Warning, TEXT("[MjKeyframe] '%s': qvel has %d values, expected %d (with free) or %d (joints only)"),
            *TName, Qvel.Num(), Dims.NvWithFree, Dims.NvJointsOnly);
    }

    if (bOverride_Ctrl && Dims.NumActuators > 0 && Ctrl.Num() != Dims.NumActuators)
    {
        UE_LOG(LogURLabImport, Warning, TEXT("[MjKeyframe] '%s': ctrl has %d values but %d actuators detected"),
            *TName, Ctrl.Num(), Dims.NumActuators);
    }

    UE_LOG(LogURLabImport, Log, TEXT("[MjKeyframe] Registered '%s' at time %f (nq_joints=%d, nq_free=%d, nv_joints=%d, nv_free=%d, nu=%d)"),
        *TName, Time, Dims.NqJointsOnly, Dims.NqWithFree, Dims.NvJointsOnly, Dims.NvWithFree, Dims.NumActuators);
}
