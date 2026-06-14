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

#include "MuJoCo/Components/Bodies/MjFrame.h"
#include "MuJoCo/Core/Spec/MjSpecWrapper.h"
#include "MuJoCo/Components/Geometry/MjGeom.h"
#include "MuJoCo/Components/Geometry/MjSite.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "MuJoCo/Components/Sensors/MjSensor.h"
#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Utils/MjBind.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "MuJoCo/Utils/MjXmlUtils.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "XmlNode.h"

UMjFrame::UMjFrame()
{
    PrimaryComponentTick.bCanEverTick = false;
}

void UMjFrame::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
    if (!Node) return;

    // --- CODEGEN_IMPORT_START ---
    if (MjXmlUtils::ReadAttrString(Node, TEXT("childclass"), childclass)) bOverride_childclass = true;
    MjUtils::ReadVec3InMeters(Node, TEXT("pos"), Pos, bOverride_Pos);
    { // canonicalize orientation (quat/euler/axisangle/xyaxes/zaxis)
        double TmpQuat[4] = {1.0, 0.0, 0.0, 0.0};
        if (MjOrientationUtils::OrientationToMjQuat(Node, CompilerSettings, TmpQuat))
        {
            Quat = MjUtils::MjToUERotation(TmpQuat);
            bOverride_Quat = true;
        }
    }
    if (bOverride_Pos)  SetRelativeLocation(Pos);
    if (bOverride_Quat) SetRelativeRotation(Quat);
    // --- CODEGEN_IMPORT_END ---
}

void UMjFrame::ExportTo(mjsFrame* Element, mjsDefault* Default)
{
    if (!Element) return;

    // --- CODEGEN_EXPORT_START ---
    if (bOverride_childclass) MjSetString(Element->childclass, childclass);
    // --- CODEGEN_EXPORT_END ---
}

void UMjFrame::Setup(USceneComponent* Parent, mjsBody* ParentBody, FMujocoSpecWrapper* Wrapper)
{
    // Frames directly under worldbody receive a null ParentBody; resolve it now.
    if (!ParentBody && Wrapper && Wrapper->Spec)
        ParentBody = mjs_findBody(Wrapper->Spec, "world");

    FTransform TargetTransform = GetRelativeTransform();

    FString NameToRegister = MjName.IsEmpty() ? GetName() : MjName;
    mjsFrame* FrameInSpec = Wrapper->CreateFrame(
        NameToRegister,
        ParentBody,
        TargetTransform
    );

    if (FrameInSpec)
    {
        m_SpecElement = FrameInSpec->element;
        // Codegen-owned ExportTo handles childclass + any future per-attr
        // fields. Pos/Quat are skipped (canon_export_skip in rules) because
        // CreateFrame already wrote them from the UE transform.
        ExportTo(FrameInSpec, nullptr);
    }

    TArray<USceneComponent*> DirectChildren = GetAttachChildren();

    for (USceneComponent* CurrentComponent : DirectChildren)
    {
        // 1. Recursive Frames.
        if (UMjFrame* MjFrameComp = Cast<UMjFrame>(CurrentComponent))
        {
            MjFrameComp->Setup(this, ParentBody, Wrapper);
            continue;
        }

        // 2. Child Bodies (attached to the parent body, not the frame
        //    directly; we respect Unreal hierarchy for coordinate lookup).
        if (UMjBody* MjBodyComp = Cast<UMjBody>(CurrentComponent))
        {
            MjBodyComp->Setup(this, ParentBody, Wrapper);
            continue;
        }

        // 3. Standard spec elements (Geoms, Sites, etc.) registered against
        //    the frame's parent body. The MuJoCo compiler bakes the frame
        //    offset into each child's pos/quat at compile time.
        if (CurrentComponent->GetClass()->ImplementsInterface(UMjSpecElement::StaticClass()))
        {
            IMjSpecElement* SpecElem = Cast<IMjSpecElement>(CurrentComponent);
            if (SpecElem)
            {
                SpecElem->RegisterToSpec(*Wrapper, ParentBody);
                m_SpecElements.Emplace(CurrentComponent);
            }
        }
    }
}

void UMjFrame::Bind(mjModel* Model, mjData* Data, const FString& Prefix)
{
    // Frames are compiled away by MuJoCo, so they don't have IDs in mjModel/mjData.
    m_Model = Model;
    m_Data = Data;
    m_ID = -1; // Not targetable at runtime by ID
}

void UMjFrame::RegisterToSpec(FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody)
{
    // MjFrame is handled recursively via Setup().
    if (ParentBody)
    {
        Setup(GetAttachParent(), ParentBody, &Wrapper);
    }
}
