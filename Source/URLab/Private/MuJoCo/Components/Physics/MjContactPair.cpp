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

#include "MuJoCo/Components/Physics/MjContactPair.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "XmlFile.h"
#include "mujoco/mujoco.h"
#include "MuJoCo/Core/Spec/MjSpecWrapper.h"
#include "MuJoCo/Utils/MjXmlUtils.h"
#include "MuJoCo/Components/MjComponent.h"
#include "MuJoCo/Components/Geometry/MjGeom.h"
#include "Utils/URLabLogging.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"

UMjContactPair::UMjContactPair()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UMjContactPair::BeginPlay()
{
	Super::BeginPlay();
}

void UMjContactPair::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
        // --- CODEGEN_IMPORT_START ---
    MjXmlUtils::ReadAttrString(Node, TEXT("name"), Name);
    if (MjXmlUtils::ReadAttrString(Node, TEXT("geom1"), geom1)) bOverride_geom1 = true;
    if (MjXmlUtils::ReadAttrString(Node, TEXT("geom2"), geom2)) bOverride_geom2 = true;
    MjXmlUtils::ReadAttrInt(Node, TEXT("condim"), condim, bOverride_condim);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("friction"), friction, bOverride_friction);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("solref"), solref, bOverride_solref);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("solreffriction"), solreffriction, bOverride_solreffriction);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("solimp"), solimp, bOverride_solimp);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("gap"), gap, bOverride_gap);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("margin"), margin, bOverride_margin);
        // --- CODEGEN_IMPORT_END ---

    if (!Node)
    {
        return;
    }

}

void UMjContactPair::ExportTo(mjsPair* Element)
{
    if (!Element) return;

    // --- CODEGEN_EXPORT_START ---
    if (bOverride_geom1) MjSetString(Element->geomname1, geom1);
    if (bOverride_geom2) MjSetString(Element->geomname2, geom2);
    if (bOverride_condim) Element->condim = condim;
    if (bOverride_friction) { for (int32 i = 0; i < FMath::Min(friction.Num(), 5); ++i) Element->friction[i] = friction[i]; }
    if (bOverride_solref) { for (int32 i = 0; i < FMath::Min(solref.Num(), 2); ++i) Element->solref[i] = solref[i]; }
    if (bOverride_solreffriction) { for (int32 i = 0; i < FMath::Min(solreffriction.Num(), 2); ++i) Element->solreffriction[i] = solreffriction[i]; }
    if (bOverride_solimp) { for (int32 i = 0; i < FMath::Min(solimp.Num(), 5); ++i) Element->solimp[i] = solimp[i]; }
    if (bOverride_gap) Element->gap = gap;
    if (bOverride_margin) Element->margin = margin;
    // --- CODEGEN_EXPORT_END ---
}

void UMjContactPair::RegisterToSpec(FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody)
{
    mjsPair* pair = mjs_addPair(Wrapper.Spec, nullptr);
    if (!Name.IsEmpty())
    {
        mjs_setName(pair->element, TCHAR_TO_UTF8(*Name));
    }
    ExportTo(pair);
    UE_LOG(LogURLabWrapper, Log, TEXT("Added contact pair: %s->%s"), *geom1, *geom2);
}

#if WITH_EDITOR
// --- CODEGEN_EDITOR_OPTIONS_START ---
TArray<FString> UMjContactPair::GetGeomOptions() const { return UMjComponent::GetSiblingComponentOptions(this, UMjGeom::StaticClass()); }
// --- CODEGEN_EDITOR_OPTIONS_END ---
#endif
