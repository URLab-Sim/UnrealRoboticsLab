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

#include "MuJoCo/Components/Physics/MjContactExclude.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "XmlFile.h"
#include "mujoco/mujoco.h"
#include "MuJoCo/Core/Spec/MjSpecWrapper.h"
#include "MuJoCo/Components/MjComponent.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "Utils/URLabLogging.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "MuJoCo/Utils/MjXmlUtils.h"

UMjContactExclude::UMjContactExclude()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UMjContactExclude::BeginPlay()
{
	Super::BeginPlay();
}

void UMjContactExclude::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
	// --- CODEGEN_IMPORT_START ---
	MjXmlUtils::ReadAttrString(Node, TEXT("name"), Name);
	if (MjXmlUtils::ReadAttrString(Node, TEXT("body1"), body1))
		bOverride_body1 = true;
	if (MjXmlUtils::ReadAttrString(Node, TEXT("body2"), body2))
		bOverride_body2 = true;
	// --- CODEGEN_IMPORT_END ---

	if (!Node)
	{
		return;
	}
}

void UMjContactExclude::ExportTo(mjsExclude* Element)
{
	if (!Element)
		return;

	// --- CODEGEN_EXPORT_START ---
	if (bOverride_body1)
		MjSetString(Element->bodyname1, body1);
	if (bOverride_body2)
		MjSetString(Element->bodyname2, body2);
	// --- CODEGEN_EXPORT_END ---
}

void UMjContactExclude::RegisterToSpec(FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody)
{
	mjsExclude* exclude = mjs_addExclude(Wrapper.Spec);
	ExportTo(exclude);
	UE_LOG(LogURLabWrapper, Log, TEXT("Added contact exclude: %s<->%s"), *body1, *body2);
}

#if WITH_EDITOR
// --- CODEGEN_EDITOR_OPTIONS_START ---
TArray<FString> UMjContactExclude::GetBodyOptions() const
{
	return UMjComponent::GetSiblingComponentOptions(this, UMjBody::StaticClass());
}
// --- CODEGEN_EDITOR_OPTIONS_END ---
#endif
