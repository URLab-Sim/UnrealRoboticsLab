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

#include "MuJoCo/Components/Actuators/MjIntVelocityActuator.h"

#include "XmlNode.h"
#include "MuJoCo/Utils/MjXmlUtils.h"
#include "Utils/URLabLogging.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"

UMjIntVelocityActuator::UMjIntVelocityActuator()
{
	Type = EMjActuatorType::IntVelocity;
}

void UMjIntVelocityActuator::ExportTo(mjsActuator* Element, mjsDefault* def)
{
	if (!Element)
		return;

	Super::ExportTo(Element, def);

	// --- CODEGEN_EXPORT_START ---
	{
		double kvBuf[1] = {bOverride_kv ? (double)kv : -1.0};
		double dampratioBuf[1] = {bOverride_dampratio ? (double)dampratio : -1.0};
		const char* SetToErr = mjs_setToIntVelocity(Element, bOverride_kp ? (double)kp : Element->gainprm[0], bOverride_kv ? kvBuf : nullptr, bOverride_dampratio ? dampratioBuf : nullptr, nullptr, bOverride_inheritrange ? (double)inheritrange : 0.0);
		if (SetToErr && *SetToErr)
		{
			UE_LOG(LogURLabBind, Warning, TEXT("mjs_setToIntVelocity on '%s': %s"), *GetName(), UTF8_TO_TCHAR(SetToErr));
		}
	}
	if (bOverride_inheritrange)
		Element->inheritrange = inheritrange;
	// --- CODEGEN_EXPORT_END ---
}

void UMjIntVelocityActuator::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
	Super::ImportFromXml(Node, CompilerSettings);
	if (!Node)
		return;

	// --- CODEGEN_IMPORT_START ---
	MjXmlUtils::ReadAttrFloat(Node, TEXT("inheritrange"), inheritrange, bOverride_inheritrange);
	MjXmlUtils::ReadAttrFloat(Node, TEXT("kp"), kp, bOverride_kp);
	MjXmlUtils::ReadAttrFloat(Node, TEXT("kv"), kv, bOverride_kv);
	MjXmlUtils::ReadAttrFloat(Node, TEXT("dampratio"), dampratio, bOverride_dampratio);
	// --- CODEGEN_IMPORT_END ---
}
