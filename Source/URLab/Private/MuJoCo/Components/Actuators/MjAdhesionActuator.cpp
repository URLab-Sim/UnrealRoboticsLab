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

#include "MuJoCo/Components/Actuators/MjAdhesionActuator.h"

#include "XmlNode.h"
#include "MuJoCo/Utils/MjXmlUtils.h"
#include "Utils/URLabLogging.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"

UMjAdhesionActuator::UMjAdhesionActuator()
{
	Type = EMjActuatorType::Adhesion;
}

void UMjAdhesionActuator::ExportTo(mjsActuator* Element, mjsDefault* def)
{
	if (!Element)
		return;

	Super::ExportTo(Element, def);

	// --- CODEGEN_EXPORT_START ---
	mjs_setToAdhesion(Element, bOverride_gain ? (double)gain : 0.0);
	// --- CODEGEN_EXPORT_END ---
}

void UMjAdhesionActuator::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
	Super::ImportFromXml(Node, CompilerSettings);
	if (!Node)
		return;

	// --- CODEGEN_IMPORT_START ---
	MjXmlUtils::ReadAttrFloat(Node, TEXT("gain"), gain, bOverride_gain);
	// --- CODEGEN_IMPORT_END ---
}
