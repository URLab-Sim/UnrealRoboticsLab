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

#include "MuJoCo/Components/Sensors/MjTorqueSensor.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"

UMjTorqueSensor::UMjTorqueSensor()
{
    Type = EMjSensorType::Torque;
}

void UMjTorqueSensor::ExportTo(mjsSensor* Element, mjsDefault* Default)
{
    if (!Element) return;

    Super::ExportTo(Element, Default);

    // --- CODEGEN_EXPORT_START ---

    // --- CODEGEN_EXPORT_END ---
}

void UMjTorqueSensor::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
    Super::ImportFromXml(Node, CompilerSettings);
    if (!Node) return;

    // --- CODEGEN_IMPORT_START ---

    // --- CODEGEN_IMPORT_END ---
}

