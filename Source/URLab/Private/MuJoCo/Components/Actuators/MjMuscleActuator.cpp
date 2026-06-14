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

#include "MuJoCo/Components/Actuators/MjMuscleActuator.h"

#include "XmlNode.h"
#include "MuJoCo/Utils/MjXmlUtils.h"
#include "Utils/URLabLogging.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"

UMjMuscleActuator::UMjMuscleActuator()
{
	Type = EMjActuatorType::Muscle;
}

void UMjMuscleActuator::ExportTo(mjsActuator* Element, mjsDefault* def)
{
	if (!Element)
		return;

	// Super first populates common attrs (transmission target etc. — needed
	// for lengthrange auto-compute), then the codegen-emitted mjs_setToMuscle
	// call inside CODEGEN_EXPORT applies the muscle preset on top.
	Super::ExportTo(Element, def);

	// --- CODEGEN_EXPORT_START ---
	{
		double timeconstBuf[2] = {(bOverride_timeconst && timeconst.Num() > 0) ? (double)timeconst[0] : -1.0, (bOverride_timeconst && timeconst.Num() > 1) ? (double)timeconst[1] : -1.0};
		double rangeBuf[2] = {(bOverride_range && range.Num() > 0) ? (double)range[0] : -1.0, (bOverride_range && range.Num() > 1) ? (double)range[1] : -1.0};
		mjs_setToMuscle(Element, timeconstBuf, bOverride_tausmooth ? (double)tausmooth : 0.0, rangeBuf, bOverride_force ? (double)force : -1.0, (bOverride_scale && scale.Num() > 0) ? (double)scale[0] : -1.0, bOverride_lmin ? (double)lmin : -1.0, bOverride_lmax ? (double)lmax : -1.0, bOverride_vmax ? (double)vmax : -1.0, bOverride_fpmax ? (double)fpmax : -1.0, bOverride_fvmax ? (double)fvmax : -1.0);
	}
	// --- CODEGEN_EXPORT_END ---
}

void UMjMuscleActuator::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
	Super::ImportFromXml(Node, CompilerSettings);
	if (!Node)
		return;

	// --- CODEGEN_IMPORT_START ---
	MjXmlUtils::ReadAttrFloatArray(Node, TEXT("timeconst"), timeconst, bOverride_timeconst);
	MjXmlUtils::ReadAttrFloat(Node, TEXT("tausmooth"), tausmooth, bOverride_tausmooth);
	MjXmlUtils::ReadAttrFloatArray(Node, TEXT("range"), range, bOverride_range);
	MjXmlUtils::ReadAttrFloat(Node, TEXT("force"), force, bOverride_force);
	MjXmlUtils::ReadAttrFloatArray(Node, TEXT("scale"), scale, bOverride_scale);
	MjXmlUtils::ReadAttrFloat(Node, TEXT("lmin"), lmin, bOverride_lmin);
	MjXmlUtils::ReadAttrFloat(Node, TEXT("lmax"), lmax, bOverride_lmax);
	MjXmlUtils::ReadAttrFloat(Node, TEXT("vmax"), vmax, bOverride_vmax);
	MjXmlUtils::ReadAttrFloat(Node, TEXT("fpmax"), fpmax, bOverride_fpmax);
	MjXmlUtils::ReadAttrFloat(Node, TEXT("fvmax"), fvmax, bOverride_fvmax);
	// --- CODEGEN_IMPORT_END ---
}
