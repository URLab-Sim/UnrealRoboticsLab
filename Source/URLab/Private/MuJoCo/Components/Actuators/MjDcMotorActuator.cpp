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

#include "MuJoCo/Components/Actuators/MjDcMotorActuator.h"

#include "XmlNode.h"
#include "MuJoCo/Utils/MjXmlUtils.h"
#include "Utils/URLabLogging.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"

UMjDcMotorActuator::UMjDcMotorActuator()
{
	Type = EMjActuatorType::DcMotor;
}

namespace
{
// Copy UE TArray<float> into a double[N] buffer, zero-padding the tail.
template <int N>
void FillDouble(double (&Out)[N], const TArray<float>& In)
{
	for (int i = 0; i < N; ++i)
	{
		Out[i] = (i < In.Num()) ? (double)In[i] : 0.0;
	}
}
} // namespace

void UMjDcMotorActuator::ExportTo(mjsActuator* Element, mjsDefault* Default)
{
	if (!Element)
		return;

	// mjs_setToDCMotor packs the per-actuator params into mjsActuator's
	// gainprm/biasprm/dynprm arrays. The call itself is codegen-emitted in
	// the CODEGEN_EXPORT block below; it uses the file-local FillDouble
	// helper to convert TArray<float> -> double[N] buffers.
	Super::ExportTo(Element, Default);

	// --- CODEGEN_EXPORT_START ---
	{
		double motorconstBuf[2];
		FillDouble(motorconstBuf, motorconst);
		double nominalBuf[3];
		FillDouble(nominalBuf, nominal);
		double saturationBuf[3];
		FillDouble(saturationBuf, saturation);
		double inductanceBuf[2];
		FillDouble(inductanceBuf, inductance);
		double coggingBuf[3];
		FillDouble(coggingBuf, cogging);
		double controllerBuf[6];
		FillDouble(controllerBuf, controller);
		double thermalBuf[6];
		FillDouble(thermalBuf, thermal);
		double lugreBuf[5];
		FillDouble(lugreBuf, lugre);
		mjs_setToDCMotor(Element, bOverride_motorconst ? motorconstBuf : nullptr, bOverride_resistance ? (double)resistance : 0.0, bOverride_nominal ? nominalBuf : nullptr, bOverride_saturation ? saturationBuf : nullptr, bOverride_inductance ? inductanceBuf : nullptr, bOverride_cogging ? coggingBuf : nullptr, bOverride_controller ? controllerBuf : nullptr, bOverride_thermal ? thermalBuf : nullptr, bOverride_lugre ? lugreBuf : nullptr, bOverride_input ? (int)input : 0);
	}
	// --- CODEGEN_EXPORT_END ---
}

void UMjDcMotorActuator::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
	Super::ImportFromXml(Node, CompilerSettings);
	if (!Node)
		return;

	// --- CODEGEN_IMPORT_START ---
	{ // xml_enum: input -> EMjDcMotorInput
		FString S = Node->GetAttribute(TEXT("input"));
		S = S.ToLower();
		bool bMatched = false;
		if (S == TEXT("voltage"))
		{
			input = EMjDcMotorInput::Voltage;
			bMatched = true;
		}
		else if (S == TEXT("position"))
		{
			input = EMjDcMotorInput::Position;
			bMatched = true;
		}
		else if (S == TEXT("velocity"))
		{
			input = EMjDcMotorInput::Velocity;
			bMatched = true;
		}
		if (bMatched)
			bOverride_input = true;
	}
	MjXmlUtils::ReadAttrFloatArray(Node, TEXT("motorconst"), motorconst, bOverride_motorconst);
	MjXmlUtils::ReadAttrFloat(Node, TEXT("resistance"), resistance, bOverride_resistance);
	MjXmlUtils::ReadAttrFloatArray(Node, TEXT("nominal"), nominal, bOverride_nominal);
	MjXmlUtils::ReadAttrFloatArray(Node, TEXT("saturation"), saturation, bOverride_saturation);
	MjXmlUtils::ReadAttrFloatArray(Node, TEXT("inductance"), inductance, bOverride_inductance);
	MjXmlUtils::ReadAttrFloatArray(Node, TEXT("cogging"), cogging, bOverride_cogging);
	MjXmlUtils::ReadAttrFloatArray(Node, TEXT("controller"), controller, bOverride_controller);
	MjXmlUtils::ReadAttrFloatArray(Node, TEXT("thermal"), thermal, bOverride_thermal);
	MjXmlUtils::ReadAttrFloatArray(Node, TEXT("lugre"), lugre, bOverride_lugre);
	// --- CODEGEN_IMPORT_END ---
}
