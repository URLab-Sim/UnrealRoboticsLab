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

#include "MuJoCo/Components/Physics/MjInertial.h"
#include "mujoco/mujoco.h"

#include "MuJoCo/Utils/MjUtils.h"
#include "MuJoCo/Utils/MjXmlUtils.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "XmlNode.h"

UMjInertial::UMjInertial()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UMjInertial::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
	if (!Node)
		return;

	// --- CODEGEN_IMPORT_START ---
	MjXmlUtils::ReadAttrFloat(Node, TEXT("mass"), mass, bOverride_mass);
	MjXmlUtils::ReadAttrFloatArray(Node, TEXT("diaginertia"), diaginertia, bOverride_diaginertia);
	MjXmlUtils::ReadAttrFloatArray(Node, TEXT("fullinertia"), fullinertia, bOverride_fullinertia);
	MjUtils::ReadVec3InMeters(Node, TEXT("pos"), Pos, bOverride_Pos);
	{ // canonicalize orientation (quat/euler/axisangle/xyaxes/zaxis)
		double TmpQuat[4] = {1.0, 0.0, 0.0, 0.0};
		if (MjOrientationUtils::OrientationToMjQuat(Node, CompilerSettings, TmpQuat))
		{
			Quat = MjUtils::MjToUERotation(TmpQuat);
			bOverride_Quat = true;
		}
	}
	if (bOverride_Pos)
		SetRelativeLocation(Pos);
	if (bOverride_Quat)
		SetRelativeRotation(Quat);
	// --- CODEGEN_IMPORT_END ---
}

void UMjInertial::RegisterToSpec(FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody)
{
	if (!ParentBody)
		return;

	// Set explicit inertial
	ParentBody->explicitinertial = 1;

	ParentBody->mass = mass;
	if (diaginertia.Num() >= 3)
	{
		ParentBody->inertia[0] = diaginertia[0];
		ParentBody->inertia[1] = diaginertia[1];
		ParentBody->inertia[2] = diaginertia[2];
	}

	if (fullinertia.Num() == 6)
	{
		for (int i = 0; i < 6; i++)
		{
			ParentBody->fullinertia[i] = fullinertia[i];
		}
	}

	// Transform from Component (Relative to Parent Body)
	FTransform RelTrans = GetRelativeTransform();
	MjUtils::UEToMjPosition(RelTrans.GetLocation(), ParentBody->ipos);
	MjUtils::UEToMjRotation(RelTrans.GetRotation(), ParentBody->iquat);
}
