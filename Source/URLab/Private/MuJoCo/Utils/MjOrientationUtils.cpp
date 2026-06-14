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

#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "XmlNode.h"
#include "MuJoCo/Utils/MjXmlUtils.h"
#include "Utils/URLabLogging.h"
#include "mujoco/mujoco.h"

// ============================================================================
// Compiler Settings
// ============================================================================

FMjCompilerSettings MjOrientationUtils::ParseCompilerSettings(const FXmlNode* RootNode)
{
	FMjCompilerSettings Settings;
	if (!RootNode)
		return Settings;

	// Search for <compiler> element among children of the root <mujoco> node
	const FXmlNode* CompilerNode = nullptr;
	for (const FXmlNode* Child : RootNode->GetChildrenNodes())
	{
		if (Child->GetTag().Equals(TEXT("compiler"), ESearchCase::IgnoreCase))
		{
			CompilerNode = Child;
			break;
		}
		// Also check includes that might contain a compiler element at top level
		if (Child->GetTag().Equals(TEXT("include"), ESearchCase::IgnoreCase))
		{
			// Includes are resolved elsewhere; compiler in includes would need
			// the included file to be loaded. For now, we only check direct children.
		}
	}

	if (!CompilerNode)
		return Settings;

	// angle: "degree" or "radian". MuJoCo's default is "degree" (Settings default).
	FString AngleStr = CompilerNode->GetAttribute(TEXT("angle"));
	if (AngleStr.Equals(TEXT("radian"), ESearchCase::IgnoreCase))
	{
		Settings.bAngleInDegrees = false;
	}
	else if (AngleStr.Equals(TEXT("degree"), ESearchCase::IgnoreCase))
	{
		Settings.bAngleInDegrees = true;
	}

	// eulerseq: 3-character string (default "xyz")
	FString EulerSeqStr = CompilerNode->GetAttribute(TEXT("eulerseq"));
	if (!EulerSeqStr.IsEmpty() && EulerSeqStr.Len() == 3)
	{
		Settings.EulerSeq = EulerSeqStr;
	}

	// meshdir: base directory for mesh file lookups
	FString MeshDirStr = CompilerNode->GetAttribute(TEXT("meshdir"));
	if (!MeshDirStr.IsEmpty())
	{
		Settings.MeshDir = MeshDirStr;
	}

	// assetdir: base directory for all assets (overrides meshdir for mesh lookups)
	FString AssetDirStr = CompilerNode->GetAttribute(TEXT("assetdir"));
	if (!AssetDirStr.IsEmpty())
	{
		Settings.AssetDir = AssetDirStr;
	}

	// autolimits: joints/tendons with range are automatically limited
	FString AutoLimitsStr = CompilerNode->GetAttribute(TEXT("autolimits"));
	if (AutoLimitsStr.Equals(TEXT("true"), ESearchCase::IgnoreCase))
	{
		Settings.bAutoLimits = true;
	}

	UE_LOG(LogURLabImport, Log, TEXT("[MjOrientationUtils] Compiler settings: angle=%s, eulerseq=%s, meshdir=%s, assetdir=%s, autolimits=%s"),
		Settings.bAngleInDegrees ? TEXT("degree") : TEXT("radian"),
		*Settings.EulerSeq,
		*Settings.MeshDir,
		*Settings.AssetDir,
		Settings.bAutoLimits ? TEXT("true") : TEXT("false"));

	return Settings;
}

// ============================================================================
// Main Entry Point
// ============================================================================

bool MjOrientationUtils::OrientationToMjQuat(const FXmlNode* Node, const FMjCompilerSettings& Settings, double OutQuat[4])
{
	if (!Node)
	{
		OutQuat[0] = 1.0;
		OutQuat[1] = 0.0;
		OutQuat[2] = 0.0;
		OutQuat[3] = 0.0;
		return false;
	}

	// priority 1: quat (w x y z)
	FString QuatStr = Node->GetAttribute(TEXT("quat"));
	if (!QuatStr.IsEmpty())
	{
		TArray<float> Parts;
		MjXmlUtils::ParseFloatArray(QuatStr, Parts);
		if (Parts.Num() >= 4)
		{
			OutQuat[0] = (double)Parts[0];
			OutQuat[1] = (double)Parts[1];
			OutQuat[2] = (double)Parts[2];
			OutQuat[3] = (double)Parts[3];
			mju_normalize4(OutQuat);
			return true;
		}
	}

	// priority 2: axisangle (x y z angle)
	FString AxisAngleStr = Node->GetAttribute(TEXT("axisangle"));
	if (!AxisAngleStr.IsEmpty())
	{
		TArray<float> Parts;
		MjXmlUtils::ParseFloatArray(AxisAngleStr, Parts);
		if (Parts.Num() >= 4)
		{
			double Angle = (double)Parts[3];
			if (Settings.bAngleInDegrees)
			{
				Angle = FMath::DegreesToRadians(Angle);
			}
			AxisAngleToQuat((double)Parts[0], (double)Parts[1], (double)Parts[2], Angle, OutQuat);
			return true;
		}
	}

	// priority 3: euler (e1 e2 e3) — sequence from compiler settings
	FString EulerStr = Node->GetAttribute(TEXT("euler"));
	if (!EulerStr.IsEmpty())
	{
		TArray<float> Parts;
		MjXmlUtils::ParseFloatArray(EulerStr, Parts);
		if (Parts.Num() >= 3)
		{
			double e1 = (double)Parts[0];
			double e2 = (double)Parts[1];
			double e3 = (double)Parts[2];

			if (Settings.bAngleInDegrees)
			{
				e1 = FMath::DegreesToRadians(e1);
				e2 = FMath::DegreesToRadians(e2);
				e3 = FMath::DegreesToRadians(e3);
			}

			EulerToQuat(e1, e2, e3, Settings.EulerSeq, OutQuat);
			return true;
		}
	}

	// priority 4: xyaxes (x1 x2 x3 y1 y2 y3)
	FString XYAxesStr = Node->GetAttribute(TEXT("xyaxes"));
	if (!XYAxesStr.IsEmpty())
	{
		TArray<float> Parts;
		MjXmlUtils::ParseFloatArray(XYAxesStr, Parts);
		if (Parts.Num() >= 6)
		{
			double Vals[6];
			for (int i = 0; i < 6; ++i)
				Vals[i] = (double)Parts[i];
			XYAxesToQuat(Vals, OutQuat);
			return true;
		}
	}

	// priority 5: zaxis (z1 z2 z3)
	FString ZAxisStr = Node->GetAttribute(TEXT("zaxis"));
	if (!ZAxisStr.IsEmpty())
	{
		TArray<float> Parts;
		MjXmlUtils::ParseFloatArray(ZAxisStr, Parts);
		if (Parts.Num() >= 3)
		{
			double Vals[3] = {(double)Parts[0], (double)Parts[1], (double)Parts[2]};
			ZAxisToQuat(Vals, OutQuat);
			return true;
		}
	}

	// No orientation attribute found — identity
	OutQuat[0] = 1.0;
	OutQuat[1] = 0.0;
	OutQuat[2] = 0.0;
	OutQuat[3] = 0.0;
	return false;
}

// ============================================================================
// Axis-Angle → Quaternion
// ============================================================================

void MjOrientationUtils::AxisAngleToQuat(double x, double y, double z, double AngleRad, double OutQuat[4])
{
	double Axis[3] = {x, y, z};
	if (mju_normalize3(Axis) < 1e-10)
	{
		OutQuat[0] = 1.0;
		OutQuat[1] = 0.0;
		OutQuat[2] = 0.0;
		OutQuat[3] = 0.0;
		return;
	}
	mju_axisAngle2Quat(OutQuat, Axis, AngleRad);
}

// ============================================================================
// Euler → Quaternion
// ============================================================================

void MjOrientationUtils::EulerToQuat(double e1, double e2, double e3, const FString& EulerSeq, double OutQuat[4])
{
	if (EulerSeq.Len() != 3)
	{
		UE_LOG(LogURLabImport, Warning, TEXT("[MjOrientationUtils] Invalid eulerseq '%s', defaulting to identity"), *EulerSeq);
		OutQuat[0] = 1.0;
		OutQuat[1] = 0.0;
		OutQuat[2] = 0.0;
		OutQuat[3] = 0.0;
		return;
	}

	double Euler[3] = {e1, e2, e3};
	const auto SeqAnsi = StringCast<ANSICHAR>(*EulerSeq);
	mju_euler2Quat(OutQuat, Euler, SeqAnsi.Get());
}

// ============================================================================
// XY-Axes → Quaternion
// ============================================================================

void MjOrientationUtils::XYAxesToQuat(const double XYAxes[6], double OutQuat[4])
{
	double X[3] = {XYAxes[0], XYAxes[1], XYAxes[2]};
	if (mju_normalize3(X) < 1e-10)
	{
		OutQuat[0] = 1.0;
		OutQuat[1] = 0.0;
		OutQuat[2] = 0.0;
		OutQuat[3] = 0.0;
		return;
	}

	double Y[3] = {XYAxes[3], XYAxes[4], XYAxes[5]};
	const double Dot = X[0] * Y[0] + X[1] * Y[1] + X[2] * Y[2];
	Y[0] -= Dot * X[0];
	Y[1] -= Dot * X[1];
	Y[2] -= Dot * X[2];
	if (mju_normalize3(Y) < 1e-10)
	{
		OutQuat[0] = 1.0;
		OutQuat[1] = 0.0;
		OutQuat[2] = 0.0;
		OutQuat[3] = 0.0;
		return;
	}

	double Z[3];
	mju_cross(Z, X, Y);
	if (mju_normalize3(Z) < 1e-10)
	{
		OutQuat[0] = 1.0;
		OutQuat[1] = 0.0;
		OutQuat[2] = 0.0;
		OutQuat[3] = 0.0;
		return;
	}

	// MuJoCo rotation matrix is row-major with COLUMNS = local axes (mjuu_frame2quat convention).
	const double Mat[9] = {
		X[0], Y[0], Z[0],
		X[1], Y[1], Z[1],
		X[2], Y[2], Z[2]};
	mju_mat2Quat(OutQuat, Mat);
}

// ============================================================================
// Z-Axis → Quaternion (minimal rotation from (0,0,1))
// ============================================================================

void MjOrientationUtils::ZAxisToQuat(const double ZAxis[3], double OutQuat[4])
{
	double Vec[3] = {ZAxis[0], ZAxis[1], ZAxis[2]};
	if (mju_normalize3(Vec) < 1e-10)
	{
		OutQuat[0] = 1.0;
		OutQuat[1] = 0.0;
		OutQuat[2] = 0.0;
		OutQuat[3] = 0.0;
		return;
	}

	if (Vec[2] > 0.9999999)
	{
		OutQuat[0] = 1.0;
		OutQuat[1] = 0.0;
		OutQuat[2] = 0.0;
		OutQuat[3] = 0.0;
		return;
	}
	if (Vec[2] < -0.9999999)
	{
		OutQuat[0] = 0.0;
		OutQuat[1] = 1.0;
		OutQuat[2] = 0.0;
		OutQuat[3] = 0.0;
		return;
	}

	// axis = cross((0,0,1), Vec) = (-Vec[1], Vec[0], 0)
	double Axis[3] = {-Vec[1], Vec[0], 0.0};
	const double Angle = FMath::Acos(FMath::Clamp(Vec[2], -1.0, 1.0));
	mju_normalize3(Axis);
	mju_axisAngle2Quat(OutQuat, Axis, Angle);
}
