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

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "MuJoCo/Components/Sensors/MjSensor.h"
#include "MuJoCo/Components/Sensors/MjUserSensor.h"
#include "MuJoCo/Components/Sensors/MjPluginSensor.h"
#include "MuJoCo/Generated/MjSensorTypeInfo.h"
#include "State/MjStateTypes.h"

// The FMjSensorTypeInfo descriptor table is the single source of truth that
// replaced the six per-type switches in MjSensor.cpp and the editor XML
// parser. These tests pin the descriptor contract each of those consumers
// relies on.

// ============================================================================
// URLab.SensorTypeInfo.TableCoversEveryType
//   Every EMjSensorType has exactly one descriptor row, and both the by-type
//   and by-tag lookups round-trip against it.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSensorTypeInfoTableComplete,
	"URLab.SensorTypeInfo.TableCoversEveryType",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjSensorTypeInfoTableComplete::RunTest(const FString& Parameters)
{
	const UEnum* Enum = StaticEnum<EMjSensorType>();
	if (!TestNotNull(TEXT("EMjSensorType reflection"), Enum))
		return false;

	TSet<EMjSensorType> Seen;
	for (int32 i = 0; i < Enum->NumEnums(); ++i)
	{
		const FString Name = Enum->GetNameStringByIndex(i);
		if (Name.EndsWith(TEXT("_MAX")))
			continue;

		const EMjSensorType Type = static_cast<EMjSensorType>(Enum->GetValueByIndex(i));
		const FMjSensorTypeInfo& Info = MjSensorTypeInfoFor(Type);
		TestEqual(FString::Printf(TEXT("descriptor.Type matches for %s"), *Name),
			Info.Type, Type);
		TestNotNull(FString::Printf(TEXT("SensorClass set for %s"), *Name),
			Info.SensorClass);

		// The MJCF tag must round-trip back to the same row.
		const FMjSensorTypeInfo* ByTag = MjSensorTypeInfoForTag(FString(Info.Tag));
		if (TestNotNull(FString::Printf(TEXT("tag lookup for %s"), *Name), ByTag))
			TestEqual(FString::Printf(TEXT("tag round-trip for %s"), *Name),
				ByTag->Type, Type);

		Seen.Add(Type);
	}

	// Table row count matches the number of live enum values.
	TestEqual(TEXT("one descriptor row per enum value"),
		MjSensorTypeInfoTable().Num(), Seen.Num());
	return true;
}

// ============================================================================
// URLab.SensorTypeInfo.TagLookupIsCaseInsensitiveAndRejectsUnknown
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSensorTypeInfoTagLookup,
	"URLab.SensorTypeInfo.TagLookupIsCaseInsensitiveAndRejectsUnknown",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjSensorTypeInfoTagLookup::RunTest(const FString& Parameters)
{
	const FMjSensorTypeInfo* Lower = MjSensorTypeInfoForTag(TEXT("gyro"));
	const FMjSensorTypeInfo* Upper = MjSensorTypeInfoForTag(TEXT("GyRo"));
	if (TestNotNull(TEXT("lowercase gyro"), Lower)
		&& TestNotNull(TEXT("mixed-case gyro"), Upper))
	{
		TestEqual(TEXT("case-insensitive tag maps to Gyro"), Lower->Type, EMjSensorType::Gyro);
		TestEqual(TEXT("case variants resolve identically"), Lower->Type, Upper->Type);
	}

	TestNull(TEXT("unknown tag returns null"),
		MjSensorTypeInfoForTag(TEXT("not_a_sensor")));
	TestNull(TEXT("bare <sensor> container is not a descriptor"),
		MjSensorTypeInfoForTag(TEXT("sensor")));
	return true;
}

// ============================================================================
// URLab.SensorTypeInfo.ObjRefPolicyMatchesExportRules
//   Spot-check the objtype/reftype export policy for each policy class.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSensorTypeInfoObjRefPolicy,
	"URLab.SensorTypeInfo.ObjRefPolicyMatchesExportRules",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjSensorTypeInfoObjRefPolicy::RunTest(const FString& Parameters)
{
	// Static objtype, no reference (site-attached sensor).
	const FMjSensorTypeInfo& Touch = MjSensorTypeInfoFor(EMjSensorType::Touch);
	TestEqual(TEXT("touch obj is static"), Touch.ObjSource, EMjSensorObjSource::Static);
	TestEqual(TEXT("touch obj is site"), Touch.ObjType, (int32)mjOBJ_SITE);
	TestEqual(TEXT("touch has no ref"), Touch.RefSource, EMjSensorObjSource::None);

	// Static objtype + static reftype (camprojection: site -> camera).
	const FMjSensorTypeInfo& CamProj = MjSensorTypeInfoFor(EMjSensorType::CamProjection);
	TestEqual(TEXT("camprojection ref is static"), CamProj.RefSource, EMjSensorObjSource::Static);
	TestEqual(TEXT("camprojection ref is camera"), CamProj.RefType, (int32)mjOBJ_CAMERA);

	// Computed objtype (rangefinder: camera or site), no reference.
	const FMjSensorTypeInfo& Range = MjSensorTypeInfoFor(EMjSensorType::RangeFinder);
	TestEqual(TEXT("rangefinder obj is computed"), Range.ObjSource, EMjSensorObjSource::Computed);
	TestEqual(TEXT("rangefinder has no ref"), Range.RefSource, EMjSensorObjSource::None);

	// Frame sensors read both objtype and reftype from the UE properties.
	const FMjSensorTypeInfo& FramePos = MjSensorTypeInfoFor(EMjSensorType::FramePos);
	TestEqual(TEXT("framepos obj from xml"), FramePos.ObjSource, EMjSensorObjSource::FromXml);
	TestEqual(TEXT("framepos ref from xml"), FramePos.RefSource, EMjSensorObjSource::FromXml);

	// insidesite: objtype from xml, reftype fixed to site.
	const FMjSensorTypeInfo& Inside = MjSensorTypeInfoFor(EMjSensorType::InsideSite);
	TestEqual(TEXT("insidesite obj from xml"), Inside.ObjSource, EMjSensorObjSource::FromXml);
	TestEqual(TEXT("insidesite ref is static site"), Inside.RefSource, EMjSensorObjSource::Static);
	TestEqual(TEXT("insidesite ref is site"), Inside.RefType, (int32)mjOBJ_SITE);

	// user reads objtype from xml but never writes a reftype; plugin writes both.
	TestEqual(TEXT("user ref is none"),
		MjSensorTypeInfoFor(EMjSensorType::User).RefSource, EMjSensorObjSource::None);
	TestEqual(TEXT("plugin ref from xml"),
		MjSensorTypeInfoFor(EMjSensorType::Plugin).RefSource, EMjSensorObjSource::FromXml);
	return true;
}

// ============================================================================
// URLab.SensorTypeInfo.SemanticValueKindAndDim
//   The metadata that feeds DescribeState (Semantic), TransformSensorReading
//   (ValueKind) and the MuJoCo output dimension.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSensorTypeInfoSemanticValueKindDim,
	"URLab.SensorTypeInfo.SemanticValueKindAndDim",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjSensorTypeInfoSemanticValueKindDim::RunTest(const FString& Parameters)
{
	const FMjSensorTypeInfo& Gyro = MjSensorTypeInfoFor(EMjSensorType::Gyro);
	TestEqual(TEXT("gyro semantic"), Gyro.Semantic, EMjSensorSemantic::Gyro);
	TestEqual(TEXT("gyro value kind"), Gyro.ValueKind, EMjSensorValueKind::Vector3);
	TestEqual(TEXT("gyro dim"), Gyro.FixedDim, 3);

	const FMjSensorTypeInfo& FrameQuat = MjSensorTypeInfoFor(EMjSensorType::FrameQuat);
	TestEqual(TEXT("framequat value kind"), FrameQuat.ValueKind, EMjSensorValueKind::Quaternion);
	TestEqual(TEXT("framequat dim"), FrameQuat.FixedDim, 4);

	const FMjSensorTypeInfo& FramePos = MjSensorTypeInfoFor(EMjSensorType::FramePos);
	TestEqual(TEXT("framepos value kind"), FramePos.ValueKind, EMjSensorValueKind::Position);

	const FMjSensorTypeInfo& FrameXAxis = MjSensorTypeInfoFor(EMjSensorType::FrameXAxis);
	TestEqual(TEXT("framexaxis semantic"), FrameXAxis.Semantic, EMjSensorSemantic::FrameAxis);
	TestEqual(TEXT("framexaxis value kind"), FrameXAxis.ValueKind, EMjSensorValueKind::Direction);

	const FMjSensorTypeInfo& FromTo = MjSensorTypeInfoFor(EMjSensorType::GeomFromTo);
	TestEqual(TEXT("fromto value kind"), FromTo.ValueKind, EMjSensorValueKind::GeomFromTo);
	TestEqual(TEXT("fromto dim"), FromTo.FixedDim, 6);

	// camprojection is a two-component scalar output.
	TestEqual(TEXT("camprojection dim"),
		MjSensorTypeInfoFor(EMjSensorType::CamProjection).FixedDim, 2);

	// Variable-length sensors report -1.
	TestEqual(TEXT("user dim is variable"),
		MjSensorTypeInfoFor(EMjSensorType::User).FixedDim, -1);
	TestEqual(TEXT("contact dim is variable"),
		MjSensorTypeInfoFor(EMjSensorType::Contact).FixedDim, -1);

	// A plain scalar (jointpos) carries the Generic->no-transform default.
	TestEqual(TEXT("jointpos value kind"),
		MjSensorTypeInfoFor(EMjSensorType::JointPos).ValueKind, EMjSensorValueKind::Scalar);
	return true;
}

// ============================================================================
// URLab.SensorTypeInfo.UnknownTypeFallsBackToAccelerometer
//   Mirrors the historical ExportTo default arm.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSensorTypeInfoFallback,
	"URLab.SensorTypeInfo.UnknownTypeFallsBackToAccelerometer",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjSensorTypeInfoFallback::RunTest(const FString& Parameters)
{
	const FMjSensorTypeInfo& Info = MjSensorTypeInfoFor(static_cast<EMjSensorType>(0xFE));
	TestEqual(TEXT("unmapped type falls back to accelerometer"),
		Info.Type, EMjSensorType::Accelerometer);
	return true;
}

// ============================================================================
// URLab.SensorTypeInfo.UserAndPluginTagsResolveToTheirSubclasses
//   Regression: the editor XML parser's tag -> UClass chain omitted the
//   <user> and <plugin> cases, so those tags silently created a base
//   UMjSensor. The descriptor table now drives that mapping.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSensorTypeInfoUserPluginSubclass,
	"URLab.SensorTypeInfo.UserAndPluginTagsResolveToTheirSubclasses",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjSensorTypeInfoUserPluginSubclass::RunTest(const FString& Parameters)
{
	const FMjSensorTypeInfo* User = MjSensorTypeInfoForTag(TEXT("user"));
	const FMjSensorTypeInfo* Plugin = MjSensorTypeInfoForTag(TEXT("plugin"));
	if (TestNotNull(TEXT("user descriptor"), User)
		&& TestNotNull(TEXT("plugin descriptor"), Plugin))
	{
		TestEqual(TEXT("user tag -> UMjUserSensor"),
			User->SensorClass, UMjUserSensor::StaticClass());
		TestEqual(TEXT("plugin tag -> UMjPluginSensor"),
			Plugin->SensorClass, UMjPluginSensor::StaticClass());
		// The bug produced the base class; assert we are past it.
		TestNotEqual(TEXT("user is not the base UMjSensor"),
			User->SensorClass, UMjSensor::StaticClass());
	}
	return true;
}
