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

#include "MuJoCo/Components/Sensors/MjSensor.h"
#include "MuJoCo/Utils/MjXmlUtils.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "XmlNode.h"
#include "Utils/URLabLogging.h"
#include "MuJoCo/Components/Defaults/MjDefault.h"
#include "MuJoCo/Core/Spec/MjSpecWrapper.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "MuJoCo/Components/Geometry/MjGeom.h"
#include "MuJoCo/Components/Geometry/MjSite.h"
#include "MuJoCo/Components/Tendons/MjTendon.h"
#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "State/MjStateTypes.h"
#include "State/MjCanonicalName.h"
#include "MuJoCo/Generated/MjSensorTypeInfo.h"

UMjSensor::UMjSensor()
{
	PrimaryComponentTick.bCanEverTick = false;

	Type = EMjSensorType::Accelerometer;
	ObjType = EMjObjType::Site;
	RefType = EMjObjType::Unknown;
	Dim = 3;
	noise = 0.0f;
	cutoff = 0.0f;
}

static EMjObjType MjObjToEnum(int obj)
{
	switch (obj)
	{
		case mjOBJ_BODY:
			return EMjObjType::Body;
		case mjOBJ_XBODY:
			return EMjObjType::XBody;
		case mjOBJ_JOINT:
			return EMjObjType::Joint;
		case mjOBJ_DOF:
			return EMjObjType::DoF;
		case mjOBJ_GEOM:
			return EMjObjType::Geom;
		case mjOBJ_SITE:
			return EMjObjType::Site;
		case mjOBJ_CAMERA:
			return EMjObjType::Camera;
		case mjOBJ_LIGHT:
			return EMjObjType::Light;
		case mjOBJ_MESH:
			return EMjObjType::Mesh;
		case mjOBJ_HFIELD:
			return EMjObjType::HField;
		case mjOBJ_TEXTURE:
			return EMjObjType::Texture;
		case mjOBJ_MATERIAL:
			return EMjObjType::Material;
		case mjOBJ_PAIR:
			return EMjObjType::Pair;
		case mjOBJ_EXCLUDE:
			return EMjObjType::Exclude;
		case mjOBJ_EQUALITY:
			return EMjObjType::Equality;
		case mjOBJ_TENDON:
			return EMjObjType::Tendon;
		case mjOBJ_ACTUATOR:
			return EMjObjType::Actuator;
		case mjOBJ_SENSOR:
			return EMjObjType::Sensor;
		case mjOBJ_NUMERIC:
			return EMjObjType::Numeric;
		case mjOBJ_TEXT:
			return EMjObjType::Text;
		case mjOBJ_TUPLE:
			return EMjObjType::Tuple;
		case mjOBJ_KEY:
			return EMjObjType::Key;
		case mjOBJ_PLUGIN:
			return EMjObjType::Plugin;
		default:
			return EMjObjType::Unknown;
	}
}

static int EnumToMjObj(EMjObjType Type)
{
	switch (Type)
	{
		case EMjObjType::Body:
			return mjOBJ_BODY;
		case EMjObjType::XBody:
			return mjOBJ_XBODY;
		case EMjObjType::Joint:
			return mjOBJ_JOINT;
		case EMjObjType::DoF:
			return mjOBJ_DOF;
		case EMjObjType::Geom:
			return mjOBJ_GEOM;
		case EMjObjType::Site:
			return mjOBJ_SITE;
		case EMjObjType::Camera:
			return mjOBJ_CAMERA;
		case EMjObjType::Light:
			return mjOBJ_LIGHT;
		case EMjObjType::Mesh:
			return mjOBJ_MESH;
		case EMjObjType::HField:
			return mjOBJ_HFIELD;
		case EMjObjType::Texture:
			return mjOBJ_TEXTURE;
		case EMjObjType::Material:
			return mjOBJ_MATERIAL;
		case EMjObjType::Pair:
			return mjOBJ_PAIR;
		case EMjObjType::Exclude:
			return mjOBJ_EXCLUDE;
		case EMjObjType::Equality:
			return mjOBJ_EQUALITY;
		case EMjObjType::Tendon:
			return mjOBJ_TENDON;
		case EMjObjType::Actuator:
			return mjOBJ_ACTUATOR;
		case EMjObjType::Sensor:
			return mjOBJ_SENSOR;
		case EMjObjType::Numeric:
			return mjOBJ_NUMERIC;
		case EMjObjType::Text:
			return mjOBJ_TEXT;
		case EMjObjType::Tuple:
			return mjOBJ_TUPLE;
		case EMjObjType::Key:
			return mjOBJ_KEY;
		case EMjObjType::Plugin:
			return mjOBJ_PLUGIN;
		default:
			return mjOBJ_UNKNOWN;
	}
}

void UMjSensor::ExportTo(mjsSensor* Element, mjsDefault* Default)
{
	if (!Element)
		return;

	MjSetString(Element->objname, TargetName);
	MjSetString(Element->refname, ReferenceName);

	// Most built-in sensor types have `dim` derived from the sensor type
	// during mj_compile (e.g. accelerometer dim=3). Writing 0 here zeros
	// out the spec's compiler-derived value. Only write when the user has
	// explicitly overridden it (custom user/plugin sensors, or sensors
	// where dim is genuinely user-controlled).
	if (Dim > 0)
		Element->dim = Dim;

	// UserAdr is read-only runtime data (mjModel::sensor_adr), not writable via mjsSensor.

	for (int i = 0; i < IntParams.Num() && i < 3; i++)
		Element->intprm[i] = IntParams[i];

	// Sensor type, objtype, and reftype all come from the codegen-emitted
	// FMjSensorTypeInfo descriptor (MuJoCo/Generated/MjSensorTypeInfo.h).
	// The descriptor says whether each of objtype/reftype is a fixed mjOBJ_*
	// literal, read from the UE ObjType/RefType property, computed from the
	// attachment (rangefinder), or left unset.
	const FMjSensorTypeInfo& Info = MjSensorTypeInfoFor(Type);
	Element->type = (mjtSensor)Info.MjType;

	switch (Info.ObjSource)
	{
		case EMjSensorObjSource::Static:
			Element->objtype = (mjtObj)Info.ObjType;
			break;
		case EMjSensorObjSource::FromXml:
			Element->objtype = (mjtObj)EnumToMjObj(ObjType);
			break;
		case EMjSensorObjSource::Computed:
			Element->objtype = (ObjType == EMjObjType::Camera) ? mjOBJ_CAMERA : mjOBJ_SITE;
			break;
		case EMjSensorObjSource::None:
			break;
	}

	switch (Info.RefSource)
	{
		case EMjSensorObjSource::Static:
			Element->reftype = (mjtObj)Info.RefType;
			break;
		case EMjSensorObjSource::FromXml:
			Element->reftype = (mjtObj)EnumToMjObj(RefType);
			break;
		case EMjSensorObjSource::Computed:
		case EMjSensorObjSource::None:
			break;
	}

	// --- CODEGEN_EXPORT_START ---
	if (bOverride_nsample)
		Element->nsample = nsample;
	if (bOverride_interp)
		Element->interp = interp;
	if (bOverride_delay)
		Element->delay = delay;
	if (bOverride_interval)
	{
		for (int32 i = 0; i < FMath::Min(interval.Num(), 2); ++i)
			Element->interval[i] = interval[i];
	}
	if (bOverride_cutoff)
		Element->cutoff = cutoff;
	if (bOverride_noise)
		Element->noise = noise;
	MjSetString(Element->objname, TargetName);
	MjSetString(Element->refname, ReferenceName);
	// --- CODEGEN_EXPORT_END ---
}

void UMjSensor::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
	if (!Node)
		return;

	// --- CODEGEN_IMPORT_START ---
	MjXmlUtils::ReadAttrString(Node, TEXT("class"), MjClassName);
	{ // xml_enum: objtype -> EMjObjType
		FString S = Node->GetAttribute(TEXT("objtype"));
		S = S.ToLower();
		if (S == TEXT("body"))
		{
			ObjType = EMjObjType::Body;
		}
		else if (S == TEXT("xbody"))
		{
			ObjType = EMjObjType::XBody;
		}
		else if (S == TEXT("joint"))
		{
			ObjType = EMjObjType::Joint;
		}
		else if (S == TEXT("dof"))
		{
			ObjType = EMjObjType::DoF;
		}
		else if (S == TEXT("geom"))
		{
			ObjType = EMjObjType::Geom;
		}
		else if (S == TEXT("site"))
		{
			ObjType = EMjObjType::Site;
		}
		else if (S == TEXT("camera"))
		{
			ObjType = EMjObjType::Camera;
		}
		else if (S == TEXT("light"))
		{
			ObjType = EMjObjType::Light;
		}
		else if (S == TEXT("mesh"))
		{
			ObjType = EMjObjType::Mesh;
		}
		else if (S == TEXT("hfield"))
		{
			ObjType = EMjObjType::HField;
		}
		else if (S == TEXT("texture"))
		{
			ObjType = EMjObjType::Texture;
		}
		else if (S == TEXT("material"))
		{
			ObjType = EMjObjType::Material;
		}
		else if (S == TEXT("pair"))
		{
			ObjType = EMjObjType::Pair;
		}
		else if (S == TEXT("exclude"))
		{
			ObjType = EMjObjType::Exclude;
		}
		else if (S == TEXT("equality"))
		{
			ObjType = EMjObjType::Equality;
		}
		else if (S == TEXT("tendon"))
		{
			ObjType = EMjObjType::Tendon;
		}
		else if (S == TEXT("actuator"))
		{
			ObjType = EMjObjType::Actuator;
		}
	}
	{ // xml_enum: reftype -> EMjObjType
		FString S = Node->GetAttribute(TEXT("reftype"));
		S = S.ToLower();
		if (S == TEXT("body"))
		{
			RefType = EMjObjType::Body;
		}
		else if (S == TEXT("xbody"))
		{
			RefType = EMjObjType::XBody;
		}
		else if (S == TEXT("joint"))
		{
			RefType = EMjObjType::Joint;
		}
		else if (S == TEXT("geom"))
		{
			RefType = EMjObjType::Geom;
		}
		else if (S == TEXT("site"))
		{
			RefType = EMjObjType::Site;
		}
		else if (S == TEXT("camera"))
		{
			RefType = EMjObjType::Camera;
		}
	}
	MjXmlUtils::ReadAttrInt(Node, TEXT("nsample"), nsample, bOverride_nsample);
	MjXmlUtils::ReadAttrInt(Node, TEXT("interp"), interp, bOverride_interp);
	MjXmlUtils::ReadAttrFloat(Node, TEXT("delay"), delay, bOverride_delay);
	MjXmlUtils::ReadAttrFloatArray(Node, TEXT("interval"), interval, bOverride_interval);
	MjXmlUtils::ReadAttrFloat(Node, TEXT("cutoff"), cutoff, bOverride_cutoff);
	MjXmlUtils::ReadAttrFloat(Node, TEXT("noise"), noise, bOverride_noise);
	// target_collation: -> TargetName
	TargetName = Node->GetAttribute(TEXT("site"));
	if (TargetName.IsEmpty())
		TargetName = Node->GetAttribute(TEXT("joint"));
	if (TargetName.IsEmpty())
		TargetName = Node->GetAttribute(TEXT("tendon"));
	if (TargetName.IsEmpty())
		TargetName = Node->GetAttribute(TEXT("actuator"));
	if (TargetName.IsEmpty())
		TargetName = Node->GetAttribute(TEXT("body"));
	if (TargetName.IsEmpty())
		TargetName = Node->GetAttribute(TEXT("geom"));
	if (TargetName.IsEmpty())
		TargetName = Node->GetAttribute(TEXT("objname"));
	if (TargetName.IsEmpty())
		TargetName = Node->GetAttribute(TEXT("camera"));
	if (TargetName.IsEmpty())
		TargetName = Node->GetAttribute(TEXT("mesh"));
	// target_collation: -> ReferenceName
	ReferenceName = Node->GetAttribute(TEXT("refname"));
	// --- CODEGEN_IMPORT_END ---

	// Determine sensor type from the XML tag name via the descriptor table.
	const FString Tag = Node->GetTag().ToLower();
	if (const FMjSensorTypeInfo* Info = MjSensorTypeInfoForTag(Tag))
		Type = Info->Type;

	// TargetName / ReferenceName (target_collations), ObjType / RefType
	// (xml_enum_attrs), MjClassName (common_imports) are all codegen-emitted
	// inside the CODEGEN_IMPORT block above. dim / adr remain hand-written
	// here because they need bespoke handling: dim is conditionally written
	// (only if XML provides it, preserving the compile-time default), and
	// adr is an out-of-band override for user sensors only.

	// dim override — used by user sensors where the dimension is not fixed by type
	FString DimStr = Node->GetAttribute(TEXT("dim"));
	if (!DimStr.IsEmpty())
	{
		int32 ParsedDim = FCString::Atoi(*DimStr);
		if (ParsedDim > 0)
			Dim = ParsedDim;
	}

	// adr — output address override for user sensors (written to mjsSensor::adr)
	{
		bool bAdrOverride = false;
		MjXmlUtils::ReadAttrInt(Node, TEXT("adr"), UserAdr, bAdrOverride);
	}
}

void UMjSensor::Bind(mjModel* Model, mjData* Data, const FString& Prefix)
{
	Super::Bind(Model, Data, Prefix);
	BindAndCacheView(m_SensorView, Prefix);
}

// Apply the MuJoCo → UE coordinate transform appropriate for each sensor type.
// The coordinate/unit family (EMjSensorValueKind) comes from the descriptor
// table; the rules are:
//   Position outputs (meters):    scale ×100, negate Y
//   Direction vector outputs:     negate Y only
//   3-D vector quantities (vel/acc/force/torque/angular): negate Y only
//   Quaternion outputs (w,x,y,z): reorder to UE (x,y,z,w) with handedness fix
//   GeomFromTo (two positions):   scale ×100, negate Y on each
//   Scalar outputs:               no transform
static void TransformSensorReading(TArray<float>& R, EMjSensorType Type)
{
	if (R.Num() == 0)
		return;

	switch (MjSensorTypeInfoFor(Type).ValueKind)
	{
		case EMjSensorValueKind::Position:
			if (R.Num() >= 3)
			{
				R[0] *= 100.0f;
				R[1] *= -100.0f;
				R[2] *= 100.0f;
			}
			break;

		case EMjSensorValueKind::Direction:
		case EMjSensorValueKind::Vector3:
			if (R.Num() >= 3)
				R[1] = -R[1];
			break;

		// Quaternion (w,x,y,z) → UE (X,Y,Z,W): X=-mjX, Y=mjY, Z=-mjZ, W=mjW.
		case EMjSensorValueKind::Quaternion:
			if (R.Num() >= 4)
			{
				const float mj_w = R[0], mj_x = R[1], mj_y = R[2], mj_z = R[3];
				R[0] = -mj_x;
				R[1] = mj_y;
				R[2] = -mj_z;
				R[3] = mj_w;
			}
			break;

		case EMjSensorValueKind::GeomFromTo:
			if (R.Num() >= 6)
			{
				R[0] *= 100.0f;
				R[1] *= -100.0f;
				R[2] *= 100.0f;
				R[3] *= 100.0f;
				R[4] *= -100.0f;
				R[5] *= 100.0f;
			}
			break;

		case EMjSensorValueKind::Scalar:
			break;
	}
}

TArray<float> UMjSensor::GetReading() const
{
	TArray<float> Result;
	if (m_SensorView.id != -1 && m_SensorView.sensordata)
	{
		for (int i = 0; i < m_SensorView.sensor_dim; ++i)
			Result.Add((float)m_SensorView.sensordata[i]);
		TransformSensorReading(Result, Type);
	}
	return Result;
}

float UMjSensor::GetScalarReading() const
{
	if (m_SensorView.id != -1 && m_SensorView.sensordata && m_SensorView.sensor_dim > 0)
	{
		return (float)m_SensorView.sensordata[0];
	}
	return 0.0f;
}

int UMjSensor::GetDimension() const
{
	if (m_SensorView.id != -1)
		return m_SensorView.sensor_dim;
	return 0;
}

FString UMjSensor::GetMjName() const
{
	if (m_SensorView.id < 0 || !m_SensorView.name)
		return FString();
	return MjUtils::MjToString(m_SensorView.name);
}

void UMjSensor::RegisterToSpec(FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody)
{
	mjsDefault* effectiveDefault = ResolveDefault(Wrapper.Spec, MjClassName);

	mjsSensor* sensor = mjs_addSensor(Wrapper.Spec);
	m_SpecElement = sensor->element;
	SetSpecElementName(Wrapper, sensor->element, mjOBJ_SENSOR);

	ExportTo(sensor, effectiveDefault);
}

void UMjSensor::DescribeState(FMjArticulationState& Out) const
{
	const SensorView& V = m_SensorView;
	if (V.id < 0 || !V.sensordata || V.sensor_dim <= 0)
		return;

	// The IR carries raw MuJoCo SI values (MuJoCo frame, double precision), like
	// joints and bodies do. The MuJoCo -> UE coordinate/unit fixup lives on the
	// display-facing accessor GetReading(), not on the serialization path.
	FMjSensorState& S = Out.Sensors.AddDefaulted_GetRef();
	S.Name = FMjCanonicalName::PartSegment(Cast<AMjArticulation>(GetOwner()), GetMjName());
	S.Semantic = MjSensorTypeInfoFor(Type).Semantic;
	S.Values.SetNumUninitialized(V.sensor_dim);
	for (int32 i = 0; i < V.sensor_dim; ++i)
		S.Values[i] = V.sensordata[i];
}

#if WITH_EDITOR
namespace
{
UClass* GetClassForObjType(EMjObjType ObjType)
{
	switch (ObjType)
	{
		case EMjObjType::Body:
			return UMjBody::StaticClass();
		case EMjObjType::Joint:
			return UMjJoint::StaticClass();
		case EMjObjType::Geom:
			return UMjGeom::StaticClass();
		case EMjObjType::Site:
			return UMjSite::StaticClass();
		case EMjObjType::Tendon:
			return UMjTendon::StaticClass();
		case EMjObjType::Actuator:
			return UMjActuator::StaticClass();
		case EMjObjType::Sensor:
			return UMjSensor::StaticClass();
		default:
			return UMjComponent::StaticClass();
	}
}
} // namespace

TArray<FString> UMjSensor::GetTargetNameOptions() const
{
	return UMjComponent::GetSiblingComponentOptions(this, GetClassForObjType(ObjType));
}

TArray<FString> UMjSensor::GetReferenceNameOptions() const
{
	if (RefType == EMjObjType::Unknown)
		return {TEXT("")};
	return UMjComponent::GetSiblingComponentOptions(this, GetClassForObjType(RefType));
}

// --- CODEGEN_EDITOR_OPTIONS_START ---
TArray<FString> UMjSensor::GetDefaultClassOptions() const
{
	return UMjComponent::GetSiblingComponentOptions(this, UMjDefault::StaticClass(), true);
}
// --- CODEGEN_EDITOR_OPTIONS_END ---
#endif
