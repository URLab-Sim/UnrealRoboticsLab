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

static EMjObjType MjObjToEnum(int obj) {
    switch(obj) {
        case mjOBJ_BODY: return EMjObjType::Body;
        case mjOBJ_XBODY: return EMjObjType::XBody;
        case mjOBJ_JOINT: return EMjObjType::Joint;
        case mjOBJ_DOF: return EMjObjType::DoF;
        case mjOBJ_GEOM: return EMjObjType::Geom;
        case mjOBJ_SITE: return EMjObjType::Site;
        case mjOBJ_CAMERA: return EMjObjType::Camera;
        case mjOBJ_LIGHT: return EMjObjType::Light;
        case mjOBJ_MESH: return EMjObjType::Mesh;
        case mjOBJ_HFIELD: return EMjObjType::HField;
        case mjOBJ_TEXTURE: return EMjObjType::Texture;
        case mjOBJ_MATERIAL: return EMjObjType::Material;
        case mjOBJ_PAIR: return EMjObjType::Pair;
        case mjOBJ_EXCLUDE: return EMjObjType::Exclude;
        case mjOBJ_EQUALITY: return EMjObjType::Equality;
        case mjOBJ_TENDON: return EMjObjType::Tendon;
        case mjOBJ_ACTUATOR: return EMjObjType::Actuator;
        case mjOBJ_SENSOR: return EMjObjType::Sensor;
        case mjOBJ_NUMERIC: return EMjObjType::Numeric;
        case mjOBJ_TEXT: return EMjObjType::Text;
        case mjOBJ_TUPLE: return EMjObjType::Tuple;
        case mjOBJ_KEY: return EMjObjType::Key;
        case mjOBJ_PLUGIN: return EMjObjType::Plugin;
        default: return EMjObjType::Unknown;
    }
}

static int EnumToMjObj(EMjObjType Type) {
    switch(Type) {
        case EMjObjType::Body:      return mjOBJ_BODY;
        case EMjObjType::XBody:     return mjOBJ_XBODY;
        case EMjObjType::Joint:     return mjOBJ_JOINT;
        case EMjObjType::DoF:       return mjOBJ_DOF;
        case EMjObjType::Geom:      return mjOBJ_GEOM;
        case EMjObjType::Site:      return mjOBJ_SITE;
        case EMjObjType::Camera:    return mjOBJ_CAMERA;
        case EMjObjType::Light:     return mjOBJ_LIGHT;
        case EMjObjType::Mesh:      return mjOBJ_MESH;
        case EMjObjType::HField:    return mjOBJ_HFIELD;
        case EMjObjType::Texture:   return mjOBJ_TEXTURE;
        case EMjObjType::Material:  return mjOBJ_MATERIAL;
        case EMjObjType::Pair:      return mjOBJ_PAIR;
        case EMjObjType::Exclude:   return mjOBJ_EXCLUDE;
        case EMjObjType::Equality:  return mjOBJ_EQUALITY;
        case EMjObjType::Tendon:    return mjOBJ_TENDON;
        case EMjObjType::Actuator:  return mjOBJ_ACTUATOR;
        case EMjObjType::Sensor:    return mjOBJ_SENSOR;
        case EMjObjType::Numeric:   return mjOBJ_NUMERIC;
        case EMjObjType::Text:      return mjOBJ_TEXT;
        case EMjObjType::Tuple:     return mjOBJ_TUPLE;
        case EMjObjType::Key:       return mjOBJ_KEY;
        case EMjObjType::Plugin:    return mjOBJ_PLUGIN;
        default:                    return mjOBJ_UNKNOWN;
    }
}


void UMjSensor::ExportTo(mjsSensor* Element, mjsDefault* Default)
{
    if (!Element) return;

    MjSetString(Element->objname, TargetName);
    MjSetString(Element->refname, ReferenceName);

    // Most built-in sensor types have `dim` derived from the sensor type
    // during mj_compile (e.g. accelerometer dim=3). Writing 0 here zeros
    // out the spec's compiler-derived value. Only write when the user has
    // explicitly overridden it (custom user/plugin sensors, or sensors
    // where dim is genuinely user-controlled).
    if (Dim > 0) Element->dim = Dim;
    
    // UserAdr is read-only runtime data (mjModel::sensor_adr), not writable via mjsSensor.

    for(int i=0; i<IntParams.Num() && i<3; i++) Element->intprm[i] = IntParams[i];
    
    switch(Type)
    {
        // --- CODEGEN_SENSOR_TYPE_SWITCH_START ---
        case EMjSensorType::Touch: Element->type = mjSENS_TOUCH; Element->objtype = mjOBJ_SITE; break;
        case EMjSensorType::Accelerometer: Element->type = mjSENS_ACCELEROMETER; Element->objtype = mjOBJ_SITE; break;
        case EMjSensorType::Velocimeter: Element->type = mjSENS_VELOCIMETER; Element->objtype = mjOBJ_SITE; break;
        case EMjSensorType::Gyro: Element->type = mjSENS_GYRO; Element->objtype = mjOBJ_SITE; break;
        case EMjSensorType::Force: Element->type = mjSENS_FORCE; Element->objtype = mjOBJ_SITE; break;
        case EMjSensorType::Torque: Element->type = mjSENS_TORQUE; Element->objtype = mjOBJ_SITE; break;
        case EMjSensorType::Magnetometer: Element->type = mjSENS_MAGNETOMETER; Element->objtype = mjOBJ_SITE; break;
        case EMjSensorType::CamProjection: Element->type = mjSENS_CAMPROJECTION; Element->objtype = mjOBJ_SITE; Element->reftype = mjOBJ_CAMERA; break;
        case EMjSensorType::RangeFinder:
            Element->type = mjSENS_RANGEFINDER; Element->objtype = (ObjType == EMjObjType::Camera) ? mjOBJ_CAMERA : mjOBJ_SITE; break;
        case EMjSensorType::JointPos: Element->type = mjSENS_JOINTPOS; Element->objtype = mjOBJ_JOINT; break;
        case EMjSensorType::JointVel: Element->type = mjSENS_JOINTVEL; Element->objtype = mjOBJ_JOINT; break;
        case EMjSensorType::BallQuat: Element->type = mjSENS_BALLQUAT; Element->objtype = mjOBJ_JOINT; break;
        case EMjSensorType::BallAngVel: Element->type = mjSENS_BALLANGVEL; Element->objtype = mjOBJ_JOINT; break;
        case EMjSensorType::JointLimitPos: Element->type = mjSENS_JOINTLIMITPOS; Element->objtype = mjOBJ_JOINT; break;
        case EMjSensorType::JointLimitVel: Element->type = mjSENS_JOINTLIMITVEL; Element->objtype = mjOBJ_JOINT; break;
        case EMjSensorType::JointLimitFrc: Element->type = mjSENS_JOINTLIMITFRC; Element->objtype = mjOBJ_JOINT; break;
        case EMjSensorType::TendonPos: Element->type = mjSENS_TENDONPOS; Element->objtype = mjOBJ_TENDON; break;
        case EMjSensorType::TendonVel: Element->type = mjSENS_TENDONVEL; Element->objtype = mjOBJ_TENDON; break;
        case EMjSensorType::TendonLimitPos: Element->type = mjSENS_TENDONLIMITPOS; Element->objtype = mjOBJ_TENDON; break;
        case EMjSensorType::TendonLimitVel: Element->type = mjSENS_TENDONLIMITVEL; Element->objtype = mjOBJ_TENDON; break;
        case EMjSensorType::TendonLimitFrc: Element->type = mjSENS_TENDONLIMITFRC; Element->objtype = mjOBJ_TENDON; break;
        case EMjSensorType::ActuatorPos: Element->type = mjSENS_ACTUATORPOS; Element->objtype = mjOBJ_ACTUATOR; break;
        case EMjSensorType::ActuatorVel: Element->type = mjSENS_ACTUATORVEL; Element->objtype = mjOBJ_ACTUATOR; break;
        case EMjSensorType::ActuatorFrc: Element->type = mjSENS_ACTUATORFRC; Element->objtype = mjOBJ_ACTUATOR; break;
        case EMjSensorType::JointActFrc: Element->type = mjSENS_JOINTACTFRC; Element->objtype = mjOBJ_JOINT; break;
        case EMjSensorType::TendonActFrc: Element->type = mjSENS_TENDONACTFRC; Element->objtype = mjOBJ_TENDON; break;
        case EMjSensorType::FramePos: Element->type = mjSENS_FRAMEPOS; break;
        case EMjSensorType::FrameQuat: Element->type = mjSENS_FRAMEQUAT; break;
        case EMjSensorType::FrameXAxis: Element->type = mjSENS_FRAMEXAXIS; break;
        case EMjSensorType::FrameYAxis: Element->type = mjSENS_FRAMEYAXIS; break;
        case EMjSensorType::FrameZAxis: Element->type = mjSENS_FRAMEZAXIS; break;
        case EMjSensorType::FrameLinVel: Element->type = mjSENS_FRAMELINVEL; break;
        case EMjSensorType::FrameAngVel: Element->type = mjSENS_FRAMEANGVEL; break;
        case EMjSensorType::FrameLinAcc: Element->type = mjSENS_FRAMELINACC; break;
        case EMjSensorType::FrameAngAcc: Element->type = mjSENS_FRAMEANGACC; break;
        case EMjSensorType::SubtreeCom: Element->type = mjSENS_SUBTREECOM; Element->objtype = mjOBJ_BODY; break;
        case EMjSensorType::SubtreeLinVel: Element->type = mjSENS_SUBTREELINVEL; Element->objtype = mjOBJ_BODY; break;
        case EMjSensorType::SubtreeAngMom: Element->type = mjSENS_SUBTREEANGMOM; Element->objtype = mjOBJ_BODY; break;
        case EMjSensorType::InsideSite: Element->type = mjSENS_INSIDESITE; Element->reftype = mjOBJ_SITE; break;
        case EMjSensorType::GeomDist: Element->type = mjSENS_GEOMDIST; break;
        case EMjSensorType::GeomNormal: Element->type = mjSENS_GEOMNORMAL; break;
        case EMjSensorType::GeomFromTo: Element->type = mjSENS_GEOMFROMTO; break;
        case EMjSensorType::Contact: Element->type = mjSENS_CONTACT; break;
        case EMjSensorType::EPotential: Element->type = mjSENS_E_POTENTIAL; Element->objtype = mjOBJ_UNKNOWN; break;
        case EMjSensorType::EKinetic: Element->type = mjSENS_E_KINETIC; Element->objtype = mjOBJ_UNKNOWN; break;
        case EMjSensorType::Clock: Element->type = mjSENS_CLOCK; Element->objtype = mjOBJ_UNKNOWN; break;
        case EMjSensorType::Tactile: Element->type = mjSENS_TACTILE; Element->objtype = mjOBJ_MESH; Element->reftype = mjOBJ_GEOM; break;
        case EMjSensorType::User: Element->type = mjSENS_USER; break;
        case EMjSensorType::Plugin: Element->type = mjSENS_PLUGIN; break;
        default: Element->type = mjSENS_ACCELEROMETER; Element->objtype = mjOBJ_SITE; break;
        // --- CODEGEN_SENSOR_TYPE_SWITCH_END ---
    }

    // Variable objtype / reftype handling. The codegen switch above sets
    // STATIC objtype/reftype literals when sensor_per_type carries a
    // mjOBJ_X value. For sensors whose objtype/reftype is "from_xml" or
    // "computed" in MuJoCo, the UE side reads ObjType / RefType properties
    // and translates them here AFTER the switch fires.
    if (Element->type >= mjSENS_FRAMEPOS && Element->type <= mjSENS_FRAMEANGACC)
    {
        Element->objtype = (mjtObj)EnumToMjObj(ObjType);
        if (RefType != EMjObjType::Unknown) Element->reftype = (mjtObj)EnumToMjObj(RefType);
    }
    else if (Type == EMjSensorType::GeomDist || Type == EMjSensorType::GeomNormal || Type == EMjSensorType::GeomFromTo || Type == EMjSensorType::Contact || Type == EMjSensorType::Plugin)
    {
        Element->objtype = (mjtObj)EnumToMjObj(ObjType);
        Element->reftype = (mjtObj)EnumToMjObj(RefType);
    }
    else if (Type == EMjSensorType::User || Type == EMjSensorType::InsideSite)
    {
        // InsideSite's reftype = mjOBJ_SITE is already set by the codegen
        // switch above; here we add the UE-driven objtype.
        Element->objtype = (mjtObj)EnumToMjObj(ObjType);
    }

        // --- CODEGEN_EXPORT_START ---
    if (bOverride_nsample) Element->nsample = nsample;
    if (bOverride_interp) Element->interp = interp;
    if (bOverride_delay) Element->delay = delay;
    if (bOverride_interval) { for (int32 i = 0; i < FMath::Min(interval.Num(), 2); ++i) Element->interval[i] = interval[i]; }
    if (bOverride_cutoff) Element->cutoff = cutoff;
    if (bOverride_noise) Element->noise = noise;
    MjSetString(Element->objname, TargetName);
    MjSetString(Element->refname, ReferenceName);
        // --- CODEGEN_EXPORT_END ---
}

void UMjSensor::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
    if (!Node) return;

        // --- CODEGEN_IMPORT_START ---
    MjXmlUtils::ReadAttrString(Node, TEXT("class"), MjClassName);
    { // xml_enum: objtype -> EMjObjType
        FString S = Node->GetAttribute(TEXT("objtype"));
        S = S.ToLower();
        if      (S == TEXT("body")) ObjType = EMjObjType::Body;
        else if (S == TEXT("xbody")) ObjType = EMjObjType::XBody;
        else if (S == TEXT("joint")) ObjType = EMjObjType::Joint;
        else if (S == TEXT("dof")) ObjType = EMjObjType::DoF;
        else if (S == TEXT("geom")) ObjType = EMjObjType::Geom;
        else if (S == TEXT("site")) ObjType = EMjObjType::Site;
        else if (S == TEXT("camera")) ObjType = EMjObjType::Camera;
        else if (S == TEXT("light")) ObjType = EMjObjType::Light;
        else if (S == TEXT("mesh")) ObjType = EMjObjType::Mesh;
        else if (S == TEXT("hfield")) ObjType = EMjObjType::HField;
        else if (S == TEXT("texture")) ObjType = EMjObjType::Texture;
        else if (S == TEXT("material")) ObjType = EMjObjType::Material;
        else if (S == TEXT("pair")) ObjType = EMjObjType::Pair;
        else if (S == TEXT("exclude")) ObjType = EMjObjType::Exclude;
        else if (S == TEXT("equality")) ObjType = EMjObjType::Equality;
        else if (S == TEXT("tendon")) ObjType = EMjObjType::Tendon;
        else if (S == TEXT("actuator")) ObjType = EMjObjType::Actuator;
    }
    { // xml_enum: reftype -> EMjObjType
        FString S = Node->GetAttribute(TEXT("reftype"));
        S = S.ToLower();
        if      (S == TEXT("body")) RefType = EMjObjType::Body;
        else if (S == TEXT("xbody")) RefType = EMjObjType::XBody;
        else if (S == TEXT("joint")) RefType = EMjObjType::Joint;
        else if (S == TEXT("geom")) RefType = EMjObjType::Geom;
        else if (S == TEXT("site")) RefType = EMjObjType::Site;
        else if (S == TEXT("camera")) RefType = EMjObjType::Camera;
    }
    MjXmlUtils::ReadAttrInt(Node, TEXT("nsample"), nsample, bOverride_nsample);
    MjXmlUtils::ReadAttrInt(Node, TEXT("interp"), interp, bOverride_interp);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("delay"), delay, bOverride_delay);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("interval"), interval, bOverride_interval);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("cutoff"), cutoff, bOverride_cutoff);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("noise"), noise, bOverride_noise);
    // target_collation: -> TargetName
    TargetName = Node->GetAttribute(TEXT("site"));
    if (TargetName.IsEmpty()) TargetName = Node->GetAttribute(TEXT("joint"));
    if (TargetName.IsEmpty()) TargetName = Node->GetAttribute(TEXT("tendon"));
    if (TargetName.IsEmpty()) TargetName = Node->GetAttribute(TEXT("actuator"));
    if (TargetName.IsEmpty()) TargetName = Node->GetAttribute(TEXT("body"));
    if (TargetName.IsEmpty()) TargetName = Node->GetAttribute(TEXT("geom"));
    if (TargetName.IsEmpty()) TargetName = Node->GetAttribute(TEXT("objname"));
    if (TargetName.IsEmpty()) TargetName = Node->GetAttribute(TEXT("camera"));
    if (TargetName.IsEmpty()) TargetName = Node->GetAttribute(TEXT("mesh"));
    // target_collation: -> ReferenceName
    ReferenceName = Node->GetAttribute(TEXT("refname"));
        // --- CODEGEN_IMPORT_END ---

    // Determine sensor type from the XML tag name
    static const TMap<FString, EMjSensorType> TagToType = {
        // --- CODEGEN_SENSOR_TAG_TO_TYPE_START ---
        {TEXT("touch"), EMjSensorType::Touch},
        {TEXT("accelerometer"), EMjSensorType::Accelerometer},
        {TEXT("velocimeter"), EMjSensorType::Velocimeter},
        {TEXT("gyro"), EMjSensorType::Gyro},
        {TEXT("force"), EMjSensorType::Force},
        {TEXT("torque"), EMjSensorType::Torque},
        {TEXT("magnetometer"), EMjSensorType::Magnetometer},
        {TEXT("camprojection"), EMjSensorType::CamProjection},
        {TEXT("rangefinder"), EMjSensorType::RangeFinder},
        {TEXT("jointpos"), EMjSensorType::JointPos},
        {TEXT("jointvel"), EMjSensorType::JointVel},
        {TEXT("ballquat"), EMjSensorType::BallQuat},
        {TEXT("ballangvel"), EMjSensorType::BallAngVel},
        {TEXT("jointlimitpos"), EMjSensorType::JointLimitPos},
        {TEXT("jointlimitvel"), EMjSensorType::JointLimitVel},
        {TEXT("jointlimitfrc"), EMjSensorType::JointLimitFrc},
        {TEXT("tendonpos"), EMjSensorType::TendonPos},
        {TEXT("tendonvel"), EMjSensorType::TendonVel},
        {TEXT("tendonlimitpos"), EMjSensorType::TendonLimitPos},
        {TEXT("tendonlimitvel"), EMjSensorType::TendonLimitVel},
        {TEXT("tendonlimitfrc"), EMjSensorType::TendonLimitFrc},
        {TEXT("actuatorpos"), EMjSensorType::ActuatorPos},
        {TEXT("actuatorvel"), EMjSensorType::ActuatorVel},
        {TEXT("actuatorfrc"), EMjSensorType::ActuatorFrc},
        {TEXT("jointactuatorfrc"), EMjSensorType::JointActFrc},
        {TEXT("tendonactuatorfrc"), EMjSensorType::TendonActFrc},
        {TEXT("framepos"), EMjSensorType::FramePos},
        {TEXT("framequat"), EMjSensorType::FrameQuat},
        {TEXT("framexaxis"), EMjSensorType::FrameXAxis},
        {TEXT("frameyaxis"), EMjSensorType::FrameYAxis},
        {TEXT("framezaxis"), EMjSensorType::FrameZAxis},
        {TEXT("framelinvel"), EMjSensorType::FrameLinVel},
        {TEXT("frameangvel"), EMjSensorType::FrameAngVel},
        {TEXT("framelinacc"), EMjSensorType::FrameLinAcc},
        {TEXT("frameangacc"), EMjSensorType::FrameAngAcc},
        {TEXT("subtreecom"), EMjSensorType::SubtreeCom},
        {TEXT("subtreelinvel"), EMjSensorType::SubtreeLinVel},
        {TEXT("subtreeangmom"), EMjSensorType::SubtreeAngMom},
        {TEXT("insidesite"), EMjSensorType::InsideSite},
        {TEXT("distance"), EMjSensorType::GeomDist},
        {TEXT("normal"), EMjSensorType::GeomNormal},
        {TEXT("fromto"), EMjSensorType::GeomFromTo},
        {TEXT("contact"), EMjSensorType::Contact},
        {TEXT("e_potential"), EMjSensorType::EPotential},
        {TEXT("e_kinetic"), EMjSensorType::EKinetic},
        {TEXT("clock"), EMjSensorType::Clock},
        {TEXT("tactile"), EMjSensorType::Tactile},
        {TEXT("user"), EMjSensorType::User},
        {TEXT("plugin"), EMjSensorType::Plugin},
        // --- CODEGEN_SENSOR_TAG_TO_TYPE_END ---
    };
    const FString Tag = Node->GetTag().ToLower();
    const EMjSensorType* Found = TagToType.Find(Tag);
    if (Found) Type = *Found;


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
        if (ParsedDim > 0) Dim = ParsedDim;
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
// Rules:
//   Position outputs (meters):    scale ×100, negate Y
//   Direction vector outputs:     negate Y only
//   3-D vector quantities (vel/acc/force/torque/angular): negate Y only
//   Quaternion outputs (w,x,y,z): reorder to UE (x,y,z,w) with handedness fix
//   Scalar outputs:               no transform
static void TransformSensorReading(TArray<float>& R, EMjSensorType Type)
{
    if (R.Num() == 0) return;

    switch (Type)
    {
    // --- Position outputs (scale ×100 cm, negate Y) ---
    case EMjSensorType::FramePos:
    case EMjSensorType::SubtreeCom:
        if (R.Num() >= 3)
        {
            R[0] *=  100.0f;
            R[1] *= -100.0f;
            R[2] *=  100.0f;
        }
        break;

    // --- Direction vectors (negate Y only) ---
    case EMjSensorType::FrameXAxis:
    case EMjSensorType::FrameYAxis:
    case EMjSensorType::FrameZAxis:
    case EMjSensorType::GeomNormal:
        if (R.Num() >= 3) R[1] = -R[1];
        break;

    // --- 3-D vector quantities: velocity, acceleration, force, torque, angular (negate Y only) ---
    case EMjSensorType::Accelerometer:
    case EMjSensorType::Velocimeter:
    case EMjSensorType::Gyro:
    case EMjSensorType::Force:
    case EMjSensorType::Torque:
    case EMjSensorType::Magnetometer:
    case EMjSensorType::BallAngVel:
    case EMjSensorType::FrameLinVel:
    case EMjSensorType::FrameAngVel:
    case EMjSensorType::FrameLinAcc:
    case EMjSensorType::FrameAngAcc:
    case EMjSensorType::SubtreeLinVel:
    case EMjSensorType::SubtreeAngMom:
        if (R.Num() >= 3) R[1] = -R[1];
        break;

    // --- Quaternion (w,x,y,z) → UE (X,Y,Z,W) with handedness fix: X=-mjX, Y=mjY, Z=-mjZ, W=mjW ---
    case EMjSensorType::BallQuat:
    case EMjSensorType::FrameQuat:
        if (R.Num() >= 4)
        {
            const float mj_w = R[0], mj_x = R[1], mj_y = R[2], mj_z = R[3];
            R[0] = -mj_x;
            R[1] =  mj_y;
            R[2] = -mj_z;
            R[3] =  mj_w;
        }
        break;

    // --- GeomFromTo: two 3D positions concatenated (scale ×100, negate Y each) ---
    case EMjSensorType::GeomFromTo:
        if (R.Num() >= 6)
        {
            R[0] *= 100.0f; R[1] *= -100.0f; R[2] *= 100.0f;
            R[3] *= 100.0f; R[4] *= -100.0f; R[5] *= 100.0f;
        }
        break;

    // --- Scalars and types with no coordinate meaning: no transform ---
    default:
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
    if (m_SensorView.id < 0 || !m_SensorView.name) return FString();
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

void UMjSensor::BuildBinaryPayload(FBufferArchive& OutBuffer) const
{
    int32 SensorID = m_ID;
    OutBuffer << SensorID;
    
    int32 NumElements = GetDimension();
    OutBuffer << NumElements;

    if (m_SensorView.id != -1 && m_SensorView.sensordata && NumElements > 0)
    {
        for (int i = 0; i < NumElements; ++i)
        {
            float Val = (float)m_SensorView.sensordata[i];
            OutBuffer << Val;
        }
    }
}

FString UMjSensor::GetTelemetryTopicName() const
{
    return FString::Printf(TEXT("sensor/%s"), *GetName());
}

#if WITH_EDITOR
namespace
{
    UClass* GetClassForObjType(EMjObjType ObjType)
    {
        switch (ObjType)
        {
            case EMjObjType::Body:    return UMjBody::StaticClass();
            case EMjObjType::Joint:   return UMjJoint::StaticClass();
            case EMjObjType::Geom:    return UMjGeom::StaticClass();
            case EMjObjType::Site:    return UMjSite::StaticClass();
            case EMjObjType::Tendon:  return UMjTendon::StaticClass();
            case EMjObjType::Actuator:return UMjActuator::StaticClass();
            case EMjObjType::Sensor:  return UMjSensor::StaticClass();
            default:                  return UMjComponent::StaticClass();
        }
    }
}

TArray<FString> UMjSensor::GetTargetNameOptions() const
{
    return UMjComponent::GetSiblingComponentOptions(this, GetClassForObjType(ObjType));
}

TArray<FString> UMjSensor::GetReferenceNameOptions() const
{
    if (RefType == EMjObjType::Unknown) return {TEXT("")};
    return UMjComponent::GetSiblingComponentOptions(this, GetClassForObjType(RefType));
}

// --- CODEGEN_EDITOR_OPTIONS_START ---
TArray<FString> UMjSensor::GetDefaultClassOptions() const { return UMjComponent::GetSiblingComponentOptions(this, UMjDefault::StaticClass(), true); }
// --- CODEGEN_EDITOR_OPTIONS_END ---
#endif
