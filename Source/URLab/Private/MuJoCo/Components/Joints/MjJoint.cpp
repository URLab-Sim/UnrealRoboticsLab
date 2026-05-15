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

#include "MuJoCo/Components/Joints/MjJoint.h"
#include "MuJoCo/Utils/MjXmlUtils.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"
#include "XmlNode.h"
#include "Utils/URLabLogging.h"
#include "MuJoCo/Core/Spec/MjSpecWrapper.h"
#include "MuJoCo/Components/Defaults/MjDefault.h"

UMjJoint::UMjJoint()
{
	PrimaryComponentTick.bCanEverTick = true;
    // MuJoCo builtin default axis is (0, 0, 1) — matches header initializer
    Type = EMjJointType::Hinge;
}


void UMjJoint::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
    if (!Node) return;

        // --- CODEGEN_IMPORT_START ---
    { // xml_enum: type -> EMjJointType
        FString S = Node->GetAttribute(TEXT("type"));
        S = S.ToLower();
        if      (S == TEXT("hinge")) Type = EMjJointType::Hinge;
        else if (S == TEXT("slide")) Type = EMjJointType::Slide;
        else if (S == TEXT("ball")) Type = EMjJointType::Ball;
        else if (S == TEXT("free")) Type = EMjJointType::Free;
        if (!S.IsEmpty()) bOverride_Type = true;
    }
    MjXmlUtils::ReadAttrInt(Node, TEXT("group"), group, bOverride_group);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("springdamper"), springdamper, bOverride_springdamper);
    MjXmlUtils::ReadAttrBool(Node, TEXT("limited"), limited, bOverride_limited);
    MjXmlUtils::ReadAttrBool(Node, TEXT("actuatorfrclimited"), actuatorfrclimited, bOverride_actuatorfrclimited);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("solreflimit"), solreflimit, bOverride_solreflimit);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("solimplimit"), solimplimit, bOverride_solimplimit);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("solreffriction"), solreffriction, bOverride_solreffriction);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("solimpfriction"), solimpfriction, bOverride_solimpfriction);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("stiffness"), stiffness, bOverride_stiffness);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("range"), range, bOverride_range);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("actuatorfrcrange"), actuatorfrcrange, bOverride_actuatorfrcrange);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("actuatorgravcomp"), actuatorgravcomp, bOverride_actuatorgravcomp);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("margin"), margin, bOverride_margin);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("ref"), ref, bOverride_ref);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("springref"), springref, bOverride_springref);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("armature"), armature, bOverride_armature);
    MjXmlUtils::ReadAttrFloatArray(Node, TEXT("damping"), damping, bOverride_damping);
    MjXmlUtils::ReadAttrFloat(Node, TEXT("frictionloss"), frictionloss, bOverride_frictionloss);
    if (bOverride_range)
    {
        if      (Type == EMjJointType::Slide) { for (float& V : range) { V *= 100.0f; } }
        else if ((Type == EMjJointType::Hinge) || (Type == EMjJointType::Ball)) { if (!CompilerSettings.bAngleInDegrees) { for (float& V : range) { V = FMath::RadiansToDegrees(V); } } }
    }
    if (bOverride_ref)
    {
        if      (Type == EMjJointType::Slide) { ref *= 100.0f; }
        else if ((Type == EMjJointType::Hinge) || (Type == EMjJointType::Ball)) { if (!CompilerSettings.bAngleInDegrees) { ref = FMath::RadiansToDegrees(ref); } }
    }
    if (bOverride_springref)
    {
        if      (Type == EMjJointType::Slide) { springref *= 100.0f; }
        else if ((Type == EMjJointType::Hinge) || (Type == EMjJointType::Ball)) { if (!CompilerSettings.bAngleInDegrees) { springref = FMath::RadiansToDegrees(springref); } }
    }
    // --- CODEGEN_IMPORT_END ---

    // Class Name
    MjXmlUtils::ReadAttrString(Node, TEXT("class"), MjClassName);

    // Axis — convert from MuJoCo to UE direction-vector convention (negate Y, no scale)
    FString AxisStr;
    if (MjXmlUtils::ReadAttrString(Node, TEXT("axis"), AxisStr))
    {
        bOverride_Axis = true;
        FVector RawAxis = MjXmlUtils::ParseVector(AxisStr);
        Axis = FVector(RawAxis.X, -RawAxis.Y, RawAxis.Z);
    }


    FString PosStr;
    if (MjXmlUtils::ReadAttrString(Node, TEXT("pos"), PosStr))
    {
        FVector RawPos = MjXmlUtils::ParseVector(PosStr);
        double p[3] = { (double)RawPos.X, (double)RawPos.Y, (double)RawPos.Z };
        SetRelativeLocation(MjUtils::MjToUEPosition(p));
    }

    // Note: autolimits (limited inferred from range) is handled by the MuJoCo spec
    // compiler (mjLIMITED_AUTO default). We do not replicate that logic here.

    UE_LOG(LogURLabImport, Log, TEXT("[MjJoint XML Import] '%s' (MjName: '%s') -> Type: %s (Overridden: %s), stiffness: %s (%f)"),
        *GetName(), *MjName,
        *StaticEnum<EMjJointType>()->GetNameStringByValue((int64)Type), bOverride_Type ? TEXT("True") : TEXT("False"),
        bOverride_stiffness ? TEXT("Set") : TEXT("Inherited"), stiffness.Num() > 0 ? stiffness[0] : 0.0f);
}

void UMjJoint::ExportTo(mjsJoint* Element, mjsDefault* Default)
{
    if (!Element) return;

    FVector _pos = GetRelativeLocation();
    if (!_pos.IsZero()) MjUtils::UEToMjPosition(_pos, Element->pos);

    if (bOverride_Axis)
    {
        // Convert UE direction vector back to MuJoCo convention (negate Y, no scale)
        Element->axis[0] = Axis.X;
        Element->axis[1] = -Axis.Y;
        Element->axis[2] = Axis.Z;
    }

    // Slide joints: convert ref from cm (UE) back to meters (MuJoCo)

        // --- CODEGEN_EXPORT_START ---
    if (bOverride_Type)
    {
        switch (Type)
        {
            case EMjJointType::Hinge: Element->type = (mjtJoint)mjJNT_HINGE; break;
            case EMjJointType::Slide: Element->type = (mjtJoint)mjJNT_SLIDE; break;
            case EMjJointType::Ball: Element->type = (mjtJoint)mjJNT_BALL; break;
            case EMjJointType::Free: Element->type = (mjtJoint)mjJNT_FREE; break;
            default: break;
        }
    }
    if (bOverride_group) Element->group = group;
    if (bOverride_springdamper) { for (int32 i = 0; i < springdamper.Num(); ++i) Element->springdamper[i] = springdamper[i]; }
    if (bOverride_limited) Element->limited = limited ? 1 : 0;
    if (bOverride_actuatorfrclimited) Element->actfrclimited = actuatorfrclimited ? 1 : 0;
    if (bOverride_solreflimit) { for (int32 i = 0; i < solreflimit.Num(); ++i) Element->solref_limit[i] = solreflimit[i]; }
    if (bOverride_solimplimit) { for (int32 i = 0; i < solimplimit.Num(); ++i) Element->solimp_limit[i] = solimplimit[i]; }
    if (bOverride_solreffriction) { for (int32 i = 0; i < solreffriction.Num(); ++i) Element->solref_friction[i] = solreffriction[i]; }
    if (bOverride_solimpfriction) { for (int32 i = 0; i < solimpfriction.Num(); ++i) Element->solimp_friction[i] = solimpfriction[i]; }
    if (bOverride_stiffness) { for (int32 i = 0; i < stiffness.Num(); ++i) Element->stiffness[i] = stiffness[i]; }
    if (bOverride_actuatorfrcrange) { for (int32 i = 0; i < actuatorfrcrange.Num(); ++i) Element->actfrcrange[i] = actuatorfrcrange[i]; }
    if (bOverride_actuatorgravcomp) Element->actgravcomp = actuatorgravcomp;
    if (bOverride_margin) Element->margin = margin;
    if (bOverride_armature) Element->armature = armature;
    if (bOverride_damping) { for (int32 i = 0; i < damping.Num(); ++i) Element->damping[i] = damping[i]; }
    if (bOverride_frictionloss) Element->frictionloss = frictionloss;
    if (bOverride_range)
    {
        for (int32 i = 0; i < range.Num(); ++i)
        {
            float V = range[i];
            if      (Type == EMjJointType::Slide) { V *= 0.01f; }
            else if ((Type == EMjJointType::Hinge) || (Type == EMjJointType::Ball)) { V = FMath::DegreesToRadians(V); }
            Element->range[i] = (mjtNum)V;
        }
    }
    if (bOverride_ref)
    {
        float V = ref;
        if      (Type == EMjJointType::Slide) { V *= 0.01f; }
        else if ((Type == EMjJointType::Hinge) || (Type == EMjJointType::Ball)) { V = FMath::DegreesToRadians(V); }
        Element->ref = (mjtNum)V;
    }
    if (bOverride_springref)
    {
        float V = springref;
        if      (Type == EMjJointType::Slide) { V *= 0.01f; }
        else if ((Type == EMjJointType::Hinge) || (Type == EMjJointType::Ball)) { V = FMath::DegreesToRadians(V); }
        Element->springref = (mjtNum)V;
    }
    // --- CODEGEN_EXPORT_END ---
}

void UMjJoint::Bind(mjModel* Model, mjData* Data, const FString& Prefix)
{
    Super::Bind(Model, Data, Prefix);
    m_JointView = BindToView<JointView>(Prefix);
    if (m_JointView.id != -1) m_ID = m_JointView.id;
    else UE_LOG(LogURLabBind, Warning, TEXT("Failed to bind Joint %s"), *GetName());
}

float UMjJoint::GetPosition() const
{
    if (m_JointView._d && m_JointView.qpos)
    {
        return (float)m_JointView.qpos[0];
    }
    return 0.0f;
}

void UMjJoint::SetPosition(float NewPosition)
{
    if (m_JointView._d && m_JointView.qpos)
    {
        m_JointView.qpos[0] = (mjtNum)NewPosition;
    }
}

float UMjJoint::GetVelocity() const
{
    if (m_JointView._d && m_JointView.qvel)
    {
        return (float)m_JointView.qvel[0];
    }
    return 0.0f;
}

void UMjJoint::SetVelocity(float NewVelocity)
{
    if (m_JointView._d && m_JointView.qvel)
    {
        m_JointView.qvel[0] = (mjtNum)NewVelocity;
    }
}

float UMjJoint::GetAcceleration() const
{
    if (m_JointView._d && m_JointView.qacc)
    {
        return (float)m_JointView.qacc[0];
    }
    return 0.0f;
}

FVector2D UMjJoint::GetJointRange() const
{
    if (m_JointView.id < 0 || !m_JointView.range) return FVector2D::ZeroVector;
    return FVector2D((float)m_JointView.range[0], (float)m_JointView.range[1]);
}

FVector UMjJoint::GetWorldAnchor() const
{
    if (m_JointView._d && m_JointView.xanchor)
    {
        return MjUtils::MjToUEPosition(m_JointView.xanchor);
    }
    return FVector::ZeroVector;
}

FVector UMjJoint::GetWorldAxis() const
{
    if (m_JointView._d && m_JointView.xaxis)
    {
        return FVector(
            (float)m_JointView.xaxis[0],
            -(float)m_JointView.xaxis[1],
            (float)m_JointView.xaxis[2]);
    }
    return FVector::ForwardVector;
}

FString UMjJoint::GetMjName() const
{
    if (m_JointView.id < 0 || !m_JointView.name) return FString();
    return MjUtils::MjToString(m_JointView.name);
}

FMuJoCoJointState UMjJoint::GetJointState() const
{
    FMuJoCoJointState State;
    if (m_JointView._d)
    {
        if (m_JointView.qpos) State.Position = (float)m_JointView.qpos[0];
        if (m_JointView.qvel) State.Velocity = (float)m_JointView.qvel[0];
        if (m_JointView.qacc) State.Acceleration = (float)m_JointView.qacc[0];
    }
    return State;
}

// --- Editor-time resolved accessors ---

EMjJointType UMjJoint::GetResolvedType() const
{
    // Runtime: use compiled model
    if (m_JointView.id >= 0)
    {
        switch (m_JointView.type)
        {
            case mjJNT_FREE:  return EMjJointType::Free;
            case mjJNT_BALL:  return EMjJointType::Ball;
            case mjJNT_SLIDE: return EMjJointType::Slide;
            case mjJNT_HINGE: return EMjJointType::Hinge;
            default:          return EMjJointType::Hinge;
        }
    }

    // Editor: local override wins
    if (bOverride_Type) return Type;

    // Walk default chain
    if (UMjDefault* Def = FindEditorDefault())
    {
        TSet<FString> Visited;
        UMjDefault* Cur = Def;
        while (Cur && !Visited.Contains(Cur->ClassName))
        {
            Visited.Add(Cur->ClassName);
            if (UMjJoint* J = Cur->FindChildOfType<UMjJoint>())
            {
                if (J->bOverride_Type) return J->Type;
            }
            Cur = Cur->ParentClassName.IsEmpty() ? nullptr
                : FindDefaultByClassName(GetOwner(), Cur->ParentClassName);
        }
    }

    // Fallback: use local Type value even without bOverride_Type.
    // This handles user-created joints where the type was set in the
    // details panel but the override toggle wasn't explicitly checked.
    // The default enum value is Hinge (MuJoCo builtin), so this is
    // safe — it returns the user's setting or the MuJoCo default.
    return Type;
}

FVector UMjJoint::GetResolvedAxis() const
{
    // Runtime: use compiled world-space axis
    if (m_JointView.id >= 0 && m_JointView.xaxis)
    {
        return FVector(
            (float)m_JointView.xaxis[0],
            -(float)m_JointView.xaxis[1],
            (float)m_JointView.xaxis[2]);
    }

    // Editor: local override wins
    if (bOverride_Axis) return Axis;

    // Walk default chain
    if (UMjDefault* Def = FindEditorDefault())
    {
        TSet<FString> Visited;
        UMjDefault* Cur = Def;
        while (Cur && !Visited.Contains(Cur->ClassName))
        {
            Visited.Add(Cur->ClassName);
            if (UMjJoint* J = Cur->FindChildOfType<UMjJoint>())
            {
                if (J->bOverride_Axis) return J->Axis;
            }
            Cur = Cur->ParentClassName.IsEmpty() ? nullptr
                : FindDefaultByClassName(GetOwner(), Cur->ParentClassName);
        }
    }

    return FVector(0, 0, 1); // MuJoCo builtin (Z-up)
}

FVector2D UMjJoint::GetResolvedRange() const
{
    // Returns the range in UE conventions: degrees for hinge/ball, cm for
    // slide. Runtime path reads from the compiled model (radians / metres)
    // and converts; editor path returns UPROPERTY values verbatim (already
    // in UE conventions thanks to the codegen unit_conversion rule).
    const EMjJointType T = (m_JointView.id >= 0) ? GetResolvedType() : Type;
    auto MjToUE = [T](float V) -> float
    {
        if (T == EMjJointType::Slide)  return V * 100.0f;           // m -> cm
        if (T == EMjJointType::Hinge || T == EMjJointType::Ball)
            return FMath::RadiansToDegrees(V);
        return V;
    };

    // Runtime: use compiled model (rad/m).
    if (m_JointView.id >= 0 && m_JointView.range)
    {
        return FVector2D(MjToUE((float)m_JointView.range[0]),
                         MjToUE((float)m_JointView.range[1]));
    }

    // Editor: local override wins. Already in UE units.
    if (bOverride_range && range.Num() >= 2)
    {
        return FVector2D(range[0], range[1]);
    }

    // Walk default chain
    if (UMjDefault* Def = FindEditorDefault())
    {
        TSet<FString> Visited;
        UMjDefault* Cur = Def;
        while (Cur && !Visited.Contains(Cur->ClassName))
        {
            Visited.Add(Cur->ClassName);
            if (UMjJoint* J = Cur->FindChildOfType<UMjJoint>())
            {
                if (J->bOverride_range && J->range.Num() >= 2)
                {
                    return FVector2D(J->range[0], J->range[1]);
                }
            }
            Cur = Cur->ParentClassName.IsEmpty() ? nullptr
                : FindDefaultByClassName(GetOwner(), Cur->ParentClassName);
        }
    }

    return FVector2D(0, 0); // MuJoCo builtin
}

bool UMjJoint::GetResolvedLimited() const
{
    // Runtime: use compiled model range
    if (m_JointView.id >= 0 && m_JointView.range)
    {
        // MuJoCo sets range to (0,0) when unlimited
        return m_JointView.range[0] != 0.0 || m_JointView.range[1] != 0.0;
    }

    // Editor: local override wins
    if (bOverride_limited) return limited;

    // Walk default chain
    if (UMjDefault* Def = FindEditorDefault())
    {
        TSet<FString> Visited;
        UMjDefault* Cur = Def;
        while (Cur && !Visited.Contains(Cur->ClassName))
        {
            Visited.Add(Cur->ClassName);
            if (UMjJoint* J = Cur->FindChildOfType<UMjJoint>())
            {
                if (J->bOverride_limited) return J->limited;
            }
            Cur = Cur->ParentClassName.IsEmpty() ? nullptr
                : FindDefaultByClassName(GetOwner(), Cur->ParentClassName);
        }
    }

    return false; // MuJoCo builtin
}

void UMjJoint::RegisterToSpec(FMujocoSpecWrapper& Wrapper, mjsBody* ParentBody)
{
    if (!ParentBody) return;

    mjsDefault* effectiveDefault = ResolveDefault(Wrapper.Spec, MjClassName);

    mjsJoint* Jnt = mjs_addJoint(ParentBody, effectiveDefault);
    m_SpecElement = Jnt->element;

    FString NameToRegister = MjName.IsEmpty() ? GetName() : MjName;
    mjs_setName(Jnt->element, TCHAR_TO_UTF8(*NameToRegister));

    ExportTo(Jnt, effectiveDefault);
}

void UMjJoint::BuildBinaryPayload(FBufferArchive& OutBuffer) const
{
    int32 JointID = m_ID;
    OutBuffer << JointID;

    float Pos = GetPosition();
    float Vel = GetVelocity();
    float Acc = GetAcceleration();

    OutBuffer << Pos;
    OutBuffer << Vel;
    OutBuffer << Acc;
}

FString UMjJoint::GetTelemetryTopicName() const
{
    return FString::Printf(TEXT("joint/%s"), *GetName());
}

#if WITH_EDITOR
TArray<FString> UMjJoint::GetDefaultClassOptions() const
{
    return GetSiblingComponentOptions(this, UMjDefault::StaticClass(), true);
}
#endif
