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

// MjImportTests.cpp
//
// Two tiers of import tests:
//
//   TIER 1 — FMjTestSession (pure MuJoCo, no UE pipeline)
//     These tests load inline MJCF XML directly via mj_parseXMLString and
//     inspect the resulting mjModel*.  They mirror the structure of MuJoCo's
//     own xml_native_reader_test.cc and serve as a reference baseline that
//     documents what MuJoCo expects from any given XML feature.
//
//   TIER 2 — FMjXmlImportSession (full URLab importer)
//     These tests pass the same XML through UMujocoGenerationAction and
//     verify that UE component properties were set correctly.  After calling
//     Compile() they also verify the compiled mjModel matches expectations.
//     A discrepancy between Tier 1 and Tier 2 always means a URLab bug.
//
//   ROUND-TRIP tests compare Tier 1 model counts with Tier 2 counts directly.
//   If ngeom/nbody/nsensor differ, the importer dropped or duplicated elements.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjTestHelpers.h"

// Component types for FindTemplate<>
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Components/Geometry/MjGeom.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "MuJoCo/Components/Sensors/MjSensor.h"
#include "MuJoCo/Components/Sensors/MjJointPosSensor.h"
#include "MuJoCo/Components/Sensors/MjJointVelSensor.h"
#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "MuJoCo/Components/Actuators/MjMotorActuator.h"
#include "MuJoCo/Components/Tendons/MjTendon.h"
#include "MuJoCo/Components/Physics/MjContactPair.h"
#include "MuJoCo/Components/Constraints/MjEquality.h"
#include "MuJoCo/Components/Keyframes/MjKeyframe.h"
#include "MuJoCo/Components/Sensors/MjCamera.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "mujoco/mjspec.h"

// =============================================================================
// TIER 1 — Pure MuJoCo baseline tests (FMjTestSession)
// Mirror MuJoCo's xml_native_reader_test.cc structure.
// =============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_MJ_BodyPos,
    "URLab.Import.MJ_BodyPos",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_MJ_BodyPos::RunTest(const FString&)
{
    FMjTestSession S;
    if (!S.CompileXml(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="b1" pos="1 2 3">
              <geom size=".1"/>
            </body>
          </worldbody>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    int bid = S.BodyId("b1");
    TestTrue(TEXT("b1 body id valid"), bid >= 0);
    TestNearlyEqual(TEXT("b1 x"), (float)S.m->body_pos[3*bid+0], 1.0f, 1e-4f);
    TestNearlyEqual(TEXT("b1 y"), (float)S.m->body_pos[3*bid+1], 2.0f, 1e-4f);
    TestNearlyEqual(TEXT("b1 z"), (float)S.m->body_pos[3*bid+2], 3.0f, 1e-4f);
    S.Cleanup();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_MJ_GeomSize,
    "URLab.Import.MJ_GeomSize",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_MJ_GeomSize::RunTest(const FString&)
{
    FMjTestSession S;
    if (!S.CompileXml(TEXT(R"(
        <mujoco>
          <worldbody>
            <body>
              <geom name="g1" type="sphere" size=".5"/>
              <geom name="g2" type="box" size="1 2 3"/>
            </body>
          </worldbody>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    int g1 = S.GeomId("g1"), g2 = S.GeomId("g2");
    TestTrue(TEXT("g1 valid"), g1 >= 0);
    TestTrue(TEXT("g2 valid"), g2 >= 0);
    TestNearlyEqual(TEXT("sphere radius"), (float)S.m->geom_size[3*g1], 0.5f, 1e-4f);
    TestNearlyEqual(TEXT("box x"),         (float)S.m->geom_size[3*g2+0], 1.0f, 1e-4f);
    TestNearlyEqual(TEXT("box y"),         (float)S.m->geom_size[3*g2+1], 2.0f, 1e-4f);
    TestNearlyEqual(TEXT("box z"),         (float)S.m->geom_size[3*g2+2], 3.0f, 1e-4f);
    S.Cleanup();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_MJ_JointRange,
    "URLab.Import.MJ_JointRange",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_MJ_JointRange::RunTest(const FString&)
{
    // MuJoCo defaults to angle="degree"; use explicit radian mode for numeric values.
    FMjTestSession S;
    if (!S.CompileXml(TEXT(R"(
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <body>
              <joint name="j1" type="hinge" range="-1.57 1.57" limited="true"/>
              <geom size=".1"/>
            </body>
          </worldbody>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    int jid = S.JointId("j1");
    TestTrue(TEXT("j1 valid"), jid >= 0);
    TestTrue(TEXT("j1 limited"),  S.m->jnt_limited[jid] != 0);
    TestNearlyEqual(TEXT("range lo"), (float)S.m->jnt_range[2*jid+0], -1.57f, 1e-3f);
    TestNearlyEqual(TEXT("range hi"), (float)S.m->jnt_range[2*jid+1],  1.57f, 1e-3f);
    S.Cleanup();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_MJ_DefaultClassOverride,
    "URLab.Import.MJ_DefaultClassOverride",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_MJ_DefaultClassOverride::RunTest(const FString&)
{
    // MuJoCo: explicit class= overrides parent childclass=
    // Default inheritance works natively because mjs_add*() receives the resolved mjsDefault*.
    FMjTestSession S;
    if (!S.CompileXml(TEXT(R"(
        <mujoco>
          <default>
            <default class="size2"><geom size="2"/></default>
            <default class="size3"><geom size="3"/></default>
          </default>
          <worldbody>
            <body childclass="size2">
              <geom name="g_inherit"/>
              <geom name="g_override" class="size3"/>
            </body>
          </worldbody>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    int g0 = S.GeomId("g_inherit"), g1 = S.GeomId("g_override");
    TestTrue(TEXT("g_inherit valid"), g0 >= 0);
    TestTrue(TEXT("g_override valid"), g1 >= 0);
    TestNearlyEqual(TEXT("inherited size2"),  (float)S.m->geom_size[3*g0], 2.0f, 1e-4f);
    TestNearlyEqual(TEXT("overridden size3"), (float)S.m->geom_size[3*g1], 3.0f, 1e-4f);
    S.Cleanup();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_MJ_FrameElement,
    "URLab.Import.MJ_FrameElement",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_MJ_FrameElement::RunTest(const FString&)
{
    // MuJoCo: <frame> applies a transform to nested elements, then is dissolved
    FMjTestSession S;
    if (!S.CompileXml(TEXT(R"(
        <mujoco>
          <worldbody>
            <frame pos="0 0 1">
              <geom name="g_in_frame" size=".1"/>
            </frame>
          </worldbody>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    int gid = S.GeomId("g_in_frame");
    TestTrue(TEXT("geom inside frame valid"), gid >= 0);
    TestNearlyEqual(TEXT("frame z offset applied"), (float)S.m->geom_pos[3*gid+2], 1.0f, 1e-4f);
    S.Cleanup();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_MJ_FrameChildclass,
    "URLab.Import.MJ_FrameChildclass",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_MJ_FrameChildclass::RunTest(const FString&)
{
    // <frame childclass="x"> propagates default to nested geoms
    FMjTestSession S;
    if (!S.CompileXml(TEXT(R"(
        <mujoco>
          <default>
            <default class="myclass"><geom size=".5"/></default>
          </default>
          <worldbody>
            <frame childclass="myclass">
              <geom name="g1"/>
              <geom name="g2" class="myclass"/>
            </frame>
          </worldbody>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    int g1 = S.GeomId("g1"), g2 = S.GeomId("g2");
    TestTrue(TEXT("g1 valid"), g1 >= 0);
    TestTrue(TEXT("g2 valid"), g2 >= 0);
    TestNearlyEqual(TEXT("g1 size from childclass"), (float)S.m->geom_size[3*g1], 0.5f, 1e-4f);
    TestNearlyEqual(TEXT("g2 size from class"),      (float)S.m->geom_size[3*g2], 0.5f, 1e-4f);
    S.Cleanup();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_MJ_TendonArmature,
    "URLab.Import.MJ_TendonArmature",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_MJ_TendonArmature::RunTest(const FString&)
{
    FMjTestSession S;
    if (!S.CompileXml(TEXT(R"(
        <mujoco>
          <worldbody>
            <site name="a"/>
            <body pos="1 0 0">
              <joint name="slide" type="slide"/>
              <geom size=".1"/>
              <site name="b"/>
            </body>
          </worldbody>
          <tendon>
            <spatial name="t_spatial" armature="1.5">
              <site site="a"/>
              <site site="b"/>
            </spatial>
            <fixed name="t_fixed" armature="2.5">
              <joint joint="slide" coef="1"/>
            </fixed>
          </tendon>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    TestEqual(TEXT("ntendon"), (int)S.m->ntendon, 2);
    TestNearlyEqual(TEXT("spatial armature"), (float)S.m->tendon_armature[0], 1.5f, 1e-4f);
    TestNearlyEqual(TEXT("fixed armature"),   (float)S.m->tendon_armature[1], 2.5f, 1e-4f);
    S.Cleanup();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_MJ_EqualityPolycoef,
    "URLab.Import.MJ_EqualityPolycoef",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_MJ_EqualityPolycoef::RunTest(const FString&)
{
    FMjTestSession S;
    if (!S.CompileXml(TEXT(R"(
        <mujoco>
          <worldbody>
            <body><joint name="j0"/><geom size="1"/></body>
            <body><joint name="j1"/><geom size="1"/></body>
          </worldbody>
          <equality>
            <joint joint1="j0" joint2="j1" polycoef="5 6 7 8 9"/>
          </equality>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    TestEqual(TEXT("neq"), (int)S.m->neq, 1);
    TestNearlyEqual(TEXT("coef0"), (float)S.m->eq_data[0], 5.0f, 1e-4f);
    TestNearlyEqual(TEXT("coef1"), (float)S.m->eq_data[1], 6.0f, 1e-4f);
    TestNearlyEqual(TEXT("coef4"), (float)S.m->eq_data[4], 9.0f, 1e-4f);
    S.Cleanup();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_MJ_OptionTimestepGravity,
    "URLab.Import.MJ_OptionTimestepGravity",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_MJ_OptionTimestepGravity::RunTest(const FString&)
{
    FMjTestSession S;
    if (!S.CompileXml(TEXT(R"(
        <mujoco>
          <option timestep="0.005" gravity="0 0 -9.81"/>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    TestNearlyEqual(TEXT("timestep"), (float)S.m->opt.timestep, 0.005f, 1e-6f);
    TestNearlyEqual(TEXT("gravity z"), (float)S.m->opt.gravity[2], -9.81f, 1e-3f);
    S.Cleanup();
    return true;
}

// =============================================================================
// TIER 2 — URLab importer tests (FMjXmlImportSession)
// =============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_URLab_BodyPos,
    "URLab.Import.URLab_BodyPos",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_URLab_BodyPos::RunTest(const FString&)
{
    // body pos="1 2 3" → UE RelativeLocation (100, -200, 300) cm
    // Coordinate rule: scale ×100, negate Y
    FMjXmlImportSession S;
    if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="b1" pos="1 2 3">
              <geom size=".1"/>
            </body>
          </worldbody>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    UMjBody* B = S.FindTemplate<UMjBody>(TEXT("b1"));
    if (!B) { AddError(TEXT("Body 'b1' not found in Blueprint")); S.Cleanup(); return false; }

    FVector Loc = B->GetRelativeLocation();
    TestNearlyEqual(TEXT("X=100 cm"),          (float)Loc.X, 100.0f,  1.0f);
    TestNearlyEqual(TEXT("Y=-200 cm (negated)"),(float)Loc.Y, -200.0f, 1.0f);
    TestNearlyEqual(TEXT("Z=300 cm"),           (float)Loc.Z, 300.0f,  1.0f);

    S.Cleanup();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_URLab_BodyIdentityQuat,
    "URLab.Import.URLab_BodyIdentityQuat",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_URLab_BodyIdentityQuat::RunTest(const FString&)
{
    // quat="1 0 0 0" is MuJoCo identity → should map to UE identity rotation
    FMjXmlImportSession S;
    if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="b1" quat="1 0 0 0">
              <geom size=".1"/>
            </body>
          </worldbody>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    UMjBody* B = S.FindTemplate<UMjBody>(TEXT("b1"));
    if (!B) { AddError(TEXT("Body 'b1' not found")); S.Cleanup(); return false; }

    FQuat Q = B->GetRelativeRotationCache().GetCachedQuat();
    TestTrue(TEXT("identity quat"), Q.Equals(FQuat::Identity, 0.01f));

    S.Cleanup();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_URLab_GeomFriction,
    "URLab.Import.URLab_GeomFriction",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_URLab_GeomFriction::RunTest(const FString&)
{
    FMjXmlImportSession S;
    if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body>
              <geom name="g1" size=".1" friction="0.8 0.1 0.01"/>
            </body>
          </worldbody>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    UMjGeom* G = S.FindTemplate<UMjGeom>(TEXT("g1"));
    if (!G) { AddError(TEXT("Geom 'g1' not found")); S.Cleanup(); return false; }

    TestNearlyEqual(TEXT("friction[0]"), G->friction[0], 0.8f,  1e-4f);
    TestNearlyEqual(TEXT("friction[1]"), G->friction[1], 0.1f,  1e-4f);
    TestNearlyEqual(TEXT("friction[2]"), G->friction[2], 0.01f, 1e-3f);

    S.Cleanup();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_URLab_JointRangeAndDamping,
    "URLab.Import.URLab_JointRangeAndDamping",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_URLab_JointRangeAndDamping::RunTest(const FString&)
{
    FMjXmlImportSession S;
    if (!S.Init(TEXT(R"(
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <body>
              <joint name="j1" type="hinge" range="-1.57 1.57" limited="true" damping="0.5"/>
              <geom size=".1"/>
            </body>
          </worldbody>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    UMjJoint* J = S.FindTemplate<UMjJoint>(TEXT("j1"));
    if (!J) { AddError(TEXT("Joint 'j1' not found")); S.Cleanup(); return false; }

    // UPROPERTY range is in UE degrees; XML had radians (compiler.angle="radian"),
    // converted on import. 1.57 rad ≈ 89.954 deg.
    const float Rad157Deg = 1.57f * 180.0f / PI;
    TestNearlyEqual(TEXT("range lo"), J->range[0], -Rad157Deg, 1e-2f);
    TestNearlyEqual(TEXT("range hi"), J->range[1],  Rad157Deg, 1e-2f);
    TestTrue(TEXT("damping has values"), J->damping.Num() > 0);
    if (J->damping.Num() > 0)
        TestNearlyEqual(TEXT("damping[0]"), J->damping[0], 0.5f, 1e-4f);

    S.Cleanup();
    return true;
}

// Regression: parser used to drop CompilerSettings when recursing into
// <default> blocks, so joints declared inside a default class always saw
// the hardcoded bAngleInDegrees=true fallback. For angle="radian" models
// (e.g. mujoco_menagerie's unitree_go1) the default joint's Range would
// then be re-scaled by π/180 in the wrong direction. Verifies the default
// joint template imports radians correctly. UPROPERTY storage is UE deg
// (codegen unit_conversion rad_to_deg_if_xml_radians), so ±0.86 rad
// imports as ±49.274 deg.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_URLab_DefaultClassJointRangeRadians,
    "URLab.Import.URLab_DefaultClassJointRangeRadians",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_URLab_DefaultClassJointRangeRadians::RunTest(const FString&)
{
    FMjXmlImportSession S;
    if (!S.Init(TEXT(R"(
        <mujoco>
          <compiler angle="radian" autolimits="true"/>
          <default>
            <default class="hip"><joint range="-0.86 0.86"/></default>
          </default>
          <worldbody>
            <body>
              <joint class="hip" name="j_hip"/>
              <geom size=".1"/>
            </body>
          </worldbody>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    UMjJoint* DefaultJoint = nullptr;
    if (S.Blueprint && S.Blueprint->SimpleConstructionScript)
    {
        for (USCS_Node* Node : S.Blueprint->SimpleConstructionScript->GetAllNodes())
        {
            UMjJoint* J = Cast<UMjJoint>(Node->ComponentTemplate);
            if (J && J->bIsDefault) { DefaultJoint = J; break; }
        }
    }
    if (!DefaultJoint) { AddError(TEXT("No default-class joint template found")); S.Cleanup(); return false; }

    if (DefaultJoint->range.Num() >= 2)
    {
        // XML had compiler.angle="radian" + range="-0.86 0.86"; UPROPERTY
        // is UE deg, so 0.86 rad -> ~49.274 deg.
        const float ExpectedDeg = 0.86f * 180.0f / PI;
        TestNearlyEqual(TEXT("default joint Range[0] (rad XML -> UE deg)"),
                        DefaultJoint->range[0], -ExpectedDeg, 1e-2f);
        TestNearlyEqual(TEXT("default joint Range[1] (rad XML -> UE deg)"),
                        DefaultJoint->range[1],  ExpectedDeg, 1e-2f);
    }
    else
    {
        AddError(TEXT("default joint Range has < 2 entries"));
    }

    S.Cleanup();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_URLab_SensorType,
    "URLab.Import.URLab_SensorType",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_URLab_SensorType::RunTest(const FString&)
{
    // Sensor XML tags must produce the correct concrete UE sensor subclass
    FMjXmlImportSession S;
    if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body>
              <joint name="j1" type="hinge"/>
              <geom size=".1"/>
            </body>
          </worldbody>
          <sensor>
            <jointpos name="s_jpos" joint="j1"/>
            <jointvel name="s_jvel" joint="j1"/>
          </sensor>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    UMjJointPosSensor* SJP = S.FindTemplate<UMjJointPosSensor>(TEXT("s_jpos"));
    UMjJointVelSensor* SJV = S.FindTemplate<UMjJointVelSensor>(TEXT("s_jvel"));

    TestNotNull(TEXT("jointpos concrete class"), SJP);
    TestNotNull(TEXT("jointvel concrete class"), SJV);

    if (SJP) TestEqual(TEXT("jointpos target joint"), SJP->TargetName, FString(TEXT("j1")));
    if (SJV) TestEqual(TEXT("jointvel target joint"), SJV->TargetName, FString(TEXT("j1")));

    S.Cleanup();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_URLab_OptionTimestep,
    "URLab.Import.URLab_OptionTimestep",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_URLab_OptionTimestep::RunTest(const FString&)
{
    // <option> attributes are parsed into AMjArticulation::SimOptions on the CDO
    FMjXmlImportSession S;
    if (!S.Init(TEXT(R"(
        <mujoco>
          <option timestep="0.005" gravity="0 0 -5"/>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    if (!S.Blueprint) { AddError(TEXT("No Blueprint")); S.Cleanup(); return false; }
    AMjArticulation* CDO = Cast<AMjArticulation>(S.Blueprint->GeneratedClass->GetDefaultObject());
    if (!CDO) { AddError(TEXT("CDO cast failed")); S.Cleanup(); return false; }

    TestNearlyEqual(TEXT("timestep"), CDO->SimOptions.Timestep, 0.005f, 1e-6f);
    TestNearlyEqual(TEXT("gravity z"), (float)CDO->SimOptions.Gravity.Z, -5.0f, 1e-4f);

    S.Cleanup();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_URLab_TendonArmature,
    "URLab.Import.URLab_TendonArmature",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_URLab_TendonArmature::RunTest(const FString&)
{
    FMjXmlImportSession S;
    if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <site name="a"/>
            <body pos="1 0 0">
              <joint name="sl" type="slide"/>
              <geom size=".1"/>
              <site name="b"/>
            </body>
          </worldbody>
          <tendon>
            <fixed name="tf" armature="2.5">
              <joint joint="sl" coef="1"/>
            </fixed>
          </tendon>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    UMjTendon* T = S.FindTemplate<UMjTendon>(TEXT("tf"));
    if (!T) { AddError(TEXT("Tendon 'tf' not found")); S.Cleanup(); return false; }

    TestNearlyEqual(TEXT("armature"), T->armature, 2.5f, 1e-4f);

    S.Cleanup();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_URLab_ContactPair,
    "URLab.Import.URLab_ContactPair",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_URLab_ContactPair::RunTest(const FString&)
{
    FMjXmlImportSession S;
    if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <geom name="floor" type="plane" size="1 1 1"/>
            <body>
              <geom name="ball" type="sphere" size=".1"/>
            </body>
          </worldbody>
          <contact>
            <pair geom1="floor" geom2="ball" condim="3"/>
          </contact>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    UMjContactPair* CP = S.FindFirstTemplate<UMjContactPair>();
    if (!CP) { AddError(TEXT("ContactPair not found")); S.Cleanup(); return false; }

    TestEqual(TEXT("geom1"), CP->geom1, FString(TEXT("floor")));
    TestEqual(TEXT("geom2"), CP->geom2, FString(TEXT("ball")));
    TestEqual(TEXT("condim"), CP->condim, 3);

    S.Cleanup();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_URLab_EqualityWeld,
    "URLab.Import.URLab_EqualityWeld",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_URLab_EqualityWeld::RunTest(const FString&)
{
    FMjXmlImportSession S;
    if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="b1"><geom size=".1"/></body>
            <body name="b2"><geom size=".1"/></body>
          </worldbody>
          <equality>
            <weld name="w1" body1="b1" body2="b2"/>
          </equality>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    UMjEquality* EQ = S.FindTemplate<UMjEquality>(TEXT("w1"));
    if (!EQ) { AddError(TEXT("Equality 'w1' not found")); S.Cleanup(); return false; }

    TestEqual(TEXT("body1"), EQ->Obj1, FString(TEXT("b1")));
    TestEqual(TEXT("body2"), EQ->Obj2, FString(TEXT("b2")));

    S.Cleanup();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_URLab_IncludeFile,
    "URLab.Import.URLab_IncludeFile",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_URLab_IncludeFile::RunTest(const FString&)
{
    // Write child XML into the same temp dir that FMjXmlImportSession uses,
    // then reference it via a relative path (MuJoCo resolves includes relative
    // to the parent XML file on Windows — absolute paths get mangled).
    FString TestDir = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLab/Tests/incl"));
    IFileManager::Get().MakeDirectory(*TestDir, true);
    FString ChildPath = FPaths::Combine(TestDir, TEXT("child.xml"));
    FFileHelper::SaveStringToFile(
        TEXT("<mujoco><body name=\"inc_body\"><geom name=\"inc_geom\" size=\".2\"/></body></mujoco>"),
        *ChildPath);

    // Use a path relative to the parent XML directory ({Saved}/URLab/Tests/)
    static const FString RelChildPath = TEXT("incl/child.xml");
    FString MainXml = FString::Printf(TEXT(R"(
        <mujoco>
          <worldbody>
            <include file="%s"/>
          </worldbody>
        </mujoco>
    )"), *RelChildPath);

    FMjXmlImportSession S;
    if (!S.Init(MainXml))
    {
        AddError(S.LastError);
        IFileManager::Get().Delete(*ChildPath);
        return false;
    }

    UMjGeom* G = S.FindTemplate<UMjGeom>(TEXT("inc_geom"));
    TestNotNull(TEXT("geom from include file found"), G);

    IFileManager::Get().Delete(*ChildPath);
    S.Cleanup();
    return true;
}

// =============================================================================
// ROUND-TRIP tests: compare Tier1 model counts with Tier2 compiled counts
// =============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_nbody,
    "URLab.Import.RoundTrip_nbody",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_nbody::RunTest(const FString&)
{
    static const TCHAR* Xml = TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="torso">
              <freejoint/>
              <geom type="sphere" size=".1"/>
              <body name="arm" pos="0 0 .2">
                <joint type="hinge"/>
                <geom type="capsule" size=".05" fromto="0 0 0 0 0 .2"/>
              </body>
            </body>
          </worldbody>
        </mujoco>
    )");

    // Reference: direct MuJoCo compile
    FMjTestSession Ref;
    if (!Ref.CompileXml(Xml)) { AddError(Ref.LastError); return false; }
    int ExpNbody = Ref.m->nbody;
    int ExpNgeom = Ref.m->ngeom;
    Ref.Cleanup();

    // URLab importer path
    FMjXmlImportSession S;
    if (!S.Init(Xml))  { AddError(S.LastError); return false; }
    if (!S.Compile())  { AddError(S.LastError); S.Cleanup(); return false; }

    TestEqual(TEXT("nbody matches reference"), (int)S.Model()->nbody, ExpNbody);
    TestEqual(TEXT("ngeom matches reference"), (int)S.Model()->ngeom, ExpNgeom);

    S.Cleanup();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_Sensors,
    "URLab.Import.RoundTrip_Sensors",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_Sensors::RunTest(const FString&)
{
    static const TCHAR* Xml = TEXT(R"(
        <mujoco>
          <worldbody>
            <body>
              <joint name="j1" type="hinge"/>
              <geom size=".1"/>
            </body>
          </worldbody>
          <sensor>
            <jointpos name="s1" joint="j1"/>
            <jointvel name="s2" joint="j1"/>
          </sensor>
        </mujoco>
    )");

    FMjTestSession Ref;
    if (!Ref.CompileXml(Xml)) { AddError(Ref.LastError); return false; }
    int ExpNsensor = Ref.m->nsensor;
    Ref.Cleanup();

    FMjXmlImportSession S;
    if (!S.Init(Xml))  { AddError(S.LastError); return false; }
    if (!S.Compile())  { AddError(S.LastError); S.Cleanup(); return false; }

    TestEqual(TEXT("nsensor matches reference"), (int)S.Model()->nsensor, ExpNsensor);

    S.Cleanup();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_Actuators,
    "URLab.Import.RoundTrip_Actuators",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_Actuators::RunTest(const FString&)
{
    static const TCHAR* Xml = TEXT(R"(
        <mujoco>
          <worldbody>
            <body>
              <joint name="j1" type="hinge"/>
              <geom size=".1"/>
            </body>
          </worldbody>
          <actuator>
            <motor name="a1" joint="j1"/>
            <position name="a2" joint="j1" kp="100"/>
          </actuator>
        </mujoco>
    )");

    FMjTestSession Ref;
    if (!Ref.CompileXml(Xml)) { AddError(Ref.LastError); return false; }
    int ExpNu = Ref.m->nu;
    Ref.Cleanup();

    FMjXmlImportSession S;
    if (!S.Init(Xml))  { AddError(S.LastError); return false; }
    if (!S.Compile())  { AddError(S.LastError); S.Cleanup(); return false; }

    TestEqual(TEXT("nu matches reference"), (int)S.Model()->nu, ExpNu);

    S.Cleanup();
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_Defaults,
    "URLab.Import.RoundTrip_Defaults",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_Defaults::RunTest(const FString&)
{
    // Default inheritance: geom inside a body with childclass="robot" should inherit
    // friction from the default.  Default values flow through mjs_add*() natively in
    // MuJoCo's spec API — this is why passing the resolved mjsDefault* to mjs_addGeom
    // is sufficient and no manual inheritance copy is needed.
    static const TCHAR* Xml = TEXT(R"(
        <mujoco>
          <default>
            <default class="robot">
              <geom friction="0.7 0.1 0.01"/>
            </default>
          </default>
          <worldbody>
            <body childclass="robot">
              <geom name="g1" size=".1"/>
            </body>
          </worldbody>
        </mujoco>
    )");

    FMjTestSession Ref;
    if (!Ref.CompileXml(Xml)) { AddError(Ref.LastError); return false; }
    int ExpNgeom = Ref.m->ngeom;
    float ExpFriction0 = (float)Ref.m->geom_friction[3 * Ref.GeomId("g1") + 0];
    Ref.Cleanup();

    FMjXmlImportSession S;
    if (!S.Init(Xml))  { AddError(S.LastError); return false; }
    if (!S.Compile())  { AddError(S.LastError); S.Cleanup(); return false; }

    // The URLab importer prefixes names so we can't use mj_name2id("g1") directly.
    // Instead compare ngeom count and check friction on the first non-worldbody geom.
    TestEqual(TEXT("ngeom matches reference"), (int)S.Model()->ngeom, ExpNgeom);
    if (S.Model()->ngeom > 0)
        TestNearlyEqual(TEXT("inherited friction[0]"), (float)S.Model()->geom_friction[0], 0.7f, 1e-4f);

    S.Cleanup();
    return true;
}

// Fix 2.11: <frame> elements are now handled by ImportNodeRecursive.
// This test verifies that geoms inside a frame are imported correctly and
// that the geom count in the compiled model matches the MuJoCo baseline.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_Frame_KnownGap,
    "URLab.Import.RoundTrip_Frame_KnownGap",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_Frame_KnownGap::RunTest(const FString&)
{
    static const TCHAR* Xml = TEXT(R"(
        <mujoco>
          <worldbody>
            <frame pos="0 0 1">
              <geom name="g_in_frame" size=".1"/>
            </frame>
          </worldbody>
        </mujoco>
    )");

    // Direct MuJoCo: 1 geom with z=1 from frame offset
    FMjTestSession Ref;
    if (!Ref.CompileXml(Xml)) { AddError(Ref.LastError); return false; }
    int ExpNgeom = Ref.m->ngeom;
    Ref.Cleanup();

    FMjXmlImportSession S;
    if (!S.Init(Xml))  { AddError(S.LastError); return false; }
    if (!S.Compile())  { AddError(S.LastError); S.Cleanup(); return false; }

    TestEqual(TEXT("ngeom matches after frame support fix"), (int)S.Model()->ngeom, ExpNgeom);

    S.Cleanup();
    return true;
}

// =============================================================================
// URLab.Import.MJ_JointAxisImport
//   MuJoCo joint axis (0,1,0) → stored UE axis should have Y negated.
//   Verifies Fix 3.5 through the importer (via UMjJoint::ImportFromXml).
// =============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_MJ_JointAxisImport,
    "URLab.Import.MJ_JointAxisImport",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_MJ_JointAxisImport::RunTest(const FString&)
{
    FMjXmlImportSession S;
    if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body>
              <joint name="j1" type="hinge" axis="0 1 0"/>
              <geom size=".1"/>
            </body>
          </worldbody>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    UMjJoint* JC = S.FindTemplate<UMjJoint>(TEXT("j1"));
    TestNotNull(TEXT("joint j1 found"), JC);
    if (JC)
    {
        // axis="0 1 0" in MuJoCo → stored in UE as (0,-1,0) after Y-negate
        TestTrue(TEXT("Axis X ≈ 0"),  FMath::Abs(JC->Axis.X) < 1e-4f);
        TestTrue(TEXT("Axis Y ≈ -1"), FMath::Abs(JC->Axis.Y + 1.0f) < 1e-4f);
        TestTrue(TEXT("Axis Z ≈ 0"),  FMath::Abs(JC->Axis.Z) < 1e-4f);
    }

    S.Cleanup();
    return true;
}

// =============================================================================
// URLab.Import.MJ_SensorTypeFromTag
//   Sensor XML tag must determine the sensor Type enum in UMjSensor.
//   Verifies Fix 2.4: ImportFromXml now reads the tag name.
// =============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_MJ_SensorTypeFromTag,
    "URLab.Import.MJ_SensorTypeFromTag",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_MJ_SensorTypeFromTag::RunTest(const FString&)
{
    FMjXmlImportSession S;
    if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body>
              <geom size=".1"/>
              <joint name="j1" type="hinge"/>
              <site name="s1"/>
            </body>
          </worldbody>
          <sensor>
            <accelerometer  name="s_acc"  site="s1"/>
            <gyro           name="s_gyro" site="s1"/>
            <jointpos       name="s_jp"   joint="j1"/>
            <velocimeter    name="s_vel"  site="s1"/>
          </sensor>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    UMjSensor* Acc  = S.FindTemplate<UMjSensor>(TEXT("s_acc"));
    UMjSensor* Gyro = S.FindTemplate<UMjSensor>(TEXT("s_gyro"));
    UMjSensor* JP   = S.FindTemplate<UMjSensor>(TEXT("s_jp"));
    UMjSensor* Vel  = S.FindTemplate<UMjSensor>(TEXT("s_vel"));

    if (Acc)  TestEqual(TEXT("s_acc  type"),  Acc->Type,  EMjSensorType::Accelerometer);
    if (Gyro) TestEqual(TEXT("s_gyro type"),  Gyro->Type, EMjSensorType::Gyro);
    if (JP)   TestEqual(TEXT("s_jp   type"),  JP->Type,   EMjSensorType::JointPos);
    if (Vel)  TestEqual(TEXT("s_vel  type"),  Vel->Type,  EMjSensorType::Velocimeter);

    S.Cleanup();
    return true;
}

// =============================================================================
// URLab.Import.MJ_MocapBody
//   mocap="true" attribute on <body> must set mocap=true.
//   Verifies Fix 2.9.
// =============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_MJ_MocapBody,
    "URLab.Import.MJ_MocapBody",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_MJ_MocapBody::RunTest(const FString&)
{
    FMjXmlImportSession S;
    if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="ref" mocap="true">
              <geom size=".1"/>
            </body>
            <body name="b1">
              <geom size=".1"/>
            </body>
          </worldbody>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    UMjBody* RefBody = S.FindTemplate<UMjBody>(TEXT("ref"));
    UMjBody* B1      = S.FindTemplate<UMjBody>(TEXT("b1"));

    if (RefBody) TestTrue(TEXT("ref body mocap=true"),  RefBody->mocap);
    if (B1)      TestTrue(TEXT("b1 body mocap=false"), !B1->mocap);

    S.Cleanup();
    return true;
}

// =============================================================================
// URLab.Import.MJ_AutoLimits
//   MuJoCo 3.x defaults autolimits="true" — a joint with range= is auto-limited.
//   Explicit limited="false" opts out.
// =============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_MJ_AutoLimits,
    "URLab.Import.MJ_AutoLimits",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_MJ_AutoLimits::RunTest(const FString&)
{
    // Default autolimits: range present → joint IS limited
    FMjTestSession SAuto;
    if (!SAuto.CompileXml(TEXT(R"(
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <body>
              <joint name="j1" type="hinge" range="-1 1"/>
              <geom size=".1"/>
            </body>
          </worldbody>
        </mujoco>
    )"))) { AddError(SAuto.LastError); return false; }
    int jid = SAuto.JointId("j1");
    TestTrue(TEXT("j1 valid"), jid >= 0);
    if (jid >= 0)
        TestTrue(TEXT("j1 IS limited by default autolimits"), SAuto.m->jnt_limited[jid] == 1);
    SAuto.Cleanup();

    // Explicit limited="false" overrides autolimits → joint NOT limited
    FMjTestSession SOverride;
    if (!SOverride.CompileXml(TEXT(R"(
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <body>
              <joint name="j2" type="hinge" range="-1 1" limited="false"/>
              <geom size=".1"/>
            </body>
          </worldbody>
        </mujoco>
    )"))) { AddError(SOverride.LastError); return false; }
    int jid2 = SOverride.JointId("j2");
    TestTrue(TEXT("j2 valid"), jid2 >= 0);
    if (jid2 >= 0)
        TestTrue(TEXT("j2 NOT limited via explicit limited=false"), SOverride.m->jnt_limited[jid2] == 0);
    SOverride.Cleanup();
    return true;
}

// =============================================================================
// URLab.Import.URLab_WeldTorqueScale
//   <weld torquescale="2.5"/> must set TorqueScale=2.5 and bOverride_TorqueScale=true
//   on the imported UMjEquality component.
//   Verifies Fix 3.16 through the URLab importer (not raw MuJoCo API).
// =============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_URLab_WeldTorqueScale,
    "URLab.Import.URLab_WeldTorqueScale",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_URLab_WeldTorqueScale::RunTest(const FString&)
{
    FMjXmlImportSession S;
    if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="b1"><geom size=".1"/></body>
            <body name="b2"><geom size=".1"/></body>
          </worldbody>
          <equality>
            <weld name="w1" body1="b1" body2="b2" torquescale="2.5"/>
          </equality>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    UMjEquality* Weld = S.FindTemplate<UMjEquality>(TEXT("w1"));
    TestNotNull(TEXT("weld equality 'w1' found"), Weld);
    if (Weld)
    {
        TestTrue(TEXT("bOverride_TorqueScale == true"), Weld->bOverride_torquescale);
        TestTrue(TEXT("TorqueScale ≈ 2.5"), FMath::Abs(Weld->torquescale - 2.5f) < 1e-4f);
    }
    S.Cleanup();
    return true;
}

// =============================================================================
// URLab.Import.URLab_ConnectAnchor
//   <connect anchor="0.1 -0.2 0.3"/> reads as TArray<float> in UE cm
//   (m_to_cm conversion on import) and bOverride_anchor=true.
//   Regression: anchor used to be FString and wasn't being written to
//   mjsEquality.data[] on export.
// =============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_URLab_ConnectAnchor,
    "URLab.Import.URLab_ConnectAnchor",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_URLab_ConnectAnchor::RunTest(const FString&)
{
    FMjXmlImportSession S;
    if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="b1"><geom size=".1"/><freejoint/></body>
            <body name="b2"><geom size=".1"/><freejoint/></body>
          </worldbody>
          <equality>
            <connect name="c1" body1="b1" body2="b2" anchor="0.1 -0.2 0.3"/>
          </equality>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    UMjEquality* EQ = S.FindTemplate<UMjEquality>(TEXT("c1"));
    if (!EQ) { AddError(TEXT("Equality 'c1' not found")); S.Cleanup(); return false; }

    TestTrue(TEXT("bOverride_anchor == true"), EQ->bOverride_anchor);
    if (EQ->anchor.Num() >= 3)
    {
        // XML 0.1 m -> UE 10 cm; XML -0.2 m -> -20 cm; XML 0.3 m -> 30 cm
        TestNearlyEqual(TEXT("anchor[0] = 10 cm"),  EQ->anchor[0],  10.0f, 1e-3f);
        TestNearlyEqual(TEXT("anchor[1] = -20 cm"), EQ->anchor[1], -20.0f, 1e-3f);
        TestNearlyEqual(TEXT("anchor[2] = 30 cm"),  EQ->anchor[2],  30.0f, 1e-3f);
    }
    else
    {
        AddError(FString::Printf(TEXT("anchor.Num() = %d (expected 3)"), EQ->anchor.Num()));
    }
    S.Cleanup();
    return true;
}

// =============================================================================
// URLab.Import.RoundTrip_ConnectAnchor
//   Verifies that the codegen-emitted cm_to_m export op on
//   mjs_data_packed_attrs.anchor packs UE cm into spec data[0..2] in m.
//   Tests the spec write directly (raw mjsEquality) since UMjEquality is
//   registered on a per-articulation child spec and the body-name namespace
//   crossing in mjs_attach is independently exercised elsewhere — what we
//   care about here is the per-slot data[] layout for the connect kind.
// =============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_ConnectAnchor,
    "URLab.Import.RoundTrip_ConnectAnchor",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_ConnectAnchor::RunTest(const FString&)
{
    UMjEquality* Eq = NewObject<UMjEquality>();
    Eq->EqualityType = EMjEqualityType::Connect;
    Eq->Obj1 = TEXT("b1");
    Eq->Obj2 = TEXT("b2");
    Eq->bOverride_anchor = true;
    Eq->anchor = { 10.0f, -20.0f, 30.0f };   // UE cm; export ×0.01 → 0.1, -0.2, 0.3 m

    mjSpec* TestSpec = mj_makeSpec();
    mjsEquality* SpecEq = mjs_addEquality(TestSpec, nullptr);
    Eq->ExportTo(SpecEq);

    TestEqual(TEXT("type == connect"), (int)SpecEq->type, (int)mjtEq::mjEQ_CONNECT);
    TestNearlyEqual(TEXT("data[0] = 0.1 m"),  (float)SpecEq->data[0],  0.1f, 1e-4f);
    TestNearlyEqual(TEXT("data[1] = -0.2 m"), (float)SpecEq->data[1], -0.2f, 1e-4f);
    TestNearlyEqual(TEXT("data[2] = 0.3 m"),  (float)SpecEq->data[2],  0.3f, 1e-4f);

    mj_deleteSpec(TestSpec);
    return true;
}

// =============================================================================
// URLab.Import.E2E_GripperConnectAnchor
//   Full URLab pipeline test: hinge-jointed gripper-like linkage with a
//   <connect> equality. After Compile(), the model must have neq==1 and the
//   anchor must survive into eq_data[0..2] in metres (XML m units).
//
//   Uses hinge joints (no freejoint) because mjs_attach errors out on
//   free-joint child bodies, which matches a real 2F-85-style topology.
// =============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_E2E_GripperConnectAnchor,
    "URLab.Import.E2E_GripperConnectAnchor",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_E2E_GripperConnectAnchor::RunTest(const FString&)
{
    // Minimal Robotiq-2F85-style 4-bar linkage (right finger pair only).
    static const TCHAR* Xml = TEXT(R"(
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <body name="base">
              <geom size="0.05"/>
              <body name="follower" pos="0 0 0.1">
                <joint name="jf" type="hinge" axis="1 0 0"/>
                <geom size="0.01"/>
              </body>
              <body name="spring_link" pos="0 0.02 0.1">
                <joint name="js" type="hinge" axis="1 0 0"/>
                <geom size="0.01"/>
              </body>
            </body>
          </worldbody>
          <equality>
            <connect name="pin" body1="follower" body2="spring_link"
                     anchor="0 -0.018 0.0065"/>
          </equality>
        </mujoco>
    )");

    FMjXmlImportSession S;
    if (!S.Init(Xml))  { AddError(S.LastError); return false; }
    if (!S.Compile())  { AddError(S.LastError); S.Cleanup(); return false; }

    const mjModel* M = S.Model();
    TestEqual(TEXT("neq after URLab pipeline"), (int)M->neq, 1);
    if (M->neq >= 1)
    {
        // Body refs survived mjs_attach prefix
        const int o1 = M->eq_obj1id[0];
        const int o2 = M->eq_obj2id[0];
        TestTrue(TEXT("eq.obj1id resolved"), o1 > 0);
        TestTrue(TEXT("eq.obj2id resolved"), o2 > 0);

        // anchor data[0..2] in spec metres (XML literal values)
        TestNearlyEqual(TEXT("eq_data[0] = 0.0"),    (float)M->eq_data[0],  0.0f,    1e-4f);
        TestNearlyEqual(TEXT("eq_data[1] = -0.018"), (float)M->eq_data[1], -0.018f,  1e-4f);
        TestNearlyEqual(TEXT("eq_data[2] = 0.0065"), (float)M->eq_data[2],  0.0065f, 1e-4f);
    }
    S.Cleanup();
    return true;
}

// =============================================================================
// URLab.Import.RoundTrip_WeldAnchorRelposeTorqueScale
//   Verifies the full weld data[] layout: anchor -> data[0..2] (with
//   cm_to_m), relpose -> data[3..9] (raw MJ format, no conversion),
//   torquescale -> data[10]. Older codegen wrote torquescale to data[7] and
//   skipped anchor/relpose entirely.
// =============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_WeldAnchorRelposeTorqueScale,
    "URLab.Import.RoundTrip_WeldAnchorRelposeTorqueScale",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_WeldAnchorRelposeTorqueScale::RunTest(const FString&)
{
    UMjEquality* Eq = NewObject<UMjEquality>();
    Eq->EqualityType = EMjEqualityType::Weld;
    Eq->Obj1 = TEXT("b1");
    Eq->Obj2 = TEXT("b2");
    Eq->bOverride_anchor = true;
    Eq->anchor = { 50.0f, 60.0f, 70.0f };   // UE cm -> 0.5, 0.6, 0.7 m
    Eq->bOverride_relpose = true;
    Eq->relpose = { 1.0f, 2.0f, 3.0f, 0.7071f, 0.0f, 0.7071f, 0.0f };  // raw MJ (m + quat)
    Eq->bOverride_torquescale = true;
    Eq->torquescale = 42.0f;

    mjSpec* TestSpec = mj_makeSpec();
    mjsEquality* SpecEq = mjs_addEquality(TestSpec, nullptr);
    Eq->ExportTo(SpecEq);

    TestEqual(TEXT("type == weld"), (int)SpecEq->type, (int)mjtEq::mjEQ_WELD);
    // anchor (cm -> m)
    TestNearlyEqual(TEXT("data[0] = 0.5 m"), (float)SpecEq->data[0], 0.5f, 1e-4f);
    TestNearlyEqual(TEXT("data[1] = 0.6 m"), (float)SpecEq->data[1], 0.6f, 1e-4f);
    TestNearlyEqual(TEXT("data[2] = 0.7 m"), (float)SpecEq->data[2], 0.7f, 1e-4f);
    // relpose pos (raw)
    TestNearlyEqual(TEXT("data[3] = relpose pos x"), (float)SpecEq->data[3], 1.0f, 1e-4f);
    TestNearlyEqual(TEXT("data[4] = relpose pos y"), (float)SpecEq->data[4], 2.0f, 1e-4f);
    TestNearlyEqual(TEXT("data[5] = relpose pos z"), (float)SpecEq->data[5], 3.0f, 1e-4f);
    // relpose quat (raw)
    TestNearlyEqual(TEXT("data[6] = relpose quat w"), (float)SpecEq->data[6], 0.7071f, 1e-4f);
    TestNearlyEqual(TEXT("data[7] = relpose quat x"), (float)SpecEq->data[7], 0.0f,    1e-4f);
    TestNearlyEqual(TEXT("data[8] = relpose quat y"), (float)SpecEq->data[8], 0.7071f, 1e-4f);
    TestNearlyEqual(TEXT("data[9] = relpose quat z"), (float)SpecEq->data[9], 0.0f,    1e-4f);
    // torquescale at slot 10 (was wrongly slot 7 in older codegen)
    TestNearlyEqual(TEXT("data[10] = torquescale"), (float)SpecEq->data[10], 42.0f, 1e-4f);

    mj_deleteSpec(TestSpec);
    return true;
}

// =============================================================================
// URLab.Import.RoundTrip_JointEqualityPolycoef
//   Joint equality with polycoef must pack into data[0..4] AND set
//   objtype = mjOBJ_JOINT. Without the objtype set, mjs_attach silently
//   drops the equality at compile time.
// =============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_JointEqualityPolycoef,
    "URLab.Import.RoundTrip_JointEqualityPolycoef",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_JointEqualityPolycoef::RunTest(const FString&)
{
    UMjEquality* Eq = NewObject<UMjEquality>();
    Eq->EqualityType = EMjEqualityType::Joint;
    Eq->Obj1 = TEXT("ja");
    Eq->Obj2 = TEXT("jb");
    Eq->bOverride_polycoef = true;
    Eq->polycoef = { 5.0f, 6.0f, 7.0f, 8.0f, 9.0f };

    mjSpec* TestSpec = mj_makeSpec();
    mjsEquality* SpecEq = mjs_addEquality(TestSpec, nullptr);
    Eq->ExportTo(SpecEq);

    TestEqual(TEXT("type == joint"), (int)SpecEq->type, (int)mjtEq::mjEQ_JOINT);
    TestEqual(TEXT("objtype == joint"), (int)SpecEq->objtype, (int)mjOBJ_JOINT);
    TestNearlyEqual(TEXT("data[0] = 5"), (float)SpecEq->data[0], 5.0f, 1e-4f);
    TestNearlyEqual(TEXT("data[1] = 6"), (float)SpecEq->data[1], 6.0f, 1e-4f);
    TestNearlyEqual(TEXT("data[2] = 7"), (float)SpecEq->data[2], 7.0f, 1e-4f);
    TestNearlyEqual(TEXT("data[3] = 8"), (float)SpecEq->data[3], 8.0f, 1e-4f);
    TestNearlyEqual(TEXT("data[4] = 9"), (float)SpecEq->data[4], 9.0f, 1e-4f);

    mj_deleteSpec(TestSpec);
    return true;
}

// =============================================================================
// URLab.Import.RoundTrip_TendonEqualityPolycoef
//   Tendon equality: same polycoef packing, but objtype must be mjOBJ_TENDON.
// =============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_TendonEqualityPolycoef,
    "URLab.Import.RoundTrip_TendonEqualityPolycoef",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_TendonEqualityPolycoef::RunTest(const FString&)
{
    UMjEquality* Eq = NewObject<UMjEquality>();
    Eq->EqualityType = EMjEqualityType::Tendon;
    Eq->Obj1 = TEXT("ta");
    Eq->Obj2 = TEXT("tb");
    Eq->bOverride_polycoef = true;
    Eq->polycoef = { 0.0f, 1.0f, 0.0f, 0.0f, 0.0f };

    mjSpec* TestSpec = mj_makeSpec();
    mjsEquality* SpecEq = mjs_addEquality(TestSpec, nullptr);
    Eq->ExportTo(SpecEq);

    TestEqual(TEXT("type == tendon"), (int)SpecEq->type, (int)mjtEq::mjEQ_TENDON);
    TestEqual(TEXT("objtype == tendon"), (int)SpecEq->objtype, (int)mjOBJ_TENDON);
    TestNearlyEqual(TEXT("data[0]"), (float)SpecEq->data[0], 0.0f, 1e-4f);
    TestNearlyEqual(TEXT("data[1]"), (float)SpecEq->data[1], 1.0f, 1e-4f);

    mj_deleteSpec(TestSpec);
    return true;
}

// =============================================================================
// URLab.Import.RoundTrip_FlexEqualityObjType
//   Flex / FlexVert / FlexStrain equalities must all set objtype = mjOBJ_FLEX.
//   Covers all three Flex* arms of the objtype switch in MjEquality.cpp.
// =============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_FlexEqualityObjType,
    "URLab.Import.RoundTrip_FlexEqualityObjType",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_FlexEqualityObjType::RunTest(const FString&)
{
    const EMjEqualityType FlexKinds[] = {
        EMjEqualityType::Flex,
        EMjEqualityType::FlexVert,
        EMjEqualityType::FlexStrain,
    };

    for (EMjEqualityType Kind : FlexKinds)
    {
        UMjEquality* Eq = NewObject<UMjEquality>();
        Eq->EqualityType = Kind;
        Eq->Obj1 = TEXT("cloth");

        mjSpec* TestSpec = mj_makeSpec();
        mjsEquality* SpecEq = mjs_addEquality(TestSpec, nullptr);
        Eq->ExportTo(SpecEq);

        const FString Label = FString::Printf(TEXT("flex-kind %d -> objtype == mjOBJ_FLEX"),
            (int)Kind);
        TestEqual(Label, (int)SpecEq->objtype, (int)mjOBJ_FLEX);

        mj_deleteSpec(TestSpec);
    }
    return true;
}

// =============================================================================
// URLab.Import.E2E_JointEqualityCoupling
//   Joint coupling equality (joint1=ja, joint2=jb, polycoef="0 1 0 0 0")
//   must survive the URLab end-to-end pipeline and resolve both joint refs
//   in the compiled mjModel. Matches the left/right driver coupling of a
//   2F-85-style gripper.
// =============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_E2E_JointEqualityCoupling,
    "URLab.Import.E2E_JointEqualityCoupling",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_E2E_JointEqualityCoupling::RunTest(const FString&)
{
    static const TCHAR* Xml = TEXT(R"(
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <body name="base">
              <geom size="0.05"/>
              <body name="a" pos="0 0 0.1">
                <joint name="ja" type="hinge" axis="1 0 0"/>
                <geom size="0.01"/>
              </body>
              <body name="b" pos="0 0.1 0.1">
                <joint name="jb" type="hinge" axis="1 0 0"/>
                <geom size="0.01"/>
              </body>
            </body>
          </worldbody>
          <equality>
            <joint name="couple" joint1="ja" joint2="jb" polycoef="0 1 0 0 0"/>
          </equality>
        </mujoco>
    )");

    FMjXmlImportSession S;
    if (!S.Init(Xml))  { AddError(S.LastError); return false; }
    if (!S.Compile())  { AddError(S.LastError); S.Cleanup(); return false; }

    const mjModel* M = S.Model();
    TestEqual(TEXT("neq == 1"), (int)M->neq, 1);
    if (M->neq >= 1)
    {
        TestEqual(TEXT("eq_type == joint"), (int)M->eq_type[0], (int)mjEQ_JOINT);
        // For a joint equality the obj ids reference joints (not bodies)
        TestTrue(TEXT("eq.obj1id resolved"), M->eq_obj1id[0] >= 0);
        TestTrue(TEXT("eq.obj2id resolved"), M->eq_obj2id[0] >= 0);
        // polycoef coefficient slot — c1=1 means joint1 tracks joint2.
        // eq_data is a flat (neq x mjNEQDATA) array; we want slot 1 of eq 0.
        TestNearlyEqual(TEXT("polycoef c1 = 1"), (float)M->eq_data[1], 1.0f, 1e-4f);
    }
    S.Cleanup();
    return true;
}

// =============================================================================
// URLab.Import.MJ_FrameSensor_ObjType
//   Tier 1: <framepos objtype="body" objname="b1"/> must compile to
//   sensor_objtype == mjOBJ_BODY in the compiled model.
//   Verifies that MuJoCo accepts the objtype+objname pattern for frame sensors.
// =============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_MJ_FrameSensor_ObjType,
    "URLab.Import.MJ_FrameSensor_ObjType",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_MJ_FrameSensor_ObjType::RunTest(const FString&)
{
    FMjTestSession S;
    if (!S.CompileXml(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="b1">
              <geom size=".1"/>
              <freejoint/>
            </body>
          </worldbody>
          <sensor>
            <framepos  name="fp1" objtype="body" objname="b1"/>
            <framequat name="fq1" objtype="body" objname="b1" reftype="body" refname="world"/>
          </sensor>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    int fpId = mj_name2id(S.m, mjOBJ_SENSOR, "fp1");
    TestTrue(TEXT("framepos sensor found"), fpId >= 0);
    if (fpId >= 0)
    {
        TestTrue(TEXT("framepos sensor_objtype == mjOBJ_BODY"),
            S.m->sensor_objtype[fpId] == mjOBJ_BODY);
    }

    int fqId = mj_name2id(S.m, mjOBJ_SENSOR, "fq1");
    TestTrue(TEXT("framequat sensor found"), fqId >= 0);
    if (fqId >= 0)
    {
        TestTrue(TEXT("framequat sensor_objtype == mjOBJ_BODY"),
            S.m->sensor_objtype[fqId] == mjOBJ_BODY);
        TestTrue(TEXT("framequat sensor_reftype == mjOBJ_BODY"),
            S.m->sensor_reftype[fqId] == mjOBJ_BODY);
    }
    S.Cleanup();
    return true;
}

// =============================================================================
// URLab.Import.URLab_FrameSensor_ObjRefType
//   Tier 2: <framepos objtype="body" objname="b1"/> → UMjSensor must have
//   ObjType == Body, ReferenceName populated, and the round-trip compile must
//   produce sensor_objtype == mjOBJ_BODY.
//   Verifies Fix 2.10 (objtype/reftype parsing in ImportFromXml).
// =============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_URLab_FrameSensor_ObjRefType,
    "URLab.Import.URLab_FrameSensor_ObjRefType",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_URLab_FrameSensor_ObjRefType::RunTest(const FString&)
{
    FMjXmlImportSession S;
    if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="b1">
              <geom size=".1"/>
              <freejoint/>
              <site name="s1"/>
            </body>
          </worldbody>
          <sensor>
            <framepos  name="fp1" objtype="body" objname="b1"/>
            <framequat name="fq1" objtype="body" objname="b1" reftype="body" refname="world"/>
          </sensor>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    // Check UMjSensor component properties
    UMjSensor* FP = S.FindTemplate<UMjSensor>(TEXT("fp1"));
    TestNotNull(TEXT("framepos sensor 'fp1' found"), FP);
    if (FP)
    {
        TestTrue(TEXT("fp1 ObjType == Body"), FP->ObjType == EMjObjType::Body);
        TestTrue(TEXT("fp1 TargetName == 'b1'"), FP->TargetName == TEXT("b1"));
    }

    UMjSensor* FQ = S.FindTemplate<UMjSensor>(TEXT("fq1"));
    TestNotNull(TEXT("framequat sensor 'fq1' found"), FQ);
    if (FQ)
    {
        TestTrue(TEXT("fq1 ObjType == Body"), FQ->ObjType == EMjObjType::Body);
        TestTrue(TEXT("fq1 RefType == Body"), FQ->RefType == EMjObjType::Body);
        TestTrue(TEXT("fq1 ReferenceName == 'world'"), FQ->ReferenceName == TEXT("world"));
    }

    // Round-trip compile: sensors must compile; names are prefixed by the actor name so
    // we cannot look them up by bare "fp1". Instead verify count and objtype by iteration.
    S.Compile();
    if (S.Manager && S.Manager->PhysicsEngine->m_model)
    {
        mjModel* M = S.Manager->PhysicsEngine->m_model;
        // Expect exactly 2 sensors (framepos + framequat)
        TestTrue(TEXT("fp1+fq1 both compile in round-trip"), M->nsensor == 2);
        // At least one sensor should have objtype == mjOBJ_BODY (framepos referencing b1)
        bool bFoundBodyObjType = false;
        for (int i = 0; i < M->nsensor; ++i)
        {
            if (M->sensor_objtype[i] == mjOBJ_BODY)
            {
                bFoundBodyObjType = true;
                break;
            }
        }
        TestTrue(TEXT("fp1 compiled sensor_objtype == mjOBJ_BODY"), bFoundBodyObjType);
    }

    S.Cleanup();
    return true;
}

// =============================================================================
// URLab.Import.DefaultClassJointAxis
//   Joint axis inherited from a <default> class must survive import→compile.
//   The default sets axis="0 1 0"; a joint that inherits (no explicit axis)
//   must compile with jnt_axis=(0,1,0) in MuJoCo, not the builtin (0,0,1).
//   A second joint with an explicit axis="1 0 0" is checked as a control.
// =============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_DefaultClassJointAxis,
    "URLab.Import.DefaultClassJointAxis",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_DefaultClassJointAxis::RunTest(const FString&)
{
    FMjXmlImportSession S;
    if (!S.Init(TEXT(R"(
        <mujoco>
          <default>
            <default class="testclass">
              <joint axis="0 1 0" armature="0.1"/>
            </default>
          </default>
          <worldbody>
            <body childclass="testclass">
              <joint name="inherited" type="hinge" range="-1 1"/>
              <joint name="explicit" type="hinge" axis="1 0 0" range="-1 1"/>
              <geom size=".1"/>
            </body>
          </worldbody>
        </mujoco>
    )"))) { AddError(S.LastError); return false; }

    // Tier 1: check imported UE properties
    UMjJoint* JInherited = S.FindTemplate<UMjJoint>(TEXT("inherited"));
    UMjJoint* JExplicit  = S.FindTemplate<UMjJoint>(TEXT("explicit"));
    TestNotNull(TEXT("inherited joint found"), JInherited);
    TestNotNull(TEXT("explicit joint found"),  JExplicit);

    if (JExplicit)
    {
        // axis="1 0 0" in MuJoCo → UE (1, 0, 0) (only Y negates, X and Z unchanged)
        TestTrue(TEXT("explicit Axis X ≈ 1"),  FMath::Abs(JExplicit->Axis.X - 1.0f) < 1e-4f);
        TestTrue(TEXT("explicit Axis Y ≈ 0"),  FMath::Abs(JExplicit->Axis.Y) < 1e-4f);
        TestTrue(TEXT("explicit Axis Z ≈ 0"),  FMath::Abs(JExplicit->Axis.Z) < 1e-4f);
    }

    // Tier 2: compile and check jnt_axis in the compiled model
    if (!S.Compile()) { AddError(S.LastError); S.Cleanup(); return false; }

    const mjModel* M = S.Model();
    TestNotNull(TEXT("compiled model"), M);
    if (!M) { S.Cleanup(); return false; }

    // Find joints by name in the compiled model
    int idInherited = mj_name2id(M, mjOBJ_JOINT, "inherited");
    int idExplicit  = mj_name2id(M, mjOBJ_JOINT, "explicit");

    // Joints may be prefixed — search with prefix if not found
    if (idInherited < 0 || idExplicit < 0)
    {
        for (int j = 0; j < M->njnt; ++j)
        {
            const char* name = mj_id2name(M, mjOBJ_JOINT, j);
            if (!name) continue;
            FString N = UTF8_TO_TCHAR(name);
            if (N.Contains(TEXT("inherited"))) idInherited = j;
            if (N.Contains(TEXT("explicit")))  idExplicit  = j;
        }
    }

    TestTrue(TEXT("inherited joint compiled"), idInherited >= 0);
    TestTrue(TEXT("explicit joint compiled"),  idExplicit >= 0);

    if (idInherited >= 0)
    {
        const mjtNum* ax = &M->jnt_axis[idInherited * 3];
        TestTrue(TEXT("inherited jnt_axis[0] ≈ 0"), FMath::Abs((float)ax[0]) < 1e-4f);
        TestTrue(TEXT("inherited jnt_axis[1] ≈ 1"), FMath::Abs((float)ax[1] - 1.0f) < 1e-4f);
        TestTrue(TEXT("inherited jnt_axis[2] ≈ 0"), FMath::Abs((float)ax[2]) < 1e-4f);
    }

    if (idExplicit >= 0)
    {
        const mjtNum* ax = &M->jnt_axis[idExplicit * 3];
        TestTrue(TEXT("explicit jnt_axis[0] ≈ 1"), FMath::Abs((float)ax[0] - 1.0f) < 1e-4f);
        TestTrue(TEXT("explicit jnt_axis[1] ≈ 0"), FMath::Abs((float)ax[1]) < 1e-4f);
        TestTrue(TEXT("explicit jnt_axis[2] ≈ 0"), FMath::Abs((float)ax[2]) < 1e-4f);
    }

    S.Cleanup();
    return true;
}

// =============================================================================
// URLab.Import.DefaultClass_JointName_Collision
//   Regression for MJCFs that share a label between a <default class="X"> and
//   a <joint name="X"> — the idiomatic Menagerie pattern for per-joint tuning
//   (vx300s, wx250s, aloha, ...). Before MjName was populated from the XML
//   name= attribute, the SCS uniqueness rule forced the joint's UE variable
//   name to "X1" while the actuator's joint="X" reference kept the raw label,
//   so MuJoCo's compiler dropped every actuator silently (nu == 0).
//   Compare URLab's nu against MuJoCo's baseline nu for the same XML; they
//   must match.
// =============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_DefaultClassJointNameCollision,
    "URLab.Import.DefaultClass_JointName_Collision",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_DefaultClassJointNameCollision::RunTest(const FString&)
{
    static const TCHAR* Xml = TEXT(R"(
        <mujoco>
          <default>
            <default class="hip">
              <joint damping="5"/>
              <position kp="100"/>
            </default>
          </default>
          <worldbody>
            <body>
              <joint name="hip" class="hip" type="hinge"/>
              <geom size=".1"/>
            </body>
          </worldbody>
          <actuator>
            <position class="hip" name="hip" joint="hip"/>
          </actuator>
        </mujoco>
    )");

    // Baseline: what MuJoCo itself produces from this XML.
    FMjTestSession Ref;
    if (!Ref.CompileXml(Xml)) { AddError(Ref.LastError); return false; }
    const int ExpNu = Ref.m->nu;
    const int ExpNjnt = Ref.m->njnt;
    Ref.Cleanup();
    TestEqual(TEXT("baseline MuJoCo nu == 1"), ExpNu, 1);
    TestEqual(TEXT("baseline MuJoCo njnt == 1"), ExpNjnt, 1);

    // Through URLab's importer + compile.
    FMjXmlImportSession S;
    if (!S.Init(Xml))  { AddError(S.LastError); return false; }
    if (!S.Compile())  { AddError(S.LastError); S.Cleanup(); return false; }

    TestEqual(TEXT("URLab nu matches MuJoCo baseline (actuator survived Default-class name collision)"),
        (int)S.Model()->nu, ExpNu);
    TestEqual(TEXT("URLab njnt matches MuJoCo baseline"),
        (int)S.Model()->njnt, ExpNjnt);

    S.Cleanup();
    return true;
}

// =============================================================================
// CAMERA + GEOM EXPORT-GAP REGRESSION TESTS
//
// These cover the audit findings that landed alongside the gripper-attach
// fix: schema attrs that were imported into UPROPERTYs but silently dropped
// on export because MuJoCo renames the underlying mjsX field
// (target -> targetbody, focal -> focal_length, shellinertia -> typeinertia,
// fluidshape -> fluid_ellipsoid, etc.). Without these tests the gaps would
// regress every time codegen_rules.json is touched.
// =============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_CameraTarget,
    "URLab.Import.RoundTrip_CameraTarget",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_CameraTarget::RunTest(const FString&)
{
    UMjCamera* Cam = NewObject<UMjCamera>();
    Cam->bOverride_target = true;
    Cam->target = TEXT("torso");

    mjSpec* TestSpec = mj_makeSpec();
    mjsBody* World = mjs_findBody(TestSpec, "world");
    mjsCamera* SpecCam = mjs_addCamera(World, nullptr);
    Cam->ExportTo(SpecCam, nullptr);

    const char* tb = mjs_getString(SpecCam->targetbody);
    TestNotNull(TEXT("targetbody mjString allocated"), tb);
    TestEqual(TEXT("targetbody == 'torso'"), FString(UTF8_TO_TCHAR(tb)), FString(TEXT("torso")));

    mj_deleteSpec(TestSpec);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_CameraProjection,
    "URLab.Import.RoundTrip_CameraProjection",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_CameraProjection::RunTest(const FString&)
{
    UMjCamera* Cam = NewObject<UMjCamera>();
    Cam->bOverride_Projection = true;
    Cam->Projection = EMjCameraProjection::Orthographic;

    mjSpec* TestSpec = mj_makeSpec();
    mjsBody* World = mjs_findBody(TestSpec, "world");
    mjsCamera* SpecCam = mjs_addCamera(World, nullptr);
    Cam->ExportTo(SpecCam, nullptr);

    TestEqual(TEXT("proj == orthographic"),
        (int)SpecCam->proj, (int)mjPROJ_ORTHOGRAPHIC);

    // Round-trip the perspective case too.
    Cam->Projection = EMjCameraProjection::Perspective;
    Cam->ExportTo(SpecCam, nullptr);
    TestEqual(TEXT("proj == perspective"),
        (int)SpecCam->proj, (int)mjPROJ_PERSPECTIVE);

    mj_deleteSpec(TestSpec);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_CameraIntrinsics2Vec,
    "URLab.Import.RoundTrip_CameraIntrinsics2Vec",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_CameraIntrinsics2Vec::RunTest(const FString&)
{
    // mjsCamera.focal_length, principal_length, sensor_size are all 2-vec
    // float arrays. Before the audit fix, focal/principal were never written
    // and sensor_size wrote 3 elements into a 2-element field (OOB).
    UMjCamera* Cam = NewObject<UMjCamera>();
    Cam->bOverride_focal = true;
    Cam->focal = { 0.5f, 0.6f };
    Cam->bOverride_principal = true;
    Cam->principal = { 0.1f, 0.2f };
    Cam->bOverride_sensorsize = true;
    Cam->sensorsize = { 0.024f, 0.018f };

    mjSpec* TestSpec = mj_makeSpec();
    mjsBody* World = mjs_findBody(TestSpec, "world");
    mjsCamera* SpecCam = mjs_addCamera(World, nullptr);
    Cam->ExportTo(SpecCam, nullptr);

    TestNearlyEqual(TEXT("focal_length[0]"), SpecCam->focal_length[0], 0.5f, 1e-6f);
    TestNearlyEqual(TEXT("focal_length[1]"), SpecCam->focal_length[1], 0.6f, 1e-6f);
    TestNearlyEqual(TEXT("principal_length[0]"), SpecCam->principal_length[0], 0.1f, 1e-6f);
    TestNearlyEqual(TEXT("principal_length[1]"), SpecCam->principal_length[1], 0.2f, 1e-6f);
    TestNearlyEqual(TEXT("sensor_size[0]"), SpecCam->sensor_size[0], 0.024f, 1e-6f);
    TestNearlyEqual(TEXT("sensor_size[1]"), SpecCam->sensor_size[1], 0.018f, 1e-6f);

    mj_deleteSpec(TestSpec);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_GeomShellInertia,
    "URLab.Import.RoundTrip_GeomShellInertia",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_GeomShellInertia::RunTest(const FString&)
{
    UMjGeom* G = NewObject<UMjGeom>();
    G->bOverride_ShellInertia = true;
    G->ShellInertia = EMjGeomInertia::Shell;

    mjSpec* TestSpec = mj_makeSpec();
    mjsBody* World = mjs_findBody(TestSpec, "world");
    mjsGeom* SpecGeom = mjs_addGeom(World, nullptr);
    G->ExportTo(SpecGeom, nullptr);

    TestEqual(TEXT("typeinertia == SHELL"),
        (int)SpecGeom->typeinertia, (int)mjINERTIA_SHELL);

    G->ShellInertia = EMjGeomInertia::Volume;
    G->ExportTo(SpecGeom, nullptr);
    TestEqual(TEXT("typeinertia == VOLUME"),
        (int)SpecGeom->typeinertia, (int)mjINERTIA_VOLUME);

    mj_deleteSpec(TestSpec);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_GeomFluidShape,
    "URLab.Import.RoundTrip_GeomFluidShape",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_GeomFluidShape::RunTest(const FString&)
{
    UMjGeom* G = NewObject<UMjGeom>();
    G->bOverride_FluidShape = true;
    G->FluidShape = EMjFluidShape::Ellipsoid;

    mjSpec* TestSpec = mj_makeSpec();
    mjsBody* World = mjs_findBody(TestSpec, "world");
    mjsGeom* SpecGeom = mjs_addGeom(World, nullptr);
    G->ExportTo(SpecGeom, nullptr);

    TestNearlyEqual(TEXT("fluid_ellipsoid == 1 for Ellipsoid"),
        (float)SpecGeom->fluid_ellipsoid, 1.0f, 1e-6f);

    G->FluidShape = EMjFluidShape::None;
    G->ExportTo(SpecGeom, nullptr);
    TestNearlyEqual(TEXT("fluid_ellipsoid == 0 for None"),
        (float)SpecGeom->fluid_ellipsoid, 0.0f, 1e-6f);

    mj_deleteSpec(TestSpec);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_GeomFluidCoef,
    "URLab.Import.RoundTrip_GeomFluidCoef",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_GeomFluidCoef::RunTest(const FString&)
{
    // mjsGeom.fluid_coefs[5] — schema attr "fluidcoef" needs the
    // attr_to_mjs_field rename to land in the right place.
    UMjGeom* G = NewObject<UMjGeom>();
    G->bOverride_fluidcoef = true;
    G->fluidcoef = { 0.5f, 0.25f, 1.5f, 1.0f, 1.0f };

    mjSpec* TestSpec = mj_makeSpec();
    mjsBody* World = mjs_findBody(TestSpec, "world");
    mjsGeom* SpecGeom = mjs_addGeom(World, nullptr);
    G->ExportTo(SpecGeom, nullptr);

    TestNearlyEqual(TEXT("fluid_coefs[0]"), (float)SpecGeom->fluid_coefs[0], 0.5f, 1e-6f);
    TestNearlyEqual(TEXT("fluid_coefs[1]"), (float)SpecGeom->fluid_coefs[1], 0.25f, 1e-6f);
    TestNearlyEqual(TEXT("fluid_coefs[2]"), (float)SpecGeom->fluid_coefs[2], 1.5f, 1e-6f);
    TestNearlyEqual(TEXT("fluid_coefs[3]"), (float)SpecGeom->fluid_coefs[3], 1.0f, 1e-6f);
    TestNearlyEqual(TEXT("fluid_coefs[4]"), (float)SpecGeom->fluid_coefs[4], 1.0f, 1e-6f);

    mj_deleteSpec(TestSpec);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_EqualitySiteMode,
    "URLab.Import.RoundTrip_EqualitySiteMode",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_EqualitySiteMode::RunTest(const FString&)
{
    // Connect equality referring to two sites (not bodies). Discriminated
    // by site1 UPROPERTY being non-empty -> objtype = mjOBJ_SITE.
    UMjEquality* Eq = NewObject<UMjEquality>();
    Eq->EqualityType = EMjEqualityType::Connect;
    Eq->site1 = TEXT("s1");
    Eq->site2 = TEXT("s2");
    Eq->Obj1 = TEXT("s1");   // populated by target_collation absorbs_attrs
    Eq->Obj2 = TEXT("s2");

    mjSpec* TestSpec = mj_makeSpec();
    mjsEquality* SpecEq = mjs_addEquality(TestSpec, nullptr);
    Eq->ExportTo(SpecEq);

    TestEqual(TEXT("type == connect"), (int)SpecEq->type, (int)mjtEq::mjEQ_CONNECT);
    TestEqual(TEXT("objtype == site (not body) when site1 non-empty"),
        (int)SpecEq->objtype, (int)mjOBJ_SITE);
    const char* n1 = mjs_getString(SpecEq->name1);
    const char* n2 = mjs_getString(SpecEq->name2);
    TestEqual(TEXT("name1 == 's1'"), FString(UTF8_TO_TCHAR(n1)), FString(TEXT("s1")));
    TestEqual(TEXT("name2 == 's2'"), FString(UTF8_TO_TCHAR(n2)), FString(TEXT("s2")));

    // And body-mode still works
    UMjEquality* EqB = NewObject<UMjEquality>();
    EqB->EqualityType = EMjEqualityType::Connect;
    EqB->Obj1 = TEXT("b1");
    EqB->Obj2 = TEXT("b2");
    mjsEquality* SpecEqB = mjs_addEquality(TestSpec, nullptr);
    EqB->ExportTo(SpecEqB);
    TestEqual(TEXT("body-mode objtype == body when site1 empty"),
        (int)SpecEqB->objtype, (int)mjOBJ_BODY);

    mj_deleteSpec(TestSpec);
    return true;
}
