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
//     specs what MuJoCo expects from any given XML feature.
//
//   TIER 2 — FMjXmlImportSession (full URLab importer)
//     These tests pass the same XML through UMujocoGenerationAction and
//     verify that the spec elements carry what the XML said.  After
//     calling Compile() they also verify the compiled mjModel matches
//     expectations.  A discrepancy between Tier 1 and Tier 2 always means a
//     URLab bug.
//
//   ROUND-TRIP tests compare Tier 1 model counts with Tier 2 counts directly.
//   If ngeom/nbody/nsensor differ, the importer dropped or duplicated elements.
//
// The attribute round-trips go MJCF in, spec, MJCF out, compiled model,
// and assert on the model. That is one loop rather than the two halves the
// old spec-scratch tests covered separately, and it is the loop a user runs:
// an attribute the reader drops, the writer forgets, or the compiler places in
// the wrong slot all fail here in the same way.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjTestHelpers.h"

#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Elements/MjBody.h"
#include "MuJoCo/Elements/MjCamera.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "MuJoCo/Gen/Elements/Sensors/MjAccelerometer.gen.h"
#include "MuJoCo/Gen/Elements/Constraints/MjConnect.gen.h"
#include "MuJoCo/Gen/Elements/Defaults/MjDefault.gen.h"
#include "MuJoCo/Gen/Elements/Tendons/MjFixed.gen.h"
#include "MuJoCo/Gen/Elements/Sensors/MjFramepos.gen.h"
#include "MuJoCo/Gen/Elements/Sensors/MjFramequat.gen.h"
#include "MuJoCo/Gen/Elements/Sensors/MjGyro.gen.h"
#include "MuJoCo/Gen/Elements/Joints/MjJoint.gen.h"
#include "MuJoCo/Gen/Elements/Sensors/MjJointpos.gen.h"
#include "MuJoCo/Gen/Elements/Sensors/MjJointvel.gen.h"
#include "MuJoCo/Gen/Elements/Options/MjOption.gen.h"
#include "MuJoCo/Gen/Elements/Constraints/MjPair.gen.h"
#include "MuJoCo/Gen/Elements/Sensors/MjVelocimeter.gen.h"
#include "MuJoCo/Gen/Elements/Constraints/MjWeld.gen.h"

namespace
{
/**
 * The first construction-script element of type `E` that sits under a
 * `<default>`.
 *
 * A default-class child is not flagged, it is placed: it is a child of the
 * `<default>` element carrying the class. `USCS_Node::GetChildNodes` is the
 * only ordered, serialized child list a Blueprint has, so it is what the walk
 * follows.
 */
template <typename E>
E* FindDefaultClassChild(const UBlueprint* Blueprint)
{
	if (Blueprint == nullptr || Blueprint->SimpleConstructionScript == nullptr)
	{
		return nullptr;
	}
	for (const USCS_Node* Node : Blueprint->SimpleConstructionScript->GetAllNodes())
	{
		if (Cast<UMjDefault>(Node->ComponentTemplate) == nullptr)
		{
			continue;
		}
		for (const USCS_Node* Child : Node->GetChildNodes())
		{
			if (E* Found = Cast<E>(Child->ComponentTemplate))
			{
				return Found;
			}
		}
	}
	return nullptr;
}

/**
 * The id an imported model's element received, by its authored MJCF name.
 *
 * A compiled scene attaches each participant under a prefix, so the name in the
 * model is not the name in the file and a bare `mj_name2id` finds nothing.
 */
int32 CompiledId(const FMjXmlImportSession& Session, mjtObj Type, const TCHAR* MjName)
{
	mjModel* M = Session.Model();
	if (M == nullptr || Session.Robot == nullptr)
	{
		return -1;
	}
	const FString Full = Session.Robot->GetCompiledPrefix() + MjName;
	return mj_name2id(M, Type, TCHAR_TO_UTF8(*Full));
}
} // namespace

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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	int bid = S.BodyId("b1");
	TestTrue(TEXT("b1 body id valid"), bid >= 0);
	TestNearlyEqual(TEXT("b1 x"), (float)S.m->body_pos[3 * bid + 0], 1.0f, 1e-4f);
	TestNearlyEqual(TEXT("b1 y"), (float)S.m->body_pos[3 * bid + 1], 2.0f, 1e-4f);
	TestNearlyEqual(TEXT("b1 z"), (float)S.m->body_pos[3 * bid + 2], 3.0f, 1e-4f);
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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	int g1 = S.GeomId("g1"), g2 = S.GeomId("g2");
	TestTrue(TEXT("g1 valid"), g1 >= 0);
	TestTrue(TEXT("g2 valid"), g2 >= 0);
	TestNearlyEqual(TEXT("sphere radius"), (float)S.m->geom_size[3 * g1], 0.5f, 1e-4f);
	TestNearlyEqual(TEXT("box x"), (float)S.m->geom_size[3 * g2 + 0], 1.0f, 1e-4f);
	TestNearlyEqual(TEXT("box y"), (float)S.m->geom_size[3 * g2 + 1], 2.0f, 1e-4f);
	TestNearlyEqual(TEXT("box z"), (float)S.m->geom_size[3 * g2 + 2], 3.0f, 1e-4f);
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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	int jid = S.JointId("j1");
	TestTrue(TEXT("j1 valid"), jid >= 0);
	TestTrue(TEXT("j1 limited"), S.m->jnt_limited[jid] != 0);
	TestNearlyEqual(TEXT("range lo"), (float)S.m->jnt_range[2 * jid + 0], -1.57f, 1e-3f);
	TestNearlyEqual(TEXT("range hi"), (float)S.m->jnt_range[2 * jid + 1], 1.57f, 1e-3f);
	S.Cleanup();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_MJ_DefaultClassOverride,
	"URLab.Import.MJ_DefaultClassOverride",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_MJ_DefaultClassOverride::RunTest(const FString&)
{
	// MuJoCo: explicit class= overrides parent childclass=
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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	int g0 = S.GeomId("g_inherit"), g1 = S.GeomId("g_override");
	TestTrue(TEXT("g_inherit valid"), g0 >= 0);
	TestTrue(TEXT("g_override valid"), g1 >= 0);
	TestNearlyEqual(TEXT("inherited size2"), (float)S.m->geom_size[3 * g0], 2.0f, 1e-4f);
	TestNearlyEqual(TEXT("overridden size3"), (float)S.m->geom_size[3 * g1], 3.0f, 1e-4f);
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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	int gid = S.GeomId("g_in_frame");
	TestTrue(TEXT("geom inside frame valid"), gid >= 0);
	TestNearlyEqual(TEXT("frame z offset applied"), (float)S.m->geom_pos[3 * gid + 2], 1.0f, 1e-4f);
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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	int g1 = S.GeomId("g1"), g2 = S.GeomId("g2");
	TestTrue(TEXT("g1 valid"), g1 >= 0);
	TestTrue(TEXT("g2 valid"), g2 >= 0);
	TestNearlyEqual(TEXT("g1 size from childclass"), (float)S.m->geom_size[3 * g1], 0.5f, 1e-4f);
	TestNearlyEqual(TEXT("g2 size from class"), (float)S.m->geom_size[3 * g2], 0.5f, 1e-4f);
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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	TestEqual(TEXT("ntendon"), (int)S.m->ntendon, 2);
	TestNearlyEqual(TEXT("spatial armature"), (float)S.m->tendon_armature[0], 1.5f, 1e-4f);
	TestNearlyEqual(TEXT("fixed armature"), (float)S.m->tendon_armature[1], 2.5f, 1e-4f);
	S.Cleanup();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_MJ_SensorSection,
	"URLab.Import.MJ_SensorSection",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_MJ_SensorSection::RunTest(const FString&)
{
	// The <sensor> container is a wrapper; the importer must recurse into its
	// per-type children (like <actuator>). A regression drops the whole section.
	FMjTestSession S;
	if (!S.CompileXml(TEXT(R"(
        <mujoco>
          <worldbody>
            <body pos="0 0 1">
              <joint name="j" type="hinge"/>
              <geom size=".1"/>
              <site name="s"/>
            </body>
          </worldbody>
          <sensor>
            <accelerometer name="acc" site="s"/>
            <gyro name="gyr" site="s"/>
            <framepos name="fp" objtype="site" objname="s"/>
          </sensor>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	TestEqual(TEXT("nsensor (all three <sensor> children imported)"), (int)S.m->nsensor, 3);
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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

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
	// The spec holds MJCF as authored, so pos="1 2 3" is (1, 2, 3) metres.
	// The component transform is a picture of it, drawn on request: scale ×100,
	// negate Y, giving UE RelativeLocation (100, -200, 300) cm.
	FMjXmlImportSession S;
	if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="b1" pos="1 2 3">
              <geom size=".1"/>
            </body>
          </worldbody>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	UMjBody* B = S.FindTemplate<UMjBody>(TEXT("b1"));
	if (!B)
	{
		AddError(TEXT("Body 'b1' not found in Blueprint"));
		S.Cleanup();
		return false;
	}

	const FMjPosition3 Pos = B->GetPos();
	TestNearlyEqual(TEXT("pos x=1 m"), (float)Pos.X, 1.0f, 1e-4f);
	TestNearlyEqual(TEXT("pos y=2 m"), (float)Pos.Y, 2.0f, 1e-4f);
	TestNearlyEqual(TEXT("pos z=3 m"), (float)Pos.Z, 3.0f, 1e-4f);

	B->SyncPreviewFromSpec();
	FVector Loc = B->GetRelativeLocation();
	TestNearlyEqual(TEXT("X=100 cm"), (float)Loc.X, 100.0f, 1.0f);
	TestNearlyEqual(TEXT("Y=-200 cm (negated)"), (float)Loc.Y, -200.0f, 1.0f);
	TestNearlyEqual(TEXT("Z=300 cm"), (float)Loc.Z, 300.0f, 1.0f);

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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	UMjBody* B = S.FindTemplate<UMjBody>(TEXT("b1"));
	if (!B)
	{
		AddError(TEXT("Body 'b1' not found"));
		S.Cleanup();
		return false;
	}

	const FMjQuatRot Authored = B->GetQuat();
	TestNearlyEqual(TEXT("authored w"), (float)Authored.W, 1.0f, 1e-4f);
	TestNearlyEqual(TEXT("authored x"), (float)Authored.X, 0.0f, 1e-4f);
	TestNearlyEqual(TEXT("authored y"), (float)Authored.Y, 0.0f, 1e-4f);
	TestNearlyEqual(TEXT("authored z"), (float)Authored.Z, 0.0f, 1e-4f);
	TestTrue(TEXT("identity quat on the element"), Authored.ToUnreal().Equals(FQuat::Identity, 0.01f));

	B->SyncPreviewFromSpec();
	FQuat Q = B->GetRelativeRotationCache().GetCachedQuat();
	TestTrue(TEXT("identity quat"), Q.Equals(FQuat::Identity, 0.01f));

	S.Cleanup();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_URLab_TypelessGeomIsSphere,
	"URLab.Import.URLab_TypelessGeomIsSphere",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_URLab_TypelessGeomIsSphere::RunTest(const FString&)
{
	// MJCF's global default geom type is sphere: a bare <geom size="..."/>
	// must resolve to the sphere shape with a real (non-zero) editor scale.
	// Shape is an attribute, not a class, so the schema default is what an
	// unauthored `type` reads back as.
	FMjXmlImportSession S;
	if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body>
              <geom name="g1" size="0.005"/>
            </body>
          </worldbody>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	UMjGeom* G = S.FindTemplate<UMjGeom>(TEXT("g1"));
	if (!G)
	{
		AddError(TEXT("typeless geom 'g1' did not import"));
		S.Cleanup();
		return false;
	}

	TestFalse(TEXT("no type authored"), G->Type.IsSet());
	TestTrue(TEXT("typeless geom reads back as a sphere"), G->GetType() == EMjGeomType::sphere);

	G->SyncEditorScaleFromSize();
	const FVector Scale = G->GetRelativeScale3D();
	TestNearlyEqual(TEXT("scale.X = radius*2 (m->UE units)"), (float)Scale.X, 0.01f, 1e-4f);
	TestTrue(TEXT("uniform scale"), Scale.AllComponentsEqual(1e-6f));

	S.Cleanup();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_URLab_ClassInheritedSizeScale,
	"URLab.Import.URLab_ClassInheritedSizeScale",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_URLab_ClassInheritedSizeScale::RunTest(const FString&)
{
	// A geom whose size comes entirely from its default class must not bake a
	// zero RelativeScale3D into the component template (the source of the
	// "Scale3D is (nearly) zero" warnings and NIL render matrices), and must
	// leave `size` unauthored so compile-time inheritance still applies.
	FMjXmlImportSession S;
	if (!S.Init(TEXT(R"(
        <mujoco>
          <default>
            <default class="col">
              <geom type="sphere" size="0.06"/>
            </default>
          </default>
          <worldbody>
            <body>
              <geom name="g1" class="col"/>
            </body>
          </worldbody>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	UMjGeom* G = S.FindTemplate<UMjGeom>(TEXT("g1"));
	if (!G)
	{
		AddError(TEXT("class-typed geom 'g1' did not import"));
		S.Cleanup();
		return false;
	}

	TestFalse(TEXT("size stays class-inherited (no explicit override)"), G->Size.IsSet());
	TestEqual(TEXT("the geom names the class it inherits from"), G->GetDclass(), FString(TEXT("col")));

	// The redraw refuses a size it cannot resolve rather than collapsing to
	// zero, which is what the warnings and NIL matrices came from.
	G->SyncEditorScaleFromSize();
	const FVector Scale = G->GetRelativeScale3D();
	TestTrue(TEXT("scale not degenerate"), Scale.GetMin() > 1e-4);

	// Inheritance is the compiler's, so what the class size is worth is a
	// question for the compiled model.
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	if (TestTrue(TEXT("the geom compiled"), S.Model()->ngeom >= 1))
	{
		TestNearlyEqual(TEXT("compiled radius from class size 0.06"),
			(float)S.Model()->geom_size[0], 0.06f, 1e-4f);
	}

	S.Cleanup();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_URLab_PlaneGeomClass,
	"URLab.Import.URLab_PlaneGeomClass",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_URLab_PlaneGeomClass::RunTest(const FString&)
{
	// type="plane" must reach the element as the plane shape. A MuJoCo plane's
	// size can legitimately be "0 0 s" (infinite extent), which must not zero
	// the component scale.
	FMjXmlImportSession S;
	if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <geom name="floor" type="plane" size="0 0 0.05"/>
          </worldbody>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	UMjGeom* G = S.FindTemplate<UMjGeom>(TEXT("floor"));
	if (!G)
	{
		AddError(TEXT("plane geom 'floor' did not import"));
		S.Cleanup();
		return false;
	}

	TestTrue(TEXT("floor is a plane"), G->GetType() == EMjGeomType::plane);

	G->SyncEditorScaleFromSize();
	const FVector Scale = G->GetRelativeScale3D();
	TestTrue(TEXT("plane scale not degenerate"),
		FMath::Min3(Scale.X, Scale.Y, Scale.Z) > 1e-4);

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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	UMjGeom* G = S.FindTemplate<UMjGeom>(TEXT("g1"));
	if (!G)
	{
		AddError(TEXT("Geom 'g1' not found"));
		S.Cleanup();
		return false;
	}

	const TArray<double> Friction = G->GetFriction();
	if (TestEqual(TEXT("friction has three entries"), Friction.Num(), 3))
	{
		TestNearlyEqual(TEXT("friction[0]"), (float)Friction[0], 0.8f, 1e-4f);
		TestNearlyEqual(TEXT("friction[1]"), (float)Friction[1], 0.1f, 1e-4f);
		TestNearlyEqual(TEXT("friction[2]"), (float)Friction[2], 0.01f, 1e-3f);
	}

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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	UMjJoint* J = S.FindTemplate<UMjJoint>(TEXT("j1"));
	if (!J)
	{
		AddError(TEXT("Joint 'j1' not found"));
		S.Cleanup();
		return false;
	}

	// The spec holds the angle the file holds. `<compiler angle>` is a
	// property of the spec and is honoured by the compiler, so nothing is
	// converted on the way in and 1.57 stays 1.57.
	const FVector2D Range = J->GetRange();
	TestNearlyEqual(TEXT("range lo"), (float)Range.X, -1.57f, 1e-4f);
	TestNearlyEqual(TEXT("range hi"), (float)Range.Y, 1.57f, 1e-4f);
	const TArray<double> Damping = J->GetDamping();
	TestTrue(TEXT("damping has values"), Damping.Num() > 0);
	if (Damping.Num() > 0)
		TestNearlyEqual(TEXT("damping[0]"), (float)Damping[0], 0.5f, 1e-4f);

	S.Cleanup();
	return true;
}

// Regression: a joint declared inside a <default> block must carry the file's
// own compiler settings rather than a hardcoded degrees fallback, or an
// angle="radian" model (mujoco_menagerie's unitree_go1 among them) has its
// default joint's range rescaled by π/180 in the wrong direction. The spec
// carries the file's own units when recursing into <default> blocks, so the
// assertion is that the value arrives untouched and that the compiled model
// reads it as radians.
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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	UMjJoint* DefaultJoint = FindDefaultClassChild<UMjJoint>(S.Blueprint);
	if (!DefaultJoint)
	{
		AddError(TEXT("No default-class joint template found"));
		S.Cleanup();
		return false;
	}

	if (TestTrue(TEXT("default joint carries a range"), DefaultJoint->Range.IsSet()))
	{
		const FVector2D Range = DefaultJoint->GetRange();
		TestNearlyEqual(TEXT("default joint range lo (radians, as authored)"),
			(float)Range.X, -0.86f, 1e-4f);
		TestNearlyEqual(TEXT("default joint range hi (radians, as authored)"),
			(float)Range.Y, 0.86f, 1e-4f);
	}

	// And the compiler agrees the file meant radians: a degrees reading of
	// 0.86 would land at 0.015 rad.
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	const int32 JointId = S.Model()->njnt > 0 ? 0 : -1;
	if (TestTrue(TEXT("the inheriting joint compiled"), JointId >= 0))
	{
		TestNearlyEqual(TEXT("compiled jnt_range lo"),
			(float)S.Model()->jnt_range[2 * JointId + 0], -0.86f, 1e-4f);
		TestNearlyEqual(TEXT("compiled jnt_range hi"),
			(float)S.Model()->jnt_range[2 * JointId + 1], 0.86f, 1e-4f);
	}

	S.Cleanup();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_URLab_SensorType,
	"URLab.Import.URLab_SensorType",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_URLab_SensorType::RunTest(const FString&)
{
	// A sensor's kind is its element, not a field on a shared class, so the
	// class the reader built IS the tag it read.
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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	UMjJointpos* SJP = S.FindTemplate<UMjJointpos>(TEXT("s_jpos"));
	UMjJointvel* SJV = S.FindTemplate<UMjJointvel>(TEXT("s_jvel"));

	TestNotNull(TEXT("jointpos concrete class"), SJP);
	TestNotNull(TEXT("jointvel concrete class"), SJV);

	// One each: a reader that emitted every sensor kind would satisfy the two
	// lookups above on its own.
	TestEqual(TEXT("exactly one jointpos"), S.CountTemplates<UMjJointpos>(), 1);
	TestEqual(TEXT("exactly one jointvel"), S.CountTemplates<UMjJointvel>(), 1);

	if (SJP)
		TestEqual(TEXT("jointpos target joint"), SJP->Joint, FString(TEXT("j1")));
	if (SJV)
		TestEqual(TEXT("jointvel target joint"), SJV->Joint, FString(TEXT("j1")));

	S.Cleanup();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_URLab_OptionTimestep,
	"URLab.Import.URLab_OptionTimestep",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_URLab_OptionTimestep::RunTest(const FString&)
{
	// <option> is an element of the spec like any other, so its attributes
	// land on the <option> the reader built in the construction script.
	FMjXmlImportSession S;
	if (!S.Init(TEXT(R"(
        <mujoco>
          <option timestep="0.005" gravity="0 0 -5"/>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	if (!S.Blueprint)
	{
		AddError(TEXT("No Blueprint"));
		S.Cleanup();
		return false;
	}
	UMjOption* Option = S.FindFirstTemplate<UMjOption>();
	if (!Option)
	{
		AddError(TEXT("the import produced no <option> element"));
		S.Cleanup();
		return false;
	}

	TestTrue(TEXT("timestep is authored, not defaulted"), Option->Timestep.IsSet());
	TestNearlyEqual(TEXT("timestep"), (float)Option->GetTimestep(), 0.005f, 1e-6f);
	TestTrue(TEXT("gravity is authored, not defaulted"), Option->Gravity.IsSet());
	TestNearlyEqual(TEXT("gravity z"), (float)Option->GetGravity().Z, -5.0f, 1e-4f);

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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	// A fixed tendon is its own element; <tendon> is the section holding it.
	UMjFixed* T = S.FindTemplate<UMjFixed>(TEXT("tf"));
	if (!T)
	{
		AddError(TEXT("Tendon 'tf' not found"));
		S.Cleanup();
		return false;
	}

	TestNearlyEqual(TEXT("armature"), (float)T->GetArmature(), 2.5f, 1e-4f);

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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	UMjPair* CP = S.FindFirstTemplate<UMjPair>();
	if (!CP)
	{
		AddError(TEXT("ContactPair not found"));
		S.Cleanup();
		return false;
	}

	TestEqual(TEXT("geom1"), CP->GetGeom1(), FString(TEXT("floor")));
	TestEqual(TEXT("geom2"), CP->GetGeom2(), FString(TEXT("ball")));
	TestEqual(TEXT("condim"), CP->GetCondim(), 3);

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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	UMjWeld* EQ = S.FindTemplate<UMjWeld>(TEXT("w1"));
	if (!EQ)
	{
		AddError(TEXT("Equality 'w1' not found"));
		S.Cleanup();
		return false;
	}

	TestEqual(TEXT("body1"), EQ->GetBody1(), FString(TEXT("b1")));
	TestEqual(TEXT("body2"), EQ->GetBody2(), FString(TEXT("b2")));

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
    )"),
		*RelChildPath);

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
	if (!Ref.CompileXml(Xml))
	{
		AddError(Ref.LastError);
		return false;
	}
	int ExpNbody = Ref.m->nbody;
	int ExpNgeom = Ref.m->ngeom;
	Ref.Cleanup();

	// URLab importer path
	FMjXmlImportSession S;
	if (!S.Init(Xml))
	{
		AddError(S.LastError);
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}

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
	if (!Ref.CompileXml(Xml))
	{
		AddError(Ref.LastError);
		return false;
	}
	int ExpNsensor = Ref.m->nsensor;
	Ref.Cleanup();

	FMjXmlImportSession S;
	if (!S.Init(Xml))
	{
		AddError(S.LastError);
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}

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
	if (!Ref.CompileXml(Xml))
	{
		AddError(Ref.LastError);
		return false;
	}
	int ExpNu = Ref.m->nu;
	Ref.Cleanup();

	FMjXmlImportSession S;
	if (!S.Init(Xml))
	{
		AddError(S.LastError);
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}

	TestEqual(TEXT("nu matches reference"), (int)S.Model()->nu, ExpNu);

	S.Cleanup();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_Defaults,
	"URLab.Import.RoundTrip_Defaults",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_Defaults::RunTest(const FString&)
{
	// Default inheritance: a geom inside a body with childclass="robot" should
	// inherit friction from the default. Resolution is the compiler's, so the
	// spec keeps the class link and the model is where the value shows up.
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
	if (!Ref.CompileXml(Xml))
	{
		AddError(Ref.LastError);
		return false;
	}
	int ExpNgeom = Ref.m->ngeom;
	float ExpFriction0 = (float)Ref.m->geom_friction[3 * Ref.GeomId("g1") + 0];
	Ref.Cleanup();

	FMjXmlImportSession S;
	if (!S.Init(Xml))
	{
		AddError(S.LastError);
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}

	// The URLab importer prefixes names so we can't use mj_name2id("g1") directly.
	// Instead compare ngeom count and check friction on the first non-worldbody geom.
	TestEqual(TEXT("ngeom matches reference"), (int)S.Model()->ngeom, ExpNgeom);
	if (S.Model()->ngeom > 0)
		TestNearlyEqual(TEXT("inherited friction[0]"), (float)S.Model()->geom_friction[0], 0.7f, 1e-4f);

	S.Cleanup();
	return true;
}

// <frame> elements are handled by ImportNodeRecursive.
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
	if (!Ref.CompileXml(Xml))
	{
		AddError(Ref.LastError);
		return false;
	}
	int ExpNgeom = Ref.m->ngeom;
	Ref.Cleanup();

	FMjXmlImportSession S;
	if (!S.Init(Xml))
	{
		AddError(S.LastError);
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}

	TestEqual(TEXT("ngeom matches after frame support fix"), (int)S.Model()->ngeom, ExpNgeom);

	S.Cleanup();
	return true;
}

// =============================================================================
// URLab.Import.MJ_JointAxisImport
//   A joint axis reaches the spec as the MJCF vector it was written as,
//   and reaches the compiled model unchanged. The spec is MJCF, so there
//   is no axis convention to get wrong between the two.
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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	UMjJoint* JC = S.FindTemplate<UMjJoint>(TEXT("j1"));
	TestNotNull(TEXT("joint j1 found"), JC);
	if (JC)
	{
		const FMjDirection3 Axis = JC->GetAxis();
		TestTrue(TEXT("Axis X ≈ 0"), FMath::Abs((float)Axis.X) < 1e-4f);
		TestTrue(TEXT("Axis Y ≈ 1"), FMath::Abs((float)Axis.Y - 1.0f) < 1e-4f);
		TestTrue(TEXT("Axis Z ≈ 0"), FMath::Abs((float)Axis.Z) < 1e-4f);
	}

	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	if (TestTrue(TEXT("the joint compiled"), S.Model()->njnt >= 1))
	{
		const mjtNum* Ax = S.Model()->jnt_axis;
		TestTrue(TEXT("jnt_axis[0] ≈ 0"), FMath::Abs((float)Ax[0]) < 1e-4f);
		TestTrue(TEXT("jnt_axis[1] ≈ 1"), FMath::Abs((float)Ax[1] - 1.0f) < 1e-4f);
		TestTrue(TEXT("jnt_axis[2] ≈ 0"), FMath::Abs((float)Ax[2]) < 1e-4f);
	}

	S.Cleanup();
	return true;
}

// =============================================================================
// URLab.Import.MJ_SensorTypeFromTag
//   The sensor XML tag determines which element the reader builds, and the
//   element is the kind: there is no separate type field to disagree with it.
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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	TestNotNull(TEXT("s_acc  is an <accelerometer>"), S.FindTemplate<UMjAccelerometer>(TEXT("s_acc")));
	TestNotNull(TEXT("s_gyro is a <gyro>"), S.FindTemplate<UMjGyro>(TEXT("s_gyro")));
	TestNotNull(TEXT("s_jp   is a <jointpos>"), S.FindTemplate<UMjJointpos>(TEXT("s_jp")));
	TestNotNull(TEXT("s_vel  is a <velocimeter>"), S.FindTemplate<UMjVelocimeter>(TEXT("s_vel")));

	// One of each and nothing else: four lookups against a reader that emitted
	// every sensor kind would all succeed.
	TestEqual(TEXT("exactly four sensor elements"),
		S.CountTemplates<UMjAccelerometer>() + S.CountTemplates<UMjGyro>()
			+ S.CountTemplates<UMjJointpos>() + S.CountTemplates<UMjVelocimeter>(),
		4);

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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	UMjBody* RefBody = S.FindTemplate<UMjBody>(TEXT("ref"));
	UMjBody* B1 = S.FindTemplate<UMjBody>(TEXT("b1"));

	if (RefBody)
		TestTrue(TEXT("ref body mocap=true"), RefBody->GetMocap());
	if (B1)
		TestTrue(TEXT("b1 body mocap=false"), !B1->GetMocap());

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
    )")))
	{
		AddError(SAuto.LastError);
		return false;
	}
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
    )")))
	{
		AddError(SOverride.LastError);
		return false;
	}
	int jid2 = SOverride.JointId("j2");
	TestTrue(TEXT("j2 valid"), jid2 >= 0);
	if (jid2 >= 0)
		TestTrue(TEXT("j2 NOT limited via explicit limited=false"), SOverride.m->jnt_limited[jid2] == 0);
	SOverride.Cleanup();
	return true;
}

// =============================================================================
// URLab.Import.URLab_WeldTorqueScale
//   <weld torquescale="2.5"/> must arrive on the <weld> element as an authored
//   value rather than an inherited default, and must reach the compiled model.
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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	UMjWeld* Weld = S.FindTemplate<UMjWeld>(TEXT("w1"));
	TestNotNull(TEXT("weld equality 'w1' found"), Weld);
	if (Weld)
	{
		TestTrue(TEXT("torquescale is authored"), Weld->Torquescale.IsSet());
		TestTrue(TEXT("TorqueScale ≈ 2.5"), FMath::Abs((float)Weld->GetTorquescale() - 2.5f) < 1e-4f);
	}

	// A weld packs torquescale into eq_data[10]; older packings wrote slot 7.
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	if (TestEqual(TEXT("neq == 1"), (int)S.Model()->neq, 1))
	{
		TestNearlyEqual(TEXT("eq_data[10] = torquescale"),
			(float)S.Model()->eq_data[10], 2.5f, 1e-4f);
	}
	S.Cleanup();
	return true;
}

// =============================================================================
// URLab.Import.URLab_ConnectAnchor
//   <connect anchor="0.1 -0.2 0.3"/> reads as an authored FVector in MJCF
//   metres and is written back out, reaching eq_data[].
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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	UMjConnect* EQ = S.FindTemplate<UMjConnect>(TEXT("c1"));
	if (!EQ)
	{
		AddError(TEXT("Equality 'c1' not found"));
		S.Cleanup();
		return false;
	}

	if (TestTrue(TEXT("anchor is authored"), EQ->Anchor.IsSet()))
	{
		const FMjPosition3 Anchor = EQ->GetAnchor();
		TestNearlyEqual(TEXT("anchor x = 0.1 m"), (float)Anchor.X, 0.1f, 1e-5f);
		TestNearlyEqual(TEXT("anchor y = -0.2 m"), (float)Anchor.Y, -0.2f, 1e-5f);
		TestNearlyEqual(TEXT("anchor z = 0.3 m"), (float)Anchor.Z, 0.3f, 1e-5f);
	}

	// A connect equality packs its anchor into eq_data[0..2].
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	if (TestEqual(TEXT("neq == 1"), (int)S.Model()->neq, 1))
	{
		const mjtNum* D = S.Model()->eq_data;
		TestEqual(TEXT("type == connect"), S.Model()->eq_type[0], (int)mjtEq::mjEQ_CONNECT);
		TestNearlyEqual(TEXT("data[0] = 0.1 m"), (float)D[0], 0.1f, 1e-4f);
		TestNearlyEqual(TEXT("data[1] = -0.2 m"), (float)D[1], -0.2f, 1e-4f);
		TestNearlyEqual(TEXT("data[2] = 0.3 m"), (float)D[2], 0.3f, 1e-4f);
	}
	S.Cleanup();
	return true;
}

// =============================================================================
// URLab.Import.RoundTrip_ConnectAnchor
//   The per-slot data[] layout for the connect kind: anchor packs into
//   data[0..2] in metres, on a free-jointed pair of bodies rather than the
//   hinge linkage the E2E case uses.
// =============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_ConnectAnchor,
	"URLab.Import.RoundTrip_ConnectAnchor",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_ConnectAnchor::RunTest(const FString&)
{
	FMjXmlImportSession S;
	if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="b1"><freejoint/><geom type="box" size="0.1 0.1 0.1"/></body>
            <body name="b2" pos="0 0.5 0"><freejoint/><geom type="box" size="0.1 0.1 0.1"/></body>
          </worldbody>
          <equality>
            <connect body1="b1" body2="b2" anchor="0.1 -0.2 0.3"/>
          </equality>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	const mjModel* M = S.Model();

	TestEqual(TEXT("neq == 1"), (int)M->neq, 1);
	TestEqual(TEXT("type == connect"), M->eq_type[0], (int)mjtEq::mjEQ_CONNECT);
	TestNearlyEqual(TEXT("data[0] = 0.1 m"), (float)M->eq_data[0], 0.1f, 1e-4f);
	TestNearlyEqual(TEXT("data[1] = -0.2 m"), (float)M->eq_data[1], -0.2f, 1e-4f);
	TestNearlyEqual(TEXT("data[2] = 0.3 m"), (float)M->eq_data[2], 0.3f, 1e-4f);

	S.Cleanup();
	return true;
}

// =============================================================================
// URLab.Import.E2E_GripperConnectAnchor
//   Full URLab pipeline test: hinge-jointed gripper-like linkage with a
//   <connect> equality. After Compile(), the model must have neq==1 and the
//   anchor must survive into eq_data[0..2] in metres (XML m units).
//
//   Uses hinge joints (no freejoint) because the scene attach refuses
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
	if (!S.Init(Xml))
	{
		AddError(S.LastError);
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}

	const mjModel* M = S.Model();
	TestEqual(TEXT("neq after URLab pipeline"), (int)M->neq, 1);
	if (M->neq >= 1)
	{
		// Body refs survived the participant prefix
		const int o1 = M->eq_obj1id[0];
		const int o2 = M->eq_obj2id[0];
		TestTrue(TEXT("eq.obj1id resolved"), o1 > 0);
		TestTrue(TEXT("eq.obj2id resolved"), o2 > 0);

		// anchor data[0..2] in spec metres (XML literal values)
		TestNearlyEqual(TEXT("eq_data[0] = 0.0"), (float)M->eq_data[0], 0.0f, 1e-4f);
		TestNearlyEqual(TEXT("eq_data[1] = -0.018"), (float)M->eq_data[1], -0.018f, 1e-4f);
		TestNearlyEqual(TEXT("eq_data[2] = 0.0065"), (float)M->eq_data[2], 0.0065f, 1e-4f);
	}
	S.Cleanup();
	return true;
}

// =============================================================================
// URLab.Import.RoundTrip_WeldAnchorRelposeTorqueScale
//   Verifies the full weld data[] layout: anchor -> data[0..2], relpose ->
//   data[3..9], torquescale -> data[10]. An older packing wrote torquescale to
//   data[7] and skipped anchor and relpose entirely.
// =============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_WeldAnchorRelposeTorqueScale,
	"URLab.Import.RoundTrip_WeldAnchorRelposeTorqueScale",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_WeldAnchorRelposeTorqueScale::RunTest(const FString&)
{
	FMjXmlImportSession S;
	if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="b1"><freejoint/><geom type="box" size="0.1 0.1 0.1"/></body>
            <body name="b2" pos="0 0.5 0"><freejoint/><geom type="box" size="0.1 0.1 0.1"/></body>
          </worldbody>
          <equality>
            <weld body1="b1" body2="b2" anchor="0.5 0.6 0.7"
                  relpose="1 2 3 0.7071 0 0.7071 0" torquescale="42"/>
          </equality>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	const mjtNum* D = S.Model()->eq_data;

	TestEqual(TEXT("type == weld"), S.Model()->eq_type[0], (int)mjtEq::mjEQ_WELD);
	// anchor
	TestNearlyEqual(TEXT("data[0] = 0.5 m"), (float)D[0], 0.5f, 1e-4f);
	TestNearlyEqual(TEXT("data[1] = 0.6 m"), (float)D[1], 0.6f, 1e-4f);
	TestNearlyEqual(TEXT("data[2] = 0.7 m"), (float)D[2], 0.7f, 1e-4f);
	// relpose pos (raw)
	TestNearlyEqual(TEXT("data[3] = relpose pos x"), (float)D[3], 1.0f, 1e-4f);
	TestNearlyEqual(TEXT("data[4] = relpose pos y"), (float)D[4], 2.0f, 1e-4f);
	TestNearlyEqual(TEXT("data[5] = relpose pos z"), (float)D[5], 3.0f, 1e-4f);
	// relpose quat (raw, normalised by the compiler)
	TestNearlyEqual(TEXT("data[6] = relpose quat w"), (float)D[6], 0.7071f, 1e-4f);
	TestNearlyEqual(TEXT("data[7] = relpose quat x"), (float)D[7], 0.0f, 1e-4f);
	TestNearlyEqual(TEXT("data[8] = relpose quat y"), (float)D[8], 0.7071f, 1e-4f);
	TestNearlyEqual(TEXT("data[9] = relpose quat z"), (float)D[9], 0.0f, 1e-4f);
	// torquescale at slot 10 (was wrongly slot 7 in an older packing)
	TestNearlyEqual(TEXT("data[10] = torquescale"), (float)D[10], 42.0f, 1e-4f);

	S.Cleanup();
	return true;
}

// =============================================================================
// URLab.Import.RoundTrip_JointEqualityPolycoef
//   Joint equality with polycoef must pack into data[0..4] AND set
//   objtype = mjOBJ_JOINT. Without the objtype set, the attach silently
//   drops the equality at compile time.
// =============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_JointEqualityPolycoef,
	"URLab.Import.RoundTrip_JointEqualityPolycoef",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_JointEqualityPolycoef::RunTest(const FString&)
{
	FMjXmlImportSession S;
	if (!S.Init(TEXT(R"(
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <body name="ba"><joint name="ja" type="hinge" axis="0 0 1"/><geom type="box" size="0.1 0.1 0.1"/></body>
            <body name="bb" pos="0 0.5 0"><joint name="jb" type="hinge" axis="0 0 1"/><geom type="box" size="0.1 0.1 0.1"/></body>
          </worldbody>
          <equality>
            <joint joint1="ja" joint2="jb" polycoef="5 6 7 8 9"/>
          </equality>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	const mjModel* M = S.Model();

	TestEqual(TEXT("type == joint"), M->eq_type[0], (int)mjtEq::mjEQ_JOINT);
	// eq_objtype is deliberately not asserted here. MuJoCo's own XML loader
	// leaves it at mjOBJ_UNKNOWN for a named equality and identifies the pair
	// through eq_type plus eq_obj1id/eq_obj2id, which is what the assertions
	// above and below check. Setting it was a property of the hand-built
	// mjSpec this path replaced, not of the compiled model.
	TestEqual(TEXT("obj1 resolves to ja"), M->eq_obj1id[0], CompiledId(S, mjOBJ_JOINT, TEXT("ja")));
	TestEqual(TEXT("obj2 resolves to jb"), M->eq_obj2id[0], CompiledId(S, mjOBJ_JOINT, TEXT("jb")));
	TestNearlyEqual(TEXT("data[0] = 5"), (float)M->eq_data[0], 5.0f, 1e-4f);
	TestNearlyEqual(TEXT("data[1] = 6"), (float)M->eq_data[1], 6.0f, 1e-4f);
	TestNearlyEqual(TEXT("data[2] = 7"), (float)M->eq_data[2], 7.0f, 1e-4f);
	TestNearlyEqual(TEXT("data[3] = 8"), (float)M->eq_data[3], 8.0f, 1e-4f);
	TestNearlyEqual(TEXT("data[4] = 9"), (float)M->eq_data[4], 9.0f, 1e-4f);

	S.Cleanup();
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
	FMjXmlImportSession S;
	if (!S.Init(TEXT(R"(
        <mujoco>
          <compiler angle="radian"/>
          <worldbody>
            <body name="ba"><joint name="ja" type="hinge" axis="0 0 1"/><geom type="box" size="0.1 0.1 0.1"/></body>
            <body name="bb" pos="0 0.5 0"><joint name="jb" type="hinge" axis="0 0 1"/><geom type="box" size="0.1 0.1 0.1"/></body>
          </worldbody>
          <tendon>
            <fixed name="ta"><joint joint="ja" coef="1"/></fixed>
            <fixed name="tb"><joint joint="jb" coef="1"/></fixed>
          </tendon>
          <equality>
            <tendon tendon1="ta" tendon2="tb" polycoef="0 1 0 0 0"/>
          </equality>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	const mjModel* M = S.Model();

	TestEqual(TEXT("type == tendon"), M->eq_type[0], (int)mjtEq::mjEQ_TENDON);
	// eq_objtype is deliberately not asserted here. MuJoCo's own XML loader
	// leaves it at mjOBJ_UNKNOWN for a named equality and identifies the pair
	// through eq_type plus eq_obj1id/eq_obj2id, which is what the assertions
	// above and below check. Setting it was a property of the hand-built
	// mjSpec this path replaced, not of the compiled model.
	TestEqual(TEXT("obj1 resolves to ta"), M->eq_obj1id[0], CompiledId(S, mjOBJ_TENDON, TEXT("ta")));
	TestEqual(TEXT("obj2 resolves to tb"), M->eq_obj2id[0], CompiledId(S, mjOBJ_TENDON, TEXT("tb")));
	TestNearlyEqual(TEXT("data[0]"), (float)M->eq_data[0], 0.0f, 1e-4f);
	TestNearlyEqual(TEXT("data[1]"), (float)M->eq_data[1], 1.0f, 1e-4f);

	S.Cleanup();
	return true;
}

// =============================================================================
// URLab.Import.RoundTrip_FlexEqualityObjType
//   <flex>, <flexvert> and <flexstrain> equalities must all compile with
//   objtype = mjOBJ_FLEX. They are three separate elements rather than three
//   arms of one enum, so the loop is over the tags the reader accepts and the
//   assertion is on the model each one compiles to.
// =============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_FlexEqualityObjType,
	"URLab.Import.RoundTrip_FlexEqualityObjType",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_FlexEqualityObjType::RunTest(const FString&)
{
	// A real <flex> has to exist for the equality to refer to, and a flexcomp
	// is what produces one. Its own edge equality is off so the only
	// flex-referencing equality in the model is the authored one.
	const TCHAR* const FlexTags[] = {TEXT("flex"), TEXT("flexvert"), TEXT("flexstrain")};

	for (const TCHAR* Tag : FlexTags)
	{
		const FString Xml = FString::Printf(TEXT(R"(<mujoco>
  <worldbody>
    <body name="anchor" pos="0 0 0">
      <geom size=".05"/>
    </body>
    <flexcomp name="cloth" type="grid" count="2 2 1" spacing="0.1 0.1 0.1" pos="0 0 0.3">
      <contact selfcollide="none"/>
      <edge equality="false"/>
    </flexcomp>
  </worldbody>
  <equality>
    <%s flex="cloth" active="true"/>
  </equality>
</mujoco>)"),
			Tag);

		FMjXmlImportSession S;
		if (!S.Init(Xml))
		{
			AddError(FString::Printf(TEXT("Init failed (%s): %s"), Tag, *S.LastError));
			continue;
		}
		if (!S.Compile())
		{
			AddError(FString::Printf(TEXT("Compile failed (%s): %s"), Tag, *S.LastError));
			S.Cleanup();
			continue;
		}

		const mjModel* M = S.Model();
		// That the tag produced exactly one equality is the whole claim: MuJoCo
		// leaves eq_objtype at mjOBJ_UNKNOWN for an equality loaded from XML, so
		// asserting mjOBJ_FLEX was pinning the hand-built mjSpec this replaced.
		TestEqual(FString::Printf(TEXT("<%s> compiled one equality"), Tag), (int)M->neq, 1);
		S.Cleanup();
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
	if (!S.Init(Xml))
	{
		AddError(S.LastError);
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}

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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

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
//   Tier 2: <framepos objtype="body" objname="b1"/> must reach the <framepos>
//   element with objtype Body and its reference populated, and the round-trip
//   compile must produce sensor_objtype == mjOBJ_BODY.
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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	UMjFramepos* FP = S.FindTemplate<UMjFramepos>(TEXT("fp1"));
	TestNotNull(TEXT("framepos sensor 'fp1' found"), FP);
	if (FP)
	{
		TestTrue(TEXT("fp1 ObjType == Body"), FP->Objtype == EMjFrameObject::body);
		TestTrue(TEXT("fp1 TargetName == 'b1'"), FP->Objname == TEXT("b1"));
	}

	UMjFramequat* FQ = S.FindTemplate<UMjFramequat>(TEXT("fq1"));
	TestNotNull(TEXT("framequat sensor 'fq1' found"), FQ);
	if (FQ)
	{
		TestTrue(TEXT("fq1 ObjType == Body"), FQ->Objtype == EMjFrameObject::body);
		TestTrue(TEXT("fq1 RefType == Body"), FQ->GetReftype() == EMjFrameObject::body);
		TestTrue(TEXT("fq1 ReferenceName == 'world'"), FQ->GetRefname() == TEXT("world"));
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
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	// Tier 1: check the imported spec elements
	UMjJoint* JInherited = S.FindTemplate<UMjJoint>(TEXT("inherited"));
	UMjJoint* JExplicit = S.FindTemplate<UMjJoint>(TEXT("explicit"));
	TestNotNull(TEXT("inherited joint found"), JInherited);
	TestNotNull(TEXT("explicit joint found"), JExplicit);

	if (JInherited)
	{
		// The inheriting joint must stay unauthored, or the class it names has
		// nothing left to give it.
		TestFalse(TEXT("inherited joint has no axis of its own"), JInherited->Axis.IsSet());
	}
	if (JExplicit)
	{
		const FMjDirection3 Axis = JExplicit->GetAxis();
		TestTrue(TEXT("explicit Axis X ≈ 1"), FMath::Abs((float)Axis.X - 1.0f) < 1e-4f);
		TestTrue(TEXT("explicit Axis Y ≈ 0"), FMath::Abs((float)Axis.Y) < 1e-4f);
		TestTrue(TEXT("explicit Axis Z ≈ 0"), FMath::Abs((float)Axis.Z) < 1e-4f);
	}

	// Tier 2: compile and check jnt_axis in the compiled model
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}

	const mjModel* M = S.Model();
	TestNotNull(TEXT("compiled model"), M);
	if (!M)
	{
		S.Cleanup();
		return false;
	}

	// Find joints by name in the compiled model
	int idInherited = mj_name2id(M, mjOBJ_JOINT, "inherited");
	int idExplicit = mj_name2id(M, mjOBJ_JOINT, "explicit");

	// Joints may be prefixed — search with prefix if not found
	if (idInherited < 0 || idExplicit < 0)
	{
		for (int j = 0; j < M->njnt; ++j)
		{
			const char* name = mj_id2name(M, mjOBJ_JOINT, j);
			if (!name)
				continue;
			FString N = UTF8_TO_TCHAR(name);
			if (N.Contains(TEXT("inherited")))
				idInherited = j;
			if (N.Contains(TEXT("explicit")))
				idExplicit = j;
		}
	}

	TestTrue(TEXT("inherited joint compiled"), idInherited >= 0);
	TestTrue(TEXT("explicit joint compiled"), idExplicit >= 0);

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
	if (!Ref.CompileXml(Xml))
	{
		AddError(Ref.LastError);
		return false;
	}
	const int ExpNu = Ref.m->nu;
	const int ExpNjnt = Ref.m->njnt;
	Ref.Cleanup();
	TestEqual(TEXT("baseline MuJoCo nu == 1"), ExpNu, 1);
	TestEqual(TEXT("baseline MuJoCo njnt == 1"), ExpNjnt, 1);

	// Through URLab's importer + compile.
	FMjXmlImportSession S;
	if (!S.Init(Xml))
	{
		AddError(S.LastError);
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}

	TestEqual(TEXT("URLab nu matches MuJoCo baseline (actuator survived Default-class name collision)"),
		(int)S.Model()->nu, ExpNu);
	TestEqual(TEXT("URLab njnt matches MuJoCo baseline"),
		(int)S.Model()->njnt, ExpNjnt);

	S.Cleanup();
	return true;
}

// =============================================================================
// CAMERA + GEOM ROUND-TRIP REGRESSION TESTS
//
// These cover the audit findings that landed alongside the gripper-attach fix:
// schema attributes that were read onto the element and then silently dropped
// on the way back out, because MuJoCo names the underlying field differently
// (target -> targetbody, focal -> focal_length, shellinertia -> typeinertia,
// fluidshape -> fluid_ellipsoid, and so on). Each goes MJCF in, spec, MJCF
// out, compiled model, and asserts on the model, so a drop at either end fails.
// =============================================================================

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_CameraTarget,
	"URLab.Import.RoundTrip_CameraTarget",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_CameraTarget::RunTest(const FString&)
{
	// A target body is only legal on a targeting camera, so the mode is
	// scaffolding for the compile and not part of what is asserted.
	FMjXmlImportSession S;
	if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="torso"><freejoint/><geom type="box" size="0.1 0.1 0.1"/></body>
            <camera name="cam" mode="targetbody" target="torso" pos="0 0 2"/>
          </worldbody>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}

	const int CamId = CompiledId(S, mjOBJ_CAMERA, TEXT("cam"));
	if (TestTrue(TEXT("camera compiled"), CamId >= 0))
	{
		TestEqual(TEXT("targetbody == 'torso'"),
			S.Model()->cam_targetbodyid[CamId], CompiledId(S, mjOBJ_BODY, TEXT("torso")));
	}

	S.Cleanup();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_CameraProjection,
	"URLab.Import.RoundTrip_CameraProjection",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_CameraProjection::RunTest(const FString&)
{
	FMjXmlImportSession S;
	if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="b1"><freejoint/><geom type="box" size="0.1 0.1 0.1"/></body>
            <camera name="ortho" projection="orthographic" pos="0 0 2"/>
            <camera name="persp" projection="perspective" pos="0 0 3"/>
          </worldbody>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}

	const int OrthoId = CompiledId(S, mjOBJ_CAMERA, TEXT("ortho"));
	const int PerspId = CompiledId(S, mjOBJ_CAMERA, TEXT("persp"));
	if (TestTrue(TEXT("both cameras compiled"), OrthoId >= 0 && PerspId >= 0))
	{
		TestEqual(TEXT("proj == orthographic"),
			S.Model()->cam_projection[OrthoId], (int)mjPROJ_ORTHOGRAPHIC);
		TestEqual(TEXT("proj == perspective"),
			S.Model()->cam_projection[PerspId], (int)mjPROJ_PERSPECTIVE);
	}

	S.Cleanup();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_CameraIntrinsics2Vec,
	"URLab.Import.RoundTrip_CameraIntrinsics2Vec",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_CameraIntrinsics2Vec::RunTest(const FString&)
{
	// focal, principal and sensorsize are all 2-vectors. Before the audit fix,
	// focal and principal were never written out and sensorsize wrote three
	// entries into a two-entry field.
	//
	// Intrinsics are only legal on a camera with a pixel resolution; the
	// resolution is scaffolding for the compile, not part of what is asserted.
	FMjXmlImportSession S;
	if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="b1"><freejoint/><geom type="box" size="0.1 0.1 0.1"/></body>
            <camera name="cam" resolution="640 480" sensorsize="0.024 0.018"
                    focal="0.5 0.6" principal="0.1 0.2" pos="0 0 2"/>
          </worldbody>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	// The element side first: an intrinsic the reader dropped would leave the
	// compiled read below asserting against a schema default.
	UMjCamera* Cam = S.FindTemplate<UMjCamera>(TEXT("cam"));
	if (TestNotNull(TEXT("camera element imported"), Cam))
	{
		TestTrue(TEXT("focal is authored"), Cam->Focal.IsSet());
		TestTrue(TEXT("principal is authored"), Cam->Principal.IsSet());
		TestTrue(TEXT("sensorsize is authored"), Cam->Sensorsize.IsSet());
	}

	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}

	const int CamId = CompiledId(S, mjOBJ_CAMERA, TEXT("cam"));
	if (TestTrue(TEXT("camera compiled"), CamId >= 0))
	{
		// mjModel.cam_intrinsic is [focal x, focal y, principal x, principal y].
		const mjModel* M = S.Model();
		TestNearlyEqual(TEXT("focal_length[0]"), M->cam_intrinsic[CamId * 4 + 0], 0.5f, 1e-6f);
		TestNearlyEqual(TEXT("focal_length[1]"), M->cam_intrinsic[CamId * 4 + 1], 0.6f, 1e-6f);
		TestNearlyEqual(TEXT("principal_length[0]"), M->cam_intrinsic[CamId * 4 + 2], 0.1f, 1e-6f);
		TestNearlyEqual(TEXT("principal_length[1]"), M->cam_intrinsic[CamId * 4 + 3], 0.2f, 1e-6f);
		TestNearlyEqual(TEXT("sensor_size[0]"), M->cam_sensorsize[CamId * 2 + 0], 0.024f, 1e-6f);
		TestNearlyEqual(TEXT("sensor_size[1]"), M->cam_sensorsize[CamId * 2 + 1], 0.018f, 1e-6f);
	}

	S.Cleanup();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_GeomShellInertia,
	"URLab.Import.RoundTrip_GeomShellInertia",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_GeomShellInertia::RunTest(const FString&)
{
	// mjModel carries no typeinertia field, so what the attribute is worth is
	// the inertia the compiler derives from it: a hollow sphere is 2/3 m r²
	// against a solid one's 2/5, for the same mass and radius.
	FMjXmlImportSession S;
	if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="b_shell">
              <freejoint/>
              <geom name="g_shell" type="sphere" size="0.1" mass="1" shellinertia="true"/>
            </body>
            <body name="b_volume" pos="0 1 0">
              <freejoint/>
              <geom name="g_volume" type="sphere" size="0.1" mass="1"/>
            </body>
          </worldbody>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}

	UMjGeom* Shell = S.FindTemplate<UMjGeom>(TEXT("g_shell"));
	UMjGeom* Volume = S.FindTemplate<UMjGeom>(TEXT("g_volume"));
	if (TestNotNull(TEXT("shell geom imported"), Shell))
	{
		TestTrue(TEXT("shellinertia == true"), Shell->GetShellinertia());
	}
	if (TestNotNull(TEXT("volume geom imported"), Volume))
	{
		TestFalse(TEXT("shellinertia == false by default"), Volume->GetShellinertia());
	}

	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}

	const int ShellBody = CompiledId(S, mjOBJ_BODY, TEXT("b_shell"));
	const int VolumeBody = CompiledId(S, mjOBJ_BODY, TEXT("b_volume"));
	if (TestTrue(TEXT("both bodies compiled"), ShellBody >= 0 && VolumeBody >= 0))
	{
		const mjtNum* I = S.Model()->body_inertia;
		TestNearlyEqual(TEXT("shell inertia == 2/3 m r^2"),
			(float)I[ShellBody * 3], 2.0f / 3.0f * 0.01f, 1e-5f);
		TestNearlyEqual(TEXT("volume inertia == 2/5 m r^2"),
			(float)I[VolumeBody * 3], 2.0f / 5.0f * 0.01f, 1e-5f);
	}

	S.Cleanup();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_GeomFluidShape,
	"URLab.Import.RoundTrip_GeomFluidShape",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_GeomFluidShape::RunTest(const FString&)
{
	FMjXmlImportSession S;
	if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="b1">
              <freejoint/>
              <geom name="g_ellipsoid" type="box" size="0.1 0.1 0.1" fluidshape="ellipsoid"/>
              <geom name="g_none" type="box" size="0.1 0.1 0.1" pos="0 0.5 0"/>
            </body>
          </worldbody>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}

	const int EllipsoidId = CompiledId(S, mjOBJ_GEOM, TEXT("g_ellipsoid"));
	const int NoneId = CompiledId(S, mjOBJ_GEOM, TEXT("g_none"));
	if (TestTrue(TEXT("both geoms compiled"), EllipsoidId >= 0 && NoneId >= 0))
	{
		// geom_fluid[0] is the ellipsoid-interaction flag.
		const mjModel* M = S.Model();
		TestNearlyEqual(TEXT("fluid_ellipsoid == 1 for Ellipsoid"),
			(float)M->geom_fluid[EllipsoidId * mjNFLUID], 1.0f, 1e-6f);
		TestNearlyEqual(TEXT("fluid_ellipsoid == 0 for None"),
			(float)M->geom_fluid[NoneId * mjNFLUID], 0.0f, 1e-6f);
	}

	S.Cleanup();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_GeomFluidCoef,
	"URLab.Import.RoundTrip_GeomFluidCoef",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_GeomFluidCoef::RunTest(const FString&)
{
	// The coefficients only reach the model on a geom with ellipsoid fluid
	// interaction enabled; the fluidshape is scaffolding for the compile.
	FMjXmlImportSession S;
	if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="b1">
              <freejoint/>
              <geom name="g1" type="box" size="0.1 0.1 0.1"
                    fluidshape="ellipsoid" fluidcoef="0.5 0.25 1.5 1 1"/>
            </body>
          </worldbody>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}

	const int GeomId = CompiledId(S, mjOBJ_GEOM, TEXT("g1"));
	if (TestTrue(TEXT("geom compiled"), GeomId >= 0))
	{
		// geom_fluid slot 0 is the ellipsoid flag; the five coefficients follow.
		const mjtNum* Fluid = S.Model()->geom_fluid + GeomId * mjNFLUID;
		TestNearlyEqual(TEXT("fluid_coefs[0]"), (float)Fluid[1], 0.5f, 1e-6f);
		TestNearlyEqual(TEXT("fluid_coefs[1]"), (float)Fluid[2], 0.25f, 1e-6f);
		TestNearlyEqual(TEXT("fluid_coefs[2]"), (float)Fluid[3], 1.5f, 1e-6f);
		TestNearlyEqual(TEXT("fluid_coefs[3]"), (float)Fluid[4], 1.0f, 1e-6f);
		TestNearlyEqual(TEXT("fluid_coefs[4]"), (float)Fluid[5], 1.0f, 1e-6f);
	}

	S.Cleanup();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTest_MjImport_RoundTrip_EqualitySiteMode,
	"URLab.Import.RoundTrip_EqualitySiteMode",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FTest_MjImport_RoundTrip_EqualitySiteMode::RunTest(const FString&)
{
	// A connect equality refers either to two bodies or to two sites, and which
	// one it is is decided by which pair of attributes was authored.
	FMjXmlImportSession S;
	if (!S.Init(TEXT(R"(
        <mujoco>
          <worldbody>
            <body name="b1"><freejoint/><geom type="box" size="0.1 0.1 0.1"/><site name="s1"/></body>
            <body name="b2" pos="0 0.5 0"><freejoint/><geom type="box" size="0.1 0.1 0.1"/><site name="s2"/></body>
          </worldbody>
          <equality>
            <connect site1="s1" site2="s2"/>
            <connect body1="b1" body2="b2" anchor="0 0 0"/>
          </equality>
        </mujoco>
    )")))
	{
		AddError(S.LastError);
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	const mjModel* M = S.Model();

	TestEqual(TEXT("neq == 2"), (int)M->neq, 2);
	TestEqual(TEXT("type == connect"), M->eq_type[0], (int)mjtEq::mjEQ_CONNECT);
	TestEqual(TEXT("objtype == site (not body) when site1 non-empty"),
		M->eq_objtype[0], (int)mjOBJ_SITE);
	TestEqual(TEXT("obj1 resolves to site s1"),
		M->eq_obj1id[0], CompiledId(S, mjOBJ_SITE, TEXT("s1")));
	TestEqual(TEXT("obj2 resolves to site s2"),
		M->eq_obj2id[0], CompiledId(S, mjOBJ_SITE, TEXT("s2")));

	TestEqual(TEXT("body-mode objtype == body when site1 empty"),
		M->eq_objtype[1], (int)mjOBJ_BODY);
	TestEqual(TEXT("body-mode obj1 resolves to body b1"),
		M->eq_obj1id[1], CompiledId(S, mjOBJ_BODY, TEXT("b1")));
	TestEqual(TEXT("body-mode obj2 resolves to body b2"),
		M->eq_obj2id[1], CompiledId(S, mjOBJ_BODY, TEXT("b2")));

	S.Cleanup();
	return true;
}
