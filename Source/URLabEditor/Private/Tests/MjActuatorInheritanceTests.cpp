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
#include "Tests/MjTestHelpers.h"
#include "MuJoCo/Core/AMjManager.h"
#include "mujoco/mujoco.h"

// ============================================================================
// Regression guard for actuator gain-parameter inheritance.
//
// mjs_setTo* write several params UNCONDITIONALLY by value (e.g. position's
// gainprm[0] = kp). URLab used to pass a -1.0 sentinel for any param that
// wasn't authored on the element, which clobbered the value the default class
// had already inherited onto the actuator — so a `<position class="X">` with no
// inline kp compiled with gainprm[0] = -1 instead of the class's kp. These
// tests compile the same MJCF through URLab and through native mj_loadXML and
// assert the compiled actuator params match: the sentinel bug makes them
// diverge, the fix makes them identical.
// ============================================================================

namespace
{
// Every actuator below inherits ALL of its gain params from a <default class>;
// none carries an inline param on the actuator element (the bug trigger).
//
// Damper is covered separately (InheritedDamperKv_Preserved): native
// mj_loadXML drops a damper's class-inherited kv, so a native diff isn't a fair
// reference for it — URLab intentionally preserves the value instead.
static const TCHAR* INHERITED_GAINS_XML = TEXT(R"(
    <mujoco>
      <default>
        <default class="pos">  <position    kp="2000" kv="100"/></default>
        <default class="ivel"> <intvelocity kp="800"  kv="12" actrange="-1 1"/></default>
        <default class="vel">  <velocity    kv="55"/>          </default>
        <default class="cyl">  <cylinder    timeconst="0.2" area="3"/></default>
        <default class="adh">  <adhesion    gain="9" ctrlrange="0 1"/></default>
      </default>
      <worldbody>
        <body name="b1"><joint name="j1" type="hinge"/><geom type="capsule" size="0.05 0.5" mass="1"/></body>
        <body name="b2"><joint name="j2" type="hinge"/><geom type="capsule" size="0.05 0.5" mass="1"/></body>
        <body name="b3"><joint name="j3" type="hinge"/><geom type="capsule" size="0.05 0.5" mass="1"/></body>
        <body name="b5"><joint name="j5" type="hinge"/><geom type="capsule" size="0.05 0.5" mass="1"/></body>
        <body name="b6"><geom type="sphere" size="0.1" mass="1"/></body>
      </worldbody>
      <actuator>
        <position    class="pos"  name="p"  joint="j1"/>
        <intvelocity class="ivel" name="iv" joint="j2"/>
        <velocity    class="vel"  name="v"  joint="j3"/>
        <cylinder    class="cyl"  name="c"  joint="j5"/>
        <adhesion    class="adh"  name="a"  body="b6"/>
      </actuator>
    </mujoco>
)");

// Compare every dyntype/gaintype/biastype + the full dynprm/gainprm/biasprm
// vectors of one actuator against a reference model. Returns false and logs on
// the first divergence.
bool ActuatorParamsMatch(
	FAutomationTestBase& Test, const TCHAR* Label,
	const mjModel* Ref, const mjModel* Got, int32 Act)
{
	const float Eps = 1e-5f;
	bool bOk = true;

	auto CheckArr = [&](const char* Name, const mjtNum* R, const mjtNum* G, int Stride, int Count)
	{
		for (int p = 0; p < Count; ++p)
		{
			const float r = (float)R[Act * Stride + p];
			const float g = (float)G[Act * Stride + p];
			if (FMath::Abs(r - g) > Eps)
			{
				Test.AddError(FString::Printf(
					TEXT("%s: %hs[%d] ref=%f got=%f"), Label, Name, p, r, g));
				bOk = false;
			}
		}
	};

	if (Ref->actuator_dyntype[Act] != Got->actuator_dyntype[Act])
	{
		Test.AddError(FString::Printf(TEXT("%s: dyntype ref=%d got=%d"),
			Label, Ref->actuator_dyntype[Act], Got->actuator_dyntype[Act]));
		bOk = false;
	}
	if (Ref->actuator_gaintype[Act] != Got->actuator_gaintype[Act])
	{
		Test.AddError(FString::Printf(TEXT("%s: gaintype ref=%d got=%d"),
			Label, Ref->actuator_gaintype[Act], Got->actuator_gaintype[Act]));
		bOk = false;
	}
	if (Ref->actuator_biastype[Act] != Got->actuator_biastype[Act])
	{
		Test.AddError(FString::Printf(TEXT("%s: biastype ref=%d got=%d"),
			Label, Ref->actuator_biastype[Act], Got->actuator_biastype[Act]));
		bOk = false;
	}
	CheckArr("gainprm", Ref->actuator_gainprm, Got->actuator_gainprm, mjNGAIN, mjNGAIN);
	CheckArr("biasprm", Ref->actuator_biasprm, Got->actuator_biasprm, mjNBIAS, mjNBIAS);
	CheckArr("dynprm", Ref->actuator_dynprm, Got->actuator_dynprm, mjNDYN, mjNDYN);
	return bOk;
}
} // namespace

// ============================================================================
// URLab.Actuator.InheritedGains_MatchNative
//   The whole class-inherited actuator family, compiled through URLab, must
//   produce the same gain/bias/dyn params as native mj_loadXML.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjActuatorInheritedGainsMatchNative,
	"URLab.Actuator.InheritedGains_MatchNative",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjActuatorInheritedGainsMatchNative::RunTest(const FString&)
{
	FMjTestSession Ref;
	if (!Ref.CompileXml(INHERITED_GAINS_XML))
	{
		AddError(Ref.LastError);
		return false;
	}

	FMjXmlImportSession S;
	if (!S.Init(INHERITED_GAINS_XML))
	{
		AddError(S.LastError);
		Ref.Cleanup();
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		S.Cleanup();
		Ref.Cleanup();
		return false;
	}

	if (S.Model()->nu != Ref.m->nu)
	{
		AddError(FString::Printf(TEXT("nu mismatch: ref=%d got=%d"),
			(int)Ref.m->nu, (int)S.Model()->nu));
	}
	else
	{
		for (int32 a = 0; a < Ref.m->nu; ++a)
		{
			const FString Label = FString::Printf(TEXT("actuator %d"), a);
			ActuatorParamsMatch(*this, *Label, Ref.m, S.Model(), a);
		}
	}

	S.Cleanup();
	Ref.Cleanup();
	return true;
}

// ============================================================================
// URLab.Actuator.InheritedPositionKp_NotClobbered
//   Pinpoint check on the exact bug: a position actuator inheriting kp=2000
//   from its default class must compile with gainprm[0] == 2000 and
//   biasprm[1] == -2000, not the -1 sentinel that used to overwrite it.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjActuatorInheritedPositionKpNotClobbered,
	"URLab.Actuator.InheritedPositionKp_NotClobbered",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjActuatorInheritedPositionKpNotClobbered::RunTest(const FString&)
{
	static const TCHAR* Xml = TEXT(R"(
        <mujoco>
          <default>
            <default class="big"><position kp="2000"/></default>
          </default>
          <worldbody>
            <body><joint name="j" type="hinge"/><geom type="capsule" size="0.05 0.5" mass="1"/></body>
          </worldbody>
          <actuator>
            <position class="big" name="p" joint="j"/>
          </actuator>
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

	if (S.Model()->nu != 1)
	{
		AddError(FString::Printf(TEXT("expected nu==1, got %d"), (int)S.Model()->nu));
		S.Cleanup();
		return false;
	}

	TestEqual(TEXT("inherited kp -> gainprm[0]"),
		(float)S.Model()->actuator_gainprm[0], 2000.0f);
	TestEqual(TEXT("position feedback -> biasprm[1] == -kp"),
		(float)S.Model()->actuator_biasprm[1], -2000.0f);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Actuator.InheritedDamperKv_Preserved
//   A damper inheriting kv from its default class compiles with
//   gainprm[2] == -kv. Native mj_loadXML drops a damper's class-inherited kv to
//   0, so this asserts the value directly rather than against a native diff:
//   URLab intentionally preserves the damping the user asked for.
//
//   TODO(jonat): the mj_loadXML drop is a verified MuJoCo bug — OneActuator()
//   seeds `kv = 0` for damper while every sibling seeds from the inherited
//   field. URLab diverges here (keeps kv); revisit whether to match MuJoCo
//   bug-for-bug for UE-vs-bridge fidelity or upstream a reader fix. See the
//   _todo_damper_inherit note in codegen_rules.json.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjActuatorInheritedDamperKvPreserved,
	"URLab.Actuator.InheritedDamperKv_Preserved",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjActuatorInheritedDamperKvPreserved::RunTest(const FString&)
{
	static const TCHAR* Xml = TEXT(R"(
        <mujoco>
          <default>
            <default class="damp"><damper kv="7" ctrlrange="0 1"/></default>
          </default>
          <worldbody>
            <body><joint name="j" type="hinge"/><geom type="capsule" size="0.05 0.5" mass="1"/></body>
          </worldbody>
          <actuator>
            <damper class="damp" name="d" joint="j"/>
          </actuator>
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

	if (S.Model()->nu != 1)
	{
		AddError(FString::Printf(TEXT("expected nu==1, got %d"), (int)S.Model()->nu));
		S.Cleanup();
		return false;
	}

	TestEqual(TEXT("inherited damper kv -> gainprm[2] == -kv"),
		(float)S.Model()->actuator_gainprm[2], -7.0f);

	// Canary for the upstream MuJoCo bug this test documents: mj_loadXML
	// currently drops the same inherited kv to 0 (OneActuator seeds kv=0 for
	// damper). If this ever fails because native returns -7, upstream fixed
	// their reader — at that point delete the divergence note (_todo_damper_
	// inherit) and fold damper back into InheritedGains_MatchNative.
	FMjTestSession Ref;
	if (Ref.CompileXml(Xml))
	{
		TestEqual(TEXT("native mj_loadXML still drops inherited damper kv (canary)"),
			(float)Ref.m->actuator_gainprm[2], 0.0f);
		Ref.Cleanup();
	}
	else
	{
		AddError(Ref.LastError);
	}

	S.Cleanup();
	return true;
}
