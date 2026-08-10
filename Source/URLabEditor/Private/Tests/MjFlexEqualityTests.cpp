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
#include "MuJoCo/Gen/Elements/Constraints/MjEqualityFlex.gen.h"
#include "MuJoCo/Gen/Elements/Constraints/MjFlexstrain.gen.h"
#include "MuJoCo/Gen/Elements/Constraints/MjFlexvert.gen.h"

namespace
{
// Minimal MJCF that compiles: one free body with a capsule geom, plus a
// flexcomp that produces a real <flex> asset. The generated flex is named
// "cloth" because flexcomp's name becomes the flex name when the flex
// section is absent. The flexcomp is authored with edge equality off so
// we can add our own explicit <equality><flex ... /> entries.
FString MakeFlexEqualityXml(const FString& EqualityChildTag)
{
	// The equality on the flex is the whole point of the test — the
	// flexcomp's own edge equality is disabled so we're the single
	// source of a flex-referencing equality in the spec.
	//
	// Topology: 2x2 grid of nodes laid out flat in XY (z count = 1).
	// MuJoCo 3.8 added a stricter orthonormal check on flex grid edge
	// vectors and rejects degenerate z dimensions when dof="trilinear"
	// (the third edge has zero length). A 2D cloth uses default dof,
	// which handles a flat grid fine.
	return FString::Printf(TEXT(R"(<mujoco>
  <worldbody>
    <body name="anchor" pos="0 0 0">
      <geom size=".05"/>
    </body>
    <flexcomp name="cloth" type="grid" count="2 2 1" spacing="0.1 0.1 0.1"
              pos="0 0 0.3">
      <contact selfcollide="none"/>
      <edge equality="false"/>
    </flexcomp>
  </worldbody>

  <equality>
    <%s flex="cloth" active="true"/>
  </equality>
</mujoco>)"),
		*EqualityChildTag);
}
} // namespace

// Parametrised helper: run the parser on a flex-equality child tag and verify
// it produced the element class that tag names, and nothing else.
//
// The kind of a flex equality is no longer an enum field on one shared class:
// <flex>, <flexvert> and <flexstrain> are three separate elements, so the class
// the reader built IS the kind. Counting all three is what makes that an
// assertion rather than a coincidence -- a reader that emitted every variant
// would satisfy the lookup on its own.
template <typename ElementClass>
static bool CheckFlexEqualityVariant(FAutomationTestBase& Tester, const FString& Tag)
{
	FMjXmlImportSession S;
	const FString Xml = MakeFlexEqualityXml(Tag);
	if (!S.Init(Xml))
	{
		Tester.AddError(FString::Printf(TEXT("Init failed (%s): %s"), *Tag, *S.LastError));
		return false;
	}

	ElementClass* Eq = S.FindFirstTemplate<ElementClass>();
	if (!Tester.TestNotNull(TEXT("equality component"), Eq))
	{
		return false;
	}

	Tester.TestEqual(FString::Printf(TEXT("<%s> → exactly one flex equality element"), *Tag),
		S.CountTemplates<UMjEqualityFlex>() + S.CountTemplates<UMjFlexvert>()
			+ S.CountTemplates<UMjFlexstrain>(),
		1);
	Tester.TestEqual(FString::Printf(TEXT("<%s>: flex attribute captured"), *Tag),
		Eq->Flex, FString(TEXT("cloth")));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjEqualityFlex,
	"URLab.Equality.Flex",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjEqualityFlex::RunTest(const FString&)
{
	return CheckFlexEqualityVariant<UMjEqualityFlex>(*this, TEXT("flex"));
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjEqualityFlexVert,
	"URLab.Equality.FlexVert",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjEqualityFlexVert::RunTest(const FString&)
{
	return CheckFlexEqualityVariant<UMjFlexvert>(*this, TEXT("flexvert"));
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjEqualityFlexStrain,
	"URLab.Equality.FlexStrain",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjEqualityFlexStrain::RunTest(const FString&)
{
	return CheckFlexEqualityVariant<UMjFlexstrain>(*this, TEXT("flexstrain"));
}
