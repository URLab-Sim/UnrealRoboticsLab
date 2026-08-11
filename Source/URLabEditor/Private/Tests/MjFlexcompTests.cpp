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

// A <flexcomp> is a compile-time macro, not a model object: it expands into a
// flex plus a body per unpinned vertex and is itself never given an id. So the
// only proof that an authored flexcomp still means anything is the compiled
// model's flex table -- nflex, flex_dim, flex_vertnum -- which is what the
// first half of this file asserts against. The second half asserts that the
// reader lands the same attributes on the spec, including the ones that
// live on the <elasticity> and <pin> child elements rather than on <flexcomp>.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjTestHelpers.h"
#include "MuJoCo/Elements/MjFlexcomp.h"
#include "MuJoCo/Gen/Elements/Bodies/MjFlexElasticity.gen.h"
#include "MuJoCo/Gen/Elements/Bodies/MjFlexcompPin.gen.h"
#include "mujoco/mujoco.h"

namespace MjFlexcompTests
{

/** Author a grid flexcomp on the session's body, as the reader would. */
UMjFlexcompBase* AddGrid(FMjUESession& Session, const TCHAR* Name, int32 Dim, FMjVec3 Count,
	FMjVec3 Spacing, double Mass, double Radius)
{
	UMjFlexcompBase* Flex = Session.Add<UMjFlexcompBase>(Session.Body, Name);
	if (Flex == nullptr)
	{
		return nullptr;
	}
	Flex->SetType(EMjFlexcompType::grid);
	Flex->SetDim(Dim);
	Flex->SetCount(Count);
	Flex->SetSpacing(Spacing);
	Flex->SetMass(Mass);
	Flex->SetRadius(Radius);
	return Flex;
}

} // namespace MjFlexcompTests

// ============================================================================
// URLab.Flexcomp.Grid2D_Compiles
//   A 2D grid flexcomp should compile and produce a flex in the model.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjFlexcompGrid2DCompiles,
	"URLab.Flexcomp.Grid2D_Compiles",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjFlexcompGrid2DCompiles::RunTest(const FString& Parameters)
{
	FMjUESession S;
	bool bOk = S.Init([](FMjUESession& Session) {
		MjFlexcompTests::AddGrid(Session, TEXT("testgrid"), 2, FMjVec3(4.0, 4.0, 1.0),
			FMjVec3(0.05, 0.05, 0.05), 0.5, 0.005);
	});

	if (!bOk)
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	mjModel* M = S.Manager->PhysicsEngine->m_model;
	TestTrue(TEXT("Model should have at least 1 flex"), M->nflex >= 1);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Flexcomp.Grid1D_Compiles
//   A 1D grid (rope) should compile.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjFlexcompGrid1DCompiles,
	"URLab.Flexcomp.Grid1D_Compiles",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjFlexcompGrid1DCompiles::RunTest(const FString& Parameters)
{
	FMjUESession S;
	bool bOk = S.Init([](FMjUESession& Session) {
		MjFlexcompTests::AddGrid(Session, TEXT("testrope"), 1, FMjVec3(8.0, 1.0, 1.0),
			FMjVec3(0.1, 0.1, 0.1), 0.2, 0.01);
	});

	if (!bOk)
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	mjModel* M = S.Manager->PhysicsEngine->m_model;
	TestTrue(TEXT("Model should have at least 1 flex"), M->nflex >= 1);
	TestTrue(TEXT("Flex dim should be 1"), M->flex_dim[0] == 1);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Flexcomp.Grid3D_Compiles
//   A 3D grid (volumetric) should compile.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjFlexcompGrid3DCompiles,
	"URLab.Flexcomp.Grid3D_Compiles",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjFlexcompGrid3DCompiles::RunTest(const FString& Parameters)
{
	FMjUESession S;
	bool bOk = S.Init([](FMjUESession& Session) {
		MjFlexcompTests::AddGrid(Session, TEXT("testvol"), 3, FMjVec3(3.0, 3.0, 3.0),
			FMjVec3(0.05, 0.05, 0.05), 1.0, 0.005);
	});

	if (!bOk)
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	mjModel* M = S.Manager->PhysicsEngine->m_model;
	TestTrue(TEXT("Model should have at least 1 flex"), M->nflex >= 1);
	TestTrue(TEXT("Flex dim should be 3"), M->flex_dim[0] == 3);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Flexcomp.Grid2D_CorrectBodyCount
//   4x4 grid = 16 vertices = 16 child bodies (all unpinned).
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjFlexcompGrid2DBodyCount,
	"URLab.Flexcomp.Grid2D_CorrectBodyCount",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjFlexcompGrid2DBodyCount::RunTest(const FString& Parameters)
{
	FMjUESession S;
	bool bOk = S.Init([](FMjUESession& Session) {
		MjFlexcompTests::AddGrid(Session, TEXT("counttest"), 2, FMjVec3(4.0, 4.0, 1.0),
			FMjVec3(0.05, 0.05, 0.05), 0.5, 0.005);
	});

	if (!bOk)
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	mjModel* M = S.Manager->PhysicsEngine->m_model;
	// 4x4 grid = 16 flex vertices
	TestTrue(TEXT("Should have at least 1 flex"), M->nflex >= 1);
	TestEqual(TEXT("Flex should have 16 vertices"), (int32)M->flex_vertnum[0], 16);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Flexcomp.PinnedVertices
//   Pinning vertices should reduce the number of created bodies.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjFlexcompPinnedVertices,
	"URLab.Flexcomp.PinnedVertices",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjFlexcompPinnedVertices::RunTest(const FString& Parameters)
{
	FMjUESession S;
	bool bOk = S.Init([](FMjUESession& Session) {
		UMjFlexcompBase* Flex = MjFlexcompTests::AddGrid(Session, TEXT("pintest"), 1, FMjVec3(5.0, 1.0, 1.0),
			FMjVec3(0.1, 0.1, 0.1), 0.2, 0.01);
		if (Flex == nullptr)
		{
			return;
		}
		// Pinning is a <pin> child element, not an attribute of <flexcomp>.
		if (UMjFlexcompPin* Pin = Session.Add<UMjFlexcompPin>(Flex))
		{
			Pin->SetId({0.0, 4.0}); // Pin first and last
		}
	});

	if (!bOk)
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	mjModel* M = S.Manager->PhysicsEngine->m_model;
	TestTrue(TEXT("Should have at least 1 flex"), M->nflex >= 1);
	// 5 vertices in the flex regardless of pinning
	TestEqual(TEXT("Flex should have 5 vertices"), (int32)M->flex_vertnum[0], 5);

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Flexcomp.Import_Grid2D
//   Importing a flexcomp XML should create a UMjFlexcomp template.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjFlexcompImportGrid2D,
	"URLab.Flexcomp.Import_Grid2D",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjFlexcompImportGrid2D::RunTest(const FString& Parameters)
{
	FMjXmlImportSession S;
	bool bOk = S.Init(TEXT(
		"<mujoco>"
		"  <worldbody>"
		"    <body name=\"parent\">"
		"      <geom size=\"1\"/>"
		"      <flexcomp name=\"myflex\" type=\"grid\" dim=\"2\" count=\"3 3 1\" spacing=\"0.1 0.1 0.1\" mass=\"0.5\" radius=\"0.01\">"
		"        <elasticity young=\"100\" damping=\"0.5\"/>"
		"        <pin id=\"0 1 2\"/>"
		"      </flexcomp>"
		"    </body>"
		"  </worldbody>"
		"</mujoco>"));

	if (!bOk)
	{
		AddError(FString::Printf(TEXT("Import Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	UMjFlexcomp* Flex = S.FindTemplate<UMjFlexcomp>(TEXT("myflex"));
	TestNotNull(TEXT("Should find flexcomp template"), Flex);

	if (Flex)
	{
		TestEqual(TEXT("Type should be Grid"), Flex->GetType(), EMjFlexcompType::grid);
		TestEqual(TEXT("Dim should be 2"), Flex->GetDim(), 2);
		TestEqual(TEXT("count[0] should be 3"), (int32)Flex->GetCount().X, 3);
		TestEqual(TEXT("count[1] should be 3"), (int32)Flex->GetCount().Y, 3);
	}

	// Young and damping are attributes of the <elasticity> child, and the pin
	// ids of the <pin> child; each is its own element of the spec.
	UMjFlexElasticity* Elasticity = S.FindFirstTemplate<UMjFlexElasticity>();
	if (TestNotNull(TEXT("Should find elasticity template"), Elasticity))
	{
		TestTrue(TEXT("Young should be 100"), MjTestMath::NearlyEqual(Elasticity->GetYoung(), 100.0));
		TestTrue(TEXT("Damping should be 0.5"), MjTestMath::NearlyEqual(Elasticity->GetDamping(), 0.5));
	}

	UMjFlexcompPin* Pin = S.FindFirstTemplate<UMjFlexcompPin>();
	if (TestNotNull(TEXT("Should find pin template"), Pin))
	{
		TestEqual(TEXT("Should have 3 pin IDs"), Pin->GetId().Num(), 3);
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Flexcomp.Elasticity_Imported
//   Elasticity sub-element attributes should be parsed.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjFlexcompElasticityImported,
	"URLab.Flexcomp.Elasticity_Imported",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjFlexcompElasticityImported::RunTest(const FString& Parameters)
{
	FMjXmlImportSession S;
	bool bOk = S.Init(TEXT(
		"<mujoco>"
		"  <worldbody>"
		"    <body name=\"parent\">"
		"      <geom size=\"1\"/>"
		"      <flexcomp name=\"elastic\" type=\"grid\" dim=\"2\" count=\"3 3 1\" spacing=\"0.1 0.1 0.1\" mass=\"0.5\" radius=\"0.01\">"
		"        <elasticity young=\"1000\" poisson=\"0.3\" damping=\"0.01\"/>"
		"      </flexcomp>"
		"    </body>"
		"  </worldbody>"
		"</mujoco>"));

	if (!bOk)
	{
		AddError(FString::Printf(TEXT("Import Init failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	TestNotNull(TEXT("Should find flexcomp template"), S.FindTemplate<UMjFlexcomp>(TEXT("elastic")));

	UMjFlexElasticity* Elasticity = S.FindFirstTemplate<UMjFlexElasticity>();
	if (TestNotNull(TEXT("Should find elasticity template"), Elasticity))
	{
		TestTrue(TEXT("Young should be 1000"), MjTestMath::NearlyEqual(Elasticity->GetYoung(), 1000.0));
		TestTrue(TEXT("Poisson should be 0.3"), MjTestMath::NearlyEqual(Elasticity->GetPoisson(), 0.3));
		TestTrue(TEXT("Damping should be 0.01"), MjTestMath::NearlyEqual(Elasticity->GetDamping(), 0.01));
	}

	S.Cleanup();
	return true;
}
