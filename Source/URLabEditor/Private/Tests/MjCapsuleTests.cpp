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

// A capsule is a <geom> whose `type` attribute says so, not a class of its own.
// Everything that used to distinguish a capsule component from a box one is a
// row of a table keyed by that attribute, and the capsule's row is the only one
// carrying a second mesh -- the two end caps -- because Unreal ships no capsule
// primitive. So these tests are about one class read through one attribute.
//
// The scale case has to ask for the redraw. UMjGeom::SyncEditorScaleFromSize is
// what draws `size` as a component scale, and it runs on registration and on a
// details-panel edit; a Blueprint construction-script template is never
// registered, so the call is explicit here for the same reason the editor makes
// it explicit after a property change.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjTestHelpers.h"
#include "MuJoCo/Elements/MjGeom.h"
#include "mujoco/mujoco.h"

// ============================================================================
// URLab.Capsule.Import_SizeForm_CreatesUMjCapsule
//   <geom type="capsule" size="radius halflength"> reads as a UMjGeom whose
//   Type is capsule and whose Size is [R, H], and whose drawn scale is the
//   cm-convention (X=Y=2R, Z=2H) the cap placement depends on.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCapsuleImportSizeForm,
	"URLab.Capsule.Import_SizeForm_CreatesUMjCapsule",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCapsuleImportSizeForm::RunTest(const FString& Parameters)
{
	const TCHAR* Xml = TEXT(R"(<mujoco>
  <worldbody>
    <body name="arm" pos="0 0 1">
      <geom name="upper" type="capsule" size="0.05 0.15" pos="0 0 0"/>
      <joint name="j" type="hinge" axis="0 1 0"/>
    </body>
  </worldbody>
</mujoco>)");

	FMjXmlImportSession S;
	if (!S.Init(Xml))
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		return false;
	}

	UMjGeom* Cap = S.FindTemplate<UMjGeom>(TEXT("upper"));
	if (!TestNotNull(TEXT("UMjGeom 'upper'"), Cap))
		return false;

	TestEqual(TEXT("Type"), static_cast<int32>(Cap->GetType()), static_cast<int32>(EMjGeomType::capsule));

	const TArray<double> Size = Cap->GetSize();
	if (TestEqual(TEXT("Size carries [radius, halflength]"), Size.Num(), 2))
	{
		TestEqual(TEXT("Radius"), Size[0], 0.05);
		TestEqual(TEXT("HalfLength"), Size[1], 0.15);
	}

	// Component scale should map radius → 2R (cm-to-UE), halflength → 2H.
	Cap->SyncEditorScaleFromSize();
	const FVector Scale = Cap->GetRelativeScale3D();
	TestTrue(TEXT("Scale.X ≈ 0.1"), FMath::IsNearlyEqual(Scale.X, 0.1, 1e-4));
	TestTrue(TEXT("Scale.Y ≈ 0.1"), FMath::IsNearlyEqual(Scale.Y, 0.1, 1e-4));
	TestTrue(TEXT("Scale.Z ≈ 0.3"), FMath::IsNearlyEqual(Scale.Z, 0.3, 1e-4));

	return true;
}

// ============================================================================
// URLab.Capsule.Import_FromToForm_ResolvedByBase
//   <geom type="capsule" fromto="..."> is a compile directive, not stored data.
//   Import folds it to pos/quat/size exactly as MuJoCo's compiler does, so the
//   attribute is gone and the half-length it implied is in size[1]. Endpoints
//   0.3 m apart along +Z -> half-length 0.15 m, radius still from size[0], and
//   the compiled model is the same either way.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCapsuleImportFromTo,
	"URLab.Capsule.Import_FromToForm_ResolvedByBase",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCapsuleImportFromTo::RunTest(const FString& Parameters)
{
	const TCHAR* Xml = TEXT(R"(<mujoco>
  <worldbody>
    <body name="arm" pos="0 0 1">
      <geom name="upper" type="capsule" size="0.05" fromto="0 0 -0.15 0 0 0.15"/>
      <joint name="j" type="hinge" axis="0 1 0"/>
    </body>
  </worldbody>
</mujoco>)");

	FMjXmlImportSession S;
	if (!S.Init(Xml))
	{
		AddError(FString::Printf(TEXT("Init failed: %s"), *S.LastError));
		return false;
	}

	UMjGeom* Cap = S.FindTemplate<UMjGeom>(TEXT("upper"));
	if (!TestNotNull(TEXT("UMjGeom 'upper'"), Cap))
		return false;

	TestFalse(TEXT("fromto does not survive import"), Cap->HasFromto());

	const TArray<double> Size = Cap->GetSize();
	if (TestEqual(TEXT("Size carries radius and half-length"), Size.Num(), 2))
	{
		TestEqual(TEXT("Radius (from size)"), Size[0], 0.05);
		TestEqual(TEXT("Half-length (from fromto)"), Size[1], 0.15);
	}

	// Midpoint of the two endpoints, in the body's frame.
	const FMjPosition3 Pos = Cap->GetPos();
	TestEqual(TEXT("pos X is the midpoint"), Pos.X, 0.0);
	TestEqual(TEXT("pos Y is the midpoint"), Pos.Y, 0.0);
	TestEqual(TEXT("pos Z is the midpoint"), Pos.Z, 0.0);

	if (!S.Compile())
	{
		AddError(S.LastError);
		return false;
	}

	const mjModel* M = S.Model();
	if (!TestNotNull(TEXT("compiled model"), (void*)M))
		return false;

	// Found by shape rather than by name: the scene assembly prefixes compiled
	// names with the actor's, and there is only one geom in this model.
	int32 GeomId = -1;
	for (int i = 0; i < M->ngeom; ++i)
	{
		if (M->geom_type[i] == mjGEOM_CAPSULE)
		{
			GeomId = i;
			break;
		}
	}
	if (!TestTrue(TEXT("the capsule is in the compiled model"), GeomId >= 0))
		return false;

	TestTrue(TEXT("radius ≈ 0.05"),
		FMath::IsNearlyEqual((float)M->geom_size[GeomId * 3 + 0], 0.05f, 1e-4f));
	TestTrue(TEXT("HalfLength resolved to ~0.15"),
		FMath::IsNearlyEqual((float)M->geom_size[GeomId * 3 + 1], 0.15f, 1e-4f));
	return true;
}

// ============================================================================
// URLab.Capsule.Compile_ProducesCapsuleGeom
//   A compiled model should report geom type mjGEOM_CAPSULE, the intended
//   radius, and matching half-length, proving we're round-tripping the shape
//   through the URLab pipeline rather than silently dropping it.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCapsuleCompile,
	"URLab.Capsule.Compile_ProducesCapsuleGeom",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCapsuleCompile::RunTest(const FString& Parameters)
{
	const TCHAR* Xml = TEXT(R"(<mujoco>
  <worldbody>
    <body name="arm" pos="0 0 1">
      <geom name="upper" type="capsule" size="0.05 0.15"/>
      <joint name="j" type="hinge" axis="0 1 0"/>
    </body>
  </worldbody>
</mujoco>)");

	FMjXmlImportSession S;
	if (!S.Init(Xml))
	{
		AddError(S.LastError);
		return false;
	}
	if (!S.Compile())
	{
		AddError(S.LastError);
		return false;
	}

	const mjModel* M = S.Model();
	if (!TestNotNull(TEXT("compiled model"), (void*)M))
		return false;

	// Locate the capsule geom — the plugin may prefix names with the actor.
	int GeomId = -1;
	for (int i = 0; i < M->ngeom; ++i)
	{
		if (M->geom_type[i] == mjGEOM_CAPSULE)
		{
			GeomId = i;
			break;
		}
	}
	if (!TestTrue(TEXT("capsule geom present in model"), GeomId >= 0))
	{
		return false;
	}

	TestTrue(TEXT("radius ≈ 0.05"),
		FMath::IsNearlyEqual((float)M->geom_size[GeomId * 3 + 0], 0.05f, 1e-4f));
	TestTrue(TEXT("halflength ≈ 0.15"),
		FMath::IsNearlyEqual((float)M->geom_size[GeomId * 3 + 1], 0.15f, 1e-4f));
	return true;
}
