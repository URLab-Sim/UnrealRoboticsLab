// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// The two spec-layer canonicalizations that have no other gate.
//
// FrameConversion pins what the kind-typed storage is FOR: a position and a
// direction are the same three doubles and cross into Unreal differently, so
// the values chosen here have no zero component and no repeated component. A
// dropped Y negation, a position rule applied to a direction, or a direction
// rule applied to a position each move a number that would sit still under a
// tidier fixture.
//
// FromtoFold pins the import's canonical form. Imported fromto becomes pose
// and persistent half-extents, including in editable default classes.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "GameFramework/Actor.h"
#include "HAL/FileManager.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "UObject/Package.h"

#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjFrameTypes.h"
#include "MuJoCo/Gen/Elements/Bodies/MjBody.gen.h"
#include "MuJoCo/Gen/Elements/Geometry/MjGeom.gen.h"
#include "MuJoCo/Gen/Elements/Joints/MjJoint.gen.h"
#include "MuJoCo/Gen/Elements/Geometry/MjSite.gen.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

namespace MjFrameTypeTests
{

UBlueprint* MakeScratchBlueprint()
{
	const FString Name = FString::Printf(TEXT("MjFrameTypes_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	UPackage* Package = CreatePackage(*(TEXT("/Temp/") + Name));
	return FKismetEditorUtilities::CreateBlueprint(AActor::StaticClass(), Package, FName(*Name), BPTYPE_Normal,
		UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
}

/** Parse `Xml` into a scratch Blueprint, or null with the failure reported. */
UBlueprint* ParseScratch(FAutomationTestBase& Test, const FString& Xml)
{
	UBlueprint* Blueprint = MakeScratchBlueprint();
	if (Blueprint == nullptr)
	{
		Test.AddError(TEXT("could not create a scratch Blueprint"));
		return nullptr;
	}
	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, Xml, TEXT("<inline>"));
	if (!Parsed.IsOk())
	{
		TArray<FString> Lines;
		for (const FMjSpecDiagnostic& Diagnostic : Parsed.Errors)
		{
			Lines.Add(Diagnostic.ToString());
		}
		Test.AddError(FString::Printf(TEXT("parse failed: %s"), *FString::Join(Lines, TEXT("; "))));
		return nullptr;
	}
	return Blueprint;
}

template <class T>
T* FindTemplate(UBlueprint& Blueprint, const TCHAR* MjName)
{
	if (Blueprint.SimpleConstructionScript == nullptr)
	{
		return nullptr;
	}
	for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
	{
		T* Template = Cast<T>(Node->ComponentTemplate);
		if (Template == nullptr)
		{
			continue;
		}
		const UMjNodeComponent* Element = Cast<UMjNodeComponent>(Template);
		if (Element != nullptr && Element->MjName.IsSet() && Element->MjName.GetValue() == MjName)
		{
			return Template;
		}
	}
	return nullptr;
}

/** Compile `Xml` with MuJoCo itself, or null with the failure reported. */
mjModel* CompileWithMuJoCo(FAutomationTestBase& Test, const FString& Label, const FString& Xml)
{
	const FString Path = FPaths::Combine(FPaths::ProjectIntermediateDir(),
		FString::Printf(TEXT("MjFrameTypes_%s_%s.xml"), *Label, *FGuid::NewGuid().ToString(EGuidFormats::Digits)));
	if (!FFileHelper::SaveStringToFile(Xml, *Path, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
	{
		Test.AddError(FString::Printf(TEXT("%s: could not stage MJCF"), *Label));
		return nullptr;
	}
	char Error[1024] = {0};
	mjModel* Model = mj_loadXML(TCHAR_TO_UTF8(*Path), nullptr, Error, sizeof(Error));
	IFileManager::Get().Delete(*Path);
	if (Model == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("%s: MJCF did not compile: %s"), *Label, UTF8_TO_TCHAR(Error)));
	}
	return Model;
}

/**
 * Compile the original and URLab's rewrite and compare the geom frames.
 *
 * The transient export projection changes the serialized form without
 * changing the compiled model, as checked against MuJoCo's own compiler.
 */
void GeomFramesAgree(FAutomationTestBase& Test, const FString& Label, const FString& OriginalXml,
	const FString& WrittenXml)
{
	mjModel* Original = CompileWithMuJoCo(Test, Label + TEXT("_original"), OriginalXml);
	if (Original == nullptr)
	{
		return;
	}
	mjModel* Written = CompileWithMuJoCo(Test, Label + TEXT("_written"), WrittenXml);
	if (Written == nullptr)
	{
		mj_deleteModel(Original);
		return;
	}

	Test.TestEqual(*FString::Printf(TEXT("%s: same geom count"), *Label), Written->ngeom, Original->ngeom);
	if (Written->ngeom == Original->ngeom)
	{
		for (int Index = 0; Index < 3 * Original->ngeom; ++Index)
		{
			Test.TestNearlyEqual(*FString::Printf(TEXT("%s: geom_pos[%d]"), *Label, Index),
				(float)Written->geom_pos[Index], (float)Original->geom_pos[Index], 1e-6f);
			Test.TestNearlyEqual(*FString::Printf(TEXT("%s: geom_size[%d]"), *Label, Index),
				(float)Written->geom_size[Index], (float)Original->geom_size[Index], 1e-6f);
		}
		for (int Index = 0; Index < 4 * Original->ngeom; ++Index)
		{
			Test.TestNearlyEqual(*FString::Printf(TEXT("%s: geom_quat[%d]"), *Label, Index),
				(float)Written->geom_quat[Index], (float)Original->geom_quat[Index], 1e-6f);
		}
	}

	mj_deleteModel(Original);
	mj_deleteModel(Written);
}

} // namespace MjFrameTypeTests

// ============================================================================
// URLab.Gen.FrameTypes.FrameConversion
//   A position and a direction are three doubles apiece and cross into Unreal
//   by different rules. Every component is non-zero and no two are equal, so a
//   dropped negation or a swapped rule moves a number.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjFrameTypesConversionTest, "URLab.Gen.FrameTypes.FrameConversion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjFrameTypesConversionTest::RunTest(const FString& Parameters)
{
	using namespace MjFrameTypeTests;

	const FString Xml = TEXT(R"(<mujoco model="frames">
  <compiler angle="radian"/>
  <worldbody>
    <body name="b" pos="0.11 0.22 0.33">
      <joint name="j" type="hinge" axis="0.6 0.48 0.64"/>
      <geom name="g" type="sphere" size="0.05"/>
    </body>
  </worldbody>
  <option gravity="0.7 0.9 -9.81"/>
</mujoco>
)");

	UBlueprint* Blueprint = ParseScratch(*this, Xml);
	if (Blueprint == nullptr)
	{
		return false;
	}

	UMjBodyBase* Body = FindTemplate<UMjBodyBase>(*Blueprint, TEXT("b"));
	UMjJoint* Joint = FindTemplate<UMjJoint>(*Blueprint, TEXT("j"));
	if (!TestNotNull(TEXT("body b"), Body) || !TestNotNull(TEXT("joint j"), Joint))
	{
		return false;
	}

	// Storage is verbatim MJCF: right-handed, metres, exactly as authored.
	const FMjPosition3 Pos = Body->GetPos();
	TestNearlyEqual(TEXT("stored pos X"), (float)Pos.X, 0.11f, 1e-6f);
	TestNearlyEqual(TEXT("stored pos Y"), (float)Pos.Y, 0.22f, 1e-6f);
	TestNearlyEqual(TEXT("stored pos Z"), (float)Pos.Z, 0.33f, 1e-6f);

	// Crossing into Unreal negates Y and turns metres into centimetres.
	const FVector UePos = Pos.ToUnreal();
	TestNearlyEqual(TEXT("pos to Unreal X"), (float)UePos.X, 11.0f, 1e-4f);
	TestNearlyEqual(TEXT("pos to Unreal Y"), (float)UePos.Y, -22.0f, 1e-4f);
	TestNearlyEqual(TEXT("pos to Unreal Z"), (float)UePos.Z, 33.0f, 1e-4f);

	const FMjPosition3 BackPos = FMjPosition3::FromUnreal(UePos);
	TestNearlyEqual(TEXT("pos round trips X"), (float)BackPos.X, (float)Pos.X, 1e-6f);
	TestNearlyEqual(TEXT("pos round trips Y"), (float)BackPos.Y, (float)Pos.Y, 1e-6f);
	TestNearlyEqual(TEXT("pos round trips Z"), (float)BackPos.Z, (float)Pos.Z, 1e-6f);

	const FMjDirection3 Axis = Joint->GetAxis();
	TestNearlyEqual(TEXT("stored axis X"), (float)Axis.X, 0.6f, 1e-6f);
	TestNearlyEqual(TEXT("stored axis Y"), (float)Axis.Y, 0.48f, 1e-6f);
	TestNearlyEqual(TEXT("stored axis Z"), (float)Axis.Z, 0.64f, 1e-6f);

	// A direction takes the same Y negation and NONE of the scaling. The
	// magnitudes here are far from 1 and far from 100, so applying the position
	// rule to a direction fails on every component rather than on none.
	const FVector UeAxis = Axis.ToUnreal();
	TestNearlyEqual(TEXT("axis to Unreal X"), (float)UeAxis.X, 0.6f, 1e-6f);
	TestNearlyEqual(TEXT("axis to Unreal Y"), (float)UeAxis.Y, -0.48f, 1e-6f);
	TestNearlyEqual(TEXT("axis to Unreal Z"), (float)UeAxis.Z, 0.64f, 1e-6f);

	const FMjDirection3 BackAxis = FMjDirection3::FromUnreal(UeAxis);
	TestNearlyEqual(TEXT("axis round trips X"), (float)BackAxis.X, (float)Axis.X, 1e-6f);
	TestNearlyEqual(TEXT("axis round trips Y"), (float)BackAxis.Y, (float)Axis.Y, 1e-6f);
	TestNearlyEqual(TEXT("axis round trips Z"), (float)BackAxis.Z, (float)Axis.Z, 1e-6f);

	// The two conversions must not agree: if they ever did, the type
	// distinction would be decoration.
	TestTrue(TEXT("position and direction rules differ"),
		!FMath::IsNearlyEqual((float)Pos.ToUnreal().X, (float)FMjDirection3(Pos.X, Pos.Y, Pos.Z).ToUnreal().X, 1e-4f));

	return true;
}

// ============================================================================
// URLab.Gen.FrameTypes.FromtoFold
//   Imported fromto is stored as canonical pose and half-extents.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjFrameTypesFromtoFoldTest, "URLab.Gen.FrameTypes.FromtoFold",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjFrameTypesFromtoFoldTest::RunTest(const FString& Parameters)
{
	using namespace MjFrameTypeTests;

	// `shin` authors its own type; `arm` inherits type and size from its default
	// class. `slab` is a box, where canonical half-length uses a different slot.
	const FString Xml = TEXT(R"(<mujoco model="ends">
  <compiler angle="radian"/>
  <default>
    <default class="limb">
      <geom type="capsule" size="0.03"/>
    </default>
  </default>
  <worldbody>
    <body name="b">
      <geom name="shin" type="capsule" size="0.02" fromto="0.1 0.2 0.3 0.1 0.2 0.7"/>
      <geom name="arm" class="limb" fromto="-0.1 0.4 0.2 0.3 0.4 0.2"/>
      <geom name="slab" type="box" size="0.05" fromto="0 0 1 0 0.6 1"/>
      <site name="tip" type="capsule" size="0.01" fromto="0.5 0 0 0.5 0 0.4"/>
    </body>
  </worldbody>
</mujoco>
)");

	UBlueprint* Blueprint = ParseScratch(*this, Xml);
	if (Blueprint == nullptr)
	{
		return false;
	}

	UMjGeomBase* Shin = FindTemplate<UMjGeomBase>(*Blueprint, TEXT("shin"));
	UMjGeomBase* Arm = FindTemplate<UMjGeomBase>(*Blueprint, TEXT("arm"));
	UMjGeomBase* Slab = FindTemplate<UMjGeomBase>(*Blueprint, TEXT("slab"));
	UMjSite* Tip = FindTemplate<UMjSite>(*Blueprint, TEXT("tip"));
	if (!TestNotNull(TEXT("geom shin"), Shin) || !TestNotNull(TEXT("geom arm"), Arm) || !TestNotNull(TEXT("geom slab"), Slab) || !TestNotNull(TEXT("site tip"), Tip))
	{
		return false;
	}

	TestFalse(TEXT("all concrete endpoints normalized"), Shin->HasFromto() || Arm->HasFromto()
		|| Slab->HasFromto() || Tip->HasFromto());
	TestTrue(TEXT("all concrete poses canonical"), Shin->HasPos() && Arm->HasPos()
		&& Slab->HasPos() && Tip->HasPos());
	TestTrue(TEXT("all concrete half-extents persist"), Shin->MjSpatialHalfExtentsCm.IsSet()
		&& Arm->MjSpatialHalfExtentsCm.IsSet() && Slab->MjSpatialHalfExtentsCm.IsSet()
		&& Tip->MjSpatialHalfExtentsCm.IsSet());
	TestFalse(TEXT("raw one-slot size cleared"), Shin->HasSize() || Slab->HasSize());

	// The writer emits the projected form, which is what MuJoCo's writer emits too.
	TArray<FMjSpecDiagnostic> WriteErrors;
	const FString Written = FSpecRef::OverBlueprint(*Blueprint).WriteMjcf(&WriteErrors);
	TestEqual(TEXT("write produced no diagnostics"), WriteErrors.Num(), 0);
	// The model is deliberately not NAMED fromto: this is a substring check over
	// the whole spec, and the attribute is what it has to be absent from.
	TestFalse(TEXT("no fromto survives into the written MJCF"), Written.Contains(TEXT("fromto")));

	// And the model is unchanged, which is the only claim that matters.
	GeomFramesAgree(*this, TEXT("fromto"), Xml, Written);

	return true;
}

// ============================================================================
// URLab.Gen.FrameTypes.FromtoFoldInheritedClass
//   The inherited directive becomes editable canonical class pose/size.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjFrameTypesFromtoInheritedTest,
	"URLab.Gen.FrameTypes.FromtoFoldInheritedClass",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjFrameTypesFromtoInheritedTest::RunTest(const FString& Parameters)
{
	using namespace MjFrameTypeTests;

	// `inherited` names its class and nothing else: type, size and fromto all
	// come from the chain. `both` authors a fromto over the class's value.
	const FString Xml = TEXT(R"(<mujoco model="inherited">
  <compiler angle="radian"/>
  <default>
    <default class="limb">
      <geom type="capsule" size="0.05"/>
      <default class="shin">
        <geom fromto="0 0 0 0 0 -0.3" size="0.049"/>
      </default>
    </default>
  </default>
  <worldbody>
    <body name="b" childclass="limb">
      <geom name="inherited" class="shin"/>
      <geom name="both" class="shin" fromto="0 0 0 0 0 -0.2"/>
    </body>
  </worldbody>
</mujoco>
)");

	UBlueprint* Blueprint = ParseScratch(*this, Xml);
	if (Blueprint == nullptr)
	{
		return false;
	}

	UMjGeomBase* Inherited = FindTemplate<UMjGeomBase>(*Blueprint, TEXT("inherited"));
	UMjGeomBase* Both = FindTemplate<UMjGeomBase>(*Blueprint, TEXT("both"));
	if (!TestNotNull(TEXT("geom inherited"), Inherited) || !TestNotNull(TEXT("geom both"), Both))
	{
		return false;
	}

	TestFalse(TEXT("inherited has no local endpoints or pose"), Inherited->HasFromto() || Inherited->HasPos());
	TestFalse(TEXT("inherited has no local half-extents"), Inherited->MjSpatialHalfExtentsCm.IsSet());
	TestFalse(TEXT("local endpoints normalized"), Both->HasFromto());
	TestTrue(TEXT("local endpoints become pose and half-extents"), Both->HasPos()
		&& Both->MjSpatialHalfExtentsCm.IsSet());

	TArray<FMjSpecDiagnostic> WriteErrors;
	const FString Written = FSpecRef::OverBlueprint(*Blueprint).WriteMjcf(&WriteErrors);
	TestEqual(TEXT("write produced no diagnostics"), WriteErrors.Num(), 0);

	// No fromto survives into exported MJCF, including its default class.
	TestFalse(TEXT("no fromto survives anywhere, the <default> tree included"),
		Written.Contains(TEXT("fromto")));

	// And MuJoCo agrees the model did not change.
	GeomFramesAgree(*this, TEXT("inherited"), Xml, Written);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjFrameTypesFromtoInertFieldsTest,
	"URLab.Gen.FrameTypes.FromtoIgnoresAuthoredQuatAndLength",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjFrameTypesFromtoInertFieldsTest::RunTest(const FString& Parameters)
{
	using namespace MjFrameTypeTests;
	// Native MuJoCo derives orientation and axial half-length from fromto even
	// when those otherwise-authored fields are present. The first size remains
	// the capsule radius.
	const FString Xml = TEXT(R"(<mujoco model="inert_fromto_fields">
  <compiler angle="radian"/>
  <worldbody><body name="b">
    <geom name="g" type="capsule" fromto="0 0 0 0 0 0.3" quat="0 1 0 0" size="0.05 0.9"/>
  </body></worldbody>
</mujoco>)");
	UBlueprint* Blueprint = ParseScratch(*this, Xml);
	if (Blueprint == nullptr)
	{
		return false;
	}
	UMjGeomBase* Geom = FindTemplate<UMjGeomBase>(*Blueprint, TEXT("g"));
	if (!TestNotNull(TEXT("geom g"), Geom))
	{
		return false;
	}
	TestFalse(TEXT("import normalizes fromto"), Geom->HasFromto());
	TestTrue(TEXT("import stores canonical pose and half-extents"), Geom->HasPos()
		&& Geom->MjSpatialHalfExtentsCm.IsSet());
	TArray<FMjSpecDiagnostic> Errors;
	const FString Written = FSpecRef::OverBlueprint(*Blueprint).WriteMjcf(&Errors);
	if (!TestEqual(TEXT("write produced no diagnostics"), Errors.Num(), 0) || Written.IsEmpty())
	{
		return false;
	}
	GeomFramesAgree(*this, TEXT("inert_fromto_fields"), Xml, Written);
	return true;
}

// ============================================================================
// URLab.Gen.FrameTypes.FromtoFoldDeclinedKeepsClass
//   A class read by an incompatible geom must fail import explicitly.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjFrameTypesFromtoDeclinedTest,
	"URLab.Gen.FrameTypes.FromtoFoldDeclinedKeepsClass",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjFrameTypesFromtoDeclinedTest::RunTest(const FString& Parameters)
{
	using namespace MjFrameTypeTests;

	// A sphere does not admit `fromto`. MuJoCo rejects this model, and it has to
	// go on rejecting it after a round trip.
	const FString Xml = TEXT(R"(<mujoco model="declined">
  <compiler angle="radian"/>
  <default>
    <default class="knob">
      <geom fromto="0 0 0 0 0 0.2"/>
    </default>
  </default>
  <worldbody>
    <body name="b">
      <geom name="ball" class="knob" type="sphere" size="0.03"/>
    </body>
  </worldbody>
</mujoco>
)");

	UBlueprint* Blueprint = MakeScratchBlueprint();
	if (!TestNotNull(TEXT("scratch Blueprint"), Blueprint)) return false;
	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, Xml, TEXT("<inline>"));
	TestFalse(TEXT("invalid inherited fromto is rejected at import"), Parsed.IsOk());
	TestTrue(TEXT("import gives a diagnostic"), Parsed.Errors.Num() > 0);

	// Native MuJoCo must agree with the rejection.
	auto CompileError = [&](const FString& Text) -> FString {
		const FString Path = FPaths::Combine(FPaths::ProjectIntermediateDir(),
			FString::Printf(TEXT("MjFromtoDeclined_%s.xml"), *FGuid::NewGuid().ToString(EGuidFormats::Digits)));
		if (!FFileHelper::SaveStringToFile(Text, *Path, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
		{
			return TEXT("<could not stage>");
		}
		char Error[1024] = {0};
		mjModel* Model = mj_loadXML(TCHAR_TO_UTF8(*Path), nullptr, Error, sizeof(Error));
		IFileManager::Get().Delete(*Path);
		if (Model != nullptr)
		{
			mj_deleteModel(Model);
			return FString();
		}
		return UTF8_TO_TCHAR(Error);
	};

	const FString OriginalError = CompileError(Xml);
	TestTrue(TEXT("MuJoCo rejects the original over the geom type"),
		OriginalError.Contains(TEXT("fromto requires capsule")));

	return true;
}

// ============================================================================
// URLab.Gen.FrameTypes.FromtoFoldIsBitExact
//   The fold writes the pose into the spec, so the compiled model is only
//   unchanged if the arithmetic is the engine's to the last bit. A capsule
//   square to an axis is the case that catches a reimplementation: its cross
//   product comes out exactly unit-length, and MuJoCo's mjuu_normvec leaves an
//   already-unit vector alone rather than dividing it by itself.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjFrameTypesFromtoBitExactTest, "URLab.Gen.FrameTypes.FromtoFoldIsBitExact",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjFrameTypesFromtoBitExactTest::RunTest(const FString& Parameters)
{
	using namespace MjFrameTypeTests;

	const FString Xml = TEXT(R"(<mujoco model="exact">
  <compiler angle="radian"/>
  <worldbody>
    <body name="b">
      <geom name="torso" type="capsule" fromto="0 -.07 0 0 .07 0" size=".07"/>
      <geom name="upper_arm" type="capsule" fromto="0 0 0 .16 -.16 -.16" size=".04"/>
      <geom name="thigh" type="capsule" fromto="0 0 0 0 .01 -.34" size=".06"/>
    </body>
  </worldbody>
</mujoco>
)");

	UBlueprint* Blueprint = ParseScratch(*this, Xml);
	if (Blueprint == nullptr)
	{
		return false;
	}

	TArray<FMjSpecDiagnostic> WriteErrors;
	const FString Written = FSpecRef::OverBlueprint(*Blueprint).WriteMjcf(&WriteErrors);
	TestEqual(TEXT("write produced no diagnostics"), WriteErrors.Num(), 0);

	mjModel* Original = CompileWithMuJoCo(*this, TEXT("exact_original"), Xml);
	if (Original == nullptr)
	{
		return false;
	}
	mjModel* Rewritten = CompileWithMuJoCo(*this, TEXT("exact_written"), Written);
	if (Rewritten == nullptr)
	{
		mj_deleteModel(Original);
		return false;
	}

	if (TestEqual(TEXT("same geom count"), Rewritten->ngeom, Original->ngeom))
	{
		// Exactly equal, not nearly: a tolerance here is what let a one-ULP
		// divergence into every inertia in the model.
		for (int Index = 0; Index < 4 * Original->ngeom; ++Index)
		{
			TestTrue(*FString::Printf(TEXT("geom_quat[%d] is bit-exact (%.17g vs %.17g)"), Index,
						 Original->geom_quat[Index], Rewritten->geom_quat[Index]),
				Original->geom_quat[Index] == Rewritten->geom_quat[Index]);
		}
		for (int Index = 0; Index < 3 * Original->ngeom; ++Index)
		{
			// Half-extents persist in UE centimetres; the metres->cm->metres
			// conversion can move the last bit, as it does for the thigh here.
			TestTrue(*FString::Printf(TEXT("geom_size[%d] differs by at most 4e-17 (%.17g vs %.17g; delta %.17g)"), Index,
				Original->geom_size[Index], Rewritten->geom_size[Index],
				Rewritten->geom_size[Index] - Original->geom_size[Index]),
				FMath::Abs(Original->geom_size[Index] - Rewritten->geom_size[Index]) <= 4e-17);
			TestTrue(*FString::Printf(TEXT("geom_pos[%d] is bit-exact"), Index),
				Original->geom_pos[Index] == Rewritten->geom_pos[Index]);
		}
	}

	mj_deleteModel(Original);
	mj_deleteModel(Rewritten);
	return true;
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
