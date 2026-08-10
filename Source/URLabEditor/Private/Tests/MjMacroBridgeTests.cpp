// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// The macro bridge, held to the only standard that means anything for it: the
// model it produces has to be the model MuJoCo's own reader produces from the
// same document.
//
// `<composite>`, `<flexcomp>` and `<replicate>` are expanded by MuJoCo's reader
// rather than by its compiler, so the spec write cannot perform one. It
// serialises the macro into a wrapper document, parses it, and attaches the
// expansion. Every part of that is a place to get it subtly wrong -- a class the
// wrapper failed to carry, an angle unit it failed to carry, a carrier that
// leaves a body behind -- and none of those show up as an error. They show up as
// a model that is nearly right, which is why the comparison is field for field
// at exact tolerance rather than a spot check.
//
// The three macro kinds are covered against the recorded goldens by
// URLab.Parity.SpecGoldens, whose corpus carries a fixture for each. What is
// left here is the pair of conditions the wrapper has to satisfy that no corpus
// fixture poses, each written as a document of its own so that the corpus and
// its goldens are left alone.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/Blueprint.h"
#include "GameFramework/Actor.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "UObject/Package.h"

#include "MuJoCo/Spec/MjSpecBuild.h"
#include "MuJoCo/Spec/MjSpecRef.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>

#include "model_diff_lib.h"
THIRD_PARTY_INCLUDES_END

namespace MjMacroBridgeTests
{

/** Indices sampled per differing field, and entries listed per category. */
constexpr int32 MaxExamples = 4;

FString DiagnosticsToString(const TArray<FMjSpecDiagnostic>& Diagnostics)
{
	TArray<FString> Lines;
	for (const FMjSpecDiagnostic& Diagnostic : Diagnostics)
	{
		Lines.Add(Diagnostic.ToString());
	}
	return FString::Join(Lines, TEXT("; "));
}

FString Utf8ToUe(const std::string& Text)
{
	return FString(UTF8_TO_TCHAR(Text.c_str()));
}

/** A throwaway Blueprint in the transient package; nothing reaches disk. */
UBlueprint* MakeScratchBlueprint()
{
	const FString Name = FString::Printf(TEXT("MjMacroBridge_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	UPackage* Package = CreatePackage(*(TEXT("/Temp/") + Name));
	return FKismetEditorUtilities::CreateBlueprint(AActor::StaticClass(), Package, FName(*Name), BPTYPE_Normal,
		UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
}

UBlueprint* ParseDocument(FAutomationTestBase& Test, const FString& Label, const FString& Xml, const FString& Path)
{
	UBlueprint* Blueprint = MakeScratchBlueprint();
	if (Blueprint == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("%s: could not create a scratch Blueprint"), *Label));
		return nullptr;
	}
	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, Xml, Path);
	if (!Parsed.IsOk())
	{
		Test.AddError(FString::Printf(TEXT("%s: parse failed: %s"), *Label, *DiagnosticsToString(Parsed.Errors)));
		return nullptr;
	}
	return Blueprint;
}

/** The whole verdict, rendered for a failure message. */
FString ReportToString(const ps::harness::DiffReport& Report)
{
	TArray<FString> Lines;
	Lines.Add(FString::Printf(TEXT("  first divergence: %s"), *Utf8ToUe(Report.FirstDivergence())));

	int32 Listed = 0;
	for (const ps::harness::SizeDiff& Size : Report.sizes)
	{
		if (Listed++ >= MaxExamples)
		{
			break;
		}
		Lines.Add(FString::Printf(TEXT("  size %s: reader %lld, bridge %lld"), *Utf8ToUe(Size.name),
			static_cast<int64>(Size.a), static_cast<int64>(Size.b)));
	}

	Listed = 0;
	for (const ps::harness::NameDiff& Name : Report.names)
	{
		if (Listed++ >= MaxExamples)
		{
			break;
		}
		Lines.Add(FString::Printf(TEXT("  name %s[%d]: reader '%s', bridge '%s'"), *Utf8ToUe(Name.objtype),
			Name.id, *Utf8ToUe(Name.a), *Utf8ToUe(Name.b)));
	}

	Listed = 0;
	for (const ps::harness::FieldDiff& Field : Report.fields)
	{
		if (Listed++ >= MaxExamples)
		{
			break;
		}
		Lines.Add(FString::Printf(TEXT("  field %s: %lld of %lld differ"), *Utf8ToUe(Field.field),
			static_cast<int64>(Field.num_diff), static_cast<int64>(Field.count_a)));
	}

	Listed = 0;
	for (const ps::harness::FieldDiff& Field : Report.invariants)
	{
		if (Listed++ >= MaxExamples)
		{
			break;
		}
		Lines.Add(FString::Printf(TEXT("  invariant %s: %lld of %lld differ"), *Utf8ToUe(Field.field),
			static_cast<int64>(Field.num_diff), static_cast<int64>(Field.count_a)));
	}
	return FString::Join(Lines, TEXT("\n"));
}

/**
 * MuJoCo's own reader over `Xml`, with nothing of ours in between.
 *
 * The reference the bridge is held to. Mounted rather than written to disk, so
 * the document under test is exactly the string above it in the file.
 */
mjModel* LoadThroughStockReader(FAutomationTestBase& Test, const FString& Label, const FString& Xml)
{
	mjVFS Vfs;
	mj_defaultVFS(&Vfs);

	const char* const Name = "__urlab_macro.xml";
	const FTCHARToUTF8 Utf8(*Xml);
	mj_addBufferVFS(&Vfs, Name, Utf8.Get(), Utf8.Length());

	char Error[1024] = {0};
	mjModel* const Model = mj_loadXML(Name, &Vfs, Error, sizeof(Error));
	mj_deleteVFS(&Vfs);

	if (Model == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("%s: MuJoCo's reader declined the document: %hs"), *Label, Error));
	}
	return Model;
}

/**
 * Read `Xml` into a spec, build it, and require the model to be MuJoCo's.
 *
 * The reference is the reader rather than any writer of ours, so nothing in the
 * comparison depends on our own MJCF being spelled a particular way. The
 * document names every bindable element, so no reservation lands on either
 * side and the name table is part of what has to agree.
 */
void CheckBridgeMatchesStockReader(
	FAutomationTestBase& Test, const FString& Label, const FString& Xml, const FString& Path)
{
	UBlueprint* const Blueprint = ParseDocument(Test, Label, Xml, Path);
	if (Blueprint == nullptr)
	{
		return;
	}
	const FSpecRef Ref = FSpecRef::OverBlueprint(*Blueprint);

	mjModel* const ViaReader = LoadThroughStockReader(Test, Label, Xml);
	if (ViaReader == nullptr)
	{
		return;
	}

	TArray<FMjSpecDiagnostic> Diagnostics;
	urlab::spec::FMjBuiltSpec Built = urlab::spec::BuildSpec(Ref, Diagnostics);
	if (Built.Spec == nullptr)
	{
		Test.AddError(FString::Printf(
			TEXT("%s: the spec write did not build: %s"), *Label, *DiagnosticsToString(Diagnostics)));
		mj_deleteModel(ViaReader);
		return;
	}

	mjModel* const ViaBridge = mj_compile(Built.Spec, nullptr);
	if (ViaBridge == nullptr)
	{
		Test.AddError(FString::Printf(
			TEXT("%s: the bridged spec did not compile: %s"), *Label, UTF8_TO_TCHAR(mjs_getError(Built.Spec))));
		mj_deleteModel(ViaReader);
		return;
	}

	std::string Err;
	const ps::harness::DiffReport Report =
		ps::harness::DiffModels(ViaReader, ViaBridge, ps::harness::Tol{0.0, 0.0}, MaxExamples, Err);
	mj_deleteModel(ViaBridge);
	mj_deleteModel(ViaReader);

	if (!Err.empty())
	{
		Test.AddError(FString::Printf(TEXT("%s: model comparison did not complete: %s"), *Label, *Utf8ToUe(Err)));
	}
	if (Report.Differs())
	{
		Test.AddError(FString::Printf(
			TEXT("%s: the macro bridge produced a different model from MuJoCo's reader:\n%s"), *Label,
			*ReportToString(Report)));
	}
}

} // namespace MjMacroBridgeTests

// ============================================================================
// URLab.MuJoCo.MacroBridge.CompilerCarriage
//   The wrapper carries the participant's <compiler>, so a macro authored in
//   degrees expands in degrees. Drop the carriage and the expansion silently
//   comes back in radians, which is a model that compiles and is wrong.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjMacroBridgeCompilerCarriageTest, "URLab.MuJoCo.MacroBridge.CompilerCarriage",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjMacroBridgeCompilerCarriageTest::RunTest(const FString& Parameters)
{
	using namespace MjMacroBridgeTests;

	const FString Xml = TEXT(R"(<mujoco model="macro_degrees">
  <compiler angle="degree"/>
  <worldbody>
    <body name="host" pos="0 0 1" euler="0 0 30">
      <geom name="host_geom" type="sphere" size="0.02"/>
      <replicate count="3" euler="0 0 45">
        <body name="rep_body" pos="0.2 0 0" euler="15 0 0">
          <joint name="rep_joint" type="hinge" axis="0 0 1"/>
          <geom name="rep_geom" type="capsule" fromto="0 0 0 0.1 0 0" size="0.01"/>
        </body>
      </replicate>
    </body>
  </worldbody>
</mujoco>)");

	CheckBridgeMatchesStockReader(*this, TEXT("macro_degrees"), Xml, TEXT("macro_degrees.xml"));
	return true;
}

// ============================================================================
// URLab.MuJoCo.MacroBridge.Assets
//   The wrapper carries the participant's asset section MINUS the names the
//   target spec already holds. Since the asset section is written before any
//   body, that difference is empty and the wrapper carries no assets at all --
//   which is what makes the bridge work, because mjs_attach rejects a repeated
//   asset name under an empty prefix. What is asserted is the relation itself:
//   every authored asset is in the built spec exactly once, and the macro
//   subtree's references to them resolve.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjMacroBridgeAssetsTest, "URLab.MuJoCo.MacroBridge.Assets",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjMacroBridgeAssetsTest::RunTest(const FString& Parameters)
{
	using namespace MjMacroBridgeTests;

	const FString Xml = TEXT(R"(<mujoco model="macro_assets">
  <asset>
    <mesh name="wedge" vertex="0 0 0  0.1 0 0  0 0.1 0  0 0 0.1"/>
    <texture name="grid" type="2d" builtin="checker" width="8" height="8"/>
    <material name="grid_mat" texture="grid"/>
  </asset>
  <worldbody>
    <body name="host" pos="0 0 1">
      <geom name="host_geom" type="mesh" mesh="wedge" material="grid_mat"/>
      <replicate count="3" offset="0.2 0 0">
        <body name="rep_body">
          <geom name="rep_geom" type="mesh" mesh="wedge" material="grid_mat"/>
        </body>
      </replicate>
    </body>
  </worldbody>
</mujoco>)");

	UBlueprint* const Blueprint = ParseDocument(*this, TEXT("macro_assets"), Xml, TEXT("macro_assets.xml"));
	if (Blueprint == nullptr)
	{
		return true;
	}

	TArray<FMjSpecDiagnostic> Diagnostics;
	urlab::spec::FMjBuiltSpec Built = urlab::spec::BuildSpec(FSpecRef::OverBlueprint(*Blueprint), Diagnostics);
	if (Built.Spec == nullptr)
	{
		AddError(FString::Printf(TEXT("macro_assets: the spec write did not build: %s"),
			*DiagnosticsToString(Diagnostics)));
		return true;
	}

	// One element per authored name, in the namespace mjs_attach collides on. A
	// wrapper that carried an asset the target already held would have failed the
	// attach outright, so reaching a built spec at all is half the assertion; the
	// count is the other half.
	struct FExpectedAsset
	{
		mjtObj Kind;
		const char* Name;
	};
	const FExpectedAsset Expected[] = {
		{mjOBJ_MESH, "wedge"},
		{mjOBJ_TEXTURE, "grid"},
		{mjOBJ_MATERIAL, "grid_mat"},
	};
	for (const FExpectedAsset& Asset : Expected)
	{
		TestTrue(FString::Printf(TEXT("the built spec holds '%s' once"), UTF8_TO_TCHAR(Asset.Name)),
			mjs_findElement(Built.Spec, Asset.Kind, Asset.Name) != nullptr);
	}

	int32 MeshCount = 0;
	for (mjsElement* Element = mjs_firstElement(Built.Spec, mjOBJ_MESH); Element != nullptr;
		 Element = mjs_nextElement(Built.Spec, Element))
	{
		++MeshCount;
	}
	TestEqual(TEXT("the bridge added no second copy of the participant's mesh"), MeshCount, 1);

	// The macro subtree's own references resolve against the target's assets,
	// which is what makes leaving them out of the wrapper safe.
	mjModel* const Model = mj_compile(Built.Spec, nullptr);
	if (Model == nullptr)
	{
		AddError(FString::Printf(TEXT("macro_assets: the bridged spec did not compile: %s"),
			UTF8_TO_TCHAR(mjs_getError(Built.Spec))));
		return true;
	}
	TestEqual(TEXT("one mesh in the compiled model"), static_cast<int32>(Model->nmesh), 1);
	TestEqual(TEXT("one material in the compiled model"), static_cast<int32>(Model->nmat), 1);
	TestEqual(TEXT("the replicate expanded"), static_cast<int32>(Model->ngeom), 4);
	mj_deleteModel(Model);

	return true;
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
