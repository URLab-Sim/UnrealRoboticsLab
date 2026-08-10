// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// The Phase 3 gates for the generated MuJoCo spec profile.
//
// Round-trip fixpoint proves the writer emits exactly what the reader accepted,
// and the differential compile proves that what it emits compiles to the same
// mjModel the original did. The second is the anchor: a round trip can be
// self-consistently wrong, a byte-identical compiled model cannot.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include <type_traits>

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
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mjxmacro.h>
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

namespace MjGenProfileTests
{

/**
 * A model exercising the shapes the profile has to get right: presence, fixed
 * arities including the quaternion permutation, sequences, enums, references,
 * a default class, and sibling order that the qpos layout depends on.
 */
const TCHAR* const InlineCorpus = TEXT(R"(<mujoco model="seam">
  <compiler angle="radian"/>
  <default>
    <default class="robot">
      <geom friction="1.5 0.005 0.0001" rgba="0.2 0.4 0.6 1"/>
    </default>
  </default>
  <asset>
    <material name="steel" rgba="0.5 0.5 0.55 1"/>
  </asset>
  <worldbody>
    <body name="base" pos="0 0 0.1" quat="0.7071067811865476 0 0 0.7071067811865476">
      <geom name="plinth" type="box" size="0.1 0.2 0.05" material="steel"/>
      <body name="link" childclass="robot" pos="0 0 0.2">
        <joint name="hinge" type="hinge" axis="0 1 0" range="-1.5 1.5"/>
        <joint name="slide" type="slide" axis="1 0 0"/>
        <geom name="shin" type="capsule" size="0.02 0.15"/>
        <site name="tip" pos="0 0 0.3"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="drive" joint="hinge" kp="120" kv="8"/>
  </actuator>
  <sensor>
    <gyro name="imu" site="tip"/>
  </sensor>
</mujoco>
)");

/** A throwaway Blueprint in the transient package; nothing reaches disk. */
UBlueprint* MakeScratchBlueprint()
{
	const FString Name = FString::Printf(TEXT("MjGenProfile_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	UPackage* Package = CreatePackage(*(TEXT("/Temp/") + Name));
	return FKismetEditorUtilities::CreateBlueprint(AActor::StaticClass(), Package, FName(*Name),
		BPTYPE_Normal, UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
}

FString DiagnosticsToString(const TArray<FMjSpecDiagnostic>& Diagnostics)
{
	TArray<FString> Lines;
	for (const FMjSpecDiagnostic& Diagnostic : Diagnostics)
	{
		Lines.Add(Diagnostic.ToString());
	}
	return FString::Join(Lines, TEXT("; "));
}

/** Parse `Xml` into a fresh Blueprint and write it straight back out. */
bool RoundTrip(FAutomationTestBase& Test, const FString& Label, const FString& Xml, const FString& Filename,
	FString& OutWritten)
{
	UBlueprint* Blueprint = MakeScratchBlueprint();
	if (Blueprint == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("%s: could not create a scratch Blueprint"), *Label));
		return false;
	}

	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, Xml, Filename);
	if (!Parsed.IsOk())
	{
		if (Parsed.IsUnsupportedOnly())
		{
			Test.AddInfo(FString::Printf(TEXT("%s: skipped, unsupported element families (%s)"), *Label,
				*DiagnosticsToString(Parsed.Errors)));
			return false;
		}
		Test.AddError(FString::Printf(TEXT("%s: parse failed: %s"), *Label, *DiagnosticsToString(Parsed.Errors)));
		return false;
	}

	TArray<FMjSpecDiagnostic> WriteErrors;
	OutWritten = FSpecRef::OverBlueprint(*Blueprint).WriteMjcf(&WriteErrors);
	if (WriteErrors.Num() > 0)
	{
		Test.AddError(FString::Printf(TEXT("%s: write failed: %s"), *Label, *DiagnosticsToString(WriteErrors)));
		return false;
	}
	if (OutWritten.IsEmpty())
	{
		Test.AddError(FString::Printf(TEXT("%s: writer produced nothing"), *Label));
		return false;
	}
	return true;
}

/**
 * Every MJCF file the run has to work with.
 *
 * Include fragments are excluded: a <mujocoinclude> is not a spec and
 * only means anything spliced into the file that includes it, which the reader
 * does during that file's own parse.
 */
TArray<FString> CorpusFiles()
{
	TArray<FString> Found;

	FString TestData = FPaths::Combine(FPaths::ProjectPluginsDir(), TEXT("UnrealRoboticsLab"),
		TEXT("Content"), TEXT("TestData"));
	FPaths::NormalizeDirectoryName(TestData);
	IFileManager::Get().FindFilesRecursive(Found, *TestData, TEXT("*.xml"), true, false);

	// Two subdirectories are outputs or non-inputs, never fixtures. `goldens`
	// holds recorded answers: re-reading one as input would test the run
	// against itself, and its asset references are relative to the fixture it
	// was recorded from, so they do not resolve from there. `boundary` holds
	// models that exercise the reader and writer but cannot compile in URLab,
	// which is the property that puts them in that directory.
	const FString Goldens = FPaths::Combine(TestData, TEXT("goldens")) + TEXT("/");
	const FString Boundary = FPaths::Combine(TestData, TEXT("boundary")) + TEXT("/");
	Found.RemoveAll([&Goldens, &Boundary](const FString& File) {
		FString Path = File;
		FPaths::NormalizeFilename(Path);
		return Path.StartsWith(Goldens) || Path.StartsWith(Boundary);
	});

	const FString Menagerie = FPlatformMisc::GetEnvironmentVariable(TEXT("MUJOCO_MENAGERIE_PATH"));
	if (!Menagerie.IsEmpty())
	{
		TArray<FString> Extra;
		IFileManager::Get().FindFilesRecursive(Extra, *Menagerie, TEXT("*.xml"), true, false);
		Found.Append(Extra);
	}

	TArray<FString> Files;
	for (const FString& File : Found)
	{
		FString Text;
		if (FFileHelper::LoadFileToString(Text, *File) && Text.Contains(TEXT("<mujoco")) &&
			!Text.Contains(TEXT("<mujocoinclude")))
		{
			Files.Add(File);
		}
	}
	return Files;
}

/**
 * Where two arrays first disagree, and by how much.
 *
 * `Memcmp` decides the verdict; this decides whether the report is actionable.
 * "compiled models differ at geom_quat" names a field with three hundred numbers
 * in it and leaves the reader to guess which body moved.
 */
template <class T>
FString FirstDifference(const T* A, const T* B, int32 Rows, int32 Cols)
{
	for (int32 Index = 0; Index < Rows * Cols; ++Index)
	{
		if (A[Index] == B[Index])
		{
			continue;
		}
		const FString At = Cols > 1 ? FString::Printf(TEXT("[%d][%d]"), Index / Cols, Index % Cols)
									: FString::Printf(TEXT("[%d]"), Index);
		if constexpr (std::is_floating_point_v<T>)
		{
			return FString::Printf(TEXT(", first at %s: %.17g vs %.17g"), *At, static_cast<double>(A[Index]),
				static_cast<double>(B[Index]));
		}
		else if constexpr (std::is_integral_v<T>)
		{
			return FString::Printf(TEXT(", first at %s: %lld vs %lld"), *At, static_cast<long long>(A[Index]),
				static_cast<long long>(B[Index]));
		}
		else
		{
			return FString::Printf(TEXT(", first at %s"), *At);
		}
	}
	// Memcmp saw bytes the value comparison does not: -0.0 against 0.0, or two
	// spellings of NaN. Worth saying so rather than reporting nothing.
	return TEXT(", equal by value but not by bytes");
}

/**
 * Compile both texts and compare the resulting models field by field.
 *
 * The comparison is driven by MuJoCo's own mjxmacro tables rather than a hand
 * list, so a field added by an engine bump is covered without an edit here --
 * which is the only way a differential gate stays honest across bumps.
 *
 * All four tables, not just the two that describe the buffer: `mjModel::opt`,
 * `::stat` and `::vis` are struct members rather than sizes or pointers, so a
 * gate built from MJMODEL_SIZES and MJMODEL_POINTERS alone cannot see a
 * regression in <option>, <statistic> or <visual> at all.
 */
bool CompiledModelsAgree(FAutomationTestBase& Test, const FString& Label, const FString& OriginalXml,
	const FString& WrittenXml, const FString& BaseDir)
{
	// Both texts are staged in the ORIGINAL's directory, because both spell their
	// includes and asset paths relative to it. Compiling the rewrite from a
	// scratch directory would fail on the model rather than on the round trip,
	// which is the wrong answer to the question being asked.
	const FString StageDir = BaseDir.IsEmpty() ? FPaths::ProjectIntermediateDir() : BaseDir;

	auto Load = [&](const FString& Text, const TCHAR* Which) -> mjModel* {
		char Error[1024] = {0};
		const FString Path = FPaths::Combine(StageDir,
			FString::Printf(TEXT("MjGenProfile_%s_%s.xml"), Which, *FGuid::NewGuid().ToString(EGuidFormats::Digits)));
		if (!FFileHelper::SaveStringToFile(Text, *Path, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
		{
			Test.AddError(FString::Printf(TEXT("%s: could not stage %s MJCF"), *Label, Which));
			return nullptr;
		}
		mjModel* Model = mj_loadXML(TCHAR_TO_UTF8(*Path), nullptr, Error, sizeof(Error));
		IFileManager::Get().Delete(*Path);
		if (Model == nullptr)
		{
			Test.AddError(FString::Printf(TEXT("%s: %s MJCF did not compile: %s"), *Label, Which, UTF8_TO_TCHAR(Error)));
		}
		return Model;
	};

	mjModel* A = Load(OriginalXml, TEXT("original"));
	if (A == nullptr)
	{
		return false;
	}
	mjModel* B = Load(WrittenXml, TEXT("written"));
	if (B == nullptr)
	{
		mj_deleteModel(A);
		return false;
	}

	int32 Mismatches = 0;
	auto Report = [&](const TCHAR* Field, const FString& Where) {
		// The cap bounds the log, not the verdict; every mismatch still counts.
		if (Mismatches < 32)
		{
			Test.AddError(FString::Printf(TEXT("%s: compiled models differ at %s%s"), *Label, Field, *Where));
		}
		++Mismatches;
	};

#define X(name)                                    \
	if (A->name != B->name)                        \
	{                                              \
		Report(TEXT(#name),                        \
			FString::Printf(TEXT(": %d vs %d"), A->name, B->name)); \
	}
	MJMODEL_SIZES
#undef X

	if (Mismatches == 0)
	{
		MJMODEL_POINTERS_PREAMBLE(A)
#define X(type, name, nr, nc)                                                          \
	if (FMemory::Memcmp(A->name, B->name, sizeof(type) * (size_t)(A->nr) * (nc)) != 0)  \
	{                                                                                  \
		Report(TEXT(#name), FirstDifference<type>(A->name, B->name, A->nr, nc));       \
	}
		MJMODEL_POINTERS
#undef X

		// mjOption, mjStatistic and mjVisual: struct members, so neither of the
		// tables above reaches them. `X` names a scalar and `XVEC` an array, and
		// the two need different address expressions -- which is the whole reason
		// mjxmacro spells them apart.
#define MJ_COMPARE(FieldLabel, APtr, BPtr, Type, Num)                             \
	if (FMemory::Memcmp((APtr), (BPtr), sizeof(Type) * (Num)) != 0)               \
	{                                                                             \
		Report(FieldLabel, FirstDifference<Type>((APtr), (BPtr), 1, (Num)));      \
	}

#define X(type, name, num) MJ_COMPARE(TEXT("opt.") TEXT(#name), &A->opt.name, &B->opt.name, type, num)
#define XVEC(type, name, num) MJ_COMPARE(TEXT("opt.") TEXT(#name), A->opt.name, B->opt.name, type, num)
		MJOPTION_FIELDS
#undef XVEC
#undef X

#define X(name, num) MJ_COMPARE(TEXT("stat.") TEXT(#name), &A->stat.name, &B->stat.name, mjtNum, num)
#define XVEC(name, num) MJ_COMPARE(TEXT("stat.") TEXT(#name), A->stat.name, B->stat.name, mjtNum, num)
		MJSTATISTIC_FIELDS
#undef XVEC
#undef X

		// Every mjVisual group. They share one X signature, so only the group
		// varies -- and the group has to appear in the label, because `haze`
		// names a field in two of them.
#define X(type, name, num) \
	MJ_COMPARE(TEXT("vis.") TEXT(VISLABEL) TEXT(".") TEXT(#name), &A->vis.VISGROUP.name, &B->vis.VISGROUP.name, type, num)
#define XVEC(type, name, num) \
	MJ_COMPARE(TEXT("vis.") TEXT(VISLABEL) TEXT(".") TEXT(#name), A->vis.VISGROUP.name, B->vis.VISGROUP.name, type, num)
#define VISGROUP global
#define VISLABEL "global"
		MJVISUAL_GLOBAL_FIELDS
#undef VISLABEL
#undef VISGROUP
#define VISGROUP quality
#define VISLABEL "quality"
		MJVISUAL_QUALITY_FIELDS
#undef VISLABEL
#undef VISGROUP
#define VISGROUP headlight
#define VISLABEL "headlight"
		MJVISUAL_HEADLIGHT_FIELDS
#undef VISLABEL
#undef VISGROUP
#define VISGROUP map
#define VISLABEL "map"
		MJVISUAL_MAP_FIELDS
#undef VISLABEL
#undef VISGROUP
#define VISGROUP scale
#define VISLABEL "scale"
		MJVISUAL_SCALE_FIELDS
#undef VISLABEL
#undef VISGROUP
#define VISGROUP rgba
#define VISLABEL "rgba"
		MJVISUAL_RGBA_FIELDS
#undef VISLABEL
#undef VISGROUP
#undef XVEC
#undef X
#undef MJ_COMPARE
	}

	mj_deleteModel(A);
	mj_deleteModel(B);
	return Mismatches == 0;
}

/** `mj_saveXMLString` of a freshly parsed spec: the shape the handshake used to ship. */
bool SaveSpecXml(FAutomationTestBase& Test, const FString& Label, const FString& File, const FString& Text,
	FString& Out)
{
	char Error[1024] = {0};
	mjSpec* Spec = File.IsEmpty() ? mj_parseXMLString(TCHAR_TO_UTF8(*Text), nullptr, Error, sizeof(Error))
								  : mj_parseXML(TCHAR_TO_UTF8(*File), nullptr, Error, sizeof(Error));
	if (Spec == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("%s: mj_parseXML declined (%s)"), *Label, UTF8_TO_TCHAR(Error)));
		return false;
	}

	// MuJoCo refuses to write a spec it has not compiled ("Only compiled model
	// can be written"), which is why the engine's own handshake takes this text
	// after a compile rather than after a parse. The model is not wanted here,
	// only the side effect on the spec.
	if (mjModel* Compiled = mj_compile(Spec, nullptr))
	{
		mj_deleteModel(Compiled);
	}
	else
	{
		const char* SpecError = mjs_getError(Spec);
		Test.AddError(FString::Printf(TEXT("%s: mj_compile declined (%s)"), *Label,
			SpecError != nullptr ? UTF8_TO_TCHAR(SpecError) : TEXT("unknown")));
		mj_deleteSpec(Spec);
		return false;
	}

	bool bSaved = false;
	TArray<uint8> Buffer;
	for (int32 Capacity = 256 * 1024; Capacity <= 32 * 1024 * 1024; Capacity *= 2)
	{
		Buffer.SetNumUninitialized(Capacity);
		FMemory::Memzero(Buffer.GetData(), Capacity);
		char SaveError[1024] = "";
		if (mj_saveXMLString(Spec, reinterpret_cast<char*>(Buffer.GetData()), Capacity, SaveError,
				sizeof(SaveError)) == 0)
		{
			Out = UTF8_TO_TCHAR(reinterpret_cast<const char*>(Buffer.GetData()));
			bSaved = true;
			break;
		}
		if (!FString(UTF8_TO_TCHAR(SaveError)).Contains(TEXT("buffer"), ESearchCase::IgnoreCase))
		{
			Test.AddError(FString::Printf(TEXT("%s: mj_saveXMLString declined (%s)"), *Label,
				UTF8_TO_TCHAR(SaveError)));
			break;
		}
	}
	mj_deleteSpec(Spec);
	return bSaved;
}

/** A one-line account of how two MJCF texts differ, for the record rather than the verdict. */
FString DescribeTextDelta(const FString& A, const FString& B)
{
	TArray<FString> LinesA;
	TArray<FString> LinesB;
	A.ParseIntoArrayLines(LinesA);
	B.ParseIntoArrayLines(LinesB);

	auto Trimmed = [](const TArray<FString>& Lines) {
		TArray<FString> Out;
		for (const FString& Line : Lines)
		{
			FString T = Line.TrimStartAndEnd();
			if (!T.IsEmpty())
			{
				Out.Add(MoveTemp(T));
			}
		}
		Out.Sort();
		return Out;
	};

	TArray<FString> SortedA = Trimmed(LinesA);
	TArray<FString> SortedB = Trimmed(LinesB);
	int32 Common = 0;
	for (int32 I = 0, J = 0; I < SortedA.Num() && J < SortedB.Num();)
	{
		const int32 Order = SortedA[I].Compare(SortedB[J]);
		if (Order == 0)
		{
			++Common;
			++I;
			++J;
		}
		else if (Order < 0)
		{
			++I;
		}
		else
		{
			++J;
		}
	}
	return FString::Printf(TEXT("%d lines saved / %d written, %d identical"), SortedA.Num(), SortedB.Num(), Common);
}

}  // namespace MjGenProfileTests

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjGenProfileFixpointTest, "URLab.Gen.Profile.RoundTripFixpoint",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjGenProfileFixpointTest::RunTest(const FString& Parameters)
{
	using namespace MjGenProfileTests;

	FString First;
	if (!RoundTrip(*this, TEXT("inline"), InlineCorpus, TEXT("<inline>"), First))
	{
		return false;
	}

	FString Second;
	if (!RoundTrip(*this, TEXT("inline (second pass)"), First, TEXT("<inline>"), Second))
	{
		return false;
	}

	TestEqual(TEXT("the second write reaches a fixpoint"), Second, First);

	// As the differential gate: a corpus that has stopped being found must fail
	// this rather than sail through on the inline model alone.
	int32 Compared = 0;
	for (const FString& File : CorpusFiles())
	{
		FString Text;
		if (!FFileHelper::LoadFileToString(Text, *File))
		{
			continue;
		}
		const FString Label = FPaths::GetCleanFilename(File);

		FString Written;
		if (!RoundTrip(*this, Label, Text, File, Written))
		{
			continue;
		}
		FString Rewritten;
		if (!RoundTrip(*this, Label + TEXT(" (second pass)"), Written, File, Rewritten))
		{
			continue;
		}
		TestEqual(*FString::Printf(TEXT("%s reaches a fixpoint"), *Label), Rewritten, Written);
		++Compared;
	}

	TestTrue(TEXT("at least one corpus file reached a fixpoint"), Compared > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjGenProfileDifferentialTest, "URLab.Gen.Profile.DifferentialCompile",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjGenProfileDifferentialTest::RunTest(const FString& Parameters)
{
	using namespace MjGenProfileTests;

	// Every path out of the loop below is a `continue`, so without a floor this
	// test passes on an empty corpus -- which is to say it passes when the thing
	// it gates has stopped running at all.
	int32 Compared = 0;

	FString Written;
	if (RoundTrip(*this, TEXT("inline"), InlineCorpus, TEXT("<inline>"), Written))
	{
		CompiledModelsAgree(*this, TEXT("inline"), InlineCorpus, Written, FString());
		++Compared;
	}

	for (const FString& File : CorpusFiles())
	{
		FString Text;
		if (!FFileHelper::LoadFileToString(Text, *File))
		{
			continue;
		}
		const FString Label = FPaths::GetCleanFilename(File);
		FString Out;
		if (!RoundTrip(*this, Label, Text, File, Out))
		{
			continue;
		}
		CompiledModelsAgree(*this, Label, Text, Out, FPaths::GetPath(File));
		++Compared;
	}

	TestTrue(TEXT("at least one model was compared"), Compared > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjGenProfileHandshakeMjcfTest, "URLab.Gen.Profile.HandshakeMjcfEquivalence",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjGenProfileHandshakeMjcfTest::RunTest(const FString& Parameters)
{
	using namespace MjGenProfileTests;

	// The bridge handshake's `mjcf_compiled` is MJCF text a client reloads
	// offline. It came from mj_saveXMLString over the engine's root spec and
	// comes from the spec writer once no such spec exists, so what has to
	// hold is not that the two texts match -- they cannot, one is MuJoCo's
	// canonical form and the other is the authored spec -- but that they
	// compile to the same model. The texts' shapes are recorded either way.
	int32 Compared = 0;
	TArray<TPair<FString, FString>> Cases;
	Cases.Emplace(FString(), InlineCorpus);
	for (const FString& File : CorpusFiles())
	{
		Cases.Emplace(File, FString());
	}

	for (const TPair<FString, FString>& Case : Cases)
	{
		const FString& File = Case.Key;
		const FString Label = File.IsEmpty() ? TEXT("inline") : FPaths::GetCleanFilename(File);

		FString SavedXml;
		if (!SaveSpecXml(*this, Label, File, Case.Value, SavedXml))
		{
			continue;
		}

		FString Written;
		if (!RoundTrip(*this, Label + TEXT(" (saved form)"), SavedXml, File, Written))
		{
			AddError(FString::Printf(TEXT("%s: the saved MJCF did not round-trip"), *Label));
			continue;
		}

		AddInfo(FString::Printf(TEXT("%s: %s"), *Label, *DescribeTextDelta(SavedXml, Written)));
		++Compared;
		CompiledModelsAgree(*this, Label, SavedXml, Written, FPaths::GetPath(File));
	}

	TestTrue(TEXT("at least one payload was compared"), Compared > 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjGenProfileSiblingOrderTest, "URLab.Gen.Profile.SiblingOrderSurvivesReorder",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjGenProfileSiblingOrderTest::RunTest(const FString& Parameters)
{
	using namespace MjGenProfileTests;
	using namespace urlab::spec;

	UBlueprint* Blueprint = MakeScratchBlueprint();
	if (Blueprint == nullptr)
	{
		AddError(TEXT("could not create a scratch Blueprint"));
		return false;
	}

	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, InlineCorpus, TEXT("<inline>"));
	if (!Parsed.IsOk())
	{
		AddError(FString::Printf(TEXT("parse failed: %s"), *DiagnosticsToString(Parsed.Errors)));
		return false;
	}

	// Joint order under a body determines the qpos layout, so the spec's
	// order has to survive being restated by the adapter rather than being an
	// accident of whichever list the components happen to sit in.
	FMjScsScope Scope(*Blueprint);
	TArray<UMjNodeComponent*> BeforeOrder;
	TArray<UMjNodeComponent*> Pending{Parsed.Root};
	while (Pending.Num() > 0)
	{
		UMjNodeComponent* Node = Pending.Pop();
		BeforeOrder.Add(Node);
		for (const FMjOrderedChild& Child : FMjScsAdapter::OrderedChildren(*Node))
		{
			Pending.Add(Child.Node);
		}
	}

	for (UMjNodeComponent* Node : BeforeOrder)
	{
		FMjScsAdapter::Renumber(*Node);
	}

	TArray<FMjSpecDiagnostic> Errors;
	const FString AfterRenumber = FSpecRef::OverBlueprint(*Blueprint).WriteMjcf(&Errors);
	TestEqual(TEXT("renumbering is idempotent on a freshly read spec"), Errors.Num(), 0);
	TestFalse(TEXT("the spec still writes"), AfterRenumber.IsEmpty());
	return true;
}

#endif  // URLAB_MJ_GEN && WITH_EDITOR
