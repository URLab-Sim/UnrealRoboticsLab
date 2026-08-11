// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// What the two compile-parity suites both need.
//
// One suite compares a compiled model against a recorded one, the other against
// the model MuJoCo's own reader produces from the same authored text. They ask
// different questions of the same three steps -- find the fixtures, parse one
// into a scratch Blueprint, compile that through the spec path with its assets
// -- and a second copy of those steps would let the two suites drift into
// testing different pipelines while both claiming to test one.

#include "CoreMinimal.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/Blueprint.h"
#include "Engine/BlueprintGeneratedClass.h"
#include "GameFramework/Actor.h"
#include "HAL/FileManager.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/AutomationTest.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "UObject/Package.h"

#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjSpecBuild.h"
#include "MuJoCo/Spec/MjSpecRef.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>

#include "model_diff_lib.h"
THIRD_PARTY_INCLUDES_END

#include <string>
#include <vector>

namespace MjParitySupport
{

/** Indices sampled per differing field, and entries listed per category. */
constexpr int32 MaxExamples = 4;

/** What a generated name begins with. One spelling, stated in MjReservedNames.h. */
const TCHAR* const ReservedPrefix = TEXT("_ps:");

/** An element family and the model size that counts it. `mjtSize`, never `int32`. */
struct FModelFamily
{
	int32 ObjType;
	mjtSize mjModel::* Count;
};

/** The families a fixture here can leave unnamed, so a reservation can appear in them. */
const FModelFamily ReservableFamilies[] = {
	{  mjOBJ_BODY,  &mjModel::nbody},
	{ mjOBJ_JOINT,   &mjModel::njnt},
	{  mjOBJ_GEOM,  &mjModel::ngeom},
	{  mjOBJ_SITE,  &mjModel::nsite},
	{mjOBJ_CAMERA,   &mjModel::ncam},
	{ mjOBJ_LIGHT, &mjModel::nlight},
};

/**
 * Every generated name in `Model`, sorted.
 *
 * Sorted rather than in model order so the result is a set a test can spell out
 * without also pinning MuJoCo's own id assignment, which is its business.
 */
inline TArray<FString> ReservedNamesOf(const mjModel* Model)
{
	TArray<FString> Names;
	for (const FModelFamily& Family : ReservableFamilies)
	{
		const int32 Count = static_cast<int32>(Model->*Family.Count);
		for (int32 Id = 0; Id < Count; ++Id)
		{
			const char* const Name = mj_id2name(Model, Family.ObjType, Id);
			if (Name == nullptr)
			{
				continue;
			}
			const FString Text(UTF8_TO_TCHAR(Name));
			if (Text.StartsWith(ReservedPrefix))
			{
				Names.Add(Text);
			}
		}
	}
	Names.Sort();
	return Names;
}

inline FString TestDataDir(const TCHAR* Leaf)
{
	return FPaths::Combine(
		FPaths::ProjectPluginsDir(), TEXT("UnrealRoboticsLab"), TEXT("Content"), TEXT("TestData"), Leaf);
}

inline FString ParityDir()
{
	return TestDataDir(TEXT("parity"));
}

inline FString GoldensDir()
{
	return TestDataDir(TEXT("goldens"));
}

/** Every fixture under `Content/TestData/parity`, one file each. */
inline TArray<FString> FixtureFiles()
{
	TArray<FString> Found;
	IFileManager::Get().FindFilesRecursive(Found, *ParityDir(), TEXT("*.xml"), true, false);
	Found.Sort();
	return Found;
}

/** `Model` in the byte form MuJoCo serialises it to, which is what "identical" means. */
inline TArray<uint8> ModelBytes(const mjModel* Model)
{
	TArray<uint8> Buffer;
	const mjtSize Size = Model != nullptr ? mj_sizeModel(Model) : 0;
	if (Size == 0)
	{
		return Buffer;
	}
	Buffer.SetNumUninitialized(static_cast<int32>(Size));
	mj_saveModel(Model, nullptr, Buffer.GetData(), static_cast<int>(Size));
	return Buffer;
}

/** A throwaway Blueprint in the transient package; nothing reaches disk. */
inline UBlueprint* MakeScratchBlueprint(const TCHAR* Prefix)
{
	const FString Name = FString::Printf(TEXT("%s_%s"), Prefix, *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	UPackage* Package = CreatePackage(*(TEXT("/Temp/") + Name));
	return FKismetEditorUtilities::CreateBlueprint(AActor::StaticClass(), Package, FName(*Name), BPTYPE_Normal,
		UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
}

inline FString DiagnosticsToString(const TArray<FMjSpecDiagnostic>& Diagnostics)
{
	TArray<FString> Lines;
	for (const FMjSpecDiagnostic& Diagnostic : Diagnostics)
	{
		Lines.Add(Diagnostic.ToString());
	}
	return FString::Join(Lines, TEXT("; "));
}

inline FString Utf8ToUe(const std::string& Text)
{
	return FString(UTF8_TO_TCHAR(Text.c_str()));
}

/** One field or invariant divergence, with a bounded sample of the indices. */
inline void AppendFieldDiffs(
	TArray<FString>& Lines, const TCHAR* Heading, const std::vector<ps::harness::FieldDiff>& Diffs)
{
	if (Diffs.empty())
	{
		return;
	}
	Lines.Add(FString::Printf(TEXT("  %s (%d):"), Heading, static_cast<int32>(Diffs.size())));

	int32 Listed = 0;
	for (const ps::harness::FieldDiff& Diff : Diffs)
	{
		if (Listed++ >= MaxExamples)
		{
			Lines.Add(FString::Printf(TEXT("    ... and %d more"), static_cast<int32>(Diffs.size()) - MaxExamples));
			break;
		}
		Lines.Add(FString::Printf(TEXT("    %s: %lld of %lld differ%s%s"), *Utf8ToUe(Diff.field),
			static_cast<int64>(Diff.num_diff), static_cast<int64>(Diff.count_a),
			Diff.note.empty() ? TEXT("") : TEXT(" -- "), *Utf8ToUe(Diff.note)));
		for (const ps::harness::FieldDiff::Example& Example : Diff.examples)
		{
			Lines.Add(FString::Printf(TEXT("      [%lld] a %.17g, b %.17g"), static_cast<int64>(Example.index),
				Example.a, Example.b));
		}
	}
}

/** The whole verdict, rendered for a failure message. */
inline FString ReportToString(const ps::harness::DiffReport& Report)
{
	TArray<FString> Lines;
	Lines.Add(FString::Printf(TEXT("  first divergence: %s"), *Utf8ToUe(Report.FirstDivergence())));

	if (!Report.sizes.empty())
	{
		Lines.Add(FString::Printf(TEXT("  sizes (%d):"), static_cast<int32>(Report.sizes.size())));
		int32 Listed = 0;
		for (const ps::harness::SizeDiff& Size : Report.sizes)
		{
			if (Listed++ >= MaxExamples)
			{
				Lines.Add(
					FString::Printf(TEXT("    ... and %d more"), static_cast<int32>(Report.sizes.size()) - MaxExamples));
				break;
			}
			Lines.Add(FString::Printf(TEXT("    %s: a %lld, b %lld"), *Utf8ToUe(Size.name),
				static_cast<int64>(Size.a), static_cast<int64>(Size.b)));
		}
	}

	if (!Report.names.empty())
	{
		Lines.Add(FString::Printf(TEXT("  names (%d):"), static_cast<int32>(Report.names.size())));
		int32 Listed = 0;
		for (const ps::harness::NameDiff& Name : Report.names)
		{
			if (Listed++ >= MaxExamples)
			{
				Lines.Add(
					FString::Printf(TEXT("    ... and %d more"), static_cast<int32>(Report.names.size()) - MaxExamples));
				break;
			}
			Lines.Add(FString::Printf(TEXT("    %s[%d]: a '%s', b '%s'"), *Utf8ToUe(Name.objtype), Name.id,
				*Utf8ToUe(Name.a), *Utf8ToUe(Name.b)));
		}
	}

	AppendFieldDiffs(Lines, TEXT("fields"), Report.fields);
	AppendFieldDiffs(Lines, TEXT("invariants"), Report.invariants);
	return FString::Join(Lines, TEXT("\n"));
}

/** Parse one document into a scratch Blueprint, ready to compile. */
inline UBlueprint* ParseFixture(
	FAutomationTestBase& Test, const TCHAR* Prefix, const FString& Label, const FString& Xml, const FString& Path)
{
	UBlueprint* Blueprint = MakeScratchBlueprint(Prefix);
	if (Blueprint == nullptr)
	{
		Test.AddError(FString::Printf(TEXT("%s: could not create a scratch Blueprint"), *Label));
		return nullptr;
	}

	// A parity fixture is authored against the supported surface, so an
	// unsupported-only parse is a fixture that stopped being covered, not a
	// reason to pass.
	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, Xml, Path);
	if (!Parsed.IsOk())
	{
		Test.AddError(FString::Printf(TEXT("%s: parse failed: %s"), *Label, *DiagnosticsToString(Parsed.Errors)));
		return nullptr;
	}
	return Blueprint;
}

/**
 * A document whose bindable elements are mostly unnamed.
 *
 * Nothing in the parity corpus is: every fixture names every element it has, so
 * the goldens say nothing about what an unnamed one compiles as. This is the
 * document that does, and one named element is in it so that the reservation
 * can be seen to leave authored names alone.
 */
const TCHAR* const UnnamedElementsXml = TEXT(R"(<mujoco model="reserved_names">
  <worldbody>
    <geom type="plane" size="1 1 0.1"/>
    <light pos="0 0 3"/>
    <body pos="0 0 1">
      <joint type="hinge" axis="0 0 1"/>
      <geom name="authored" type="box" size="0.1 0.1 0.1"/>
      <site pos="0 0 0"/>
      <camera pos="0 0 1"/>
      <body pos="0.2 0 0">
        <joint type="hinge" axis="0 1 0"/>
        <geom type="sphere" size="0.05"/>
      </body>
    </body>
  </worldbody>
</mujoco>)");

/**
 * A document with no `model` attribute and nothing else left unnamed.
 *
 * The untitled case on its own: every element carries a name, so the only thing
 * that can differ from what MuJoCo's own reader produces is the model name
 * itself, which is the whole point of the fixture.
 */
const TCHAR* const UntitledModelXml = TEXT(R"(<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1"/>
    <light name="key" pos="0 0 3"/>
    <body name="link">
      <joint name="hinge" type="hinge" axis="0 0 1"/>
      <geom name="box" type="box" size="0.1 0.1 0.1"/>
      <site name="tip" pos="0 0 0"/>
    </body>
  </worldbody>
</mujoco>)");

/** One asset's bytes under the name the spec references it by. */
struct FSpecAsset
{
	FString Name;
	TArray<uint8> Bytes;
};

/** Bytes and mount names for a spec compiled on its own. */
class FSpecAssetCollector final : public IMjAssetSink
{
public:
	TArray<FSpecAsset> Assets;

	void OnMesh(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Take(Request, Bytes); }
	void OnTexture(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Take(Request, Bytes); }
	void OnHeightField(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Take(Request, Bytes); }

private:
	void Take(const FMjAssetRequest& Request, const TArray<uint8>& Bytes)
	{
		if (!Request.VfsName.IsEmpty() && Bytes.Num() > 0)
		{
			Assets.Add(FSpecAsset{Request.VfsName, Bytes});
		}
	}
};

/**
 * Compile one spec through the spec path, assets and all.
 *
 * The bytes are mounted under the names the sink emits and the spec's
 * references are pointed at those same names, which is what mounting a sink's
 * output requires whether or not anything is composed. Leaving the references
 * as authored puts the lookup on MuJoCo's basename fallback, and a model whose
 * visual and collision meshes share basenames then compiles clean with the
 * wrong geometry -- a harness that did that would be reporting its own bug as
 * the product's.
 *
 * Returns null on failure, having reported why. The caller owns the model, and
 * holds the spec it came from: a compiled model never outlives its spec here,
 * the same ordering the compiled scene keeps.
 */
inline mjModel* CompileThroughSpecPath(
	FAutomationTestBase& Test, const FString& Label, const FSpecRef& Spec, urlab::spec::FMjBuiltSpec& Built)
{
	TArray<FMjSpecDiagnostic> Diagnostics;
	Built = urlab::spec::BuildSpec(Spec, Diagnostics);
	if (Built.Spec == nullptr)
	{
		Test.AddError(
			FString::Printf(TEXT("%s: the spec did not build: %s"), *Label, *DiagnosticsToString(Diagnostics)));
		return nullptr;
	}

	FSpecAssetCollector Collector;
	FMjAssetSink Sink(Collector);
	Sink.Collect(Spec);
	for (const FMjAssetRequest& Request : Sink.GetRequests())
	{
		if (Request.bMissing)
		{
			Test.AddError(FString::Printf(TEXT("%s: asset '%s' could not be read from '%s'"), *Label, *Request.Name,
				*Request.ResolvedPath));
		}
	}

	// The same rewrite the scene builder applies, for the same reason: the mount
	// names are complete, so the spec's meshdir would prepend a directory to
	// them and the lookup would land on MuJoCo's basename fallback instead of
	// the file it asked for.
	urlab::spec::MjNamespaceAssets(Built, Sink.GetRequests());

	mjVFS Vfs;
	mj_defaultVFS(&Vfs);
	for (const FSpecAsset& Asset : Collector.Assets)
	{
		mj_addBufferVFS(&Vfs, TCHAR_TO_UTF8(*Asset.Name), Asset.Bytes.GetData(), Asset.Bytes.Num());
	}
	mjModel* const Model = mj_compile(Built.Spec, &Vfs);
	mj_deleteVFS(&Vfs);

	if (Model == nullptr)
	{
		Test.AddError(FString::Printf(
			TEXT("%s: the built spec did not compile: %s"), *Label, UTF8_TO_TCHAR(mjs_getError(Built.Spec))));
	}
	return Model;
}

} // namespace MjParitySupport

#endif // URLAB_MJ_GEN && WITH_EDITOR
