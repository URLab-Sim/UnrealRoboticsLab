// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// The way back out: what `file` says once the element holds an asset.
//
// Import gives a `<mesh>` or `<texture>` the UAsset its file became, and a user
// can then point that reference somewhere else entirely. MuJoCo cannot read a
// UStaticMesh, so something has to put a file back -- but only for the elements
// where the reference and the file have actually parted company. A model the
// user only looked at has to write back exactly as it was read.
//
// The strong assertion here is the compile. A model naming a file that does not
// exist cannot load; the same model with an asset dropped on it loads, and that
// is only possible if the pass really wrote the geometry MuJoCo then read.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/StaticMesh.h"
#include "Engine/Texture2D.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"

#include "MuJoCo/Spec/MjAssetFiles.h"
#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjSpecBuild.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Elements/MjMesh.h"
#include "MuJoCo/Elements/MjTexture.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

namespace MjAssetFileTests
{

/** One asset's bytes under the name the spec references it by. */
struct FScratchAsset
{
	FString Name;
	TArray<uint8> Bytes;
};

/** Bytes and mount names for a spec compiled on its own. */
class FScratchAssetCollector final : public IMjAssetSink
{
public:
	TArray<FScratchAsset> Assets;

	void OnMesh(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Take(Request, Bytes); }
	void OnTexture(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Take(Request, Bytes); }
	void OnHeightField(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Take(Request, Bytes); }

private:
	void Take(const FMjAssetRequest& Request, const TArray<uint8>& Bytes)
	{
		if (!Request.VfsName.IsEmpty() && Bytes.Num() > 0)
		{
			Assets.Add(FScratchAsset{Request.VfsName, Bytes});
		}
	}
};

/**
 * Compile one spec the way the engine does, minus the composition.
 *
 * The export pass runs first, because it is what puts a file back for an
 * element whose reference and file have parted company -- and this whole file
 * is about whether the compiler then finds what it wrote. The spec's own `file`
 * references are left as authored and the bytes are mounted under the names the
 * sink emits, which is what a spec compiled on its own asks for: nothing is
 * namespaced because nothing is composed.
 *
 * Null when the model does not compile, which several cases here require.
 */
mjModel* CompileStandalone(const FSpecRef& Spec, TArray<FMjSpecDiagnostic>& OutDiagnostics)
{
	MjSyncAssetFiles(Spec);

	urlab::spec::FMjBuiltSpec Built = urlab::spec::BuildSpec(Spec, OutDiagnostics);
	if (Built.Spec == nullptr)
	{
		return nullptr;
	}

	FScratchAssetCollector Collector;
	FMjAssetSink Sink(Collector);
	Sink.Collect(Spec);

	mjVFS Vfs;
	mj_defaultVFS(&Vfs);
	for (const FScratchAsset& Asset : Collector.Assets)
	{
		mj_addBufferVFS(&Vfs, TCHAR_TO_UTF8(*Asset.Name), Asset.Bytes.GetData(), Asset.Bytes.Num());
	}
	mjModel* const Model = mj_compile(Built.Spec, &Vfs);
	mj_deleteVFS(&Vfs);

	if (Model == nullptr)
	{
		FMjSpecDiagnostic& Diagnostic = OutDiagnostics.AddDefaulted_GetRef();
		Diagnostic.Message = UTF8_TO_TCHAR(mjs_getError(Built.Spec));
	}
	return Model;
}

/**
 * A spec over a live actor, read from a path that really exists.
 *
 * The path is what an asset's `file` resolves against, and it is therefore also
 * where a dumped asset has to land, so a spec read from nowhere would only
 * ever exercise the fallback.
 */
struct FScratchDoc
{
	UWorld* World = nullptr;
	AActor* Actor = nullptr;
	FString Directory;

	~FScratchDoc()
	{
		if (World != nullptr)
		{
			World->DestroyWorld(false);
		}
	}

	FSpecRef Ref() const { return FSpecRef::OverActor(*Actor); }
};

bool Parse(FAutomationTestBase& Test, FScratchDoc& Doc, const FString& Xml)
{
	const FString Stem = FString::Printf(TEXT("MjFile_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	Doc.Directory =
		FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("URLab/TestModels") / Stem);
	Doc.World = UWorld::CreateWorld(EWorldType::Editor, /*bInformEngineOfWorld=*/false, FName(*Stem));
	if (Doc.World == nullptr)
	{
		Test.AddError(TEXT("could not create a scratch world"));
		return false;
	}
	Doc.Actor = Doc.World->SpawnActor<AActor>();
	if (Doc.Actor == nullptr)
	{
		Test.AddError(TEXT("could not spawn a scratch actor"));
		return false;
	}
	const FMjSpecParseResult Parsed = MjParseIntoActor(*Doc.Actor, Xml, Doc.Directory / (Stem + TEXT(".xml")));
	if (!Parsed.IsOk())
	{
		Test.AddError(TEXT("parse failed"));
		return false;
	}
	return true;
}

template <class T>
T* FirstElement(const FScratchDoc& Doc)
{
	return Doc.Actor->FindComponentByClass<T>();
}

/** A cube with real triangles, which is what a mesh export needs to have. */
UStaticMesh* EngineCube()
{
	return LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
}

/**
 * A small image with authored source pixels.
 *
 * Not `FImageUtils::CreateCheckerboardTexture`: that one fills the platform
 * data and leaves the source empty, which is a texture an import never
 * produces and the export cannot read. Every asset a user could actually pick
 * in the editor carries its source, so the fixture carries one too.
 */
UTexture2D* PaintedTexture()
{
	constexpr int32 Size = 4;
	TArray<uint8> Pixels;
	Pixels.SetNumUninitialized(Size * Size * 4);
	for (int32 Index = 0; Index < Size * Size; ++Index)
	{
		const uint8 Shade = static_cast<uint8>(Index * 16);
		Pixels[Index * 4 + 0] = Shade;
		Pixels[Index * 4 + 1] = 255 - Shade;
		Pixels[Index * 4 + 2] = Shade;
		Pixels[Index * 4 + 3] = 255;
	}

	UTexture2D* Texture = NewObject<UTexture2D>(GetTransientPackage(), NAME_None, RF_Transient);
	Texture->Source.Init(Size, Size, /*NumSlices=*/1, /*NumMips=*/1, TSF_BGRA8, Pixels.GetData());
	Texture->UpdateResource();
	return Texture;
}

}  // namespace MjAssetFileTests

// ============================================================================
// URLab.Export.AnUntouchedModelKeepsItsOwnPaths
//
// Reading a model and writing it back must not rewrite what it said. The
// element remembers which asset its `file` stands for, so "untouched" is a
// comparison and not a heuristic -- and an import leaves every element in
// exactly that state.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjUntouchedPathsTest, "URLab.Export.AnUntouchedModelKeepsItsOwnPaths",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjUntouchedPathsTest::RunTest(const FString& Parameters)
{
	using namespace MjAssetFileTests;

	FScratchDoc Doc;
	if (!Parse(*this, Doc,
			TEXT("<mujoco model=\"kept\"><asset>")
			TEXT("<mesh name=\"part\" file=\"meshes/part.obj\"/>")
			TEXT("<texture name=\"skin\" type=\"2d\" file=\"images/skin.png\"/>")
			TEXT("</asset><worldbody><geom type=\"box\" size=\".1 .1 .1\"/></worldbody></mujoco>")))
	{
		return false;
	}

	UMjMesh* Mesh = FirstElement<UMjMesh>(Doc);
	UMjTexture* Texture = FirstElement<UMjTexture>(Doc);
	if (!TestNotNull(TEXT("the mesh element"), Mesh) || !TestNotNull(TEXT("the texture element"), Texture))
	{
		return false;
	}

	// As the import pass leaves them: the reference set, and the record of what
	// `file` stands for agreeing with it.
	UStaticMesh* Imported = EngineCube();
	Mesh->MeshAsset = Imported;
	Mesh->FileAsset = FSoftObjectPath(Imported);
	UTexture2D* Painted = PaintedTexture();
	Texture->TextureAsset = Painted;
	Texture->FileAsset = FSoftObjectPath(Painted);

	TestFalse(TEXT("an imported mesh is not stale"), Mesh->IsFileStale());
	TestFalse(TEXT("an imported texture is not stale"), Texture->IsFileStale());

	MjSyncAssetFiles(Doc.Ref());

	TestEqual(TEXT("the mesh keeps the path the model gave it"), Mesh->File.Get(FString()),
		FString(TEXT("meshes/part.obj")));
	TestEqual(TEXT("the texture keeps the path the model gave it"), Texture->File.Get(FString()),
		FString(TEXT("images/skin.png")));

	// And an element with no asset at all -- an import that could not read the
	// file, which is every asset in a model whose files have moved -- is left
	// alone rather than being emptied out.
	Mesh->MeshAsset = nullptr;
	MjSyncAssetFiles(Doc.Ref());
	TestEqual(TEXT("an element with no asset keeps its path too"), Mesh->File.Get(FString()),
		FString(TEXT("meshes/part.obj")));
	return true;
}

// ============================================================================
// URLab.Export.ASwappedMeshCompilesFromWhatWasWritten
//
// The model names an OBJ that does not exist, so it cannot compile. Dropping a
// UStaticMesh on the element has to make it compile, and the only way that
// happens is if the export really wrote those triangles where the rewritten
// `file` says they are.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSwappedMeshTest, "URLab.Export.ASwappedMeshCompilesFromWhatWasWritten",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjSwappedMeshTest::RunTest(const FString& Parameters)
{
	using namespace MjAssetFileTests;

	FScratchDoc Doc;
	if (!Parse(*this, Doc,
			TEXT("<mujoco model=\"swap\"><asset><mesh name=\"part\" file=\"part.obj\"/></asset>")
			TEXT("<worldbody><body name=\"b\"><geom name=\"g\" type=\"mesh\" mesh=\"part\"/></body></worldbody>")
			TEXT("</mujoco>")))
	{
		return false;
	}

	UMjMesh* Mesh = FirstElement<UMjMesh>(Doc);
	if (!TestNotNull(TEXT("the mesh element"), Mesh))
	{
		return false;
	}

	{
		TArray<FMjSpecDiagnostic> Ignored;
		mjModel* const Before = CompileStandalone(Doc.Ref(), Ignored);
		TestNull(TEXT("a model naming an OBJ that is not there does not compile"), Before);
		if (Before != nullptr)
		{
			mj_deleteModel(Before);
		}
	}

	UStaticMesh* Cube = EngineCube();
	if (!TestNotNull(TEXT("the engine's cube"), Cube))
	{
		return false;
	}
	Mesh->MeshAsset = Cube;
	TestTrue(TEXT("a swapped asset makes the file stale"), Mesh->IsFileStale());

	TArray<FMjSpecDiagnostic> Diagnostics;
	mjModel* const After = CompileStandalone(Doc.Ref(), Diagnostics);
	for (const FMjSpecDiagnostic& Error : Diagnostics)
	{
		AddError(Error.ToString());
	}
	TestNotNull(TEXT("with the asset dropped on it, the model compiles"), After);
	if (After != nullptr)
	{
		mj_deleteModel(After);
	}

	const FString Written = Mesh->File.Get(FString());
	TestEqual(TEXT("and `file` names what was written"), Written, FString(TEXT("urlab_assets/part.obj")));
	TestTrue(TEXT("which is on disk beside the model"), FPaths::FileExists(Doc.Directory / Written));
	TestFalse(TEXT("the element is no longer stale"), Mesh->IsFileStale());

	// Idempotent: nothing about the second write depends on the first not
	// having happened, and a compile per PIE session must not re-export.
	MjSyncAssetFiles(Doc.Ref());
	TestEqual(TEXT("a second pass changes nothing"), Mesh->File.Get(FString()), Written);

	return true;
}

// ============================================================================
// URLab.Export.ASwappedTextureCompilesFromWhatWasWritten
//
// The same claim for images, which take a different road out: source pixels to
// PNG rather than cooked triangles to OBJ.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSwappedTextureTest, "URLab.Export.ASwappedTextureCompilesFromWhatWasWritten",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjSwappedTextureTest::RunTest(const FString& Parameters)
{
	using namespace MjAssetFileTests;

	FScratchDoc Doc;
	if (!Parse(*this, Doc,
			TEXT("<mujoco model=\"swaptex\"><asset>")
			TEXT("<texture name=\"skin\" type=\"2d\" file=\"skin.png\"/>")
			TEXT("<material name=\"m\" texture=\"skin\"/>")
			TEXT("</asset><worldbody>")
			TEXT("<geom name=\"g\" type=\"box\" size=\".1 .1 .1\" material=\"m\"/>")
			TEXT("</worldbody></mujoco>")))
	{
		return false;
	}

	UMjTexture* Texture = FirstElement<UMjTexture>(Doc);
	if (!TestNotNull(TEXT("the texture element"), Texture))
	{
		return false;
	}

	{
		TArray<FMjSpecDiagnostic> Ignored;
		mjModel* const Before = CompileStandalone(Doc.Ref(), Ignored);
		TestNull(TEXT("a model naming a PNG that is not there does not compile"), Before);
		if (Before != nullptr)
		{
			mj_deleteModel(Before);
		}
	}

	Texture->TextureAsset = PaintedTexture();
	TestTrue(TEXT("a swapped asset makes the file stale"), Texture->IsFileStale());

	TArray<FMjSpecDiagnostic> Diagnostics;
	mjModel* const After = CompileStandalone(Doc.Ref(), Diagnostics);
	for (const FMjSpecDiagnostic& Error : Diagnostics)
	{
		AddError(Error.ToString());
	}
	TestNotNull(TEXT("with the asset dropped on it, the model compiles"), After);
	if (After != nullptr)
	{
		mj_deleteModel(After);
	}

	const FString Written = Texture->File.Get(FString());
	TestEqual(TEXT("and `file` names what was written"), Written, FString(TEXT("urlab_assets/skin.png")));
	TestTrue(TEXT("which is on disk beside the model"), FPaths::FileExists(Doc.Directory / Written));

	return true;
}

#endif  // URLAB_MJ_GEN && WITH_EDITOR
