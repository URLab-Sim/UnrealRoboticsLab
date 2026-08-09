// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// A skin's file is a file the compiler needs.
//
// `<mesh>`, `<texture>` and `<hfield>` were collected by the asset pass and
// `<skin>` was not, so a skinned model compiled against a VFS that did not
// carry its `.skn` -- and MuJoCo resolves a skin exactly as it resolves a mesh,
// through `meshdir` and the model file's own directory
// (`user_mesh.cc:3141`), so there was never a second rule to write.
//
// What is asserted here is the mount name, not the bytes: the VFS falls back to
// a case-insensitive basename match across every mount, so two participants
// each carrying their own `body.skn` would silently share one unless the
// prefix reaches the skin like it reaches everything else.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"

#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSpecRef.h"

namespace MjSkinSinkTests
{
using namespace urlab::spec;

/** Which callback each asset arrived through, which is the thing under test. */
class FRecordingSink final : public IMjAssetSink
{
public:
	void OnMesh(const FMjAssetRequest& Request, const TArray<uint8>&) override { Meshes.Add(Request.VfsName); }
	void OnTexture(const FMjAssetRequest& Request, const TArray<uint8>&) override
	{
		Textures.Add(Request.VfsName);
	}
	void OnHeightField(const FMjAssetRequest& Request, const TArray<uint8>&) override
	{
		HeightFields.Add(Request.VfsName);
	}
	void OnSkin(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override
	{
		Skins.Add(Request.VfsName);
		SkinBytes += Bytes.Num();
	}
	void OnMissing(const FMjAssetRequest& Request) override { Missing.Add(Request.Name); }

	TArray<FString> Meshes;
	TArray<FString> Textures;
	TArray<FString> HeightFields;
	TArray<FString> Skins;
	TArray<FString> Missing;
	int32 SkinBytes = 0;
};

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
};

}  // namespace MjSkinSinkTests

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjAssetSinkCollectsSkinFiles,
	"URLab.Doc.AssetSinkCollectsSkinFiles",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjAssetSinkCollectsSkinFiles::RunTest(const FString& Parameters)
{
	using namespace MjSkinSinkTests;

	const FString Stem = FString::Printf(TEXT("MjSkin_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));

	FScratchDoc Doc;
	Doc.Directory =
		FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("URLab/TestModels") / Stem);

	// The bytes are never parsed here -- the pass reads a file, it does not
	// compile one -- but they have to exist, because "missing" is answered by
	// the resolution and a missing asset takes a different callback entirely.
	const TArray<uint8> Bytes = {'S', 'K', 'N', 0, 1, 2, 3, 4};
	const FString SkinPath = Doc.Directory / TEXT("assets/body.skn");
	if (!TestTrue(TEXT("wrote a scratch skin file"), FFileHelper::SaveArrayToFile(Bytes, *SkinPath)))
	{
		return false;
	}

	const FString Xml = TEXT(R"(<mujoco model="skinned">
  <compiler meshdir="assets"/>
  <asset>
    <skin name="cloth" file="body.skn"/>
  </asset>
  <worldbody>
    <body name="base"/>
  </worldbody>
</mujoco>
)");

	Doc.World = UWorld::CreateWorld(EWorldType::Editor, /*bInformEngineOfWorld=*/false, FName(*Stem));
	if (!TestNotNull(TEXT("scratch world"), Doc.World))
	{
		return false;
	}
	Doc.Actor = Doc.World->SpawnActor<AActor>();
	if (!TestNotNull(TEXT("scratch actor"), Doc.Actor))
	{
		return false;
	}
	if (!TestTrue(TEXT("the skinned model parses"),
			MjParseIntoActor(*Doc.Actor, Xml, Doc.Directory / (Stem + TEXT(".xml"))).IsOk()))
	{
		return false;
	}

	FRecordingSink Recorder;
	FMjAssetSink Pass(Recorder);
	// The scene-assembly convention: participant-prefixed basenames, because the
	// VFS matches on basename across every mount.
	Pass.VfsPrefix = TEXT("p0_");
	Pass.Collect(FSpecRef::OverActor(*Doc.Actor));

	TestEqual(TEXT("nothing was reported missing"), Recorder.Missing.Num(), 0);
	TestEqual(TEXT("no mesh was invented"), Recorder.Meshes.Num(), 0);
	if (!TestEqual(TEXT("the skin was collected"), Recorder.Skins.Num(), 1))
	{
		return false;
	}
	TestEqual(TEXT("mounted under the prefixed basename"), Recorder.Skins[0], FString(TEXT("p0_body.skn")));
	TestEqual(TEXT("its bytes came with it"), Recorder.SkinBytes, Bytes.Num());

	// The pass resolved it through meshdir, which is the rule MuJoCo applies to
	// a skin as well as to a mesh.
	const TArray<FMjAssetRequest>& Requests = Pass.GetRequests();
	if (TestEqual(TEXT("one request"), Requests.Num(), 1))
	{
		TestEqual(TEXT("named as authored"), Requests[0].Name, FString(TEXT("cloth")));
		TestFalse(TEXT("the file resolved"), Requests[0].bMissing);
		TestTrue(TEXT("resolved through meshdir"), Requests[0].ResolvedPath.EndsWith(TEXT("assets/body.skn")));
	}

	IFileManager::Get().DeleteDirectory(*Doc.Directory, /*RequireExists=*/false, /*Tree=*/true);
	return true;
}

#endif  // URLAB_MJ_GEN && WITH_EDITOR
