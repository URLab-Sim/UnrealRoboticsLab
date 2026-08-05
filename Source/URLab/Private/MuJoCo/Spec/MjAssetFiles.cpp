// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjAssetFiles.h"

#include "HAL/FileManager.h"
#include "Misc/Paths.h"

#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjSpecRef.h"

#if URLAB_MJ_GEN
#include "MuJoCo/Elements/MjMesh.h"
#include "MuJoCo/Elements/MjTexture.h"
#endif

namespace
{

#if URLAB_MJ_GEN

/**
 * The pass, as a sink.
 *
 * Over the same walk the importer and the compiler use, so an element cannot be
 * exported under one set of asset directories and then read back under another.
 * The bytes are not wanted here -- what is on disk is exactly what this may be
 * about to replace -- which is why the pass runs with `bLoadBytes` off.
 */
class FMjAssetFileSync final : public IMjAssetSink
{
public:
	void OnMesh(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override
	{
		UMjMesh* Mesh = Cast<UMjMesh>(Request.Element);
		if (Mesh != nullptr && Mesh->IsFileStale())
		{
			Mesh->DumpAssetToFile(Request.BaseDirectory);
		}
	}

	void OnTexture(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override
	{
		UMjTexture* Texture = Cast<UMjTexture>(Request.Element);
		if (Texture != nullptr && Texture->IsFileStale())
		{
			Texture->DumpAssetToFile(Request.BaseDirectory);
		}
	}

	void OnHeightField(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override {}
};

#endif  // URLAB_MJ_GEN

}  // namespace

FMjDumpTarget MjDumpTargetFor(const FString& BaseDirectory, const FString& Name, const TCHAR* Extension)
{
	FMjDumpTarget Out;
	const FString Leaf = FString(MjDumpedAssetFolder) / Name + Extension;

	if (BaseDirectory.IsEmpty())
	{
		Out.FullPath = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("URLab") / Leaf);
		Out.FileAttribute = Out.FullPath;
	}
	else
	{
		Out.FullPath = FPaths::ConvertRelativePathToFull(BaseDirectory / Leaf);
		Out.FileAttribute = Leaf.Replace(TEXT("\\"), TEXT("/"));
	}

	IFileManager::Get().MakeDirectory(*FPaths::GetPath(Out.FullPath), /*Tree=*/true);
	return Out;
}

void MjSyncAssetFiles(const FSpecRef& Spec)
{
#if URLAB_MJ_GEN
	if (!Spec.IsValid())
	{
		return;
	}
	FMjAssetFileSync Sync;
	FMjAssetSink Pass(Sync);
	Pass.bLoadBytes = false;
	Pass.Collect(Spec);
#else
	(void)Spec;
#endif
}
