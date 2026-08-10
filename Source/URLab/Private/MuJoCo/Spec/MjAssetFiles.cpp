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
 *
 * A missing file arrives here too, and it is the case this exists for: an
 * element whose Unreal asset was swapped names a file that has not been written
 * yet, so being told it is absent is the signal to write it, not to skip it.
 */
class FMjAssetFileSync final : public IMjAssetSink
{
public:
	void OnMesh(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Sync(Request); }

	void OnTexture(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Sync(Request); }

	void OnHeightField(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override {}

	void OnMissing(const FMjAssetRequest& Request) override { Sync(Request); }

private:
	static void Sync(const FMjAssetRequest& Request)
	{
		if (UMjMesh* Mesh = Cast<UMjMesh>(Request.Element))
		{
			if (Mesh->IsFileStale())
			{
				Mesh->DumpAssetToFile(Request.BaseDirectory);
			}
			return;
		}
		if (UMjTexture* Texture = Cast<UMjTexture>(Request.Element))
		{
			if (Texture->IsFileStale())
			{
				Texture->DumpAssetToFile(Request.BaseDirectory);
			}
		}
	}
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
