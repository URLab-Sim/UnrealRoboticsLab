// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Elements/MjTexture.h"

#include "Engine/Texture2D.h"
#include "ImageUtils.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

#include "MuJoCo/Spec/MjAssetFiles.h"
#include "MuJoCo/Spec/MjAssetResolve.h"
#include "MuJoCo/Spec/MjAssetSink.h"
#include "Utils/URLabLogging.h"

bool UMjTexture::IsFileStale() const
{
	return TextureAsset != nullptr && FSoftObjectPath(TextureAsset.Get()) != FileAsset;
}

bool UMjTexture::DumpAssetToFile(const FString& BaseDirectory)
{
#if WITH_EDITORONLY_DATA
	if (TextureAsset == nullptr)
	{
		return false;
	}

	// The authored pixels rather than the platform data: the latter is block
	// compressed, and a PNG made from it would hand MuJoCo the compression
	// artefacts as if the user had painted them.
	FImage Image;
	if (!TextureAsset->Source.GetMipImage(Image, 0))
	{
		UE_LOG(LogURLab, Warning, TEXT("[MjTexture] '%s' has no source pixels to export from '%s'"),
			*MjAssetElementName(*this), *TextureAsset->GetName());
		return false;
	}

	TArray64<uint8> Png;
	if (!FImageUtils::CompressImage(Png, TEXT("png"), Image))
	{
		UE_LOG(LogURLab, Error, TEXT("[MjTexture] could not encode '%s' as PNG"), *TextureAsset->GetName());
		return false;
	}

	const FString Name = MjSanitizeAssetName(MjAssetElementName(*this));
	if (Name.IsEmpty())
	{
		return false;
	}
	const FMjDumpTarget Target = MjDumpTargetFor(BaseDirectory, Name, TEXT(".png"));

	if (!FFileHelper::SaveArrayToFile(Png, *Target.FullPath))
	{
		UE_LOG(LogURLab, Error, TEXT("[MjTexture] could not write '%s'"), *Target.FullPath);
		return false;
	}

	// A builtin texture is one MuJoCo generates and a file it would then ignore.
	// The image is now the user's, so the generator has to go.
	Builtin.Reset();

	File = Target.FileAttribute;
	FileAsset = FSoftObjectPath(TextureAsset.Get());
	return true;
#else
	(void)BaseDirectory;
	return false;
#endif
}
