// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "MuJoCo/Core/MjModelSourcePayload.h"

#include "Dom/JsonObject.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace
{
bool IsExactManifestKey(const FString& Key)
{
	return Key == TEXT("format") || Key == TEXT("path") || Key == TEXT("assets");
}

FString ResolvePath(const FString& ManifestDirectory, const FString& OfferedPath)
{
	FString Resolved = FPaths::IsRelative(OfferedPath)
		? FPaths::Combine(ManifestDirectory, OfferedPath)
		: OfferedPath;
	Resolved = FPaths::ConvertRelativePathToFull(Resolved);
	FPaths::NormalizeFilename(Resolved);
	FPaths::CollapseRelativeDirectories(Resolved);
	return Resolved;
}

bool LoadRequiredBytes(const FString& Kind, const FString& Path,
	TArray<uint8>& OutBytes, FString& OutError)
{
	if (!FFileHelper::LoadFileToArray(OutBytes, *Path))
	{
		OutError = FString::Printf(TEXT("failed to read MuJoCo %s file '%s'"), *Kind, *Path);
		return false;
	}
	if (OutBytes.IsEmpty())
	{
		OutError = FString::Printf(TEXT("MuJoCo %s file '%s' is empty"), *Kind, *Path);
		return false;
	}
	return true;
}
}

bool FMjModelSourceLoader::LoadManifest(const FString& ManifestPath,
	FMjModelSourcePayload& OutPayload, FString& OutError)
{
	OutError.Reset();
	if (ManifestPath.IsEmpty())
	{
		OutError = TEXT("MuJoCo source manifest path is empty");
		return false;
	}

	const FString FullManifestPath = FPaths::ConvertRelativePathToFull(ManifestPath);
	FString JsonText;
	if (!FFileHelper::LoadFileToString(JsonText, *FullManifestPath))
	{
		OutError = FString::Printf(TEXT("failed to read MuJoCo source manifest '%s'"),
			*FullManifestPath);
		return false;
	}
	TSharedPtr<FJsonObject> Manifest;
	const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonText);
	if (!FJsonSerializer::Deserialize(Reader, Manifest) || !Manifest.IsValid())
	{
		OutError = FString::Printf(TEXT("MuJoCo source manifest '%s' is not a JSON object"),
			*FullManifestPath);
		return false;
	}
	for (const TPair<FString, TSharedPtr<FJsonValue>>& Field : Manifest->Values)
	{
		if (!IsExactManifestKey(Field.Key))
		{
			OutError = FString::Printf(TEXT("MuJoCo source manifest has unknown field '%s'"),
				*Field.Key);
			return false;
		}
	}

	FString Format;
	FString SourcePath;
	if (!Manifest->TryGetStringField(TEXT("format"), Format) || Format.IsEmpty()
		|| !Manifest->TryGetStringField(TEXT("path"), SourcePath) || SourcePath.IsEmpty())
	{
		OutError = TEXT("MuJoCo source manifest requires nonempty string fields 'format' and 'path'");
		return false;
	}

	FMjModelSourcePayload Candidate;
	if (Format == TEXT("mjb")) Candidate.Kind = EMjModelSourceKind::Mjb;
	else if (Format == TEXT("mjz")) Candidate.Kind = EMjModelSourceKind::Mjz;
	else if (Format == TEXT("xml")) Candidate.Kind = EMjModelSourceKind::XmlWithAssets;
	else
	{
		OutError = FString::Printf(TEXT("unsupported MuJoCo source format '%s'"), *Format);
		return false;
	}

	const FString ManifestDirectory = FPaths::GetPath(FullManifestPath);
	const FString FullSourcePath = ResolvePath(ManifestDirectory, SourcePath);
	if (!LoadRequiredBytes(TEXT("primary"), FullSourcePath, Candidate.PrimaryBytes, OutError))
	{
		return false;
	}
	Candidate.Label = FullSourcePath;

	if (Manifest->HasField(TEXT("assets")))
	{
		const TSharedPtr<FJsonObject>* AssetsObject = nullptr;
		if (!Manifest->TryGetObjectField(TEXT("assets"), AssetsObject)
			|| !AssetsObject || !AssetsObject->IsValid())
		{
			OutError = TEXT("MuJoCo source manifest field 'assets' must be an object");
			return false;
		}
		for (const TPair<FString, TSharedPtr<FJsonValue>>& Asset : (*AssetsObject)->Values)
		{
			FString FilePath;
			if (Asset.Key.IsEmpty() || !Asset.Value.IsValid()
				|| !Asset.Value->TryGetString(FilePath) || FilePath.IsEmpty())
			{
				OutError = TEXT("MuJoCo source asset names and file paths must be nonempty strings");
				return false;
			}
			TArray<uint8> AssetBytes;
			const FString FullAssetPath = ResolvePath(ManifestDirectory, FilePath);
			if (!LoadRequiredBytes(FString::Printf(TEXT("asset '%s'"), *Asset.Key),
				FullAssetPath, AssetBytes, OutError))
			{
				return false;
			}
			Candidate.Assets.Add(Asset.Key, MoveTemp(AssetBytes));
		}
	}

	OutPayload = MoveTemp(Candidate);
	return true;
}
