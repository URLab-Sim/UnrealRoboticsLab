// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Entity/MjModelSource.h"

#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace MjModelSource
{
namespace
{
/** A MuJoCo error slot, or a fallback when it is empty. Never returns an empty string. */
FString ErrorText(const char* Slot, const TCHAR* Fallback)
{
	return (Slot != nullptr && Slot[0] != '\0') ? FString(UTF8_TO_TCHAR(Slot)) : FString(Fallback);
}

/** Mount every named blob into the VFS under its bare filename. Empty names/blobs are skipped. */
void MountAssets(mjVFS& Vfs, const TMap<FString, TArray<uint8>>& Assets)
{
	for (const TPair<FString, TArray<uint8>>& Asset : Assets)
	{
		if (Asset.Key.IsEmpty() || Asset.Value.Num() == 0)
		{
			continue;
		}
		mj_addBufferVFS(&Vfs, TCHAR_TO_UTF8(*Asset.Key), Asset.Value.GetData(), Asset.Value.Num());
	}
}

/** Compile a spec against its VFS and take ownership of the spec either way (compile does not). */
mjModel* CompileAndDeleteSpec(mjSpec* Spec, const mjVFS* Vfs, FString& OutError)
{
	mjModel* const Model = mj_compile(Spec, Vfs);
	if (Model == nullptr)
	{
		OutError = ErrorText(mjs_getError(Spec), TEXT("mj_compile failed"));
	}
	mj_deleteSpec(Spec);
	return Model;
}

/** Prebuilt buffer: load straight in. Version-locked to this libmujoco. */
mjModel* FromMjb(const TArray<uint8>& Bytes, FString& OutError)
{
	mjModel* const Model = mj_loadModelBuffer(Bytes.GetData(), Bytes.Num());
	if (Model == nullptr)
	{
		OutError = TEXT("mj_loadModelBuffer failed (version-mismatched MJB?)");
	}
	return Model;
}

/** XML text + referenced asset bytes: parse to a spec against a VFS of the assets, then compile. */
mjModel* FromXml(const TArray<uint8>& Bytes, const TMap<FString, TArray<uint8>>& Assets, FString& OutError)
{
	mjVFS Vfs;
	mj_defaultVFS(&Vfs);
	MountAssets(Vfs, Assets);

	// mj_parseXMLString wants a NUL-terminated C string; the wire bytes carry no terminator.
	TArray<uint8> Terminated(Bytes);
	Terminated.Add(0);

	char Error[1024] = {0};
	mjSpec* const Spec = mj_parseXMLString(
		reinterpret_cast<const char*>(Terminated.GetData()), &Vfs, Error, sizeof(Error));
	if (Spec == nullptr)
	{
		OutError = ErrorText(Error, TEXT("mj_parseXMLString failed"));
		mj_deleteVFS(&Vfs);
		return nullptr;
	}

	mjModel* const Model = CompileAndDeleteSpec(Spec, &Vfs, OutError);
	mj_deleteVFS(&Vfs);
	return Model;
}

/** MuJoCo archive (.mjz): parse from a FILE with the native decoder (which extracts
 *  the archive's contents into a VFS), then compile WITH that VFS so the model's
 *  assets resolve. The archive's root model must be named model.xml. */
mjModel* FromMjzFile(const FString& Path, const TMap<FString, TArray<uint8>>& Assets, FString& OutError)
{
	mjVFS Vfs;
	mj_defaultVFS(&Vfs);
	MountAssets(Vfs, Assets);  // optional extra assets mounted alongside the archive

	// content_type "" (empty, not null): mj_parse reads the .mjz from disk, decodes it,
	// and extracts its files into Vfs. A zip mounted as a VFS *buffer* cannot be decoded
	// (the zip provider needs a real seekable file), so the byte overload stages a temp
	// file. mj_compile is then given the SAME Vfs so the extracted assets resolve --
	// passing a null VFS to compile is what makes an asset-bearing mjz fail to build.
	char Error[1024] = {0};
	mjSpec* const Spec = mj_parse(TCHAR_TO_UTF8(*Path), "", &Vfs, Error, sizeof(Error));
	if (Spec == nullptr)
	{
		OutError = ErrorText(Error, TEXT("mj_parse could not decode the mjz (root model must be model.xml)"));
		mj_deleteVFS(&Vfs);
		return nullptr;
	}

	mjModel* const Model = CompileAndDeleteSpec(Spec, &Vfs, OutError);  // mj_compile(Spec, &Vfs)
	mj_deleteVFS(&Vfs);
	return Model;
}

/** MuJoCo archive from memory (wire bytes): the decoder needs a real file path, so
 *  stage the bytes to a temp file, decode+compile, then remove it. */
mjModel* FromMjz(const TArray<uint8>& Bytes, const TMap<FString, TArray<uint8>>& Assets, FString& OutError)
{
	const FString TempPath = FPaths::CreateTempFilename(
		*FPaths::ProjectIntermediateDir(), TEXT("urlab_mjz_"), TEXT(".mjz"));
	if (!FFileHelper::SaveArrayToFile(Bytes, *TempPath))
	{
		OutError = FString::Printf(TEXT("could not stage the mjz to a temp file '%s'"), *TempPath);
		return nullptr;
	}
	mjModel* const Model = FromMjzFile(TempPath, Assets, OutError);
	IFileManager::Get().Delete(*TempPath);
	return Model;
}
} // namespace

mjModel* FromBytes(const TArray<uint8>& Bytes, const FString& Format,
	const TMap<FString, TArray<uint8>>& Assets, FString& OutError)
{
	OutError.Reset();
	if (Bytes.Num() == 0)
	{
		OutError = TEXT("empty model source buffer");
		return nullptr;
	}
	if (Format.Equals(TEXT("mjb"), ESearchCase::IgnoreCase))
	{
		return FromMjb(Bytes, OutError);
	}
	if (Format.Equals(TEXT("xml"), ESearchCase::IgnoreCase))
	{
		return FromXml(Bytes, Assets, OutError);
	}
	if (Format.Equals(TEXT("mjz"), ESearchCase::IgnoreCase))
	{
		return FromMjz(Bytes, Assets, OutError);
	}
	OutError = FString::Printf(TEXT("unknown model source format '%s' (expected mjb|xml|mjz)"), *Format);
	return nullptr;
}

bool CompileFileToMjb(const FString& Path, const FString& Format,
	TArray<uint8>& OutMjb, FString& OutError)
{
	OutError.Reset();
	OutMjb.Reset();

	mjModel* Model = nullptr;
	if (Format.Equals(TEXT("xml"), ESearchCase::IgnoreCase))
	{
		// Disk parse+compile: mj_loadXML resolves meshdir/texturedir and every
		// <include> relative to the file's own directory, so a normal on-disk scene
		// (e.g. a menagerie scene.xml with an assets/ subdir) needs no VFS mounting.
		char Error[1024] = {0};
		Model = mj_loadXML(TCHAR_TO_UTF8(*Path), nullptr, Error, sizeof(Error));
		if (Model == nullptr)
		{
			OutError = ErrorText(Error, TEXT("mj_loadXML failed"));
			return false;
		}
	}
	else if (Format.Equals(TEXT("mjz"), ESearchCase::IgnoreCase))
	{
		// Already on disk -- parse it in place (no temp-file staging needed).
		Model = FromMjzFile(Path, TMap<FString, TArray<uint8>>(), OutError);
		if (Model == nullptr)
		{
			return false;
		}
	}
	else if (Format.Equals(TEXT("mjb"), ESearchCase::IgnoreCase))
	{
		TArray<uint8> Bytes;
		if (!FFileHelper::LoadFileToArray(Bytes, *Path))
		{
			OutError = FString::Printf(TEXT("could not read mjb '%s'"), *Path);
			return false;
		}
		Model = FromMjb(Bytes, OutError);
		if (Model == nullptr)
		{
			return false;
		}
	}
	else
	{
		OutError = FString::Printf(TEXT("unknown model file format '%s' (expected mjb|xml|mjz)"), *Format);
		return false;
	}

	const int32 Size = mj_sizeModel(Model);
	OutMjb.SetNumUninitialized(Size);
	mj_saveModel(Model, nullptr, OutMjb.GetData(), Size);
	mj_deleteModel(Model);
	if (OutMjb.Num() == 0)
	{
		OutError = TEXT("compiled model serialized to zero bytes");
		return false;
	}
	return true;
}
} // namespace MjModelSource
