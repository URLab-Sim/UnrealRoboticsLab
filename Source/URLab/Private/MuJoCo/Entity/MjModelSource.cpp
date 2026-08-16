// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Entity/MjModelSource.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace MjModelSource
{
namespace
{
// Name the mjz archive is mounted under: the ".mjz" extension is what the decoder registry matches
// against when no explicit content type is given, so the decoder is selected by extension.
const char* const kMjzMount = "model.mjz";

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

/** MuJoCo archive: mount it, decode through the resource decoder registry to a spec, then compile. */
mjModel* FromMjz(const TArray<uint8>& Bytes, const TMap<FString, TArray<uint8>>& Assets, FString& OutError)
{
	mjVFS Vfs;
	mj_defaultVFS(&Vfs);
	// An mjz is normally self-contained; mount any side assets first so a reference into them resolves.
	MountAssets(Vfs, Assets);
	if (mj_addBufferVFS(&Vfs, kMjzMount, Bytes.GetData(), Bytes.Num()) != 0)
	{
		OutError = TEXT("could not mount the mjz buffer in the VFS");
		mj_deleteVFS(&Vfs);
		return nullptr;
	}

	// content_type null: mj_parse opens the mounted resource and finds the decoder by its ".mjz"
	// extension in the registry, so no MIME string has to be hardcoded here.
	char Error[1024] = {0};
	mjSpec* const Spec = mj_parse(kMjzMount, nullptr, &Vfs, Error, sizeof(Error));
	if (Spec == nullptr)
	{
		OutError = ErrorText(Error, TEXT("mj_parse could not decode the mjz (no decoder registered?)"));
		mj_deleteVFS(&Vfs);
		return nullptr;
	}

	mjModel* const Model = CompileAndDeleteSpec(Spec, &Vfs, OutError);
	mj_deleteVFS(&Vfs);
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
} // namespace MjModelSource
