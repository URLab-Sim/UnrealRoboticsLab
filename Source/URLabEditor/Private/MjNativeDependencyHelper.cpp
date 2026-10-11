// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "MjNativeDependencyHelper.h"

#include "HAL/FileManager.h"
#include "HAL/PlatformProcess.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "Containers/StringConv.h"

namespace
{
FString HelperPath(const FString& PluginDir)
{
#if PLATFORM_WINDOWS
	const TCHAR* PlatformDir = TEXT("Win64");
	const TCHAR* Program = TEXT("urlab_mj_deps.exe");
#elif PLATFORM_MAC
	const TCHAR* PlatformDir = TEXT("Mac");
	const TCHAR* Program = TEXT("urlab_mj_deps");
#else
	const TCHAR* PlatformDir = TEXT("Linux");
	const TCHAR* Program = TEXT("urlab_mj_deps");
#endif
	const FString Staged = FPaths::Combine(PluginDir, TEXT("Binaries"), PlatformDir, Program);
	if (FPaths::FileExists(Staged)) return Staged;
	return FPaths::Combine(PluginDir, TEXT("third_party/install/protospec/bin"), Program);
}

bool ReadU32(const TArray<uint8>& Bytes, int64& Offset, uint32& Out)
{
	if (Offset < 0 || Offset + 4 > Bytes.Num()) return false;
	Out = static_cast<uint32>(Bytes[Offset]) |
		(static_cast<uint32>(Bytes[Offset + 1]) << 8) |
		(static_cast<uint32>(Bytes[Offset + 2]) << 16) |
		(static_cast<uint32>(Bytes[Offset + 3]) << 24);
	Offset += 4;
	return true;
}
} // namespace

bool MjDiscoverNativeDependencies(const FString& SourcePath, TArray<FString>& OutPaths, FString& OutError)
{
	OutPaths.Reset();
	OutError.Reset();
	const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("UnrealRoboticsLab"));
	if (!Plugin.IsValid())
	{
		OutError = TEXT("UnrealRoboticsLab plugin was not found");
		return false;
	}
	const FString Program = HelperPath(Plugin->GetBaseDir());
	if (!FPaths::FileExists(Program))
	{
		OutError = FString::Printf(TEXT("native dependency helper is missing: %s; run protospec/build.sh or build.ps1"), *Program);
		return false;
	}
	// ExecProcess takes a command-line string. A quote in a file path cannot be
	// passed unambiguously by this small protocol, so fail explicitly.
	if (SourcePath.Contains(TEXT("\"")))
	{
		OutError = TEXT("native dependency scanner cannot pass a quoted source path");
		return false;
	}
	const FString TempDir = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLab"), TEXT("NativeDependencies"));
	IFileManager::Get().MakeDirectory(*TempDir, /*Tree=*/true);
	const FString OutputPath = FPaths::Combine(TempDir,
		FGuid::NewGuid().ToString(EGuidFormats::Digits) + TEXT(".bin"));
	IFileManager::Get().Delete(*OutputPath, /*RequireExists=*/false, /*EvenReadOnly=*/true, /*Quiet=*/true);
	const FString Args = FString::Printf(TEXT("\"%s\" \"%s\""), *SourcePath, *OutputPath);
	int32 ExitCode = -1;
	FString StdOut, StdErr;
	const bool bLaunched = FPlatformProcess::ExecProcess(*Program, *Args, &ExitCode, &StdOut, &StdErr);
	TArray<uint8> Bytes;
	const bool bLoaded = FFileHelper::LoadFileToArray(Bytes, *OutputPath);
	IFileManager::Get().Delete(*OutputPath, /*RequireExists=*/false, /*EvenReadOnly=*/true, /*Quiet=*/true);
	if (!bLaunched || ExitCode != 0)
	{
		OutError = FString::Printf(TEXT("native dependency helper failed for '%s' (exit %d): %s"),
			*SourcePath, ExitCode, *StdErr.Left(1024));
		return false;
	}
	if (!bLoaded || Bytes.Num() < 12 || FMemory::Memcmp(Bytes.GetData(), "URLDEP01", 8) != 0)
	{
		OutError = TEXT("native dependency helper returned no valid framed output");
		return false;
	}
	int64 Offset = 8;
	uint32 Count = 0;
	if (!ReadU32(Bytes, Offset, Count) || Count == 0 || Count > static_cast<uint32>((Bytes.Num() - Offset) / 5))
	{
		OutError = TEXT("native dependency helper returned an invalid path count");
		return false;
	}
	for (uint32 Index = 0; Index < Count; ++Index)
	{
		uint32 Length = 0;
		if (!ReadU32(Bytes, Offset, Length) || Length == 0 || Length > static_cast<uint64>(Bytes.Num() - Offset))
		{
			OutPaths.Reset();
			OutError = TEXT("native dependency helper returned a truncated path");
			return false;
		}
		const FUTF8ToTCHAR Text(reinterpret_cast<const ANSICHAR*>(Bytes.GetData() + Offset), Length);
		OutPaths.Add(FString(Text.Length(), Text.Get()));
		Offset += Length;
	}
	if (Offset != Bytes.Num())
	{
		OutPaths.Reset();
		OutError = TEXT("native dependency helper returned trailing bytes");
		return false;
	}
	return true;
}
