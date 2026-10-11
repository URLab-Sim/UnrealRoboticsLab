// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Transport/ShmSession.h"

#include "HAL/FileManager.h"
#include "HAL/PlatformProcess.h"
#include "Misc/Paths.h"

const TCHAR* FURLabShmSession::StateSegmentFileName = TEXT("state.shm");
const TCHAR* FURLabShmSession::RenderSegmentFileName = TEXT("render.shm");

FString FURLabShmSession::DirForSessionId(const FString& InSessionId)
{
	const FString Sid = InSessionId.IsEmpty() ? TEXT("live") : InSessionId;
	return FPaths::ConvertRelativePathToFull(
		FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLabShm"), Sid));
}

FString FURLabShmSession::FilePath(const FString& FileName) const
{
	return FPaths::Combine(Dir, FileName);
}

void FURLabShmSession::EnsureDirectoryExists() const
{
	if (!Dir.IsEmpty())
	{
		IFileManager::Get().MakeDirectory(*Dir, /*Tree=*/true);
	}
}

FURLabShmSession FURLabShmSession::Resolve(const FString& InstanceLabel, int32 InstancePort)
{
	const FString Label = InstanceLabel.IsEmpty() ? FString(TEXT("live")) : InstanceLabel;
	const uint32 Pid = FPlatformProcess::GetCurrentProcessId();

	FURLabShmSession Session;
	Session.SessionId = InstancePort > 0
						  ? FString::Printf(TEXT("%s_p%d_%u"), *Label, InstancePort, Pid)
						  : FString::Printf(TEXT("%s_%u"), *Label, Pid);
	Session.Dir = DirForSessionId(Session.SessionId);
	return Session;
}

FString FURLabShmSession::MakeCameraSegmentFileName(const FString& CanonicalName)
{
	// A canonical camera name is "<articulation>/<part>". The slash cannot
	// survive into a file name, so it becomes the separator between the two
	// segments; a name with no slash (a scene-level camera) keeps an empty part
	// segment rather than being rejected, so the file name is always derivable
	// from whatever the hello advertised.
	FString ArtSeg;
	FString PartSeg;
	if (!CanonicalName.Split(TEXT("/"), &ArtSeg, &PartSeg))
	{
		ArtSeg = CanonicalName;
	}
	return FString::Printf(TEXT("cam_%s_%s.shm"), *ArtSeg, *PartSeg);
}
