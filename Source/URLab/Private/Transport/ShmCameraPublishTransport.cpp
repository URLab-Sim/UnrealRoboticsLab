// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.

#include "Transport/ShmCameraPublishTransport.h"

#include "HAL/FileManager.h"
#include "Misc/Paths.h"

#include "MuJoCo/Capture/MjCameraTypes.h"
#include "Transport/ShmPublishTransport.h"
#include "Utils/URLabLogging.h"

void UURLabShmCameraPublishTransport::Configure(FIntPoint InResolution,
	const FString& InBaseSessionId)
{
	Resolution = InResolution;
	BaseSessionId = InBaseSessionId.IsEmpty() ? TEXT("live") : InBaseSessionId;
}

void UURLabShmCameraPublishTransport::OpenCameraChannel(int32 CameraIndex,
	const FString& CanonicalName, int32 StreamPortIndex)
{
	if (Writers.Contains(CameraIndex))
	{
		return;
	}

	// The bare "live" session segment is process-global on one host, so several
	// editors acting as render servers would all open the same cam_*.shm.
	// Parameterise it per editor process with the same scheme the RPC and state
	// segments use (base label + pid). BaseSessionId (set via Configure) is the
	// SAME resolved base the state transport used — UURLabShmPublishTransport::
	// ResolveActiveSessionBase, i.e. the dispatcher's active session id once a
	// hello has minted one, else "live" — so running it through the identical
	// MakeInstanceSessionId transform lands state.shm and cam_*.shm in one dir;
	// the client learns that dir from the hello's shm_session_dir (set by
	// UURLabShmPublishTransport::AppendHandshakeBlock) and opens both under it.
	const FString Dir = UURLabShmPublishTransport::ResolveSessionDir(
		UURLabShmPublishTransport::MakeInstanceSessionId(BaseSessionId));
	IFileManager::Get().MakeDirectory(*Dir, /*Tree=*/true);
	// Derive the SHM stem from the canonical name (which honors a re-homed camera's
	// pinned override), so it matches the advertised ZMQ topic.
	FString CanonArtStr, CanonPartStr;
	if (!CanonicalName.Split(TEXT("/"), &CanonArtStr, &CanonPartStr))
	{
		CanonArtStr = CanonicalName;
	}
	const FString FileName = FString::Printf(TEXT("cam_%s_%s.shm"), *CanonArtStr, *CanonPartStr);
	const FString FullPath = FPaths::Combine(Dir, FileName);

	TUniquePtr<FCameraShmWriter> Writer = MakeUnique<FCameraShmWriter>();
	if (!Writer->Open(FullPath, Resolution))
	{
		return;
	}
	UE_LOG(LogURLabNet, Log, TEXT("[ShmCameraPublishTransport] '%s' SHM broadcast at %s"),
		*CanonicalName, *FullPath);
	Writers.Add(CameraIndex, MoveTemp(Writer));
}

void UURLabShmCameraPublishTransport::PublishCameraFrame(const FMjCameraWireFrame& Frame)
{
	TUniquePtr<FCameraShmWriter>* Found = Writers.Find(Frame.CameraIndex);
	if (!Found || !Found->IsValid())
	{
		return;
	}

	FMjCameraFrameMeta Meta;
	Meta.FrameId = Frame.FrameId;
	Meta.SimTime = Frame.SimTime;
	Meta.Width = static_cast<uint32>(Frame.Width);
	Meta.Height = static_cast<uint32>(Frame.Height);
	Meta.CaptureUnixTime = Frame.CaptureUnixSeconds;

	(*Found)->PushFrame(Frame.Bytes, static_cast<uint32>(Frame.NumBytes), Meta);
}

void UURLabShmCameraPublishTransport::CloseCameraChannel(int32 CameraIndex)
{
	if (TUniquePtr<FCameraShmWriter>* Found = Writers.Find(CameraIndex))
	{
		if (Found->IsValid())
		{
			(*Found)->Close(/*bDeleteFile=*/true);
		}
		Writers.Remove(CameraIndex);
	}
}

void UURLabShmCameraPublishTransport::TransportShutdown()
{
	TArray<int32> Open;
	Writers.GetKeys(Open);
	for (int32 CameraIndex : Open)
	{
		CloseCameraChannel(CameraIndex);
	}
}
