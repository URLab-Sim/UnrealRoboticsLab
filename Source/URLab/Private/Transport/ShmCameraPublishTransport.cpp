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
#include "Utils/URLabLogging.h"

void UURLabShmCameraPublishTransport::Configure(FIntPoint InResolution,
	const FURLabShmSession& InSession)
{
	Resolution = InResolution;
	Session = InSession;
}

void UURLabShmCameraPublishTransport::OpenCameraChannel(int32 CameraIndex,
	const FString& CanonicalName)
{
	if (Writers.Contains(CameraIndex))
	{
		return;
	}

	// The session was handed to this transport, not derived here: deriving it
	// per transport is what put cam_*.shm in a directory the handshake never
	// advertised. A camera standing on its own (a unit test) still needs a valid
	// directory, so fall back to the process-default session.
	if (!Session.IsValid())
	{
		Session = FURLabShmSession::Resolve(FString(), /*InstancePort=*/0);
	}
	Session.EnsureDirectoryExists();
	// Derive the SHM stem from the canonical name (which honours a re-homed
	// camera's pinned override), so it matches the advertised ZMQ topic. The
	// derivation is shared with the session hello, which advertises this exact
	// file name per camera as `shm_file_name`, so the client never has to
	// reconstruct it.
	const FString FullPath =
		Session.FilePath(FURLabShmSession::MakeCameraSegmentFileName(CanonicalName));

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
