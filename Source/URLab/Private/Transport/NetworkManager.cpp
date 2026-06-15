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
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#include "Transport/NetworkManager.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Components/Sensors/MjCamera.h"
#include "Utils/URLabLogging.h"

UMjNetworkManager::UMjNetworkManager()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void UMjNetworkManager::UpdateCameraStreamingState()
{
	UE_LOG(LogURLabNet, Log, TEXT("UpdateCameraStreamingState: Setting all cameras to streaming=%s (Global Toggle)"), bEnableAllCameras ? TEXT("TRUE") : TEXT("FALSE"));

	FScopeLock Lock(&CameraMutex);
	int32 Count = 0;
	for (UMjCamera* Cam : ActiveCameras)
	{
		if (Cam)
		{
			// bEnableAllCameras forces every camera to broadcast (legacy
			// "stream all"). Otherwise respect each camera's own broadcast
			// flags; cameras with neither set stay dormant and only capture
			// when requested (per-camera capture gating in TickComponent).
			if (bEnableAllCameras)
			{
				Cam->bEnableZmqBroadcast = true;
				Cam->bEnableShmBroadcast = true;
			}
			const bool bStream = Cam->bEnableZmqBroadcast || Cam->bEnableShmBroadcast;
			Cam->SetStreamingEnabled(bStream);
			Count++;
			UE_LOG(LogURLabNet, Log, TEXT(" - %s Camera: %s on Actor: %s"), bStream ? TEXT("Streaming") : TEXT("Dormant"), *Cam->GetName(), Cam->GetOwner() ? *Cam->GetOwner()->GetName() : TEXT("None"));
		}
	}
	UE_LOG(LogURLabNet, Log, TEXT("Global camera toggle processed %d cameras."), Count);
}

void UMjNetworkManager::RegisterCamera(UMjCamera* Cam)
{
	if (!Cam)
		return;
	FScopeLock Lock(&CameraMutex);
	ActiveCameras.AddUnique(Cam);

	// bEnableAllCameras forces broadcast on (legacy "stream all"). Otherwise
	// honour the camera's authored broadcast flags. A camera with no broadcast
	// stays dormant (no GPU capture) until a client requests it via
	// include_cameras, at which point per-camera gating spins it up.
	if (bEnableAllCameras)
	{
		Cam->bEnableZmqBroadcast = true;
		Cam->bEnableShmBroadcast = true;
	}
	const bool bStream = Cam->bEnableZmqBroadcast || Cam->bEnableShmBroadcast;
	if (bStream)
	{
		Cam->SetStreamingEnabled(true);
	}

	UE_LOG(LogURLabNet, Log, TEXT("UMjNetworkManager: Registered Camera %s (%s). Total: %d"),
		*Cam->GetName(), bStream ? TEXT("streaming") : TEXT("dormant"), ActiveCameras.Num());
}

void UMjNetworkManager::UnregisterCamera(UMjCamera* Cam)
{
	if (!Cam)
		return;
	FScopeLock Lock(&CameraMutex);
	ActiveCameras.Remove(Cam);
	UE_LOG(LogURLabNet, Log, TEXT("UMjNetworkManager: Unregistered Camera %s. Total: %d"), *Cam->GetName(), ActiveCameras.Num());
}

TArray<UMjCamera*> UMjNetworkManager::GetActiveCameras()
{
	FScopeLock Lock(&CameraMutex);
	return ActiveCameras;
}
