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

#include "Bridge/RpcDispatcher.h"
#include "Bridge/OpRegistry.h"
#include "Transport/ZmqRpcTransport.h"
#include "Bridge/MsgpackHelpers.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "MuJoCo/Components/Sensors/MjSensor.h"
#include "MuJoCo/Components/Sensors/MjCamera.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "MuJoCo/Components/Bodies/MjBody.h"
#include "MuJoCo/Components/Controllers/MjArticulationController.h"
#include "MuJoCo/Input/MjPerturbation.h"
#include "MuJoCo/Input/MjTwistController.h"
#include "Transport/NetworkManager.h"
#include "Transport/ShmPublishTransport.h"
#include "Transport/ShmRpcTransport.h"
#include "Transport/RpcTransport.h"
#include "Transport/ShmRegion.h" // FMjShmHeader (header_size in shm_rpc block)
#include "Bridge/BridgeServer.h"
#include "Replay/MjReplayManager.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/Base64.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "Internationalization/Regex.h"
#include "HAL/FileManager.h"
#include "EngineUtils.h"
#include "Engine/World.h"
#include "Misc/Guid.h"
#include "Utils/URLabLogging.h"

void FURLabRpcDispatcher::BuildCameraNameMap(AAMjManager* Manager,
	TMap<FString, UMjCamera*>& OutByName)
{
	if (!Manager)
		return;
	// Register every form a client may key on:
	//   - "<art>/camera/<camera>" (matches the zmq_topic the hello handshake
	//                              advertises)
	//   - "<art>/<camera>"        (qualified)
	//   - "<camera>"              (bare, canonical)
	//   - raw UE component / MJCF name (back-compat)
	// First writer wins per key, so a second camera sharing a bare name cannot
	// hide an already-registered distinct camera (still reachable via qualified).
	auto AddName = [&OutByName](const FString& Key, UMjCamera* C) {
		UMjCamera*& Slot = OutByName.FindOrAdd(Key);
		if (Slot == nullptr)
			Slot = C;
	};
	for (AMjArticulation* Art : Manager->GetAllArticulations())
	{
		if (!Art)
			continue;
		TArray<UMjCamera*> Cameras;
		Art->GetComponents<UMjCamera>(Cameras);
		for (UMjCamera* C : Cameras)
		{
			if (!C || C->bIsDefault)
				continue;
			const FString ArtName = Art->GetName();
			const FString Canon = C->GetCanonicalName();
			AddName(Canon, C);
			AddName(ArtName + TEXT("/") + Canon, C);
			AddName(ArtName + TEXT("/camera/") + Canon, C);
			const FString CompName = C->GetName();
			const FString MjNameRaw = C->GetMjName();
			AddName(CompName, C);
			AddName(ArtName + TEXT("/") + CompName, C);
			if (!MjNameRaw.IsEmpty())
			{
				AddName(MjNameRaw, C);
				AddName(ArtName + TEXT("/") + MjNameRaw, C);
			}
		}
	}
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::ApplyCameraStreamingGameThread(
	AAMjManager* Manager, const TMap<FString, TPair<bool, bool>>& Requests)
{
	// Game thread: render target + ZMQ/SHM workers must be set up here.
	check(IsInGameThread());
	TSharedPtr<FJsonObject> Out = MakeShared<FJsonObject>();
	if (!Manager)
		return Out;

	TMap<FString, UMjCamera*> ByName;
	BuildCameraNameMap(Manager, ByName);

	for (const TPair<FString, TPair<bool, bool>>& Req : Requests)
	{
		UMjCamera** Found = ByName.Find(Req.Key);
		if (!Found || !*Found)
		{
			UE_LOG(LogURLabNet, Warning,
				TEXT("[set_camera_streaming] camera '%s' not found"), *Req.Key);
			continue;
		}
		UMjCamera* Cam = *Found;
		const bool bZmq = Req.Value.Key;
		const bool bShm = Req.Value.Value;

		const bool bStream = bZmq || bShm;
		// Idempotent: only (re)build when the requested flags actually change.
		// SetStreamingEnabled(false) destroys the RenderTarget, so toggling on
		// every call orphans any consumer bound to it -- the editor Simulate
		// camera-feed widget then freezes on its last frame. Clients call this
		// on every discover/set_mode, so unchanged requests MUST be no-ops.
		const bool bUnchanged = (Cam->bEnableZmqBroadcast == bZmq)
			&& (Cam->bEnableShmBroadcast == bShm)
			&& (Cam->IsStreamingActive() == bStream);
		if (!bUnchanged)
		{
			Cam->bEnableZmqBroadcast = bZmq;
			Cam->bEnableShmBroadcast = bShm;
			Cam->SetStreamingEnabled(false);
			if (bStream)
				Cam->SetStreamingEnabled(true);
		}

		TSharedPtr<FJsonObject> CamObj = MakeShared<FJsonObject>();
		CamObj->SetBoolField(TEXT("streaming"), bStream);
		CamObj->SetBoolField(TEXT("zmq"), bZmq);
		CamObj->SetBoolField(TEXT("shm"), bShm);
		if (bZmq)
		{
			FString Endpoint = Cam->GetActualZmqEndpoint();
			Endpoint.ReplaceInline(TEXT("*"), TEXT("127.0.0.1"));
			CamObj->SetStringField(TEXT("zmq_endpoint"), Endpoint);
			AMjArticulation* Art = Cast<AMjArticulation>(Cam->GetOwner());
			const FString Prefix = Art ? Art->GetName()
									   : (Cam->GetOwner() ? Cam->GetOwner()->GetName() : TEXT("unknown"));
			CamObj->SetStringField(TEXT("zmq_topic"),
				FString::Printf(TEXT("%s/camera/%s"), *Prefix, *Cam->GetCanonicalName()));
		}
		// Key the reply by the canonical name so the bridge always gets a
		// stable identity back regardless of which alias it requested.
		Out->SetObjectField(Cam->GetCanonicalName(), CamObj);
	}
	return Out;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetCameraStreaming(
	const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr)
		return MakeError(TEXT("not_ready"), TEXT("Manager missing"));

	const TSharedPtr<FJsonObject>* CamObj = nullptr;
	if (!Req->TryGetObjectField(TEXT("cameras"), CamObj) || !CamObj || !CamObj->IsValid())
		return MakeError(TEXT("missing_field"),
			TEXT("set_camera_streaming requires a 'cameras' object"));

	// Parse per-camera requests. Value forms:
	//   true / false           -> both transports on / off
	//   { "zmq": b, "shm": b }  -> per-transport; an omitted sub-field is off
	//                              UNLESS both are omitted, which means "both on"
	TMap<FString, TPair<bool, bool>> Requests; // key -> {zmq, shm}
	for (const TPair<FString, TSharedPtr<FJsonValue>>& Kv : (*CamObj)->Values)
	{
		if (!Kv.Value.IsValid())
			continue;
		bool bZmq = true;
		bool bShm = true;
		bool bBool = false;
		const TSharedPtr<FJsonObject>* Sub = nullptr;
		if (Kv.Value->TryGetBool(bBool))
		{
			bZmq = bBool;
			bShm = bBool;
		}
		else if (Kv.Value->TryGetObject(Sub) && Sub && Sub->IsValid())
		{
			bool z = false, s = false;
			const bool bHasZ = (*Sub)->TryGetBoolField(TEXT("zmq"), z);
			const bool bHasS = (*Sub)->TryGetBoolField(TEXT("shm"), s);
			if (bHasZ || bHasS)
			{
				bZmq = bHasZ ? z : false;
				bShm = bHasS ? s : false;
			}
			// else leave both true (default = enable both)
		}
		Requests.Add(Kv.Key, TPair<bool, bool>(bZmq, bShm));
	}

	if (Requests.Num() == 0)
		return MakeError(TEXT("bad_request"), TEXT("'cameras' had no usable entries"));

	// Camera RT / worker setup is game-thread only; marshal and wait.
	struct FResult
	{
		FEvent* Done = nullptr;
		TSharedPtr<FJsonObject> Cameras;
	};
	TSharedPtr<FResult, ESPMode::ThreadSafe> Res = MakeShared<FResult, ESPMode::ThreadSafe>();
	Res->Done = FPlatformProcess::GetSynchEventFromPool(/*bIsManualReset=*/false);
	TWeakObjectPtr<AAMjManager> WeakMgr(Mgr);
	AsyncTask(ENamedThreads::GameThread, [Res, WeakMgr, Requests]() {
		if (AAMjManager* M = WeakMgr.Get())
			Res->Cameras = ApplyCameraStreamingGameThread(M, Requests);
		Res->Done->Trigger();
	});

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	if (Res->Done->Wait(5000))
	{
		FPlatformProcess::ReturnSynchEventToPool(Res->Done);
		Res->Done = nullptr;
		Reply->SetStringField(TEXT("op"), TEXT("set_camera_streaming_ok"));
		Reply->SetObjectField(TEXT("cameras"),
			Res->Cameras.IsValid() ? Res->Cameras : MakeShared<FJsonObject>());
		return Reply;
	}
	// Timed out — leave the event un-pooled (the task still references it).
	return MakeError(TEXT("timeout"), TEXT("set_camera_streaming game-thread apply timed out"));
}

namespace
{
// Parsed per-camera latency + capture-rate request. bSetCapture marks whether
// the on_state_change / max_fps capture knobs were present (so we only override
// them when the caller actually sent them).
struct FCameraDelayReq
{
	float DelaySeconds = 0.0f;
	float JitterSeconds = 0.0f;
	bool bWallClock = false;
	int32 Seed = 0;
	bool bOnStateChange = true;
	float MaxFps = 0.0f;
	bool bSetCapture = false;
};
} // namespace

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetCameraDelay(
	const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr)
		return MakeError(TEXT("not_ready"), TEXT("Manager missing"));

	const TSharedPtr<FJsonObject>* CamObj = nullptr;
	if (!Req->TryGetObjectField(TEXT("cameras"), CamObj) || !CamObj || !CamObj->IsValid())
		return MakeError(TEXT("missing_field"),
			TEXT("set_camera_delay requires a 'cameras' object"));

	// Parse per-camera requests. A bare number means delay_s with defaults; an
	// object carries delay_s / jitter_s / clock / seed / on_state_change / max_fps.
	TMap<FString, FCameraDelayReq> Requests;
	for (const TPair<FString, TSharedPtr<FJsonValue>>& Kv : (*CamObj)->Values)
	{
		if (!Kv.Value.IsValid())
			continue;
		FCameraDelayReq R;
		double Num = 0.0;
		const TSharedPtr<FJsonObject>* Sub = nullptr;
		if (Kv.Value->TryGetNumber(Num))
		{
			R.DelaySeconds = static_cast<float>(Num);
		}
		else if (Kv.Value->TryGetObject(Sub) && Sub && Sub->IsValid())
		{
			double D = 0.0;
			if ((*Sub)->TryGetNumberField(TEXT("delay_s"), D))
				R.DelaySeconds = static_cast<float>(D);
			double J = 0.0;
			if ((*Sub)->TryGetNumberField(TEXT("jitter_s"), J))
				R.JitterSeconds = static_cast<float>(J);
			FString Clock;
			if ((*Sub)->TryGetStringField(TEXT("clock"), Clock))
				R.bWallClock = Clock.Equals(TEXT("wall"), ESearchCase::IgnoreCase);
			int32 Seed = 0;
			if ((*Sub)->TryGetNumberField(TEXT("seed"), Seed))
				R.Seed = Seed;
			bool bOnState = true;
			if ((*Sub)->TryGetBoolField(TEXT("on_state_change"), bOnState))
			{
				R.bOnStateChange = bOnState;
				R.bSetCapture = true;
			}
			double Fps = 0.0;
			if ((*Sub)->TryGetNumberField(TEXT("max_fps"), Fps))
			{
				R.MaxFps = static_cast<float>(Fps);
				R.bSetCapture = true;
			}
		}
		Requests.Add(Kv.Key, R);
	}

	if (Requests.Num() == 0)
		return MakeError(TEXT("bad_request"), TEXT("'cameras' had no usable entries"));

	// SetCameraDelay / SetCaptureRate touch state the game thread reads each tick;
	// marshal and wait, like set_camera_streaming.
	struct FResult
	{
		FEvent* Done = nullptr;
		TSharedPtr<FJsonObject> Cameras;
	};
	TSharedPtr<FResult, ESPMode::ThreadSafe> Res = MakeShared<FResult, ESPMode::ThreadSafe>();
	Res->Done = FPlatformProcess::GetSynchEventFromPool(/*bIsManualReset=*/false);
	TWeakObjectPtr<AAMjManager> WeakMgr(Mgr);
	AsyncTask(ENamedThreads::GameThread, [Res, WeakMgr, Requests]() {
		TSharedPtr<FJsonObject> Out = MakeShared<FJsonObject>();
		if (AAMjManager* M = WeakMgr.Get())
		{
			TMap<FString, UMjCamera*> ByName;
			FURLabRpcDispatcher::BuildCameraNameMap(M, ByName);
			for (const TPair<FString, FCameraDelayReq>& Req : Requests)
			{
				UMjCamera** Found = ByName.Find(Req.Key);
				if (!Found || !*Found)
				{
					UE_LOG(LogURLabNet, Warning,
						TEXT("[set_camera_delay] camera '%s' not found"), *Req.Key);
					continue;
				}
				UMjCamera* Cam = *Found;
				const FCameraDelayReq& V = Req.Value;
				Cam->SetCameraDelay(V.DelaySeconds, V.JitterSeconds, V.bWallClock, V.Seed);
				if (V.bSetCapture)
					Cam->SetCaptureRate(V.bOnStateChange, V.MaxFps);

				TSharedPtr<FJsonObject> CamOut = MakeShared<FJsonObject>();
				CamOut->SetNumberField(TEXT("delay_s"), Cam->DelaySeconds);
				CamOut->SetNumberField(TEXT("jitter_s"), Cam->DelayJitterSeconds);
				CamOut->SetStringField(TEXT("clock"),
					Cam->bDelayUseWallClock ? TEXT("wall") : TEXT("sim"));
				CamOut->SetBoolField(TEXT("on_state_change"), Cam->bCaptureOnStateChange);
				CamOut->SetNumberField(TEXT("max_fps"), Cam->CaptureMaxFps);
				Out->SetObjectField(Cam->GetCanonicalName(), CamOut);
			}
		}
		Res->Cameras = Out;
		Res->Done->Trigger();
	});

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	if (Res->Done->Wait(5000))
	{
		FPlatformProcess::ReturnSynchEventToPool(Res->Done);
		Res->Done = nullptr;
		Reply->SetStringField(TEXT("op"), TEXT("set_camera_delay_ok"));
		Reply->SetObjectField(TEXT("cameras"),
			Res->Cameras.IsValid() ? Res->Cameras : MakeShared<FJsonObject>());
		return Reply;
	}
	return MakeError(TEXT("timeout"), TEXT("set_camera_delay game-thread apply timed out"));
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::BuildCamerasBlock(AAMjManager* Manager,
	const TMap<FString, ECameraInclude>& CameraSpec,
	const TMap<FString, uint64>& MinFrameIds, int32 TimeoutMs)
{
	(void)TimeoutMs; // retrieval is non-blocking; param kept for ABI compatibility
	TSharedPtr<FJsonObject> Cams = MakeShared<FJsonObject>();
	if (!Manager || CameraSpec.Num() == 0)
		return Cams;

	UWorld* World = Manager->GetWorld();
	if (!World)
		return Cams;

	TMap<FString, UMjCamera*> ByName;
	BuildCameraNameMap(Manager, ByName);

	// Non-blocking retrieval from each camera's frame-history ring. No capture,
	// no FlushRenderingCommands — the per-tick async readback (decoupled from
	// the step, like MuJoCo simulate's render thread) keeps the ring fresh.
	// A client gets the frame for a specific step by passing its frame_id
	// (MinFrameIds); otherwise it gets the latest available frame.
	for (const TPair<FString, ECameraInclude>& Spec : CameraSpec)
	{
		UMjCamera** Found = ByName.Find(Spec.Key);
		if (!Found || !*Found)
		{
			UE_LOG(LogURLabNet, Warning,
				TEXT("[BuildCamerasBlock] camera '%s' not found; dropped from reply"),
				*Spec.Key);
			continue;
		}
		UMjCamera* Cam = *Found;

		// Mark consumed so per-camera capture gating keeps this camera live.
		Cam->TouchRequested();

		const uint64* MinIdPtr = MinFrameIds.Find(Spec.Key);
		const uint64 MinId = MinIdPtr ? *MinIdPtr : 0;

		FMjCameraFrame Frame;
		if (!Cam->GetFrame(MinId, Frame))
		{
			// Not ready yet (camera just activated, or the requested step's
			// frame hasn't been rendered/read back). Omit; the client retries
			// on a later step. First request of a dormant camera always misses.
			continue;
		}

		TSharedPtr<FJsonObject> CamObj = MakeShared<FJsonObject>();
		CamObj->SetNumberField(TEXT("width"), Frame.Width);
		CamObj->SetNumberField(TEXT("height"), Frame.Height);
		CamObj->SetNumberField(TEXT("frame_id"), static_cast<double>(Frame.FrameId));
		CamObj->SetNumberField(TEXT("sim_time"), Frame.SimTime);

		if (Cam->CaptureMode == EMjCameraMode::Depth)
		{
			if (Frame.Depth.Num() == 0)
				continue;
			CamObj->SetStringField(TEXT("dtype"), TEXT("float32"));
			FURLabMsgpackUtil::SetBinaryField(CamObj, TEXT("data"),
				reinterpret_cast<const uint8*>(Frame.Depth.GetData()),
				Frame.Depth.Num() * sizeof(float));
		}
		else
		{
			// Real / SemSeg / InstanceSeg all ship 4-byte BGRA. Bridge
			// discriminates the seg modes by the camera_topics handshake.
			if (Frame.Color.Num() == 0)
				continue;
			CamObj->SetStringField(TEXT("dtype"), TEXT("bgra8"));
			FURLabMsgpackUtil::SetBinaryField(CamObj, TEXT("data"),
				reinterpret_cast<const uint8*>(Frame.Color.GetData()),
				Frame.Color.Num() * sizeof(FColor));
		}
		Cams->SetObjectField(Spec.Key, CamObj);
	}
	return Cams;
}

