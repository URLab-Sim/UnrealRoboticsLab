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

#include "Transport/ZmqSubscribeTransport.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Core/AMjManager.h"
#include "Transport/NetworkManager.h"
#include "MuJoCo/Elements/MjActuatorRuntime.h"
#include "zmq.h"
#include "MuJoCo/Elements/MjCamera.h"
#include "Serialization/JsonSerializer.h"
#include "Dom/JsonObject.h"
#include "Policies/CondensedJsonPrintPolicy.h"
#include "Utils/URLabLogging.h"

void UURLabZmqSubscribeTransport::SetOwningManager(AAMjManager* InMgr)
{
	OwningManager = InMgr;
}

bool UURLabZmqSubscribeTransport::TransportInit()
{
	InitZmqSocket();
	return bIsInitialized;
}

void UURLabZmqSubscribeTransport::TransportShutdown()
{
	ShutdownZmqSocket();
}

void UURLabZmqSubscribeTransport::InitZmqSocket()
{
	if (bIsInitialized)
		return;

	ZmqContext = zmq_ctx_new();

	// Setup Subscriber (Controls)
	ControlSubscriber = zmq_socket(ZmqContext, ZMQ_SUB);
	int rc = zmq_bind(ControlSubscriber, TCHAR_TO_UTF8(*ControlEndpoint));
	if (rc != 0)
	{
		UE_LOG(LogURLabNet, Error, TEXT("Failed to bind Control SUB at %s"), *ControlEndpoint);
		if (GEngine)
		{
			GEngine->AddOnScreenDebugMessage(-1, 10.f, FColor::Red,
				FString::Printf(TEXT("URLab: ZMQ bind failed on %s — check for port conflicts"), *ControlEndpoint));
		}
	}

	AAMjManager* Manager = OwningManager.Get();
	if (Manager)
	{
		for (AMjArticulation* Artic : Manager->GetAllArticulations())
		{
			if (Artic)
			{
				FString ControlFilter = FString::Printf(TEXT("%s/control "), *Artic->GetName());
				{
					const FTCHARToUTF8 FilterUtf8(*ControlFilter);
					zmq_setsockopt(ControlSubscriber, ZMQ_SUBSCRIBE, FilterUtf8.Get(), FilterUtf8.Length());
				}
				UE_LOG(LogURLabNet, Log, TEXT("ZmqControlSubscriber Subscribed to: %s"), *ControlFilter);
			}
		}
	}
	else
	{
		UE_LOG(LogURLabNet, Warning, TEXT("ZmqControlSubscriber: Parent is not AAMuJoCoManager!"));
		// Fallback generic subscribe if testing
		zmq_setsockopt(ControlSubscriber, ZMQ_SUBSCRIBE, "control ", 8);
	}

	// Setup Publisher (Info)
	InfoPublisher = zmq_socket(ZmqContext, ZMQ_PUB);
	rc = zmq_bind(InfoPublisher, TCHAR_TO_UTF8(*InfoEndpoint));
	if (rc != 0)
	{
		UE_LOG(LogURLabNet, Error, TEXT("Failed to bind Info PUB at %s"), *InfoEndpoint);
		if (GEngine)
		{
			GEngine->AddOnScreenDebugMessage(-1, 10.f, FColor::Red,
				FString::Printf(TEXT("URLab: ZMQ bind failed on %s — check for port conflicts"), *InfoEndpoint));
		}
	}

	bIsInitialized = true;
	UE_LOG(LogURLabNet, Log, TEXT("ZmqControlSubscriber Initialized."));
}

void UURLabZmqSubscribeTransport::ShutdownZmqSocket()
{
	if (!bIsInitialized)
		return;

	zmq_close(ControlSubscriber);
	zmq_close(InfoPublisher);
	zmq_ctx_term(ZmqContext);

	ControlSubscriber = nullptr;
	InfoPublisher = nullptr;
	ZmqContext = nullptr;
	bIsInitialized = false;
}

void UURLabZmqSubscribeTransport::BuildCache(mjModel* m)
{
	ActuatorCache.Empty();
	ActuatorToArticulationName.Empty();
	if (!m)
		return;

	AAMjManager* Manager = OwningManager.Get();
	if (!Manager)
		return;

	for (AMjArticulation* Articulation : Manager->GetAllArticulations())
	{
		if (!Articulation)
			continue;

		const FName ArtName(*Articulation->GetName());
		for (UMjNodeComponent* Actuator : Articulation->GetActuators())
		{
			if (Actuator == nullptr || !Actuator->GetBoundId().IsSet())
			{
				continue;
			}
			const int32 Id = Actuator->GetBoundId().GetValue();
			ActuatorCache.Add(Actuator->MjName.Get(Actuator->GetName()), Id);
			ActuatorComponentCache.Add(Id, Actuator);
			ActuatorToArticulationName.Add(Id, ArtName);
		}
	}
	bCacheBuilt = true;
	UE_LOG(LogURLabNet, Log, TEXT("ZmqControlSubscriber: Built cache for %d actuators"), ActuatorCache.Num());
}

void UURLabZmqSubscribeTransport::BroadcastInfo(mjModel* m)
{
	if (!InfoPublisher)
		return;

	AAMjManager* Manager = OwningManager.Get();
	if (!Manager)
		return;

	// Broadcast an info message per robot
	for (AMjArticulation* Articulation : Manager->GetAllArticulations())
	{
		if (!Articulation)
			continue;

		FString ArticName = Articulation->GetName();
		FString Prefix = ArticName + "_";

		TSharedPtr<FJsonObject> RootObject = MakeShareable(new FJsonObject);
		RootObject->SetStringField("type", "actuator_list");
		RootObject->SetStringField("robot", ArticName);

		TArray<TSharedPtr<FJsonValue>> NamesArray;
		TArray<TSharedPtr<FJsonValue>> IdsArray;
		TArray<TSharedPtr<FJsonValue>> MinsArray;
		TArray<TSharedPtr<FJsonValue>> MaxsArray;

		for (auto& Pair : ActuatorCache)
		{
			// Only include actuators for this specific robot
			if (Pair.Key.StartsWith(Prefix))
			{
				NamesArray.Add(MakeShareable(new FJsonValueString(Pair.Key)));
				IdsArray.Add(MakeShareable(new FJsonValueNumber(Pair.Value)));

				static constexpr float kDefaultCtrlMin = -100.0f;
				static constexpr float kDefaultCtrlMax = 100.0f;
				int id = Pair.Value;
				float min_val = kDefaultCtrlMin; // Default reasonable fallback if not limited
				float max_val = kDefaultCtrlMax;
				if (m->actuator_ctrllimited[id])
				{
					min_val = (float)m->actuator_ctrlrange[id * 2];
					max_val = (float)m->actuator_ctrlrange[id * 2 + 1];
				}
				MinsArray.Add(MakeShareable(new FJsonValueNumber(min_val)));
				MaxsArray.Add(MakeShareable(new FJsonValueNumber(max_val)));
			}
		}

		RootObject->SetArrayField("names", NamesArray);
		RootObject->SetArrayField("ids", IdsArray);
		RootObject->SetArrayField("mins", MinsArray);
		RootObject->SetArrayField("maxs", MaxsArray);

		// NEW: Include all Cameras for discovery
		TArray<TSharedPtr<FJsonValue>> CameraArray;

		TArray<UMjCamera*> ActiveCameras = Manager->NetworkManager ? Manager->NetworkManager->GetActiveCameras() : TArray<UMjCamera*>();
		for (UMjCamera* Cam : ActiveCameras)
		{
			if (Cam && Cam->GetWorld() == Manager->GetWorld())
			{
				TSharedPtr<FJsonObject> CamObj = MakeShareable(new FJsonObject);
				CamObj->SetStringField("name", Cam->GetName());

				FString Endpoint = Cam->GetActualZmqEndpoint();
				// Convert wildcard bind address back to local loopback for the Python client
				Endpoint.ReplaceInline(TEXT("*"), TEXT("127.0.0.1"));
				CamObj->SetStringField("endpoint", Endpoint);

				CameraArray.Add(MakeShareable(new FJsonValueObject(CamObj)));
			}
		}
		RootObject->SetArrayField("camera_list", CameraArray);

		FString JsonString;
		TSharedRef<TJsonWriter<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>> Writer = TJsonWriterFactory<TCHAR, TCondensedJsonPrintPolicy<TCHAR>>::Create(&JsonString);
		FJsonSerializer::Serialize(RootObject.ToSharedRef(), Writer);

		// Send via ZMQ. UTF-8 byte count, not TCHAR count.
		const FTCHARToUTF8 JsonUtf8(*JsonString);
		int rc = zmq_send(InfoPublisher, JsonUtf8.Get(), JsonUtf8.Length(), 0);
		if (rc == -1)
		{
			UE_LOG(LogURLabNet, Error, TEXT("ZmqControlSubscriber: FAILED to broadcast Info JSON!"));
		}
	}
}

void UURLabZmqSubscribeTransport::PreStep(mjModel* m, mjData* d)
{
	if (!bIsInitialized)
	{
		InitZmqSocket();
		if (!bIsInitialized)
			return;
	}

	if (!bCacheBuilt)
	{
		BuildCache(m);
	}

	// Step server owns the ctrl write path while paused — drain any inbound
	// SUB messages but do NOT broadcast actuator info or apply control. Drain
	// avoids a backlog blowing up the queue while we're in stepped mode.
	if (AAMjManager* Mgr = OwningManager.Get())
	{
		if (Mgr->bPublishersPaused.load(std::memory_order_acquire))
		{
			while (true)
			{
				zmq_msg_t Drain;
				zmq_msg_init(&Drain);
				int rc = zmq_msg_recv(&Drain, ControlSubscriber, ZMQ_DONTWAIT);
				zmq_msg_close(&Drain);
				if (rc == -1)
					break;
			}
			return;
		}
	}

	// Broadcast Info: frequently at startup (every 50 steps for first 5s),
	// then periodically (every 500 steps ~1s)
	static constexpr int32 kInfoBroadcastFast = 50;
	static constexpr int32 kInfoBroadcastSlow = 500;
	int BroadcastInterval = (TotalStepCount < 2500) ? kInfoBroadcastFast : kInfoBroadcastSlow;
	if (++InfoBroadcastCounter >= BroadcastInterval)
	{
		BroadcastInfo(m);
		InfoBroadcastCounter = 0;
	}
	TotalStepCount++;

	// Read all available messages from SUB socket (Non-blocking)
	while (true)
	{
		zmq_msg_t msg;
		zmq_msg_init(&msg);
		int rc = zmq_msg_recv(&msg, ControlSubscriber, ZMQ_DONTWAIT);

		if (rc == -1)
		{
			zmq_msg_close(&msg);
			break;
		}

		// Extract topic for routing
		int TopicSize = zmq_msg_size(&msg);
		TArray<char> TopicBuf;
		TopicBuf.SetNum(TopicSize + 1);
		FMemory::Memcpy(TopicBuf.GetData(), zmq_msg_data(&msg), TopicSize);
		TopicBuf[TopicSize] = '\0';
		FString Topic = UTF8_TO_TCHAR(TopicBuf.GetData());
		Topic.TrimEndInline();

		// Support multi-part topic filtering
		int more;
		size_t more_size = sizeof(more);
		zmq_getsockopt(ControlSubscriber, ZMQ_RCVMORE, &more, &more_size);
		zmq_msg_close(&msg);

		if (!more)
			continue;

		// Receive Payload Frame
		zmq_msg_t payload_msg;
		zmq_msg_init(&payload_msg);
		rc = zmq_msg_recv(&payload_msg, ControlSubscriber, 0);
		if (rc == -1)
		{
			zmq_msg_close(&payload_msg);
			break;
		}

		int size = zmq_msg_size(&payload_msg);
		char* data = (char*)zmq_msg_data(&payload_msg);

		// --- Handle control messages ---
		if (size >= 4)
		{
			AAMjManager* Manager = OwningManager.Get();
			if (Manager)
			{
				// Assumes x86-64 alignment and little-endian. For cross-platform, use memcpy + ntohl.
				int32 NumControls = *(int32*)(data);
				int32 ExpectedSize = 4 + NumControls * 8; // 4 + (4 + 4) * N

				if (size >= ExpectedSize)
				{
					int32* IDPtr = (int32*)(data + 4);
					float* ValPtr = (float*)(data + 8);

					static constexpr int32 kControlLogInterval = 500;
					bool bShouldLog = (++ControlLogCounter % kControlLogInterval == 1); // Log every 500th batch

					for (int i = 0; i < NumControls; ++i)
					{
						int32 Idx = *IDPtr;
						float Value = *ValPtr;

						if (UMjNodeComponent** ActuatorPtr = ActuatorComponentCache.Find(Idx))
						{
							if (*ActuatorPtr)
							{
								FURLabRpcDispatcher* Dispatcher = Manager->GetStepDispatcher();
								if (Dispatcher)
								{
									const FName* ArtName = ActuatorToArticulationName.Find(Idx);
									if (ArtName && Dispatcher->GetControlOwnership().GetActiveOwners().Contains(*ArtName))
									{
										IDPtr = (int32*)((char*)IDPtr + 8);
										ValPtr = (float*)((char*)ValPtr + 8);
										continue;
									}
								}
								UMjActuatorRuntime::SetNetworkControl(*ActuatorPtr, Value);
							}
						}
						else if (bShouldLog)
						{
							UE_LOG(LogURLabNet, Warning, TEXT("ZmqControl: Actuator ID %d not found in cache (cache size: %d)"), Idx, ActuatorComponentCache.Num());
						}

						IDPtr = (int32*)((char*)IDPtr + 8);
						ValPtr = (float*)((char*)ValPtr + 8);
					}

					if (bShouldLog)
					{
						UE_LOG(LogURLabNet, Log, TEXT("ZmqControl: Applied %d controls (first val: %.4f, cache size: %d)"), NumControls, NumControls > 0 ? *(float*)(data + 8) : 0.0f, ActuatorComponentCache.Num());
					}
				}
				else
				{
					UE_LOG(LogURLabNet, Warning, TEXT("ZmqControl: Size mismatch — got %d bytes, expected %d (NumControls=%d)"), size, ExpectedSize, NumControls);
				}
			}
		}

		zmq_msg_close(&payload_msg);
	}
}
