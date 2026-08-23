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
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Entity/MjEntity.h"
#include "MuJoCo/Entity/MjControl.h"
#include "MuJoCo/Entity/MjControlIngress.h"
#include "zmq.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Utils/MsgpackHelpers.h"
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
	int rcControl = zmq_bind(ControlSubscriber, TCHAR_TO_UTF8(*ControlEndpoint));
	if (rcControl != 0)
	{
		UE_LOG(LogURLabNet, Error, TEXT("Failed to bind Control SUB at %s"), *ControlEndpoint);
		if (GEngine)
		{
			GEngine->AddOnScreenDebugMessage(-1, 10.f, FColor::Red,
				FString::Printf(TEXT("URLab: ZMQ bind failed on %s — check for port conflicts"), *ControlEndpoint));
		}
	}

	AAMjManager* Manager = OwningManager.Get();
	if (Manager && Manager->PhysicsEngine)
	{
		for (const FMjEntity& E : Manager->PhysicsEngine->GetEntityPartition())
		{
			FString ControlFilter = FString::Printf(TEXT("%s/control "), *E.Name.ToString());
			{
				const FTCHARToUTF8 FilterUtf8(*ControlFilter);
				zmq_setsockopt(ControlSubscriber, ZMQ_SUBSCRIBE, FilterUtf8.Get(), FilterUtf8.Length());
			}
			UE_LOG(LogURLabNet, Log, TEXT("ZmqControlSubscriber Subscribed to: %s"), *ControlFilter);
		}
	}
	else
	{
		UE_LOG(LogURLabNet, Warning, TEXT("ZmqControlSubscriber: Parent is not AAMuJoCoManager!"));
		// Fallback generic subscribe if testing
		zmq_setsockopt(ControlSubscriber, ZMQ_SUBSCRIBE, "control ", 8);
	}

	// A failed bind means the transport cannot function. Free all resources
	// and leave bIsInitialized=false so callers (and ShutdownZmqSocket) see
	// the failure instead of a false-positive init.
	if (rcControl != 0)
	{
		zmq_close(ControlSubscriber);
		zmq_ctx_term(ZmqContext);
		ControlSubscriber = nullptr;
		ZmqContext = nullptr;
		return;
	}

	bIsInitialized = true;
	UE_LOG(LogURLabNet, Log, TEXT("ZmqControlSubscriber Initialized."));
}

void UURLabZmqSubscribeTransport::ShutdownZmqSocket()
{
	if (!bIsInitialized)
		return;

	zmq_close(ControlSubscriber);
	zmq_ctx_term(ZmqContext);

	ControlSubscriber = nullptr;
	ZmqContext = nullptr;
	bIsInitialized = false;
}

void UURLabZmqSubscribeTransport::BuildCache(mjModel* m)
{
	ActuatorToEntityName.Empty();
	if (!m)
		return;

	AAMjManager* Manager = OwningManager.Get();
	if (!Manager || !Manager->PhysicsEngine)
		return;

	for (const FMjEntity& E : Manager->PhysicsEngine->GetEntityPartition())
	{
		for (int32 Id : E.ActuatorIds)
		{
			if (Id < 0 || Id >= m->nu)
				continue;
			ActuatorToEntityName.Add(Id, E.Name);
		}
	}
	bCacheBuilt = true;
	UE_LOG(LogURLabNet, Log, TEXT("ZmqControlSubscriber: Built cache for %d actuators"), ActuatorToEntityName.Num());
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

		const int32 PayloadSize = (int32)zmq_msg_size(&payload_msg);
		const uint8* PayloadData = (const uint8*)zmq_msg_data(&payload_msg);

		// --- Handle control messages ---
		// Payload is a msgpack map `{ids:[...], vals:[...]}` (source-of-truth
		// §9.3). Parsed via FURLabMsgpackUtil -- bounds-checked, no raw casts.
		TSharedPtr<FJsonObject> Payload;
		if (PayloadSize > 0 && FURLabMsgpackUtil::UnpackToJsonObject(PayloadData, PayloadSize, Payload) && Payload.IsValid())
		{
			AAMjManager* Manager = OwningManager.Get();
			if (Manager && Manager->PhysicsEngine)
			{
				IMjControlIngress* Ingress = Manager->PhysicsEngine->GetControlIngress();

				const TArray<TSharedPtr<FJsonValue>>* IdsArr = nullptr;
				const TArray<TSharedPtr<FJsonValue>>* ValsArr = nullptr;
				if (Payload->TryGetArrayField(TEXT("ids"), IdsArr) && IdsArr &&
					Payload->TryGetArrayField(TEXT("vals"), ValsArr) && ValsArr)
				{
					const int32 NumControls = FMath::Min(IdsArr->Num(), ValsArr->Num());

					static constexpr int32 kControlLogInterval = 500;
					const bool bShouldLog = (++ControlLogCounter % kControlLogInterval == 1); // Log every 500th batch

					if (IdsArr->Num() != ValsArr->Num())
					{
						UE_LOG(LogURLabNet, Warning, TEXT("ZmqControl: ids/vals length mismatch (ids=%d, vals=%d) -- applying %d"),
							IdsArr->Num(), ValsArr->Num(), NumControls);
					}

					for (int32 i = 0; i < NumControls; ++i)
					{
						const TSharedPtr<FJsonValue>& IdVal = (*IdsArr)[i];
						const TSharedPtr<FJsonValue>& ValVal = (*ValsArr)[i];
						if (!IdVal.IsValid() || !ValVal.IsValid())
							continue;

						const int32 Idx = (int32)IdVal->AsNumber();
						const double Value = ValVal->AsNumber();

						if (const FName* EntityName = ActuatorToEntityName.Find(Idx))
						{
							// A remote RPC owner takes exclusive control of its entity's actuators;
							// the network stream stays hands-off until the claim is released.
							const bool bOwnedElsewhere =
								Manager->PhysicsEngine->GetControlOwners().Contains(*EntityName);
							if (!bOwnedElsewhere && Ingress)
							{
								Ingress->WriteCtrl(*EntityName, Idx, Value);
							}
						}
						else if (bShouldLog)
						{
							UE_LOG(LogURLabNet, Warning, TEXT("ZmqControl: Actuator ID %d not found in cache (cache size: %d)"), Idx, ActuatorToEntityName.Num());
						}
					}

					if (bShouldLog && NumControls > 0)
					{
						const double FirstVal = (*ValsArr)[0].IsValid() ? (*ValsArr)[0]->AsNumber() : 0.0;
						UE_LOG(LogURLabNet, Log, TEXT("ZmqControl: Applied %d controls (first val: %.4f, cache size: %d)"), NumControls, FirstVal, ActuatorToEntityName.Num());
					}
				}
				else
				{
					UE_LOG(LogURLabNet, Warning, TEXT("ZmqControl: control payload missing 'ids'/'vals' arrays"));
				}
			}
		}
		else if (PayloadSize > 0)
		{
			UE_LOG(LogURLabNet, Warning, TEXT("ZmqControl: failed to unpack msgpack control payload (%d bytes)"), PayloadSize);
		}

		zmq_msg_close(&payload_msg);
	}
}
