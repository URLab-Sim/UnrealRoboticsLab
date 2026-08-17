// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. This plugin incorporates
// third-party software: MuJoCo (Apache 2.0). See ThirdPartyNotices.txt.

#include "MuJoCo/Fast/MjbTransportBus.h"

#include "Transport/ClientSubscribeTransport.h"
#include "Utils/URLabLogging.h"

void UMjbTransportBus::Start(const FString& Endpoint)
{
	if (Endpoint.IsEmpty() || BusTransport)
	{
		return;
	}
	// The renderer subscribes to the owner's "geoms" broadcast through the agnostic
	// client-subscribe transport (backend chosen by the base). The worker delivers
	// each newest payload to OnBusMessage; the game thread decodes + applies it in Tick.
	BusTransport = UURLabClientSubscribeTransport::Create(this, Endpoint, TEXT("geoms"),
		UURLabClientSubscribeTransport::FOnClientMessage::CreateUObject(this, &UMjbTransportBus::OnBusMessage));
	if (!BusTransport)
	{
		UE_LOG(LogURLab, Error, TEXT("[MjbScene] transform bus connect failed: %s"), *Endpoint);
		return;
	}
	bRxPending = false;
	UE_LOG(LogURLab, Log, TEXT("[MjbScene] subscribing to transform bus %s"), *Endpoint);
}

void UMjbTransportBus::Stop()
{
	if (BusTransport)
	{
		BusTransport->TransportShutdown();
		BusTransport = nullptr;
	}
	FScopeLock Lock(&FrameMutex);
	RxFrame.Reset();
	bRxPending = false;
}

bool UMjbTransportBus::IsConnected() const
{
	return BusTransport != nullptr;
}

bool UMjbTransportBus::TakeLatestFrame(TArray<uint8>& Out)
{
	FScopeLock Lock(&FrameMutex);
	if (bRxPending && RxFrame.Num() > 0)
	{
		Out = RxFrame;
		bRxPending = false;
		return true;
	}
	return false;
}

void UMjbTransportBus::OnBusMessage(const FString& /*Topic*/, const TArray<uint8>& Payload)
{
	// Worker thread: no UObject / msgpack work here -- just stash the newest raw
	// payload for the game thread (Tick) to decode + apply.
	if (Payload.Num() <= 0)
	{
		return;
	}
	{
		FScopeLock Lock(&FrameMutex);
		RxFrame = Payload;
		bRxPending = true;
	}
	bEverReceived.store(true, std::memory_order_release);
}
