// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Transport/ClientSubscribeTransport.h"

#include "Transport/ZmqClientSubscribeTransport.h"

UURLabClientSubscribeTransport* UURLabClientSubscribeTransport::Create(UObject* Outer,
	const FString& Endpoint, const FString& Topic, FOnClientMessage Callback)
{
	UURLabClientSubscribeTransport* Sub = NewObject<UURLabZmqClientSubscribeTransport>(Outer);
	Sub->Configure(Endpoint, Topic, Callback);
	if (!Sub->TransportInit())
	{
		return nullptr;
	}
	return Sub;
}
