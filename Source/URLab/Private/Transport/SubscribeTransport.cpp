// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Transport/SubscribeTransport.h"
// TWeakObjectPtr<UURLabBridgeServer> assignment needs the full type.
#include "Bridge/BridgeServer.h"

void UURLabSubscribeTransport::SetOwningBridge(UURLabBridgeServer* Bridge)
{
	OwningBridge = Bridge;
}
