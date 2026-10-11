// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Transport/TransportBase.h"
// TWeakObjectPtr<UURLabBridgeServer> assignment needs the full type.
#include "Bridge/BridgeServer.h"

void UURLabTransportBase::SetOwningBridge(UURLabBridgeServer* Bridge)
{
	OwningBridge = Bridge;
}

void UURLabTransportBase::BeginDestroy()
{
	// Class-default and archetype objects never bound anything, and the role
	// bases leave TransportShutdown pure -- calling it on their CDO would trip
	// the pure-virtual trap during engine shutdown.
	if (!HasAnyFlags(RF_ClassDefaultObject | RF_ArchetypeObject))
	{
		TransportShutdown();
	}
	Super::BeginDestroy();
}
