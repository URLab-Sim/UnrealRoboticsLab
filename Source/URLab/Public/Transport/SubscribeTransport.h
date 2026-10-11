// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "Transport/TransportBase.h"
#include "SubscribeTransport.generated.h"

/**
 * @class UURLabSubscribeTransport
 * @brief Server-side control ingress: the inbound leg of the non-camera stream.
 *
 * Drains whatever an external controller sent since the last step and applies
 * it to the authoritative mjData just before mj_step. That is why it lives on
 * the physics thread's PreStep and has no worker of its own.
 *
 * Distinct from `UURLabClientSubscribeTransport`, which is a client-side render
 * sink with no bridge, no step hooks and no authority over the sim.
 */
UCLASS(Abstract)
class URLAB_API UURLabSubscribeTransport : public UURLabTransportBase
{
	GENERATED_BODY()
};
