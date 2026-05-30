// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "SubscribeTransport.generated.h"

class UURLabBridgeServer;

/**
 * @class UURLabSubscribeTransport
 * @brief Abstract base for client -> server streaming transports.
 *
 * Folds in `UURLabZmqSubscribeTransport` (xfrc / external-control SUB
 * channel). Future ROS analogue: `rclcpp::Subscription`.
 *
 * Concrete transports own a worker thread that pumps inbound messages
 * onto a per-topic delegate. The bridge wires `Subscribe(topic, cb)`
 * once per topic; the callback fires from the worker thread, so
 * implementations must not assume game-thread context.
 */
UCLASS(Abstract)
class URLAB_API UURLabSubscribeTransport : public UObject
{
    GENERATED_BODY()

public:
    /** Per-message delegate: topic + raw bytes. Callback runs on the
     *  transport's worker thread. */
    DECLARE_DELEGATE_TwoParams(FOnMessage, const FString& /*Topic*/,
                                           const TArray<uint8>& /*Bytes*/);

    void SetOwningBridge(UURLabBridgeServer* Bridge);
    UURLabBridgeServer* GetOwningBridge() const { return OwningBridge.Get(); }

    /** Register a callback for `Topic`. Backends route inbound messages
     *  on that topic to the supplied delegate. Calling Subscribe twice
     *  on the same topic replaces the prior callback. */
    virtual void Subscribe(const FString& Topic,
                           FOnMessage Callback) PURE_VIRTUAL(UURLabSubscribeTransport::Subscribe, );

    virtual bool TransportInit() PURE_VIRTUAL(UURLabSubscribeTransport::TransportInit, return false;);
    virtual void TransportShutdown() PURE_VIRTUAL(UURLabSubscribeTransport::TransportShutdown, );
    virtual FString GetTransportName() const PURE_VIRTUAL(UURLabSubscribeTransport::GetTransportName, return FString(); );

    // --- Per-step physics hooks (Async thread) ----------------------------
    // Default-empty. Concrete subscribe transports that drain inbound
    // messages into mjData on each step (e.g. UURLabZmqSubscribeTransport)
    // override PreStep to apply control / xfrc just before mj_step.

    virtual void PreStep(struct mjModel_* /*m*/, struct mjData_* /*d*/) {}
    virtual void PostStep(struct mjModel_* /*m*/, struct mjData_* /*d*/) {}

protected:
    TWeakObjectPtr<UURLabBridgeServer> OwningBridge;
};
