// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#pragma once

#include "CoreMinimal.h"
#include "Transport/PoseStreamProducer.h"
#include <atomic>

class FMjSimulationInteractionOwner;

/** Opaque, independently revocable command binding for one live pose producer. */
class URLAB_API FURLabPoseStreamServiceBinding
{
public:
	explicit FURLabPoseStreamServiceBinding(
		TSharedRef<FURLabPoseStreamProducer, ESPMode::ThreadSafe> InProducer,
		TSharedPtr<FMjSimulationInteractionOwner, ESPMode::ThreadSafe> InInteractionOwner = nullptr)
		: Producer(MoveTemp(InProducer)), InteractionOwner(MoveTemp(InInteractionOwner)) {}
	bool IsActive() const { return bActive.load(std::memory_order_acquire); }
	void Deactivate();
	TSharedPtr<FURLabPoseStreamProducer, ESPMode::ThreadSafe> ResolveProducer() const
	{
		return IsActive() ? Producer.ToSharedPtr()
			: TSharedPtr<FURLabPoseStreamProducer, ESPMode::ThreadSafe>();
	}
	TSharedPtr<FMjSimulationInteractionOwner, ESPMode::ThreadSafe>
	ResolveInteractionOwner() const
	{
		return IsActive() ? InteractionOwner : nullptr;
	}
private:
	std::atomic<bool> bActive{true};
	TSharedRef<FURLabPoseStreamProducer, ESPMode::ThreadSafe> Producer;
	TSharedPtr<FMjSimulationInteractionOwner, ESPMode::ThreadSafe> InteractionOwner;
};
