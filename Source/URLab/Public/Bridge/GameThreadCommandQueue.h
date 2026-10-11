// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "Containers/Queue.h"
#include "Templates/Function.h"

/**
 * Shared, dispatcher-independent FIFO for accepted game-thread work.
 * Producers serialize admission separately; this mutex only protects the queue
 * and its one-drain flag. The drain invokes owned work after releasing it.
 */
class URLAB_API FURLabGameThreadCommandQueue
{
public:
	struct FWork
	{
		TUniqueFunction<void()> Execute;
		TUniqueFunction<void()> Cancel;
	};

	/** Enqueues one accepted operation. True asks the caller to schedule the drain. */
	bool Enqueue(FWork&& Work);
	/** Runs all queued operations in order on the game thread. Safe after the dispatcher is destroyed. */
	void Drain();
	/** Cancels queued work synchronously. Work already removed by Drain finishes normally. */
	void CloseAndCancel();

private:
	FCriticalSection Mutex;
	TQueue<FWork, EQueueMode::Mpsc> Queue;
	bool bDrainArmed = false;
	bool bDrainRunning = false;
	bool bClosed = false;
};
