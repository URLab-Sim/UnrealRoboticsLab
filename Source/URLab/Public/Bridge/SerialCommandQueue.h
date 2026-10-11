// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "Containers/Queue.h"
#include "Templates/Function.h"

/**
 * Dispatcher-independent FIFO for short background domain operations.
 *
 * Enqueue order is established by the caller's admission lock. Exactly one
 * drainer executes work at a time. CloseAndCancel owns and completes every
 * queued record synchronously; a record already removed by Drain completes
 * with its truthful domain outcome.
 */
class URLAB_API FURLabSerialCommandQueue
{
public:
	struct FWork
	{
		TUniqueFunction<void()> Execute;
		TUniqueFunction<void()> Cancel;
	};

	/** Enqueue accepted work. True asks the caller to schedule one background drain. */
	bool Enqueue(FWork&& Work);
	/** Drain accepted work serially. Safe after the dispatcher has been destroyed. */
	void Drain();
	/** Cancel all queued records now. Already-running work finishes normally. */
	void CloseAndCancel();

private:
	FCriticalSection Mutex;
	TQueue<FWork, EQueueMode::Mpsc> Queue;
	bool bDrainArmed = false;
	bool bDrainRunning = false;
	bool bClosed = false;
};
