// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "Bridge/GameThreadCommandQueue.h"

#include "HAL/Event.h"
#include "HAL/PlatformProcess.h"
#include "Misc/ScopeLock.h"

namespace
{
FCriticalSection GQueueRegistryMutex;
TArray<TWeakPtr<FURLabGameThreadCommandQueue, ESPMode::ThreadSafe>> GRegisteredQueues;
FEvent* GGameThreadWakeEvent = nullptr;

FEvent* EnsureWakeEventLocked()
{
	if (!GGameThreadWakeEvent)
	{
		GGameThreadWakeEvent = FPlatformProcess::GetSynchEventFromPool(false);
	}
	return GGameThreadWakeEvent;
}
} // namespace

void FURLabGameThreadCommandQueue::RegisterQueue(
	const TSharedRef<FURLabGameThreadCommandQueue, ESPMode::ThreadSafe>& Queue)
{
	FScopeLock Lock(&GQueueRegistryMutex);
	EnsureWakeEventLocked();
	GRegisteredQueues.RemoveAll([](const TWeakPtr<FURLabGameThreadCommandQueue, ESPMode::ThreadSafe>& Entry)
	{
		return !Entry.IsValid();
	});
	GRegisteredQueues.Add(Queue);
}

FEvent* FURLabGameThreadCommandQueue::GetWakeEvent()
{
	FScopeLock Lock(&GQueueRegistryMutex);
	return EnsureWakeEventLocked();
}

void FURLabGameThreadCommandQueue::NotifyWakeEvent()
{
	if (FEvent* Event = GetWakeEvent())
	{
		Event->Trigger();
	}
}

bool FURLabGameThreadCommandQueue::DrainAllOnGameThread()
{
	check(IsInGameThread());
	TArray<TSharedPtr<FURLabGameThreadCommandQueue, ESPMode::ThreadSafe>, TInlineAllocator<8>> LiveQueues;
	{
		FScopeLock Lock(&GQueueRegistryMutex);
		for (int32 Index = GRegisteredQueues.Num() - 1; Index >= 0; --Index)
		{
			if (TSharedPtr<FURLabGameThreadCommandQueue, ESPMode::ThreadSafe> Pinned =
				GRegisteredQueues[Index].Pin())
			{
				LiveQueues.Add(MoveTemp(Pinned));
			}
			else
			{
				GRegisteredQueues.RemoveAtSwap(Index);
			}
		}
	}
	bool bAnyExecuted = false;
	for (const TSharedPtr<FURLabGameThreadCommandQueue, ESPMode::ThreadSafe>& Queue : LiveQueues)
	{
		if (Queue)
		{
			bAnyExecuted |= Queue->Drain();
		}
	}
	return bAnyExecuted;
}

bool FURLabGameThreadCommandQueue::Enqueue(FWork&& Work)
{
	TUniqueFunction<void()> Cancel;
	bool bStartDrain = false;
	bool bQueued = false;
	{
		FScopeLock Lock(&Mutex);
		if (bClosed)
		{
			Cancel = MoveTemp(Work.Cancel);
		}
		else
		{
			Queue.Enqueue(MoveTemp(Work));
			bQueued = true;
			if (!bDrainArmed)
			{
				bDrainArmed = true;
				bStartDrain = true;
			}
		}
	}
	if (bQueued)
	{
		NotifyWakeEvent();
	}
	if (Cancel) Cancel();
	return bStartDrain;
}

bool FURLabGameThreadCommandQueue::Drain()
{
	check(IsInGameThread());
	{
		FScopeLock Lock(&Mutex);
		// A native callback can synchronously re-enter typed dispatch on the game
		// thread. The outer drainer retains ownership so accepted work cannot run
		// ahead of the operation that caused the callback.
		if (bDrainRunning) return false;
		bDrainRunning = true;
	}
	bool bExecutedAny = false;
	for (;;)
	{
		FWork Work;
		{
			FScopeLock Lock(&Mutex);
			if (!Queue.Dequeue(Work))
			{
				bDrainArmed = false;
				bDrainRunning = false;
				return bExecutedAny;
			}
		}
		if (Work.Execute)
		{
			bExecutedAny = true;
			Work.Execute();
		}
	}
}

void FURLabGameThreadCommandQueue::CloseAndCancel()
{
	TArray<FWork> Cancelled;
	{
		FScopeLock Lock(&Mutex);
		if (bClosed) return;
		bClosed = true;
		FWork Work;
		while (Queue.Dequeue(Work))
		{
			Cancelled.Add(MoveTemp(Work));
		}
		bDrainArmed = false;
	}
	for (FWork& Work : Cancelled)
	{
		if (Work.Cancel) Work.Cancel();
	}
}
