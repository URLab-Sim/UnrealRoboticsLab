// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "Bridge/GameThreadCommandQueue.h"

#include "Misc/ScopeLock.h"

bool FURLabGameThreadCommandQueue::Enqueue(FWork&& Work)
{
	TUniqueFunction<void()> Cancel;
	bool bStartDrain = false;
	{
		FScopeLock Lock(&Mutex);
		if (bClosed)
		{
			Cancel = MoveTemp(Work.Cancel);
		}
		else
		{
			Queue.Enqueue(MoveTemp(Work));
			if (!bDrainArmed)
			{
				bDrainArmed = true;
				bStartDrain = true;
			}
		}
	}
	if (Cancel) Cancel();
	return bStartDrain;
}

void FURLabGameThreadCommandQueue::Drain()
{
	check(IsInGameThread());
	{
		FScopeLock Lock(&Mutex);
		// A native callback can synchronously re-enter typed dispatch on the game
		// thread. The outer drainer retains ownership so accepted work cannot run
		// ahead of the operation that caused the callback.
		if (bDrainRunning) return;
		bDrainRunning = true;
	}
	for (;;)
	{
		FWork Work;
		{
			FScopeLock Lock(&Mutex);
			if (!Queue.Dequeue(Work))
			{
				bDrainArmed = false;
				bDrainRunning = false;
				return;
			}
		}
		if (Work.Execute) Work.Execute();
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
