// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "Bridge/SerialCommandQueue.h"

#include "Misc/ScopeLock.h"

bool FURLabSerialCommandQueue::Enqueue(FWork&& Work)
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

void FURLabSerialCommandQueue::Drain()
{
	{
		FScopeLock Lock(&Mutex);
		if (bDrainRunning || bClosed) return;
		bDrainRunning = true;
	}
	for (;;)
	{
		FWork Work;
		{
			FScopeLock Lock(&Mutex);
			if (bClosed || !Queue.Dequeue(Work))
			{
				bDrainArmed = false;
				bDrainRunning = false;
				return;
			}
		}
		if (Work.Execute) Work.Execute();
	}
}

void FURLabSerialCommandQueue::CloseAndCancel()
{
	TArray<FWork> Cancelled;
	{
		FScopeLock Lock(&Mutex);
		if (bClosed) return;
		bClosed = true;
		FWork Work;
		while (Queue.Dequeue(Work)) Cancelled.Add(MoveTemp(Work));
		bDrainArmed = false;
	}
	for (FWork& Work : Cancelled)
	{
		if (Work.Cancel) Work.Cancel();
	}
}
