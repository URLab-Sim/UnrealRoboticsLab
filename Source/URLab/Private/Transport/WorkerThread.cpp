// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Transport/WorkerThread.h"

#include "HAL/RunnableThread.h"

FURLabWorkerThread::~FURLabWorkerThread()
{
	Stop();
}

bool FURLabWorkerThread::Start(const TCHAR* ThreadName, TFunction<void()> Body,
	TFunction<void()> OnStop)
{
	if (Thread != nullptr)
	{
		return true;
	}

	Runnable = new FBody(MoveTemp(Body), MoveTemp(OnStop));
	Thread = FRunnableThread::Create(Runnable, ThreadName);
	if (Thread == nullptr)
	{
		delete Runnable;
		Runnable = nullptr;
		return false;
	}
	return true;
}

void FURLabWorkerThread::Stop()
{
	if (Thread != nullptr)
	{
		// Kill(true) calls FRunnable::Stop (which sets the caller's stop flag)
		// and then joins, so a caller that already set the flag pays nothing
		// extra and one that forgot is still safe.
		Thread->Kill(/*bShouldWait=*/true);
		delete Thread;
		Thread = nullptr;
	}
	// FRunnableThread never owns the runnable; this is the delete every
	// transport used to have to remember.
	delete Runnable;
	Runnable = nullptr;
}
