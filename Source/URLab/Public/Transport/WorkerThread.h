// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"
#include "Templates/Function.h"

class FRunnableThread;

/**
 * @class FURLabWorkerThread
 * @brief One worker thread + its runnable, owned together.
 *
 * Five transports each declared their own FRunnable subclass whose entire body
 * was `Owner->RunSomething()` in Run and `Owner->bStop = true` in Stop, then
 * each hand-managed the `FRunnableThread*` / `FRunnable*` pair -- FRunnableThread
 * does not own its runnable, so every one of them had to remember to delete it
 * or leak one per bind/unbind cycle. This is that shape, once.
 *
 * `Body` runs on the worker until it returns. `OnStop` is invoked from
 * FRunnable::Stop and must do nothing but set the flag the body polls: it runs
 * on the thread calling Stop, not on the worker.
 *
 * Stop() joins. The destructor stops, so a transport that forgets cannot leave
 * a thread running against freed state -- but a transport owning ZMQ sockets
 * must still stop the worker BEFORE closing them.
 */
class URLAB_API FURLabWorkerThread
{
public:
	FURLabWorkerThread() = default;
	~FURLabWorkerThread();

	FURLabWorkerThread(const FURLabWorkerThread&) = delete;
	FURLabWorkerThread& operator=(const FURLabWorkerThread&) = delete;

	/** Start `Body` on a new thread named `ThreadName`. Returns false when the
	 *  platform could not create the thread, in which case nothing is owned.
	 *  Calling Start on an already-running worker is a no-op returning true. */
	bool Start(const TCHAR* ThreadName, TFunction<void()> Body, TFunction<void()> OnStop);

	/** Signal, join, and release the thread and its runnable. Idempotent. */
	void Stop();

	bool IsRunning() const { return Thread != nullptr; }

private:
	/** The shim itself: the only FRunnable subclass in the transport layer. */
	class FBody final : public FRunnable
	{
	public:
		FBody(TFunction<void()> InBody, TFunction<void()> InOnStop)
			: BodyFn(MoveTemp(InBody)), OnStopFn(MoveTemp(InOnStop)) {}
		virtual uint32 Run() override
		{
			if (BodyFn) { BodyFn(); }
			return 0;
		}
		virtual void Stop() override
		{
			if (OnStopFn) { OnStopFn(); }
		}

	private:
		TFunction<void()> BodyFn;
		TFunction<void()> OnStopFn;
	};

	FBody* Runnable = nullptr;
	FRunnableThread* Thread = nullptr;
};
