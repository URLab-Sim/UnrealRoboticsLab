// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#include "Transport/RosRpcTransport.h"

#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2
#include "Transport/RosContext.h"
#include "Ros/UrlabRclCore.h"
#include "HAL/RunnableThread.h"
#include "HAL/Runnable.h"
#include "HAL/PlatformProcess.h"
#include "Utils/URLabLogging.h"

// Poll cadence for the executor loop. The wait set blocks up to this long once
// subscriptions exist; until then SpinSome returns immediately and the trailing
// sleep keeps the thread off a busy loop.
namespace
{
constexpr int64 RosSpinTimeoutNs = 50 * 1000 * 1000; // 50 ms
constexpr float RosIdleSleepSeconds = 0.02f;
}

class FRosExecutorRunnable : public FRunnable
{
public:
	explicit FRosExecutorRunnable(UURLabRosRpcTransport* InTransport)
		: Transport(InTransport) {}
	virtual uint32 Run() override
	{
		Transport->RunExecutorLoop();
		return 0;
	}
	virtual void Stop() override { Transport->bStop = true; }

private:
	UURLabRosRpcTransport* Transport;
};

bool UURLabRosRpcTransport::TransportInit()
{
	if (bIsInitialized)
	{
		return true;
	}
	if (!FURLabRosContext::Get().Initialize())
	{
		UE_LOG(LogURLab, Warning,
			TEXT("UURLabRosRpcTransport: ROS context unavailable; not binding."));
		return false;
	}

	bStop = false;
	bIsInitialized = true;
	WorkerRunnable = new FRosExecutorRunnable(this);
	WorkerThread = FRunnableThread::Create(WorkerRunnable, TEXT("URLabRosExecutor"));

	UE_LOG(LogURLab, Log, TEXT("UURLabRosRpcTransport initialised."));
	return true;
}

void UURLabRosRpcTransport::TransportShutdown()
{
	if (!bIsInitialized)
	{
		return;
	}
	bStop = true;
	if (WorkerThread)
	{
		WorkerThread->WaitForCompletion();
		delete WorkerThread;
		WorkerThread = nullptr;
	}
	// FRunnableThread never owns the runnable; delete it so the bind/unbind cycle
	// does not leak one runnable each time.
	delete WorkerRunnable;
	WorkerRunnable = nullptr;
	bIsInitialized = false;
}

void UURLabRosRpcTransport::RunExecutorLoop()
{
	while (!bStop.load(std::memory_order_acquire))
	{
		UrlabRclContext* Ctx = FURLabRosContext::Get().GetHandle();
		if (Ctx)
		{
			// No services or subscriptions are registered yet, so this drains the
			// (empty) wait set. ROS control-in adds subscriptions here and routes
			// their callbacks through ResolveDispatcher()->Dispatch().
			UrlabRcl_SpinSome(Ctx, RosSpinTimeoutNs);
		}
		FPlatformProcess::Sleep(RosIdleSleepSeconds);
	}
}

#else  // URLAB_WITH_ROS2

// Absent-ROS stubs so the class links; every real caller is fenced off too.
bool UURLabRosRpcTransport::TransportInit() { return false; }
void UURLabRosRpcTransport::TransportShutdown() {}
void UURLabRosRpcTransport::RunExecutorLoop() {}

#endif  // URLAB_WITH_ROS2
