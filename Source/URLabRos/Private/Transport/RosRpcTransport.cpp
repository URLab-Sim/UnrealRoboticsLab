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

// The node name FURLabRosContext creates the process ROS node under; the control
// source id every ROS write is tagged with derives from it. ROS-agnostic, so it
// is defined outside the ROS fence.
FString UURLabRosRpcTransport::RosControlSourceId()
{
	return TEXT("ros:urlab");
}

#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2
#include "Transport/RosContext.h"
#include "Ros/UrlabRclCore.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/RpcDispatcher.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "MuJoCo/Input/MjTwistController.h"
#include "State/MjCanonicalName.h"
#include "HAL/RunnableThread.h"
#include "HAL/Runnable.h"
#include "HAL/PlatformProcess.h"
#include "URLabRosLog.h"

// Poll cadence for the executor loop. The wait set blocks up to this long when
// subscriptions exist; the trailing sleep keeps the thread off a busy loop when
// no manager is live and there is nothing to spin.
namespace
{
constexpr int64 RosSpinTimeoutNs = 50 * 1000 * 1000; // 50 ms
constexpr float RosIdleSleepSeconds = 0.02f;
}

// Per-articulation command binding. Holds the identity used to resolve + gate the
// write (Art->GetName(), the GetArticulation + ownership key) and the two
// subscription handles. The stable heap address is handed to the core as the
// callback User pointer.
struct FRosArtCommand
{
	UURLabRosRpcTransport* Transport = nullptr;
	FString ArtName;
	UrlabRclCtrlSub* CtrlSub = nullptr;
	UrlabRclTwistSub* TwistSub = nullptr;
};

namespace
{
void RosCtrlTrampoline(const double* Values, int32_t Count, void* User)
{
	FRosArtCommand* Cmd = static_cast<FRosArtCommand*>(User);
	if (Cmd && Cmd->Transport)
	{
		Cmd->Transport->HandleRosCtrl(Cmd->ArtName, Values, static_cast<int32>(Count));
	}
}

void RosTwistTrampoline(const double Linear[3], const double Angular[3], void* User)
{
	FRosArtCommand* Cmd = static_cast<FRosArtCommand*>(User);
	if (Cmd && Cmd->Transport)
	{
		Cmd->Transport->HandleRosTwist(Cmd->ArtName, Linear, Angular);
	}
}
}  // namespace

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
		UE_LOG(LogURLabRos, Warning,
			TEXT("UURLabRosRpcTransport: ROS context unavailable; not binding."));
		return false;
	}

	bStop = false;
	bIsInitialized = true;
	WorkerRunnable = new FRosExecutorRunnable(this);
	WorkerThread = FRunnableThread::Create(WorkerRunnable, TEXT("URLabRosExecutor"));

	UE_LOG(LogURLabRos, Log, TEXT("UURLabRosRpcTransport initialised."));
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
	// The executor loop tears its subscriptions down on exit; this guards the case
	// where the thread never started.
	TeardownCommandSubscriptions();
	bIsInitialized = false;
}

void UURLabRosRpcTransport::RunExecutorLoop()
{
	while (!bStop.load(std::memory_order_acquire))
	{
		UrlabRclContext* Ctx = FURLabRosContext::Get().GetHandle();
		if (Ctx)
		{
			SyncCommandSubscriptions(Ctx);
			UrlabRcl_SpinSome(Ctx, RosSpinTimeoutNs);
		}
		FPlatformProcess::Sleep(RosIdleSleepSeconds);
	}
	// Destroy handles on the thread that created them, per the core contract.
	TeardownCommandSubscriptions();
}

void UURLabRosRpcTransport::SyncCommandSubscriptions(UrlabRclContext* Ctx)
{
	UURLabBridgeServer* Bridge = GetOwningBridge();
	AAMjManager* Mgr = Bridge ? Bridge->GetActiveManager() : nullptr;
	if (!Mgr)
	{
		if (bHaveSubscriptions)
		{
			TeardownCommandSubscriptions();
		}
		return;
	}

	const uint32 Version = Mgr->GetStateCollector().GetStructureVersion();
	if (bHaveSubscriptions && Mgr == SubscribedManager.Get()
		&& Version == SubscribedStructureVersion)
	{
		return;
	}
	RebuildCommandSubscriptions(Ctx);
}

void UURLabRosRpcTransport::RebuildCommandSubscriptions(UrlabRclContext* Ctx)
{
	TeardownCommandSubscriptions();
	if (!Ctx)
	{
		return;
	}

	UURLabBridgeServer* Bridge = GetOwningBridge();
	AAMjManager* Mgr = Bridge ? Bridge->GetActiveManager() : nullptr;
	if (!Mgr)
	{
		return;
	}

	for (AMjArticulation* Art : Mgr->GetAllArticulations())
	{
		if (!Art)
		{
			continue;
		}
		// The topic uses the canonical, ROS-legal segment (matching the publish
		// side); the raw actor name resolves the art and keys ownership.
		const FString Segment = FMjCanonicalName::ArtSegment(Art).ToString();

		FRosArtCommand* Cmd = new FRosArtCommand();
		Cmd->Transport = this;
		Cmd->ArtName = Art->GetName();

		const FString CtrlTopic = FString::Printf(TEXT("/%s/cmd_ctrl"), *Segment);
		Cmd->CtrlSub = UrlabRcl_CreateCtrlSub(Ctx, TCHAR_TO_UTF8(*CtrlTopic),
			&RosCtrlTrampoline, Cmd);
		if (Cmd->CtrlSub == nullptr)
		{
			UE_LOG(LogURLabRos, Warning, TEXT("ROS: cmd_ctrl subscription failed for %s (%hs)"),
				*CtrlTopic, UrlabRcl_LastError());
		}

		const FString VelTopic = FString::Printf(TEXT("/%s/cmd_vel"), *Segment);
		Cmd->TwistSub = UrlabRcl_CreateTwistSub(Ctx, TCHAR_TO_UTF8(*VelTopic),
			&RosTwistTrampoline, Cmd);
		if (Cmd->TwistSub == nullptr)
		{
			UE_LOG(LogURLabRos, Warning, TEXT("ROS: cmd_vel subscription failed for %s (%hs)"),
				*VelTopic, UrlabRcl_LastError());
		}

		if (Cmd->CtrlSub == nullptr && Cmd->TwistSub == nullptr)
		{
			delete Cmd;
			continue;
		}
		ArtCommands.Add(Cmd);
	}

	SubscribedManager = Mgr;
	SubscribedStructureVersion = Mgr->GetStateCollector().GetStructureVersion();
	bHaveSubscriptions = true;
}

void UURLabRosRpcTransport::TeardownCommandSubscriptions()
{
	for (FRosArtCommand* Cmd : ArtCommands)
	{
		if (!Cmd)
		{
			continue;
		}
		UrlabRcl_DestroyCtrlSub(Cmd->CtrlSub);
		UrlabRcl_DestroyTwistSub(Cmd->TwistSub);
		delete Cmd;
	}
	ArtCommands.Reset();
	SubscribedManager = nullptr;
	SubscribedStructureVersion = 0;
	bHaveSubscriptions = false;
}

void UURLabRosRpcTransport::HandleRosCtrl(const FString& ArtName, const double* Values, int32 Count)
{
	++RosCtrlCallbackCount;

	FURLabRpcDispatcher* Disp = ResolveDispatcher();
	if (!Disp)
	{
		return;
	}

	// Ownership gate first: a ROS write to an art this source does not own is
	// dropped. Ok also heartbeats the claim.
	const FName ArtKey(*ArtName);
	FString CurrentOwner;
	if (Disp->GetControlOwnership().CheckWrite(ArtKey, RosControlSourceId(), CurrentOwner)
		!= FMjControlOwnership::EWriteCheck::Ok)
	{
		return;
	}

	// ROS control is a Live-mode surface; direct / puppet bundle control into
	// their step / push calls, so drop the write outside Live.
	if (Disp->GetActiveStepMode() != EStepMode::Live)
	{
		return;
	}

	UURLabBridgeServer* Bridge = GetOwningBridge();
	AAMjManager* Mgr = Bridge ? Bridge->GetActiveManager() : nullptr;
	if (!Mgr)
	{
		return;
	}
	AMjArticulation* Art = Mgr->GetArticulation(ArtName);
	if (!Art)
	{
		return;
	}

	// Float64MultiArray values are in the art's actuator-list order; stage each on
	// its actuator's NetworkValue, the same path ApplyStepCtrl writes to.
	TArray<UMjActuator*> Acts = Art->GetActuators();
	const int32 N = FMath::Min<int32>(Count, Acts.Num());
	for (int32 i = 0; i < N; ++i)
	{
		if (Acts[i])
		{
			Acts[i]->SetNetworkControl(static_cast<float>(Values[i]));
		}
	}
}

void UURLabRosRpcTransport::HandleRosTwist(const FString& ArtName, const double Linear[3],
	const double Angular[3])
{
	++RosTwistCallbackCount;

	FURLabRpcDispatcher* Disp = ResolveDispatcher();
	if (!Disp)
	{
		return;
	}

	const FName ArtKey(*ArtName);
	FString CurrentOwner;
	if (Disp->GetControlOwnership().CheckWrite(ArtKey, RosControlSourceId(), CurrentOwner)
		!= FMjControlOwnership::EWriteCheck::Ok)
	{
		return;
	}

	if (Disp->GetActiveStepMode() != EStepMode::Live)
	{
		return;
	}

	UURLabBridgeServer* Bridge = GetOwningBridge();
	AAMjManager* Mgr = Bridge ? Bridge->GetActiveManager() : nullptr;
	if (!Mgr)
	{
		return;
	}
	AMjArticulation* Art = Mgr->GetArticulation(ArtName);
	if (!Art)
	{
		return;
	}
	UMjTwistController* TC = Art->FindComponentByClass<UMjTwistController>();
	if (!TC)
	{
		return;
	}

	// geometry_msgs/Twist maps as the set_twist RPC does: linear (vx, vy, _),
	// angular (_, _, yaw_rate).
	TC->SetTwist(static_cast<float>(Linear[0]), static_cast<float>(Linear[1]),
		static_cast<float>(Angular[2]));
}

void UURLabRosRpcTransport::ApplyRosCtrlForTest(const FString& ArtName,
	const TArray<double>& Values)
{
	HandleRosCtrl(ArtName, Values.GetData(), Values.Num());
}

bool UURLabRosRpcTransport::PublishAndPumpCtrlForTest(const FString& Topic,
	const TArray<double>& Values)
{
	UrlabRclContext* Ctx = FURLabRosContext::Get().GetHandle();
	if (!Ctx)
	{
		return false;
	}

	// Build the subscriptions on THIS thread; the executor thread must not be
	// running so no rcl handle is touched from two threads.
	RebuildCommandSubscriptions(Ctx);

	UrlabRclCtrlPub* Pub = UrlabRcl_CreateCtrlPub(Ctx, TCHAR_TO_UTF8(*Topic));
	if (Pub == nullptr)
	{
		TeardownCommandSubscriptions();
		return false;
	}

	const int64 Before = RosCtrlCallbackCount.load();
	bool bFired = false;
	// Re-publish each round so a message sent before intra-process discovery
	// completes is not the only one; bounded so a wire failure still returns.
	for (int32 Attempt = 0; Attempt < 200 && !bFired; ++Attempt)
	{
		UrlabRcl_PublishCtrl(Pub, Values.GetData(), Values.Num());
		UrlabRcl_SpinSome(Ctx, 20 * 1000 * 1000);  // 20 ms
		bFired = RosCtrlCallbackCount.load() > Before;
		if (!bFired)
		{
			FPlatformProcess::Sleep(0.01f);
		}
	}

	UrlabRcl_DestroyCtrlPub(Pub);
	TeardownCommandSubscriptions();
	return bFired;
}

#else  // URLAB_WITH_ROS2

// Absent-ROS stubs so the class links; every real caller is fenced off too.
bool UURLabRosRpcTransport::TransportInit() { return false; }
void UURLabRosRpcTransport::TransportShutdown() {}
void UURLabRosRpcTransport::RunExecutorLoop() {}
void UURLabRosRpcTransport::SyncCommandSubscriptions(UrlabRclContext*) {}
void UURLabRosRpcTransport::RebuildCommandSubscriptions(UrlabRclContext*) {}
void UURLabRosRpcTransport::TeardownCommandSubscriptions() {}
void UURLabRosRpcTransport::HandleRosCtrl(const FString&, const double*, int32) {}
void UURLabRosRpcTransport::HandleRosTwist(const FString&, const double[3], const double[3]) {}
void UURLabRosRpcTransport::ApplyRosCtrlForTest(const FString&, const TArray<double>&) {}
bool UURLabRosRpcTransport::PublishAndPumpCtrlForTest(const FString&, const TArray<double>&)
{
	return false;
}

#endif  // URLAB_WITH_ROS2
