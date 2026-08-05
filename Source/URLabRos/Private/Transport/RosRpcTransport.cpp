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
#include "MuJoCo/Elements/MjActuatorRuntime.h"
#include "MuJoCo/Input/MjTwistController.h"
#include "State/MjCanonicalName.h"
#include "State/MjStateTypes.h"
#include "Dom/JsonObject.h"
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
// write (Art->GetName(), the GetArticulation + ownership key) and the per-art
// subscription + service handles. The stable heap address is handed to the core as
// the callback / service User pointer.
struct FRosArtCommand
{
	UURLabRosRpcTransport* Transport = nullptr;
	FString ArtName;
	UrlabRclCtrlSub* CtrlSub = nullptr;
	UrlabRclTwistSub* TwistSub = nullptr;
	UrlabRclJointStateSub* JointCommandSub = nullptr;
	UrlabRclTriggerService* ClaimSrv = nullptr;
	UrlabRclTriggerService* ReleaseSrv = nullptr;
};

// Per declared user-input-channel binding. Carries the routing identity
// (canonical art segment or None for scene, the channel name, its kind) and its
// Float64MultiArray subscription handle.
struct FRosUserInputSub
{
	UURLabRosRpcTransport* Transport = nullptr;
	FName ArtOrNone;
	FName Channel;
	EMjUserChannelKind Kind = EMjUserChannelKind::Scalar;
	UrlabRclCtrlSub* Sub = nullptr;
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

void RosJointCommandTrampoline(const char** Names, const double* Positions,
	int32_t Count, void* User)
{
	FRosArtCommand* Cmd = static_cast<FRosArtCommand*>(User);
	if (Cmd && Cmd->Transport)
	{
		Cmd->Transport->HandleRosJointCommand(Cmd->ArtName, Names, Positions,
			static_cast<int32>(Count));
	}
}

void RosClaimTrampoline(void* User, int32_t* OutSuccess, char* OutMessage, int32_t Cap)
{
	FRosArtCommand* Cmd = static_cast<FRosArtCommand*>(User);
	if (Cmd && Cmd->Transport)
	{
		Cmd->Transport->HandleRosClaimRelease(Cmd->ArtName, /*bClaim=*/true,
			OutSuccess, OutMessage, Cap);
	}
}

void RosReleaseTrampoline(void* User, int32_t* OutSuccess, char* OutMessage, int32_t Cap)
{
	FRosArtCommand* Cmd = static_cast<FRosArtCommand*>(User);
	if (Cmd && Cmd->Transport)
	{
		Cmd->Transport->HandleRosClaimRelease(Cmd->ArtName, /*bClaim=*/false,
			OutSuccess, OutMessage, Cap);
	}
}

void RosUserInputTrampoline(const double* Values, int32_t Count, void* User)
{
	FRosUserInputSub* Sub = static_cast<FRosUserInputSub*>(User);
	if (Sub && Sub->Transport)
	{
		Sub->Transport->HandleRosUserChannel(Sub->ArtOrNone, Sub->Channel, Sub->Kind,
			Values, static_cast<int32>(Count));
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

		// JointState jog input: standard joint_state_publisher_gui publishes here.
		const FString JogTopic = FString::Printf(TEXT("/%s/joint_command"), *Segment);
		Cmd->JointCommandSub = UrlabRcl_CreateJointStateSub(Ctx, TCHAR_TO_UTF8(*JogTopic),
			&RosJointCommandTrampoline, Cmd);
		if (Cmd->JointCommandSub == nullptr)
		{
			UE_LOG(LogURLabRos, Warning, TEXT("ROS: joint_command subscription failed for %s (%hs)"),
				*JogTopic, UrlabRcl_LastError());
		}

		// claim_control / release_control as std_srvs/Trigger services; the art is
		// encoded in the service name.
		const FString ClaimName = FString::Printf(TEXT("/%s/claim_control"), *Segment);
		Cmd->ClaimSrv = UrlabRcl_CreateTriggerService(Ctx, TCHAR_TO_UTF8(*ClaimName),
			&RosClaimTrampoline, Cmd);
		if (Cmd->ClaimSrv == nullptr)
		{
			UE_LOG(LogURLabRos, Warning, TEXT("ROS: claim_control service failed for %s (%hs)"),
				*ClaimName, UrlabRcl_LastError());
		}
		const FString ReleaseName = FString::Printf(TEXT("/%s/release_control"), *Segment);
		Cmd->ReleaseSrv = UrlabRcl_CreateTriggerService(Ctx, TCHAR_TO_UTF8(*ReleaseName),
			&RosReleaseTrampoline, Cmd);
		if (Cmd->ReleaseSrv == nullptr)
		{
			UE_LOG(LogURLabRos, Warning, TEXT("ROS: release_control service failed for %s (%hs)"),
				*ReleaseName, UrlabRcl_LastError());
		}

		if (Cmd->CtrlSub == nullptr && Cmd->TwistSub == nullptr
			&& Cmd->JointCommandSub == nullptr && Cmd->ClaimSrv == nullptr
			&& Cmd->ReleaseSrv == nullptr)
		{
			delete Cmd;
			continue;
		}
		ArtCommands.Add(Cmd);
	}

	// One subscription per declared user-input channel. Numeric kinds ride the
	// Float64MultiArray (cmd_ctrl) sub shape; text-family channels take input over
	// the byte transports (set_user_channels), not over ROS.
	TArray<FMjUserInputChannelInfo> InputChannels;
	Mgr->GetUserInputChannels(InputChannels);
	for (const FMjUserInputChannelInfo& Info : InputChannels)
	{
		if (Info.Kind == EMjUserChannelKind::String || Info.Kind == EMjUserChannelKind::Struct)
		{
			continue;  // text-family channels have no ROS input subscription
		}
		const FString Topic = Info.ArtSegment.IsEmpty()
			? FString::Printf(TEXT("/urlab/user/%s"), *Info.Channel.ToString())
			: FString::Printf(TEXT("/%s/user/%s"), *Info.ArtSegment, *Info.Channel.ToString());

		FRosUserInputSub* Binding = new FRosUserInputSub();
		Binding->Transport = this;
		Binding->ArtOrNone = Info.ArtSegment.IsEmpty() ? NAME_None : FName(*Info.ArtSegment);
		Binding->Channel = Info.Channel;
		Binding->Kind = Info.Kind;
		Binding->Sub = UrlabRcl_CreateCtrlSub(Ctx, TCHAR_TO_UTF8(*Topic),
			&RosUserInputTrampoline, Binding);
		if (Binding->Sub == nullptr)
		{
			UE_LOG(LogURLabRos, Warning, TEXT("ROS: user input subscription failed for %s (%hs)"),
				*Topic, UrlabRcl_LastError());
			delete Binding;
			continue;
		}
		UserInputSubs.Add(Binding);
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
		UrlabRcl_DestroyJointStateSub(Cmd->JointCommandSub);
		UrlabRcl_DestroyTriggerService(Cmd->ClaimSrv);
		UrlabRcl_DestroyTriggerService(Cmd->ReleaseSrv);
		delete Cmd;
	}
	ArtCommands.Reset();

	for (FRosUserInputSub* Binding : UserInputSubs)
	{
		if (!Binding)
		{
			continue;
		}
		UrlabRcl_DestroyCtrlSub(Binding->Sub);
		delete Binding;
	}
	UserInputSubs.Reset();

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

void UURLabRosRpcTransport::HandleRosJointCommand(const FString& ArtName,
	const char** Names, const double* Positions, int32 Count)
{
	++RosJointCommandCallbackCount;

	FURLabRpcDispatcher* Disp = ResolveDispatcher();
	if (!Disp)
	{
		return;
	}

	// Same gating as cmd_ctrl: ownership first (also heartbeats the claim), then
	// Live mode only.
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
	if (!Art || !Names || !Positions)
	{
		return;
	}

	// Resolve a commanded name to an actuator two ways: by the joint a
	// joint-transmission actuator drives (the JointState / URDF joint name a jog
	// GUI echoes back), and by the actuator's own name. The latter reaches
	// actuators with no 1-DoF joint target -- e.g. a tendon-driven gripper
	// actuator -- so a controller can command the gripper as "<actuator>".
	TMap<FString, UMjActuator*> ByName;
	TArray<UMjActuator*> Acts = Art->GetActuators();
	ByName.Reserve(Acts.Num() * 2);
	for (UMjActuator* Act : Acts)
	{
		if (!Act)
		{
			continue;
		}
		if (Act->TransmissionType == EMjActuatorTrnType::Joint && !Act->TargetName.IsEmpty())
		{
			ByName.Add(FMjCanonicalName::PartSegment(Art, Act->TargetName).ToString(), Act);
		}
		ByName.Add(FMjCanonicalName::PartSegment(Art, Act->GetMjName()).ToString(), Act);
	}

	for (int32 i = 0; i < Count; ++i)
	{
		if (!Names[i])
		{
			continue;
		}
		const FString JointName = UTF8_TO_TCHAR(Names[i]);
		if (UMjActuator** Found = ByName.Find(JointName))
		{
			if (*Found)
			{
				(*Found)->SetNetworkControl(static_cast<float>(Positions[i]));
			}
		}
	}
}

void UURLabRosRpcTransport::HandleRosUserChannel(FName ArtOrNone, FName Channel,
	EMjUserChannelKind Kind, const double* Values, int32 Count)
{
	++RosUserChannelCallbackCount;

	UURLabBridgeServer* Bridge = GetOwningBridge();
	AAMjManager* Mgr = Bridge ? Bridge->GetActiveManager() : nullptr;
	if (!Mgr)
	{
		return;
	}

	// User-channel input is app-level data owned by user logic: no ownership gate
	// and no Live-mode gate (unlike control writes, which fight the physics
	// authority). The declaring component validates the value against its declared
	// kind.
	FMjUserChannel Value;
	Value.Name = Channel;
	Value.Kind = Kind;
	if (Values && Count > 0)
	{
		Value.Values.Append(Values, Count);
	}
	Mgr->ApplyUserChannelInput(ArtOrNone, Channel, Value);
}

void UURLabRosRpcTransport::HandleRosClaimRelease(const FString& ArtName, bool bClaim,
	int32* OutSuccess, char* OutMessage, int32 OutMessageCap)
{
	auto WriteMessage = [OutMessage, OutMessageCap](const FString& Msg) {
		if (OutMessage && OutMessageCap > 0)
		{
			FCStringAnsi::Strncpy(OutMessage, TCHAR_TO_UTF8(*Msg), OutMessageCap);
		}
	};
	if (OutSuccess)
	{
		*OutSuccess = 0;
	}

	FURLabRpcDispatcher* Disp = ResolveDispatcher();
	if (!Disp)
	{
		WriteMessage(TEXT("no active dispatcher"));
		return;
	}

	// Build the request the ZMQ/SHM path builds: source preset to the ROS node id,
	// session preset to the active session so Dispatch's session gate passes. TTL is
	// not settable over the Trigger service, so the default TTL applies.
	TSharedPtr<FJsonObject> Req = MakeShared<FJsonObject>();
	Req->SetStringField(TEXT("op"), bClaim ? TEXT("claim_control") : TEXT("release_control"));
	Req->SetStringField(TEXT("articulation"), ArtName);
	Req->SetStringField(TEXT("source"), RosControlSourceId());
	Req->SetStringField(TEXT("session_id"), Disp->GetActiveSessionId());

	const TSharedPtr<FJsonObject> Reply = Disp->Dispatch(Req);
	FString ReplyOp;
	if (Reply.IsValid())
	{
		Reply->TryGetStringField(TEXT("op"), ReplyOp);
	}

	const bool bOk = ReplyOp.Equals(bClaim ? TEXT("claim_control_ok") : TEXT("release_control_ok"));
	if (OutSuccess)
	{
		*OutSuccess = bOk ? 1 : 0;
	}
	if (bOk)
	{
		FString Owner;
		if (Reply.IsValid())
		{
			Reply->TryGetStringField(TEXT("owner"), Owner);
		}
		WriteMessage(bClaim
			? FString::Printf(TEXT("%s claimed by %s"), *ArtName,
				Owner.IsEmpty() ? *RosControlSourceId() : *Owner)
			: FString::Printf(TEXT("%s released"), *ArtName));
	}
	else
	{
		FString Code, Message;
		if (Reply.IsValid())
		{
			Reply->TryGetStringField(TEXT("code"), Code);
			Reply->TryGetStringField(TEXT("message"), Message);
		}
		WriteMessage(Message.IsEmpty() ? Code : Message);
	}
}

void UURLabRosRpcTransport::ApplyRosCtrlForTest(const FString& ArtName,
	const TArray<double>& Values)
{
	HandleRosCtrl(ArtName, Values.GetData(), Values.Num());
}

void UURLabRosRpcTransport::ApplyRosJointCommandForTest(const FString& ArtName,
	const TArray<FString>& Names, const TArray<double>& Positions)
{
	// Build the stable UTF-8 pointer array the wire callback would hand in.
	TArray<TArray<ANSICHAR>> NameBytes;
	NameBytes.Reserve(Names.Num());
	TArray<const char*> NamePtrs;
	NamePtrs.Reserve(Names.Num());
	for (const FString& Name : Names)
	{
		FTCHARToUTF8 Conv(*Name);
		TArray<ANSICHAR>& Bytes = NameBytes.AddDefaulted_GetRef();
		Bytes.Append(reinterpret_cast<const ANSICHAR*>(Conv.Get()), Conv.Length());
		Bytes.Add('\0');
		NamePtrs.Add(Bytes.GetData());
	}
	const int32 N = FMath::Min(Names.Num(), Positions.Num());
	HandleRosJointCommand(ArtName, N > 0 ? NamePtrs.GetData() : nullptr,
		N > 0 ? Positions.GetData() : nullptr, N);
}

bool UURLabRosRpcTransport::ApplyRosClaimReleaseForTest(const FString& ArtName, bool bClaim)
{
	int32 Success = 0;
	char Message[256] = {0};
	HandleRosClaimRelease(ArtName, bClaim, &Success, Message, sizeof(Message));
	return Success != 0;
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
void UURLabRosRpcTransport::HandleRosJointCommand(const FString&, const char**, const double*, int32) {}
void UURLabRosRpcTransport::HandleRosUserChannel(FName, FName, EMjUserChannelKind, const double*, int32) {}
void UURLabRosRpcTransport::HandleRosClaimRelease(const FString&, bool, int32*, char*, int32) {}
void UURLabRosRpcTransport::ApplyRosCtrlForTest(const FString&, const TArray<double>&) {}
void UURLabRosRpcTransport::ApplyRosJointCommandForTest(const FString&, const TArray<FString>&, const TArray<double>&) {}
bool UURLabRosRpcTransport::ApplyRosClaimReleaseForTest(const FString&, bool) { return false; }
bool UURLabRosRpcTransport::PublishAndPumpCtrlForTest(const FString&, const TArray<double>&)
{
	return false;
}

#endif  // URLAB_WITH_ROS2
