// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Bridge/BridgeServer.h"
#include "Bridge/RpcDispatcher.h"
#include "Transport/ZmqRpcTransport.h"
#include "Transport/ShmRpcTransport.h"
#include "Transport/RosRpcTransport.h"
#include "Transport/RosPublishTransport.h"
#include "Transport/PublishTransport.h"
#include "MuJoCo/Core/AMjManager.h"
#include "Utils/URLabLogging.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformMisc.h"
#include "Engine/Engine.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/Guid.h"
#include "HAL/PlatformTime.h"

namespace
{
/** Parse the trailing port from a "tcp://host:port" endpoint. Returns 0 when
 *  no numeric port is present. */
int32 ParseEndpointPort(const FString& Endpoint)
{
	int32 ColonIdx = INDEX_NONE;
	if (Endpoint.FindLastChar(TEXT(':'), ColonIdx))
	{
		const FString PortStr = Endpoint.Mid(ColonIdx + 1);
		if (!PortStr.IsEmpty() && PortStr.IsNumeric())
			return FCString::Atoi(*PortStr);
	}
	return 0;
}

/** Resolve the step-RPC endpoint, letting an operator override the port per
 *  editor instance without editing the project INI. Precedence: command-line
 *  `-URLabStepPort=N`, then the `URLAB_STEP_PORT` environment variable, then
 *  the requested endpoint unchanged. This is what lets many render-server
 *  editors run side by side on one host, each on its own port. */
FString ResolveStepEndpoint(const FString& Requested)
{
	int32 OverridePort = 0;
	if (!FParse::Value(FCommandLine::Get(), TEXT("URLabStepPort="), OverridePort))
	{
		const FString Env = FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_STEP_PORT"));
		if (!Env.IsEmpty() && Env.IsNumeric())
			OverridePort = FCString::Atoi(*Env);
	}
	if (OverridePort <= 0)
		return Requested;

	int32 ColonIdx = INDEX_NONE;
	if (Requested.FindLastChar(TEXT(':'), ColonIdx))
		return FString::Printf(TEXT("%s:%d"), *Requested.Left(ColonIdx), OverridePort);
	return FString::Printf(TEXT("tcp://0.0.0.0:%d"), OverridePort);
}
} // namespace

UURLabBridgeServer::UURLabBridgeServer() = default;

void UURLabBridgeServer::EnsureDispatcher()
{
	if (!Dispatcher.IsValid())
	{
		Dispatcher = MakeUnique<FURLabRpcDispatcher>();
		Dispatcher->SetOwningBridge(this);
	}
}

void UURLabBridgeServer::ApplyPerformanceOverrides()
{
	if (bPacingOverridden || !GEngine)
		return;

	auto Override = [](const TCHAR* Name, const TCHAR* Command, float& OutSaved, bool& OutHad)
	{
		if (IConsoleVariable* CVar = IConsoleManager::Get().FindConsoleVariable(Name))
		{
			OutHad = true;
			OutSaved = CVar->GetFloat();
		}
		GEngine->Exec(nullptr, Command);
	};

	Override(TEXT("r.VSync"), TEXT("r.VSync 0"), SavedVSync, bHadVSync);
#if WITH_EDITOR
	Override(TEXT("r.VSyncEditor"), TEXT("r.VSyncEditor 0"), SavedVSyncEditor, bHadVSyncEditor);
#endif
	Override(TEXT("t.MaxFPS"), TEXT("t.MaxFPS 240"), SavedMaxFPS, bHadMaxFPS);

	// Keep rendering when the editor is not the foreground window. Without this
	// Slate throttles the whole app to a few FPS in the background, which starves
	// camera capture/readback -- a headless RPC client then sees near-zero frame
	// throughput even though the bridge is serving.
	Override(TEXT("Slate.bAllowThrottling"), TEXT("Slate.bAllowThrottling 0"),
		SavedSlateThrottle, bHadSlateThrottle);
	Override(TEXT("t.IdleWhenNotForeground"), TEXT("t.IdleWhenNotForeground 0"),
		SavedIdleWhenNotForeground, bHadIdleWhenNotForeground);

	bPacingOverridden = true;
	UE_LOG(LogURLabNet, Log,
		TEXT("UURLabBridgeServer: disabled editor frame pacing + background throttling "
			 "while serving (VSync off, MaxFPS 240, Slate throttling off)"));
}

void UURLabBridgeServer::RestorePerformanceOverrides()
{
	if (!bPacingOverridden || !GEngine)
		return;

	auto Restore = [](const TCHAR* Name, float Value, bool bHad)
	{
		if (bHad)
			GEngine->Exec(nullptr, *FString::Printf(TEXT("%s %g"), Name, Value));
	};

	Restore(TEXT("r.VSync"), SavedVSync, bHadVSync);
#if WITH_EDITOR
	Restore(TEXT("r.VSyncEditor"), SavedVSyncEditor, bHadVSyncEditor);
#endif
	Restore(TEXT("t.MaxFPS"), SavedMaxFPS, bHadMaxFPS);
	Restore(TEXT("Slate.bAllowThrottling"), SavedSlateThrottle, bHadSlateThrottle);
	Restore(TEXT("t.IdleWhenNotForeground"), SavedIdleWhenNotForeground, bHadIdleWhenNotForeground);

	bPacingOverridden = false;
	UE_LOG(LogURLabNet, Log, TEXT("UURLabBridgeServer: restored editor frame pacing"));
}

void UURLabBridgeServer::BeginDestroy()
{
	Stop();
	Super::BeginDestroy();
}

void UURLabBridgeServer::Start(const FString& StepEndpoint)
{
	EnsureDispatcher();

	// Empty endpoint: dispatcher only, no transports (test path).
	if (StepEndpoint.IsEmpty())
		return;

	EnsureZmqBound(ResolveStepEndpoint(StepEndpoint));
}

bool UURLabBridgeServer::EnsureZmqBound(const FString& Endpoint)
{
	if (Endpoint.IsEmpty())
		return false;

	EnsureDispatcher();

	for (const TObjectPtr<UURLabRpcTransport>& T : RpcTransports)
	{
		UURLabZmqRpcTransport* Existing = Cast<UURLabZmqRpcTransport>(T);
		if (Existing && Existing->StepEndpoint == Endpoint)
			return true;
	}

	// NAME_None: let UE pick a fresh unique name. A fixed name collides on
	// rebind while the previous worker is still tearing down.
	UURLabZmqRpcTransport* Zmq = NewObject<UURLabZmqRpcTransport>(this, NAME_None);
	Zmq->StepEndpoint = Endpoint;
	Zmq->SetOwningBridge(this);
	if (!Zmq->TransportInit())
	{
		UE_LOG(LogURLabNet, Error,
			TEXT("UURLabBridgeServer: ZMQ bind failed on %s"), *Endpoint);
		return false;
	}
	RpcTransports.Add(Zmq);
	ApplyPerformanceOverrides();
	UE_LOG(LogURLabNet, Log,
		TEXT("UURLabBridgeServer: ZMQ REP bound at %s"), *Endpoint);
	return true;
}

bool UURLabBridgeServer::EnsureShmBound(const FString& SessionId)
{
	EnsureDispatcher();

	const FString Sid = SessionId.IsEmpty() ? FString(TEXT("live")) : SessionId;

	for (const TObjectPtr<UURLabRpcTransport>& T : RpcTransports)
	{
		UURLabShmRpcTransport* Existing = Cast<UURLabShmRpcTransport>(T);
		if (Existing)
		{
			const FString ExistingSid = Existing->SessionId.IsEmpty()
										  ? FString(TEXT("live"))
										  : Existing->SessionId;
			if (ExistingSid == Sid)
				return true;
		}
	}

	// Fold the instance's step port into the SHM session name so the per-process
	// unique id stays traceable back to the instance (the process id already
	// guarantees uniqueness). Read it from the bound ZMQ transport rather than
	// re-resolving, so both wire ends agree.
	int32 StepPort = 0;
	for (const TObjectPtr<UURLabRpcTransport>& T : RpcTransports)
	{
		if (UURLabZmqRpcTransport* Zmq = Cast<UURLabZmqRpcTransport>(T))
		{
			StepPort = ParseEndpointPort(Zmq->StepEndpoint);
			break;
		}
	}

	UURLabShmRpcTransport* Shm = NewObject<UURLabShmRpcTransport>(this, NAME_None);
	Shm->SessionId = SessionId; // empty -> defaults to "live" inside Init
	Shm->InstancePort = StepPort;
	Shm->SetOwningBridge(this);
	if (!Shm->TransportInit())
	{
		UE_LOG(LogURLabNet, Error,
			TEXT("UURLabBridgeServer: SHM open failed (session=%s)"), *Sid);
		return false;
	}
	RpcTransports.Add(Shm);
	ApplyPerformanceOverrides();
	UE_LOG(LogURLabNet, Log,
		TEXT("UURLabBridgeServer: SHM RPC bound (session=%s)"), *Sid);
	return true;
}

bool UURLabBridgeServer::EnsureRosBound()
{
	EnsureDispatcher();

	// RPC / control leg: one ROS executor transport, persisting across PIE like
	// the other RPC transports. Its TransportInit brings up the process-wide ROS
	// context; a false return means ROS is unavailable (feature off or no live
	// context), so there is nothing to bind.
	bool bHaveRpc = false;
	for (const TObjectPtr<UURLabRpcTransport>& T : RpcTransports)
	{
		if (Cast<UURLabRosRpcTransport>(T))
		{
			bHaveRpc = true;
			break;
		}
	}
	if (!bHaveRpc)
	{
		UURLabRosRpcTransport* Ros = NewObject<UURLabRosRpcTransport>(this, NAME_None);
		Ros->SetOwningBridge(this);
		if (!Ros->TransportInit())
		{
			UE_LOG(LogURLabNet, Log,
				TEXT("UURLabBridgeServer: ROS unavailable; EnsureRosBound is a no-op."));
			return false;
		}
		RpcTransports.Add(Ros);
		ApplyPerformanceOverrides();
		UE_LOG(LogURLabNet, Log, TEXT("UURLabBridgeServer: ROS RPC transport bound"));
	}

	// Publish / fan-out leg: registered with the live manager, which owns per-PIE
	// publish transports and tears them down in EndPlay. When no manager is live
	// yet the publish leg is deferred to the next EnsureRosBound with one present.
	if (AAMjManager* Manager = GetActiveManager())
	{
		bool bHavePub = false;
		for (const TObjectPtr<UURLabPublishTransport>& T : Manager->ManagerOwnedPublishTransports)
		{
			if (Cast<UURLabRosPublishTransport>(T))
			{
				bHavePub = true;
				break;
			}
		}
		if (!bHavePub)
		{
			UURLabRosPublishTransport* Pub = NewObject<UURLabRosPublishTransport>(Manager, NAME_None);
			if (Pub->TransportInit())
			{
				Manager->ManagerOwnedPublishTransports.Add(Pub);
				UE_LOG(LogURLabNet, Log,
					TEXT("UURLabBridgeServer: ROS publish transport registered with manager"));
			}
		}
	}
	else
	{
		UE_LOG(LogURLabNet, Warning,
			TEXT("UURLabBridgeServer: EnsureRosBound with no active manager; "
				 "publish leg deferred until a manager is live."));
	}

	return true;
}

void UURLabBridgeServer::Stop()
{
	// Drain before tearing transports down so blocking handlers see the
	// flag on their next 50ms tick and return `shutting_down` instead of
	// pinning the worker thread.
	if (Dispatcher.IsValid())
	{
		Dispatcher->SetDraining(true);
	}

	for (const TObjectPtr<UURLabRpcTransport>& T : RpcTransports)
	{
		if (T)
			T->TransportShutdown();
	}
	RpcTransports.Reset();
	RestorePerformanceOverrides();

	if (!Dispatcher.IsValid())
		return;
	Dispatcher->Shutdown();
	Dispatcher.Reset();
	ActiveManager.Reset();
}

void UURLabBridgeServer::RegisterManager(AAMjManager* InManager)
{
	ActiveManager = InManager;
	if (Dispatcher.IsValid() && InManager)
	{
		Dispatcher->Init(InManager);
	}
}

void UURLabBridgeServer::UnregisterManager(AAMjManager* InManager)
{
	// Idempotent against mismatched cleanup orderings during PIE teardown.
	if (ActiveManager.Get() != InManager)
		return;
	if (Dispatcher.IsValid())
	{
		// Per-cycle teardown only — session, encoding, and observation
		// level are bridge-level and survive PIE end / level changes.
		// BridgeServer::Stop is what fully drops them.
		Dispatcher->OnManagerGone();
	}
	ActiveManager.Reset();
}

double UURLabBridgeServer::LeaseNow() const
{
	return LeaseClockOverrideForTest >= 0.0 ? LeaseClockOverrideForTest : FPlatformTime::Seconds();
}

bool UURLabBridgeServer::IsLeaseHeldInternal(double NowSeconds)
{
	if (bLeaseHeld && (NowSeconds - LeaseLastActivitySeconds) > LeaseTtlSeconds)
	{
		// Idle past its TTL: auto-release so the next acquire succeeds.
		bLeaseHeld = false;
		LeaseId.Empty();
		LeaseOwner.Empty();
	}
	return bLeaseHeld;
}

bool UURLabBridgeServer::TryAcquireLease(const FString& Owner, double TtlSeconds,
	FString& OutLeaseId, FString& OutExistingLeaseId)
{
	FScopeLock Lock(&LeaseMutex);
	const double Now = LeaseNow();
	if (IsLeaseHeldInternal(Now))
	{
		OutExistingLeaseId = LeaseId;
		return false;
	}

	bLeaseHeld = true;
	LeaseId = FGuid::NewGuid().ToString(EGuidFormats::DigitsWithHyphens);
	LeaseOwner = Owner;
	LeaseTtlSeconds = TtlSeconds;
	LeaseLastActivitySeconds = Now;
	OutLeaseId = LeaseId;
	return true;
}

bool UURLabBridgeServer::ReleaseLease(const FString& InLeaseId)
{
	FScopeLock Lock(&LeaseMutex);
	if (!bLeaseHeld || !LeaseId.Equals(InLeaseId))
		return false;

	bLeaseHeld = false;
	LeaseId.Empty();
	LeaseOwner.Empty();
	return true;
}

void UURLabBridgeServer::TouchLease()
{
	FScopeLock Lock(&LeaseMutex);
	if (bLeaseHeld)
		LeaseLastActivitySeconds = LeaseNow();
}

bool UURLabBridgeServer::IsLeaseHeld()
{
	FScopeLock Lock(&LeaseMutex);
	return IsLeaseHeldInternal(LeaseNow());
}

FString UURLabBridgeServer::GetLeaseId() const
{
	FScopeLock Lock(&LeaseMutex);
	return LeaseId;
}

void UURLabBridgeServer::SetLeaseClockForTest(double NowSeconds)
{
	FScopeLock Lock(&LeaseMutex);
	LeaseClockOverrideForTest = NowSeconds;
}
