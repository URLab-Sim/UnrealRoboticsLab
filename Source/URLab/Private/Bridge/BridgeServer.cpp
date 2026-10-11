// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "Bridge/BridgeServer.h"
#include "Bridge/RpcDispatcher.h"
#include "Bridge/InstanceRegistry.h"
#include "Bridge/BridgeServerConfigUtils.h"
#include "Transport/ZmqRpcTransport.h"
#include "Transport/RpcTransport.h"
#include "Transport/MjExternalTransportProvider.h"
#include "Utils/URLabLogging.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformMisc.h"
#include "HAL/PlatformProcess.h"
#include "Engine/Engine.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/Guid.h"
#include "HAL/PlatformTime.h"

namespace
{
/** The dialable dm_env_rpc endpoint this instance should advertise, or empty
 *  when it has none. Presence is decided by a BOUND transport, never by the
 *  configured port: DmEnvPort defaults to the well-known 50051, so keying the
 *  advertisement off the config alone published a gRPC endpoint from every
 *  broadcasting build, including builds with the URLabDmEnvRpc module compiled
 *  out. A transport is only in this list after its TransportInit reported a
 *  successful bind, which is the closest thing to "something is listening" the
 *  core can observe without depending on the optional module's types. The port
 *  comes from the config, which is also where the transport reads its bind port,
 *  so the advertised port is the bound one. Carries a scheme: a scheme-less
 *  host:port is routed to ZMQ by the client transport factory. */
FString ResolveAdvertisedGrpcEndpoint(
	const TArray<TObjectPtr<UURLabRpcTransport>>& Transports, int32 DmEnvPort)
{
	if (DmEnvPort <= 0)
		return FString();

	for (const TObjectPtr<UURLabRpcTransport>& T : Transports)
	{
		if (T && T->GetTransportName() == TEXT("dm_env_rpc"))
		{
			return FString::Printf(TEXT("grpc://%s:%d"),
				FPlatformProcess::ComputerName(), DmEnvPort);
		}
	}
	return FString();
}

} // namespace

UURLabBridgeServer::UURLabBridgeServer() = default;

void UURLabBridgeServer::SetInstanceConfig(const FURLabBridgeServerConfig& InConfig)
{
	InstanceConfig = InConfig;
	bInstanceConfigResolved = true;
	RefreshCommandEndpointSnapshot();
}

const FURLabBridgeServerConfig& UURLabBridgeServer::ResolveInstanceConfig()
{
	if (!bInstanceConfigResolved)
	{
		URLabBridgeServerConfigUtils::LoadFromIni(InstanceConfig);
		URLabBridgeServerConfigUtils::ApplyEnvAndCommandLineOverrides(InstanceConfig);
		bInstanceConfigResolved = true;
	}
	return InstanceConfig;
}

const FURLabShmSession& UURLabBridgeServer::GetShmSession()
{
	if (!ShmSession.IsValid())
	{
		const FURLabBridgeServerConfig& Cfg = ResolveInstanceConfig();
		// The step port comes from the resolved config, which is also the port
		// the REP socket binds -- so the session name is traceable to the
		// instance the registry and the handshake advertise, by construction
		// rather than by two derivations agreeing.
		ShmSession = FURLabShmSession::Resolve(Cfg.InstanceId, Cfg.StepPort);
	}
	return ShmSession;
}

void UURLabBridgeServer::CollectCommandEndpoints(TArray<FString>& Out) const
{
	FScopeLock Lock(&AdvertisementMutex);
	Out = CommandEndpointSnapshot;
}

FString UURLabBridgeServer::NormalizeBoundCommandEndpoint(
	const FString& Bound, const FString& DialHost)
{
	if (!Bound.StartsWith(TEXT("tcp://")))
		return FString();
	for (const FString& Prefix : {FString(TEXT("tcp://0.0.0.0:")),
		FString(TEXT("tcp://[::]:")), FString(TEXT("tcp://*:"))})
	{
		if (Bound.StartsWith(Prefix))
		{
			return FString::Printf(TEXT("tcp://%s:%s"), *DialHost,
				*Bound.RightChop(Prefix.Len()));
		}
	}
	return Bound;
}

void UURLabBridgeServer::RefreshCommandEndpointSnapshot()
{
	TArray<FString> Updated;
	const FString DialHost = FPlatformProcess::ComputerName();
	for (const TObjectPtr<UURLabRpcTransport>& Transport : RpcTransports)
	{
		if (const UURLabZmqRpcTransport* Zmq = Cast<UURLabZmqRpcTransport>(Transport))
		{
			const FString Endpoint = NormalizeBoundCommandEndpoint(
				Zmq->GetBoundEndpoint(), DialHost);
			if (Endpoint.IsEmpty())
				continue;
			Updated.AddUnique(Endpoint);
		}
	}
	const FString Grpc = ResolveAdvertisedGrpcEndpoint(RpcTransports, InstanceConfig.DmEnvPort);
	if (!Grpc.IsEmpty())
	{
		Updated.AddUnique(Grpc);
	}
	FScopeLock Lock(&AdvertisementMutex);
	CommandEndpointSnapshot = MoveTemp(Updated);
}

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

	auto Override = [](const TCHAR* Name, const TCHAR* Command, float& OutSaved, bool& OutHad) {
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

	auto Restore = [](const TCHAR* Name, float Value, bool bHad) {
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

	// Pin the config here even on the no-transport test path, so every later
	// reader (manager, cameras, SHM session) sees the same resolved copy rather
	// than loading its own at a different moment.
	ResolveInstanceConfig();

	// Empty endpoint: dispatcher only, no transports (test path). Do not
	// register a discoverable entry for a dispatcher with no wire.
	if (StepEndpoint.IsEmpty())
		return;

	EnsureZmqBound(StepEndpoint);

	// Register this instance for broker-less discovery on the bridge-server lifecycle.
	// This runs for the editor subsystem's server AND the cooked/packaged server, so a
	// packaged owner/render server is discoverable by the browser/pool/session too. The
	// bridge is the single writer. `discoverable` is a service and is enforced exactly
	// here: an instance that declines it stands up its wire and writes no entry, so
	// nothing can find it -- there is no per-request gate because there is no request.
	if (!InstanceConfig.bDiscoverable)
	{
		UE_LOG(LogURLabNet, Log,
			TEXT("UURLabBridgeServer: %s declined; this instance serves but is not listed"),
			MjCapabilityName(EMjCapability::Discoverable));
		return;
	}
	WriteRegistryEntry();

	// Refresh on a ticker so the entry's mtime stays fresh (discovery treats a
	// too-old entry as dead) and its busy/manager fields track the live state.
	if (!RegistryHeartbeatHandle.IsValid())
	{
		RegistryHeartbeatHandle = FTSTicker::GetCoreTicker().AddTicker(
			FTickerDelegate::CreateUObject(this, &UURLabBridgeServer::RefreshRegistryHeartbeat),
			/*DelaySeconds=*/10.0f);
	}
}

void UURLabBridgeServer::WriteRegistryEntry()
{
	if (CachedUrlabVersion.IsEmpty() && Dispatcher.IsValid())
		CachedUrlabVersion = Dispatcher->URLabVersion;

	FURLabRegistryLiveState Live;
	Live.UrlabVersion = CachedUrlabVersion;
	Live.bManagerPresent = false;
	Live.bBusy = IsLeaseHeld();

	// Runtime grants, from the same set the session hello advertises them from and the
	// same one the dispatcher enforces against. They used to be read off the manager,
	// which is why an instance with no manager -- every render server -- advertised no
	// grants at all and could not satisfy a pool filter.
	CollectGrantTokens(Live.GrantTokens);

	Live.GrpcEndpoint = ResolveAdvertisedGrpcEndpoint(RpcTransports, InstanceConfig.DmEnvPort);

	FURLabInstanceRegistry::WriteEntry(InstanceConfig, Live);
}

bool UURLabBridgeServer::RefreshRegistryHeartbeat(float /*DeltaTime*/)
{
	if (!IsRunning())
		return false; // dispatcher gone: stop ticking

	// The heartbeat is the same write: rewriting refreshes the mtime every
	// reader ages against AND re-reads the live state, so a model load or a
	// lease change shows up in discovery within one heartbeat instead of
	// carrying the values captured at server start.
	WriteRegistryEntry();
	return true; // keep ticking
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
	RefreshCommandEndpointSnapshot();
	ApplyPerformanceOverrides();
	UE_LOG(LogURLabNet, Log,
		TEXT("UURLabBridgeServer: ZMQ REP bound at %s"), *Endpoint);
	return true;
}

bool UURLabBridgeServer::EnsureExternalTransportsBound()
{
	// The concrete transports live in a separate, optional module that installs
	// factory hooks at startup. Absent that module the hooks are unbound, so there
	// is nothing to bind and the core names no external transport type.
	if (!FMjExternalTransportProvider::HasControlRpcTransport())
	{
		UE_LOG(LogURLabNet, Log,
			TEXT("UURLabBridgeServer: external control transport unavailable; "
				 "EnsureExternalTransportsBound is a no-op."));
		return false;
	}

	EnsureDispatcher();

	// RPC / control leg: iterate every registered external control-RPC factory and
	// bind each one that is not already present. Each external transport persists
	// across PIE like the other RPC transports; its TransportInit brings up the
	// process-wide context and reports the actual runtime/bind result -- a false
	// return means that backend is unavailable this run, so it is skipped without
	// blocking the others. Dedup is by the transport's own GetTransportName(), never
	// a literal, so re-entering PIE re-matches every already-bound backend
	// (including "dm_env_rpc") and never accumulates a dead duplicate. Because
	// this is a list and not a single slot, ROS ("ros2-rpc") and gRPC ("dm_env_rpc")
	// bind side by side instead of the later one evicting the earlier.
	bool bAnyRpc = false;
	for (const FMjExternalRpcTransportFactory& Reg : FMjExternalTransportProvider::ControlRpcTransportFactories)
	{
		if (!Reg.Factory.IsBound())
		{
			continue;
		}

		bool bHaveRpc = false;
		for (const TObjectPtr<UURLabRpcTransport>& T : RpcTransports)
		{
			if (T && T->GetTransportName() == Reg.TransportName.ToString())
			{
				bHaveRpc = true;
				break;
			}
		}
		if (bHaveRpc)
		{
			bAnyRpc = true;
			continue;
		}

		UURLabRpcTransport* External = Reg.Factory.Execute(this);
		if (!External || !External->TransportInit())
		{
			UE_LOG(LogURLabNet, Log,
				TEXT("UURLabBridgeServer: external control runtime '%s' unavailable; skipped."),
				*Reg.TransportName.ToString());
			continue;
		}
		RpcTransports.Add(External);
		RefreshCommandEndpointSnapshot();
		ApplyPerformanceOverrides();
		bAnyRpc = true;
		UE_LOG(LogURLabNet, Log,
			TEXT("UURLabBridgeServer: control RPC transport '%s' bound"),
			*Reg.TransportName.ToString());
	}

	return bAnyRpc;
}

void UURLabBridgeServer::Stop()
{
	// Stop the discovery heartbeat and drop this instance's registry entry
	// first, so a shutting-down instance stops advertising immediately.
	if (RegistryHeartbeatHandle.IsValid())
	{
		FTSTicker::GetCoreTicker().RemoveTicker(RegistryHeartbeatHandle);
		RegistryHeartbeatHandle.Reset();
	}
	FURLabInstanceRegistry::RemoveEntry(InstanceConfig);

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
	{
		FScopeLock Lock(&AdvertisementMutex);
		CommandEndpointSnapshot.Reset();
	}
	RestorePerformanceOverrides();

	if (!Dispatcher.IsValid())
		return;
	Dispatcher->Shutdown();
	Dispatcher.Reset();
}

void UURLabBridgeServer::SetGrants(const FMjCapabilitySet& InGrants)
{
	FMjCapabilitySet CompletePolicy;
	for (int32 i = 0; i < URLabWire::Capability::Count; ++i)
	{
		const EMjCapability Cap = static_cast<EMjCapability>(i);
		if (MjCapabilityKind(Cap) == URLabWire::Capability::EKind::Permission)
		{
			CompletePolicy.Set(Cap, InGrants.Has(Cap));
		}
	}
	FScopeLock Lock(&AdvertisementMutex);
	Grants = CompletePolicy;
}

void UURLabBridgeServer::CollectGrantTokens(TArray<FString>& Out) const
{
	FScopeLock Lock(&AdvertisementMutex);
	Out.Reset();
	for (int32 Index = 0; Index < URLabWire::Capability::Count; ++Index)
	{
		const EMjCapability Capability = static_cast<EMjCapability>(Index);
		if (MjCapabilityKind(Capability) == URLabWire::Capability::EKind::Permission
			&& Grants.Has(Capability))
		{
			// Grants are authority capabilities, so their canonical protocol token is
			// the capability name. Do not expose the generated legacy Wire spelling.
			Out.Add(MjCapabilityName(Capability));
		}
	}
}

bool UURLabBridgeServer::GrantsPermission(EMjCapability Cap) const
{
	FScopeLock Lock(&AdvertisementMutex);
	return Grants.Has(Cap);
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
