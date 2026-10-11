// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

// ============================================================================
// MjTransportConsolidationTests.cpp
//
// The transport layer had six abstractions with no common parent, eight ZMQ
// contexts (one per camera), five copies of the same worker-thread shim, three
// disagreeing derivations of the shared-memory session directory, and default
// ports written down in nine places. These specs pin the single versions.
//
// They are contract tests: no live sockets except the one bind the precedence
// spec needs, which skips itself when the port is taken.
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"
#include "HAL/PlatformMisc.h"
#include "HAL/PlatformProcess.h"
#include "UObject/Package.h"
#include <atomic>

#include "Bridge/BridgeServer.h"
#include "Bridge/BridgeServerConfig.h"
#include "MjClientTransportDoubles.h"
#include "Transport/CameraPublishTransport.h"
#include "Transport/ClientSubscribeTransport.h"
#include "Transport/MjExternalTransportProvider.h"
#include "Transport/PublishTransport.h"
#include "Transport/RpcClientTransport.h"
#include "Transport/RpcTransport.h"
#include "Transport/ShmSession.h"
#include "Transport/SubscribeTransport.h"
#include "Transport/TransportBase.h"
#include "Transport/WorkerThread.h"
#include "Transport/ZmqContext.h"
#include "Transport/ZmqRpcTransport.h"

// ---------------------------------------------------------------------------
// URLab.Transport.OneBaseUnderEveryRole
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjTransportOneBase,
	"URLab.Transport.OneBaseUnderEveryRole",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjTransportOneBase::RunTest(const FString& Parameters)
{
	// Every role derives from the shared base. Without this a seventh role could
	// be added beside the others and quietly repeat the lifecycle again.
	const TArray<UClass*> Roles = {
		UURLabPublishTransport::StaticClass(),
		UURLabSubscribeTransport::StaticClass(),
		UURLabClientSubscribeTransport::StaticClass(),
		UURLabRpcTransport::StaticClass(),
		UURLabRpcClientTransport::StaticClass(),
		UURLabCameraPublishTransport::StaticClass(),
	};
	for (UClass* Role : Roles)
	{
		TestTrue(FString::Printf(TEXT("%s derives from UURLabTransportBase"), *Role->GetName()),
			Role->IsChildOf(UURLabTransportBase::StaticClass()));
	}
	return true;
}

// ---------------------------------------------------------------------------
// URLab.Transport.OneZmqContext
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjTransportOneZmqContext,
	"URLab.Transport.OneZmqContext",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjTransportOneZmqContext::RunTest(const FString& Parameters)
{
	// A live editor may already hold references (the bridge's REP socket), so
	// measure against the count on entry rather than against zero.
	const int32 Baseline = FURLabZmqContext::GetRefCount();

	void* A = FURLabZmqContext::Acquire();
	void* B = FURLabZmqContext::Acquire();
	void* C = FURLabZmqContext::Acquire();
	if (!TestNotNull(TEXT("context created"), A))
	{
		return false;
	}
	// Three holders, one context: this is the whole point. A context per camera
	// was three thread pools serving three sockets.
	TestEqual(TEXT("second acquire returns the same context"), B, A);
	TestEqual(TEXT("third acquire returns the same context"), C, A);
	TestEqual(TEXT("refcount tracks holders"), FURLabZmqContext::GetRefCount(), Baseline + 3);

	FURLabZmqContext::Release(A);
	TestNull(TEXT("release clears the caller's handle"), A);
	TestEqual(TEXT("refcount drops on release"), FURLabZmqContext::GetRefCount(), Baseline + 2);

	// Releasing an already-cleared handle must not decrement again, or the last
	// real holder would find its context terminated underneath it.
	FURLabZmqContext::Release(A);
	TestEqual(TEXT("releasing a null handle is a no-op"),
		FURLabZmqContext::GetRefCount(), Baseline + 2);

	FURLabZmqContext::Release(B);
	FURLabZmqContext::Release(C);
	TestEqual(TEXT("refcount returns to baseline"),
		FURLabZmqContext::GetRefCount(), Baseline);
	return true;
}

// ---------------------------------------------------------------------------
// URLab.Transport.OneWorkerThreadShim
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjTransportWorkerThread,
	"URLab.Transport.OneWorkerThreadShim",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjTransportWorkerThread::RunTest(const FString& Parameters)
{
	std::atomic<bool> bStop{false};
	std::atomic<int32> Ran{0};
	std::atomic<int32> StopCalls{0};

	{
		FURLabWorkerThread Worker;
		TestFalse(TEXT("not running before Start"), Worker.IsRunning());

		const bool bStarted = Worker.Start(TEXT("URLab.Test.Worker"),
			[&bStop, &Ran]()
			{
				++Ran;
				while (!bStop.load(std::memory_order_acquire))
				{
					FPlatformProcess::SleepNoStats(0.001f);
				}
			},
			[&bStop, &StopCalls]()
			{
				++StopCalls;
				bStop.store(true, std::memory_order_release);
			});
		if (!TestTrue(TEXT("worker started"), bStarted))
		{
			return false;
		}
		TestTrue(TEXT("running after Start"), Worker.IsRunning());

		// Stop must signal AND join: a transport closes its sockets immediately
		// after, and a still-running body would read freed handles.
		Worker.Stop();
		TestFalse(TEXT("not running after Stop"), Worker.IsRunning());
		TestEqual(TEXT("body ran once"), Ran.load(), 1);
		TestTrue(TEXT("stop callback fired"), StopCalls.load() >= 1);

		// Idempotent, because TransportShutdown is and it delegates here.
		Worker.Stop();
		TestFalse(TEXT("still not running after a second Stop"), Worker.IsRunning());
	}
	return true;
}

// ---------------------------------------------------------------------------
// URLab.Transport.OneShmSessionDirectory
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjTransportShmSession,
	"URLab.Transport.OneShmSessionDirectory",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjTransportShmSession::RunTest(const FString& Parameters)
{
	const FURLabShmSession Session = FURLabShmSession::Resolve(TEXT("farm"), 5570);
	TestTrue(TEXT("resolved session is valid"), Session.IsValid());

	// The identity carries the label, the instance's step port and the process
	// id, so two editors on one host never open each other's segments.
	const FString Pid = FString::Printf(TEXT("%u"), FPlatformProcess::GetCurrentProcessId());
	TestTrue(TEXT("session id carries the label"), Session.SessionId.StartsWith(TEXT("farm_")));
	TestTrue(TEXT("session id carries the step port"), Session.SessionId.Contains(TEXT("_p5570_")));
	TestTrue(TEXT("session id carries the process id"), Session.SessionId.EndsWith(Pid));

	// Latest state and camera segments share the bridge-owned directory.
	const FString StatePath = Session.FilePath(TEXT("state.shm"));
	const FString CamPath =
		Session.FilePath(FURLabShmSession::MakeCameraSegmentFileName(TEXT("arm0/wrist")));
	TestEqual(TEXT("state segment is in the session dir"), FPaths::GetPath(StatePath), Session.Dir);
	TestEqual(TEXT("camera segment is in the session dir"), FPaths::GetPath(CamPath), Session.Dir);

	// The directory is absolute, because it is what the handshake advertises to
	// a client in another process.
	TestTrue(TEXT("session dir is absolute"), FPaths::IsRelative(Session.Dir) == false);
	TestEqual(TEXT("id maps back to the same dir"),
		FURLabShmSession::DirForSessionId(Session.SessionId), Session.Dir);

	// A canonical camera name is "<art>/<part>"; the slash cannot survive into a
	// file name, and a client that kept it could never open the ring.
	const FString CamFile = FURLabShmSession::MakeCameraSegmentFileName(TEXT("arm0/wrist"));
	TestEqual(TEXT("camera segment name"), CamFile, FString(TEXT("cam_arm0_wrist.shm")));
	TestFalse(TEXT("camera segment name is a file name, not a path"),
		CamFile.Contains(TEXT("/")));

	// No label, no port: still a per-process directory, never a shared "live".
	const FURLabShmSession Bare = FURLabShmSession::Resolve(FString(), 0);
	TestEqual(TEXT("empty label becomes live"), Bare.SessionId,
		FString::Printf(TEXT("live_%s"), *Pid));
	TestNotEqual(TEXT("a different instance gets a different dir"), Bare.Dir, Session.Dir);
	return true;
}

// ---------------------------------------------------------------------------
// URLab.Transport.BridgeOwnsTheSessionAndTheConfig
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjTransportBridgeOwnsSession,
	"URLab.Transport.BridgeOwnsTheSessionAndTheConfig",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjTransportBridgeOwnsSession::RunTest(const FString& Parameters)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>(GetTransientPackage());

	FURLabBridgeServerConfig Cfg;
	Cfg.InstanceId = TEXT("spec");
	Cfg.StepPort = 5999;
	Server->SetInstanceConfig(Cfg);

	// An owner-supplied config is final: ResolveInstanceConfig must not go back
	// to the INI and command line and produce a second, different answer.
	TestEqual(TEXT("supplied config is the resolved config"),
		Server->ResolveInstanceConfig().StepPort, 5999);

	const FURLabShmSession& First = Server->GetShmSession();
	// The session is derived from the config the bridge holds, so the directory
	// in the handshake is the directory the transports were handed.
	TestTrue(TEXT("session id carries the instance label"),
		First.SessionId.StartsWith(TEXT("spec_")));
	TestTrue(TEXT("session id carries the configured step port"),
		First.SessionId.Contains(TEXT("_p5999_")));

	// Resolved once: a second call must not re-derive, or the three transports
	// could again be handed three directories.
	const FString FirstDir = First.Dir;
	TestEqual(TEXT("session is resolved once"), Server->GetShmSession().Dir, FirstDir);

	Server->Stop();
	return true;
}

// ---------------------------------------------------------------------------
// URLab.Transport.NoPerClassDefaultPorts
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjTransportNoDefaultPortLiterals,
	"URLab.Transport.NoPerClassDefaultPorts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjTransportNoDefaultPortLiterals::RunTest(const FString& Parameters)
{
	// A default endpoint on a transport is a second place a port is written
	// down, and the two drift. Empty means "ask the net config", whose struct
	// defaults are now the only definition.
	const UURLabZmqRpcTransport* RepCdo =
		UURLabZmqRpcTransport::StaticClass()->GetDefaultObject<UURLabZmqRpcTransport>();
	TestTrue(TEXT("REP transport has no default endpoint literal"),
		RepCdo->StepEndpoint.IsEmpty());

	// The one definition site, so a reader can point at it.
	const FURLabBridgeServerConfig Defaults;
	TestEqual(TEXT("default step port"), Defaults.StepPort, 5559);
	TestEqual(TEXT("default state port"), Defaults.StatePort, 5555);
	return true;
}
