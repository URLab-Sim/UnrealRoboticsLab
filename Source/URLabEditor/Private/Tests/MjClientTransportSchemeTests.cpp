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
// endorsed by, or sponsored by Epic Games, Inc.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

// ============================================================================
// MjClientTransportSchemeTests.cpp
//
// The client transport factories choose the backend by endpoint SCHEME.
// UURLabClientSubscribeTransport::Create
// and UURLabRpcClientTransport::Create map "tcp://" -> the built-in ZMQ backend,
// "grpc://" -> the gRPC request factory, and "shm://" -> the SHM viewer backend. This pins
// follow-up that "shm:// now resolves".
//
// The routing is proved with socket-free test-double factories registered into the
// scheme->factory map, so no live network is touched: a grpc:// / shm:// endpoint
// must invoke the matching registered factory (proving the scheme parse + lookup),
// and an unregistered scheme (tcp://) must fall through to the concrete ZMQ class.
//
// The second spec pins the other half of the contract: the ZMQ fallback belongs to
// tcp:// and the scheme-less form ONLY. An unregistered non-ZMQ scheme -- grpc:// on
// a platform where URLabDmEnvRpc is excluded by its PlatformAllowList -- must fail
// with an error naming the scheme and the module rather than quietly building a ZMQ
// socket that then fails naming neither.
//
// shm:// is the exception, and this is where that shows: the shared-memory render
// subscribe is in-core, so an unregistered shm:// SUBSCRIBE now resolves to the
// built-in reader rather than refusing. The shm:// RPC client is still nobody's, so
// that leg still refuses. The two legs disagreeing is the point.
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "MjClientTransportDoubles.h"
#include "Transport/ClientSubscribeTransport.h"
#include "Transport/RpcClientTransport.h"
#include "Transport/ShmClientSubscribeTransport.h"
#include "Transport/ZmqClientSubscribeTransport.h"
#include "Transport/ZmqRpcClientTransport.h"
#include "Transport/MjExternalTransportProvider.h"
#include "UObject/Package.h"

// ---------------------------------------------------------------------------
// URLab.Transport.ClientSchemeSelectsBackend
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjClientSchemeSelectsBackend,
	"URLab.Transport.ClientSchemeSelectsBackend",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjClientSchemeSelectsBackend::RunTest(const FString& Parameters)
{
	UObject* Outer = GetTransientPackage();

	// -- SUBSCRIBE side --------------------------------------------------------
	// gRPC intentionally has no viewer subscribe factory. SHM may still be supplied by
	// an external test double (and has an in-core fallback).
	auto& SubMap = FMjExternalTransportProvider::ClientSubscribeTransportFactories;
	FMjMakeExternalClientSubscribeTransport SavedGrpcSub;
	if (const auto* F = SubMap.Find(TEXT("grpc"))) SavedGrpcSub = *F;
	SubMap.Remove(TEXT("grpc"));
	const bool bHadShmSub = SubMap.Contains(TEXT("shm"));

	SubMap.Add(TEXT("shm"), FMjMakeExternalClientSubscribeTransport::CreateLambda(
		[](UObject* O) -> UURLabClientSubscribeTransport* {
			UMjFakeClientSubscribeTransport* T = NewObject<UMjFakeClientSubscribeTransport>(O);
			T->SentinelName = TEXT("fake-shm-sub");
			return T;
		}));

	UURLabClientSubscribeTransport::FOnClientMessage Noop;

	AddExpectedErrorPlain(TEXT("no client transport registered for scheme"),
		EAutomationExpectedErrorFlags::Contains, 0);
	TestNull(TEXT("grpc:// viewer subscribe is unsupported"),
		UURLabClientSubscribeTransport::Create(
			Outer, TEXT("grpc://127.0.0.1:50051"), TEXT("render"), Noop));
	{
		// The flagged follow-up: shm:// must now resolve to the SHM factory.
		UURLabClientSubscribeTransport* Sub = UURLabClientSubscribeTransport::Create(
			Outer, TEXT("shm:///tmp/urlab_shm_session"), TEXT("render"), Noop);
		TestNotNull(TEXT("shm:// subscribe resolves"), Sub);
		if (Sub)
		{
			TestEqual(TEXT("shm:// -> SHM subscribe factory"),
				Sub->GetTransportName(), FString(TEXT("fake-shm-sub")));
			Sub->TransportShutdown();
		}
	}

	// -- RPC-CLIENT side -------------------------------------------------------
	auto& RpcMap = FMjExternalTransportProvider::RpcClientTransportFactories;
	const bool bHadGrpcRpc = RpcMap.Contains(TEXT("grpc"));

	RpcMap.Add(TEXT("grpc"), FMjMakeExternalRpcClientTransport::CreateLambda(
		[](UObject* O) -> UURLabRpcClientTransport* {
			UMjFakeRpcClientTransport* T = NewObject<UMjFakeRpcClientTransport>(O);
			T->SentinelName = TEXT("fake-grpc-req");
			return T;
		}));


	{
		UURLabRpcClientTransport* C = UURLabRpcClientTransport::Create(
			Outer, TEXT("grpc://127.0.0.1:50051"));
		TestNotNull(TEXT("grpc:// rpc resolves"), C);
		if (C)
		{
			TestEqual(TEXT("grpc:// -> gRPC rpc factory"),
				C->GetTransportName(), FString(TEXT("fake-grpc-req")));
			C->TransportShutdown();
		}
	}


	// -- ZMQ fallback for an unregistered scheme (tcp://) ----------------------
	// tcp has no factory entry, so Create must build the concrete ZMQ class. This
	// constructs the real backend; keep it best-effort so a sandbox that cannot
	// open a loopback socket does not spuriously fail the routing assertions.
	// Ensure tcp is genuinely unregistered first.
	SubMap.Remove(TEXT("tcp"));
	RpcMap.Remove(TEXT("tcp"));
	{
		UURLabClientSubscribeTransport* Sub = UURLabClientSubscribeTransport::Create(
			Outer, TEXT("tcp://127.0.0.1:5561"), TEXT("render"), Noop);
		if (Sub)
		{
			TestNotNull(TEXT("tcp:// -> concrete ZMQ subscribe class"),
				Cast<UURLabZmqClientSubscribeTransport>(Sub));
			Sub->TransportShutdown();
		}
		else
		{
			AddInfo(TEXT("tcp:// subscribe Create returned null (loopback SUB could "
				"not start in this environment); ZMQ-fallback type not asserted."));
		}
	}
	{
		UURLabRpcClientTransport* C = UURLabRpcClientTransport::Create(
			Outer, TEXT("tcp://127.0.0.1:5571"));
		if (C)
		{
			TestNotNull(TEXT("tcp:// -> concrete ZMQ rpc class"),
				Cast<UURLabZmqRpcClientTransport>(C));
			C->TransportShutdown();
		}
		else
		{
			AddInfo(TEXT("tcp:// rpc Create returned null in this environment; "
				"ZMQ-fallback type not asserted."));
		}
	}

	// -- Restore the factory maps so later tests see the real registration -----
	if (SavedGrpcSub.IsBound()) SubMap.Add(TEXT("grpc"), SavedGrpcSub);
	if (!bHadShmSub) SubMap.Remove(TEXT("shm"));
	if (!bHadGrpcRpc) RpcMap.Remove(TEXT("grpc"));

	return true;
}

// ---------------------------------------------------------------------------
// URLab.Transport.UnknownSchemeFailsLoudly
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjClientUnknownSchemeFailsLoudly,
	"URLab.Transport.UnknownSchemeFailsLoudly",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjClientUnknownSchemeFailsLoudly::RunTest(const FString& Parameters)
{
	UObject* Outer = GetTransientPackage();

	// -- SchemeOf: a scheme is "text before ://", never "text before :" -------
	// A scheme-less "host:port" (which the registry still writes for gRPC
	// endpoints) must read as no-scheme, not as a scheme named after the host.
	TestEqual(TEXT("SchemeOf grpc://"),
		FMjExternalTransportProvider::SchemeOf(TEXT("grpc://127.0.0.1:50051")), FString(TEXT("grpc")));
	TestEqual(TEXT("SchemeOf tcp://"),
		FMjExternalTransportProvider::SchemeOf(TEXT("tcp://127.0.0.1:5559")), FString(TEXT("tcp")));
	TestEqual(TEXT("SchemeOf host:port has no scheme"),
		FMjExternalTransportProvider::SchemeOf(TEXT("127.0.0.1:5559")), FString());

	// -- The ZMQ allow-list ---------------------------------------------------
	TestTrue(TEXT("empty scheme is ZMQ's"),
		FMjExternalTransportProvider::IsBuiltInZmqScheme(FString()));
	TestTrue(TEXT("tcp is ZMQ's"),
		FMjExternalTransportProvider::IsBuiltInZmqScheme(TEXT("tcp")));
	TestFalse(TEXT("grpc is not ZMQ's"),
		FMjExternalTransportProvider::IsBuiltInZmqScheme(TEXT("grpc")));
	TestFalse(TEXT("shm is not ZMQ's"),
		FMjExternalTransportProvider::IsBuiltInZmqScheme(TEXT("shm")));

	// The error names the scheme and the module that would have supplied it.
	const FString Described = FMjExternalTransportProvider::DescribeUnavailableScheme(
		TEXT("grpc"), TEXT("grpc://127.0.0.1:50051"));
	TestTrue(TEXT("error names the scheme"), Described.Contains(TEXT("grpc")));
	TestTrue(TEXT("error names the module"), Described.Contains(TEXT("URLabDmEnvRpc")));

	// -- Create must refuse, not fall back ------------------------------------
	// Unregister both external schemes for the duration so this reproduces the
	// off-Linux / no-module case even on a box where URLabDmEnvRpc is loaded.
	auto& SubMap = FMjExternalTransportProvider::ClientSubscribeTransportFactories;
	auto& RpcMap = FMjExternalTransportProvider::RpcClientTransportFactories;
	FMjMakeExternalClientSubscribeTransport SavedGrpcSub, SavedShmSub;
	FMjMakeExternalRpcClientTransport SavedGrpcRpc, SavedShmRpc;
	if (const auto* F = SubMap.Find(TEXT("grpc"))) SavedGrpcSub = *F;
	if (const auto* F = SubMap.Find(TEXT("shm"))) SavedShmSub = *F;
	if (const auto* F = RpcMap.Find(TEXT("grpc"))) SavedGrpcRpc = *F;
	if (const auto* F = RpcMap.Find(TEXT("shm"))) SavedShmRpc = *F;
	SubMap.Remove(TEXT("grpc"));
	SubMap.Remove(TEXT("shm"));
	RpcMap.Remove(TEXT("grpc"));
	RpcMap.Remove(TEXT("shm"));

	// The refusal is logged at Error; that is the point of the fix, so expect it
	// rather than letting the automation framework read it as a failure.
	AddExpectedErrorPlain(TEXT("no client transport registered for scheme"),
		EAutomationExpectedErrorFlags::Contains, 0);

	UURLabClientSubscribeTransport::FOnClientMessage Noop;
	TestNull(TEXT("grpc:// subscribe refuses when unregistered"),
		UURLabClientSubscribeTransport::Create(
			Outer, TEXT("grpc://127.0.0.1:50051"), TEXT("render"), Noop));
	{
		// Shared memory is no longer somebody else's module to supply: with no
		// factory registered at all, Create builds the in-core reader. It attaches
		// to the owner's ring when one appears, so a directory that does not exist
		// yet is not a failure.
		UURLabClientSubscribeTransport* Sub = UURLabClientSubscribeTransport::Create(
			Outer, TEXT("shm:///tmp/urlab_shm_session"), TEXT("render"), Noop);
		TestNotNull(TEXT("shm:// subscribe resolves in-core with no factory"), Sub);
		if (Sub)
		{
			TestNotNull(TEXT("shm:// -> concrete SHM subscribe class"),
				Cast<UURLabShmClientSubscribeTransport>(Sub));
			Sub->TransportShutdown();
		}
	}
	TestNull(TEXT("grpc:// rpc refuses when unregistered"),
		UURLabRpcClientTransport::Create(Outer, TEXT("grpc://127.0.0.1:50051")));
	TestNull(TEXT("shm:// rpc refuses when unregistered"),
		UURLabRpcClientTransport::Create(Outer, TEXT("shm:///tmp/urlab_shm_session")));

	// -- Restore --------------------------------------------------------------
	if (SavedGrpcSub.IsBound()) SubMap.Add(TEXT("grpc"), SavedGrpcSub);
	if (SavedShmSub.IsBound()) SubMap.Add(TEXT("shm"), SavedShmSub);
	if (SavedGrpcRpc.IsBound()) RpcMap.Add(TEXT("grpc"), SavedGrpcRpc);
	if (SavedShmRpc.IsBound()) RpcMap.Add(TEXT("shm"), SavedShmRpc);

	return true;
}
