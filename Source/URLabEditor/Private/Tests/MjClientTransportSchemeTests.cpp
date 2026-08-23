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
// Tier-1 GAP 1.b (UE): the client transport factories choose the backend by
// endpoint SCHEME (source-of-truth §9.1). UURLabClientSubscribeTransport::Create
// and UURLabRpcClientTransport::Create map "tcp://" -> the built-in ZMQ backend,
// "grpc://" -> the gRPC factory, "shm://" -> the SHM factory. This pins the flagged
// follow-up that "shm:// now resolves".
//
// The routing is proved with socket-free test-double factories registered into the
// scheme->factory map, so no live network is touched: a grpc:// / shm:// endpoint
// must invoke the matching registered factory (proving the scheme parse + lookup),
// and an unregistered scheme (tcp://) must fall through to the concrete ZMQ class.
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "MjClientTransportDoubles.h"
#include "Transport/ClientSubscribeTransport.h"
#include "Transport/RpcClientTransport.h"
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
	// Register socket-free doubles for the two external schemes, keyed by scheme.
	auto& SubMap = FMjExternalTransportProvider::ClientSubscribeTransportFactories;
	const bool bHadGrpcSub = SubMap.Contains(TEXT("grpc"));
	const bool bHadShmSub = SubMap.Contains(TEXT("shm"));

	SubMap.Add(TEXT("grpc"), FMjMakeExternalClientSubscribeTransport::CreateLambda(
		[](UObject* O) -> UURLabClientSubscribeTransport* {
			UMjFakeClientSubscribeTransport* T = NewObject<UMjFakeClientSubscribeTransport>(O);
			T->SentinelName = TEXT("fake-grpc-sub");
			return T;
		}));
	SubMap.Add(TEXT("shm"), FMjMakeExternalClientSubscribeTransport::CreateLambda(
		[](UObject* O) -> UURLabClientSubscribeTransport* {
			UMjFakeClientSubscribeTransport* T = NewObject<UMjFakeClientSubscribeTransport>(O);
			T->SentinelName = TEXT("fake-shm-sub");
			return T;
		}));

	UURLabClientSubscribeTransport::FOnClientMessage Noop;

	{
		UURLabClientSubscribeTransport* Sub = UURLabClientSubscribeTransport::Create(
			Outer, TEXT("grpc://127.0.0.1:50051"), TEXT("render"), Noop);
		TestNotNull(TEXT("grpc:// subscribe resolves"), Sub);
		if (Sub)
		{
			TestEqual(TEXT("grpc:// -> gRPC subscribe factory"),
				Sub->GetTransportName(), FString(TEXT("fake-grpc-sub")));
			Sub->TransportShutdown();
		}
	}
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
	const bool bHadShmRpc = RpcMap.Contains(TEXT("shm"));

	RpcMap.Add(TEXT("grpc"), FMjMakeExternalRpcClientTransport::CreateLambda(
		[](UObject* O) -> UURLabRpcClientTransport* {
			UMjFakeRpcClientTransport* T = NewObject<UMjFakeRpcClientTransport>(O);
			T->SentinelName = TEXT("fake-grpc-req");
			return T;
		}));
	RpcMap.Add(TEXT("shm"), FMjMakeExternalRpcClientTransport::CreateLambda(
		[](UObject* O) -> UURLabRpcClientTransport* {
			UMjFakeRpcClientTransport* T = NewObject<UMjFakeRpcClientTransport>(O);
			T->SentinelName = TEXT("fake-shm-req");
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
	{
		UURLabRpcClientTransport* C = UURLabRpcClientTransport::Create(
			Outer, TEXT("shm:///tmp/urlab_shm_session"));
		TestNotNull(TEXT("shm:// rpc resolves"), C);
		if (C)
		{
			TestEqual(TEXT("shm:// -> SHM rpc factory"),
				C->GetTransportName(), FString(TEXT("fake-shm-req")));
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
	if (!bHadGrpcSub) SubMap.Remove(TEXT("grpc"));
	if (!bHadShmSub) SubMap.Remove(TEXT("shm"));
	if (!bHadGrpcRpc) RpcMap.Remove(TEXT("grpc"));
	if (!bHadShmRpc) RpcMap.Remove(TEXT("shm"));

	return true;
}
