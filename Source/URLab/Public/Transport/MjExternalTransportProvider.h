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

#pragma once

#include "CoreMinimal.h"

class UURLabRpcTransport;
class UURLabClientSubscribeTransport;
class UURLabRpcClientTransport;
class UURLabCameraPublishTransport;
class UURLabBridgeServer;

/**
 * @struct FMjExternalTransportProvider
 * @brief Factory hooks an optional, out-of-core transport module installs at
 *        startup so the core can create its transports without naming their
 *        concrete types.
 *
 * The baseline transports (ZMQ, SHM) are created directly by the core. An
 * additional transport module that ships as a separate, optional UE module
 * depends on the core and installs these hooks in its StartupModule; the core
 * invokes them through the abstract base pointers it already owns. When the optional module is absent
 * the hooks stay unbound and the core simply has no such transport.
 *
 * The factory is responsible for NewObject-ing its transport (with the supplied
 * outer) and any owner wiring (SetOwningBridge, consumer registration); the core
 * only calls TransportInit and adds the result to the appropriate owner list.
 */
DECLARE_DELEGATE_RetVal_OneParam(UURLabRpcTransport*, FMjMakeExternalRpcTransport, UURLabBridgeServer*);
DECLARE_DELEGATE_RetVal_OneParam(UURLabClientSubscribeTransport*, FMjMakeExternalClientSubscribeTransport, UObject* /*Outer*/);
DECLARE_DELEGATE_RetVal_OneParam(UURLabRpcClientTransport*, FMjMakeExternalRpcClientTransport, UObject* /*Outer*/);
DECLARE_DELEGATE_RetVal_OneParam(UURLabCameraPublishTransport*, FMjMakeExternalCameraPublishTransport, UObject* /*Outer*/);

/**
 * @struct FMjExternalRpcTransportFactory
 * @brief One registered server-side control-RPC factory, keyed by the transport
 *        name its product reports via GetTransportName().
 *
 * The name is the dedup key EnsureExternalTransportsBound uses so a factory binds
 * exactly once per bridge (never a literal like "ros2-rpc"). Registration is a
 * list, not a single slot, so ROS ("ros2-rpc") and gRPC ("dm_env_rpc") bind side
 * by side instead of the later-loading module evicting the earlier.
 */
struct FMjExternalRpcTransportFactory
{
	FName TransportName;
	FMjMakeExternalRpcTransport Factory;
};

struct URLAB_API FMjExternalTransportProvider
{
	/** Registered external request/reply + control-in transport factories, each
	 *  keyed by the transport name its product reports. Empty when no external
	 *  module is loaded. EnsureExternalTransportsBound iterates this list, dedups by
	 *  name, and binds each to the bridge -- so ROS and gRPC control transports
	 *  coexist and re-entering PIE never double-binds the same name. Each
	 *  module APPENDS its factory in StartupModule instead of overwriting a slot. */
	static TArray<FMjExternalRpcTransportFactory> ControlRpcTransportFactories;

	/** External CLIENT-side render sink factories, keyed by endpoint SCHEME (e.g.
	 *  "shm"). UURLabClientSubscribeTransport::Create parses the scheme from
	 *  the endpoint and selects the matching factory -- so gRPC and SHM each answer
	 *  their own scheme side by side. The factory returns an unconfigured transport
	 *  for the given Outer; the caller Configures + inits it exactly as for the
	 *  built-in ZMQ backend. No entry for a scheme => the core falls back to ZMQ. */
	static TMap<FString, FMjMakeExternalClientSubscribeTransport> ClientSubscribeTransportFactories;

	/** External CLIENT-side request/reply transport factories, keyed by endpoint
	 *  SCHEME (e.g. "grpc", "shm"). UURLabRpcClientTransport::Create parses the
	 *  scheme and selects the matching factory. Returns an unconfigured transport
	 *  for the given Outer; the caller Configures the endpoint + inits it. No entry
	 *  for a scheme => the core falls back to ZMQ. */
	static TMap<FString, FMjMakeExternalRpcClientTransport> RpcClientTransportFactories;

	/** Creates an external per-camera IMAGE egress transport (the role
	 *  UURLabPublishTransport excludes). Returns an unconfigured transport for the
	 *  given Outer; the caller opens per-camera channels and inits it. Unbound =>
	 *  the core uses its built-in camera backend. (Single-cast: no module binds it
	 *  today and its one consumer, MjCamera, has no H1/H2 exposure.) */
	static FMjMakeExternalCameraPublishTransport MakeCameraPublishTransport;

	/** True when at least one external module has registered a control RPC factory. */
	static bool HasControlRpcTransport();

	/** The scheme portion of an endpoint -- the text before "://" -- which is the
	 *  key the client-side factory maps use. Empty when the endpoint carries no
	 *  scheme. The "://" (rather than a bare ':') is required so a scheme-less
	 *  "host:port", which the registry still writes for gRPC endpoints, reads as
	 *  "no scheme" instead of inventing a scheme named after the host. */
	static FString SchemeOf(const FString& Endpoint);

	/** True when Scheme is one the built-in ZMQ client transports can carry, i.e.
	 *  the caller may fall back to ZMQ when no factory is registered. Only "tcp"
	 *  and the empty (scheme-less "host:port") form qualify: every other scheme
	 *  names a backend that some other module has to supply, and handing it to a
	 *  ZMQ socket only produces a zmq_connect failure naming neither the backend
	 *  nor the module that was meant to provide it. */
	static bool IsBuiltInZmqScheme(const FString& Scheme);

	/** True when a client-side render-sink factory is registered for Scheme. */
	static bool HasClientSubscribeScheme(const FString& Scheme);

	/** True when a client-side request/reply factory is registered for Scheme. */
	static bool HasRpcClientScheme(const FString& Scheme);

	/** One human-readable line explaining why Endpoint cannot be served: the
	 *  scheme, the module that would provide it (and the platforms that module
	 *  ships on), and the platform actually running. Used for the single error a
	 *  failed Create logs, so an operator on a platform where the backend module
	 *  is excluded is told that rather than being shown a ZMQ connect failure. */
	static FString DescribeUnavailableScheme(const FString& Scheme, const FString& Endpoint);
};
