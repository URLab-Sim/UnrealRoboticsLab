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
class UURLabPublishTransport;
class UURLabClientSubscribeTransport;
class UURLabRpcClientTransport;
class UURLabCameraPublishTransport;
class UURLabBridgeServer;
class AAMjManager;

/**
 * @struct FMjExternalTransportProvider
 * @brief Factory hooks an optional, out-of-core transport module installs at
 *        startup so the core can create its transports without naming their
 *        concrete types.
 *
 * The baseline transports (ZMQ, SHM) are created directly by the core. An
 * additional transport module that ships as a separate, optional UE module
 * depends on the core and installs these hooks in its StartupModule; the core
 * invokes them through the abstract base pointers (UURLabRpcTransport /
 * UURLabPublishTransport) it already owns. When the optional module is absent
 * the hooks stay unbound and the core simply has no such transport.
 *
 * The factory is responsible for NewObject-ing its transport (with the supplied
 * outer) and any owner wiring (SetOwningBridge, consumer registration); the core
 * only calls TransportInit and adds the result to the appropriate owner list.
 */
DECLARE_DELEGATE_RetVal_OneParam(UURLabRpcTransport*, FMjMakeExternalRpcTransport, UURLabBridgeServer*);
DECLARE_DELEGATE_RetVal_OneParam(UURLabPublishTransport*, FMjMakeExternalPublishTransport, AAMjManager*);
DECLARE_DELEGATE_RetVal_OneParam(UURLabClientSubscribeTransport*, FMjMakeExternalClientSubscribeTransport, UObject* /*Outer*/);
DECLARE_DELEGATE_RetVal_OneParam(UURLabRpcClientTransport*, FMjMakeExternalRpcClientTransport, UObject* /*Outer*/);
DECLARE_DELEGATE_RetVal_OneParam(UURLabCameraPublishTransport*, FMjMakeExternalCameraPublishTransport, UObject* /*Outer*/);
// Owner -> external module: one raw render-bus frame per step, tagged by its tier
// topic. Currently always "render" -- per-body transforms + optional debug
// (source-of-truth §8.1/§8.2, the true mirror payload). The bytes are identical
// to what the matching ZMQ topic PUB carries. An optional module (gRPC) binds
// this to cache + server-stream the tier to its own subscribers, selecting by
// topic -- so the gRPC egress is decoupled from any bound ZMQ viewer bus.
// Multicast so >1 backend can listen; unbound is a no-op.
DECLARE_MULTICAST_DELEGATE_TwoParams(FMjViewerFrameSink, const FString& /*Topic*/, const TArray<uint8>& /*Payload*/);
// Render subscriber -> owner (the reverse leg of OnViewerFrame). A mirror's
// subscribe request negotiates the debug tier (§8.2) by carrying
// {contacts, overlay, maxcontacts} in its subscribe payload. An external server
// (gRPC) broadcasts that RAW subscribe payload here when a mirror subscribes; the
// owning AAMjManager binds this, parses it via ParseRenderDebugCaps, and folds it
// (union) into its thread-safe ActiveRenderDebugCaps so PublishRenderFrame
// serializes the requested contact/overlay tier. Carrying the raw bytes (not a
// typed struct) keeps this header decoupled from AAMjManager::FMjRenderDebugCaps.
// A lean mirror sends the bare {format:"render"} payload, which parses to no caps,
// so the owner still streams zero debug bytes. Multicast so a single owner picks
// it up; unbound (no owner) is a no-op.
DECLARE_MULTICAST_DELEGATE_OneParam(FMjRenderDebugCapsSink, const TArray<uint8>& /*SubscribePayload*/);

/**
 * @struct FMjExternalRpcTransportFactory
 * @brief One registered server-side control-RPC factory, keyed by the transport
 *        name its product reports via GetTransportName().
 *
 * The name is the dedup key EnsureExternalTransportsBound uses so a factory binds
 * exactly once per bridge (never a literal like "ros2-rpc"). Registration is a
 * list, not a single slot, so ROS ("ros2-rpc") and gRPC ("dm_env_rpc") bind side
 * by side instead of the later-loading module evicting the earlier (H1).
 */
struct FMjExternalRpcTransportFactory
{
	FName TransportName;
	FMjMakeExternalRpcTransport Factory;
};

/**
 * @struct FMjExternalPublishTransportFactory
 * @brief One registered server-side state-publish factory, keyed by the transport
 *        name its product reports via GetTransportName(). Same list-with-name-dedup
 *        model as FMjExternalRpcTransportFactory, on the manager's fan-out leg.
 */
struct FMjExternalPublishTransportFactory
{
	FName TransportName;
	FMjMakeExternalPublishTransport Factory;
};

struct URLAB_API FMjExternalTransportProvider
{
	/** Registered external request/reply + control-in transport factories, each
	 *  keyed by the transport name its product reports. Empty when no external
	 *  module is loaded. EnsureExternalTransportsBound iterates this list, dedups by
	 *  name, and binds each to the bridge -- so ROS and gRPC control transports
	 *  coexist (H1) and re-entering PIE never double-binds the same name (H2). Each
	 *  module APPENDS its factory in StartupModule instead of overwriting a slot. */
	static TArray<FMjExternalRpcTransportFactory> ControlRpcTransportFactories;

	/** Registered external per-step state consumer transport factories, each keyed
	 *  by transport name, bound onto the manager's fan-out leg. Same list model as
	 *  the control-RPC leg so >1 producer egress (e.g. ROS) coexists. */
	static TArray<FMjExternalPublishTransportFactory> StatePublishTransportFactories;

	/** External CLIENT-side render sink factories, keyed by endpoint SCHEME (e.g.
	 *  "grpc", "shm"). UURLabClientSubscribeTransport::Create parses the scheme from
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

	/** Broadcast one raw render-bus frame per step to any external backend (e.g. the
	 *  gRPC server), tagged by tier topic (currently always "render" transforms).
	 *  Unbound => no extra fan-out. */
	static FMjViewerFrameSink OnViewerFrame;

	/** A render subscriber's negotiated debug-tier caps, carried owner-ward as the
	 *  raw subscribe payload. Broadcast by an external server (gRPC) when a mirror
	 *  subscribes; bound by the owning AAMjManager, which parses + unions it into
	 *  ActiveRenderDebugCaps (thread-safe). The reverse leg of OnViewerFrame.
	 *  Unbound (no owner listening) => no-op. */
	static FMjRenderDebugCapsSink OnRenderDebugCapsRequested;

	/** True when at least one external module has registered a control RPC factory. */
	static bool HasControlRpcTransport();

	/** The scheme portion of an endpoint (the text before the first ':'), the key
	 *  the client-side factory maps use. Empty when the endpoint carries no scheme. */
	static FString SchemeOf(const FString& Endpoint);
};
