// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "BridgeServerConfig.generated.h"

/**
 * @struct FURLabBridgeServerConfig
 * @brief Persisted settings for the bridge server. Lives in
 *        Plugins/UnrealRoboticsLab/Config/LocalUnrealRoboticsLab.ini
 *        under [BridgeServer]. Same INI as the Python override path.
 */
USTRUCT(BlueprintType)
struct URLAB_API FURLabBridgeServerConfig
{
	GENERATED_BODY()

	/** Start the bridge server when the editor opens. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Bridge")
	bool bAutoStart = true;

	/** TCP port for the request/reply step RPC channel. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Bridge")
	int32 StepPort = 5559;

	/** TCP port for the state PUB channel. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Bridge")
	int32 StatePort = 5555;

	/** Human-readable instance name that seeds every namespaced resource
	 *  (SHM session dir, registry file, wake objects). Empty resolves to the
	 *  SHM layer's default ("live") for a single editor, or "instance_{index}"
	 *  when a farm index is set. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Bridge")
	FString InstanceId;

	/** Zero-based ordinal in a render farm. -1 means "not a farm member":
	 *  ports keep their single-editor values. When >= 0 the step/state/camera
	 *  ports derive from PortBase + Index*PortStride unless overridden. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Bridge")
	int32 InstanceIndex = -1;

	/** First port of the strided per-instance block. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Bridge")
	int32 PortBase = 5559;

	/** Port span reserved to each instance (step, state, several camera
	 *  ports, and headroom for future channels). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Bridge")
	int32 PortStride = 10;

	/** First camera stream port; cameras allocate upward from here. 0 means
	 *  "derive" (StepPort + 2, or PortBase + Index*PortStride + 2 for a farm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Bridge")
	int32 CamBasePort = 0;

	/** Interface the RPC, state, and camera sockets bind. Default 0.0.0.0 so
	 *  remote clients can connect; pin a NIC or loopback to restrict reach. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Bridge")
	FString BindAddress = TEXT("0.0.0.0");

	/** Owner-side render broadcast: when true, an owner binds a PUB on
	 *  RenderBusPort and publishes the render frame each step so consumers can
	 *  subscribe. The frame carries resolved per-body transforms
	 *  (bxpos/bxquat, plus camera transforms) and never qpos. When an external
	 *  client owns the physics it broadcasts instead, so this stays off.
	 *  Set by URLAB_BROADCAST_ON_RENDER_BUS or the `pose_stream` OUTPUT capability,
	 *  never by a command-line flag of its own. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Bridge")
	bool bBroadcastOnRenderBus = false;

	/** Write and heartbeat a registry entry so browsers, pools and farms can find
	 *  this instance. The `discoverable` SERVICE capability, and independent of
	 *  bBroadcastOnRenderBus above: one CLI token used to set both, so a farm node could
	 *  not be findable without also broadcasting every frame, or broadcast to the
	 *  consumers it already had without being listed for every new one. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Bridge")
	bool bDiscoverable = true;

	/** TCP port for the render bus (owner PUB / subscriber SUB). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Bridge")
	int32 RenderBusPort = 5560;

	/** gRPC (dm_env_rpc) listen port this instance serves, when the optional
	 *  URLabDmEnvRpc module is present. Defaults to the well-known dm_env_rpc
	 *  port (matches UURLabDmEnvRpcTransport's own default and the Python side's
	 *  session.DEFAULT_GRPC_PORT) so a broadcasting owner is discoverable by a
	 *  gRPC bus-joining subscriber with no extra config. Overridden by -URLabNet=grpc=
	 *  or a direct -URLabDmEnvPort= (see ApplyEnvAndCommandLineOverrides), which
	 *  also feeds the transport's own command-line reader so the two stay in
	 *  sync. Only advertised in the registry (InstanceRegistry.cpp) when this
	 *  instance is a published owner (bBroadcastOnRenderBus). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "URLab|Bridge")
	int32 DmEnvPort = 50051;
};
