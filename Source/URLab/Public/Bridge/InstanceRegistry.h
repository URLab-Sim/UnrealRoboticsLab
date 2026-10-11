// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"

struct FURLabBridgeServerConfig;

/**
 * The parts of a registry entry that come from the LIVE process rather than
 * from its boot configuration. Passed as a struct, not as defaulted trailing
 * parameters: `Ngeom` used to default to 0 and no call site ever overrode it,
 * so every Unreal owner advertised itself as having no geometry while the same
 * process's typed render metadata reported the true count. A struct makes each
 * omission visible at the call site instead of silently correct-looking.
 */
struct FURLabRegistryLiveState
{
	/** Build version string of this binary. */
	FString UrlabVersion;

	/** A live AAMjManager is attached (an editor is discoverable pre-PIE). */
	bool bManagerPresent = false;

	/** The cooperative lease is held by some client. */
	bool bBusy = false;

	/** Geom count of the loaded model, 0 when no model is up. */
	int32 Ngeom = 0;

	/** Runtime grants: what a peer is permitted to ask of this instance, already
	 *  resolved to their advertised wire tokens. These are what the registry's
	 *  `capabilities` array carries, and they come from the bridge rather than from
	 *  a manager, because half the instances that publish an entry have no manager. */
	TArray<FString> GrantTokens;

	/** Dialable dm_env_rpc endpoint, scheme included ("grpc://host:port").
	 *  Empty when no gRPC server is listening in this process, which is the
	 *  only condition under which the entry advertises one. */
	FString GrpcEndpoint;
};

/**
 * @class FURLabInstanceRegistry
 * @brief File-based, broker-less discovery for render-farm instances.
 *
 * Each process writes {registry_dir}/{instance_id}_{pid}.json on server start
 * and deletes it on clean shutdown; a client globs the directory to discover
 * live instances without any daemon. The Python owner
 * (render_owner.py::_write_registry) writes the SAME entry shape into the
 * SAME directory, and every reader in both repositories applies the same
 * liveness rule to it: fresh mtime (30 s heartbeat TTL) AND a running pid.
 *
 * Entry shape (every key is written on every entry, so one file is never two
 * different shapes depending on a launch flag):
 *
 *   instance_id, index, pid, host, role, capabilities[], features[], scene,
 *   ngeom, step_port, state_port, cam_base_port, control, bus, grpc,
 *   transports[], manager_present, busy, urlab_version, registry_written_at
 */
class URLAB_API FURLabInstanceRegistry
{
public:
	/** Protocol/build features this binary implements: what the wire can do
	 *  here. NOT grants: a grant says what a peer is permitted to ask for, and
	 *  the two used to share the `capabilities` key, which is why a pool
	 *  filtering on `camera_rpc` could never match an Unreal owner. */
	static const TArray<FString>& Features();


	/** This instance's id, falling back to the shared default when the config
	 *  leaves it empty. Also its `scene` name on both faces, so the registry
	 *  entry and the render handshake never disagree about what to call it. */
	static FString ResolveInstanceId(const FURLabBridgeServerConfig& Cfg);

	/** Registry directory: URLAB_REGISTRY_DIR override, else
	 *  %LOCALAPPDATA%/URLab/registry on Windows and ~/.cache/URLab/registry
	 *  (honouring XDG_CACHE_HOME) on Linux. */
	static FString ResolveRegistryDir();

	/** Path of this instance's entry file within ResolveRegistryDir(). */
	static FString ResolveEntryPath(const FURLabBridgeServerConfig& Cfg);

	/** Write (or overwrite) this instance's entry file, atomically. Also the
	 *  heartbeat: rewriting refreshes the mtime every reader ages against. */
	static void WriteEntry(const FURLabBridgeServerConfig& Cfg, const FURLabRegistryLiveState& Live);

	/** Delete this instance's entry file. Safe when it was never written. */
	static void RemoveEntry(const FURLabBridgeServerConfig& Cfg);
};
