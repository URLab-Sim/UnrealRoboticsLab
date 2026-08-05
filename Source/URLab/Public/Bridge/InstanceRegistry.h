// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"

struct FURLabBridgeServerConfig;

/**
 * @class FURLabInstanceRegistry
 * @brief File-based, broker-less discovery for render-farm instances.
 *
 * Each editor process writes {registry_dir}/{instance_id}_{pid}.json on
 * server start with the same fields as the `hello` instance block plus a
 * `registry_written_at` timestamp, and deletes it on clean shutdown. A
 * client reads the directory to discover live instances without any daemon.
 */
class URLAB_API FURLabInstanceRegistry
{
public:
	/** Feature flags advertised by this build. Single source shared by the
	 *  handshake `instance` block and the registry file. */
	static const TArray<FString>& Capabilities();

	/** Registry directory: URLAB_REGISTRY_DIR override, else
	 *  %LOCALAPPDATA%/URLab/registry on Windows and ~/.cache/URLab/registry
	 *  (honouring XDG_CACHE_HOME) on Linux. */
	static FString ResolveRegistryDir();

	/** Path of this instance's entry file within ResolveRegistryDir(). */
	static FString ResolveEntryPath(const FURLabBridgeServerConfig& Cfg);

	/** Write (or overwrite) this instance's entry file. */
	static void WriteEntry(const FURLabBridgeServerConfig& Cfg, const FString& UrlabVersion,
		bool bManagerPresent, bool bBusy);

	/** Rewrite the entry to refresh its mtime for the staleness heartbeat. */
	static void RefreshEntry(const FURLabBridgeServerConfig& Cfg, const FString& UrlabVersion,
		bool bManagerPresent, bool bBusy);

	/** Delete this instance's entry file. Safe when it was never written. */
	static void RemoveEntry(const FURLabBridgeServerConfig& Cfg);
};
