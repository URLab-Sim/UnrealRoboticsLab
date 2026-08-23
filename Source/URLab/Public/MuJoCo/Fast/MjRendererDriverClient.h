// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"

/**
 * @struct FMjRendererDriverClient
 * @brief Stateless ZMQ REQ/REP client that fetches a model from a fast-path Driver.
 *
 * A pure network helper (no state, no UObject): a Renderer or an editor command
 * calls FetchModel to pull a Driver's compiled model and its transform-bus endpoint
 * over the Driver's control channel.
 */
struct URLAB_API FMjRendererDriverClient
{
	/**
	 * Fetch a model and its transform-bus endpoint from a Driver over a ZMQ
	 * REQ/REP control channel. Sends a msgpack `{op:"fastpath_hello"}` and reads
	 * back the Driver's model plus `bus` endpoint. The Driver's `model_format`
	 * selects the source: `mjb` (default) loads its `mjb` bytes as-is, while `xml`
	 * recompiles the served MJCF and its `vfs_assets` bundle with this renderer's
	 * own libmujoco -- immune to MJB version skew -- normalizing either to the MJB
	 * returned in OutMjb. MJCF text served under `mjb` by an owner that failed to
	 * declare `model_format` is detected (an MJB never starts with '<') and
	 * compiled the same way. Synchronous with a short timeout; safe to call from the
	 * editor or a headless driver. Returns false with OutError on any failure.
	 */
	static bool FetchModel(const FString& ControlEndpoint,
		TArray<uint8>& OutMjb, FString& OutBusEndpoint, FString& OutError);
};
