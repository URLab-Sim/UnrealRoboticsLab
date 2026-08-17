// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"

/**
 * One discovered fast-path owner: a live sim a render slave can join. Read from
 * the shared registry directory (the entries a FastPathOwner writes). This is the
 * runtime mirror of the editor's URLabLevelOps::FMjDriverInfo, usable from a
 * packaged game's server browser.
 */
struct FMjDriverInfo
{
	FString InstanceId;
	FString Scene;
	FString Host;
	FString Control; // REQ/REP control endpoint (serves the MJB via fastpath_hello)
	FString Bus;     // geoms transform-bus endpoint
	int32 Ngeom = 0;
	int32 Pid = 0;
};

namespace URLabFastPath
{
/**
 * Read the shared registry directory and return every live fast-path owner
 * (entries whose role/capabilities include "fastpath_owner"). Entries older than
 * the heartbeat TTL are skipped. Returns true on success (an empty list is not an
 * error); false with OutError only on a hard failure.
 */
URLAB_API bool DiscoverOwners(TArray<FMjDriverInfo>& OutOwners, FString& OutError);
} // namespace URLabFastPath
