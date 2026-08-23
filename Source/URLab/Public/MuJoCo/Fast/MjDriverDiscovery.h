// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"

class FJsonObject;

/**
 * One discovered fast-path Driver: a live sim a Renderer can join. Read from
 * the shared registry directory (the entries a FastPathDriver writes). This is the
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
/** The registry role/capability string that marks a joinable fast-path owner. */
extern const TCHAR* const OwnerRole;

/**
 * The one shared role/capability match for the role-filtered registry readers.
 * A parsed registry entry is an owner when its "role" is "fastpath_owner" OR its
 * "capabilities" array contains "fastpath_owner". Kept as a single helper so this
 * rule lives in one place UE-side and matches the Python semantics
 * (pool.is_owner_entry). Returns false for a null object.
 */
URLAB_API bool IsOwnerEntry(const TSharedPtr<FJsonObject>& Obj);

/**
 * Read the shared registry directory and return every live fast-path driver
 * (entries whose role/capabilities include "fastpath_owner"). Entries older than
 * the heartbeat TTL are skipped. Returns true on success (an empty list is not an
 * error); false with OutError only on a hard failure.
 */
URLAB_API bool DiscoverDrivers(TArray<FMjDriverInfo>& OutDrivers, FString& OutError);
} // namespace URLabFastPath
