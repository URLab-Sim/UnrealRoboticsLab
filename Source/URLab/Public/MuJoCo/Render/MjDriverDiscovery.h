// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"

class FJsonObject;

/**
 * One discovered render Driver: a live sim a Renderer can join. Read from
 * the shared registry directory (the entries a render owner writes). This is the
 * runtime counterpart of the editor's URLabLevelOps::FMjDriverInfo, usable from a
 * packaged game's server browser.
 */
struct FMjDriverInfo
{
	FString InstanceId;
	FString Scene;
	FString Host;
	FString Control; // REQ/REP control endpoint (serves the MJB via render_hello)
	FString Bus;     // geoms transform-bus endpoint
	int32 Ngeom = 0;
	int32 Pid = 0;
};

namespace URLabRender
{
/** The registry role/capability string that marks a joinable render owner. */
extern const TCHAR* const OwnerRole;

/**
 * The one shared role/capability match for the role-filtered registry readers.
 * A parsed registry entry is an owner when its "role" is "render_owner" OR its
 * "capabilities" array contains "render_owner". Kept as a single helper so this
 * rule lives in one place UE-side and matches the Python semantics
 * (pool.is_owner_entry). Returns false for a null object.
 */
URLAB_API bool IsOwnerEntry(const TSharedPtr<FJsonObject>& Obj);

/**
 * Read the shared registry directory and return every live render driver
 * (entries whose role/capabilities include "render_owner"). An entry is pruned
 * when its mtime is older than the heartbeat TTL OR its recorded pid is no longer
 * a live process (a crash/kill can leave a fresh-mtime file behind). Returns true
 * on success (an empty list is not an error); false with OutError only on a hard
 * failure.
 */
URLAB_API bool DiscoverDrivers(TArray<FMjDriverInfo>& OutDrivers, FString& OutError);
} // namespace URLabRender
