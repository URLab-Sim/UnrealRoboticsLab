// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Core/MjCapabilities.h"

/**
 * @file MjLauncherFlags.h
 * @brief Parsers for the launcher surface, and the only place any of it is spelled.
 *
 * The grammar:
 *   -URLabWorkflow=simulation | render | viewer
 *   -URLabSimulationSource=<manifest.json>
 *   -URLabClock=stepped | freerun
 *   -URLabCaps=<capability>,...                       (csv; absent=lean; negatable: -accept_input)
 *   -URLabScene=level=<n>,origin=X;Y;Z,base,quality=off,cammax=N,camnear=<cm>,noexposure,overlay=<mask>,maxcontacts=<n>
 *   -URLabNet=id=,index=,portbase=,stride=,step=,state=,bus=,cam=,grpc=,bind=
 *
 * Simulation alone may name a source manifest. Render workflows receive their scenes
 * through their own render contracts.
 *
 * The capability tokens are not listed here on purpose: they come from
 * URLabWire::Capability, generated from wire/urlab_wire.yaml, so this file cannot hold a
 * second copy of the list to fall behind.
 */
namespace URLabLauncherFlags
{
// ---- generic csv sub-key readers (shared by Caps / Scene / Net) --------------------------------
//
// The csv flags carry commas as their own separators, so the whole "-URLabScene=a=1,b,c=2" value is
// read with FParse's separator-stop DISABLED (otherwise it would truncate at the first comma), then
// split here. FlagName includes the trailing '=' (e.g. TEXT("URLabScene=")).

/** value for a `key=value` token in a csv flag; false if the flag or key is absent. */
URLAB_API bool GetCsvValue(const TCHAR* FlagName, const TCHAR* Key, FString& OutValue);
/** true if a bare token (e.g. `base`) is present in a csv flag. */
URLAB_API bool HasCsvToken(const TCHAR* FlagName, const TCHAR* Key);

// ---- -URLabWorkflow -----------------------------------------------------------------------------

enum class EWorkflowKind : uint8
{
	None,     // -URLabWorkflow absent
	Invalid,  // present and unrecognised, which is a refusal rather than a silent lean boot
	Simulation,
	Render,
	Viewer
};

/** Parse the canonical workflow selector. Any legacy -URLabDrive token is invalid. */
URLAB_API EWorkflowKind ParseWorkflow(FString& OutValue);

// ---- -URLabSimulationSource -------------------------------------------------------------------

/** Path to the JSON source manifest. The flag is valid only with -URLabWorkflow=simulation.
 *  False means the flag is absent; an explicitly empty value returns true with an empty path
 *  so the boot resolver can refuse it. */
URLAB_API bool SimulationSourceManifest(FString& OutManifestPath);

// ---- -URLabClock ------------------------------------------------------------------------------

enum class EClockKind : uint8
{
	Absent,
	Stepped,
	FreeRun,
	Invalid
};

/** Parse the local simulation clock. Absent resolves to the boot default in
 * FMjRendererBoot; an unrecognised token is returned as Invalid for explicit refusal. */
URLAB_API EClockKind ParseClock(FString& OutValue);

// ---- -URLabCaps --------------------------------------------------------------------------------

/** Parse -URLabCaps. Each token is a capability name from URLabWire::Capability; a leading
 *  '-' negates it. Only the tokens actually named come back as named, because several boot
 *  rules must tell "absent" from "explicitly off". An unknown token is logged and skipped,
 *  which is the one place a silent skip is right: the flag is a set, not an instruction. */
URLAB_API FMjCapabilitySet ParseCaps();

/** True when the retired source-finder selector was supplied. The boot resolver uses
 * this only to reject the removed route explicitly; no compatibility behavior remains. */
URLAB_API bool RetiredSourceFindValue(FString& OutValue);

// ---- -URLabScene -------------------------------------------------------------------------------

/** origin=X;Y;Z (UE cm) (note ';' separators, since ',' delimits
 *  scene keys). Returns true only for a well-formed triple. */
URLAB_API bool SceneOrigin(FVector& OutOrigin);
/** level=<n>. */
URLAB_API bool SceneLevel(FString& OutLevel);
/** base. */
URLAB_API bool SceneBaseLevel();
/** quality=off. */
URLAB_API bool SceneNoQuality();
/** cammax=N. */
URLAB_API bool SceneCamMaxHeight(int32& OutPx);
/** camnear=<cm>. */
URLAB_API bool SceneCamNearClipCm(float& OutCm);
/** noexposure. */
URLAB_API bool SceneNoAutoExposure();
/** overlay=<mask> — a debug-overlay CONSUMER boot flag. Sets the
 *  initial `mj.RenderOverlayMask` cvar (which mjVIS_* overlays a subscriber draws) AND
 *  tells the subscriber to negotiate the owner's debug tier (StreamContacts +
 *  StreamOverlay) so the [OWNER] overlays (contacts/CoM/tendons/...) actually
 *  arrive on the wire. `<mask>` is a decimal or 0x-hex mjVIS_* bitmask. Returns
 *  true (filling OutMask) when present; the runtime cvar still toggles drawing. */
URLAB_API bool SceneOverlayMask(int32& OutMask);
/** maxcontacts=<n> — optional cap on the contact count the owner serializes onto
 *  each frame (0/absent = uncapped). Only meaningful with
 *  overlay=. Returns true (filling OutMax) when present. */
URLAB_API bool SceneOverlayMaxContacts(int32& OutMax);

// ---- -URLabNet ---------------------------------------------------------------------------------

/** value for a -URLabNet sub-key (id/index/portbase/stride/step/state/bus/cam/control/grpc/bind).
 *  Thin alias for GetCsvValue on the net flag, so callers name the key and nothing else. */
URLAB_API bool NetValue(const TCHAR* Key, FString& OutValue);

/** The gRPC transport reads its own bind port off the command line as the internal
 *  -URLabDmEnvPort= token, in a module the bridge config cannot re-plumb. These two are how
 *  -URLabNet=grpc= reaches it, kept here so the command line has exactly one reader and one
 *  writer in the whole plugin. */
URLAB_API bool DmEnvPort(int32& OutPort);
/** Append -URLabDmEnvPort=<Port> unless one is already present. Idempotent, because the bridge
 *  config is applied more than once per boot. */
URLAB_API void EnsureDmEnvPortToken(int32 Port);

// ---- test support ------------------------------------------------------------------------------

/**
 * RAII override of the process command line, restoring the original on scope exit.
 *
 * Lives with the parsers rather than in a test file so the "one reader of the command line"
 * invariant stays greppable: a test that needs a synthetic launch line drives it through the
 * same surface production code parses, instead of touching FCommandLine itself.
 */
struct URLAB_API FScopedCommandLine
{
	explicit FScopedCommandLine(const TCHAR* Line);
	~FScopedCommandLine();

	FScopedCommandLine(const FScopedCommandLine&) = delete;
	FScopedCommandLine& operator=(const FScopedCommandLine&) = delete;

private:
	FString Saved;
};
} // namespace URLabLauncherFlags
