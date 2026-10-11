// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

/**
 * Every plugin asset this module loads by path, in one place.
 *
 * These are reached through LoadObject at runtime, which the cooker cannot see,
 * so they only exist in a packaged build because Config/DefaultGame.ini asks
 * for their directories. That coupling used to be invisible: the paths lived as
 * string literals scattered across six files, the cook rule lived in the HOST
 * project, and a mismatch between them degraded silently — geoms fell back to
 * default lighting, overlays drew opaque, the sky dome did not appear, and
 * nothing in the log connected the symptom to a missing asset.
 *
 * Naming them here does two things: the literals stop being duplicated, and
 * VerifyRequiredContent() can say at boot exactly which ones did not cook.
 */
namespace URLabContent
{
// --- materials -----------------------------------------------------------
/** Master material every imported geom instances. Missing => default lighting. */
URLAB_API extern const TCHAR* const MasterMaterial;
/** Translucent per-instance-colour parent for overlay meshes. Missing => opaque overlays. */
URLAB_API extern const TCHAR* const OverlayMaterial;
/** Sky-dome material. Missing => no dome is drawn. */
URLAB_API extern const TCHAR* const SkyDomeMaterial;
/** Unlit id-carrier the segmentation cameras tint. Missing => segmentation cannot be served. */
URLAB_API extern const TCHAR* const SegIdMaterial;

/**
 * Try to load every asset above and log one Error listing whatever is absent.
 *
 * Call once during startup. Returns true when everything resolved. The point is
 * a single actionable line at boot instead of N separate silent degradations
 * discovered later by looking at a render that seems slightly wrong.
 */
URLAB_API bool VerifyRequiredContent();

/**
 * Did SegIdMaterial resolve?
 *
 * The one required asset whose absence is not a degradation. Every other entry
 * above makes something look wrong -- unlit geoms, opaque overlays, no sky --
 * and the render still means what it says. Without this one a segmentation
 * capture would produce an image of nothing, or of whatever the fallback
 * material happens to draw, and calling that a segmentation would be a lie. So
 * it is answerable on its own: `meta` advertises the segmentation capture modes
 * only when this is true, and `render_frame` refuses them otherwise, which turns
 * a mis-cooked package into one clear refusal instead of a buffer of zeroes.
 *
 * Answered from the probe VerifyRequiredContent() already performs at module
 * startup, so it is a cached bool and safe to read from an RPC thread. False
 * before that runs, which is the safe direction: a server that has not finished
 * starting does not claim to serve segmentation.
 */
URLAB_API bool HasSegIdMaterial();
} // namespace URLabContent
