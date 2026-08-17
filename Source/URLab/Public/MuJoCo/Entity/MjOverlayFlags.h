// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

/**
 * MuJoCo simulate visualization parity: mirrors MuJoCo's own mjvOption flag set 1:1, so the wire is
 * a straight passthrough with no translation layer. A composable toggle bitmask on the renderer, NOT
 * a mode -- "render server" / "viewer" style bundles never re-appear as modes. Sized to
 * mjNVISFLAG/mjNRNDFLAG at runtime from the linked lib; one UMjOverlayRenderer reads it and emits
 * overlays grouped by technique (DrawDebug / pooled mesh / per-geom MID / visibility).
 */
struct URLAB_API FMjOverlayFlags
{
	TArray<uint8> VisFlags;   // mjNVISFLAG (mjVIS_*)

	// Per-group visibility masks (collision-vs-visual, etc.).
	TArray<uint8> GeomGroup;
	TArray<uint8> SiteGroup;
	TArray<uint8> JointGroup;
	TArray<uint8> TendonGroup;
	TArray<uint8> ActuatorGroup;
};
