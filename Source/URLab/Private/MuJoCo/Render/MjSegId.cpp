// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. This plugin incorporates
// third-party software: MuJoCo (Apache 2.0). See ThirdPartyNotices.txt.

#include "MuJoCo/Render/MjSegId.h"

#include "MuJoCo/Core/MjContentPaths.h"
#include "Wire/UrlabWire.gen.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

int32 MjSegObjTypeForMode(ERenderCameraMode Mode)
{
	// MuJoCo's own enum, not a copy of its numbers: the values cross the wire and
	// a client compares them against mujoco.mjtObj, so they have to come from the
	// same header the client's mujoco was built from.
	switch (Mode)
	{
		case ERenderCameraMode::InstanceSegmentation:
			return static_cast<int32>(mjOBJ_GEOM);
		case ERenderCameraMode::SemanticSegmentation:
			return static_cast<int32>(mjOBJ_BODY);
		default:
			return static_cast<int32>(mjOBJ_UNKNOWN);
	}
}

// ---------------------------------------------------------------------------
// What this build will serve
//
// Lives beside the encoding rather than in the header with the enum, because the
// answer is not a property of the vocabulary: it depends on whether the one
// asset that can write an id into a pixel actually cooked.
// ---------------------------------------------------------------------------

// STILL FALSE, and deliberately so. Everything the id path needs is built and
// individually verified -- the unlit M_MjSegId asset loads and reports itself
// unlit, it exposes the SegId parameter, each sibling's MID is written with the
// right lanes (1,0,0 / 2,0,0 / ...), the siblings sit at the right world
// transforms and are visible, the camera captures with CaptureSource
// SCS_SceneColorHDR and PrimitiveRenderMode PRM_UseShowOnlyList into a
// PF_FloatRGBA target with 4 entries in ShowOnlyComponents, and the readback
// decodes and ships HxWx2 int32 that the client reshapes correctly.
//
// What does NOT work is the one step in the middle: the emissive never reaches
// the readback. Every pixel comes back a dark grey with R==G==B (see
// urlab.Cam.SegDiag), which is the scene's shading and not any id -- so every
// pixel decodes to the background -1. Captured with SCS_BaseColor the same
// geometry returns a uniform ~0.10 grey, and an unlit material has NO base
// colour, which says the siblings are drawing as something other than the
// material they are wearing.
//
// Until that is understood, advertising these tokens would hand a caller a
// buffer of -1 under a name that promises ids. So the advert stays at two modes
// and render_frame refuses the other two, which is the same answer as before the
// id path existed -- just now for a reason with a known next step rather than
// four reasons of principle.
bool MjSegmentationIsProven()
{
	return false;
}

TArray<const TCHAR*> MjServableCaptureModes()
{
	TArray<const TCHAR*> Modes({URLabWire::CaptureMode::Real, URLabWire::CaptureMode::Depth});
	if (MjSegmentationIsProven() && URLabContent::HasSegIdMaterial())
	{
		Modes.Add(URLabWire::CaptureMode::Semantic);
		Modes.Add(URLabWire::CaptureMode::Instance);
	}
	return Modes;
}

bool MjIsCaptureModeServable(ERenderCameraMode Mode)
{
	if (Mode == ERenderCameraMode::Real || Mode == ERenderCameraMode::Depth)
	{
		return true;
	}
	return MjIsSegmentationMode(Mode) && MjSegmentationIsProven()
		&& URLabContent::HasSegIdMaterial();
}
