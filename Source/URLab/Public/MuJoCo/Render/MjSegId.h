// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Capture/MjCameraTypes.h"

/**
 * @file MjSegId.h
 * @brief The one segmentation-id encoding, shared by the material that writes it
 *        and the readback that reads it.
 *
 * Segmentation answers "which MuJoCo object is this pixel?", and the answer has
 * to be an id the client already holds -- it has the same mjModel -- rather than
 * a colour it would need a shipped palette to invert. So the renderer writes the
 * id itself into the image and the readback decodes it exactly.
 *
 * ## Why three byte lanes and not one number
 *
 * The id travels as the emissive colour of an unlit material into Unreal's scene
 * colour, and scene colour is PF_FloatRGBA -- HALF precision. A half represents
 * integers exactly only up to 2048, so a raw id in a single channel would be
 * silently wrong for any model with more than 2048 geoms, and wrong in the worst
 * possible way: plausible, off by one or two, and only above a threshold no test
 * would happen to cross. Three lanes of 0..255 are each exactly representable
 * and together carry 24 bits, which is more geoms than MuJoCo can compile.
 *
 * It is also exactly what MuJoCo's own OpenGL segmentation does
 * (`segid = r + g*256 + b*65536`), so the two renderers encode the same way.
 *
 * ## Why the id is offset by one
 *
 * Nothing is drawn where no sibling covers a pixel, so the background is the
 * scene-colour clear: zero. Geom 0 is a real geom in every MuJoCo model, so a
 * raw id would make "the first geom" and "empty space" the same pixel. The
 * encoding therefore carries `id + 1` and decodes 0 back to -1. MuJoCo reserves
 * zero the same way, and `mujoco.Renderer` fills unoccupied pixels with -1.
 *
 * ## What the id MEANS
 *
 * Both segmentation modes emit the same `(objid, objtype)` pair
 * `mujoco.Renderer` returns, in MuJoCo's own vocabulary:
 *
 *   - InstanceSegmentation -> objid is a GEOM id, objtype is `mjOBJ_GEOM`.
 *     This is the mode that matches `mujoco.Renderer` exactly.
 *   - SemanticSegmentation -> objid is a BODY id, objtype is `mjOBJ_BODY`.
 *     Same pair, same decoding, one grouping coarser. MuJoCo has no second
 *     segmentation to be compatible with here, so this is an addition rather
 *     than a deviation.
 *
 * `objtype` is constant across a whole capture, which is why it is not encoded
 * per pixel: spending one of three lanes on a value that never varies within an
 * image would cost id range and buy nothing.
 */

/** Name of the vector parameter on `M_MjSegId` that carries the encoded id. */
inline const TCHAR* const MjSegIdParameterName = TEXT("SegId");

/** Highest id this encoding can carry. 24 bits, less the reserved zero. */
inline constexpr int32 MjSegIdMax = 0xFFFFFF - 1;

/**
 * Encode an object id as the three byte lanes the material writes to emissive.
 *
 * A negative id encodes as black, which decodes back to -1 -- the same answer
 * the background gives, and the right one for a sibling whose object could not
 * be identified.
 */
inline FLinearColor MjSegIdToColor(int32 Id)
{
	if (Id < 0 || Id > MjSegIdMax)
	{
		return FLinearColor(0.0f, 0.0f, 0.0f, 1.0f);
	}
	const uint32 Encoded = static_cast<uint32>(Id) + 1u;
	return FLinearColor(
		static_cast<float>(Encoded & 0xFFu),
		static_cast<float>((Encoded >> 8) & 0xFFu),
		static_cast<float>((Encoded >> 16) & 0xFFu),
		1.0f);
}

/**
 * Decode three scene-colour lanes back to an object id, or -1 for background.
 *
 * Rounds rather than truncates: the lanes are exact in half precision, but they
 * pass through a float render target and a copy, and 254.99999 must not decode
 * as 254. A lane outside 0..255 means the pixel is not one of ours -- a stray
 * bloom, a sky, a blended edge -- and the whole pixel reads as background,
 * because a partially valid id is not an id.
 */
inline int32 MjSegIdFromLanes(float R, float G, float B)
{
	const float Lanes[3] = {R, G, B};
	uint32 Encoded = 0;
	for (int32 i = 0; i < 3; ++i)
	{
		const float Rounded = FMath::RoundToFloat(Lanes[i]);
		if (Rounded < 0.0f || Rounded > 255.0f)
		{
			return -1;
		}
		Encoded |= static_cast<uint32>(Rounded) << (8 * i);
	}
	// Zero is the reserved background, not geom 0.
	return Encoded == 0u ? -1 : static_cast<int32>(Encoded - 1u);
}

/**
 * The `mjtObj` value every pixel of a capture in this mode carries.
 *
 * `mjOBJ_GEOM` for instance segmentation, `mjOBJ_BODY` for semantic, and
 * `mjOBJ_UNKNOWN` for a mode that is not segmentation at all. Defined in the
 * .cpp so this header does not drag mujoco.h into everything that names a
 * capture mode.
 */
URLAB_API int32 MjSegObjTypeForMode(ERenderCameraMode Mode);

/**
 * Whether the id path has been shown to produce real ids on real pixels.
 *
 * FALSE today. The encoding, the material, the capture setup, the readback and
 * the wire are all built and individually verified; the emissive does not reach
 * the readback, so every pixel decodes to background. This gates the `meta`
 * advert and the render_frame refusal, so a caller is told no rather than handed
 * a buffer of -1. See the comment on the definition for the evidence.
 */
URLAB_API bool MjSegmentationIsProven();
