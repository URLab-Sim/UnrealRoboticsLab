// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// The segmentation ID material, as code rather than as a graph nobody can review.
//
// Segmentation used to render a HASHED COLOUR: each geom got a sibling mesh
// tinted from a Halton palette keyed on a UClass-name hash, captured through the
// tone curve into an 8-bit target. Four separate things made those pixels
// impossible to turn back into ids -- the colour was not an id, the palette key
// was a per-process FName index, the tone curve moved the value, and the
// grouping was (class, body) rather than the geom id `mujoco.Renderer` reports.
//
// This material is the fix for the first three. It is UNLIT, so its emissive
// goes straight into scene colour in the base pass with no lighting, no tone
// curve and no post-process between the value written and the value read back.
// One vector parameter, `SegId`, carries the id; the capture reads scene colour
// HDR into a float render target and decodes it exactly.
//
// The encoding is MuJoCo's own: the id is written as three BYTE lanes,
//
//     R = id & 0xFF        G = (id >> 8) & 0xFF        B = (id >> 16) & 0xFF
//
// which is what MuJoCo's OpenGL segmentation does (`r + g*256 + b*65536`), and
// it is chosen for a hard reason rather than symmetry. Unreal's scene colour is
// PF_FloatRGBA -- HALF precision -- so a float channel represents integers
// exactly only up to 2048. A raw id in one channel would be silently wrong for
// any model with more than 2048 geoms. Three lanes of 0..255 are each exactly
// representable, and together they carry 24 bits.
//
// Zero is reserved for the background, so what is encoded is `id + 1` and a
// zero pixel decodes to "nothing here" (-1). MuJoCo reserves it the same way.
//
// Object TYPE is not encoded per pixel. It is constant for a whole capture --
// instance mode is all mjOBJ_GEOM, semantic mode is all mjOBJ_BODY -- so
// spending a lane on a value that never varies within one image would buy
// nothing and cost id range.
//
// Run it with:  UnrealEditor-Cmd <project> -run=MjBuildSegIdMaterial

#include "CoreMinimal.h"
#include "Commandlets/Commandlet.h"

#include "MjSegIdMaterial.generated.h"

class UMaterial;

namespace urlab::editor
{

/**
 * Build (or rebuild) `/UnrealRoboticsLab/Materials/M_MjSegId`.
 *
 * Replaces the asset's expression graph wholesale, so it is idempotent and an
 * existing asset is upgraded in place, keeping every reference to it valid.
 * `bSave` writes the package; pass false to build one for inspection only.
 */
UMaterial* BuildMjSegIdMaterial(bool bSave);

} // namespace urlab::editor

/** Commandlet wrapper, so the asset can be rebuilt without opening the editor. */
UCLASS()
class UMjBuildSegIdMaterialCommandlet : public UCommandlet
{
	GENERATED_BODY()

public:
	virtual int32 Main(const FString& Params) override;
};
