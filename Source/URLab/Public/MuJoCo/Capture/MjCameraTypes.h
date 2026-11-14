// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.

#pragma once

#include "CoreMinimal.h"
#include "Wire/UrlabWire.gen.h"
#include "Render/RenderCameraTypes.h"

// EMjCameraTrackingMode, EMjCameraProjection moved to
// MuJoCo/Generated/MjArticulationEnums.h.

// ---------------------------------------------------------------------------
// The wire spelling of a capture mode
//
// One mapping, both directions, used by everything that has to name a mode on
// the wire: the per-camera `mode` in the session handshake, the `capture_mode`
// field on `render_frame`, and the `capture_modes` advert on `meta`. The tokens
// come from the generated wire vocabulary, so a rename can only happen in
// wire/urlab_wire.yaml.
// ---------------------------------------------------------------------------

/** The wire token for a mode. Total: every enumerator has a token. */
inline const TCHAR* MjCaptureModeToWire(ERenderCameraMode Mode)
{
	switch (Mode)
	{
		case ERenderCameraMode::Depth:
			return URLabWire::CaptureMode::Depth;
		case ERenderCameraMode::SemanticSegmentation:
			return URLabWire::CaptureMode::Semantic;
		case ERenderCameraMode::InstanceSegmentation:
			return URLabWire::CaptureMode::Instance;
		case ERenderCameraMode::Real:
		default:
			return URLabWire::CaptureMode::Real;
	}
}

/** Parse a wire token. False (leaving OutMode untouched) for anything the enum
 *  does not spell -- the caller reports that as a bad request rather than
 *  falling back to a default, because silently rendering colour for a token the
 *  client thought meant depth is the failure this field exists to prevent. */
inline bool MjCaptureModeFromWire(const FString& Token, ERenderCameraMode& OutMode)
{
	if (Token.Equals(URLabWire::CaptureMode::Real, ESearchCase::CaseSensitive))
	{
		OutMode = ERenderCameraMode::Real;
		return true;
	}
	if (Token.Equals(URLabWire::CaptureMode::Depth, ESearchCase::CaseSensitive))
	{
		OutMode = ERenderCameraMode::Depth;
		return true;
	}
	if (Token.Equals(URLabWire::CaptureMode::Semantic, ESearchCase::CaseSensitive))
	{
		OutMode = ERenderCameraMode::SemanticSegmentation;
		return true;
	}
	if (Token.Equals(URLabWire::CaptureMode::Instance, ESearchCase::CaseSensitive))
	{
		OutMode = ERenderCameraMode::InstanceSegmentation;
		return true;
	}
	return false;
}

/**
 * The modes this server will actually serve on `render_frame`, in schema order.
 * `meta` advertises exactly this list and the render handler refuses anything
 * outside it, so the advert and the enforcement cannot disagree.
 *
 * `real` and `depth` are unconditional: the mode picks the render target's pixel
 * format (BGRA8 / R32f) and the capture source (SCS_FinalColorLDR /
 * SCS_SceneDepth) and that is the whole dependency, so both hold in an editor
 * session, a headless run and a cooked package alike.
 *
 * The two segmentation modes depend on one cooked asset -- the unlit id-carrier
 * `M_MjSegId` -- because that material is what writes an id into a pixel. It is
 * listed in the plugin's own DirectoriesToAlwaysCook, so it is present unless a
 * host project overrode packaging wholesale; when it is not, this list is short
 * and a client is refused with a reason instead of handed an image of nothing.
 * That check is why this is a function call and not a constant.
 *
 * Not thread-affine: the answer is a boolean recorded at module startup.
 */
URLAB_API TArray<const TCHAR*> MjServableCaptureModes();

/** Whether MjServableCaptureModes() contains this mode. */
URLAB_API bool MjIsCaptureModeServable(ERenderCameraMode Mode);

/** True for the two segmentation modes, which share every code path that is not
 *  about which id they carry. */
inline bool MjIsSegmentationMode(ERenderCameraMode Mode)
{
	return Mode == ERenderCameraMode::SemanticSegmentation
		|| Mode == ERenderCameraMode::InstanceSegmentation;
}
