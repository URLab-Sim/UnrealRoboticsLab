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
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#pragma once

#include "CoreMinimal.h"

class AMjArticulation;

/**
 * The single owner of every name assembled or stripped for the state IR. A
 * canonical segment is legal as a ROS name segment, a tf2 frame_id, and a future
 * URDF link/joint name, so the tf2 tree a later phase publishes drops in with no
 * renaming.
 */
struct URLAB_API FMjCanonicalName
{
	/** Replace every char outside [A-Za-z0-9_] with '_', prefixing '_' when the
	 *  first char is a digit. Empty input returns empty. */
	static FString Sanitize(const FString& Segment);

	/** Canonical articulation segment. If a stable LogicalId ever lands it swaps
	 *  in here and nowhere else. */
	static FName ArtSegment(const AMjArticulation* Art);

	/** Canonical part segment: the compiled name with the compile-time
	 *  "<ArtName>_" prefix stripped, then sanitized. This is the one place that
	 *  strip happens. Art may be null (falls back to Sanitize(MjName)). */
	static FName PartSegment(const AMjArticulation* Art, const FString& MjName);

	/** "<art>/<part>" (e.g. "go2/imu_gyro"). */
	static FString Full(FName Art, FName Part);
};
