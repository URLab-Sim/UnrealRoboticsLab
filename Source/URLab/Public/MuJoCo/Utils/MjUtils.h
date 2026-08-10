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
#include "mujoco/mujoco.h"


/**
 * @class MjUtils
 * @brief Static utility class for MuJoCo <-> Unreal Engine helper functions.
 *
 * Frame and unit conversion is NOT here: it lives in URLabAxisConv, which is the
 * one place a handedness or unit audit has to read. This class provides:
 * - String conversions (char* <-> FString)
 * - MJCF attribute parsing helpers
 * - Debug drawing of compiled geoms and joints
 */
class URLAB_API MjUtils
{
public:
	/**
	 * @brief Converts a C-style string (possibly null) to an Unreal Engine FString.
	 *
	 * @param text Pointer to the C-string.
	 * @return FString The converted string, or empty if input is null.
	 */
	static FString MjToString(const char* text);

	/**
	 * @brief Copies an Unreal Engine FString into a fixed-size char buffer.
	 * Ensures null-termination.
	 *
	 * @param text The Unreal Engine string to convert.
	 * @param buffer Pointer to the destination char buffer.
	 * @param bufferSize The size of the destination buffer.
	 */
	static void StringToMj(const FString& text, char* buffer, int bufferSize);
	/**
	 * @brief Parses a "fromto" string ("x1 y1 z1 x2 y2 z2") into Start and End vectors.
	 *
	 * @param FromToStr The string containing 6 float values.
	 * @param OutStart The output start vector.
	 * @param OutEnd The output end vector.
	 * @return true if successful (6 values parsed), false otherwise.
	 */
	static bool ParseFromTo(const FString& FromToStr, FVector& OutStart, FVector& OutEnd);

	/**
	 * @brief Renders the collision geometries for a specific MuJoCo Geom (Primitives and Convex Hulls).
	 *
	 * @param World The UWorld context.
	 * @param m The MuJoCo model.
	 * @param geom_view The geometry view containing state and configuration.
	 * @param DrawColor The color to draw the wireframes.
	 * @param Multiplier Scaling factor for coordinate conversion.
	 */
	static void DrawDebugGeom(UWorld* World, const mjModel* m, const mjData* d, int32 GeomId,
		const FColor& DrawColor = FColor::Magenta, float Multiplier = 100.0f);

	/**
	 * @brief The same drawing, from a world transform the caller already holds.
	 *
	 * The shape comes from the model, which a compile fixes; only the pose comes
	 * from simulation state. A game-thread caller reads that pose out of the
	 * engine's published render snapshot rather than out of live mjData, and
	 * hands it here.
	 *
	 * @param GeomPos The geom's world position, 3 mjtNum (MuJoCo frame).
	 * @param GeomMat The geom's world orientation, 9 mjtNum row-major.
	 */
	static void DrawDebugGeom(UWorld* World, const mjModel* m, int32 GeomId,
		const mjtNum* GeomPos, const mjtNum* GeomMat,
		const FColor& DrawColor = FColor::Magenta, float Multiplier = 100.0f);

	/**
	 * @brief Draws joint range arc (hinge) or range bar (slide) with position indicator.
	 *
	 * @param World The UE world.
	 * @param Anchor World-space anchor position (UE coordinates, cm).
	 * @param Axis World-space axis direction (unit vector, UE coordinates).
	 * @param JointType MuJoCo joint type (mjJNT_HINGE or mjJNT_SLIDE).
	 * @param bLimited Whether the joint has limits enabled.
	 * @param RangeMin Lower limit (radians for hinge, meters for slide).
	 * @param RangeMax Upper limit (radians for hinge, meters for slide).
	 * @param CurrentPos Current joint position (radians or meters). Use NaN to skip.
	 * @param RefPos Reference position (qpos0). Use NaN to skip.
	 * @param ArcRadius Radius of the range arc in cm (default 10cm).
	 */
	static void DrawDebugJoint(UWorld* World, const FVector& Anchor, const FVector& Axis,
		int JointType, bool bLimited, float RangeMin, float RangeMax,
		float CurrentPos = NAN, float RefPos = NAN, float ArcRadius = 10.0f);

	/**
	 * @brief Prettifies an Unreal/MuJoCo name by stripping common unique ID suffixes (like _UAID_).
	 *
	 * @param Name The original raw name.
	 * @param PrefixToStrip Optional prefix (e.g. robot name) to also remove.
	 * @return FString The cleaned, human-readable name.
	 */
	static FString PrettifyName(const FString& Name, const FString& PrefixToStrip = TEXT(""));
};
