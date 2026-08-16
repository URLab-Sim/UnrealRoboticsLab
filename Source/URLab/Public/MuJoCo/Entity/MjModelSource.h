// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#pragma once

#include "CoreMinimal.h"

struct mjModel_;
typedef struct mjModel_ mjModel;

/**
 * Normalize a wire model source to a compiled mjModel, whatever form it arrived in. The receiver
 * always compiles xml/mjz with its OWN libmujoco, so those two formats are immune to the MJB
 * version skew that forces a prebuilt buffer to be re-described downstream.
 */
namespace MjModelSource
{
/**
 * Compile Bytes to an mjModel. Format is "mjb" | "xml" | "mjz" (case-insensitive):
 *   mjb  loaded directly (fast, but version-locked to this libmujoco);
 *   xml  parsed to a spec against Assets mounted in a VFS, then compiled;
 *   mjz  decoded to a spec through the resource decoder registry, then compiled.
 * Assets are keyed by the bare filename the model references them under (ignored for mjb;
 * mjz archives are usually self-contained but any extras are mounted alongside).
 *
 * Returns a newly-owned mjModel the caller must mj_deleteModel, or nullptr with OutError set.
 */
URLAB_API mjModel* FromBytes(const TArray<uint8>& Bytes, const FString& Format,
	const TMap<FString, TArray<uint8>>& Assets, FString& OutError);
} // namespace MjModelSource
