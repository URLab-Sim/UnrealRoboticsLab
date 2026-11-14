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
 *   mjz  expanded in memory (stored/deflate zip), then compiled as its model.xml with the
 *        archive entries as assets.
 * Assets are keyed by the filename the model references them under (ignored for mjb;
 * mjz archives are usually self-contained but any extras are mounted alongside). Supplied
 * asset names/data must be non-empty, every VFS mount must succeed, and XML may not contain
 * an embedded NUL byte.
 *
 * xml/mjz never read the host filesystem: asset names and every file reference must be
 * relative paths without '..', a root, a drive or a scheme, and every file reference
 * (including <include> and sub-models) must name a supplied asset. Violations fail before
 * MuJoCo parses anything.
 *
 * Returns a newly-owned mjModel the caller must mj_deleteModel, or nullptr with OutError set.
 */
URLAB_API mjModel* FromBytes(const TArray<uint8>& Bytes, const FString& Format,
	const TMap<FString, TArray<uint8>>& Assets, FString& OutError);

/**
 * Compile a model file on disk to an MJB buffer with this libmujoco. Format is
 * "mjb" | "xml" | "mjz" (case-insensitive). For "xml" the file is parsed with
 * mj_loadXML, so its meshdir/texturedir and <include>s resolve from the file's own
 * directory on disk -- no side-channel assets needed. "mjz" is read and compiled
 * like FromBytes (self-contained); "mjb" is loaded and re-saved. Used by the render
 * launcher so -URLabFastXml / -URLabFastMjz can boot the render server directly.
 *
 * Returns true and fills OutMjb, or false with OutError set.
 */
URLAB_API bool CompileFileToMjb(const FString& Path, const FString& Format,
	TArray<uint8>& OutMjb, FString& OutError);
} // namespace MjModelSource
