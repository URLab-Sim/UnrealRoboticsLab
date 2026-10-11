// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#pragma once

#include "CoreMinimal.h"

/**
 * @file MjRenderTags.h
 * @brief The re-index tag channel shared by the renderer view and its subobjects.
 *
 * A body / geom / instance / camera carries a "<prefix><id>" name tag so a saved level
 * can map its components back to MuJoCo ids. One maker and one parser keep the writer
 * and the reader in lockstep, with no hardcoded prefix lengths; they live here because
 * the camera rig writes the camera tags and the view reads all four back.
 */
namespace MjRenderTags
{
constexpr const TCHAR* Body = TEXT("MjbBody=");
constexpr const TCHAR* Geom = TEXT("MjbGeom=");
constexpr const TCHAR* Ism = TEXT("MjbIsm=");
constexpr const TCHAR* Cam = TEXT("MjbCam=");

inline FName Make(const TCHAR* Prefix, int32 Id)
{
	return FName(*FString::Printf(TEXT("%s%d"), Prefix, Id));
}

inline int32 ParseId(const FString& Tag, const TCHAR* Prefix)
{
	return Tag.StartsWith(Prefix) ? FCString::Atoi(*Tag.Mid(FCString::Strlen(Prefix))) : -1;
}
} // namespace MjRenderTags
