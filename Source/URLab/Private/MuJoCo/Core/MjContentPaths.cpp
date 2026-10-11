// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Core/MjContentPaths.h"

#include "Materials/MaterialInterface.h"
#include "UObject/UObjectGlobals.h"

#include <atomic>

DEFINE_LOG_CATEGORY_STATIC(LogURLabContent, Log, All);

namespace URLabContent
{
const TCHAR* const MasterMaterial =
	TEXT("/UnrealRoboticsLab/Materials/M_MuJoCo_Master.M_MuJoCo_Master");
const TCHAR* const OverlayMaterial =
	TEXT("/UnrealRoboticsLab/Materials/M_MjOverlay.M_MjOverlay");
const TCHAR* const SkyDomeMaterial =
	TEXT("/UnrealRoboticsLab/Materials/M_MjSkyDome.M_MjSkyDome");
const TCHAR* const SegIdMaterial =
	TEXT("/UnrealRoboticsLab/Materials/M_MjSegId.M_MjSegId");

namespace
{
/** Set once by VerifyRequiredContent; see URLabContent::HasSegIdMaterial. */
std::atomic<bool> GHasSegIdMaterial{false};

struct FRequiredAsset
{
	const TCHAR* Path;
	const TCHAR* Consequence;
};

// Ordered by how confusing the failure is when it happens silently.
const FRequiredAsset kRequired[] = {
	{MasterMaterial,      TEXT("imported geoms fall back to default lighting")},
	{OverlayMaterial,     TEXT("debug overlays draw opaque instead of translucent")},
	{SkyDomeMaterial,     TEXT("no sky dome is drawn")},
	{SegIdMaterial,       TEXT("segmentation is not served at all -- see HasSegIdMaterial")},
};
} // namespace

bool VerifyRequiredContent()
{
	TArray<FString> Missing;
	for (const FRequiredAsset& Asset : kRequired)
	{
		const bool bIsSegId = (Asset.Path == SegIdMaterial);
		// StaticLoadObject rather than a typed LoadObject: the widget is a
		// Blueprint generated class, not a UMaterialInterface, and here we only
		// care whether the package resolves at all.
		const bool bLoaded = StaticLoadObject(UObject::StaticClass(), nullptr, Asset.Path) != nullptr;
		if (bIsSegId)
		{
			// Recorded rather than merely reported, because this one changes what
			// the server ADVERTISES. Written on the game thread at module startup
			// and read afterwards from RPC threads, which is why it is atomic and
			// why it starts false.
			GHasSegIdMaterial.store(bLoaded, std::memory_order_release);
		}
		if (!bLoaded)
		{
			Missing.Add(FString::Printf(TEXT("  %s  (%s)"), Asset.Path, Asset.Consequence));
		}
	}

	if (Missing.Num() == 0)
	{
		UE_LOG(LogURLabContent, Verbose,
			TEXT("all %d required plugin assets resolved"), UE_ARRAY_COUNT(kRequired));
		return true;
	}

	// One actionable line, not N silent degradations found later by noticing the
	// render looks wrong. In a cooked build this almost always means the
	// directories were not cooked; the plugin's own Config/DefaultGame.ini asks
	// for them, so a host project that overrides packaging settings wholesale is
	// the usual cause.
	UE_LOG(LogURLabContent, Error,
		TEXT("%d of %d required plugin assets did not load. In a packaged build this ")
		TEXT("normally means their directories were not cooked; the plugin's ")
		TEXT("Config/DefaultGame.ini lists them under DirectoriesToAlwaysCook. Missing:\n%s"),
		Missing.Num(), static_cast<int32>(UE_ARRAY_COUNT(kRequired)),
		*FString::Join(Missing, TEXT("\n")));
	return false;
}

bool HasSegIdMaterial()
{
	return GHasSegIdMaterial.load(std::memory_order_acquire);
}
} // namespace URLabContent
