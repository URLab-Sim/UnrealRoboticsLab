// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "MuJoCo/Core/MjOwnedModel.h"

/** Portable owned input accepted by MuJoCo simulation and render workflows. */
struct URLAB_API FMjModelSourcePayload
{
	EMjModelSourceKind Kind = EMjModelSourceKind::Mjb;
	TArray<uint8> PrimaryBytes;
	TMap<FString, TArray<uint8>> Assets;
	FString Label;
};

/** Local manifest ingress for a reusable owned MuJoCo source payload. */
class URLAB_API FMjModelSourceLoader
{
public:
	/**
	 * Load exactly {format,path,assets?}. Relative paths resolve against the
	 * manifest directory and all bytes are copied into the returned payload.
	 */
	static bool LoadManifest(const FString& ManifestPath,
		FMjModelSourcePayload& OutPayload, FString& OutError);
};
