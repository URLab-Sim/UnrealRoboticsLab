// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#pragma once

#include "CoreMinimal.h"
#include "Render/RenderSceneResources.h"

class UMaterialInstanceDynamic;
class UMaterialInterface;
class UTexture2D;
class UWorld;

URLAB_API const TCHAR* RenderMaterialRoleParameter(ERenderMaterialRole Role);
URLAB_API void RenderMaterialRoleSampling(ERenderMaterialRole Role, bool& bOutSRGB, bool& bOutNormal);
URLAB_API UTexture2D* RenderNeutralTexture(ERenderMaterialRole Role);
URLAB_API UMaterialInterface* LoadRenderMasterMaterial();

/** Ensure every material used by a staged scene has its active-world shader map. */
URLAB_API bool EnsureRenderMaterialsReadyForWorld(const UWorld& World,
	TConstArrayView<UMaterialInterface*> Materials, FString& OutError);

/** Write every scalar and a neutral value for every texture role onto the shared master. */
URLAB_API void ApplyRenderMaterialParameters(UMaterialInstanceDynamic& Instance,
	const FRenderMaterialResource& Material, const FLinearColor& BaseColor,
	const FVector2D& TexUniformScale);
