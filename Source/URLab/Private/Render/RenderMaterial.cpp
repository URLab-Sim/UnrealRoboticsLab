// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Render/RenderMaterial.h"

#include "Engine/Texture2D.h"
#include "Engine/World.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "MaterialShared.h"
#include "Misc/App.h"
#include "RHIGlobals.h"

namespace
{
constexpr const TCHAR* RoleParameters[] = {
	TEXT("RgbTexture"), TEXT("OcclusionTexture"), TEXT("RoughnessTexture"),
	TEXT("MetallicTexture"), TEXT("NormalTexture"), TEXT("OpacityTexture"),
	TEXT("EmissiveTexture"), TEXT("RgbaTexture"), TEXT("OrmTexture")};
static_assert(UE_ARRAY_COUNT(RoleParameters) == static_cast<int32>(ERenderMaterialRole::Count));

constexpr const TCHAR* MasterMaterialPath =
	TEXT("/UnrealRoboticsLab/Materials/M_MuJoCo_Master.M_MuJoCo_Master");
constexpr const TCHAR* NeutralWhitePath =
	TEXT("/Engine/EngineResources/WhiteSquareTexture.WhiteSquareTexture");
constexpr const TCHAR* NeutralLinearWhitePath =
	TEXT("/UnrealRoboticsLab/Materials/T_URLabNeutralLinearWhite.T_URLabNeutralLinearWhite");
constexpr const TCHAR* NeutralNormalPath =
	TEXT("/Engine/EngineMaterials/FlatNormal.FlatNormal");
}

const TCHAR* RenderMaterialRoleParameter(ERenderMaterialRole Role)
{
	const int32 Index = static_cast<int32>(Role);
	return Index >= 0 && Index < static_cast<int32>(ERenderMaterialRole::Count)
		? RoleParameters[Index]
		: TEXT("");
}

void RenderMaterialRoleSampling(ERenderMaterialRole Role, bool& bOutSRGB, bool& bOutNormal)
{
	bOutSRGB = Role == ERenderMaterialRole::Rgb || Role == ERenderMaterialRole::Rgba
		|| Role == ERenderMaterialRole::Emissive;
	bOutNormal = Role == ERenderMaterialRole::Normal;
}

UTexture2D* RenderNeutralTexture(ERenderMaterialRole Role)
{
	const bool bColor = Role == ERenderMaterialRole::Rgb || Role == ERenderMaterialRole::Rgba
		|| Role == ERenderMaterialRole::Emissive;
	return LoadObject<UTexture2D>(nullptr, Role == ERenderMaterialRole::Normal
		? NeutralNormalPath : bColor ? NeutralWhitePath : NeutralLinearWhitePath);
}

UMaterialInterface* LoadRenderMasterMaterial()
{
	return LoadObject<UMaterialInterface>(nullptr, MasterMaterialPath);
}

bool EnsureRenderMaterialsReadyForWorld(const UWorld& World,
	TConstArrayView<UMaterialInterface*> Materials, FString& OutError)
{
	OutError.Reset();
	if (!FApp::CanEverRender() || GUsingNullRHI)
	{
		return true;
	}

	const EShaderPlatform ShaderPlatform =
		GetFeatureLevelShaderPlatform_Checked(World.GetFeatureLevel());
	TSet<UMaterialInterface*> UniqueMaterials;
	for (UMaterialInterface* Material : Materials)
	{
		if (!Material || UniqueMaterials.Contains(Material))
		{
			continue;
		}
		UniqueMaterials.Add(Material);
		Material->EnsureIsComplete();
		const FMaterialResource* Resource = Material->GetMaterialResource(ShaderPlatform);
		if (!Resource)
		{
			OutError = FString::Printf(
				TEXT("render material '%s' has no shader resource for the active world feature level"),
				*Material->GetPathName());
			return false;
		}
		if (!Resource->IsGameThreadShaderMapComplete())
		{
			OutError = FString::Printf(
				TEXT("render material '%s' has an incomplete shader map after preparation"),
				*Material->GetPathName());
			return false;
		}
	}
	return true;
}

void ApplyRenderMaterialParameters(UMaterialInstanceDynamic& Instance,
	const FRenderMaterialResource& Material, const FLinearColor& BaseColor,
	const FVector2D& TexUniformScale)
{
	Instance.SetVectorParameterValue(TEXT("BaseColor"), BaseColor);
	Instance.SetScalarParameterValue(TEXT("Metallic"), FMath::Clamp(Material.Metallic, 0.0f, 1.0f));
	Instance.SetScalarParameterValue(TEXT("Roughness"), FMath::Clamp(Material.Roughness, 0.0f, 1.0f));
	Instance.SetScalarParameterValue(TEXT("Specular"), FMath::Clamp(Material.Specular, 0.0f, 1.0f));
	Instance.SetScalarParameterValue(TEXT("Reflectance"), FMath::Clamp(Material.Reflectance, 0.0f, 1.0f));
	Instance.SetScalarParameterValue(TEXT("Emission"), FMath::Max(Material.Emission, 0.0f));

	FVector2D Repeat = Material.TexRepeat;
	if (Material.bTexUniform)
	{
		if (TexUniformScale.X > 0.0) Repeat.X *= TexUniformScale.X;
		if (TexUniformScale.Y > 0.0) Repeat.Y *= TexUniformScale.Y;
	}
	Instance.SetScalarParameterValue(TEXT("TexRepeatU"), static_cast<float>(Repeat.X));
	Instance.SetScalarParameterValue(TEXT("TexRepeatV"), static_cast<float>(Repeat.Y));

	for (int32 Index = 0; Index < static_cast<int32>(ERenderMaterialRole::Count); ++Index)
	{
		const ERenderMaterialRole Role = static_cast<ERenderMaterialRole>(Index);
		Instance.SetTextureParameterValue(RenderMaterialRoleParameter(Role), RenderNeutralTexture(Role));
	}
}
