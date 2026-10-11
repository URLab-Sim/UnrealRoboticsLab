// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MjSegIdMaterial.h"

#include "AssetRegistry/AssetRegistryModule.h"
#include "MaterialEditingLibrary.h"
#include "Materials/Material.h"
#include "Materials/MaterialExpressionVectorParameter.h"
#include "Misc/PackageName.h"
#include "UObject/Package.h"
#include "UObject/SavePackage.h"

#include "MuJoCo/Render/MjSegId.h"
#include "URLabEditorLogging.h"

namespace urlab::editor
{
namespace
{
const TCHAR* const kSegIdPackageName = TEXT("/UnrealRoboticsLab/Materials/M_MjSegId");
const TCHAR* const kSegIdAssetName = TEXT("M_MjSegId");

/** Load the existing asset, or make the package and the asset for a new one.
 *  Suffixed rather than shadowing the master material's identical helper: these
 *  two files share a unity translation unit, where two anonymous namespaces are
 *  one anonymous namespace. */
UMaterial* OpenOrCreateSegId()
{
	if (UMaterial* Existing = LoadObject<UMaterial>(nullptr, kSegIdPackageName, nullptr, LOAD_NoWarn | LOAD_Quiet))
	{
		return Existing;
	}
	UPackage* Package = CreatePackage(kSegIdPackageName);
	if (Package == nullptr)
	{
		return nullptr;
	}
	Package->FullyLoad();
	UMaterial* Material = NewObject<UMaterial>(Package, FName(kSegIdAssetName), RF_Public | RF_Standalone);
	if (Material != nullptr)
	{
		FAssetRegistryModule::AssetCreated(Material);
	}
	return Material;
}
} // namespace

UMaterial* BuildMjSegIdMaterial(bool bSave)
{
	UMaterial* Material = OpenOrCreateSegId();
	if (Material == nullptr)
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Could not open or create %s"), kSegIdPackageName);
		return nullptr;
	}

	// Wholesale replacement rather than a patch: the graph is defined here and
	// nowhere else, so whatever the asset held before is not worth merging with.
	const TArray<TObjectPtr<UMaterialExpression>> Existing = Material->GetExpressionCollection().Expressions;
	for (UMaterialExpression* Expression : Existing)
	{
		if (Expression != nullptr)
		{
			UMaterialEditingLibrary::DeleteMaterialExpression(Material, Expression);
		}
	}

	// UNLIT is the whole point. An unlit surface writes its emissive straight
	// into scene colour during the base pass: no lighting, no shadowing, no
	// ambient occlusion, no tone curve, nothing between the number this material
	// is handed and the number the capture reads back. A lit material could not
	// carry an id at all -- its base colour is multiplied by whatever light
	// happens to reach the surface.
	Material->MaterialDomain = MD_Surface;
	Material->BlendMode = BLEND_Opaque;
	Material->SetShadingModel(MSM_Unlit);
	// Opaque and depth-tested, so a sibling in front of another occludes it and
	// the id image agrees with the colour image about what is visible.
	Material->bCastDynamicShadowAsMasked = false;
	// A MuJoCo geom is routinely viewed from inside (a room-sized box, a hollow
	// mesh). A one-sided id surface would leave those pixels reading background.
	Material->TwoSided = true;

	// The one parameter. Its RGB is `id + 1` split into three byte lanes, which
	// the runtime writes through MjSegIdToColor; the alpha is unused because an
	// unlit opaque material has no opacity input.
	UMaterialExpressionVectorParameter* SegId =
		Cast<UMaterialExpressionVectorParameter>(UMaterialEditingLibrary::CreateMaterialExpression(
			Material, UMaterialExpressionVectorParameter::StaticClass(), -400, 0));
	if (SegId == nullptr)
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Could not create the SegId parameter node"));
		return nullptr;
	}
	SegId->ParameterName = MjSegIdParameterName;
	// Black is the background id: an unset sibling reads as "nothing here"
	// rather than as geom 0, which is a real geom in every MuJoCo model.
	SegId->DefaultValue = FLinearColor(0.0f, 0.0f, 0.0f, 1.0f);

	UMaterialEditingLibrary::ConnectMaterialProperty(SegId, TEXT("RGB"), MP_EmissiveColor);

	UMaterialEditingLibrary::RecompileMaterial(Material);

	if (bSave)
	{
		UPackage* Package = Material->GetPackage();
		Package->MarkPackageDirty();
		const FString FileName =
			FPackageName::LongPackageNameToFilename(kSegIdPackageName, FPackageName::GetAssetPackageExtension());
		FSavePackageArgs SaveArgs;
		SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
		SaveArgs.SaveFlags = SAVE_NoError;
		if (!UPackage::SavePackage(Package, Material, *FileName, SaveArgs))
		{
			UE_LOG(LogURLabEditor, Error, TEXT("Could not save %s"), *FileName);
			return nullptr;
		}
		UE_LOG(LogURLabEditor, Display, TEXT("Wrote %s"), *FileName);
	}
	return Material;
}

} // namespace urlab::editor

int32 UMjBuildSegIdMaterialCommandlet::Main(const FString& Params)
{
	return urlab::editor::BuildMjSegIdMaterial(/*bSave=*/true) != nullptr ? 0 : 1;
}
