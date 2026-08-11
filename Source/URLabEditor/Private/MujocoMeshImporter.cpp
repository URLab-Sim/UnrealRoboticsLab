// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MujocoMeshImporter.h"

#include "AssetImportTask.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "AssetToolsModule.h"
#include "Engine/StaticMesh.h"
#include "Engine/Texture2D.h"
#include "Factories/FbxFactory.h"
#include "Factories/FbxImportUI.h"
#include "Factories/FbxStaticMeshImportData.h"
#include "FileHelpers.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "ImageUtils.h"
#include "Misc/FileHelper.h"
#include "Misc/PackageName.h"
#include "Misc/Paths.h"
#include "PackageTools.h"
#include "UObject/Package.h"
#include "UObject/SavePackage.h"

#include "MuJoCo/Spec/MjAssetResolve.h"

#include "URLabEditorLogging.h"

namespace urlab::editor
{
namespace
{

/** True when the mesh Unreal produced has geometry a renderer can actually draw. */
bool ValidateMesh(UStaticMesh* Mesh, const FString& MeshName)
{
	if (!Mesh)
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Mesh validation failed: Mesh is null"));
		return false;
	}

	if (!Mesh->GetRenderData())
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Mesh '%s' has no render data"), *MeshName);
		return false;
	}

	if (Mesh->GetRenderData()->LODResources.Num() == 0)
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Mesh '%s' has no LOD resources"), *MeshName);
		return false;
	}

	const FStaticMeshLODResources& LOD0 = Mesh->GetRenderData()->LODResources[0];

	if (LOD0.VertexBuffers.StaticMeshVertexBuffer.GetNumVertices() == 0)
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Mesh '%s' has empty vertex buffer"), *MeshName);
		return false;
	}

	if (LOD0.IndexBuffer.GetNumIndices() == 0)
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Mesh '%s' has empty index buffer"), *MeshName);
		return false;
	}

	const int32 NumVertices = LOD0.VertexBuffers.StaticMeshVertexBuffer.GetNumVertices();
	const int32 NumTriangles = LOD0.IndexBuffer.GetNumIndices() / 3;
	const int32 NumUVChannels = LOD0.VertexBuffers.StaticMeshVertexBuffer.GetNumTexCoords();

	UE_LOG(LogURLabEditor, Log, TEXT("Mesh '%s' validation: %d vertices, %d triangles, %d UV channels"),
		*MeshName, NumVertices, NumTriangles, NumUVChannels);

	if (NumUVChannels == 0)
	{
		UE_LOG(LogURLabEditor, Warning, TEXT("Mesh '%s' has no UV channels - materials may not display correctly"), *MeshName);
	}

	return true;
}

/** One import attempt at a chosen tangent basis; null when Unreal produced nothing. */
UStaticMesh* AttemptMeshImport(const FString& SourcePath, const FString& DestinationPath, const FString& AssetName,
	EFBXNormalGenerationMethod::Type NormalMethod)
{
	IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools").Get();

	UAssetImportTask* ImportTask = NewObject<UAssetImportTask>();
	ImportTask->Filename = SourcePath;
	ImportTask->DestinationPath = DestinationPath;
	// The element's name, not the file's: MJCF lets `<mesh name="tabletop"
	// file="table.obj"/>` differ, and every geom refers to the element.
	ImportTask->DestinationName = AssetName;
	ImportTask->bAutomated = true;
	ImportTask->bSave = true;
	ImportTask->bReplaceExisting = true;
	ImportTask->bReplaceExistingSettings = true;

	const FString Extension = FPaths::GetExtension(SourcePath).ToLower();

	if (Extension == "fbx" || Extension == "obj" || Extension == "t3d")
	{
		UFbxFactory* FbxFactory = NewObject<UFbxFactory>();
		ImportTask->Factory = FbxFactory;

		UFbxImportUI* ImportUI = NewObject<UFbxImportUI>();
		ImportUI->bImportMesh = true;
		ImportUI->bImportTextures = false;
		ImportUI->bImportMaterials = false;
		ImportUI->bAutomatedImportShouldDetectType = false;
		ImportUI->MeshTypeToImport = FBXIT_StaticMesh;

		ImportUI->StaticMeshImportData->bCombineMeshes = true;
		ImportUI->StaticMeshImportData->bRemoveDegenerates = true;
		ImportUI->StaticMeshImportData->bComputeWeightedNormals = true;
		ImportUI->StaticMeshImportData->bGenerateLightmapUVs = true;
		ImportUI->StaticMeshImportData->NormalImportMethod = EFBXNormalImportMethod::FBXNIM_ComputeNormals;
		ImportUI->StaticMeshImportData->NormalGenerationMethod = NormalMethod;

		// Collision is the spec's job, not the mesh's, and Nanite refuses
		// the self-intersecting geometry a convex-decomposed collision hull has.
		ImportUI->StaticMeshImportData->bAutoGenerateCollision = false;
		ImportUI->StaticMeshImportData->bBuildReversedIndexBuffer = true;
		ImportUI->StaticMeshImportData->bBuildNanite = false;

		FbxFactory->ImportUI = ImportUI;
		FbxFactory->EnableShowOption();
	}
	else
	{
		// GLTF and GLB go through Interchange, which picks its own factory. The
		// fine-grained normal settings above are lost, but those formats carry
		// their own tangent data and rarely need them.
		ImportTask->Factory = nullptr;

		UE_LOG(LogURLabEditor, Log, TEXT("Using automated factory detection for mesh: %s"), *SourcePath);
	}

	TArray<UAssetImportTask*> ImportTasks;
	ImportTasks.Add(ImportTask);
	AssetTools.ImportAssetTasks(ImportTasks);

	TArray<UObject*> ImportedAssets;
	for (UObject* Obj : ImportTask->GetObjects())
	{
		if (Obj)
		{
			ImportedAssets.Add(Obj);
		}
	}

	UE_LOG(LogURLabEditor, Log, TEXT("[ImportMeshAsset] Import returned %d objects:"), ImportedAssets.Num());
	for (int32 i = 0; i < ImportedAssets.Num(); ++i)
	{
		UObject* Obj = ImportedAssets[i];
		UE_LOG(LogURLabEditor, Log, TEXT("  [%d] %s (%s) at %s"),
			i, *Obj->GetName(), *Obj->GetClass()->GetName(), *Obj->GetPathName());
	}

	// A GLB import returns its textures too, and sometimes first.
	UStaticMesh* Mesh = nullptr;
	for (UObject* Obj : ImportedAssets)
	{
		Mesh = Cast<UStaticMesh>(Obj);
		if (Mesh)
		{
			break;
		}
	}

	if (!Mesh)
	{
		const FString MeshName = AssetName;

		// Interchange nests its output under a per-file folder, and which one it
		// picks depends on the format, so all of them are tried before falling
		// back to asking the registry what actually landed.
		const TArray<FString> SearchPaths = {
			FString::Printf(TEXT("%s/%s/StaticMeshes/%s.%s"), *DestinationPath, *MeshName, *MeshName, *MeshName),
			FString::Printf(TEXT("%s/%s/StaticMeshes/%s"), *DestinationPath, *MeshName, *MeshName),
			FString::Printf(TEXT("%s/%s.%s"), *DestinationPath, *MeshName, *MeshName),
		};

		for (const FString& SearchPath : SearchPaths)
		{
			Mesh = LoadObject<UStaticMesh>(nullptr, *SearchPath);
			if (Mesh)
			{
				UE_LOG(LogURLabEditor, Log, TEXT("[ImportMeshAsset] Found mesh at: %s"), *SearchPath);
				break;
			}
			UE_LOG(LogURLabEditor, Log, TEXT("[ImportMeshAsset] Not found at: %s"), *SearchPath);
		}

		if (!Mesh)
		{
			const FString SearchDir = FString::Printf(TEXT("%s/%s"), *DestinationPath, *MeshName);
			UE_LOG(LogURLabEditor, Log, TEXT("[ImportMeshAsset] Searching asset registry under: %s"), *SearchDir);

			IAssetRegistry& AssetRegistry = FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry").Get();
			TArray<FAssetData> Assets;
			AssetRegistry.GetAssetsByPath(FName(*SearchDir), Assets, true);

			for (const FAssetData& Asset : Assets)
			{
				UE_LOG(LogURLabEditor, Log, TEXT("  Registry: %s (%s)"), *Asset.AssetName.ToString(), *Asset.AssetClassPath.ToString());
				if (Asset.AssetClassPath.GetAssetName() == TEXT("StaticMesh"))
				{
					Mesh = Cast<UStaticMesh>(Asset.GetAsset());
					if (Mesh)
					{
						UE_LOG(LogURLabEditor, Log, TEXT("[ImportMeshAsset] Found mesh via registry: %s"), *Asset.GetObjectPathString());
						break;
					}
				}
			}
		}
	}

	if (!Mesh)
	{
		return nullptr;
	}

	// Interchange leaves its own materials in the mesh's slots, and those
	// reference textures the import stripped. A null texture sampled from the
	// render thread crashes the content browser as soon as it thumbnails the
	// asset (UE-23902), so every slot is reset to the default surface material.
	for (FStaticMaterial& Mat : Mesh->GetStaticMaterials())
	{
		Mat.MaterialInterface = UMaterial::GetDefaultMaterial(MD_Surface);
	}

	// Without this the asset reports zero bounds, and a zero-bounds mesh is
	// culled from every view.
	Mesh->Build();
	Mesh->CalculateExtendedBounds();

	UPackage* Package = Mesh->GetOutermost();
	FEditorFileUtils::PromptForCheckoutAndSave({Package}, false, false);

	UE_LOG(LogURLabEditor, Log, TEXT("Imported mesh '%s' - Bounds: %s"),
		*FPaths::GetBaseFilename(SourcePath),
		*Mesh->GetBoundingBox().GetSize().ToString());

	return Mesh;
}

} // namespace

UStaticMesh* ImportMeshAsset(const FString& SourcePath, const FString& DestinationPath, const FString& AssetName)
{
	if (SourcePath.IsEmpty() || DestinationPath.IsEmpty() || AssetName.IsEmpty())
	{
		return nullptr;
	}

	// The same sanitiser the preview's lookup uses, so the name the import
	// writes and the name the preview asks for cannot diverge.
	const FString FileName = MjSanitizeAssetName(AssetName);
	const FString PackageName = UPackageTools::SanitizePackageName(FPaths::Combine(DestinationPath, FileName));

	if (UStaticMesh* ExistingMesh = LoadObject<UStaticMesh>(nullptr, *PackageName))
	{
		return ExistingMesh;
	}

	UE_LOG(LogURLabEditor, Log, TEXT("Importing mesh from: %s to %s"), *SourcePath, *DestinationPath);

	FString ActualSourcePath = SourcePath;
	const FString BasePath = FPaths::ChangeExtension(SourcePath, "");

	const TArray<FString> Extensions = {TEXT("fbx"), TEXT("glb"), TEXT("gltf")};
	bool bFoundHigherPriority = false;

	for (const FString& Ext : Extensions)
	{
		const FString PotentialPath = BasePath + TEXT(".") + Ext;
		if (FPaths::FileExists(PotentialPath))
		{
			ActualSourcePath = PotentialPath;
			bFoundHigherPriority = true;
			UE_LOG(LogURLabEditor, Log, TEXT("Found higher priority mesh file: %s"), *ActualSourcePath);
			break;
		}
	}

	if (!bFoundHigherPriority && !FPaths::FileExists(ActualSourcePath))
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Source mesh file does not exist: %s"), *ActualSourcePath);
		return nullptr;
	}

	UStaticMesh* ImportedMesh = AttemptMeshImport(ActualSourcePath, DestinationPath, FileName, EFBXNormalGenerationMethod::MikkTSpace);

	if (ImportedMesh && ValidateMesh(ImportedMesh, FileName))
	{
		UE_LOG(LogURLabEditor, Log, TEXT("Successfully imported mesh '%s' with MikkTSpace"), *FileName);
		return ImportedMesh;
	}

	// MikkTSpace needs clean UVs and rejects the overlapping vertices an OBJ
	// exported from a CAD tool routinely has; Unreal's own generator does not.
	if (ImportedMesh)
	{
		UE_LOG(LogURLabEditor, Warning, TEXT("Mesh '%s' has issues with MikkTSpace, attempting fallback with BuiltIn normals"), *FileName);
	}
	else
	{
		UE_LOG(LogURLabEditor, Warning, TEXT("Failed to import mesh '%s' with MikkTSpace, attempting fallback"), *FileName);
	}

	ImportedMesh = AttemptMeshImport(ActualSourcePath, DestinationPath, FileName, EFBXNormalGenerationMethod::BuiltIn);

	if (ImportedMesh && ValidateMesh(ImportedMesh, FileName))
	{
		UE_LOG(LogURLabEditor, Warning, TEXT("Successfully imported mesh '%s' with BuiltIn normals (fallback)"), *FileName);
		return ImportedMesh;
	}

	UE_LOG(LogURLabEditor, Error, TEXT("Failed to import mesh '%s' - all import methods failed"), *FileName);
	return nullptr;
}

UTexture2D* ImportTextureAsset(
	const FString& SourcePath, const FString& DestinationPath, const FString& AssetName, bool bSrgb)
{
	if (SourcePath.IsEmpty() || DestinationPath.IsEmpty() || AssetName.IsEmpty())
	{
		return nullptr;
	}

	// The same sanitiser the preview's lookup uses, so the name the import
	// writes and the name the preview asks for cannot diverge.
	const FString FileName = MjSanitizeAssetName(AssetName);
	const FString PackageName = UPackageTools::SanitizePackageName(FPaths::Combine(DestinationPath, FileName));

	if (UTexture2D* ExistingTexture = LoadObject<UTexture2D>(nullptr, *PackageName))
	{
		return ExistingTexture;
	}

	UE_LOG(LogURLabEditor, Log, TEXT("Importing texture from: %s to %s"), *SourcePath, *DestinationPath);

	TArray<uint8> FileData;
	if (!FFileHelper::LoadFileToArray(FileData, *SourcePath))
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Failed to load texture file: %s"), *SourcePath);
		return nullptr;
	}

	const FString Extension = FPaths::GetExtension(SourcePath).ToLower();
	EImageFormat ImageFormat = EImageFormat::Invalid;

	if (Extension == TEXT("png"))
	{
		ImageFormat = EImageFormat::PNG;
	}
	else if (Extension == TEXT("jpg") || Extension == TEXT("jpeg"))
	{
		ImageFormat = EImageFormat::JPEG;
	}
	else if (Extension == TEXT("tga"))
	{
		ImageFormat = EImageFormat::TGA;
	}
	else if (Extension == TEXT("bmp"))
	{
		ImageFormat = EImageFormat::BMP;
	}
	else
	{
		UE_LOG(LogURLabEditor, Warning, TEXT("Unsupported texture format: %s"), *Extension);
		return nullptr;
	}

	IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(FName("ImageWrapper"));
	TSharedPtr<IImageWrapper> ImageWrapper = ImageWrapperModule.CreateImageWrapper(ImageFormat);

	if (!ImageWrapper.IsValid() || !ImageWrapper->SetCompressed(FileData.GetData(), FileData.Num()))
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Failed to decode texture: %s"), *SourcePath);
		return nullptr;
	}

	TArray<uint8> RawData;
	if (!ImageWrapper->GetRaw(ERGBFormat::BGRA, 8, RawData))
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Failed to get raw texture data: %s"), *SourcePath);
		return nullptr;
	}

	UPackage* Package = CreatePackage(*PackageName);
	Package->FullyLoad();

	UTexture2D* NewTexture = NewObject<UTexture2D>(Package, FName(*FileName), RF_Public | RF_Standalone);

	NewTexture->Source.Init(ImageWrapper->GetWidth(), ImageWrapper->GetHeight(), 1, 1, TSF_BGRA8, RawData.GetData());

	// A base-colour image is authored in sRGB and a data map -- roughness,
	// metallic, occlusion, ORM -- is not. Which one this is comes from the
	// spec's own `colorspace`, not from a guess about the file.
	NewTexture->SRGB = bSrgb;
	NewTexture->CompressionSettings = TextureCompressionSettings::TC_Default;
	NewTexture->MipGenSettings = TextureMipGenSettings::TMGS_FromTextureGroup;
	NewTexture->UpdateResource();

	Package->MarkPackageDirty();
	FAssetRegistryModule::AssetCreated(NewTexture);

	const FString PackageFileName = FPackageName::LongPackageNameToFilename(PackageName, FPackageName::GetAssetPackageExtension());
	FSavePackageArgs SaveArgs;
	SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
	SaveArgs.SaveFlags = SAVE_NoError;
	UPackage::SavePackage(Package, NewTexture, *PackageFileName, SaveArgs);

	UE_LOG(LogURLabEditor, Log, TEXT("Successfully imported texture: %s"), *FileName);
	return NewTexture;
}

} // namespace urlab::editor
