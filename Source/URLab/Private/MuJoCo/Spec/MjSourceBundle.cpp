// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MuJoCo/Spec/MjSourceBundle.h"

#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/SecureHash.h"
#include "Serialization/MemoryWriter.h"
#include "MuJoCo/Elements/MjMesh.h"
#include "MuJoCo/Elements/MjTexture.h"
#include "MuJoCo/Elements/MjModelAsset.h"
#include "MuJoCo/Elements/MjFlexcomp.h"
#include "MuJoCo/Gen/Elements/Options/MjCompiler.gen.h"
#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjModelAssetResources.h"
#include "MuJoCo/Spec/MjSpecRef.h"

#if URLAB_MJ_GEN
#include "MjSpecNodes.h"
#include "MuJoCo/Gen/MjDispatch.gen.h"
#include "MuJoCo/Gen/MjReflect.gen.h"
#include "Engine/Texture2D.h"
#include "ImageCore.h"
#include "MeshDescription.h"
#endif

namespace
{
#if URLAB_MJ_GEN
void Report(TArray<FMjSpecDiagnostic>& Diagnostics, const FString& Message)
{
	FMjSpecDiagnostic& Diagnostic = Diagnostics.AddDefaulted_GetRef();
	Diagnostic.Message = Message;
}

FString StableResourceName(const UMjNodeComponent& Node, const TCHAR* Kind, const TCHAR* Extension)
{
	return FString::Printf(TEXT("__urlab_%s_%llu%s"), Kind, Node.Serial, Extension);
}

FString AssetFingerprint(const UMjNodeComponent& Node)
{
#if WITH_EDITOR
	const UStaticMesh* SourceMesh = nullptr;
	if (const UMjMesh* Mesh = Cast<UMjMesh>(&Node)) SourceMesh = Mesh->MeshAsset;
	else if (const UMjFlexcomp* Flexcomp = Cast<UMjFlexcomp>(&Node)) SourceMesh = Flexcomp->MeshAsset;
	if (SourceMesh != nullptr)
	{
		FMeshDescription* Description = SourceMesh->GetMeshDescription(0);
		if (Description == nullptr) return FString();
		TArray<uint8> Serialized;
		FMemoryWriter Writer(Serialized);
		Description->Serialize(Writer);
		return FSHA1::HashBuffer(Serialized.GetData(), Serialized.Num()).ToString();
	}
#endif
#if WITH_EDITORONLY_DATA
	if (const UMjTexture* Texture = Cast<UMjTexture>(&Node))
	{
		if (Texture->TextureAsset == nullptr) return FString();
		FImage Image;
		if (!Texture->TextureAsset->Source.GetMipImage(Image, 0)) return FString();
		return FSHA1::HashBuffer(Image.RawData.GetData(), Image.RawData.Num()).ToString();
	}
#endif
	return FString();
}

bool ExportGeneratedAsset(UMjNodeComponent& Node, const FString& Attribute,
	const FString& GeneratedFile, const FString& GeneratedName, FMjPreparedAssetResource& Out,
	FString& OutError)
{
	const FString TempDir = FPaths::Combine(FPaths::ProjectIntermediateDir(),
		TEXT("MjGeneratedAssets"), FGuid::NewGuid().ToString(EGuidFormats::Digits));
	IFileManager::Get().MakeDirectory(*TempDir, true);
	FString ExportedFile;
	FSoftObjectPath AssetPath;
	FString Fingerprint;
	bool bExported = false;
	if (UMjMesh* Mesh = Cast<UMjMesh>(&Node))
	{
		UMjMesh* Copy = DuplicateObject<UMjMesh>(Mesh, GetTransientPackage());
		bExported = Copy != nullptr && Copy->DumpAssetToFile(TempDir, GeneratedName);
		if (bExported) ExportedFile = Copy->GetFile();
		if (Mesh->MeshAsset)
		{
			AssetPath = FSoftObjectPath(Mesh->MeshAsset.Get());
			Fingerprint = AssetFingerprint(Node);
		}
	}
	else if (UMjFlexcomp* Flexcomp = Cast<UMjFlexcomp>(&Node))
	{
		UMjMesh* Exporter = NewObject<UMjMesh>(GetTransientPackage());
		Exporter->MeshAsset = Flexcomp->MeshAsset;
		Exporter->MjName = Flexcomp->MjName;
		bExported = Exporter->DumpAssetToFile(TempDir, GeneratedName);
		if (bExported) ExportedFile = Exporter->GetFile();
		if (Flexcomp->MeshAsset)
		{
			AssetPath = FSoftObjectPath(Flexcomp->MeshAsset.Get());
			Fingerprint = AssetFingerprint(Node);
		}
	}
	else if (UMjTexture* Texture = Cast<UMjTexture>(&Node))
	{
		UMjTexture* Copy = DuplicateObject<UMjTexture>(Texture, GetTransientPackage());
		bExported = Copy != nullptr && Copy->DumpAssetToFile(TempDir, GeneratedName);
		if (bExported) ExportedFile = Copy->GetFile();
		if (Texture->TextureAsset)
		{
			AssetPath = FSoftObjectPath(Texture->TextureAsset.Get());
			Fingerprint = AssetFingerprint(Node);
		}
	}
	if (!bExported || ExportedFile.IsEmpty() || Fingerprint.IsEmpty() ||
		!FFileHelper::LoadFileToArray(Out.Bytes, *FPaths::Combine(TempDir, ExportedFile)))
	{
		OutError = FString::Printf(TEXT("asset '%s' did not produce compiler data from its current UE source"),
			*Node.GetName());
		IFileManager::Get().DeleteDirectory(*TempDir, false, true);
		return false;
	}
	IFileManager::Get().DeleteDirectory(*TempDir, false, true);
	if (UMjMesh* Mesh = Cast<UMjMesh>(&Node)) Mesh->File = GeneratedFile;
	else if (UMjTexture* Texture = Cast<UMjTexture>(&Node)) Texture->File = GeneratedFile;
	else if (UMjFlexcomp* Flexcomp = Cast<UMjFlexcomp>(&Node)) Flexcomp->File = GeneratedFile;
	Out.Attribute = Attribute;
	Out.File = GeneratedFile;
	Out.VfsName = GeneratedFile;
	Out.AssetPath = AssetPath;
	Out.Fingerprint = Fingerprint;
	return true;
}

bool HasPreparedRow(const UMjNodeComponent& Node, const FString& Attribute, const FString& File)
{
	return Node.PreparedAssetResources.ContainsByPredicate([&](const FMjPreparedAssetResource& Resource)
	{
		return Resource.Attribute == Attribute && Resource.File == File && !Resource.Bytes.IsEmpty();
	});
}
#endif
}

void MjFinalizeImportedAuthoredResources(const FSpecRef& Spec)
{
#if URLAB_MJ_GEN
	for (UMjNodeComponent* Node : urlab::spec::MjSpecNodesOf(Spec).Nodes)
	{
		if (Node == nullptr) continue;
		Node->SourceFile.Reset();
		if (UMjCompiler* Compiler = Cast<UMjCompiler>(Node))
		{
			// Generated resource filenames already name the shared VFS namespace;
			// imported host paths and path-stripping directives no longer apply.
			Compiler->ClearAssetdir();
			Compiler->ClearMeshdir();
			Compiler->ClearTexturedir();
			Compiler->ClearStrippath();
		}
	}
#else
	(void)Spec;
#endif
}

bool MjPrepareAssetResource(UMjNodeComponent& Node, FString& OutError)
{
#if URLAB_MJ_GEN && WITH_EDITOR
	if (UMjModelAsset* ModelAsset = Cast<UMjModelAsset>(&Node))
	{
		TArray<FMjPreparedAssetResource> Generated;
		if (!MjBuildChildModelResources(*ModelAsset, Generated, OutError)) return false;
		if (Generated.IsEmpty() || Generated[0].Attribute != TEXT("file") ||
			Generated[0].File.IsEmpty() || Generated[0].Bytes.IsEmpty())
		{
			OutError = FString::Printf(TEXT("nested model '%s' did not produce its generated XML resource"),
				*Node.GetName());
			return false;
		}
		ModelAsset->File = Generated[0].File;
		Node.PreparedAssetResources = MoveTemp(Generated);
		return true;
	}
	UMjMesh* const Mesh = Cast<UMjMesh>(&Node);
	UMjTexture* const Texture = Cast<UMjTexture>(&Node);
	UMjFlexcomp* const Flexcomp = Cast<UMjFlexcomp>(&Node);
	if (Mesh == nullptr && Texture == nullptr && Flexcomp == nullptr) return true;
	if (Flexcomp != nullptr && !Flexcomp->GetFile().IsEmpty() &&
		Flexcomp->GetType() != EMjFlexcompType::mesh)
	{
		OutError = FString::Printf(TEXT("file-backed flexcomp '%s' is unsupported unless type=mesh; a UE StaticMesh cannot represent this flexcomp source"),
			*Node.GetName());
		return false;
	}
	const UObject* Asset = Mesh != nullptr ? static_cast<UObject*>(Mesh->MeshAsset.Get())
		: (Texture != nullptr ? static_cast<UObject*>(Texture->TextureAsset.Get())
			: static_cast<UObject*>(Flexcomp->MeshAsset.Get()));
	if (Asset == nullptr)
	{
		const FString File = Mesh != nullptr ? Mesh->GetFile()
			: (Texture != nullptr ? Texture->GetFile() : Flexcomp->GetFile());
		if (File.IsEmpty()) return true; // Inline assets have no external payload.
		OutError = FString::Printf(TEXT("asset node '%s' names '%s' but has no authored UE asset"),
			*Node.GetName(), *File);
		return false;
	}
	const TCHAR* Kind = Mesh != nullptr ? TEXT("mesh")
		: (Texture != nullptr ? TEXT("texture") : TEXT("flexcomp"));
	const TCHAR* Extension = Texture != nullptr ? TEXT(".png") : TEXT(".obj");
	const FString GeneratedFile = StableResourceName(Node, Kind, Extension);
	const FString GeneratedName = FPaths::GetBaseFilename(GeneratedFile);
	FMjPreparedAssetResource Prepared;
	if (!ExportGeneratedAsset(Node, TEXT("file"), GeneratedFile, GeneratedName, Prepared, OutError))
		return false;
	Node.PreparedAssetResources.RemoveAll([](const FMjPreparedAssetResource& Resource)
	{
		return Resource.Attribute == TEXT("file");
	});
	Node.PreparedAssetResources.Add(MoveTemp(Prepared));
	return true;
#else
	(void)Node;
	OutError = TEXT("asset preparation requires the editor");
	return false;
#endif
}

bool MjBuildSourceBundle(const FSpecRef& Spec, FMjSourceBundle& Out,
	TArray<FMjSpecDiagnostic>& OutDiagnostics)
{
#if URLAB_MJ_GEN && WITH_EDITOR
	Out = FMjSourceBundle();
	const urlab::spec::FMjSpecNodes Nodes = urlab::spec::MjSpecNodesOf(Spec);
	for (UMjNodeComponent* Node : Nodes.Nodes)
	{
		if (Node == nullptr) continue;
		FString Error;
		if (!MjPrepareAssetResource(*Node, Error))
		{
			Report(OutDiagnostics, Error);
			return false;
		}
	}

	TArray<FMjXmlAttributeOverride> Overrides;
	for (const UMjNodeComponent* Node : Nodes.Nodes)
	{
		if (Node == nullptr) continue;
		MjAddSelectedMeshCompileOverrides(*Node, Overrides);
		ps::mjcf::ElementType Type{};
		if (!ps::ue::ElementTypeOfNode(*Node, Type)) continue;
		const ps::ue::FMjElementThunks& Fields = ps::ue::Thunks(Type);
		for (int32 FieldId = 0; FieldId < Fields.FieldCount; ++FieldId)
		{
			const ps::ue::FMjFieldSpec& Field = Fields.Fields[FieldId];
			if (!Field.bFileResource || !Fields.Has(Node, FieldId)) continue;
			FString File;
			if (!Fields.Format(Node, FieldId, File) || File.IsEmpty()) continue;
			const FMjPreparedAssetResource* Resource = Node->PreparedAssetResources.FindByPredicate(
				[&](const FMjPreparedAssetResource& Candidate)
				{
					return Candidate.Attribute == Field.XmlName && Candidate.File == File &&
						!Candidate.Bytes.IsEmpty();
				});
			if (Resource == nullptr)
			{
				Report(OutDiagnostics, FString::Printf(
					TEXT("resource '%s' on '%s' has no generated UE-authored payload"),
					*File, *Node->GetName()));
				return false;
			}
			Overrides.Add({Node->Serial, Field.XmlName, Resource->VfsName});
		}
	}
	Out.Xml = Spec.WriteMjcf(&OutDiagnostics, &Overrides);
	if (Out.Xml.IsEmpty()) return false;
	Out.RootResourceKey = TEXT("scene.xml");
	TMap<FString, int32> ResourceRows;
	for (const UMjNodeComponent* Node : Nodes.Nodes)
	{
		if (Node == nullptr) continue;
		for (const FMjPreparedAssetResource& Resource : Node->PreparedAssetResources)
		{
			if (Resource.VfsName.IsEmpty() || Resource.Bytes.IsEmpty()) continue;
			if (const int32* ExistingRow = ResourceRows.Find(Resource.VfsName))
			{
				if (Out.Resources[*ExistingRow].Bytes != Resource.Bytes)
				{
					Report(OutDiagnostics, FString::Printf(TEXT("generated resource name collision '%s' has different payloads"),
						*Resource.VfsName));
					return false;
				}
				continue; // Reusing the same child Blueprint produces one VFS row.
			}
			ResourceRows.Add(Resource.VfsName, Out.Resources.Num());
			FMjGeneratedBundleResource& Exported = Out.Resources.AddDefaulted_GetRef();
			Exported.Key = Resource.VfsName;
			Exported.Bytes = Resource.Bytes;
		}
	}
	return true;
#else
	(void)Spec; (void)Out;
	FMjSpecDiagnostic& Diagnostic = OutDiagnostics.AddDefaulted_GetRef();
	Diagnostic.Message = TEXT("generated asset bundle export requires the editor and MuJoCo authoring");
	return false;
#endif
}

bool FMjSourceBundle::WriteToDirectory(const FString& Directory, FString& OutXmlPath,
	FString& OutError) const
{
	if (Xml.IsEmpty() || RootResourceKey.IsEmpty())
	{
		OutError = TEXT("the generated asset bundle is empty");
		return false;
	}
	const FString RootDir = FPaths::ConvertRelativePathToFull(Directory);
	TSet<FString> Destinations;
	for (const FMjGeneratedBundleResource& Resource : Resources)
	{
		FString Relative = Resource.Key;
		FPaths::NormalizeFilename(Relative);
		if (Relative.IsEmpty() || !FPaths::IsRelative(Relative) || Relative.Contains(TEXT("..")) ||
			Destinations.Contains(Relative.ToLower()))
		{
			OutError = FString::Printf(TEXT("generated resource name collision '%s'"), *Resource.Key);
			return false;
		}
		Destinations.Add(Relative.ToLower());
		const FString Dest = FPaths::Combine(RootDir, Relative);
		if (!IFileManager::Get().MakeDirectory(*FPaths::GetPath(Dest), true) ||
			!FFileHelper::SaveArrayToFile(Resource.Bytes, *Dest))
		{
			OutError = FString::Printf(TEXT("could not write generated resource '%s'"), *Dest);
			return false;
		}
	}
	OutXmlPath = FPaths::Combine(RootDir, TEXT("scene.xml"));
	FTCHARToUTF8 XmlUtf8(*Xml);
	TArray<uint8> XmlBytes;
	XmlBytes.Append(reinterpret_cast<const uint8*>(XmlUtf8.Get()), XmlUtf8.Length());
	if (!IFileManager::Get().MakeDirectory(*RootDir, true) ||
		!FFileHelper::SaveArrayToFile(XmlBytes, *OutXmlPath))
	{
		OutError = FString::Printf(TEXT("could not write generated MJCF '%s'"), *OutXmlPath);
		return false;
	}
	return true;
}
