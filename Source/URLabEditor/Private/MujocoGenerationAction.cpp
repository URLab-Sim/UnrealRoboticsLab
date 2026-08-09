// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MujocoGenerationAction.h"

#include "EditorUtilityLibrary.h"
#include "Engine/Blueprint.h"
#include "Engine/StaticMesh.h"
#include "Engine/Texture2D.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "PackageTools.h"

#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Spec/MjAssetSink.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjAssetResolve.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Elements/MjMesh.h"
#include "MuJoCo/Elements/MjTexture.h"

#include "MujocoMeshImporter.h"
#include "URLabEditorLogging.h"

namespace
{

/**
 * Take `Blueprint`'s existing spec out of its construction script.
 *
 * A parse adds a spec root, it does not replace one, so re-reading a
 * Blueprint that already holds a spec would leave two roots side by side --
 * and every question asked of that Blueprint afterwards is answered by whichever
 * root is reached first, which is the stale one.
 *
 * Only elements go. The Blueprint's own scene root and anything the user added
 * beside the spec are left alone.
 */
void ClearSpec(UBlueprint& Blueprint)
{
	USimpleConstructionScript* Scs = Blueprint.SimpleConstructionScript;
	if (Scs == nullptr)
	{
		return;
	}

	TArray<USCS_Node*> Roots;
	for (USCS_Node* Node : Scs->GetRootNodes())
	{
		if (Node != nullptr && Cast<UMjNodeComponent>(Node->ComponentTemplate) != nullptr)
		{
			Roots.Add(Node);
		}
	}

	// Leaves first. Removing a root that still holds children strands them in
	// the script's flat node list, where nothing can reach them but the variable
	// name they still occupy.
	auto RemoveSubtree = [Scs](USCS_Node* Node, auto& Self) -> void {
		for (USCS_Node* Child : TArray<USCS_Node*>(Node->GetChildNodes()))
		{
			if (Child != nullptr)
			{
				Self(Child, Self);
			}
		}
		Scs->RemoveNode(Node);
	};

	for (USCS_Node* Root : Roots)
	{
		RemoveSubtree(Root, RemoveSubtree);
	}
}

/**
 * The editor's end of the spec's asset pass.
 *
 * The requests arrive already resolved -- meshdir and texturedir folded, each
 * path taken relative to the file its own element was read from -- so all that
 * is left here is the Unreal half: a mesh file becomes a UStaticMesh and an
 * image becomes a UTexture2D, both saved into the model's own import folder.
 *
 * The bytes are ignored in favour of the path, because the importers want a
 * file: the mesh one looks for a better-formatted sibling of the same name
 * before it commits to the file the spec named, and Unreal's asset tools
 * read from disk regardless.
 *
 * What comes back is written onto the element that asked for it. Where an asset
 * lands is Unreal's decision -- the FBX factory writes `Meshes/<name>` and
 * Interchange writes `Meshes/<name>/StaticMeshes/<name>`, and which one runs
 * depends on whether a GLB was prepared beside the source -- so the importer's
 * own answer is the only reliable one, and it is kept rather than re-derived.
 */
class FMjImportAssetSink final : public IMjAssetSink
{
public:
	explicit FMjImportAssetSink(const FSpecRef& Spec)
		: DestinationPath(MjImportedAssetPath(Spec))
		, MeshPath(MjImportedMeshPath(Spec))
	{
	}

	void OnMesh(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override
	{
		if (Request.ResolvedPath.IsEmpty())
		{
			// Inline geometry: the element carries its own vertices and faces.
			return;
		}
		UStaticMesh* Asset = urlab::editor::ImportMeshAsset(Request.ResolvedPath, MeshPath, Request.Name);
		if (UMjMesh* Element = Cast<UMjMesh>(Request.Element))
		{
			Element->MeshAsset = Asset;
			Element->FileAsset = FSoftObjectPath(Asset);
		}
	}

	void OnTexture(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override
	{
		if (Request.ResolvedPath.IsEmpty())
		{
			// A procedural texture (`builtin="checker"`), which MuJoCo generates
			// at compile time and no file backs. Nothing here can produce those
			// pixels, so a material that names one keeps its flat colour.
			return;
		}
		// Named after the element, not the file: a `<layer>` names a texture and
		// the preview looks the asset up by that name.
		UTexture2D* Asset =
			urlab::editor::ImportTextureAsset(Request.ResolvedPath, DestinationPath, Request.Name, IsSrgb(Request));
		if (UMjTexture* Element = Cast<UMjTexture>(Request.Element))
		{
			Element->TextureAsset = Asset;
			Element->FileAsset = FSoftObjectPath(Asset);
		}
	}

	void OnHeightField(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override
	{
		// MuJoCo reads a height field's file itself at compile time, and there is
		// no Unreal asset that corresponds to one.
	}

	void OnMissing(const FMjAssetRequest& Request) override
	{
		UE_LOG(LogURLabEditor, Warning, TEXT("Asset '%s' is named by the model but could not be read: %s"),
			*Request.Name, *Request.ResolvedPath);
	}

private:
	/**
	 * Whether the image is colour or data, per the element's `colorspace`.
	 *
	 * MJCF's `auto` means "decide from the file", and every format the importer
	 * reads carries colour, so `auto` is sRGB here. Only an explicit
	 * `colorspace="linear"` says otherwise, which is how a roughness or ORM map
	 * declares itself.
	 */
	static bool IsSrgb(const FMjAssetRequest& Request)
	{
		if (const UMjTexture* Texture = Cast<UMjTexture>(Request.Element))
		{
			return Texture->GetColorspace() != EMjColorSpace::linear;
		}
		return true;
	}

	FString DestinationPath;
	FString MeshPath;
};

/** Import the Unreal assets `Blueprint`'s parsed spec references. */
void ImportSpecAssets(UBlueprint& Blueprint)
{
	const FSpecRef Spec = FSpecRef::OverBlueprint(Blueprint);
	if (!Spec.IsValid())
	{
		return;
	}

	// The folder is derived from the spec rather than passed in, so the
	// import and the preview's later lookup cannot disagree about where an
	// asset went: both ask MjImportedAssetPath the same question.
	FMjImportAssetSink Importer(Spec);
	FMjAssetSink Pass(Importer);
	// The importers read from disk themselves, so the only thing reading the
	// bytes here would establish is that the file is there, which the pass
	// answers without them.
	Pass.bLoadBytes = false;
	Pass.Collect(Spec);
}

/** Report one parse's diagnostics against the file they came from. */
void LogDiagnostics(const FMjSpecParseResult& Result, const FString& Filename)
{
	for (const FMjSpecDiagnostic& Diagnostic : Result.Warnings)
	{
		UE_LOG(LogURLabEditor, Warning, TEXT("%s"), *Diagnostic.ToString());
	}
	for (const FMjSpecDiagnostic& Diagnostic : Result.Errors)
	{
		UE_LOG(LogURLabEditor, Error, TEXT("%s"), *Diagnostic.ToString());
	}
	if (!Result.IsOk())
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Failed to read MJCF '%s'"), *Filename);
	}
}

}  // namespace

UMujocoGenerationAction::UMujocoGenerationAction()
{
	SupportedClasses.Add(UBlueprint::StaticClass());
}

void UMujocoGenerationAction::GenerateMuJoCoComponents()
{
	UE_LOG(LogURLabEditor, Log, TEXT("Generating MuJoCo model components"));

	for (UObject* Asset : UEditorUtilityLibrary::GetSelectedAssets())
	{
		UBlueprint* Blueprint = Cast<UBlueprint>(Asset);
		if (Blueprint == nullptr || Blueprint->GeneratedClass == nullptr
			|| !Blueprint->GeneratedClass->IsChildOf(AMjArticulation::StaticClass()))
		{
			continue;
		}

		const AMjArticulation* Cdo = Cast<AMjArticulation>(Blueprint->GeneratedClass->GetDefaultObject());
		if (Cdo == nullptr || Cdo->MuJoCoXMLFile.FilePath.IsEmpty())
		{
			UE_LOG(LogURLabEditor, Error, TEXT("No XML File Path set in Blueprint Defaults for %s"), *Blueprint->GetName());
			continue;
		}

		GenerateForBlueprint(Blueprint, Cdo->MuJoCoXMLFile.FilePath);
	}
}

bool UMujocoGenerationAction::GenerateForBlueprint(UBlueprint* Blueprint, const FString& XmlPath)
{
	if (Blueprint == nullptr || XmlPath.IsEmpty())
	{
		return false;
	}

	FString Xml;
	if (!FFileHelper::LoadFileToString(Xml, *XmlPath))
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Failed to read MJCF file: %s"), *XmlPath);
		return false;
	}

	return GenerateFromXml(Blueprint, Xml, XmlPath);
}

bool UMujocoGenerationAction::GenerateFromXml(UBlueprint* Blueprint, const FString& Xml, const FString& Filename)
{
	if (Blueprint == nullptr)
	{
		return false;
	}

	ClearSpec(*Blueprint);

	const FMjSpecParseResult Result = MjParseIntoBlueprint(*Blueprint, Xml, Filename);
	LogDiagnostics(Result, Filename);
	if (!Result.IsOk())
	{
		return false;
	}

	ImportSpecAssets(*Blueprint);

	FKismetEditorUtilities::CompileBlueprint(Blueprint);
	return true;
}

void UMujocoGenerationAction::SetupEmptyArticulation(UBlueprint* Blueprint)
{
	if (Blueprint == nullptr)
	{
		return;
	}

	// An empty spec rather than a hand-built tree, so that a Blueprint the
	// user starts from scratch and one read from a file are the same shape from
	// the first component onwards. There is no `<worldbody>` element to create:
	// a body authored directly under the model root is a world body.
	const FString Xml = FString::Printf(TEXT("<mujoco model=\"%s\"/>"), *Blueprint->GetName());

	ClearSpec(*Blueprint);

	const FMjSpecParseResult Result = MjParseIntoBlueprint(*Blueprint, Xml, FString());
	LogDiagnostics(Result, Blueprint->GetName());

	FKismetEditorUtilities::CompileBlueprint(Blueprint);
}
