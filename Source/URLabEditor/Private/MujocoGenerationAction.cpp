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

#include "Logging/MessageLog.h"
#include "Logging/TokenizedMessage.h"

#include "MujocoMeshImporter.h"
#include "URLabEditorLogging.h"

namespace
{

/**
 * The editor's Messages panel, which is where an import failure has to land.
 *
 * The output log is a firehose nobody is watching during an import, so a model
 * that failed to read looked exactly like one that worked until the user tried
 * to use it. `Notify` raises the panel; the log lines stay where they were,
 * because the log is still the thing a bug report gets pasted from.
 */
const FName MjMessageLogName(TEXT("URLab"));

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
		UActorComponent* const Template = Node->ComponentTemplate;
		Scs->RemoveNode(Node);

		// A removed template keeps its name in the Blueprint until garbage
		// collection takes it, and the naming pass declines any name something
		// else still holds. That is how a reimport used to come back with every
		// component suffixed `_1`, and the one after it `_2`. Moving the old
		// template aside hands the name back to the read that follows.
		if (Template != nullptr)
		{
			Template->Rename(nullptr, GetTransientPackage(),
				REN_DontCreateRedirectors | REN_DoNotDirty | REN_NonTransactional);
		}
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
	FMessageLog MessageLog(MjMessageLogName);

	for (const FMjSpecDiagnostic& Diagnostic : Result.Warnings)
	{
		UE_LOG(LogURLabEditor, Warning, TEXT("%s"), *Diagnostic.ToString());
		MessageLog.Warning(FText::FromString(Diagnostic.ToString()));
	}
	for (const FMjSpecDiagnostic& Diagnostic : Result.Errors)
	{
		UE_LOG(LogURLabEditor, Error, TEXT("%s"), *Diagnostic.ToString());
		MessageLog.Error(FText::FromString(Diagnostic.ToString()));
	}
	if (!Result.IsOk())
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Failed to read MJCF '%s'"), *Filename);
		// Worded to match the log line above: the two are the same statement in
		// two places, and searching for one should find the other.
		MessageLog.Error(FText::Format(
			NSLOCTEXT("URLab", "ReadFailed", "Failed to read MJCF '{0}'."),
			FText::FromString(Filename)));
		// Forced, because the default filter would let a run whose only entries
		// are the ones just written stay silent.
		MessageLog.Notify(NSLOCTEXT("URLab", "ReadFailedToast", "MuJoCo model could not be read"),
			EMessageSeverity::Error, /*bForce=*/true);
	}
}

} // namespace

UMujocoGenerationAction::UMujocoGenerationAction()
{
	SupportedClasses.Add(UBlueprint::StaticClass());
}

bool UMujocoGenerationAction::GenerateForSelectedBlueprint(UBlueprint* Blueprint)
{
	if (Blueprint == nullptr || Blueprint->GeneratedClass == nullptr
		|| !Blueprint->GeneratedClass->IsChildOf(AMjArticulation::StaticClass()))
	{
		return false;
	}

	const AMjArticulation* Cdo = Cast<AMjArticulation>(Blueprint->GeneratedClass->GetDefaultObject());
	if (Cdo == nullptr || Cdo->MuJoCoXMLFile.FilePath.IsEmpty())
	{
		UE_LOG(LogURLabEditor, Error, TEXT("No XML File Path set in Blueprint Defaults for %s"), *Blueprint->GetName());
		FMessageLog MessageLog(MjMessageLogName);
		MessageLog.Error(FText::Format(
			NSLOCTEXT("URLab", "NoXmlPath", "No XML File Path set in Blueprint Defaults for {0}"),
			FText::FromString(Blueprint->GetName())));
		MessageLog.Notify(NSLOCTEXT("URLab", "NoXmlPathToast", "MuJoCo Blueprint has no XML file path set"),
			EMessageSeverity::Error, /*bForce=*/true);
		return false;
	}

	return GenerateForBlueprint(Blueprint, Cdo->MuJoCoXMLFile.FilePath);
}

void UMujocoGenerationAction::GenerateMuJoCoComponents()
{
	UE_LOG(LogURLabEditor, Log, TEXT("Generating MuJoCo model components"));

	for (UObject* Asset : UEditorUtilityLibrary::GetSelectedAssets())
	{
		GenerateForSelectedBlueprint(Cast<UBlueprint>(Asset));
	}
}

bool UMujocoGenerationAction::GenerateForBlueprint(UBlueprint* Blueprint, const FString& XmlPath,
	const FMjDocParseOptions& Options)
{
	if (Blueprint == nullptr || XmlPath.IsEmpty())
	{
		return false;
	}

	FString Xml;
	if (!FFileHelper::LoadFileToString(Xml, *XmlPath))
	{
		UE_LOG(LogURLabEditor, Error, TEXT("Failed to read MJCF file: %s"), *XmlPath);
		FMessageLog MessageLog(MjMessageLogName);
		MessageLog.Error(FText::Format(
			NSLOCTEXT("URLab", "FileUnreadable", "Failed to read MJCF file: {0}"),
			FText::FromString(XmlPath)));
		MessageLog.Notify(NSLOCTEXT("URLab", "FileUnreadableToast", "MuJoCo model file could not be opened"),
			EMessageSeverity::Error, /*bForce=*/true);
		return false;
	}

	return GenerateFromXml(Blueprint, Xml, XmlPath, Options);
}

bool UMujocoGenerationAction::GenerateFromXml(UBlueprint* Blueprint, const FString& Xml, const FString& Filename,
	const FMjDocParseOptions& Options)
{
	if (Blueprint == nullptr)
	{
		return false;
	}

	ClearSpec(*Blueprint);

	const FMjSpecParseResult Result = MjParseIntoBlueprint(*Blueprint, Xml, Filename, Options);
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
