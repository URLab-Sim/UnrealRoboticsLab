// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "MuJoCo/Spec/MjAssetSink.h"

#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MjSpecNodes.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Elements/MjMesh.h"

namespace
{
const FMjPreparedAssetResource* PreparedResource(const UMjNodeComponent& Element,
	const FString& Attribute, const FString& File)
{
	for (const FMjPreparedAssetResource& Resource : Element.PreparedAssetResources)
		if (Resource.Attribute == Attribute && Resource.File == File && !Resource.Bytes.IsEmpty())
			return &Resource;
	return nullptr;
}
}

void MjAddSelectedMeshCompileOverrides(const UMjNodeComponent& Node,
	TArray<FMjXmlAttributeOverride>& OutOverrides)
{
	const UMjMesh* const Mesh = Cast<UMjMesh>(&Node);
	if (Mesh == nullptr || Mesh->MeshAsset == nullptr) return;
	for (const TCHAR* Attribute : {TEXT("vertex"), TEXT("normal"), TEXT("texcoord"), TEXT("face")})
	{
		OutOverrides.Add({Node.Serial, Attribute, FString(), true});
	}
}

namespace
{
/**
 * The directory an asset path resolves against.
 *
 * MuJoCo's ReadAttrFile first checks the root model's compiler asset directory.
 * For an element from an include, it falls back to the included file's own
 * directory with no compiler prefix (xml_util.cc::ResolveFilePath). Both
 * branches matter: build/model/scene.xml can supply marble.png while its
 * compiler says texturedir=asset and the file is beside scene.xml.
 */
FString AssetBaseDirectory(const UMjNodeComponent& Root, const UMjNodeComponent& Element,
	const FString& AssetDir, const FString& File)
{
	const FString RootFile = Root.SourceFile.IsEmpty() ? Element.SourceFile : Root.SourceFile;
	const FString RootDir = RootFile.IsEmpty() ? FString() : FPaths::GetPath(RootFile);
	const FString Primary = AssetDir.IsEmpty() ? RootDir
		: (FPaths::IsRelative(AssetDir) ? FPaths::Combine(RootDir, AssetDir) : AssetDir);
	if (File.IsEmpty() || !FPaths::IsRelative(File) ||
		RootFile.IsEmpty() || FPaths::FileExists(FPaths::Combine(Primary, File)))
	{
		return Primary;
	}
	if (!Element.SourceFile.IsEmpty() && Element.SourceFile != RootFile)
	{
		const FString IncludeDir = FPaths::GetPath(Element.SourceFile);
		if (FPaths::FileExists(FPaths::Combine(IncludeDir, File)))
		{
			return IncludeDir;
		}
	}
	return Primary;
}
} // namespace

#if URLAB_MJ_GEN

#include "MuJoCo/Spec/MjTreeAdapters.h"
#include "MuJoCo/Gen/MjDispatch.gen.h"
#include "MuJoCo/Gen/MjReflect.gen.h"

namespace
{
using namespace urlab::spec;

/** The MJCF string attribute `Attr` of `Node` when it was authored. */
bool AuthoredString(UMjNodeComponent& Node, const char* Attr, FString& Out)
{
	psm::ElementType Type{};
	if (!gen::ElementTypeOfNode(Node, Type)) return false;
	const gen::FMjElementThunks& Fields = gen::Thunks(Type);
	const FString Name = UTF8_TO_TCHAR(Attr);
	for (int32 Id = 0; Id < Fields.FieldCount; ++Id)
	{
		if (Fields.Fields[Id].Kind == gen::EMjFieldKind::String &&
			FStringView(Fields.Fields[Id].XmlName).Equals(Name, ESearchCase::CaseSensitive) &&
			Fields.Has(&Node, Id))
			return Fields.Format(&Node, Id, Out);
	}
	return false;
}

/** The same, for a caller that cannot act on the difference. */
FString StringAttribute(UMjNodeComponent& Node, const char* Attr)
{
	FString Out;
	AuthoredString(Node, Attr, Out);
	return Out;
}

/**
 * A node's children in whichever graph the spec lives in.
 *
 * A Blueprint's templates are linked only by USCS_Node::ChildNodes -- they are
 * never attached to one another -- so reading the attachment tree over an SCS
 * spec walks an empty list and the whole pass silently finds nothing.
 */
TArray<FMjOrderedChild> ChildrenOf(const FSpecRef& Spec, UMjNodeComponent& Parent)
{
#if WITH_EDITOR
	if (Spec.GetGraph() == EMjSpecGraph::Scs && Spec.GetBlueprint() != nullptr)
	{
		FMjScsScope Scope(*Spec.GetBlueprint());
		return FMjScsAdapter::OrderedChildren(Parent);
	}
#endif
	return FMjInstanceAdapter::OrderedChildren(Parent);
}

/** `Name` with an ordinal before its extension: `base.obj`, `base_2.obj`, ... */
FString NumberedName(const FString& Name, int32 Ordinal)
{
	const FString Extension = FPaths::GetExtension(Name, /*bIncludeDot=*/true);
	return FString::Printf(TEXT("%s_%d%s"), *Name.LeftChop(Extension.Len()), Ordinal, *Extension);
}

/**
 * `Desired`, made unique across the names already mounted.
 *
 * `Mounted` maps each name taken to the file behind it. An element naming a
 * file that is already mounted keeps that mount -- one file referenced twice is
 * one file -- and only a DIFFERENT file takes an ordinal. The walk is in spec
 * order, so the first claimant keeps the clean name and the answer does not
 * depend on the order a directory happened to be read in.
 *
 * Compared case-insensitively, because the basename fallback this exists to
 * keep out of the decision is.
 */
FString UniqueMountName(TMap<FString, FString>& Mounted, const FString& Desired, const FString& ResolvedPath)
{
	FString Name = Desired;
	for (int32 Ordinal = 2;; ++Ordinal)
	{
		const FString* const Taken = Mounted.Find(Name.ToLower());
		if (Taken == nullptr)
		{
			Mounted.Add(Name.ToLower(), ResolvedPath);
			return Name;
		}
		if (*Taken == ResolvedPath)
		{
			return Name;
		}
		Name = NumberedName(Desired, Ordinal);
	}
}

/**
 * The spec-level meshdir / texturedir, read off <compiler>.
 *
 * Authored fields only, which is the reader's own rule: `assetdir` stands in
 * for both and each of the two overrides it only where the document actually
 * says so. Reading them as plain values instead makes an unauthored field an
 * empty string that overrides `assetdir`, and a `<compiler>` carrying nothing
 * relevant erase what an earlier one set.
 */
void ReadAssetDirectories(const FSpecRef& Spec, UMjNodeComponent& Root,
	FString& OutAssetDir, FString& OutMeshDir, FString& OutTextureDir)
{
	for (const FMjOrderedChild& Child : ChildrenOf(Spec, Root))
	{
		psm::ElementType Type{};
		if (!gen::ElementTypeOfNode(*Child.Node, Type) || Type != psm::ElementType::Compiler)
		{
			continue;
		}
		FString AssetDir;
		if (AuthoredString(*Child.Node, "assetdir", AssetDir))
		{
			OutAssetDir = AssetDir;
			OutMeshDir = AssetDir;
			OutTextureDir = AssetDir;
		}
		FString Directory;
		if (AuthoredString(*Child.Node, "meshdir", Directory))
		{
			OutMeshDir = Directory;
		}
		if (AuthoredString(*Child.Node, "texturedir", Directory))
		{
			OutTextureDir = Directory;
		}
	}
}
} // namespace

#endif // URLAB_MJ_GEN

FString MjAssetElementName(const UMjNodeComponent& Element)
{
	if (Element.MjName.IsSet() && !Element.MjName.GetValue().IsEmpty())
	{
		return Element.MjName.GetValue();
	}
#if URLAB_MJ_GEN
	psm::ElementType Type{};
	if (gen::ElementTypeOfNode(const_cast<UMjNodeComponent&>(Element), Type))
	{
		const gen::FMjElementThunks& Fields = gen::Thunks(Type);
		for (int32 Id = 0; Id < Fields.FieldCount; ++Id)
		{
			const gen::FMjFieldSpec& Field = Fields.Fields[Id];
			if (!Field.bFileResource || !Fields.Has(&Element, Id)) continue;
			FString File;
			if (Fields.Format(&Element, Id, File) && !File.IsEmpty())
				return FPaths::GetBaseFilename(File);
		}
	}
#endif
	return FString();
}

FString MjResolveAssetPath(const UMjNodeComponent& Element, const FString& AssetDir, const FString& File)
{
	if (File.IsEmpty())
	{
		return FString();
	}
	// MuJoCo takes an absolute `file` as it stands; only a relative one goes
	// through the directories. A spec whose asset was exported back out of
	// Unreal with nowhere relative to be is the case that makes the difference.
	return FPaths::IsRelative(File)
			 ? FPaths::ConvertRelativePathToFull(FPaths::Combine(AssetBaseDirectory(Element, Element, AssetDir, File), File))
			 : File;
}

void MjReadAssetDirectories(const FSpecRef& Spec, UMjNodeComponent& Root,
	FString& OutAssetDir, FString& OutMeshDir, FString& OutTextureDir)
{
#if URLAB_MJ_GEN
	ReadAssetDirectories(Spec, Root, OutAssetDir, OutMeshDir, OutTextureDir);
#else
	(void)Spec; (void)Root; (void)OutAssetDir; (void)OutMeshDir; (void)OutTextureDir;
#endif
}

void FMjAssetSink::Collect(const FSpecRef& Spec)
{
	Requests.Reset();
#if URLAB_MJ_GEN
	UMjNodeComponent* Root = Spec.GetRoot();
	if (Root == nullptr || Sink == nullptr)
	{
		return;
	}

	FString AssetDir;
	FString MeshDir;
	FString TextureDir;
	MjReadAssetDirectories(Spec, *Root, AssetDir, MeshDir, TextureDir);

	// One entry per mount this pass hands out: the name, and the file it
	// carries. Two assets that would take one name are what this is for.
	TMap<FString, FString> Mounted;

	for (UMjNodeComponent* Element : urlab::spec::MjSpecNodesOf(Spec).Nodes)
	{
		if (Element == nullptr) continue;
		psm::ElementType Type{};
		if (!gen::ElementTypeOfNode(*Element, Type))
		{
			continue;
		}
		const bool bMesh = Type == psm::ElementType::Mesh;
		const bool bTexture = Type == psm::ElementType::Texture;
		const bool bHeightField = Type == psm::ElementType::Hfield;
		// The XML reader first consults assetdir, but mjCSkin::Compile
		// prepends meshdir before loading `.skn` (`user_mesh.cc:3141`).
		const bool bSkin = Type == psm::ElementType::Skin;
		const bool bModel = Type == psm::ElementType::ModelAsset;
		const bool bFlexcomp = Type == psm::ElementType::Flexcomp;
		if (!bMesh && !bTexture && !bHeightField && !bSkin && !bModel && !bFlexcomp)
		{
			continue;
		}

		const FString Directory = bModel ? FString()
			: (bTexture ? TextureDir : (bHeightField ? AssetDir : MeshDir));
		const gen::FMjElementThunks& Fields = gen::Thunks(Type);
		for (int32 FieldId = 0; FieldId < Fields.FieldCount; ++FieldId)
		{
			const gen::FMjFieldSpec& Field = Fields.Fields[FieldId];
			if (!Field.bFileResource || !Fields.Has(Element, FieldId)) continue;
			FString File;
			if (!Fields.Format(Element, FieldId, File) || File.IsEmpty()) continue;

			FMjAssetRequest Request;
			Request.Element = Element;
			Request.Attribute = Field.XmlName;
			Request.Name = MjAssetElementName(*Element);
			Request.File = File;
			Request.BaseDirectory = AssetBaseDirectory(*Root, *Element, Directory, File);
			Request.ResolvedPath = FPaths::IsRelative(File)
				? FPaths::ConvertRelativePathToFull(FPaths::Combine(Request.BaseDirectory, File)) : File;
			const FMjPreparedAssetResource* Prepared = PreparedResource(*Element, Request.Attribute, File);
			FString Desired = Prepared != nullptr && !Prepared->VfsName.IsEmpty()
				? Prepared->VfsName : (VfsPrefix.IsEmpty() ? File : VfsPrefix + FPaths::GetCleanFilename(File));
			if (Prepared != nullptr && !VfsPrefix.IsEmpty())
			{
				Desired = VfsPrefix + FPaths::GetCleanFilename(Prepared->VfsName);
			}
			Request.VfsName = UniqueMountName(Mounted, Desired, Request.ResolvedPath);

			TArray<uint8> Bytes;
			if (!Request.ResolvedPath.IsEmpty())
			{
				if (Prepared != nullptr)
				{
					if (bLoadBytes) Bytes = Prepared->Bytes;
					Request.bMissing = Prepared->Bytes.IsEmpty();
				}
				else if (bRequirePersistentBytes)
				{
					Request.bMissing = true;
				}
				else
				{
					Request.bMissing = !FPaths::FileExists(Request.ResolvedPath);
					if (!Request.bMissing && bLoadBytes)
						Request.bMissing = !FFileHelper::LoadFileToArray(Bytes, *Request.ResolvedPath);
				}
			}

			Requests.Add(Request);
			if (Request.bMissing) Sink->OnMissing(Request);
			else if (bMesh || bFlexcomp) Sink->OnMesh(Request, Bytes);
			else if (bTexture) Sink->OnTexture(Request, Bytes);
			else if (bSkin) Sink->OnSkin(Request, Bytes);
			else if (bModel) Sink->OnModel(Request, Bytes);
			else Sink->OnHeightField(Request, Bytes);
		}
	}

	// Child-model rows with an empty Attribute are generated VFS dependencies,
	// not authored MJCF file attributes. Mount them separately after the normal
	// element walk so every compile/export consumer sees the same closure.
	for (UMjNodeComponent* Node : urlab::spec::MjSpecNodesOf(Spec).Nodes)
	{
		if (Node == nullptr) continue;
		for (const FMjPreparedAssetResource& Resource : Node->PreparedAssetResources)
		{
			if (!Resource.Attribute.IsEmpty()) continue;

			FMjAssetRequest Request;
			Request.Element = Node;
			Request.Name = Resource.VfsName;
			Request.File = Resource.File;
			Request.VfsName = Resource.VfsName;
			Request.bMissing = Resource.Bytes.IsEmpty();
			Requests.Add(Request);
			if (Request.bMissing) Sink->OnMissing(Request);
			else Sink->OnDependency(Request, Resource.Bytes);
		}
	}
#else
	(void)Spec;
#endif
}
