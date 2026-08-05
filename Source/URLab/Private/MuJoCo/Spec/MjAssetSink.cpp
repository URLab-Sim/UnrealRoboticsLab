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
#include "MuJoCo/Spec/MjNodeComponent.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Spec/MjSpecProfile.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"

THIRD_PARTY_INCLUDES_START
#include "protospec/model_core.h"
#include "reflect.h"
THIRD_PARTY_INCLUDES_END

namespace
{
using namespace urlab::spec;

/** The MJCF string attribute `Attr` of `Node`, or empty. */
FString StringAttribute(UMjNodeComponent& Node, const char* Attr)
{
	FString Out;
	gen::DispatchByType(Node, [&](auto& Element) {
		using P = FMjInstanceProfile;
		using E = std::decay_t<decltype(Element)>;
		const int FieldId = pssdk::internal::FieldIdByName(gen::TMjElementType<E>::Value, Attr);
		if (FieldId < 0)
		{
			return;
		}
		std::string Text;
		if (pssdk::internal::GetStrField<P>(Element, FieldId, Text))
		{
			Out = gen::FMjStrPolicy::FromUtf8(Text);
		}
	});
	return Out;
}

/**
 * The directory an asset path resolves against.
 *
 * MJCF resolves an asset path relative to the model file, then through the
 * compiler's meshdir / texturedir. The element records the file it came from, so
 * an included sub-spec's assets resolve beside the include rather than
 * beside the root -- which is the whole reason provenance is per element.
 */
FString AssetBaseDirectory(UMjNodeComponent& Element, const FString& AssetDir)
{
	const FString SourceDir = Element.SourceFile.IsEmpty() ? FString() : FPaths::GetPath(Element.SourceFile);
	if (AssetDir.IsEmpty())
	{
		return SourceDir;
	}
	if (FPaths::IsRelative(AssetDir))
	{
		return FPaths::Combine(SourceDir, AssetDir);
	}
	return AssetDir;
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

/** The spec-level meshdir / texturedir, read off <compiler>. */
void ReadAssetDirectories(const FSpecRef& Spec, UMjNodeComponent& Root, FString& OutMeshDir, FString& OutTextureDir)
{
	for (const FMjOrderedChild& Child : ChildrenOf(Spec, Root))
	{
		psm::ElementType Type{};
		if (!gen::ElementTypeOfNode(*Child.Node, Type) || Type != psm::ElementType::Compiler)
		{
			continue;
		}
		const FString AssetDir = StringAttribute(*Child.Node, "assetdir");
		OutMeshDir = StringAttribute(*Child.Node, "meshdir");
		OutTextureDir = StringAttribute(*Child.Node, "texturedir");
		if (!AssetDir.IsEmpty())
		{
			// assetdir is the fallback for both, per the schema.
			if (OutMeshDir.IsEmpty())
			{
				OutMeshDir = AssetDir;
			}
			if (OutTextureDir.IsEmpty())
			{
				OutTextureDir = AssetDir;
			}
		}
	}
}
}  // namespace

#endif  // URLAB_MJ_GEN

FString MjAssetElementName(const UMjNodeComponent& Element)
{
	if (Element.MjName.IsSet() && !Element.MjName.GetValue().IsEmpty())
	{
		return Element.MjName.GetValue();
	}
#if URLAB_MJ_GEN
	const FString File = StringAttribute(const_cast<UMjNodeComponent&>(Element), "file");
	if (!File.IsEmpty())
	{
		return FPaths::GetBaseFilename(File);
	}
#endif
	return FString();
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

	FString MeshDir;
	FString TextureDir;
	ReadAssetDirectories(Spec, *Root, MeshDir, TextureDir);

	for (const FMjOrderedChild& Section : ChildrenOf(Spec, *Root))
	{
		psm::ElementType SectionType{};
		if (!gen::ElementTypeOfNode(*Section.Node, SectionType) || SectionType != psm::ElementType::Asset)
		{
			continue;
		}
		for (const FMjOrderedChild& Asset : ChildrenOf(Spec, *Section.Node))
		{
			psm::ElementType Type{};
			if (!gen::ElementTypeOfNode(*Asset.Node, Type))
			{
				continue;
			}
			const bool bMesh = Type == psm::ElementType::Mesh;
			const bool bTexture = Type == psm::ElementType::Texture;
			const bool bHeightField = Type == psm::ElementType::Hfield;
			if (!bMesh && !bTexture && !bHeightField)
			{
				continue;
			}

			FMjAssetRequest Request;
			Request.Element = Asset.Node;
			Request.Name = MjAssetElementName(*Asset.Node);
			Request.BaseDirectory = AssetBaseDirectory(*Asset.Node, bTexture ? TextureDir : MeshDir);

			const FString File = StringAttribute(*Asset.Node, "file");
			if (!File.IsEmpty())
			{
				// MuJoCo takes an absolute `file` as it stands; only a relative one
				// goes through the directories. A spec whose asset was exported
				// back out of Unreal with nowhere relative to be is the case that
				// makes the difference.
				Request.ResolvedPath = FPaths::IsRelative(File)
					? FPaths::ConvertRelativePathToFull(FPaths::Combine(Request.BaseDirectory, File))
					: File;
				Request.VfsName = VfsPrefix + FPaths::GetCleanFilename(File);
			}

			TArray<uint8> Bytes;
			if (!Request.ResolvedPath.IsEmpty() && bLoadBytes)
			{
				Request.bMissing = !FFileHelper::LoadFileToArray(Bytes, *Request.ResolvedPath);
			}

			Requests.Add(Request);
			if (Request.bMissing)
			{
				Sink->OnMissing(Request);
			}
			else if (bMesh)
			{
				Sink->OnMesh(Request, Bytes);
			}
			else if (bTexture)
			{
				Sink->OnTexture(Request, Bytes);
			}
			else
			{
				Sink->OnHeightField(Request, Bytes);
			}
		}
	}
#else
	(void)Spec;
#endif
}
