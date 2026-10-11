// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MuJoCo/Spec/MjSpecBuild.h"

#if URLAB_MJ_GEN

#include "MjReservedNames.h"
#include "MjSpecNodes.h"
#include "MuJoCo/Elements/MjModelAsset.h"
#include "MuJoCo/Gen/MjDispatch.gen.h"
#include "MuJoCo/Spec/MjNodeComponent.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

namespace urlab::spec
{
namespace
{
struct FMountedAsset
{
	FString Name;
	TArray<uint8> Bytes;
};

class FParseAssetSink final : public IMjAssetSink
{
public:
	TArray<FMountedAsset> Mounted;
	void OnMesh(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Add(Request, Bytes); }
	void OnTexture(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Add(Request, Bytes); }
	void OnHeightField(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Add(Request, Bytes); }
	void OnSkin(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Add(Request, Bytes); }
	void OnModel(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Add(Request, Bytes); }
	void OnDependency(const FMjAssetRequest& Request, const TArray<uint8>& Bytes) override { Add(Request, Bytes); }

private:
	void Add(const FMjAssetRequest& Request, const TArray<uint8>& Bytes)
	{
		if (!Request.VfsName.IsEmpty() && !Bytes.IsEmpty())
		{
			for (const FMountedAsset& Existing : Mounted)
			{
				if (Existing.Name == Request.VfsName)
				{
					return; // The sink intentionally shares one mount for one source file.
				}
			}
			Mounted.Add({Request.VfsName, Bytes});
		}
	}
};

struct FBindingName
{
	const UMjNodeComponent* Node = nullptr;
	mjtObj Type = mjOBJ_UNKNOWN;
	FString Name;
};

void Report(TArray<FMjSpecDiagnostic>& Diags, const FString& Message)
{
	FMjSpecDiagnostic& Diag = Diags.AddDefaulted_GetRef();
	Diag.Message = Message;
}
} // namespace

FMjBuiltSpec::FMjBuiltSpec(FMjBuiltSpec&& Other)
	: Spec(Other.Spec), ElementFor(MoveTemp(Other.ElementFor))
{
	Other.Spec = nullptr;
}

FMjBuiltSpec& FMjBuiltSpec::operator=(FMjBuiltSpec&& Other)
{
	if (this != &Other)
	{
		if (Spec != nullptr) mj_deleteSpec(Spec);
		Spec = Other.Spec;
		ElementFor = MoveTemp(Other.ElementFor);
		Other.Spec = nullptr;
	}
	return *this;
}

FMjBuiltSpec::~FMjBuiltSpec()
{
	if (Spec != nullptr) mj_deleteSpec(Spec);
}

FMjBuiltSpec BuildSpec(const FSpecRef& Root, TArray<FMjSpecDiagnostic>& OutDiags,
	const FString& VfsPrefix)
{
	FMjBuiltSpec Built;
	if (!Root.IsValid())
	{
		Report(OutDiags, TEXT("the spec handle names no component tree"));
		return Built;
	}

	// The same pass that the scene compiler uses later decides the mount names.
	FParseAssetSink AssetCollector;
	FMjAssetSink AssetSink(AssetCollector);
	AssetSink.VfsPrefix = VfsPrefix;
	AssetSink.bRequirePersistentBytes = true;
	AssetSink.Collect(Root);
	for (const FMjAssetRequest& Request : AssetSink.GetRequests())
	{
		if (!Request.bMissing) continue;
		Report(OutDiags, FString::Printf(TEXT("asset '%s' has no persisted compiler bytes"),
			*Request.File));
	}
	if (MjAnyError(OutDiags)) return Built;

	FString Xml;
	TArray<FBindingName> Manifest;
	{
		const FMjReservedNames Reserved(Root);
		TArray<FMjXmlAttributeOverride> Overrides;
		const FMjSpecNodes Nodes = MjSpecNodesOf(Root);
		for (const UMjNodeComponent* Node : Nodes.Nodes)
		{
			psm::ElementType Type{};
			if (Node != nullptr && gen::ElementTypeOfNode(*Node, Type) && Type == psm::ElementType::Compiler)
			{
				Overrides.Add({Node->Serial, TEXT("meshdir"), FString()});
				Overrides.Add({Node->Serial, TEXT("texturedir"), FString()});
			}
			if (Node != nullptr) MjAddSelectedMeshCompileOverrides(*Node, Overrides);
		}
		for (const FMjAssetRequest& Request : AssetSink.GetRequests())
		{
			if (Request.Element == nullptr || Request.Attribute.IsEmpty() || Request.File.IsEmpty() || Request.VfsName.IsEmpty()) continue;
			if (!Request.Element->MjName.IsSet() || Request.Element->MjName.GetValue().IsEmpty())
			{
				Overrides.Add({Request.Element->Serial, TEXT("name"), Request.Name});
			}
			Overrides.Add({Request.Element->Serial, Request.Attribute, Request.VfsName});
		}
		for (const UMjNodeComponent* Node : Nodes.Nodes)
		{
			psm::ElementType ElementType{};
			if (Node == nullptr || Nodes.Unnamable.Contains(Node)
				|| !gen::ElementTypeOfNode(*Node, ElementType))
			{
				continue;
			}
			const mjtObj Type = static_cast<mjtObj>(MjBindableObjType(ElementType));
			FString Name = Node->MjName.IsSet() ? Node->MjName.GetValue() : FString();
			if (Name.IsEmpty()) Name = MjAssetElementName(*Node);
			if (Type != mjOBJ_UNKNOWN && !Name.IsEmpty())
			{
				Manifest.Add({Node, Type, Name});
			}
		}
		Xml = Root.WriteMjcf(&OutDiags, &Overrides);
	}
	if (Xml.IsEmpty() || MjAnyError(OutDiags))
	{
		if (Xml.IsEmpty() && !MjAnyError(OutDiags))
		{
			Report(OutDiags, TEXT("MJCF writer returned an empty document"));
		}
		return Built;
	}

	mjVFS Vfs;
	mj_defaultVFS(&Vfs);
	for (const FMountedAsset& Asset : AssetCollector.Mounted)
	{
		if (mj_addBufferVFS(&Vfs, TCHAR_TO_UTF8(*Asset.Name), Asset.Bytes.GetData(), Asset.Bytes.Num()) != 0)
		{
			Report(OutDiags, FString::Printf(TEXT("could not mount asset '%s' for MJCF parse"), *Asset.Name));
			mj_deleteVFS(&Vfs);
			return Built;
		}
	}
	char Error[1024] = {};
	const FTCHARToUTF8 Utf8(*Xml);
	Built.Spec = mj_parseXMLString(Utf8.Get(), &Vfs, Error, sizeof(Error));
	mj_deleteVFS(&Vfs);
	if (Built.Spec == nullptr)
	{
		Report(OutDiags, FString::Printf(TEXT("MJCF parse failed: %s"), UTF8_TO_TCHAR(Error)));
		return Built;
	}

	// worldbody has no XML name; it denotes the parser's existing world body.
	const FMjSpecNodes Nodes = MjSpecNodesOf(Root);
	for (const UMjNodeComponent* Node : Nodes.Nodes)
	{
		psm::ElementType Type{};
		if (Node != nullptr && Nodes.Unnamable.Contains(Node)
			&& gen::ElementTypeOfNode(*Node, Type) && Type == psm::ElementType::Body)
		{
			if (mjsBody* World = mjs_findBody(Built.Spec, "world"))
			{
				Built.ElementFor.Add(Node, World->element);
			}
		}
	}
	for (const FBindingName& Entry : Manifest)
	{
		const FTCHARToUTF8 Name(*Entry.Name);
		mjsElement* Element = mjs_findElement(Built.Spec, Entry.Type, Name.Get());
		if (Element == nullptr)
		{
			Report(OutDiags, FString::Printf(TEXT("MJCF parse lost binding for '%s'"), *Entry.Name));
			Built = FMjBuiltSpec();
			return Built;
		}
		Built.ElementFor.Add(Entry.Node, Element);
	}
	return Built;
}

} // namespace urlab::spec

#endif // URLAB_MJ_GEN
