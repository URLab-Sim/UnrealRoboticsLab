// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "MjUeXml.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Gen/MjDispatch.gen.h"
#include "MuJoCo/Gen/MjReflect.gen.h"
#include "MuJoCo/Spec/MjFieldText.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjNodeFactories.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"
#include "Utils/URLabLogging.h"

#include "GameFramework/Actor.h"
#if WITH_EDITOR
#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#endif

THIRD_PARTY_INCLUDES_START
#include <filesystem>
#include "include.h"
#include "resolve.h"
#include "tinyxml2.h"
THIRD_PARTY_INCLUDES_END

namespace urlab::spec::io
{
namespace
{
using ps::mjcf::ElementType;

FString FromUtf8(const char* Text) { return UTF8_TO_TCHAR(Text != nullptr ? Text : ""); }
std::string ToUtf8(const FString& Text)
{
	FTCHARToUTF8 Converted(*Text);
	return std::string(Converted.Get(), Converted.Length());
}

struct FPendingOrientation
{
	UMjNodeComponent* Node = nullptr;
	int32 FieldId = -1;
	ps::core::OrientKind Kind = ps::core::OrientKind::Quat;
	double Raw[6]{};
};

class FReader
{
public:
	FReader(EMjSpecGraph InGraph, const FString& InFilename)
		: Graph(InGraph), Filename(InFilename) {}

	FMjSpecParseResult Read(const FString& Text, const FMjDocParseOptions& Options)
	{
		const std::string Bytes = ToUtf8(Text);
		if (Xml.Parse(Bytes.data(), Bytes.size()) != tinyxml2::XML_SUCCESS)
		{
			Error(nullptr, FString::Printf(TEXT("invalid XML: %s"), *FromUtf8(Xml.ErrorStr())));
			return MoveTemp(Result);
		}
		const std::string File = ToUtf8(Filename);
		const std::string Directory = std::filesystem::path(File).parent_path().string();
		ps::mjcf::io::IncludeResult Includes =
			ps::mjcf::io::ExpandIncludes(Xml, Directory, File, Options.bAllowExternalIncludes);
		Provenance = MoveTemp(Includes.provenance);
		for (const ps::Diagnostic& Finding : Includes.errors)
		{
			FMjSpecDiagnostic& Diagnostic = Result.Errors.AddDefaulted_GetRef();
			Diagnostic.Message = FromUtf8(Finding.message.c_str());
			Diagnostic.File = FromUtf8(Finding.loc.file.c_str());
			Diagnostic.Line = Finding.loc.line;
		}
		if (!Result.Errors.IsEmpty()) return MoveTemp(Result);
		auto* RootXml = Xml.RootElement();
		if (RootXml == nullptr || FCStringAnsi::Strcmp(RootXml->Name(), "mujoco") != 0)
		{
			Error(RootXml, TEXT("the root element must be <mujoco>"));
			return MoveTemp(Result);
		}
		ReadElement(*RootXml, ElementType::Model, nullptr, 1);
		if (Result.Errors.IsEmpty()) ResolveOrientations();
		if (!Result.Errors.IsEmpty()) Result.Root = nullptr;
		return MoveTemp(Result);
	}

private:
	void Error(const tinyxml2::XMLElement* XmlElement, const FString& Message, bool bUnsupported = false)
	{
		FMjSpecDiagnostic& Diagnostic = Result.Errors.AddDefaulted_GetRef();
		Diagnostic.Message = Message;
		Diagnostic.bUnsupportedElement = bUnsupported;
		if (XmlElement != nullptr)
		{
			const auto Found = Provenance.find(XmlElement);
			Diagnostic.File = Found == Provenance.end() ? Filename : FromUtf8(Found->second.file.c_str());
			Diagnostic.Line = Found == Provenance.end() ? XmlElement->GetLineNum() : Found->second.line;
		}
	}

	UMjNodeComponent* Construct(ElementType Type, UMjNodeComponent* Parent)
	{
		UClass* Generated = ps::ue::ClassForElement(Type);
		if (Generated == nullptr) return nullptr;
		UClass* Class = MjElementClass(Type, Generated);
#if WITH_EDITOR
		if (Graph == EMjSpecGraph::Scs)
		{
			FMjScsScope* Scope = FMjScsScope::Current();
			if (Scope == nullptr) return nullptr;
			USimpleConstructionScript& Scs = Scope->GetScs();
			USCS_Node* ScsNode = Scs.CreateNode(Class, MjMakeNodeName(Type, Scs));
			if (ScsNode == nullptr) return nullptr;
			UMjNodeComponent* Node = Cast<UMjNodeComponent>(ScsNode->ComponentTemplate);
			if (Node == nullptr) return nullptr;
			Node->EnsureSerial();
			if (Parent == nullptr)
			{
				Scs.AddNode(ScsNode);
				Scope->NoteNodeCreated(*Node, *ScsNode);
			}
			else FMjScsAdapter::Adopt<UMjNodeComponent>(*Parent, MjAppendChild, Node);
			return Node;
		}
#endif
		FMjInstanceScope* Scope = FMjInstanceScope::Current();
		if (Scope == nullptr) return nullptr;
		AActor& Owner = Scope->GetOwner();
		UMjNodeComponent* Node = NewObject<UMjNodeComponent>(&Owner, Class,
			MjMakeNodeName(Type, Owner), RF_Transactional);
		if (Node == nullptr) return nullptr;
		Node->EnsureSerial();
		if (Parent == nullptr) Owner.SetRootComponent(Node);
		Node->RegisterComponent();
		Owner.AddInstanceComponent(Node);
		if (Parent != nullptr) FMjInstanceAdapter::Adopt<UMjNodeComponent>(*Parent, MjAppendChild, Node);
		return Node;
	}

	static int32 FieldId(const ps::ue::FMjElementThunks& Fields, FStringView Name)
	{
		for (int32 I = 0; I < Fields.FieldCount; ++I)
			if (Name.Equals(Fields.Fields[I].XmlName, ESearchCase::CaseSensitive)) return I;
		return -1;
	}

	bool CollectOrientation(UMjNodeComponent& Node, ElementType Type,
		const tinyxml2::XMLElement& XmlElement, FStringView Name, FStringView Text)
	{
		ps::core::OrientKind Kind;
		int32 Count = 0;
		if (Name.Equals(TEXT("quat"))) { Kind = ps::core::OrientKind::Quat; Count = 4; }
		else if (Name.Equals(TEXT("axisangle"))) { Kind = ps::core::OrientKind::AxisAngle; Count = 4; }
		else if (Name.Equals(TEXT("xyaxes"))) { Kind = ps::core::OrientKind::XYAxes; Count = 6; }
		else if (Name.Equals(TEXT("zaxis"))) { Kind = ps::core::OrientKind::ZAxis; Count = 3; }
		else if (Name.Equals(TEXT("euler"))) { Kind = ps::core::OrientKind::Euler; Count = 3; }
		else return false;
		const int32 QuatField = FieldId(ps::ue::Thunks(Type), TEXT("quat"));
		if (QuatField < 0) return false;
		TArray<double> Values;
		if (!fieldtext::Parse(Text, Values, Count, Count))
		{
			Error(&XmlElement, FString::Printf(TEXT("invalid orientation attribute '%s'"), *FString(Name)));
			return true;
		}
		for (const FPendingOrientation& Earlier : Orientations)
			if (Earlier.Node == &Node)
			{
				Error(&XmlElement, TEXT("multiple orientation spellings on one element"));
				return true;
			}
		FPendingOrientation& Pending = Orientations.AddDefaulted_GetRef();
		Pending.Node = &Node;
		Pending.FieldId = QuatField;
		Pending.Kind = Kind;
		for (int32 I = 0; I < Count; ++I) Pending.Raw[I] = Values[I];
		return true;
	}

	void ReadElement(tinyxml2::XMLElement& XmlElement, ElementType Type,
		UMjNodeComponent* Parent, int32 Depth)
	{
		if (Depth > 256) { Error(&XmlElement, TEXT("MJCF element nesting exceeds 256")); return; }
		UMjNodeComponent* Node = Construct(Type, Parent);
		if (Node == nullptr) { Error(&XmlElement, TEXT("cannot construct UE spec element")); return; }
		if (Parent == nullptr) Result.Root = Node;
		const auto Found = Provenance.find(&XmlElement);
		Node->SourceFile = Found == Provenance.end() ? Filename : FromUtf8(Found->second.file.c_str());
		Node->SourceLine = Found == Provenance.end() ? XmlElement.GetLineNum() : Found->second.line;
		const ps::ue::FMjElementThunks& Fields = ps::ue::Thunks(Type);
		for (const tinyxml2::XMLAttribute* Attr = XmlElement.FirstAttribute(); Attr != nullptr; Attr = Attr->Next())
		{
			const FString Name = FromUtf8(Attr->Name());
			const FString Value = FromUtf8(Attr->Value());
			if (CollectOrientation(*Node, Type, XmlElement, Name, Value)) continue;
			const int32 Id = FieldId(Fields, Name);
			if (Id < 0 && Fields.bNativeAliasIgnoresUnknownAttributes)
			{
				// MuJoCo validates <frame> and <replicate> through alias=body,
				// bypassing their own attribute lists. Its native reader ignores
				// extra attributes there (for example legacy replicate prefix).
				// Keep the same compiled semantics and make the dropped spelling visible.
				UE_LOG(LogURLabImport, Warning,
					TEXT("%s(%d): native MJCF ignores attribute '%s' on <%s>; it will not be exported"),
					*Node->SourceFile, Node->SourceLine, *Name, *FromUtf8(XmlElement.Name()));
				continue;
			}
			if (Id < 0 || Fields.Fields[Id].bElementText)
			{
				Error(&XmlElement, FString::Printf(TEXT("unknown MJCF attribute '%s' on <%s>"), *Name, *FromUtf8(XmlElement.Name())));
				continue;
			}
			if (!Fields.Parse(Node, Id, Value))
				Error(&XmlElement, FString::Printf(TEXT("invalid MJCF attribute '%s'"), *Name));
		}
		for (int32 Id = 0; Id < Fields.FieldCount; ++Id)
		{
			if (!Fields.Fields[Id].bElementText || XmlElement.GetText() == nullptr) continue;
			if (!Fields.Parse(Node, Id, FromUtf8(XmlElement.GetText())))
				Error(&XmlElement, TEXT("invalid MJCF element text"));
		}
		for (tinyxml2::XMLElement* Child = XmlElement.FirstChildElement(); Child != nullptr;
			Child = Child->NextSiblingElement())
		{
			ElementType ChildType{};
			int32 Slot = -1;
			if (!ps::ue::ElementTypeForChildTag(Type, FromUtf8(Child->Name()), ChildType, Slot))
			{
				Error(Child, FString::Printf(TEXT("unsupported MJCF child <%s> under <%s>"),
					*FromUtf8(Child->Name()), *FromUtf8(XmlElement.Name())), true);
				continue;
			}
			ReadElement(*Child, ChildType, Node, Depth + 1);
		}
	}

	void ResolveOrientations()
	{
		ps::core::OrientContext Context;
		for (tinyxml2::XMLElement* Child = Xml.RootElement()->FirstChildElement(); Child != nullptr;
			Child = Child->NextSiblingElement())
		{
			if (FCStringAnsi::Strcmp(Child->Name(), "compiler") != 0) continue;
			if (const char* Angle = Child->Attribute("angle")) Context.degree = FCStringAnsi::Strcmp(Angle, "radian") != 0;
			if (const char* Sequence = Child->Attribute("eulerseq")) Context.eulerseq = Sequence;
		}
		for (const FPendingOrientation& Pending : Orientations)
		{
			const std::array<double, 4> Quat = ps::core::ResolveOrientation(Pending.Kind, Pending.Raw, Context);
			const FString Text = FString::Printf(TEXT("%.17g %.17g %.17g %.17g"), Quat[0], Quat[1], Quat[2], Quat[3]);
			ElementType Type{};
			if (!ps::ue::ElementTypeOfNode(*Pending.Node, Type) ||
				!ps::ue::Thunks(Type).Parse(Pending.Node, Pending.FieldId, Text))
				Error(nullptr, TEXT("cannot store normalized orientation"));
		}
	}

	EMjSpecGraph Graph;
	FString Filename;
	tinyxml2::XMLDocument Xml;
	ps::mjcf::io::ProvenanceMap Provenance;
	FMjSpecParseResult Result;
	TArray<FPendingOrientation> Orientations;
};
} // namespace

#if WITH_EDITOR
FMjSpecParseResult ParseUeIntoScs(UBlueprint& Blueprint, const FString& Xml,
	const FString& Filename, const FMjDocParseOptions& Options)
{
	FMjScsScope Scope(Blueprint);
	return FReader(EMjSpecGraph::Scs, Filename).Read(Xml, Options);
}
#endif

FMjSpecParseResult ParseUeIntoInstance(AActor& Actor, const FString& Xml,
	const FString& Filename, const FMjDocParseOptions& Options)
{
	FMjInstanceScope Scope(Actor);
	return FReader(EMjSpecGraph::Instance, Filename).Read(Xml, Options);
}
} // namespace urlab::spec::io

#endif // URLAB_MJ_GEN
