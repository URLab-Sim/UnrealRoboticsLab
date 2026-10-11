// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "MjUeXml.h"

#if URLAB_MJ_GEN

#include "MuJoCo/Gen/MjDispatch.gen.h"
#include "MuJoCo/Gen/MjReflect.gen.h"
#include "MuJoCo/Spec/MjFieldText.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjTreeAdapters.h"
#include "MjFromtoFold.h"

THIRD_PARTY_INCLUDES_START
#include "tinyxml2.h"
THIRD_PARTY_INCLUDES_END

namespace urlab::spec::io
{
namespace
{
using ps::mjcf::ElementType;

void Report(TArray<FMjSpecDiagnostic>* Errors, const UMjNodeComponent* Node, const FString& Message)
{
	if (Errors == nullptr) return;
	FMjSpecDiagnostic& Error = Errors->AddDefaulted_GetRef();
	Error.Message = Message;
	if (Node != nullptr)
	{
		Error.File = Node->SourceFile;
		Error.Line = Node->SourceLine;
	}
}

std::string Utf8(FStringView Value)
{
	FTCHARToUTF8 Converted(Value.GetData(), Value.Len());
	return std::string(Converted.Get(), Converted.Length());
}

FString FromUtf8(const char* Data)
{
	return UTF8_TO_TCHAR(Data != nullptr ? Data : "");
}

bool WriteNode(tinyxml2::XMLDocument& Xml, tinyxml2::XMLNode& Parent,
	const FSpecRef& Spec, const UMjNodeComponent& Node, const char* Tag,
	bool bTopLevel, bool bInDefault, const TArray<FMjXmlAttributeOverride>* Overrides,
	TArray<FMjSpecDiagnostic>* Errors)
{
	ElementType Type{};
	if (!ps::ue::ElementTypeOfNode(Node, Type))
	{
		Report(Errors, &Node, TEXT("component is not an MJCF element"));
		return false;
	}
	const ps::ue::FMjElementThunks& Fields = ps::ue::Thunks(Type);
	auto* Element = Xml.NewElement(Tag);
	TArray<double> SpatialSize;
	const bool bSpatialSize = (Type == ElementType::Geom || Type == ElementType::Site)
		&& MjSpatialSizeOverride(Node, bInDefault, SpatialSize);
	for (int32 FieldId = 0; FieldId < Fields.FieldCount; ++FieldId)
	{
		const ps::ue::FMjFieldSpec& Field = Fields.Fields[FieldId];
		if (bSpatialSize && FStringView(Field.XmlName).Equals(TEXT("size"), ESearchCase::CaseSensitive))
			continue;
		bool bRemoved = false;
		if (Overrides != nullptr)
		{
			for (const FMjXmlAttributeOverride& Override : *Overrides)
				if (Override.Serial == Node.Serial && Override.bRemove &&
					FStringView(Field.XmlName).Equals(Override.Attribute, ESearchCase::CaseSensitive))
				{
					bRemoved = true;
					break;
				}
		}
		if (bRemoved) continue;
		if (!Fields.Has(&Node, FieldId)) continue;
		FString Value;
		if (!Fields.Format(&Node, FieldId, Value))
		{
			Report(Errors, &Node, FString::Printf(TEXT("cannot format MJCF attribute '%s'"), Field.XmlName));
			return false;
		}
		const std::string Name = Utf8(Field.XmlName);
		const std::string Text = Utf8(Value);
		if (Field.bElementText) Element->SetText(Text.c_str());
		else Element->SetAttribute(Name.c_str(), Text.c_str());
	}
	if (Overrides != nullptr)
	{
		for (const FMjXmlAttributeOverride& Override : *Overrides)
		{
			if (Override.Serial != Node.Serial || Override.bRemove) continue;
			const std::string Name = Utf8(Override.Attribute);
			const std::string Text = Utf8(Override.Value);
			Element->SetAttribute(Name.c_str(), Text.c_str());
		}
	}
	if (bSpatialSize)
	{
		if (SpatialSize.IsEmpty()) Element->DeleteAttribute("size");
		else
		{
			const std::string Text = Utf8(fieldtext::Format(SpatialSize));
			Element->SetAttribute("size", Text.c_str());
		}
	}
	for (const FMjOrderedChild& Child : MjOrderedChildrenOf(Spec, const_cast<UMjNodeComponent&>(Node)))
	{
		ElementType ChildType{};
		if (Child.Node == nullptr || !ps::ue::ElementTypeOfNode(*Child.Node, ChildType))
		{
			Report(Errors, Child.Node, TEXT("child is not an MJCF element"));
			return false;
		}
		const TCHAR* ContextTag = ps::ue::ChildTagFor(Type, ChildType);
		if (ContextTag == nullptr)
		{
			Report(Errors, Child.Node, TEXT("child has no MJCF tag in this parent context"));
			return false;
		}
		const std::string ChildTag = Utf8(ContextTag);
		if (!WriteNode(Xml, *Element, Spec, *Child.Node, ChildTag.c_str(), Type == ElementType::Model,
			bInDefault || Type == ElementType::Default,
			Overrides, Errors)) return false;
	}
	// The model has implicit top-level sections even when none was authored.
	// Omit only their empty representations, never an empty concrete element.
	if (bTopLevel && Element->NoChildren() && Element->FirstAttribute() == nullptr)
		return true;
	Parent.InsertEndChild(Element);
	return true;
}

FString Print(tinyxml2::XMLDocument& Xml)
{
	tinyxml2::XMLPrinter Printer(nullptr, /*compact=*/false);
	Xml.Print(&Printer);
	return FromUtf8(Printer.CStr());
}
} // namespace

FString WriteUeXml(const FSpecRef& Spec, TArray<FMjSpecDiagnostic>* OutErrors,
	const TArray<FMjXmlAttributeOverride>* Overrides)
{
	if (!Spec.IsValid())
	{
		Report(OutErrors, nullptr, TEXT("the spec handle names no component tree"));
		return FString();
	}
	tinyxml2::XMLDocument Xml;
	if (!WriteNode(Xml, Xml, Spec, *Spec.GetRoot(), "mujoco", false, false, Overrides, OutErrors))
		return FString();
	return Print(Xml);
}

FString WriteUeElementXml(const FSpecRef& Spec, const UMjNodeComponent& Node,
	TArray<FMjSpecDiagnostic>* OutErrors)
{
	ElementType Type{};
	if (!ps::ue::ElementTypeOfNode(Node, Type))
	{
		Report(OutErrors, &Node, TEXT("component is not an MJCF element"));
		return FString();
	}
	const TCHAR* Tag = ps::ue::TagForElement(Type);
	if (Tag == nullptr) return FString();
	const std::string Name = Utf8(Tag);
	tinyxml2::XMLDocument Xml;
	if (!WriteNode(Xml, Xml, Spec, Node, Name.c_str(), false, false, nullptr, OutErrors))
		return FString();
	return Print(Xml);
}
} // namespace urlab::spec::io

#endif // URLAB_MJ_GEN
