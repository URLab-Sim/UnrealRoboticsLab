// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// What the components panel reads like after an import.
//
// The tree's SHAPE was never wrong: `<actuator>`, `<sensor>`, `<asset>` and the
// rest are real zero-attribute container elements in the schema, so each is
// already a node with its section's elements beneath it, and `<worldbody>` is
// the model's world-slot `body`. What was wrong is that every node was named
// after its XML tag with an unconditional ordinal appended, so the sections read
// `actuator_0` and the world body read `body_23` -- a structure nobody could
// navigate and a world body the user concluded was missing.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "GameFramework/Actor.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "UObject/Package.h"

#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjElementIdentity.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Gen/Elements/Bodies/MjBody.gen.h"
#include "MuJoCo/Gen/MjDispatch.gen.h"

namespace MjNodeNameTests
{

UBlueprint* MakeScratchBlueprint()
{
	const FString Name = FString::Printf(TEXT("MjNodeNames_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	UPackage* Package = CreatePackage(*(TEXT("/Temp/") + Name));
	return FKismetEditorUtilities::CreateBlueprint(AActor::StaticClass(), Package, FName(*Name), BPTYPE_Normal,
		UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
}

/** Every SCS variable name in the Blueprint. */
TSet<FString> VariableNames(UBlueprint& Blueprint)
{
	TSet<FString> Out;
	if (Blueprint.SimpleConstructionScript == nullptr)
	{
		return Out;
	}
	for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
	{
		if (Node != nullptr && Cast<UMjNodeComponent>(Node->ComponentTemplate) != nullptr)
		{
			Out.Add(Node->GetVariableName().ToString());
		}
	}
	return Out;
}

/**
 * Whether the components panel shows an element under a name that names IT.
 *
 * Its MJCF name, or -- when two elements of different types share that name,
 * which is legal MJCF -- its MJCF name and what it is. Nothing else: an ordinal
 * suffix is the disambiguation this item replaced, and a rule that merely looked
 * for "the name, an underscore, something" would accept it back.
 */
bool ShownUnderItsOwnName(const FString& Variable, const FString& MjName, urlab::spec::psm::ElementType Type)
{
	return Variable == MjName || Variable == MjName + TEXT("_") + urlab::spec::gen::TagForElement(Type);
}

UBlueprint* ParseScratch(FAutomationTestBase& Test, const FString& Xml, const FString& Filename)
{
	UBlueprint* Blueprint = MakeScratchBlueprint();
	if (Blueprint == nullptr)
	{
		Test.AddError(TEXT("could not create a scratch Blueprint"));
		return nullptr;
	}
	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, Xml, Filename);
	if (!Parsed.IsOk())
	{
		TArray<FString> Lines;
		for (const FMjSpecDiagnostic& Diagnostic : Parsed.Errors)
		{
			Lines.Add(Diagnostic.ToString());
		}
		Test.AddError(FString::Printf(TEXT("parse failed: %s"), *FString::Join(Lines, TEXT("; "))));
		return nullptr;
	}
	return Blueprint;
}

} // namespace MjNodeNameTests

// ============================================================================
// URLab.Import.NodeNamesReadLikeTheMjcf
//   Over MuJoCo's own humanoid, because the complaint was about a real robot:
//   the sections are there and unsuffixed, the world body is called what the
//   schema calls it, and the bodies and joints carry the names the MJCF gave
//   them rather than ordinals.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNodeNamesReadableTest, "URLab.Import.NodeNamesReadLikeTheMjcf",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjNodeNamesReadableTest::RunTest(const FString& Parameters)
{
	using namespace MjNodeNameTests;

	const FString File = FPaths::Combine(FPaths::ProjectPluginsDir(), TEXT("UnrealRoboticsLab"), TEXT("Content"),
		TEXT("TestData"), TEXT("humanoid.xml"));
	FString Xml;
	if (!TestTrue(TEXT("the humanoid fixture is on disk"), FFileHelper::LoadFileToString(Xml, *File)))
	{
		return false;
	}

	UBlueprint* Blueprint = ParseScratch(*this, Xml, File);
	if (Blueprint == nullptr)
	{
		return false;
	}
	const TSet<FString> Names = VariableNames(*Blueprint);

	// The sections a user goes looking for, spelled as the schema spells them.
	// Every top-level section is present whether or not this document authors
	// one: the humanoid writes no <sensor>, and the panel still offers it,
	// unset, so adding a sensor is editing a field rather than knowing that a
	// section can be conjured. Unset is what keeps that free -- an unauthored
	// section writes nothing, which MjSectionFoldTests holds to.
	for (const TCHAR* Section : {TEXT("worldbody"), TEXT("asset"), TEXT("actuator"), TEXT("tendon"), TEXT("contact"),
			 TEXT("keyframe"), TEXT("default"), TEXT("sensor"), TEXT("option"), TEXT("compiler")})
	{
		TestTrue(*FString::Printf(TEXT("section '%s' is a node and keeps its bare name"), Section),
			Names.Contains(Section));
	}

	// Named elements carry their own names. This is what makes a robot with
	// twenty-nine joints navigable at all.
	for (const TCHAR* Named : {TEXT("torso"), TEXT("head"), TEXT("pelvis"), TEXT("shin_left"), TEXT("foot_right"),
			 TEXT("abdomen_z"), TEXT("hip_x_right"), TEXT("knee_left"), TEXT("hamstring_right")})
	{
		TestTrue(*FString::Printf(TEXT("element '%s' is named after itself"), Named), Names.Contains(Named));
	}

	// MJCF names are unique per element type, not across types: the humanoid has
	// a body `torso` and a geom `torso`. The first in spec order keeps the
	// bare name and only the second is suffixed, by what it is.
	TestTrue(TEXT("a name shared by two element types suffixes only the second"), Names.Contains(TEXT("torso_geom")));
	TestFalse(TEXT("a named element is never disambiguated by an ordinal"), Names.Contains(TEXT("torso_1")));

	// And no NAMED element is left on the ordinal it was constructed with. An
	// unnamed one has nothing better to be called than its tag, so it keeps a
	// tag and an ordinal -- that is the schema's answer, not a naming failure.
	//
	// The suffix is checked for what it IS, not merely for being present: a
	// `StartsWith(name + "_")` accepts `torso_1` as readily as `torso_geom`, so
	// the very disambiguation this item replaced would pass the sweep that is
	// supposed to forbid it. The only legal suffix is the element's own tag.
	int32 Stranded = 0;
	for (USCS_Node* Node : Blueprint->SimpleConstructionScript->GetAllNodes())
	{
		const UMjNodeComponent* Element = Node != nullptr ? Cast<UMjNodeComponent>(Node->ComponentTemplate) : nullptr;
		if (Element == nullptr || !Element->MjName.IsSet())
		{
			continue;
		}
		const FString Variable = Node->GetVariableName().ToString();
		const FString Expected = Element->MjName.GetValue();

		urlab::spec::psm::ElementType Type{};
		if (!urlab::spec::MjElementTypeOfNode(*Element, Type))
		{
			continue;
		}
		if (!ShownUnderItsOwnName(Variable, Expected, Type))
		{
			++Stranded;
			AddError(FString::Printf(TEXT("element '%s' is shown as '%s', which is neither its name nor its name "
										  "suffixed by what it is"),
				*Expected, *Variable));
		}
	}
	TestEqual(TEXT("every named element is shown under its own name"), Stranded, 0);

	// The sweep's own guard. `torso_1` is exactly the shape this item removed,
	// and a rule that merely looks for "starts with the name and an underscore"
	// would accept it, so the sweep could not catch a return to it. Fed the
	// three cases directly, the rule has to sort them.
	TestFalse(TEXT("an ordinal suffix is not a name"),
		ShownUnderItsOwnName(TEXT("torso_1"), TEXT("torso"), urlab::spec::psm::ElementType::Geom));
	TestTrue(TEXT("a type suffix is"),
		ShownUnderItsOwnName(TEXT("torso_geom"), TEXT("torso"), urlab::spec::psm::ElementType::Geom));
	TestTrue(TEXT("and so is the bare name"),
		ShownUnderItsOwnName(TEXT("torso"), TEXT("torso"), urlab::spec::psm::ElementType::Body));
	TestFalse(TEXT("another element's tag is not this element's suffix"),
		ShownUnderItsOwnName(TEXT("torso_joint"), TEXT("torso"), urlab::spec::psm::ElementType::Geom));

	// The world body specifically: it is the model's world-slot `body`, and
	// naming it from its element type reads `body`, which is why the user
	// concluded there was no world body at all.
	// Through the SCS node tree, which is where a template's parentage lives --
	// a template is never registered, so its component attach parent is not it.
	USCS_Node* RootNode = nullptr;
	for (USCS_Node* Node : Blueprint->SimpleConstructionScript->GetRootNodes())
	{
		if (Node != nullptr && Cast<UMjNodeComponent>(Node->ComponentTemplate) != nullptr)
		{
			RootNode = Node;
			break;
		}
	}
	FString WorldBodyName;
	if (TestNotNull(TEXT("the spec has a root node"), RootNode))
	{
		// The root is the spec rather than an element, so it has no MJCF
		// `name`; the naming pass falls back to `<mujoco model=...>`, the label
		// the spec does carry, rather than leaving it on `mujoco_0`, the one
		// node in the tree whose name names nothing.
		TestEqual(TEXT("the spec root is shown under the model's own name"),
			RootNode->GetVariableName().ToString(), FString(TEXT("Humanoid")));

		for (USCS_Node* Child : RootNode->GetChildNodes())
		{
			if (Child != nullptr && Cast<UMjBodyBase>(Child->ComponentTemplate) != nullptr)
			{
				WorldBodyName = Child->GetVariableName().ToString();
				break;
			}
		}
	}
	TestEqual(TEXT("the model's world-slot body is shown as 'worldbody'"), WorldBodyName, FString(TEXT("worldbody")));

	return true;
}

// ============================================================================
// URLab.Import.AnUnlabelledSpecRootKeepsItsTag
//   `model` is optional. Without it the root has no label of its own, and the
//   honest name is the schema's tag -- still not an ordinal.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNodeNamesRootFallbackTest, "URLab.Import.AnUnlabelledSpecRootKeepsItsTag",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjNodeNamesRootFallbackTest::RunTest(const FString& Parameters)
{
	using namespace MjNodeNameTests;

	const TCHAR* const Xml = TEXT(R"(<mujoco>
  <worldbody>
    <body name="b"/>
  </worldbody>
</mujoco>
)");

	UBlueprint* Blueprint = ParseScratch(*this, Xml, TEXT("<inline>"));
	if (Blueprint == nullptr)
	{
		return false;
	}

	FString RootName;
	for (USCS_Node* Node : Blueprint->SimpleConstructionScript->GetRootNodes())
	{
		if (Node != nullptr && Cast<UMjNodeComponent>(Node->ComponentTemplate) != nullptr)
		{
			RootName = Node->GetVariableName().ToString();
			break;
		}
	}
	TestEqual(TEXT("an unlabelled root is shown as its tag"), RootName, FString(TEXT("mujoco")));

	return true;
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
