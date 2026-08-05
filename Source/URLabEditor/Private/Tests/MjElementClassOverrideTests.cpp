// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// The class-override registry: does a hand subclass reach the spec, and
// does the spec still behave as though it had not?
//
// The second half is the one that matters. A subclass that the reader builds
// but the writer cannot serialise, or that dispatch stops recognising, would be
// a presentation layer bought at the cost of the spec being a spec. So
// the round trip is asserted through an overridden element, not just the class
// of the node the reader produced.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "GameFramework/Actor.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/Guid.h"
#include "UObject/Package.h"

#include "MuJoCo/Spec/MjSpecRef.h"
#include "MuJoCo/Spec/MjElementIdentity.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjNodeFactories.h"

#include "Tests/MjElementClassOverrideFixture.h"

namespace MjElementClassOverrideTests
{
using namespace urlab::spec;

const TCHAR* const Corpus = TEXT(R"(<mujoco model="override">
  <worldbody>
    <body name="base" pos="0 0 0.1">
      <geom name="plinth" type="box" size="0.1 0.2 0.05"/>
    </body>
  </worldbody>
</mujoco>
)");

UBlueprint* MakeScratchBlueprint()
{
	const FString Name = FString::Printf(TEXT("MjOverride_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	UPackage* Package = CreatePackage(*(TEXT("/Temp/") + Name));
	return FKismetEditorUtilities::CreateBlueprint(AActor::StaticClass(), Package, FName(*Name),
		BPTYPE_Normal, UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
}

/**
 * Restores whatever the process had registered, so a test cannot leak into one.
 *
 * The whole table, not the entries this file happens to touch: the registrations
 * that must survive are the ones module startup made, and naming them here would
 * silently drop every one added afterwards.
 */
struct FScopedOverrides
{
	FScopedOverrides() : Saved(MjSnapshotElementClasses()) {}
	~FScopedOverrides() { MjRestoreElementClasses(Saved); }

	TMap<int32, UClass*> Saved;
};

/**
 * The first node of `Blueprint`'s template graph that is a `Generated` element.
 *
 * Keyed on the generated base class rather than on the schema element type,
 * because that is the question being asked -- a node found this way is one an
 * override would have replaced -- and because the generated runtime tables are
 * not exported from the runtime module for an editor-side caller to reach.
 */
UMjNodeComponent* FindNodeOfClass(UBlueprint& Blueprint, const UClass* Generated)
{
	if (Blueprint.SimpleConstructionScript == nullptr)
	{
		return nullptr;
	}
	for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
	{
		UMjNodeComponent* Element = Cast<UMjNodeComponent>(Node->ComponentTemplate);
		if (Element != nullptr && Element->GetClass()->IsChildOf(Generated))
		{
			return Element;
		}
	}
	return nullptr;
}

}  // namespace MjElementClassOverrideTests

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjElementClassOverrideBuildsSubclass,
	"URLab.Doc.ElementClassOverrideBuildsSubclass",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjElementClassOverrideBuildsSubclass::RunTest(const FString& Parameters)
{
	using namespace MjElementClassOverrideTests;
	using namespace urlab::spec;

	FScopedOverrides Restore;

	// Unregistered, the reader builds the generated class exactly as before.
	{
		MjResetElementClasses();
		UBlueprint* Blueprint = MakeScratchBlueprint();
		if (!TestNotNull(TEXT("scratch Blueprint"), Blueprint))
		{
			return false;
		}
		const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, Corpus, TEXT("<inline>"));
		TestTrue(TEXT("baseline parse succeeds"), Parsed.IsOk());

		UMjNodeComponent* Geom = FindNodeOfClass(*Blueprint, UMjGeomBase::StaticClass());
		if (TestNotNull(TEXT("baseline geom node"), Geom))
		{
			TestTrue(TEXT("unregistered geom is the generated class"),
				Geom->GetClass() == UMjGeomBase::StaticClass());
		}
	}

	// Registered, the same read produces the subclass instead.
	{
		MjResetElementClasses();
		MjSetElementClass(psm::ElementType::Geom, UMjTestPresentationGeom::StaticClass());
		MjSetElementClass(psm::ElementType::Body, UMjTestPresentationBody::StaticClass());

		UBlueprint* Blueprint = MakeScratchBlueprint();
		if (!TestNotNull(TEXT("scratch Blueprint"), Blueprint))
		{
			return false;
		}
		const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, Corpus, TEXT("<inline>"));
		TestTrue(TEXT("overridden parse succeeds"), Parsed.IsOk());

		UMjNodeComponent* Geom = FindNodeOfClass(*Blueprint, UMjGeomBase::StaticClass());
		if (TestNotNull(TEXT("overridden geom node"), Geom))
		{
			TestTrue(TEXT("registered geom is the subclass"),
				Geom->GetClass() == UMjTestPresentationGeom::StaticClass());
		}

		// The world body is a body too, so this also proves the override
		// applies at the root of the tree and not only to leaves.
		UMjNodeComponent* Body = FindNodeOfClass(*Blueprint, UMjBodyBase::StaticClass());
		if (TestNotNull(TEXT("overridden body node"), Body))
		{
			TestTrue(TEXT("registered body is the subclass"),
				Body->GetClass() == UMjTestPresentationBody::StaticClass());
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjElementClassOverrideKeepsTheSpec,
	"URLab.Doc.ElementClassOverrideKeepsTheSpec",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjElementClassOverrideKeepsTheSpec::RunTest(const FString& Parameters)
{
	using namespace MjElementClassOverrideTests;
	using namespace urlab::spec;

	FScopedOverrides Restore;

	// Read once with no override and once with, and require the two specs
	// to write out the same MJCF. A subclass that dispatch or the writer
	// treated as anything other than its generated base would diverge here.
	FString Plain;
	{
		MjResetElementClasses();
		UBlueprint* Blueprint = MakeScratchBlueprint();
		if (!TestNotNull(TEXT("scratch Blueprint"), Blueprint))
		{
			return false;
		}
		TestTrue(TEXT("plain parse"), MjParseIntoBlueprint(*Blueprint, Corpus, TEXT("<inline>")).IsOk());
		Plain = FSpecRef::OverBlueprint(*Blueprint).WriteMjcf();
	}

	FString Overridden;
	{
		MjResetElementClasses();
		MjSetElementClass(psm::ElementType::Geom, UMjTestPresentationGeom::StaticClass());
		MjSetElementClass(psm::ElementType::Body, UMjTestPresentationBody::StaticClass());

		UBlueprint* Blueprint = MakeScratchBlueprint();
		if (!TestNotNull(TEXT("scratch Blueprint"), Blueprint))
		{
			return false;
		}
		TestTrue(TEXT("overridden parse"), MjParseIntoBlueprint(*Blueprint, Corpus, TEXT("<inline>")).IsOk());
		Overridden = FSpecRef::OverBlueprint(*Blueprint).WriteMjcf();
	}

	TestTrue(TEXT("the plain read wrote something"), !Plain.IsEmpty());
	TestEqual(TEXT("a presentation subclass writes as its generated base"), Overridden, Plain);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSubclassIsItsGeneratedElement,
	"URLab.Doc.PresentationSubclassIsItsGeneratedElement",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSubclassIsItsGeneratedElement::RunTest(const FString& Parameters)
{
	using namespace MjElementClassOverrideTests;
	using namespace urlab::spec;

	// This test registers and resets, so it takes the fixture like the others:
	// without it the reset below wipes the registrations module startup made and
	// every later test builds generated classes where a hand subclass is meant.
	FScopedOverrides Restore;

	// The claim the whole subclass shape rests on: a hand class over a
	// generated element is not a second kind of element, because identity
	// walks the superclass chain to the nearest generated base. If that ever
	// stopped holding, a subclass would read and write as nothing at all --
	// silently, since every one of those paths fails closed.
	psm::ElementType Type{};
	if (TestTrue(TEXT("a generated geom class has an element type"),
			MjElementTypeOfClass(UMjGeomBase::StaticClass(), Type)))
	{
		TestTrue(TEXT("and it is Geom"), Type == psm::ElementType::Geom);
	}
	if (TestTrue(TEXT("a subclass of it also has one"),
			MjElementTypeOfClass(UMjTestPresentationGeom::StaticClass(), Type)))
	{
		TestTrue(TEXT("and it is the same Geom"), Type == psm::ElementType::Geom);
	}
	if (TestTrue(TEXT("a subclass instance reports it too"),
			MjElementTypeOfNode(*GetDefault<UMjTestPresentationGeom>(), Type)))
	{
		TestTrue(TEXT("as Geom"), Type == psm::ElementType::Geom);
	}

	// The inverse still names the generated class, not the registration: the
	// override is a construction decision and not a change of what the element
	// IS, which is why writing and dispatch never consult it.
	MjSetElementClass(psm::ElementType::Geom, UMjTestPresentationGeom::StaticClass());
	TestTrue(TEXT("the element still names its generated class"),
		MjGeneratedClassOf(psm::ElementType::Geom) == UMjGeomBase::StaticClass());
	MjResetElementClasses();

	TestEqual(TEXT("and keeps its MJCF tag"), FString(MjTagOf(psm::ElementType::Geom)), FString(TEXT("geom")));

	// The abstract base is not an element, so classification has something to
	// refuse and callers keying off it are not looking at a spec node.
	TestFalse(TEXT("the node base is not an element"),
		MjElementTypeOfClass(UMjNodeComponent::StaticClass(), Type));
	TestFalse(TEXT("nor is a plain scene component"),
		MjElementTypeOfClass(USceneComponent::StaticClass(), Type));
	TestFalse(TEXT("nor is nothing at all"), MjElementTypeOfClass(nullptr, Type));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjElementClassOverrideIsPerElement,
	"URLab.Doc.ElementClassOverrideIsPerElement",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjElementClassOverrideIsPerElement::RunTest(const FString& Parameters)
{
	using namespace MjElementClassOverrideTests;
	using namespace urlab::spec;

	FScopedOverrides Restore;
	MjResetElementClasses();

	// The fallback is the caller's, so an unregistered element is whatever the
	// factory was going to build and never another element's registration.
	TestTrue(TEXT("unregistered falls back"),
		MjElementClass(psm::ElementType::Geom, UMjGeomBase::StaticClass()) == UMjGeomBase::StaticClass());

	MjSetElementClass(psm::ElementType::Geom, UMjTestPresentationGeom::StaticClass());
	TestTrue(TEXT("registered wins over the fallback"),
		MjElementClass(psm::ElementType::Geom, UMjGeomBase::StaticClass())
			== UMjTestPresentationGeom::StaticClass());
	TestTrue(TEXT("a sibling element is untouched"),
		MjElementClass(psm::ElementType::Body, UMjBodyBase::StaticClass()) == UMjBodyBase::StaticClass());

	MjResetElementClasses();
	TestTrue(TEXT("reset restores the fallback"),
		MjElementClass(psm::ElementType::Geom, UMjGeomBase::StaticClass()) == UMjGeomBase::StaticClass());
	return true;
}

#endif  // URLAB_MJ_GEN && WITH_EDITOR
