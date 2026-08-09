// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// The inherited value the panel puts on an unset row, and what it costs.
//
// Two things are worth pinning. The string, because "0.3 (from visual)" is the
// whole feature and a wrong class name is a confident lie about where a number
// came from. And the number of effective contexts one refresh builds, because
// the natural way to write this row builder -- ask the spec once per attribute
// -- is quadratic in the model, and it is the exact shape of lag this project
// has already paid for once.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "UObject/PropertyOptional.h"
#include "UObject/UnrealType.h"

#include "MjEffectiveDetails.h"
#include "MuJoCo/Gen/Elements/Geometry/MjGeom.gen.h"
#include "MuJoCo/Spec/MjEffective.h"
#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSpecRef.h"

namespace MjEffectiveDetailsTests
{
using namespace urlab::spec;

/** A geom that authors nothing: every number it uses comes off its class. */
const TCHAR* const Corpus = TEXT(R"(<mujoco model="inherited">
  <default>
    <default class="visual">
      <geom type="sphere" size="0.3" rgba="1 0 0 1" contype="0"/>
    </default>
  </default>
  <worldbody>
    <body name="base">
      <geom name="shell" class="visual"/>
    </body>
  </worldbody>
</mujoco>
)");

struct FScratchDoc
{
	UWorld* World = nullptr;
	AActor* Actor = nullptr;

	~FScratchDoc()
	{
		if (World != nullptr)
		{
			World->DestroyWorld(false);
		}
	}
};

bool Parse(FAutomationTestBase& Test, FScratchDoc& Doc)
{
	const FString Stem = FString::Printf(TEXT("MjEff_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	Doc.World = UWorld::CreateWorld(EWorldType::Editor, /*bInformEngineOfWorld=*/false, FName(*Stem));
	if (Doc.World == nullptr)
	{
		Test.AddError(TEXT("could not create a scratch world"));
		return false;
	}
	Doc.Actor = Doc.World->SpawnActor<AActor>();
	if (Doc.Actor == nullptr)
	{
		Test.AddError(TEXT("could not spawn a scratch actor"));
		return false;
	}
	const FString Path =
		FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("URLab/TestModels") / Stem);
	if (!MjParseIntoActor(*Doc.Actor, Corpus, Path / (Stem + TEXT(".xml"))).IsOk())
	{
		Test.AddError(TEXT("parse failed"));
		return false;
	}
	return true;
}

/** The optional property named `Name` on `Node`'s class, or null. */
const FOptionalProperty* OptionalNamed(const UMjNodeComponent& Node, const TCHAR* Name)
{
	return CastField<FOptionalProperty>(Node.GetClass()->FindPropertyByName(FName(Name)));
}

}  // namespace MjEffectiveDetailsTests

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjEffectiveDetailsNamesTheClassThatSuppliedTheValue,
	"URLab.Editor.EffectiveDetailsNamesTheClassThatSuppliedTheValue",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjEffectiveDetailsNamesTheClassThatSuppliedTheValue::RunTest(const FString& Parameters)
{
	using namespace MjEffectiveDetailsTests;

	FScratchDoc Doc;
	if (!Parse(*this, Doc))
	{
		return false;
	}

	UMjGeomBase* const Geom = Doc.Actor->FindComponentByClass<UMjGeomBase>();
	if (!TestNotNull(TEXT("the geom component"), Geom))
	{
		return false;
	}
	TestFalse(TEXT("the geom authors no size"), Geom->Size.IsSet());

	const FOptionalProperty* const Size = OptionalNamed(*Geom, TEXT("Size"));
	if (!TestNotNull(TEXT("the Size property is optional"), Size))
	{
		return false;
	}

	FString Text;
	FString ClassName;
	TestTrue(TEXT("size resolves through the class chain"),
		FMjEffectiveDetails::ResolveInherited(*Geom, *Size, Text, ClassName));
	TestEqual(TEXT("named the class that authored it"), ClassName, FString(TEXT("visual")));
	TestTrue(FString::Printf(TEXT("the value reads as the authored one, got '%s'"), *Text),
		Text.Contains(TEXT("0.3")));

	// An attribute nobody authored anywhere has nothing to report, and saying
	// so is what keeps every other row clean.
	if (const FOptionalProperty* const Margin = OptionalNamed(*Geom, TEXT("Margin")))
	{
		FString Unused;
		FString UnusedClass;
		TestFalse(TEXT("an attribute no layer authored reports nothing"),
			FMjEffectiveDetails::ResolveInherited(*Geom, *Margin, Unused, UnusedClass));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjEffectiveDetailsBuildsOneContextPerRefresh,
	"URLab.Editor.EffectiveDetailsBuildsOneContextPerRefresh",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjEffectiveDetailsBuildsOneContextPerRefresh::RunTest(const FString& Parameters)
{
	using namespace MjEffectiveDetailsTests;

	FScratchDoc Doc;
	if (!Parse(*this, Doc))
	{
		return false;
	}

	UMjGeomBase* const Geom = Doc.Actor->FindComponentByClass<UMjGeomBase>();
	if (!TestNotNull(TEXT("the geom component"), Geom))
	{
		return false;
	}

	TArray<const FOptionalProperty*> Asked;
	for (const TCHAR* Name : {TEXT("Size"), TEXT("Rgba"), TEXT("Type"), TEXT("Contype"), TEXT("Margin")})
	{
		if (const FOptionalProperty* const Property = OptionalNamed(*Geom, Name))
		{
			Asked.Add(Property);
		}
	}
	if (!TestTrue(TEXT("several attributes to ask about"), Asked.Num() >= 4))
	{
		return false;
	}

	// One scope around the batch, exactly as the customization opens one around
	// a refresh. Without it this is one whole-spec walk per attribute.
	const int64 Before = MjEffectiveContextBuilds();
	{
		FMjEffectiveScope Scope(*Geom);
		for (const FOptionalProperty* const Property : Asked)
		{
			FString Text;
			FString ClassName;
			FMjEffectiveDetails::ResolveInherited(*Geom, *Property, Text, ClassName);
		}
	}
	const int64 Built = MjEffectiveContextBuilds() - Before;

	TestEqual(TEXT("one context for the whole refresh"), Built, static_cast<int64>(1));
	return true;
}

#endif  // URLAB_MJ_GEN && WITH_EDITOR
