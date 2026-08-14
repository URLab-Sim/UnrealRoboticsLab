// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// How a name shared across element types reads in the components panel.
//
// MuJoCo names elements uniquely within a type, so a body, a geom and a joint
// may all be called `torso`; Unreal needs one name per component in a Blueprint,
// so one of the three has to give. The disambiguation is ours, not Unreal's, and
// it used to be an ordinal: whichever came second read `torso_1` and whichever
// came third read `torso_2`, which says nothing about either. Since two NAMED
// elements can only ever collide across types, the type is the real distinction,
// and that is what the suffix now carries.
//
// None of this reaches the model. The MJCF name lives in its own field and that
// field, never the component's Unreal name, is what feeds the spec.

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

#include "MuJoCo/Spec/MjNodeComponent.h"
#include "MuJoCo/Spec/MjSpecBuild.h"
#include "MuJoCo/Spec/MjSpecRef.h"

#include "Tests/MjParitySupport.h"

namespace MjNamingTests
{

UBlueprint* MakeScratchBlueprint()
{
	const FString Name = FString::Printf(TEXT("MjNaming_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	UPackage* Package = CreatePackage(*(TEXT("/Temp/") + Name));
	return FKismetEditorUtilities::CreateBlueprint(AActor::StaticClass(), Package, FName(*Name), BPTYPE_Normal,
		UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
}

UBlueprint* ParseScratch(FAutomationTestBase& Test, const FString& Xml)
{
	UBlueprint* Blueprint = MakeScratchBlueprint();
	if (Blueprint == nullptr)
	{
		Test.AddError(TEXT("could not create a scratch Blueprint"));
		return nullptr;
	}
	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, Xml, TEXT("<inline>"));
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

/** Every SCS variable name holding an element of the spec. */
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

/** How many elements still carry `MjName`, whatever the components panel shows. */
int32 CountWithMjName(UBlueprint& Blueprint, const TCHAR* MjName)
{
	int32 Count = 0;
	for (USCS_Node* Node : Blueprint.SimpleConstructionScript->GetAllNodes())
	{
		const UMjNodeComponent* Element = Node != nullptr ? Cast<UMjNodeComponent>(Node->ComponentTemplate) : nullptr;
		if (Element != nullptr && Element->MjName.IsSet() && Element->MjName.GetValue() == MjName)
		{
			++Count;
		}
	}
	return Count;
}

} // namespace MjNamingTests

// ============================================================================
// URLab.Import.ACrossTypeNameClashSuffixesByType
//   A body, a joint and a geom all called `torso`, which is legal MJCF and does
//   happen. The body reads first, so it keeps the bare name; the other two are
//   told apart by what they are.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCrossTypeNameClashTest, "URLab.Import.ACrossTypeNameClashSuffixesByType",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjCrossTypeNameClashTest::RunTest(const FString& Parameters)
{
	using namespace MjNamingTests;

	const TCHAR* const Xml = TEXT(R"(<mujoco model="clash">
  <worldbody>
    <body name="torso" pos="0 0 1">
      <joint name="torso" type="hinge" axis="0 0 1"/>
      <geom name="torso" type="box" size="0.1 0.1 0.1"/>
    </body>
  </worldbody>
</mujoco>
)");

	UBlueprint* Blueprint = ParseScratch(*this, Xml);
	if (Blueprint == nullptr)
	{
		return false;
	}
	const TSet<FString> Names = VariableNames(*Blueprint);

	TestTrue(TEXT("the first claimant keeps the bare name"), Names.Contains(TEXT("torso")));
	TestTrue(TEXT("the geom is told apart by being a geom"), Names.Contains(TEXT("torso_geom")));
	TestTrue(TEXT("the joint is told apart by being a joint"), Names.Contains(TEXT("torso_joint")));

	for (const TCHAR* Ordinal : {TEXT("torso_1"), TEXT("torso_2")})
	{
		TestFalse(*FString::Printf(TEXT("no named element is left on '%s'"), Ordinal), Names.Contains(Ordinal));
	}

	// The compiled model is untouched by any of this: the spec's name comes from
	// the element's own MJCF name field, and the naming pass never writes to it.
	TestEqual(TEXT("all three elements still carry the MJCF name they were authored with"),
		CountWithMjName(*Blueprint, TEXT("torso")), 3);

	const FString Mjcf = FSpecRef::OverBlueprint(*Blueprint).WriteMjcf();
	int32 Written = 0;
	int32 At = 0;
	while ((At = Mjcf.Find(TEXT("name=\"torso\""), ESearchCase::CaseSensitive, ESearchDir::FromStart, At)) >= 0)
	{
		++Written;
		At += 1;
	}
	TestEqual(TEXT("and the MJCF the spec writes still names all three 'torso'"), Written, 3);

	// The proof the item rests on, which the assertions above only imply: the
	// model this document compiles to through our path is byte for byte the
	// model MuJoCo's own reader compiles it to. Every element here is named, so
	// there is no generated name to explain and nothing to allow for -- the
	// comparison is the whole file. A suffix that had leaked into the spec would
	// move a name table and fail here.
	{
		using namespace MjParitySupport;

		urlab::spec::FMjBuiltSpec Built;
		mjModel* const Ours =
			CompileThroughSpecPath(*this, TEXT("clash"), FSpecRef::OverBlueprint(*Blueprint), Built);

		char Error[1024] = {0};
		mjSpec* const StockSpec = mj_parseXMLString(TCHAR_TO_UTF8(Xml), nullptr, Error, sizeof(Error));
		mjModel* const Stock = StockSpec != nullptr ? mj_compile(StockSpec, nullptr) : nullptr;
		if (StockSpec == nullptr)
		{
			AddError(FString::Printf(TEXT("stock refused the clash document: %s"), UTF8_TO_TCHAR(Error)));
		}

		if (Ours != nullptr && Stock != nullptr)
		{
			const TArray<uint8> OurBytes = ModelBytes(Ours);
			const TArray<uint8> StockBytes = ModelBytes(Stock);
			if (TestTrue(TEXT("both models serialise to something"), OurBytes.Num() > 0 && StockBytes.Num() > 0))
			{
				TestEqual(TEXT("our compiled model is the size stock's is"), OurBytes.Num(), StockBytes.Num());
				if (OurBytes.Num() == StockBytes.Num())
				{
					TestEqual(TEXT("and byte for byte the same model, suffixes and all"),
						FMemory::Memcmp(OurBytes.GetData(), StockBytes.GetData(), OurBytes.Num()), 0);
				}
			}
		}

		if (Stock != nullptr)
		{
			mj_deleteModel(Stock);
		}
		if (StockSpec != nullptr)
		{
			mj_deleteSpec(StockSpec);
		}
		if (Ours != nullptr)
		{
			mj_deleteModel(Ours);
		}
	}

	return !HasAnyErrors();
}

// ============================================================================
// URLab.Import.UnnamedElementsKeepTheirOrdinals
//   The guard on the change. An unnamed element has nothing but its tag to be
//   told apart by, so two of them under one parent still need a number -- and
//   that is the schema's answer, not a naming failure.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjUnnamedKeepOrdinalsTest, "URLab.Import.UnnamedElementsKeepTheirOrdinals",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjUnnamedKeepOrdinalsTest::RunTest(const FString& Parameters)
{
	using namespace MjNamingTests;

	const TCHAR* const Xml = TEXT(R"(<mujoco model="unnamed">
  <worldbody>
    <body name="link" pos="0 0 1">
      <geom type="box" size="0.1 0.1 0.1"/>
      <geom type="sphere" size="0.1" pos="0.3 0 0"/>
    </body>
  </worldbody>
</mujoco>
)");

	UBlueprint* Blueprint = ParseScratch(*this, Xml);
	if (Blueprint == nullptr)
	{
		return false;
	}
	const TSet<FString> Names = VariableNames(*Blueprint);

	TestTrue(TEXT("the first unnamed geom is shown as its tag"), Names.Contains(TEXT("geom")));
	TestTrue(TEXT("the second is shown as its tag and an ordinal"), Names.Contains(TEXT("geom_1")));

	return true;
}

#endif // URLAB_MJ_GEN && WITH_EDITOR
