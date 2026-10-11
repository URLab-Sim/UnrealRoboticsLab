// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR

#include "Engine/Blueprint.h"
#include "GameFramework/Actor.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/Guid.h"
#include "UObject/Package.h"

#include "MuJoCo/Spec/MjSpecRef.h"

THIRD_PARTY_INCLUDES_START
#include <mujoco/mujoco.h>
THIRD_PARTY_INCLUDES_END

namespace
{
UBlueprint* ScratchBlueprint()
{
	const FString Name = FString::Printf(TEXT("MjAliasAttrs_%s"), *FGuid::NewGuid().ToString(EGuidFormats::Digits));
	UPackage* Package = CreatePackage(*(TEXT("/Temp/") + Name));
	return FKismetEditorUtilities::CreateBlueprint(AActor::StaticClass(), Package, FName(*Name),
		BPTYPE_Normal, UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
}

mjModel* CompileXml(const FString& Xml)
{
	char Error[1024]{};
	mjSpec* Spec = mj_parseXMLString(TCHAR_TO_UTF8(*Xml), nullptr, Error, sizeof(Error));
	if (Spec == nullptr) return nullptr;
	mjModel* Model = mj_compile(Spec, nullptr);
	mj_deleteSpec(Spec);
	return Model;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjAliasIgnoredAttributeTest,
	"URLab.MuJoCo.DirectUeReader.NativeAliasIgnoredAttributes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjAliasIgnoredAttributeTest::RunTest(const FString& Parameters)
{
	const FString Source = TEXT(R"(<mujoco>
  <worldbody>
    <frame ignored_frame="legacy"><geom name="frame_geom" size=".1"/></frame>
    <replicate count="2" prefix="legacy" ignored_replicate="legacy">
      <body name="replicated"><geom size=".1"/></body>
    </replicate>
  </worldbody>
</mujoco>)");
	mjModel* Native = CompileXml(Source);
	if (!TestNotNull(TEXT("MuJoCo accepts ignored alias attributes"), Native)) return false;

	UBlueprint* Blueprint = ScratchBlueprint();
	if (!TestNotNull(TEXT("scratch Blueprint"), Blueprint)) { mj_deleteModel(Native); return false; }
	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, Source, TEXT("<inline>"));
	if (!TestTrue(TEXT("direct UE reader accepts native-ignored alias attributes"), Parsed.IsOk()))
	{
		for (const FMjSpecDiagnostic& D : Parsed.Errors) AddError(D.ToString());
		mj_deleteModel(Native);
		return false;
	}
	TArray<FMjSpecDiagnostic> WriteErrors;
	const FString Written = FSpecRef::OverBlueprint(*Blueprint).WriteMjcf(&WriteErrors);
	for (const FMjSpecDiagnostic& D : WriteErrors) AddError(D.ToString());
	TestFalse(TEXT("ignored frame spelling is absent"), Written.Contains(TEXT("ignored_frame=")));
	TestFalse(TEXT("ignored replicate spelling is absent"), Written.Contains(TEXT("ignored_replicate=")));
	TestFalse(TEXT("native-ignored prefix is absent"), Written.Contains(TEXT("prefix=")));
	mjModel* Exported = CompileXml(Written);
	if (TestNotNull(TEXT("exported XML compiles"), Exported))
	{
		TestEqual(TEXT("body count"), Exported->nbody, Native->nbody);
		TestEqual(TEXT("geom count"), Exported->ngeom, Native->ngeom);
		mj_deleteModel(Exported);
	}
	mj_deleteModel(Native);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjOrdinaryUnknownAttributeTest,
	"URLab.MuJoCo.DirectUeReader.OrdinaryUnknownAttribute",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjOrdinaryUnknownAttributeTest::RunTest(const FString& Parameters)
{
	const FString Source = TEXT("<mujoco><worldbody><geom size='.1' unexplained='x'/></worldbody></mujoco>");
	mjModel* Native = CompileXml(Source);
	TestNull(TEXT("MuJoCo rejects unknown ordinary attribute"), Native);
	if (Native != nullptr) mj_deleteModel(Native);
	UBlueprint* Blueprint = ScratchBlueprint();
	if (!TestNotNull(TEXT("scratch Blueprint"), Blueprint)) return false;
	const FMjSpecParseResult Parsed = MjParseIntoBlueprint(*Blueprint, Source, TEXT("<inline>"));
	TestFalse(TEXT("direct UE reader rejects unknown ordinary attribute"), Parsed.IsOk());
	bool bFound = false;
	for (const FMjSpecDiagnostic& D : Parsed.Errors)
	{
		bFound |= D.Message.Contains(TEXT("unknown MJCF attribute 'unexplained' on <geom>"));
	}
	TestTrue(TEXT("explicit geom diagnostic"), bFound);
	return !HasAnyErrors();
}

#endif  // URLAB_MJ_GEN && WITH_EDITOR
