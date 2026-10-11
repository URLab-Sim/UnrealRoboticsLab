// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "MuJoCo/Spec/MjGenHooks.h"

#if URLAB_MJ_GEN && WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS

#include "MjUeXml.h"
#include "MuJoCo/Spec/MjSpecRef.h"

#include "Engine/Blueprint.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/FileManager.h"
#include "Kismet2/KismetEditorUtilities.h"
#include "Misc/AutomationTest.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "UObject/Package.h"

namespace
{
FString Diagnostics(const FMjSpecParseResult& Result)
{
	TArray<FString> Lines;
	for (const FMjSpecDiagnostic& Error : Result.Errors) Lines.Add(Error.ToString());
	return FString::Join(Lines, TEXT("; "));
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDirectUeReaderTest,
	"URLab.MuJoCo.DirectUeReader.ActorAndScs",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDirectUeReaderTest::RunTest(const FString& Parameters)
{
	const FString Scratch = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("URLabTests"),
		FGuid::NewGuid().ToString(EGuidFormats::Digits));
	IFileManager::Get().MakeDirectory(*Scratch, true);
	const FString PartFile = FPaths::Combine(Scratch, TEXT("part.xml"));
	const FString ModelFile = FPaths::Combine(Scratch, TEXT("model.xml"));
	FFileHelper::SaveStringToFile(TEXT("<mujoco><geom name='included' type='sphere' size='0.04'/></mujoco>"), *PartFile);
	const FString Xml = TEXT(R"(<mujoco model='direct'>
  <default><default class='colored'><geom rgba='0.2 0.3 0.4 1'/></default></default>
  <asset><texture name='tex' type='2d' builtin='flat' width='2' height='2'/><material name='mat' texture='tex'/></asset>
  <worldbody><body name='base' euler='0 0 1.5707963267948966'>
    <joint name='hinge' type='hinge' axis='0 0 1'/>
    <geom name='shape' class='colored' type='sphere' size='0.1' material='mat'/>
    <include file='part.xml'/>
  </body></worldbody>
  <actuator><position name='drive' joint='hinge' kp='10'/></actuator>
  <compiler angle='radian'/>
</mujoco>)");

	UWorld* World = UWorld::CreateWorld(EWorldType::Game, false);
	if (!TestNotNull(TEXT("test world"), World)) return false;
	FWorldContext& Context = GEngine->CreateNewWorldContext(EWorldType::Game);
	Context.SetCurrentWorld(World);
	AActor* Actor = World->SpawnActor<AActor>();
	const FMjSpecParseResult ActorResult = urlab::spec::io::ParseUeIntoInstance(*Actor, Xml, ModelFile, {});
	TestTrue(*FString::Printf(TEXT("actor direct reader: %s"), *Diagnostics(ActorResult)), ActorResult.IsOk());
	if (ActorResult.IsOk())
	{
		const FString Export = FSpecRef::OverActor(*Actor).WriteMjcf();
		TestTrue(TEXT("actor retains material texture alias"), Export.Contains(TEXT("texture=\"tex\"")));
		TestTrue(TEXT("actor retains joint reference"), Export.Contains(TEXT("joint=\"hinge\"")));
		TestTrue(TEXT("actor expands include"), Export.Contains(TEXT("name=\"included\"")));
		TestTrue(TEXT("actor normalizes euler to quaternion"), Export.Contains(TEXT("quat=\"")));
		TestFalse(TEXT("actor discards transient euler"), Export.Contains(TEXT("euler=")));
	}

	const FString BlueprintName = FString::Printf(TEXT("MjDirectReader_%s"),
		*FGuid::NewGuid().ToString(EGuidFormats::Digits));
	UPackage* Package = CreatePackage(*(TEXT("/Temp/") + BlueprintName));
	UBlueprint* Blueprint = FKismetEditorUtilities::CreateBlueprint(AActor::StaticClass(), Package,
		FName(*BlueprintName), BPTYPE_Normal, UBlueprint::StaticClass(), UBlueprintGeneratedClass::StaticClass());
	if (TestNotNull(TEXT("test Blueprint"), Blueprint))
	{
		const FMjSpecParseResult ScsResult = urlab::spec::io::ParseUeIntoScs(*Blueprint, Xml, ModelFile, {});
		TestTrue(*FString::Printf(TEXT("SCS direct reader: %s"), *Diagnostics(ScsResult)), ScsResult.IsOk());
		if (ScsResult.IsOk())
		{
			const FString Export = FSpecRef::OverBlueprint(*Blueprint).WriteMjcf();
			TestTrue(TEXT("SCS retains actuator subtype"), Export.Contains(TEXT("<position")));
			TestTrue(TEXT("SCS expands include"), Export.Contains(TEXT("name=\"included\"")));
		}
	}
	GEngine->DestroyWorldContext(World);
	World->DestroyWorld(false);
	IFileManager::Get().DeleteDirectory(*Scratch, false, true);
	return true;
}

#endif // URLAB_MJ_GEN && WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS
