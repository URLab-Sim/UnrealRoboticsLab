// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#if WITH_EDITOR

#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "MjNativeDependencyHelper.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjNativeDependencyHelperTest,
	"URLab.MuJoCo.SourceBundle.NativeDependencyIsolation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjNativeDependencyHelperTest::RunTest(const FString& Parameters)
{
	const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("UnrealRoboticsLab"));
	if (!TestTrue(TEXT("plugin available"), Plugin.IsValid())) return false;
	const FString TestDir = FPaths::Combine(Plugin->GetBaseDir(),
		TEXT("third_party/MuJoCo/src/test/xml/testdata"));
	const FString Parent = FPaths::Combine(TestDir, TEXT("parent_model.xml"));
	const FString Child = FPaths::Combine(TestDir, TEXT("child.xml"));
	if (!TestTrue(TEXT("nested model fixture available"), FPaths::FileExists(Parent))) return false;
	TArray<FString> Paths;
	FString Error;
	if (!TestTrue(TEXT("native recursive scan succeeds"), MjDiscoverNativeDependencies(Parent, Paths, Error)))
	{
		AddError(Error);
		return false;
	}
	TestTrue(TEXT("root document retained"), Paths.Contains(Parent));
	TestTrue(TEXT("nested model document retained"), Paths.Contains(Child));

	const FString Missing = FPaths::Combine(TestDir, TEXT("urlab_missing_dependency_probe.xml"));
	Paths.Reset();
	Error.Reset();
	TestFalse(TEXT("missing file fails in helper process"), MjDiscoverNativeDependencies(Missing, Paths, Error));
	TestTrue(TEXT("missing-file failure is explicit"), Error.Contains(TEXT("failed")));
	TestTrue(TEXT("no partial dependency list escapes"), Paths.IsEmpty());
	return !HasAnyErrors();
}

#endif  // WITH_EDITOR
