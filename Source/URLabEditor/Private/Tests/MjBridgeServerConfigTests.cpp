// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

// Tests for FURLabBridgeServerConfig + URLabBridgeServerConfigUtils.
//
// We don't want to clobber the user's actual LocalUnrealRoboticsLab.ini, so
// each test uses a temp scratch INI in <ProjectSaved>/URLabTest/ via the
// GConfig API directly. The utils functions also work against arbitrary
// paths thanks to GConfig's filename-based caching.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"
#include "Misc/ConfigCacheIni.h"
#include "HAL/FileManager.h"

#include "Bridge/BridgeServerConfig.h"
#include "Bridge/BridgeServerConfigUtils.h"

namespace
{
    /** Caller-supplied-path variant of LoadFromIni / SaveToIni. The public
     *  utils target the plugin INI; here we use FConfigFile directly so we
     *  can test against scratch paths without clobbering the user's real
     *  config. */
    void LoadFrom(const FString& Path, FURLabBridgeServerConfig& Out)
    {
        if (!FPaths::FileExists(Path)) return;
        const TCHAR* S = URLabBridgeServerConfigUtils::SectionName;
        FConfigFile File;
        File.Read(Path);
        bool TmpBool = false;
        int32 TmpInt = 0;
        if (File.GetBool(S, TEXT("AutoStart"),    TmpBool)) Out.bAutoStart    = TmpBool;
        if (File.GetInt (S, TEXT("StepPort"),     TmpInt))  Out.StepPort      = TmpInt;
        if (File.GetInt (S, TEXT("StatePort"),    TmpInt))  Out.StatePort     = TmpInt;
        if (File.GetBool(S, TEXT("StopOnPIEEnd"), TmpBool)) Out.bStopOnPIEEnd = TmpBool;
    }

    void SaveTo(const FString& Path, const FURLabBridgeServerConfig& In)
    {
        IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), /*Tree=*/true);
        const TCHAR* S = URLabBridgeServerConfigUtils::SectionName;
        FConfigFile File;
        File.Read(Path);
        File.SetString(S, TEXT("AutoStart"),    In.bAutoStart    ? TEXT("True") : TEXT("False"));
        File.SetInt64 (S, TEXT("StepPort"),     In.StepPort);
        File.SetInt64 (S, TEXT("StatePort"),    In.StatePort);
        File.SetString(S, TEXT("StopOnPIEEnd"), In.bStopOnPIEEnd ? TEXT("True") : TEXT("False"));
        File.Dirty = true;
        File.Write(Path);
    }

    FString MakeScratchIniPath(const FString& Tag)
    {
        const FString Dir = FPaths::ConvertRelativePathToFull(
            FPaths::ProjectSavedDir() / TEXT("URLabTest"));
        return Dir / FString::Printf(TEXT("BridgeServerConfig_%s.ini"), *Tag);
    }
}

// ---------------------------------------------------------------------------
// 1. Defaults: an unset struct + missing INI yields the documented defaults.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBridgeServerConfigDefaults,
    "URLab.BridgeServerConfig.Defaults",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBridgeServerConfigDefaults::RunTest(const FString& Parameters)
{
    FURLabBridgeServerConfig Cfg;
    TestTrue (TEXT("AutoStart default"),     Cfg.bAutoStart);
    TestEqual(TEXT("StepPort default"),      Cfg.StepPort,  5559);
    TestEqual(TEXT("StatePort default"),     Cfg.StatePort, 5555);
    TestFalse(TEXT("StopOnPIEEnd default"),  Cfg.bStopOnPIEEnd);

    // Round-trip vs missing path: leaves struct at defaults.
    const FString Path = MakeScratchIniPath(TEXT("missing"));
    IFileManager::Get().Delete(*Path, /*RequireExists=*/false, /*EvenReadOnly=*/true);
    LoadFrom(Path, Cfg);

    TestTrue (TEXT("Missing INI: AutoStart unchanged"),    Cfg.bAutoStart);
    TestEqual(TEXT("Missing INI: StepPort unchanged"),     Cfg.StepPort, 5559);
    return true;
}

// ---------------------------------------------------------------------------
// 2. Save / load round-trip preserves every field.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBridgeServerConfigRoundTrip,
    "URLab.BridgeServerConfig.RoundTrip",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBridgeServerConfigRoundTrip::RunTest(const FString& Parameters)
{
    FURLabBridgeServerConfig Out;
    Out.bAutoStart    = false;
    Out.StepPort      = 6001;
    Out.StatePort     = 6002;
    Out.bStopOnPIEEnd = true;

    const FString Path = MakeScratchIniPath(TEXT("roundtrip"));
    IFileManager::Get().Delete(*Path, /*RequireExists=*/false, /*EvenReadOnly=*/true);

    SaveTo(Path, Out);
    TestTrue(TEXT("INI file written"), FPaths::FileExists(Path));

    FURLabBridgeServerConfig In;
    LoadFrom(Path, In);

    TestEqual(TEXT("AutoStart roundtrip"),    In.bAutoStart,    Out.bAutoStart);
    TestEqual(TEXT("StepPort roundtrip"),     In.StepPort,      Out.StepPort);
    TestEqual(TEXT("StatePort roundtrip"),    In.StatePort,     Out.StatePort);
    TestEqual(TEXT("StopOnPIEEnd roundtrip"), In.bStopOnPIEEnd, Out.bStopOnPIEEnd);
    return true;
}

// ---------------------------------------------------------------------------
// 3. Partial INI: keys that aren't written keep the struct's default.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBridgeServerConfigPartial,
    "URLab.BridgeServerConfig.PartialIni",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBridgeServerConfigPartial::RunTest(const FString& Parameters)
{
    const FString Path = MakeScratchIniPath(TEXT("partial"));
    IFileManager::Get().Delete(*Path, /*RequireExists=*/false, /*EvenReadOnly=*/true);
    IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), /*Tree=*/true);

    // Write only StepPort; the rest stay default in the struct.
    const TCHAR* S = URLabBridgeServerConfigUtils::SectionName;
    {
        FConfigFile File;
        File.SetInt64(S, TEXT("StepPort"), 7777);
        File.Dirty = true;
        File.Write(Path);
    }

    FURLabBridgeServerConfig Cfg;  // all defaults
    LoadFrom(Path, Cfg);

    TestEqual(TEXT("StepPort honoured"),        Cfg.StepPort,     7777);
    TestEqual(TEXT("StatePort default kept"),   Cfg.StatePort,    5555);
    TestTrue (TEXT("AutoStart default kept"),   Cfg.bAutoStart);
    TestFalse(TEXT("StopOnPIEEnd default kept"),Cfg.bStopOnPIEEnd);
    return true;
}
