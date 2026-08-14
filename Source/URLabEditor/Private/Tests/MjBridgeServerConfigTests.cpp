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
	if (!FPaths::FileExists(Path))
		return;
	const TCHAR* S = URLabBridgeServerConfigUtils::SectionName;
	FConfigFile File;
	File.Read(Path);
	bool TmpBool = false;
	int32 TmpInt = 0;
	if (File.GetBool(S, TEXT("AutoStart"), TmpBool))
		Out.bAutoStart = TmpBool;
	if (File.GetInt(S, TEXT("StepPort"), TmpInt))
		Out.StepPort = TmpInt;
	if (File.GetInt(S, TEXT("StatePort"), TmpInt))
		Out.StatePort = TmpInt;
	FString TmpStr;
	if (File.GetString(S, TEXT("InstanceId"), TmpStr))
		Out.InstanceId = TmpStr;
	if (File.GetInt(S, TEXT("InstanceIndex"), TmpInt))
		Out.InstanceIndex = TmpInt;
	if (File.GetInt(S, TEXT("PortBase"), TmpInt))
		Out.PortBase = TmpInt;
	if (File.GetInt(S, TEXT("PortStride"), TmpInt))
		Out.PortStride = TmpInt;
	if (File.GetInt(S, TEXT("CamBasePort"), TmpInt))
		Out.CamBasePort = TmpInt;
	if (File.GetString(S, TEXT("BindAddress"), TmpStr))
		Out.BindAddress = TmpStr;
	if (File.GetBool(S, TEXT("StopOnPIEEnd"), TmpBool))
		Out.bStopOnPIEEnd = TmpBool;
}

void SaveTo(const FString& Path, const FURLabBridgeServerConfig& In)
{
	IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), /*Tree=*/true);
	const TCHAR* S = URLabBridgeServerConfigUtils::SectionName;
	FConfigFile File;
	File.Read(Path);
	File.SetString(S, TEXT("AutoStart"), In.bAutoStart ? TEXT("True") : TEXT("False"));
	File.SetInt64(S, TEXT("StepPort"), In.StepPort);
	File.SetInt64(S, TEXT("StatePort"), In.StatePort);
	File.SetString(S, TEXT("InstanceId"), *In.InstanceId);
	File.SetInt64(S, TEXT("InstanceIndex"), In.InstanceIndex);
	File.SetInt64(S, TEXT("PortBase"), In.PortBase);
	File.SetInt64(S, TEXT("PortStride"), In.PortStride);
	File.SetInt64(S, TEXT("CamBasePort"), In.CamBasePort);
	File.SetString(S, TEXT("BindAddress"), *In.BindAddress);
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
} // namespace

// ---------------------------------------------------------------------------
// 1. Defaults: an unset struct + missing INI yields the documented defaults.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBridgeServerConfigDefaults,
	"URLab.BridgeServerConfig.Defaults",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBridgeServerConfigDefaults::RunTest(const FString& Parameters)
{
	FURLabBridgeServerConfig Cfg;
	TestTrue(TEXT("AutoStart default"), Cfg.bAutoStart);
	TestEqual(TEXT("StepPort default"), Cfg.StepPort, 5559);
	TestEqual(TEXT("StatePort default"), Cfg.StatePort, 5555);
	TestFalse(TEXT("StopOnPIEEnd default"), Cfg.bStopOnPIEEnd);

	// Round-trip vs missing path: leaves struct at defaults.
	const FString Path = MakeScratchIniPath(TEXT("missing"));
	IFileManager::Get().Delete(*Path, /*RequireExists=*/false, /*EvenReadOnly=*/true);
	LoadFrom(Path, Cfg);

	TestTrue(TEXT("Missing INI: AutoStart unchanged"), Cfg.bAutoStart);
	TestEqual(TEXT("Missing INI: StepPort unchanged"), Cfg.StepPort, 5559);
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
	Out.bAutoStart = false;
	Out.StepPort = 6001;
	Out.StatePort = 6002;
	Out.bStopOnPIEEnd = true;

	const FString Path = MakeScratchIniPath(TEXT("roundtrip"));
	IFileManager::Get().Delete(*Path, /*RequireExists=*/false, /*EvenReadOnly=*/true);

	SaveTo(Path, Out);
	TestTrue(TEXT("INI file written"), FPaths::FileExists(Path));

	FURLabBridgeServerConfig In;
	LoadFrom(Path, In);

	TestEqual(TEXT("AutoStart roundtrip"), In.bAutoStart, Out.bAutoStart);
	TestEqual(TEXT("StepPort roundtrip"), In.StepPort, Out.StepPort);
	TestEqual(TEXT("StatePort roundtrip"), In.StatePort, Out.StatePort);
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

	FURLabBridgeServerConfig Cfg; // all defaults
	LoadFrom(Path, Cfg);

	TestEqual(TEXT("StepPort honoured"), Cfg.StepPort, 7777);
	TestEqual(TEXT("StatePort default kept"), Cfg.StatePort, 5555);
	TestTrue(TEXT("AutoStart default kept"), Cfg.bAutoStart);
	TestFalse(TEXT("StopOnPIEEnd default kept"), Cfg.bStopOnPIEEnd);
	return true;
}

// ---------------------------------------------------------------------------
// 4. New per-instance fields survive a Save / Load round-trip.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBridgeServerConfigInstanceRoundTrip,
	"URLab.BridgeServerConfig.InstanceRoundTrip",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBridgeServerConfigInstanceRoundTrip::RunTest(const FString& Parameters)
{
	FURLabBridgeServerConfig Out;
	Out.InstanceId = TEXT("instance_7");
	Out.InstanceIndex = 7;
	Out.PortBase = 6000;
	Out.PortStride = 20;
	Out.CamBasePort = 6142;
	Out.BindAddress = TEXT("127.0.0.1");

	const FString Path = MakeScratchIniPath(TEXT("instance_roundtrip"));
	IFileManager::Get().Delete(*Path, /*RequireExists=*/false, /*EvenReadOnly=*/true);

	SaveTo(Path, Out);
	FURLabBridgeServerConfig In;
	LoadFrom(Path, In);

	TestEqual(TEXT("InstanceId roundtrip"), In.InstanceId, Out.InstanceId);
	TestEqual(TEXT("InstanceIndex roundtrip"), In.InstanceIndex, Out.InstanceIndex);
	TestEqual(TEXT("PortBase roundtrip"), In.PortBase, Out.PortBase);
	TestEqual(TEXT("PortStride roundtrip"), In.PortStride, Out.PortStride);
	TestEqual(TEXT("CamBasePort roundtrip"), In.CamBasePort, Out.CamBasePort);
	TestEqual(TEXT("BindAddress roundtrip"), In.BindAddress, Out.BindAddress);
	return true;
}

// ---------------------------------------------------------------------------
// 5. DerivePorts: farm index derives the strided port block; explicit ports
//    win; index < 0 preserves single-editor defaults.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBridgeServerConfigDerivePorts,
	"URLab.BridgeServerConfig.DerivePorts",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBridgeServerConfigDerivePorts::RunTest(const FString& Parameters)
{
	using namespace URLabBridgeServerConfigUtils;

	// Index 0, defaults (base 5559, stride 10): 5559 / 5560 / 5561.
	{
		FURLabBridgeServerConfig Cfg;
		Cfg.InstanceIndex = 0;
		DerivePorts(Cfg, /*Step=*/false, /*State=*/false, /*Cam=*/false);
		TestEqual(TEXT("index0 StepPort"), Cfg.StepPort, 5559);
		TestEqual(TEXT("index0 StatePort"), Cfg.StatePort, 5560);
		TestEqual(TEXT("index0 CamBasePort"), Cfg.CamBasePort, 5561);
		TestEqual(TEXT("index0 InstanceId derived"), Cfg.InstanceId, FString(TEXT("instance_0")));
	}

	// Index 3, base 5559, stride 10: 5589 / 5590 / 5591.
	{
		FURLabBridgeServerConfig Cfg;
		Cfg.InstanceIndex = 3;
		DerivePorts(Cfg, false, false, false);
		TestEqual(TEXT("index3 StepPort"), Cfg.StepPort, 5589);
		TestEqual(TEXT("index3 StatePort"), Cfg.StatePort, 5590);
		TestEqual(TEXT("index3 CamBasePort"), Cfg.CamBasePort, 5591);
	}

	// Explicit StepPort survives derivation; the rest still derive.
	{
		FURLabBridgeServerConfig Cfg;
		Cfg.InstanceIndex = 3;
		Cfg.StepPort = 7000;
		DerivePorts(Cfg, /*Step=*/true, /*State=*/false, /*Cam=*/false);
		TestEqual(TEXT("explicit StepPort kept"), Cfg.StepPort, 7000);
		TestEqual(TEXT("derived StatePort alongside explicit step"), Cfg.StatePort, 5590);
		TestEqual(TEXT("derived CamBasePort alongside explicit step"), Cfg.CamBasePort, 5591);
	}

	// Index < 0: single-editor defaults untouched; CamBasePort falls back to
	// StepPort + 2.
	{
		FURLabBridgeServerConfig Cfg; // InstanceIndex == -1 by default
		DerivePorts(Cfg, false, false, false);
		TestEqual(TEXT("no-index StepPort default"), Cfg.StepPort, 5559);
		TestEqual(TEXT("no-index StatePort default"), Cfg.StatePort, 5555);
		TestEqual(TEXT("no-index CamBasePort = StepPort+2"), Cfg.CamBasePort, 5561);
		TestTrue(TEXT("no-index InstanceId stays empty"), Cfg.InstanceId.IsEmpty());
	}

	// Explicit CamBasePort is not overwritten by the StepPort+2 fallback.
	{
		FURLabBridgeServerConfig Cfg;
		Cfg.CamBasePort = 5900;
		DerivePorts(Cfg, false, false, /*Cam=*/true);
		TestEqual(TEXT("explicit CamBasePort kept"), Cfg.CamBasePort, 5900);
	}

	return true;
}

// ---------------------------------------------------------------------------
// 6. ApplyEnvAndCommandLineOverrides derives ports from a farm index when no
//    env / command-line override is present in the test process. The explicit-
//    override precedence (env / -URLab* beating derivation) and BindAddress
//    override are validated in the live-launch integration test, since process
//    env / command line can't be set portably from a unit test.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBridgeServerConfigApplyOverrides,
	"URLab.BridgeServerConfig.ApplyOverrides",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBridgeServerConfigApplyOverrides::RunTest(const FString& Parameters)
{
	FURLabBridgeServerConfig Cfg;
	Cfg.InstanceIndex = 2; // as if -URLabInstanceIndex=2 / INI had set it
	URLabBridgeServerConfigUtils::ApplyEnvAndCommandLineOverrides(Cfg);

	// base 5559 + 2*10 = 5579 block.
	TestEqual(TEXT("apply index2 StepPort"), Cfg.StepPort, 5579);
	TestEqual(TEXT("apply index2 StatePort"), Cfg.StatePort, 5580);
	TestEqual(TEXT("apply index2 CamBasePort"), Cfg.CamBasePort, 5581);
	TestEqual(TEXT("apply index2 InstanceId"), Cfg.InstanceId, FString(TEXT("instance_2")));
	return true;
}

// ---------------------------------------------------------------------------
// 7. BuildCameraEndpoint: cameras allocate one port each, upward from
//    CamBasePort, on the configured BindAddress. Distinct instances (distinct
//    CamBasePort blocks) never overlap, which is what lets N editors stream
//    cameras concurrently.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBridgeServerConfigCameraEndpoint,
	"URLab.BridgeServerConfig.CameraEndpoint",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBridgeServerConfigCameraEndpoint::RunTest(const FString& Parameters)
{
	using namespace URLabBridgeServerConfigUtils;

	// Instance 0 (CamBasePort 5561): cameras land on 5561, 5562, 5563 ...
	{
		FURLabBridgeServerConfig Cfg;
		Cfg.InstanceIndex = 0;
		DerivePorts(Cfg, false, false, false); // CamBasePort -> 5561
		TestEqual(TEXT("inst0 cam0"), BuildCameraEndpoint(Cfg, 0), FString(TEXT("tcp://0.0.0.0:5561")));
		TestEqual(TEXT("inst0 cam1"), BuildCameraEndpoint(Cfg, 1), FString(TEXT("tcp://0.0.0.0:5562")));
		TestEqual(TEXT("inst0 cam2"), BuildCameraEndpoint(Cfg, 2), FString(TEXT("tcp://0.0.0.0:5563")));
	}

	// Instance 1 (CamBasePort 5571): its camera block is strictly above
	// instance 0's, so two instances never collide on a camera port.
	{
		FURLabBridgeServerConfig Cfg;
		Cfg.InstanceIndex = 1;
		DerivePorts(Cfg, false, false, false); // CamBasePort -> 5571
		TestEqual(TEXT("inst1 cam0"), BuildCameraEndpoint(Cfg, 0), FString(TEXT("tcp://0.0.0.0:5571")));
		TestEqual(TEXT("inst1 cam1"), BuildCameraEndpoint(Cfg, 1), FString(TEXT("tcp://0.0.0.0:5572")));
	}

	// BindAddress is honoured (consistent with the step/state sockets), and a
	// negative index clamps to the block base rather than underflowing.
	{
		FURLabBridgeServerConfig Cfg;
		Cfg.CamBasePort = 6000;
		Cfg.BindAddress = TEXT("127.0.0.1");
		TestEqual(TEXT("bind addr honoured"),
			BuildCameraEndpoint(Cfg, 0), FString(TEXT("tcp://127.0.0.1:6000")));
		TestEqual(TEXT("negative index clamps to base"),
			BuildCameraEndpoint(Cfg, -5), FString(TEXT("tcp://127.0.0.1:6000")));
	}

	return true;
}
