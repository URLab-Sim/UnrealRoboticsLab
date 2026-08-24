// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc.

// ============================================================================
// MjCapsFlagTests.cpp
//
// The Phase 1.1 launcher-flag surface (source-of-truth §14; capability model §5),
// grounded in URLabLauncherFlags (MjLauncherFlags.cpp). Covers the flags a
// discovery/browser-join launch actually reads:
//
//   -URLabCaps=serve,publish,cameras,input,vr   (csv, negatable: -input)   [§5]
//   -URLabScene=...,overlay=<hex|dec>,maxcontacts=<n>,...                   [§14]
//   -URLabSourceFind=discover[:scene] | browse                             [§14]
//   -URLabDrive / -URLabModel  (a light check of the sibling readers)      [§14]
//
// Each parser reads FCommandLine::Get() directly, so a test installs a synthetic
// command line for the duration of the check and restores the real one after
// (FScopedCommandLine). No process state leaks between tests.
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Misc/CommandLine.h"
#include "MuJoCo/Fast/MjLauncherFlags.h"

namespace
{
/** RAII override of the process command line, restoring the original on scope exit
 *  so the launcher-flag parsers (which all read FCommandLine::Get()) can be driven
 *  from a synthetic line without disturbing the rest of the automation run. */
struct FScopedCommandLine
{
	FString Saved;

	explicit FScopedCommandLine(const TCHAR* Line)
	{
		Saved = FCommandLine::Get();
		FCommandLine::Set(Line);
	}
	~FScopedCommandLine()
	{
		FCommandLine::Set(*Saved);
	}
};
} // namespace

// ---------------------------------------------------------------------------
// URLab.Flags.CapsParse
//
// -URLabCaps grammar (§5): unset caps stay unset (TOptional not set), positive
// tokens enable, a leading '-' negates. Absent flag => nothing set (lean default).
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCapsParse,
	"URLab.Flags.CapsParse",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjCapsParse::RunTest(const FString& Parameters)
{
	// All five enabled.
	{
		FScopedCommandLine Cmd(TEXT("-URLabCaps=serve,publish,cameras,input,vr"));
		const URLabLauncherFlags::FCaps Caps = URLabLauncherFlags::ParseCaps();
		TestTrue(TEXT("serve set+true"), Caps.bServe.IsSet() && Caps.bServe.GetValue());
		TestTrue(TEXT("publish set+true"), Caps.bPublish.IsSet() && Caps.bPublish.GetValue());
		TestTrue(TEXT("cameras set+true"), Caps.bCameras.IsSet() && Caps.bCameras.GetValue());
		TestTrue(TEXT("input set+true"), Caps.bInput.IsSet() && Caps.bInput.GetValue());
		TestTrue(TEXT("vr set+true"), Caps.bVr.IsSet() && Caps.bVr.GetValue());
		TestTrue(TEXT("CapsWantVr() true when vr requested"), URLabLauncherFlags::CapsWantVr());
	}

	// Negation: only mentioned caps are set; -input is set-to-false; others unset.
	{
		FScopedCommandLine Cmd(TEXT("-URLabCaps=serve,-input"));
		const URLabLauncherFlags::FCaps Caps = URLabLauncherFlags::ParseCaps();
		TestTrue(TEXT("serve set+true"), Caps.bServe.IsSet() && Caps.bServe.GetValue());
		TestTrue(TEXT("input is set"), Caps.bInput.IsSet());
		TestFalse(TEXT("-input negates to false"), Caps.bInput.GetValue());
		TestFalse(TEXT("unmentioned publish stays unset"), Caps.bPublish.IsSet());
		TestFalse(TEXT("unmentioned cameras stays unset"), Caps.bCameras.IsSet());
		TestFalse(TEXT("unmentioned vr stays unset"), Caps.bVr.IsSet());
		TestFalse(TEXT("CapsWantVr() false when vr absent"), URLabLauncherFlags::CapsWantVr());
	}

	// Absent flag: lean default, nothing set.
	{
		FScopedCommandLine Cmd(TEXT("-SomeOtherFlag=1"));
		const URLabLauncherFlags::FCaps Caps = URLabLauncherFlags::ParseCaps();
		TestFalse(TEXT("serve unset when flag absent"), Caps.bServe.IsSet());
		TestFalse(TEXT("publish unset when flag absent"), Caps.bPublish.IsSet());
		TestFalse(TEXT("cameras unset when flag absent"), Caps.bCameras.IsSet());
		TestFalse(TEXT("input unset when flag absent"), Caps.bInput.IsSet());
		TestFalse(TEXT("vr unset when flag absent"), Caps.bVr.IsSet());
		TestFalse(TEXT("CapsWantVr() false when flag absent"), URLabLauncherFlags::CapsWantVr());
	}

	return true;
}

// ---------------------------------------------------------------------------
// URLab.Flags.SceneOverlayMask
//
// -URLabScene=overlay=<mask> parses a decimal or 0x-hex mjVIS_* bitmask, and
// coexists with sibling scene keys (level, maxcontacts) in the same csv value.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSceneOverlayMask,
	"URLab.Flags.SceneOverlayMask",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSceneOverlayMask::RunTest(const FString& Parameters)
{
	// Hex mask.
	{
		FScopedCommandLine Cmd(TEXT("-URLabScene=overlay=0x2a"));
		int32 Mask = -1;
		TestTrue(TEXT("hex overlay parses"), URLabLauncherFlags::SceneOverlayMask(Mask));
		TestEqual(TEXT("0x2a -> 42"), Mask, 42);
	}

	// Decimal mask.
	{
		FScopedCommandLine Cmd(TEXT("-URLabScene=overlay=16"));
		int32 Mask = -1;
		TestTrue(TEXT("decimal overlay parses"), URLabLauncherFlags::SceneOverlayMask(Mask));
		TestEqual(TEXT("16 -> 16"), Mask, 16);
	}

	// Overlay coexisting with other csv keys; maxcontacts+level read from the same value.
	{
		FScopedCommandLine Cmd(TEXT("-URLabScene=level=3,overlay=0x10,maxcontacts=8"));
		int32 Mask = -1;
		TestTrue(TEXT("overlay parses amid other keys"), URLabLauncherFlags::SceneOverlayMask(Mask));
		TestEqual(TEXT("0x10 -> 16"), Mask, 16);

		int32 MaxContacts = -1;
		TestTrue(TEXT("maxcontacts parses"), URLabLauncherFlags::SceneOverlayMaxContacts(MaxContacts));
		TestEqual(TEXT("maxcontacts=8"), MaxContacts, 8);

		FString Level;
		TestTrue(TEXT("level parses"), URLabLauncherFlags::SceneLevel(Level));
		TestEqual(TEXT("level=3"), Level, FString(TEXT("3")));
	}

	// Absent overlay key => not set.
	{
		FScopedCommandLine Cmd(TEXT("-URLabScene=level=1"));
		int32 Mask = -1;
		TestFalse(TEXT("overlay absent -> false"), URLabLauncherFlags::SceneOverlayMask(Mask));
	}

	return true;
}

// ---------------------------------------------------------------------------
// URLab.Flags.SourceFind
//
// -URLabSourceFind=discover[:scene] | browse — the two mutually exclusive
// source-discovery modes a browser-join launch selects between (§14).
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSourceFind,
	"URLab.Flags.SourceFind",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSourceFind::RunTest(const FString& Parameters)
{
	// Bare discover: discover true, no scene filter, browse false.
	{
		FScopedCommandLine Cmd(TEXT("-URLabSourceFind=discover"));
		FString Scene = TEXT("dirty");
		TestTrue(TEXT("discover recognised"), URLabLauncherFlags::SourceFindDiscover(Scene));
		TestTrue(TEXT("bare discover has empty scene filter"), Scene.IsEmpty());
		TestFalse(TEXT("browse false for discover"), URLabLauncherFlags::SourceFindBrowse());
	}

	// Scene-filtered discover.
	{
		FScopedCommandLine Cmd(TEXT("-URLabSourceFind=discover:arena"));
		FString Scene;
		TestTrue(TEXT("discover:scene recognised"), URLabLauncherFlags::SourceFindDiscover(Scene));
		TestEqual(TEXT("scene filter captured"), Scene, FString(TEXT("arena")));
	}

	// Browse mode.
	{
		FScopedCommandLine Cmd(TEXT("-URLabSourceFind=browse"));
		TestTrue(TEXT("browse recognised"), URLabLauncherFlags::SourceFindBrowse());
		FString Scene;
		TestFalse(TEXT("discover false for browse"), URLabLauncherFlags::SourceFindDiscover(Scene));
	}

	// Absent flag: neither mode.
	{
		FScopedCommandLine Cmd(TEXT("-NoSourceFindHere"));
		FString Scene;
		TestFalse(TEXT("discover false when flag absent"), URLabLauncherFlags::SourceFindDiscover(Scene));
		TestFalse(TEXT("browse false when flag absent"), URLabLauncherFlags::SourceFindBrowse());
	}

	return true;
}

// ---------------------------------------------------------------------------
// URLab.Flags.DriveAndModel
//
// Light coverage of the sibling launcher readers a joiner also consults:
// -URLabDrive scheme classification (stream endpoint keeps its scheme) and
// -URLabModel format-by-extension.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDriveAndModel,
	"URLab.Flags.DriveAndModel",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDriveAndModel::RunTest(const FString& Parameters)
{
	// -URLabDrive=sim
	{
		FScopedCommandLine Cmd(TEXT("-URLabDrive=sim"));
		TestTrue(TEXT("drive=sim"), URLabLauncherFlags::DriveIsSim());
		TestFalse(TEXT("drive!=push"), URLabLauncherFlags::DriveIsPush());
	}

	// -URLabDrive=stream:grpc://... keeps the grpc:// scheme.
	{
		FScopedCommandLine Cmd(TEXT("-URLabDrive=stream:grpc://127.0.0.1:50051"));
		FString Ep;
		TestTrue(TEXT("stream endpoint present"), URLabLauncherFlags::DriveStreamEndpoint(Ep));
		TestEqual(TEXT("endpoint preserves scheme"), Ep, FString(TEXT("grpc://127.0.0.1:50051")));
		FString GrpcEp;
		TestTrue(TEXT("classified as grpc stream"), URLabLauncherFlags::DriveStreamGrpcEndpoint(GrpcEp));
		FString TcpEp;
		TestFalse(TEXT("not a tcp stream"), URLabLauncherFlags::DriveStreamTcpEndpoint(TcpEp));
	}

	// -URLabModel format from extension.
	{
		FScopedCommandLine Cmd(TEXT("-URLabModel=/data/scene.mjz"));
		FString Path, Format;
		TestTrue(TEXT("model parses"), URLabLauncherFlags::ParseModel(Path, Format));
		TestEqual(TEXT("path captured"), Path, FString(TEXT("/data/scene.mjz")));
		TestEqual(TEXT("mjz format from extension"), Format, FString(TEXT("mjz")));
	}
	{
		FScopedCommandLine Cmd(TEXT("-URLabModel=/data/scene.xml"));
		FString Path, Format;
		TestTrue(TEXT("model parses"), URLabLauncherFlags::ParseModel(Path, Format));
		TestEqual(TEXT("xml format from extension"), Format, FString(TEXT("xml")));
	}

	return true;
}
