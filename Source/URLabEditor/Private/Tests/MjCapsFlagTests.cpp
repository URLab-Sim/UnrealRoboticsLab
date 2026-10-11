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
// The launcher-flag surface, grounded in URLabLauncherFlags (MjLauncherFlags.cpp).
// Covers the flags a discovery/browser-join launch actually reads:
//
//   -URLabCaps=<capability>,...   (csv, negatable: -accept_input)
//   -URLabScene=...,overlay=<hex|dec>,maxcontacts=<n>,...
//   -URLabWorkflow                (including rejection of the retired drive flag)
//
// Every parser reads the process command line, so a test installs a synthetic one for the
// duration of the check and restores the real one after. The RAII helper belongs to the
// flag layer (URLabLauncherFlags::FScopedCommandLine), not to this file: the command line
// has exactly one reader and one writer in the plugin, and a test is not an exception.
// No process state leaks between tests.
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Bridge/BridgeServer.h"
#include "MuJoCo/Render/MjLauncherFlags.h"
#include "MuJoCo/Render/MjRendererBoot.h"
#include "Wire/UrlabWire.gen.h"

namespace
{
using FScopedCommandLine = URLabLauncherFlags::FScopedCommandLine;
} // namespace

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// -URLabCaps grammar: unnamed capabilities stay unnamed, positive tokens enable,
// a leading '-' negates. Absent flag => nothing named (lean default). The tokens
// are the capability names themselves, so there is no second vocabulary to keep
// in step with the first.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCapsParse,
	"URLab.Flags.CapsParse",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjCapsParse::RunTest(const FString& Parameters)
{
	// Every capability named, including both halves of each token that used to be
	// compound: `publish` was discoverable + pose_stream and `cameras` was
	// camera_frames + camera_rpc, which is exactly why they were two booleans apiece.
	{
		FScopedCommandLine Cmd(TEXT("-URLabCaps=rpc,discoverable,pose_stream,camera_frames,")
			TEXT("camera_rpc,accept_input,accept_model_load,free_fly_pawn"));
		const FMjCapabilitySet Caps = URLabLauncherFlags::ParseCaps();
		for (int32 i = 0; i < URLabWire::Capability::Count; ++i)
		{
			const EMjCapability Cap = static_cast<EMjCapability>(i);
			TestTrue(FString::Printf(TEXT("%s named on"), MjCapabilityName(Cap)),
				Caps.IsNamed(Cap) && Caps.Has(Cap));
		}
	}

	// Negation: only mentioned capabilities are named; -accept_input is named false.
	{
		FScopedCommandLine Cmd(TEXT("-URLabCaps=rpc,-accept_input"));
		const FMjCapabilitySet Caps = URLabLauncherFlags::ParseCaps();
		TestTrue(TEXT("rpc named on"),
			Caps.IsNamed(EMjCapability::Rpc) && Caps.Has(EMjCapability::Rpc));
		TestTrue(TEXT("accept_input is named"), Caps.IsNamed(EMjCapability::AcceptInput));
		TestFalse(TEXT("-accept_input negates to false"), Caps.Has(EMjCapability::AcceptInput));
		TestFalse(TEXT("unmentioned pose_stream stays unnamed"),
			Caps.IsNamed(EMjCapability::PoseStream));
		TestFalse(TEXT("unmentioned camera_frames stays unnamed"),
			Caps.IsNamed(EMjCapability::CameraFrames));
		TestFalse(TEXT("unmentioned free_fly_pawn stays unnamed"),
			Caps.IsNamed(EMjCapability::FreeFlyPawn));
	}

	// The two compound tokens really are gone rather than aliased: naming one of the
	// halves must not drag the other in with it.
	{
		FScopedCommandLine Cmd(TEXT("-URLabCaps=discoverable"));
		const FMjCapabilitySet Caps = URLabLauncherFlags::ParseCaps();
		TestTrue(TEXT("findable"), Caps.Has(EMjCapability::Discoverable));
		TestFalse(TEXT("and quiet"), Caps.Has(EMjCapability::PoseStream));
		TestFalse(TEXT("pose_stream unnamed"), Caps.IsNamed(EMjCapability::PoseStream));
	}
	{
		FScopedCommandLine Cmd(TEXT("-URLabCaps=camera_frames"));
		const FMjCapabilitySet Caps = URLabLauncherFlags::ParseCaps();
		TestTrue(TEXT("cameras built and streamed"), Caps.Has(EMjCapability::CameraFrames));
		TestFalse(TEXT("but the camera ops are not authorised by that alone"),
			Caps.IsNamed(EMjCapability::CameraRpc));
	}

	// A token nobody parses is ignored rather than silently enabling something. The old
	// spellings are exactly that now.
	{
		FScopedCommandLine Cmd(TEXT("-URLabCaps=serve,publish,cameras,input,vr"));
		AddExpectedError(TEXT("no capability is called"), EAutomationExpectedErrorFlags::Contains, 5);
		const FMjCapabilitySet Caps = URLabLauncherFlags::ParseCaps();
		TestEqual(TEXT("the deleted spellings name nothing"), (int32)Caps.Named, 0);
	}

	// Absent flag: lean default, nothing named.
	{
		FScopedCommandLine Cmd(TEXT("-SomeOtherFlag=1"));
		const FMjCapabilitySet Caps = URLabLauncherFlags::ParseCaps();
		TestEqual(TEXT("nothing named when the flag is absent"), (int32)Caps.Named, 0);
		TestEqual(TEXT("nothing enabled when the flag is absent"), (int32)Caps.Enabled, 0);
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
// URLab.Flags.SourceFindRetired
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSourceFind,
	"URLab.Flags.SourceFindRetired",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSourceFind::RunTest(const FString& Parameters)
{
	for (const TCHAR* Value : {TEXT("discover"), TEXT("discover:arena"), TEXT("browse")})
	{
		FScopedCommandLine Cmd(*FString::Printf(TEXT("-URLabSourceFind=%s"), Value));
		FString Parsed;
		TestTrue(TEXT("retired selector presence is detected"),
			URLabLauncherFlags::RetiredSourceFindValue(Parsed));
		TestEqual(TEXT("retired selector value is preserved for diagnostics"),
			Parsed, FString(Value));
		const FMjRendererBootPlan Plan = FMjRendererBoot::FromCommandLine();
		TestEqual(TEXT("retired selector is explicitly refused"),
			Plan.Action, EMjBootAction::Fail);
		TestTrue(TEXT("refusal names retired selector"), Plan.Error.Contains(TEXT("retired")));
	}
	FScopedCommandLine Absent(TEXT("-NoSourceFindHere"));
	FString Parsed;
	TestFalse(TEXT("absent retired selector stays absent"),
		URLabLauncherFlags::RetiredSourceFindValue(Parsed));

	return true;
}

// ---------------------------------------------------------------------------
// URLab.Flags.Drive
//
// Workflow selector classification: a viewer source keeps its scheme, and a
// value nobody recognises is refused rather than read as "flag absent".
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDriveAndModel,
	"URLab.Flags.Drive",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDriveAndModel::RunTest(const FString& Parameters)
{
 for(const auto& Case:{TPair<FString,URLabLauncherFlags::EWorkflowKind>(TEXT("simulation"),URLabLauncherFlags::EWorkflowKind::Simulation),{TEXT("render"),URLabLauncherFlags::EWorkflowKind::Render},{TEXT("viewer"),URLabLauncherFlags::EWorkflowKind::Viewer}}){FScopedCommandLine Cmd(*FString::Printf(TEXT("-URLabWorkflow=%s"),*Case.Key));FString Value;TestEqual(*Case.Key,URLabLauncherFlags::ParseWorkflow(Value),Case.Value);}
 {FScopedCommandLine Cmd(TEXT("-URLabDrive=sim"));FString Value;TestEqual(TEXT("old drive rejected"),URLabLauncherFlags::ParseWorkflow(Value),URLabLauncherFlags::EWorkflowKind::Invalid);}
 return !HasAnyErrors();
}

// ---------------------------------------------------------------------------
// URLab.Flags.EditorLaunchPlan
//
// There is one resolver now, and PIE and the cooked binary both boot from what it
// returns -- so this pins the reading itself rather than either launcher's copy of it.
// `subscribe:<ep>` is the render bus and never a control channel. gRPC subscribe is
// refused, and a named render drive with no model is a render server
// standing by rather than a refusal: every scene arrives over the wire now.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjEditorLaunchPlanResolve,
	"URLab.Flags.EditorLaunchPlan",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjEditorLaunchPlanResolve::RunTest(const FString& Parameters)
{
 {FScopedCommandLine Cmd(TEXT("-URLabWorkflow=simulation"));const auto P=FMjRendererBoot::FromCommandLine();TestEqual(TEXT("simulation action"),P.Action,EMjBootAction::Simulation);}
 {FScopedCommandLine Cmd(TEXT("-URLabWorkflow=render"));const auto P=FMjRendererBoot::FromCommandLine();TestEqual(TEXT("render action"),P.Action,EMjBootAction::Build);TestEqual(TEXT("render internal source"),P.Drive,EMjDrive::Request);}
 {FScopedCommandLine Cmd(TEXT("-URLabWorkflow=viewer"));const auto P=FMjRendererBoot::FromCommandLine();TestEqual(TEXT("viewer action"),P.Action,EMjBootAction::Viewer);TestTrue(TEXT("viewer has no implicit producer endpoint"),P.BusEndpoint.IsEmpty());}
 {FScopedCommandLine Cmd(TEXT("-URLabDrive=request"));const auto P=FMjRendererBoot::FromCommandLine();TestEqual(TEXT("legacy drive fails"),P.Action,EMjBootAction::Fail);}
 {FScopedCommandLine Cmd(TEXT(""));const auto P=FMjRendererBoot::FromCommandLine();TestEqual(TEXT("default preserved"),P.Action,EMjBootAction::None);}
 return !HasAnyErrors();
}

// ---------------------------------------------------------------------------
// URLab.Flags.BootPlanFields
//
// The plan is the whole boot, not just its action: drive, endpoints, model, the five
// capabilities, the scene keys and the net identity. Each of these used to be re-read
// from the command line at the point of use -- inside BeginPlay, inside the bridge
// config, inside the spawner -- which is what let one boot hold several answers to
// "what drives this instance".
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// URLab.Flags.BootPlanFields
//
// The plan is the whole boot, not just its action: drive, endpoints, capabilities, the
// scene keys and the net identity. Each of these used to be re-read from the command line
// at the point of use -- inside BeginPlay, inside the bridge config, inside the spawner --
// which is what let one boot hold several answers to "what drives this instance".
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBootPlanFields,
	"URLab.Flags.BootPlanFields",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjBootPlanFields::RunTest(const FString& Parameters)
{
	{
		FScopedCommandLine Cmd(TEXT("-URLabWorkflow=simulation ")
			TEXT("-URLabCaps=rpc,camera_frames,-accept_input ")
			TEXT("-URLabScene=level=/Game/Maps/Foo,origin=10;20;30,base,quality=off,")
			TEXT("cammax=720,camnear=2.5,noexposure,overlay=0x40,maxcontacts=64 ")
			TEXT("-URLabNet=id=rig7,index=3,grpc=50055,bind=0.0.0.0"));
		const FMjRendererBootPlan Plan = FMjRendererBoot::FromCommandLine();

		TestEqual(TEXT("sim owns physics"),
			static_cast<int32>(Plan.Drive), static_cast<int32>(EMjDrive::Sim));

		// Named capabilities resolve AND record that they were named; unnamed ones are lean
		// and stay unnamed, which is what lets an authored actor keep its own switch.
		TestTrue(TEXT("rpc named on"),
			Plan.Caps.Has(EMjCapability::Rpc) && Plan.Caps.IsNamed(EMjCapability::Rpc));
		TestTrue(TEXT("camera_frames named on"),
			Plan.Caps.Has(EMjCapability::CameraFrames)
				&& Plan.Caps.IsNamed(EMjCapability::CameraFrames));
		TestTrue(TEXT("accept_input named off"),
			!Plan.Caps.Has(EMjCapability::AcceptInput)
				&& Plan.Caps.IsNamed(EMjCapability::AcceptInput));
		TestFalse(TEXT("pose_stream unnamed"), Plan.Caps.IsNamed(EMjCapability::PoseStream));
		TestFalse(TEXT("pose_stream lean"), Plan.Caps.Has(EMjCapability::PoseStream));
		TestFalse(TEXT("free_fly_pawn unnamed"), Plan.Caps.IsNamed(EMjCapability::FreeFlyPawn));

		const FMjCapabilitySet Grants = FMjRendererBoot::ResolvePermissionPolicy(
			Plan, EMjPermissionProfile::Simulation);
		TestTrue(TEXT("simulation grants camera RPC by default"),
			Grants.Has(EMjCapability::CameraRpc));
		TestFalse(TEXT("accept_input honours the negation"),
			Grants.Has(EMjCapability::AcceptInput));
		TestTrue(TEXT("simulation grants model load by default"),
			Grants.Has(EMjCapability::AcceptModelLoad));

		TestEqual(TEXT("level"), Plan.Scene.LevelPath, FString(TEXT("/Game/Maps/Foo")));
		TestEqual(TEXT("origin"), Plan.Scene.Origin, FVector(10.0, 20.0, 30.0));
		TestTrue(TEXT("base level"), Plan.Scene.bBaseLevel);
		TestTrue(TEXT("quality off"), Plan.Scene.bNoQuality);
		TestEqual(TEXT("cammax"), Plan.Scene.CamMaxHeight, 720);
		TestEqual(TEXT("hex overlay mask"), Plan.Scene.OverlayMask, 0x40);
		TestEqual(TEXT("maxcontacts"), Plan.Scene.OverlayMaxContacts, 64);

		TestEqual(TEXT("net id"), Plan.Net.InstanceId, FString(TEXT("rig7")));
		TestEqual(TEXT("net index"), Plan.Net.InstanceIndex, 3);
		TestEqual(TEXT("net grpc port"), Plan.Net.GrpcPort, 50055);
		TestEqual(TEXT("net bind"), Plan.Net.BindAddress, FString(TEXT("0.0.0.0")));
	}

	// A render server gets the model-load permission by default, because a scene can reach
	// it no other way -- which is what makes the gate safe to have at all.
	{
		FScopedCommandLine Cmd(TEXT("-URLabWorkflow=render -URLabCaps=rpc"));
		const FMjRendererBootPlan Plan = FMjRendererBoot::FromCommandLine();
		const FMjCapabilitySet Grants = FMjRendererBoot::ResolvePermissionPolicy(
			Plan, EMjPermissionProfile::Render);
		TestTrue(TEXT("a render server exists to be loaded into"),
			Grants.Has(EMjCapability::AcceptModelLoad));
	}

	// The absent sentinels matter as much as the values: they are what "leave whatever the
	// level authored" means downstream.
	{
		FScopedCommandLine Cmd(TEXT("-URLabWorkflow=render"));
		const FMjRendererBootPlan Plan = FMjRendererBoot::FromCommandLine();
		TestEqual(TEXT("no cammax"), Plan.Scene.CamMaxHeight, -1);
		TestEqual(TEXT("no overlay"), Plan.Scene.OverlayMask, 0);
		TestEqual(TEXT("no net index"), Plan.Net.InstanceIndex, -1);
		TestEqual(TEXT("no grpc port"), Plan.Net.GrpcPort, 0);
		TestFalse(TEXT("camera_frames unnamed"), Plan.Caps.IsNamed(EMjCapability::CameraFrames));
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjWorkflowPermissionPolicy,
	"URLab.Flags.WorkflowPermissionPolicy",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjWorkflowPermissionPolicy::RunTest(const FString& Parameters)
{
	FMjRendererBootPlan Plan;
	const FMjCapabilitySet Simulation = FMjRendererBoot::ResolvePermissionPolicy(
		Plan, EMjPermissionProfile::Simulation);
	TestTrue(TEXT("simulation resolves camera RPC explicitly"),
		Simulation.IsNamed(EMjCapability::CameraRpc));
	TestTrue(TEXT("simulation resolves input explicitly"),
		Simulation.IsNamed(EMjCapability::AcceptInput));
	TestTrue(TEXT("simulation resolves model load explicitly"),
		Simulation.IsNamed(EMjCapability::AcceptModelLoad));
	TestTrue(TEXT("simulation camera RPC default"), Simulation.Has(EMjCapability::CameraRpc));
	TestTrue(TEXT("simulation input default"), Simulation.Has(EMjCapability::AcceptInput));
	TestTrue(TEXT("simulation model-load default"),
		Simulation.Has(EMjCapability::AcceptModelLoad));

	const FMjCapabilitySet Render = FMjRendererBoot::ResolvePermissionPolicy(
		Plan, EMjPermissionProfile::Render);
	TestTrue(TEXT("render camera RPC default"), Render.Has(EMjCapability::CameraRpc));
	TestFalse(TEXT("render input default"), Render.Has(EMjCapability::AcceptInput));
	TestTrue(TEXT("render model-load default"), Render.Has(EMjCapability::AcceptModelLoad));

	Plan.Caps.Set(EMjCapability::CameraFrames, true);
	Plan.Caps.Set(EMjCapability::CameraRpc, false);
	Plan.Caps.Set(EMjCapability::AcceptInput, true);
	Plan.Caps.Set(EMjCapability::AcceptModelLoad, false);
	const FMjCapabilitySet Overridden = FMjRendererBoot::ResolvePermissionPolicy(
		Plan, EMjPermissionProfile::Render);
	TestFalse(TEXT("camera frames never authorise camera RPC"),
		Overridden.Has(EMjCapability::CameraRpc));
	TestTrue(TEXT("named input overrides render default"),
		Overridden.Has(EMjCapability::AcceptInput));
	TestFalse(TEXT("named model-load negation overrides render default"),
		Overridden.Has(EMjCapability::AcceptModelLoad));

	UURLabBridgeServer* Bridge = NewObject<UURLabBridgeServer>();
	if (!TestNotNull(TEXT("bridge"), Bridge)) return false;
	TestFalse(TEXT("unactivated bridge denies camera RPC"),
		Bridge->GrantsPermission(EMjCapability::CameraRpc));
	TestFalse(TEXT("unactivated bridge denies input"),
		Bridge->GrantsPermission(EMjCapability::AcceptInput));
	TestFalse(TEXT("unactivated bridge denies model load"),
		Bridge->GrantsPermission(EMjCapability::AcceptModelLoad));

	Bridge->SetGrants(Simulation);
	TestTrue(TEXT("simulation policy installed"),
		Bridge->GrantsPermission(EMjCapability::AcceptInput));
	Bridge->SetGrants(Render);
	TestFalse(TEXT("render policy clears the previous input grant"),
		Bridge->GrantsPermission(EMjCapability::AcceptInput));
	FMjCapabilitySet Empty;
	Bridge->SetGrants(Empty);
	TestFalse(TEXT("complete replacement clears camera RPC"),
		Bridge->GrantsPermission(EMjCapability::CameraRpc));
	TestFalse(TEXT("complete replacement clears model load"),
		Bridge->GrantsPermission(EMjCapability::AcceptModelLoad));
	return true;
}
