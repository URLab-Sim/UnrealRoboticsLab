// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "MuJoCo/Render/MjRendererBoot.h"

#include "MuJoCo/Render/MjLauncherFlags.h"
#include "Wire/UrlabWire.gen.h"

namespace
{
/** -URLabScene, resolved. Every key keeps its "absent" sentinel so a value authored on
 *  the actor is only overridden when the operator actually named the key. */
FMjBootScene ResolveScene()
{
	FMjBootScene Scene;
	URLabLauncherFlags::SceneOrigin(Scene.Origin);
	URLabLauncherFlags::SceneLevel(Scene.LevelPath);
	Scene.bBaseLevel = URLabLauncherFlags::SceneBaseLevel();
	Scene.bNoQuality = URLabLauncherFlags::SceneNoQuality();
	URLabLauncherFlags::SceneCamMaxHeight(Scene.CamMaxHeight);
	if (URLabLauncherFlags::SceneOverlayMask(Scene.OverlayMask))
	{
		URLabLauncherFlags::SceneOverlayMaxContacts(Scene.OverlayMaxContacts);
	}
	return Scene;
}

/** The -URLabScene keys the render and viewer workflows apply. A key they would
 *  ignore refuses the boot, so an operator is never told a setting took effect
 *  when it did not. The render server applies cammax as the default camera
 *  height cap of every install. */
TArray<FString> HonouredSceneKeys(EMjBootAction Action)
{
	TArray<FString> Keys;
	if (Action == EMjBootAction::Build)
	{
		Keys.Add(TEXT("cammax"));
	}
	return Keys;
}

const TCHAR* WorkflowNameFor(EMjBootAction Action)
{
	return Action == EMjBootAction::Viewer ? TEXT("viewer") : TEXT("render");
}

void RefuseUnhonouredSceneKeys(FMjRendererBootPlan& Plan)
{
	if (Plan.Action != EMjBootAction::Build && Plan.Action != EMjBootAction::Viewer)
	{
		return;
	}
	TArray<FString> Keys;
	URLabLauncherFlags::SceneKeys(Keys);
	const TArray<FString> Honoured = HonouredSceneKeys(Plan.Action);
	TArray<FString> Ignored;
	for (const FString& Key : Keys)
	{
		if (!Honoured.Contains(Key))
		{
			Ignored.Add(Key);
		}
	}
	if (Ignored.IsEmpty())
	{
		return;
	}
	Plan.Error = FString::Printf(
		TEXT("-URLabScene key(s) '%s' are not applied by -URLabWorkflow=%s (it applies: %s)"),
		*FString::Join(Ignored, TEXT(",")), WorkflowNameFor(Plan.Action),
		Honoured.IsEmpty() ? TEXT("none") : *FString::Join(Honoured, TEXT(",")));
	Plan.Action = EMjBootAction::Fail;
}
} // namespace

const TCHAR* MjDriveToken(EMjDrive Drive)
{
	switch (Drive)
	{
		case EMjDrive::Sim:
			return URLabWire::Drive::Sim;
		case EMjDrive::Request:
			return URLabWire::Drive::Request;
		default:
			return URLabWire::Drive::Subscribe;
	}
}

FMjBootNet FMjRendererBoot::ResolveNet()
{
	FMjBootNet Net;
	FString Val;
	if (URLabLauncherFlags::NetValue(TEXT("id"), Val))
	{
		Net.InstanceId = Val;
	}
	if (URLabLauncherFlags::NetValue(TEXT("index"), Val) && Val.IsNumeric())
	{
		Net.InstanceIndex = FCString::Atoi(*Val);
	}
	if (URLabLauncherFlags::NetValue(TEXT("grpc"), Val) && Val.IsNumeric())
	{
		Net.GrpcPort = FCString::Atoi(*Val);
	}
	if (URLabLauncherFlags::NetValue(TEXT("bind"), Val))
	{
		Net.BindAddress = Val;
	}
	if (URLabLauncherFlags::NetValue(TEXT("parent"), Val) && Val.IsNumeric())
	{
		const int64 Pid = FCString::Atoi64(*Val);
		Net.ParentPid = Pid > 0 && Pid <= MAX_uint32 ? static_cast<uint32>(Pid) : 0;
	}
	return Net;
}

FMjRendererBootPlan FMjRendererBoot::FromCommandLine()
{
	FMjRendererBootPlan Plan;
	Plan.Caps = URLabLauncherFlags::ParseCaps();
	Plan.Scene = ResolveScene();
	Plan.Net = ResolveNet();

	FString WorkflowValue;
	const URLabLauncherFlags::EWorkflowKind WorkflowKind = URLabLauncherFlags::ParseWorkflow(WorkflowValue);
	const bool bHasSimulationSourceFlag =
		URLabLauncherFlags::SimulationSourceManifest(Plan.SimulationSourceManifest);
	FString ClockValue;
	const URLabLauncherFlags::EClockKind ClockKind =
		URLabLauncherFlags::ParseClock(ClockValue);
	Plan.bClockModeExplicit = ClockKind != URLabLauncherFlags::EClockKind::Absent;
	if (ClockKind == URLabLauncherFlags::EClockKind::FreeRun)
	{
		Plan.ClockMode = EMjSimulationClockMode::FreeRun;
	}

	switch (WorkflowKind)
	{
		case URLabLauncherFlags::EWorkflowKind::Simulation:
			Plan.Drive = EMjDrive::Sim;
			break;
		case URLabLauncherFlags::EWorkflowKind::Render:
			Plan.Drive = EMjDrive::Request;
			break;
		default:
			// Subscribe is also the fallback for an absent flag: a renderer with no named
			// drive is a consumer until something says otherwise.
			Plan.Drive = EMjDrive::Subscribe;
			break;
	}

	FString RetiredSourceFind;
	const bool bRetiredSourceFind =
		URLabLauncherFlags::RetiredSourceFindValue(RetiredSourceFind);

	if (ClockKind == URLabLauncherFlags::EClockKind::Invalid)
	{
		Plan.Action = EMjBootAction::Fail;
		Plan.Error = FString::Printf(
			TEXT("-URLabClock=%s is not a clock mode; expected stepped or freerun"),
			*ClockValue);
	}
	else if (WorkflowKind == URLabLauncherFlags::EWorkflowKind::Invalid)
	{
		Plan.Action = EMjBootAction::Fail;
		Plan.Error = FString::Printf(TEXT("legacy or invalid workflow selector '%s'; expected -URLabWorkflow=simulation|render|viewer"),*WorkflowValue);
	}
	else if (bRetiredSourceFind)
	{
		Plan.Action = EMjBootAction::Fail;
		Plan.Error = FString::Printf(TEXT("retired -URLabSourceFind selector '%s'; use the explicit viewer workflow and bind contract"), *RetiredSourceFind);
	}
	else if (bHasSimulationSourceFlag && Plan.SimulationSourceManifest.IsEmpty())
	{
		Plan.Action = EMjBootAction::Fail;
		Plan.Error = TEXT("-URLabSimulationSource requires a non-empty JSON manifest path");
	}
	else if (bHasSimulationSourceFlag && WorkflowKind != URLabLauncherFlags::EWorkflowKind::Simulation)
	{
		Plan.Action = EMjBootAction::Fail;
		Plan.Error = TEXT("-URLabSimulationSource is valid only with -URLabWorkflow=simulation");
	}
	else if (Plan.bClockModeExplicit
		&& WorkflowKind != URLabLauncherFlags::EWorkflowKind::None
		&& WorkflowKind != URLabLauncherFlags::EWorkflowKind::Simulation)
	{
		Plan.Action = EMjBootAction::Fail;
		Plan.Error = TEXT("-URLabClock is valid only for a simulation workflow");
	}
	else if (WorkflowKind == URLabLauncherFlags::EWorkflowKind::Simulation)
	{
		Plan.Action = EMjBootAction::Simulation;
	}
	else if (WorkflowKind == URLabLauncherFlags::EWorkflowKind::Viewer)
	{
		Plan.Action = EMjBootAction::Viewer;
	}
	else if (WorkflowKind == URLabLauncherFlags::EWorkflowKind::Render)
	{
		// A render server boots model-free with its bridge live; the first client
		// install brings the scene.
		Plan.Action = EMjBootAction::Build;
	}
	else
	{
		// No launch flag at all: a normal map boot.
		Plan.Action = EMjBootAction::None;
	}
	RefuseUnhonouredSceneKeys(Plan);
	return Plan;
}

FMjRendererBootPlan FMjRendererBoot::ForRpcLoad(const TArray<uint8>& Mjb, const FString& MjbFilePath,
	const FString& BusEndpoint, const FVector& Origin, bool bBaseLevel,
	bool bCameras)
{
	FMjRendererBootPlan Plan;
	Plan.Action = EMjBootAction::Build;
	Plan.Drive = EMjDrive::Subscribe;
	Plan.ModelBytes = Mjb;
	Plan.ModelPath = MjbFilePath;
	Plan.BusEndpoint = BusEndpoint;
	Plan.Scene.Origin = Origin;
	Plan.Scene.bBaseLevel = bBaseLevel;
	// The caller named it, so it wins over an authored default.
	Plan.Caps.Set(EMjCapability::CameraFrames, bCameras);
	Plan.Caps.Set(EMjCapability::CameraRpc, bCameras);
	return Plan;
}

bool FMjRendererBoot::WantsBridge(const FMjRendererBootPlan& Plan, EMjDrive Drive, bool bHasBus)
{
	const FMjCapabilitySet& Caps = Plan.Caps;
	const bool bImplicitRpc =
		Drive == EMjDrive::Request                        // serves the requests that drive it
		|| bHasBus                                           // a serving subscriber
		|| Caps.Has(EMjCapability::FreeFlyPawn);          // a free-fly viewer's bridge
	return Caps.Resolve(EMjCapability::Rpc, Drive == EMjDrive::Sim || bImplicitRpc)
		|| Caps.Has(EMjCapability::Discoverable)
		|| Caps.Has(EMjCapability::PoseStream);
}

FMjCapabilitySet FMjRendererBoot::ResolvePermissionPolicy(
	const FMjRendererBootPlan& Plan, EMjPermissionProfile Profile)
{
	const FMjCapabilitySet& Caps = Plan.Caps;
	FMjCapabilitySet Grants;
	Grants.Set(EMjCapability::CameraRpc,
		Caps.Resolve(EMjCapability::CameraRpc, true));
	Grants.Set(EMjCapability::AcceptInput,
		Caps.Resolve(EMjCapability::AcceptInput, Profile == EMjPermissionProfile::Simulation));
	Grants.Set(EMjCapability::AcceptModelLoad,
		Caps.Resolve(EMjCapability::AcceptModelLoad, true));
	Grants.Set(EMjCapability::AcceptEditorCommands,
		Caps.Resolve(EMjCapability::AcceptEditorCommands, false));
	return Grants;
}
