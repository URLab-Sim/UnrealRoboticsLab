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
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

// ============================================================================
// MjRenderFeatureTests.cpp
//
// Renderer feature tests:
//
//  * Workflow parsing from -URLabWorkflow.
//  * The overlay-mask (mjtVisFlag bit i) decode gating the ISM overlay path.
//
// Simulation clocks and scene ownership are configured independently from the
// boot workflow axis. The simulation workflow is the only
// launcher-level choice this file asserts.
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Components/InstancedStaticMeshComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"

#include "Bridge/BridgeServer.h"
#include "Bridge/BridgeSubsystem.h"
#include "Bridge/RpcDispatcher.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/ScopeExit.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Render/MjLauncherFlags.h"
#include "MuJoCo/Render/MjRendererBoot.h"
#include "MuJoCo/Render/MjRendererLauncher.h"
#include "Transport/MjExternalTransportProvider.h"
#include "Wire/RequestEnvelope.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace MjRenderFeatureTest
{
// Save/restore the process command line around a parse, so flag tests never leak state
// into a co-running test. The helper lives with the parsers: the command line has one
// reader and one writer in the plugin, and a test is not an exception to that.
using FScopedCommandLine = URLabLauncherFlags::FScopedCommandLine;

// A world + actor with a scene root to host the overlay renderer.
struct FHost
{
	UWorld* World = nullptr;
	AActor* Actor = nullptr;

	bool Init()
	{
		World = UWorld::CreateWorld(EWorldType::Game, false);
		if (!World || !GEngine)
		{
			return false;
		}
		FWorldContext& Ctx = GEngine->CreateNewWorldContext(EWorldType::Game);
		Ctx.SetCurrentWorld(World);
		Actor = World->SpawnActor<AActor>();
		if (!Actor)
		{
			return false;
		}
		USceneComponent* Root = NewObject<USceneComponent>(Actor, TEXT("Root"));
		Actor->SetRootComponent(Root);
		Root->RegisterComponent();
		return true;
	}

	void Cleanup()
	{
		if (World)
		{
			World->DestroyWorld(false);
			GEngine->DestroyWorldContext(World);
			World = nullptr;
			Actor = nullptr;
		}
	}

	~FHost() { Cleanup(); }
};

} // namespace MjRenderFeatureTest

// ---------------------------------------------------------------------------
// URLab.Render.DriveParsedFromFlag
//   -URLabWorkflow selects simulation, render, or viewer; legacy drive tokens are rejected.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDriveParsedFromFlag,
	"URLab.Render.DriveParsedFromFlag",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDriveParsedFromFlag::RunTest(const FString& Parameters)
{
 using namespace URLabLauncherFlags;
 for(const auto& Case:{TPair<FString,EWorkflowKind>(TEXT("simulation"),EWorkflowKind::Simulation),{TEXT("render"),EWorkflowKind::Render},{TEXT("viewer"),EWorkflowKind::Viewer}}){FScopedCommandLine Cmd(*FString::Printf(TEXT("-URLabWorkflow=%s"),*Case.Key));FString Value;TestEqual(*Case.Key,ParseWorkflow(Value),Case.Value);}
 {FScopedCommandLine Cmd(TEXT("-URLabDrive=request"));FString Value;TestEqual(TEXT("legacy drive refused"),ParseWorkflow(Value),EWorkflowKind::Invalid);}
 {FScopedCommandLine Cmd(TEXT(""));FString Value;TestEqual(TEXT("absent workflow"),ParseWorkflow(Value),EWorkflowKind::None);}
 return !HasAnyErrors();
}

// ---------------------------------------------------------------------------
// URLab.Render.PushRenderServerHasNoManager
//   Model-free request boot stands up only the world-owned bridge. No manager or
//   placeholder simulation exists before an explicit scene load.
//
//   This drives the real launcher standup path and then inspects the world and
//   bootstrap reply. Nothing here is asserted by reading the source.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjPushRenderServerHasNoManager,
	"URLab.Render.PushRenderServerHasNoManager",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjPushRenderServerHasNoManager::RunTest(const FString& Parameters)
{
	using namespace MjRenderFeatureTest;

	FScopedCommandLine CommandLine(
		TEXT("-URLabWorkflow=render -URLabCaps=rpc,camera_rpc,accept_model_load"));
	// A request server serves the render requests that drive it, so it wants a bridge
	// without ever naming the capability. This is the implication that used to spawn
	// a manager.
	FMjRendererBootPlan Plan;
	Plan.Drive = EMjDrive::Request;
	TestTrue(TEXT("a request boot wants a bridge"),
		FMjRendererBoot::WantsBridge(Plan, EMjDrive::Request, /*bHasBus=*/false));

	FHost H;
	if (!H.Init())
	{
		AddError(TEXT("could not create a game world"));
		return false;
	}

	// Own the bridge in this world and skip the listener bind, so the test drives the
	// cooked branch without fighting the editor's bridge for the configured port.
	UURLabBridgeSubsystem::SetUnboundForTest(true);
	ON_SCOPE_EXIT { UURLabBridgeSubsystem::SetUnboundForTest(false); };
	TArray<FMjExternalRpcTransportFactory> SavedExternalFactories =
		MoveTemp(FMjExternalTransportProvider::ControlRpcTransportFactories);
	FMjExternalTransportProvider::ControlRpcTransportFactories.Reset();
	ON_SCOPE_EXIT
	{
		FMjExternalTransportProvider::ControlRpcTransportFactories =
			MoveTemp(SavedExternalFactories);
	};

	UMjRendererLauncher* Launcher = H.World->GetSubsystem<UMjRendererLauncher>();
	if (!TestNotNull(TEXT("world has the shared renderer launcher"), Launcher))
	{
		return false;
	}
	Launcher->OnWorldBeginPlay(*H.World);
	UURLabBridgeServer* Bridge = UURLabBridgeSubsystem::Find(H.World);
	if (!TestNotNull(TEXT("model-free request boot stood a bridge up"), Bridge))
	{
		return false;
	}
	TestTrue(TEXT("the bridge is running"), Bridge->IsRunning());
	TestTrue(TEXT("render camera RPC grant is active"),
		Bridge->GrantsPermission(EMjCapability::CameraRpc));
	TestTrue(TEXT("explicit model load grant is active"),
		Bridge->GrantsPermission(EMjCapability::AcceptModelLoad));
	TestFalse(TEXT("render input remains ungranted"),
		Bridge->GrantsPermission(EMjCapability::AcceptInput));

	// The gate: no AAMjManager was spawned into the world to get here.
	{
		TArray<AActor*> Managers;
		UGameplayStatics::GetAllActorsOfClass(H.World, AAMjManager::StaticClass(), Managers);
		TestEqual(TEXT("no AAMjManager in a request render server's world"), Managers.Num(), 0);
	}

	// The request boot exposes the independent data and typed render services.
	FURLabRpcDispatcher* Disp = Bridge->GetDispatcher();
	if (TestNotNull(TEXT("the bridge has a dispatcher"), Disp))
	{
		TArray<uint8> ConnectRequest;
		TArray<uint8> ConnectReplyBytes;
		FURLabConnectReply ConnectReply;
		TestTrue(TEXT("strict connect request encodes"),
			FURLabRequestEnvelopeCodec::EncodeConnectRequest({2}, ConnectRequest).IsOk());
		TestTrue(TEXT("strict connect uses production byte dispatch"),
			Disp->DispatchBytes(ConnectRequest, TEXT("zmq"), ConnectReplyBytes));
		if (TestTrue(TEXT("strict connect reply decodes"),
			FURLabRequestEnvelopeCodec::DecodeConnectReply(
				ConnectReplyBytes, ConnectReply).IsOk()))
		{
			TestTrue(TEXT("model-free request boot advertises typed data service"),
				ConnectReply.Services.Contains(TEXT("data")));
			TestTrue(TEXT("model-free request boot advertises typed render service"),
				ConnectReply.Services.Contains(TEXT("render")));
			TestFalse(TEXT("model-free request boot advertises no simulation"),
				ConnectReply.Services.Contains(TEXT("simulation")));
			TestTrue(TEXT("network data transfer is available"),
				ConnectReply.DataOffers.Contains(TEXT("network")));
		}
	}

	H.Cleanup();
	return !HasAnyErrors();
}
