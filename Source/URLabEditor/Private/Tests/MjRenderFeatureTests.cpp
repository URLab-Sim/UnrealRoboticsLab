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
// Renderer feature tests for work landed since v0.6.0-beta (source-of-truth §2,
// §8.5, §14):
//
//  * Drive parsing from the one launcher flag axis (-URLabDrive), §2 / §14.
//  * The overlay-mask (mjtVisFlag bit i) decode gating the ISM overlay path, §8.5.
//  * The perturbation drag gizmo rendered through the pooled ISM overlay
//    (DrawDragSpring), NOT DrawDebug*, §8.5.
//
// step-mode is deliberately NOT parsed from a flag: §3 makes it a RUNTIME
// sub-axis of a sim Drive (set_mode -> {FreeRun,Stepped,StatePushed}), reached
// over RPC, never a boot flag. The boot axis a launcher selects is Drive; the
// sim/producer choice (-URLabDrive=sim) is the flag-level half of that pair and
// is what this file asserts.
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Misc/CommandLine.h"

#include "Components/InstancedStaticMeshComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"

#include "MuJoCo/Core/MjRenderSnapshot.h"
#include "MuJoCo/Entity/MjOverlayRenderer.h"
#include "MuJoCo/Fast/MjLauncherFlags.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace MjRenderFeatureTest
{
// Save/restore the process command line around a parse, so flag tests never leak
// state into a co-running test.
struct FScopedCommandLine
{
	FString Saved;
	explicit FScopedCommandLine(const TCHAR* NewLine)
	{
		Saved = FCommandLine::Get();
		FCommandLine::Set(NewLine);
	}
	~FScopedCommandLine() { FCommandLine::Set(*Saved); }
};

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

// Total instances across every ISM the overlay renderer pooled onto its owner --
// the overlay path renders each primitive as one ISM instance (§8.5), so this is
// how many overlay primitives were emitted this frame.
int32 TotalOverlayInstances(AActor* Actor)
{
	int32 Total = 0;
	TArray<UInstancedStaticMeshComponent*> Isms;
	Actor->GetComponents<UInstancedStaticMeshComponent>(Isms);
	for (const UInstancedStaticMeshComponent* Ism : Isms)
	{
		if (Ism)
		{
			Total += Ism->GetInstanceCount();
		}
	}
	return Total;
}

} // namespace MjRenderFeatureTest

// ---------------------------------------------------------------------------
// URLab.Fast.DriveParsedFromFlag
//   -URLabDrive is the one primary axis (§2/§14): sim | stream:<ep> | push |
//   await. Parse each spelling and the stream endpoint scheme extraction.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjDriveParsedFromFlag,
	"URLab.Fast.DriveParsedFromFlag",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjDriveParsedFromFlag::RunTest(const FString& Parameters)
{
	using namespace URLabLauncherFlags;
	using namespace MjRenderFeatureTest;

	// sim -> Producer.
	{
		FScopedCommandLine Cmd(TEXT("-URLabDrive=sim"));
		FString Ep;
		TestEqual(TEXT("sim parses to Drive::Sim"), ParseDrive(Ep), EDriveKind::Sim);
		TestTrue(TEXT("DriveIsSim() true for sim"), DriveIsSim());
		TestFalse(TEXT("DriveIsPush() false for sim"), DriveIsPush());
	}

	// push -> Consumer (forced-render).
	{
		FScopedCommandLine Cmd(TEXT("-URLabDrive=push"));
		FString Ep;
		TestEqual(TEXT("push parses to Drive::Push"), ParseDrive(Ep), EDriveKind::Push);
		TestTrue(TEXT("DriveIsPush() true for push"), DriveIsPush());
	}

	// await -> served placeholder.
	{
		FScopedCommandLine Cmd(TEXT("-URLabDrive=await"));
		FString Ep;
		TestEqual(TEXT("await parses to Drive::Await"), ParseDrive(Ep), EDriveKind::Await);
		TestTrue(TEXT("DriveIsAwait() true for await"), DriveIsAwait());
	}

	// stream:<endpoint> -> Consumer, endpoint carries its transport scheme (§9.1).
	{
		FScopedCommandLine Cmd(TEXT("-URLabDrive=stream:tcp://127.0.0.1:5561"));
		FString Ep;
		TestEqual(TEXT("stream parses to Drive::Stream"), ParseDrive(Ep), EDriveKind::Stream);
		TestEqual(TEXT("stream endpoint preserved verbatim"), Ep, FString(TEXT("tcp://127.0.0.1:5561")));
		FString TcpEp;
		TestTrue(TEXT("tcp:// scheme selects the ZMQ endpoint accessor"), DriveStreamTcpEndpoint(TcpEp));
		TestEqual(TEXT("tcp endpoint keeps its scheme"), TcpEp, FString(TEXT("tcp://127.0.0.1:5561")));
		FString GrpcEp;
		TestFalse(TEXT("grpc accessor rejects a tcp endpoint"), DriveStreamGrpcEndpoint(GrpcEp));
	}

	// grpc:// stream selects the gRPC join.
	{
		FScopedCommandLine Cmd(TEXT("-URLabDrive=stream:grpc://host:50051"));
		FString GrpcEp;
		TestTrue(TEXT("grpc:// scheme selects the gRPC endpoint accessor"), DriveStreamGrpcEndpoint(GrpcEp));
		TestEqual(TEXT("grpc endpoint keeps its scheme"), GrpcEp, FString(TEXT("grpc://host:50051")));
	}

	// Absent flag -> None (BeginPlay then derives from the boot signals, §2).
	{
		FScopedCommandLine Cmd(TEXT(""));
		FString Ep;
		TestEqual(TEXT("absent -URLabDrive parses to None"), ParseDrive(Ep), EDriveKind::None);
	}

	return true;
}

// ---------------------------------------------------------------------------
// URLab.Fast.OverlayMaskGatesIsmOverlay
//   The overlay renderer draws each enabled mjVIS_* overlay as ISM instances,
//   gated by FMjOverlayFlags.VisFlags[i] (bit i = mjtVisFlag). An all-off mask
//   emits zero primitives; flipping the inertia bit emits inertia boxes.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjOverlayMaskGatesIsmOverlay,
	"URLab.Fast.OverlayMaskGatesIsmOverlay",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjOverlayMaskGatesIsmOverlay::RunTest(const FString& Parameters)
{
	using namespace MjRenderFeatureTest;

	// One massive body so DrawInertia has a box to emit.
	char szErr[1000] = "";
	mjSpec* Spec = mj_parseXMLString(
		"<mujoco><worldbody>"
		"  <body name=\"ball\" pos=\"0 0 0.5\">"
		"    <joint type=\"free\"/>"
		"    <geom type=\"box\" size=\"0.1 0.1 0.1\" density=\"1000\"/>"
		"  </body>"
		"</worldbody></mujoco>",
		nullptr, szErr, sizeof(szErr));
	if (!Spec)
	{
		AddError(FString::Printf(TEXT("parse failed: %hs"), szErr));
		return false;
	}
	mjModel* m = mj_compile(Spec, nullptr);
	mj_deleteSpec(Spec);
	if (!m)
	{
		AddError(TEXT("compile failed"));
		return false;
	}
	mjData* d = mj_makeData(m);
	mj_forward(m, d);

	FHost H;
	if (!TestTrue(TEXT("host world/actor"), H.Init()))
	{
		mj_deleteData(d);
		mj_deleteModel(m);
		return false;
	}

	UMjOverlayRenderer* OR = NewObject<UMjOverlayRenderer>(H.Actor, TEXT("Overlay"));
	H.Actor->AddInstanceComponent(OR);
	OR->SetupAttachment(H.Actor->GetRootComponent());
	OR->RegisterComponent();
	OR->SetModel(m);

	// A snapshot carrying just the inertia-frame poses DrawInertia reads.
	FMjRenderSnapshot Snap;
	Snap.XiPos.SetNumUninitialized(3 * m->nbody);
	Snap.XiMat.SetNumUninitialized(9 * m->nbody);
	FMemory::Memcpy(Snap.XiPos.GetData(), d->xipos, sizeof(mjtNum) * 3 * m->nbody);
	FMemory::Memcpy(Snap.XiMat.GetData(), d->ximat, sizeof(mjtNum) * 9 * m->nbody);

	// (1) All vis flags off -> nothing drawn.
	OR->Flags.VisFlags.Init(0, mjNVISFLAG);
	OR->DrawOverlays(Snap);
	const int32 OffCount = TotalOverlayInstances(H.Actor);
	TestEqual(TEXT("an all-off mask emits zero overlay instances"), OffCount, 0);

	// (2) Flip bit mjVIS_INERTIA -> inertia box(es) appear.
	OR->Flags.VisFlags.Init(0, mjNVISFLAG);
	OR->Flags.VisFlags[mjVIS_INERTIA] = 1;
	OR->DrawOverlays(Snap);
	const int32 InertiaCount = TotalOverlayInstances(H.Actor);
	TestTrue(FString::Printf(TEXT("the mjVIS_INERTIA bit emits inertia boxes (%d instances)"), InertiaCount),
		InertiaCount > 0);

	// (3) Clearing the bit again drops back to zero (per-frame clear + gate).
	OR->Flags.VisFlags.Init(0, mjNVISFLAG);
	OR->DrawOverlays(Snap);
	TestEqual(TEXT("clearing the bit clears the overlay"), TotalOverlayInstances(H.Actor), 0);

	mj_deleteData(d);
	mj_deleteModel(m);
	return true;
}

// ---------------------------------------------------------------------------
// URLab.Fast.PerturbGizmoRendersViaIsmOverlay
//   The interactive drag gizmo (UMjPerturbation -> UMjOverlayRenderer::
//   DrawDragSpring) renders through the pooled ISM overlay substrate, not
//   DrawDebug*: a grab-point marker + spring arrow become ISM instances (§8.5).
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjPerturbGizmoRendersViaIsmOverlay,
	"URLab.Fast.PerturbGizmoRendersViaIsmOverlay",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjPerturbGizmoRendersViaIsmOverlay::RunTest(const FString& Parameters)
{
	using namespace MjRenderFeatureTest;

	FHost H;
	if (!TestTrue(TEXT("host world/actor"), H.Init()))
	{
		return false;
	}

	UMjOverlayRenderer* OR = NewObject<UMjOverlayRenderer>(H.Actor, TEXT("Overlay"));
	H.Actor->AddInstanceComponent(OR);
	OR->SetupAttachment(H.Actor->GetRootComponent());
	OR->RegisterComponent();
	// DrawDragSpring reads no model (the drag layer is transform-only).

	// No drag drawn yet.
	TestEqual(TEXT("no overlay instances before a drag"), TotalOverlayInstances(H.Actor), 0);

	// An active translate drag: grab-point sphere + spring arrow (arrow = shaft
	// cylinder + cone head), all as ISM instances.
	const FVector Grab(0, 0, 0);
	const FVector Target(50, 0, 0);
	OR->DrawDragSpring(Grab, Target, /*bTranslate=*/true, /*bRotate=*/false, FVector::ZeroVector);

	const int32 DragCount = TotalOverlayInstances(H.Actor);
	TestTrue(FString::Printf(TEXT("the drag gizmo renders via ISM instances (%d), not DrawDebug"), DragCount),
		DragCount > 0);
	// Grab marker (1 sphere) + spring arrow (>=2: shaft + head).
	TestTrue(FString::Printf(TEXT("marker + spring arrow emitted (%d instances)"), DragCount),
		DragCount >= 2);

	// Releasing the drag clears the drag layer.
	OR->ClearDragSpring();
	TestEqual(TEXT("releasing the drag clears the gizmo"), TotalOverlayInstances(H.Actor), 0);

	return true;
}
