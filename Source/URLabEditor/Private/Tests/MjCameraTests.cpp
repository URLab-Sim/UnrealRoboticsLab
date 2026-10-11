// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Editor.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Components/SceneComponent.h"
#include "Render/RenderCameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Misc/App.h"
#include "RenderingThread.h"

namespace
{
FRenderFrameIdentity CameraTestIdentity(uint64 FrameId = 2)
{
	FRenderFrameIdentity Identity;
	Identity.SceneRevision = TEXT("camera-test-scene");
	Identity.FrameId = FrameId;
	Identity.SourceTime = 0.25;
	Identity.ClockEpoch = 7;
	Identity.bHasSourceTime = true;
	return Identity;
}

struct FNeutralCameraHost
{
	UWorld* World = nullptr;
	AActor* Actor = nullptr;
	bool Init()
	{
		World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
		if (!World) return false;
		Actor = World->SpawnActor<AActor>();
		if (!Actor) return false;
		USceneComponent* Root = NewObject<USceneComponent>(Actor);
		Actor->SetRootComponent(Root);
		Root->RegisterComponent();
		return true;
	}
	void Cleanup() { if (Actor) Actor->Destroy(); Actor = nullptr; World = nullptr; }
	~FNeutralCameraHost() { Cleanup(); }
};

URenderCameraComponent* AddRenderCamera(FNeutralCameraHost& Host, ERenderCameraMode Mode,
	const TCHAR* Name = TEXT("TestCamera"))
{
	URenderCameraComponent* Camera = NewObject<URenderCameraComponent>(Host.Actor);
	if (!Camera) return nullptr;
	FRenderCameraDesc Desc;
	Desc.CanonicalName = Name;
	Desc.Capture.Mode = Mode;
	Desc.Optics.Resolution = FIntPoint(64, 32);
	FString Error;
	if (!Camera->Configure(Desc, Error)) return nullptr;
	Camera->SetupAttachment(Host.Actor->GetRootComponent());
	Camera->RegisterComponent();
	Camera->SetSceneVisibility(true);
	Camera->SetAppliedFrameIdentity(CameraTestIdentity());
	return Camera;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRenderCameraTargetFormats,
	"URLab.Camera.NeutralTargetFormats",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FRenderCameraTargetFormats::RunTest(const FString& Parameters)
{
	FNeutralCameraHost Host;
	if (!TestTrue(TEXT("neutral editor host"), Host.Init())) return false;
	URenderCameraComponent* Color = AddRenderCamera(Host, ERenderCameraMode::Real, TEXT("color"));
	URenderCameraComponent* Depth = AddRenderCamera(Host, ERenderCameraMode::Depth, TEXT("depth"));
	if (!TestNotNull(TEXT("color camera"), Color) || !TestNotNull(TEXT("depth camera"), Depth))
	{
		Host.Cleanup(); return false;
	}
	Color->SetStreamingEnabled(true);
	Depth->SetStreamingEnabled(true);
	if (TestNotNull(TEXT("color target"), Color->RenderTarget.Get()))
		TestEqual(TEXT("color target is RGBA8"), int32(Color->RenderTarget->RenderTargetFormat),
			int32(ETextureRenderTargetFormat::RTF_RGBA8));
	if (TestNotNull(TEXT("depth target"), Depth->RenderTarget.Get()))
		TestEqual(TEXT("depth target is R32f"), int32(Depth->RenderTarget->RenderTargetFormat),
			int32(ETextureRenderTargetFormat::RTF_R32f));
	Host.Cleanup();
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRenderCameraExplicitSegmentationSet,
	"URLab.Camera.ExplicitSegmentationComponents",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FRenderCameraExplicitSegmentationSet::RunTest(const FString& Parameters)
{
	FNeutralCameraHost Host;
	if (!TestTrue(TEXT("neutral editor host"), Host.Init())) return false;
	URenderCameraComponent* Camera = AddRenderCamera(Host,
		ERenderCameraMode::InstanceSegmentation, TEXT("seg"));
	if (!TestNotNull(TEXT("camera"), Camera)) { Host.Cleanup(); return false; }
	UStaticMeshComponent* Mesh = NewObject<UStaticMeshComponent>(Host.Actor);
	Mesh->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube")));
	Mesh->RegisterComponent();
	TArray<UPrimitiveComponent*> ShowOnly{Mesh};
	Camera->SetVisibilityComponents(ShowOnly, {}, 11);
	Camera->SetStreamingEnabled(true);
	if (TestNotNull(TEXT("capture component"), Camera->CaptureComponent.Get()))
	{
		TestEqual(TEXT("one explicit component"), Camera->CaptureComponent->ShowOnlyComponents.Num(), 1);
		TestEqual(TEXT("show-only primitive mode"), int32(Camera->CaptureComponent->PrimitiveRenderMode),
			int32(ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList));
	}
	TestEqual(TEXT("producer object type retained"), Camera->GetSegmentationObjectType(), 11);
	Host.Cleanup();
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRenderCameraExactCaptureIdentity,
	"URLab.Camera.RenderOnDemandExactIdentity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FRenderCameraExactCaptureIdentity::RunTest(const FString& Parameters)
{
	FNeutralCameraHost Host;
	if (!TestTrue(TEXT("neutral editor host"), Host.Init())) return false;
	URenderCameraComponent* Camera = AddRenderCamera(Host, ERenderCameraMode::Real);
	if (!TestNotNull(TEXT("camera"), Camera)) { Host.Cleanup(); return false; }
	Camera->SetStreamingEnabled(true);
	if (!FApp::CanEverRender())
	{
		AddInfo(TEXT("No RHI: run offscreen to validate GPU pixels and exact capture identity"));
		Host.Cleanup(); return true;
	}
	FlushRenderingCommands();
	Camera->IssueSyncCapture();
	FlushRenderingCommands();
	Camera->WaitAndHarvestReadbacks(1.0);
	FRenderCameraFrame Frame;
	if (TestTrue(TEXT("capture completed"), Camera->GetFrame(0, Frame)))
	{
		const FRenderFrameIdentity Expected = CameraTestIdentity();
		TestEqual(TEXT("scene revision"), Frame.Identity.SceneRevision, Expected.SceneRevision);
		TestEqual(TEXT("clock epoch"), Frame.Identity.ClockEpoch, Expected.ClockEpoch);
		TestEqual(TEXT("frame id"), Frame.Identity.FrameId, Expected.FrameId);
		TestEqual(TEXT("source time flag"), Frame.Identity.bHasSourceTime, true);
		TestTrue(TEXT("source time"), FMath::IsNearlyEqual(Frame.Identity.SourceTime,
			Expected.SourceTime, 1e-9));
		TestEqual(TEXT("captured width"), Frame.Width, 64);
		TestEqual(TEXT("captured height"), Frame.Height, 32);
		TestEqual(TEXT("pixel count"), Frame.Color.Num(), 2048);
	}
	Host.Cleanup();
	return !HasAnyErrors();
}
