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
#include "RenderExactCaptureTestSupport.h"

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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRenderCameraDepthTranslucentStandIns,
	"URLab.Camera.DepthTranslucentStandIns",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FRenderCameraDepthTranslucentStandIns::RunTest(const FString& Parameters)
{
	FNeutralCameraHost Host;
	if (!TestTrue(TEXT("neutral editor host"), Host.Init())) return false;
	URenderCameraComponent* Depth = AddRenderCamera(Host, ERenderCameraMode::Depth, TEXT("depth"));
	URenderCameraComponent* Color = AddRenderCamera(Host, ERenderCameraMode::Real, TEXT("color"));
	URenderCameraComponent* Seg = AddRenderCamera(Host,
		ERenderCameraMode::InstanceSegmentation, TEXT("seg"));
	if (!TestNotNull(TEXT("depth camera"), Depth) || !TestNotNull(TEXT("color camera"), Color)
		|| !TestNotNull(TEXT("seg camera"), Seg))
	{
		Host.Cleanup(); return false;
	}
	auto MakeMesh = [&Host]() -> UPrimitiveComponent*
	{
		UStaticMeshComponent* Mesh = NewObject<UStaticMeshComponent>(Host.Actor);
		Mesh->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube")));
		Mesh->RegisterComponent();
		return Mesh;
	};
	UPrimitiveComponent* Translucent = MakeMesh();
	UPrimitiveComponent* StandIn = MakeMesh();
	UPrimitiveComponent* OpaqueSeg = MakeMesh();
	UPrimitiveComponent* SemanticSeg = MakeMesh();
	// A capsule stand-in's cap is a separate primitive attached beneath it.
	UPrimitiveComponent* StandInCap = MakeMesh();
	StandInCap->AttachToComponent(StandIn, FAttachmentTransformRules::KeepRelativeTransform);
	UPrimitiveComponent* TranslucentCap = MakeMesh();
	TranslucentCap->AttachToComponent(Translucent, FAttachmentTransformRules::KeepRelativeTransform);
	auto IsHidden = [](URenderCameraComponent* Camera, UPrimitiveComponent* Component)
	{
		return Camera->CaptureComponent->HiddenComponents.Contains(Component);
	};
	const TArray<UPrimitiveComponent*> InstanceSubjects{StandIn, OpaqueSeg};
	const TArray<UPrimitiveComponent*> SemanticSubjects{SemanticSeg};
	const TArray<UPrimitiveComponent*> DepthStandIns{StandIn};
	const TArray<UPrimitiveComponent*> DepthTranslucent{Translucent};
	for (URenderCameraComponent* Camera : {Depth, Color, Seg})
	{
		Camera->SetNeutralSegmentationComponents(InstanceSubjects, SemanticSubjects);
		Camera->SetNeutralDepthComponents(DepthStandIns, DepthTranslucent);
		Camera->SetStreamingEnabled(true);
	}

	TestEqual(TEXT("depth renders the colour primitive set"),
		int32(Depth->CaptureComponent->PrimitiveRenderMode),
		int32(ESceneCapturePrimitiveRenderMode::PRM_LegacySceneCapture));
	TestTrue(TEXT("depth hides the translucent primitive"), IsHidden(Depth, Translucent));
	TestTrue(TEXT("depth hides the translucent primitive's cap"), IsHidden(Depth, TranslucentCap));
	TestFalse(TEXT("depth draws its opaque stand-in"), IsHidden(Depth, StandIn));
	TestFalse(TEXT("depth draws the stand-in's cap"), IsHidden(Depth, StandInCap));
	TestTrue(TEXT("depth hides other segmentation siblings"), IsHidden(Depth, OpaqueSeg));
	TestTrue(TEXT("depth hides semantic siblings"), IsHidden(Depth, SemanticSeg));

	TestFalse(TEXT("colour draws the translucent primitive"), IsHidden(Color, Translucent));
	TestFalse(TEXT("colour draws the translucent primitive's cap"), IsHidden(Color, TranslucentCap));
	TestTrue(TEXT("colour hides the stand-in"), IsHidden(Color, StandIn));
	TestTrue(TEXT("colour hides the stand-in's cap"), IsHidden(Color, StandInCap));

	TestEqual(TEXT("segmentation draws its subjects and their caps"),
		Seg->CaptureComponent->ShowOnlyComponents.Num(), 3);
	TestTrue(TEXT("segmentation draws the stand-in's cap"),
		Seg->CaptureComponent->ShowOnlyComponents.Contains(StandInCap));
	TestFalse(TEXT("segmentation does not draw the translucent primitive"),
		Seg->CaptureComponent->ShowOnlyComponents.Contains(Translucent));

	// An explicitly hidden translucent primitive keeps its depth stand-in hidden.
	Depth->SetVisibilityComponents({}, DepthTranslucent, -1);
	TestTrue(TEXT("explicitly hidden translucent primitive stays hidden"), IsHidden(Depth, Translucent));
	TestTrue(TEXT("its stand-in is hidden too"), IsHidden(Depth, StandIn));
	TestTrue(TEXT("with its cap"), IsHidden(Depth, StandInCap));
	Depth->SetVisibilityComponents({}, {}, -1);
	TestFalse(TEXT("stand-in returns with the primitive"), IsHidden(Depth, StandIn));

	// Depth after segmentation on one camera must not inherit the show-only list.
	FRenderCameraCaptureOverride Override;
	Override.Mode = ERenderCameraMode::Depth;
	FRenderCameraEffectiveConfiguration Effective;
	FRenderCameraOverrideToken Token;
	FString Error;
	TestEqual(TEXT("segmentation uses the show-only list"),
		int32(Seg->CaptureComponent->PrimitiveRenderMode),
		int32(ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList));
	if (TestTrue(TEXT("depth override resolves"), Seg->ResolveCaptureConfiguration(Override, Effective, Error))
		&& TestTrue(TEXT("depth override begins"), Seg->BeginCaptureOverride(Effective, Token, Error)))
	{
		TestEqual(TEXT("overridden depth renders the colour primitive set"),
			int32(Seg->CaptureComponent->PrimitiveRenderMode),
			int32(ESceneCapturePrimitiveRenderMode::PRM_LegacySceneCapture));
		TestTrue(TEXT("overridden depth draws the stand-in"), !IsHidden(Seg, StandIn)
			&& IsHidden(Seg, Translucent));
		TestTrue(TEXT("override restores"), Seg->RestoreCaptureOverride(Token, Error));
	}
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
	const FRenderCameraCaptureCompletion Completion =
		URLabRenderTest::CaptureExact(*Camera, 1.0);
	if (TestTrue(TEXT("capture completed"), Completion.IsSuccessful() && Completion.Frame.IsValid()))
	{
		const FRenderCameraFrame& Frame = *Completion.Frame;
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
