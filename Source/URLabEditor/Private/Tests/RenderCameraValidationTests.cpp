// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Misc/AutomationTest.h"

#include "Editor.h"
#include "Engine/World.h"
#include "Render/CameraStreamController.h"
#include "Render/RenderCameraComponent.h"
#include "Render/RenderCameraRig.h"
#include "Render/URLabRenderScene.h"

#include <limits>

#if WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS

namespace RenderCameraValidationTest
{
void MakeScene(const FString& Revision, FRenderScene& OutScene,
	FRenderSceneResources& OutResources, FRenderFrame& OutFrame)
{
	OutScene = FRenderScene();
	OutResources = FRenderSceneResources();
	OutFrame = FRenderFrame();

	OutScene.SceneRevision = Revision;
	FRenderCameraDesc& Camera = OutScene.Cameras.AddDefaulted_GetRef();
	Camera.CanonicalName = TEXT("model/main_camera");

	OutResources.SceneRevision = Revision;
	OutFrame.Identity.SceneRevision = Revision;
	OutFrame.Identity.FrameId = 41;
	OutFrame.Identity.SourceTime = 1.25;
	OutFrame.Identity.ClockEpoch = 7;
	OutFrame.Identity.bHasSourceTime = true;
	OutFrame.CameraSceneTransforms.Add(FTransform(
		FQuat(FVector::UpVector, 0.25), FVector(10.0, 20.0, 30.0)));
}

bool IdentityEquals(const FRenderFrameIdentity& A, const FRenderFrameIdentity& B)
{
	return A.SceneRevision == B.SceneRevision
		&& A.FrameId == B.FrameId
		&& A.SourceTime == B.SourceTime
		&& A.ClockEpoch == B.ClockEpoch
		&& A.bHasSourceTime == B.bHasSourceTime;
}
} // namespace RenderCameraValidationTest

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRenderCameraLiveStreamRestoresManualPolicy,
	"URLab.Render.CameraValidation.LiveStreamRestoresManualCapturePolicy",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRenderCameraLiveStreamRestoresManualPolicy::RunTest(const FString& Parameters)
{
	UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
	if (!TestNotNull(TEXT("editor world"), World)) return false;
	AURLabRenderScene* SceneActor = World->SpawnActor<AURLabRenderScene>();
	if (!TestNotNull(TEXT("render scene actor"), SceneActor)) return false;
	FRenderScene Scene;
	FRenderSceneResources Resources;
	FRenderFrame InitialFrame;
	const FString Revision = TEXT("camera-live-policy");
	RenderCameraValidationTest::MakeScene(Revision, Scene, Resources, InitialFrame);
	Scene.Cameras[0].Capture.bManualCaptureOnly = true;
	FRenderSceneInstallOptions Options;
	Options.bInitiallyVisible = true;
	Options.bPersistentEditorActors = true;
	FString Error;
	if (!TestTrue(TEXT("manual-only camera scene installs"), SceneActor->InstallScene(
		FRenderSceneInstallPayload(MoveTemp(Scene), MoveTemp(Resources),
			MoveTemp(InitialFrame)), Options, Error)))
	{
		AddError(Error);
		SceneActor->Destroy();
		return false;
	}
	URenderCameraComponent* Camera = SceneActor->GetCameraRig()
		? SceneActor->GetCameraRig()->GetCamera(0) : nullptr;
	if (!TestNotNull(TEXT("installed camera"), Camera))
	{
		SceneActor->Destroy();
		return false;
	}
	TestTrue(TEXT("exact-job baseline is manual-only"), Camera->bManualCaptureOnly);
	FURLabCameraStreamProducerConfig Config;
	Config.BindEndpoint = TEXT("tcp://127.0.0.1:*");
	TUniquePtr<FURLabCameraStreamController> Controller =
		FURLabCameraStreamController::Create(Config, Error);
	if (!TestTrue(TEXT("live stream controller creates"), Controller.IsValid()))
	{
		AddError(Error);
		SceneActor->Destroy();
		return false;
	}
	FURLabCameraStreamDescriptor Descriptor;
	TestTrue(TEXT("explicit live stream enables"), Controller->Configure(*SceneActor,
		Revision, TEXT("model/main_camera"), true, 15.0, Descriptor, Error));
	TestFalse(TEXT("live stream temporarily permits scheduled captures"),
		Camera->bManualCaptureOnly);
	Controller->Disable();
	TestTrue(TEXT("disable restores exact-job manual-only baseline"),
		Camera->bManualCaptureOnly);

	Camera->CaptureMode = static_cast<ERenderCameraMode>(255);
	TestFalse(TEXT("unsupported live encoding rolls back"), Controller->Configure(*SceneActor,
		Revision, TEXT("model/main_camera"), true, 15.0, Descriptor, Error));
	TestTrue(TEXT("failed enable restores manual-only baseline"),
		Camera->bManualCaptureOnly);
	Controller.Reset();
	SceneActor->Destroy();
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRenderCameraInvalidOpticsRejected,
	"URLab.Render.CameraValidation.InvalidOpticsRejected",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRenderCameraInvalidOpticsRejected::RunTest(const FString& Parameters)
{
	FRenderScene Baseline;
	FRenderSceneResources BaselineResources;
	FRenderFrame BaselineFrame;
	RenderCameraValidationTest::MakeScene(
		TEXT("camera-optics-baseline"), Baseline, BaselineResources, BaselineFrame);
	FString Error;
	if (!TestTrue(TEXT("baseline camera scene is valid"),
		Baseline.IsValid(BaselineResources, 0b111, Error)))
	{
		AddError(Error);
		return false;
	}

	static const TCHAR* Labels[] = {
		TEXT("NaN vertical field of view"),
		TEXT("out-of-range projection"),
		TEXT("perspective field of view at 180 degrees"),
		TEXT("nonpositive near clip"),
		TEXT("far clip not beyond near clip"),
		TEXT("nonpositive orthographic width"),
		TEXT("NaN intrinsic focal length"),
	};
	for (int32 Case = 0; Case < UE_ARRAY_COUNT(Labels); ++Case)
	{
		FRenderScene Scene;
		FRenderSceneResources Resources;
		FRenderFrame Frame;
		RenderCameraValidationTest::MakeScene(
			FString::Printf(TEXT("camera-optics-invalid-%d"), Case), Scene, Resources, Frame);
		FRenderCameraOptics& Optics = Scene.Cameras[0].Optics;
		switch (Case)
		{
		case 0:
			Optics.VerticalFovDegrees = std::numeric_limits<float>::quiet_NaN();
			break;
		case 1:
			Optics.Projection = static_cast<ERenderCameraProjection>(255);
			break;
		case 2:
			Optics.VerticalFovDegrees = 180.0f;
			break;
		case 3:
			Optics.NearClipCm = 0.0f;
			break;
		case 4:
			Optics.FarClipCm = Optics.NearClipCm;
			break;
		case 5:
			Optics.Projection = ERenderCameraProjection::Orthographic;
			Optics.OrthographicWidthCm = 0.0f;
			break;
		case 6:
			Optics.bHasIntrinsics = true;
			Optics.FocalPixelsX = std::numeric_limits<float>::quiet_NaN();
			Optics.FocalPixelsY = 100.0f;
			break;
		default:
			checkNoEntry();
			break;
		}
		Error.Reset();
		TestFalse(Labels[Case], Scene.IsValid(Resources, 0b111, Error));
		TestFalse(*FString::Printf(TEXT("%s reports a diagnostic"), Labels[Case]),
			Error.IsEmpty());
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRenderCameraInvalidFrameIsTransactional,
	"URLab.Render.CameraValidation.InvalidFrameIsTransactional",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRenderCameraInvalidFrameIsTransactional::RunTest(const FString& Parameters)
{
	UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
	if (!TestNotNull(TEXT("editor world"), World)) return false;
	AURLabRenderScene* SceneActor = World->SpawnActor<AURLabRenderScene>();
	if (!TestNotNull(TEXT("render scene actor"), SceneActor)) return false;

	FRenderScene Scene;
	FRenderSceneResources Resources;
	FRenderFrame InitialFrame;
	RenderCameraValidationTest::MakeScene(
		TEXT("camera-frame-transaction"), Scene, Resources, InitialFrame);
	FRenderSceneInstallOptions Options;
	Options.bPersistentEditorActors = true;
	FString Error;
	if (!TestTrue(TEXT("camera scene installs"), SceneActor->InstallScene(
		FRenderSceneInstallPayload(MoveTemp(Scene), MoveTemp(Resources), FRenderFrame(InitialFrame)), Options, Error)))
	{
		AddError(Error);
		SceneActor->Destroy();
		return false;
	}

	URenderCameraRig* Rig = SceneActor->GetCameraRig();
	URenderCameraComponent* Camera = Rig ? Rig->GetCamera(0) : nullptr;
	if (!TestNotNull(TEXT("installed camera rig"), Rig)
		|| !TestNotNull(TEXT("installed camera"), Camera))
	{
		SceneActor->Destroy();
		return false;
	}
	const FTransform OriginalTransform = Camera->GetRelativeTransform();
	const FRenderFrameIdentity OriginalIdentity = Camera->GetAppliedFrameIdentity();
	const FRenderFrame OriginalAppliedFrame = SceneActor->GetAppliedFrame();

	auto TestUnchanged = [this, SceneActor, Rig, Camera, &OriginalTransform,
		&OriginalIdentity, &OriginalAppliedFrame](const TCHAR* Label)
	{
		TestTrue(*FString::Printf(TEXT("%s keeps rig identity"), Label),
			SceneActor->GetCameraRig() == Rig && Rig->GetCamera(0) == Camera);
		TestTrue(*FString::Printf(TEXT("%s keeps exact camera transform"), Label),
			Camera->GetRelativeTransform().Equals(OriginalTransform, 0.0f));
		TestTrue(*FString::Printf(TEXT("%s keeps exact camera frame identity"), Label),
			RenderCameraValidationTest::IdentityEquals(
				Camera->GetAppliedFrameIdentity(), OriginalIdentity));
		TestTrue(*FString::Printf(TEXT("%s keeps applied scene frame"), Label),
			RenderCameraValidationTest::IdentityEquals(
				SceneActor->GetAppliedFrame().Identity, OriginalAppliedFrame.Identity)
			&& SceneActor->GetAppliedFrame().CameraSceneTransforms[0].Equals(
				OriginalAppliedFrame.CameraSceneTransforms[0], 0.0f));
	};

	FRenderFrame WrongRevision = InitialFrame;
	WrongRevision.Identity.SceneRevision = TEXT("another-revision");
	WrongRevision.Identity.FrameId = 42;
	WrongRevision.CameraSceneTransforms[0].SetTranslation(FVector(900.0, 0.0, 0.0));
	TestFalse(TEXT("wrong-revision camera frame rejects before mutation"),
		SceneActor->ApplyFrame(WrongRevision, Error));
	TestUnchanged(TEXT("wrong-revision rejection"));

	FRenderFrame ScaledCamera = InitialFrame;
	ScaledCamera.Identity.FrameId = 43;
	ScaledCamera.CameraSceneTransforms[0].SetScale3D(FVector(2.0, 1.0, 1.0));
	TestFalse(TEXT("scaled camera frame rejects before mutation"),
		SceneActor->ApplyFrame(ScaledCamera, Error));
	TestUnchanged(TEXT("invalid-transform rejection"));

	SceneActor->Destroy();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRenderCameraFailedReplacementPreservesLiveRig,
	"URLab.Render.CameraValidation.FailedReplacementPreservesLiveRig",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FRenderCameraFailedReplacementPreservesLiveRig::RunTest(const FString& Parameters)
{
	UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
	if (!TestNotNull(TEXT("editor world"), World)) return false;
	AURLabRenderScene* SceneActor = World->SpawnActor<AURLabRenderScene>();
	if (!TestNotNull(TEXT("render scene actor"), SceneActor)) return false;

	FRenderScene InitialScene;
	FRenderSceneResources InitialResources;
	FRenderFrame InitialFrame;
	RenderCameraValidationTest::MakeScene(
		TEXT("camera-live-a"), InitialScene, InitialResources, InitialFrame);
	FRenderSceneInstallOptions Options;
	Options.bInitiallyVisible = true;
	Options.bPersistentEditorActors = true;
	FString Error;
	if (!TestTrue(TEXT("initial camera scene installs"), SceneActor->InstallScene(
		FRenderSceneInstallPayload(MoveTemp(InitialScene), MoveTemp(InitialResources), FRenderFrame(InitialFrame)), Options, Error)))
	{
		AddError(Error);
		SceneActor->Destroy();
		return false;
	}
	URenderCameraRig* OriginalRig = SceneActor->GetCameraRig();
	URenderCameraComponent* OriginalCamera = OriginalRig ? OriginalRig->GetCamera(0) : nullptr;
	if (!TestNotNull(TEXT("original camera rig"), OriginalRig)
		|| !TestNotNull(TEXT("original camera"), OriginalCamera))
	{
		SceneActor->Destroy();
		return false;
	}
	TestTrue(TEXT("original camera starts visible"), OriginalCamera->IsVisible());

	FRenderScene Replacement;
	FRenderSceneResources ReplacementResources;
	FRenderFrame ReplacementFrame;
	RenderCameraValidationTest::MakeScene(
		TEXT("camera-live-b"), Replacement, ReplacementResources, ReplacementFrame);
	Replacement.Cameras[0].Optics.NearClipCm = 0.0f;
	TestFalse(TEXT("replacement with invalid camera is rejected"), SceneActor->InstallScene(
		FRenderSceneInstallPayload(MoveTemp(Replacement), MoveTemp(ReplacementResources), FRenderFrame(ReplacementFrame)), Options, Error));

	TestTrue(TEXT("failed replacement retains the live rig"),
		SceneActor->GetCameraRig() == OriginalRig);
	TestTrue(TEXT("failed replacement retains the exact camera component"),
		OriginalRig->GetCamera(0) == OriginalCamera);
	TestTrue(TEXT("retained camera remains owned by the live scene"),
		OriginalCamera->GetOwner() == SceneActor);
	TestTrue(TEXT("retained camera remains visible"), OriginalCamera->IsVisible());
	TestEqual(TEXT("failed replacement keeps scene revision"),
		SceneActor->GetScene().SceneRevision, FString(TEXT("camera-live-a")));
	TestTrue(TEXT("failed replacement keeps applied frame identity"),
		RenderCameraValidationTest::IdentityEquals(
			SceneActor->GetAppliedFrame().Identity, InitialFrame.Identity));

	SceneActor->SetSceneVisible(false);
	TestFalse(TEXT("retained camera still follows scene visibility"), OriginalCamera->IsVisible());
	SceneActor->SetSceneVisible(true);
	TestTrue(TEXT("retained camera can be shown again"), OriginalCamera->IsVisible());
	SceneActor->Destroy();
	return true;
}

#endif
