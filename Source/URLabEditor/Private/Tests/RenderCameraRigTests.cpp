#include "Misc/AutomationTest.h"
#include "Editor.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Components/SceneComponent.h"
#include "Render/RenderCameraRig.h"
#include "Render/RenderCameraComponent.h"
#include "MuJoCo/Render/MjRenderSceneAdapter.h"
#include "mujoco/mujoco.h"

#if WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRenderCameraRigVisibility,
	"URLab.Render.CameraRig.StagedVisibilityAndLookup",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRenderCameraRigVisibility::RunTest(const FString&)
{
	UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
	if (!TestNotNull(TEXT("editor world"), World)) return false;
	AActor* Owner = World->SpawnActor<AActor>();
	USceneComponent* Root = NewObject<USceneComponent>(Owner);
	Owner->SetRootComponent(Root);
	Root->RegisterComponent();

	FRenderScene Scene;
	Scene.SceneRevision = TEXT("camera-rig-visible");
	Scene.Cameras.AddDefaulted_GetRef().CanonicalName = TEXT("arm/wrist_camera");
	FRenderFrame Frame;
	Frame.Identity.SceneRevision = Scene.SceneRevision;
	Frame.CameraSceneTransforms.Add(FTransform(FVector(10, 20, 30)));
	URenderCameraRig* Rig = NewObject<URenderCameraRig>(Owner);
	FString Error;
	if (!TestTrue(TEXT("hidden staged rig builds"), Rig->Build(Root, Scene, Frame, false, Error)))
	{
		AddError(Error); Owner->Destroy(); return false;
	}
	URenderCameraComponent* Camera = Rig->FindByCanonicalName(TEXT("arm/wrist_camera"));
	if (!TestNotNull(TEXT("canonical lookup"), Camera) ||
		!TestNotNull(TEXT("capture child"), Camera ? Camera->CaptureComponent.Get() : nullptr))
	{
		Owner->Destroy(); return false;
	}
	Camera->SetStreamingEnabled(true);
	TestFalse(TEXT("camera hidden while staged"), Camera->IsVisible());
	TestFalse(TEXT("capture hidden while staged"), Camera->CaptureComponent->IsVisible());
	Rig->SetVisible(true);
	TestTrue(TEXT("camera visible after commit"), Camera->IsVisible());
	TestTrue(TEXT("capture visible after commit"), Camera->CaptureComponent->IsVisible());
	Owner->Destroy();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRenderCameraRigRejectsDuplicateNames,
	"URLab.Render.CameraRig.DuplicateCanonicalNamesInvalid",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRenderCameraRigRejectsDuplicateNames::RunTest(const FString&)
{
	FRenderScene Scene;
	Scene.SceneRevision = TEXT("camera-rig-duplicates");
	Scene.Cameras.AddDefaulted_GetRef().CanonicalName = TEXT("same/camera");
	Scene.Cameras.AddDefaulted_GetRef().CanonicalName = TEXT("same/camera");
	FRenderSceneResources Resources;
	Resources.SceneRevision = Scene.SceneRevision;
	FString Error;
	TestFalse(TEXT("duplicate camera canonical names rejected"), Scene.IsValid(Resources, 0b111, Error));
	TestTrue(TEXT("diagnostic identifies ambiguous identity"), Error.Contains(TEXT("ambiguous")));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRenderCameraRigFixedCameraFollowsBody,
	"URLab.Render.CameraRig.FixedCameraFollowsCartesianBody",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FRenderCameraRigFixedCameraFollowsBody::RunTest(const FString&)
{
	constexpr const char* Xml =
		"<mujoco><worldbody><body name='carrier'><geom type='sphere' size='.1'/>"
		"<camera name='mounted' pos='.1 0 0'/></body></worldbody></mujoco>";
	char ErrorBuffer[1024]{};
	mjSpec* Spec = mj_parseXMLString(Xml, nullptr, ErrorBuffer, sizeof(ErrorBuffer));
	if (!TestNotNull(TEXT("fixture spec"), Spec)) return false;
	mjModel* Model = mj_compile(Spec, nullptr);
	mj_deleteSpec(Spec);
	if (!TestNotNull(TEXT("fixture model"), Model)) return false;
	mjData* Data = mj_makeData(Model);
	if (!TestNotNull(TEXT("fixture data"), Data)) { mj_deleteModel(Model); return false; }
	mj_forward(Model, Data);

	FMjRenderSceneAdapter Adapter;
	FRenderScene Scene;
	FRenderSceneResources Resources;
	FString Error;
	const bool bExtracted = Adapter.Extract(Model, Data, TEXT("fixed-camera"), 0b111,
		1024, false, nullptr, Scene, Resources, Error);
	if (!TestTrue(TEXT("adapter extracts fixed camera"), bExtracted))
	{
		AddError(Error); mj_deleteData(Data); mj_deleteModel(Model); return false;
	}
	TArray<double> Positions;
	Positions.Init(0.0, 3 * Model->nbody);
	TArray<double> Quaternions;
	Quaternions.Init(0.0, 4 * Model->nbody);
	for (int32 Body = 0; Body < Model->nbody; ++Body) Quaternions[4 * Body] = 1.0;
	FRenderFrame First, Moved;
	TestTrue(TEXT("initial Cartesian frame"), Adapter.FrameFromCartesian(
		Positions, Quaternions, {}, {}, 1, 0.0, true, 0, First, Error));
	Positions[3] = 1.0; // carrier +X metre; world body remains fixed
	TestTrue(TEXT("moved Cartesian frame"), Adapter.FrameFromCartesian(
		Positions, Quaternions, {}, {}, 2, 0.1, true, 0, Moved, Error));
	if (TestEqual(TEXT("one camera"), Moved.CameraSceneTransforms.Num(), 1))
	{
		const FVector Delta = Moved.CameraSceneTransforms[0].GetTranslation()
			- First.CameraSceneTransforms[0].GetTranslation();
		TestTrue(TEXT("mounted camera follows body by converted metre"),
			Delta.Equals(FVector(100.0, 0.0, 0.0), 0.01));
	}
	mj_deleteData(Data);
	mj_deleteModel(Model);
	return true;
}

#endif
