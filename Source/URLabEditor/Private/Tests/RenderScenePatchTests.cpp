#include "Misc/AutomationTest.h"

#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Render/RenderWorldSubsystem.h"
#include "Render/URLabRenderScene.h"
#include "Workflows/RenderServerWorldSubsystem.h"
#include "Workflows/ViewerWorldSubsystem.h"

#if WITH_DEV_AUTOMATION_TESTS
namespace URLabRenderScenePatchTest
{
struct FWorldFixture
{
	UWorld* World = nullptr;
	bool Create()
	{
		World=UWorld::CreateWorld(EWorldType::Game,false);
		if(!World||!GEngine)return false;
		GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
		return true;
	}
	~FWorldFixture()
	{
		if(!World)return;
		if(auto* Render=World->GetSubsystem<UURLabRenderServerWorldSubsystem>())Render->ShutdownWorkflow();
		if(auto* Viewer=World->GetSubsystem<UURLabViewerWorldSubsystem>())Viewer->ShutdownWorkflow();
		World->DestroyWorld(false);GEngine->DestroyWorldContext(World);
	}
};

FURLabRenderOwnerSessionScopePtr MakeScope(const TCHAR* Name)
{
	FString Error;return FURLabRenderOwnerSessionScope::Create(Name,Error);
}

FRenderSceneInstallPayload PatchableScene(const FString& Revision)
{
	FRenderScene Scene;Scene.SceneRevision=Revision;
	Scene.Bodies.AddDefaulted_GetRef().Name=TEXT("body");
	FRenderGeomDesc& Geom=Scene.Geoms.AddDefaulted_GetRef();
	Geom.BodyIndex=0;Geom.Kind=ERenderGeomKind::Sphere;Geom.SizeCm=FVector(10.0);
	Geom.Name=TEXT("sphere");
	FRenderCameraDesc& Camera=Scene.Cameras.AddDefaulted_GetRef();Camera.CanonicalName=TEXT("front");
	FRenderLightDesc& Light=Scene.Lights.AddDefaulted_GetRef();Light.Name=TEXT("key");
	Light.Kind=ERenderLightKind::Point;Light.Intensity=100.0f;Light.RangeCm=500.0f;
	FRenderSceneResources Resources;Resources.SceneRevision=Revision;
	Resources.Geoms.AddDefaulted_GetRef().MaterialIndex=0;
	Resources.Materials.AddDefaulted_GetRef().Rgba=FLinearColor::White;
	FRenderFrame Frame;Frame.Identity.SceneRevision=Revision;Frame.Identity.FrameId=1;
	Frame.BodySceneTransforms.Add(FTransform::Identity);
	Frame.CameraSceneTransforms.Add(FTransform::Identity);
	Frame.LightSceneTransforms.Add(FTransform::Identity);
	return FRenderSceneInstallPayload(MoveTemp(Scene),MoveTemp(Resources),MoveTemp(Frame));
}


}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRenderScenePatchAtomicOwnerTransaction,
	"URLab.Workflows.RenderScenePatch.AtomicOwnerTransaction",
	EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)

bool FRenderScenePatchAtomicOwnerTransaction::RunTest(const FString&)
{
	using namespace URLabRenderScenePatchTest;
	FWorldFixture Fixture;if(!TestTrue(TEXT("world creates"),Fixture.Create()))return false;
	auto* Workflow=Fixture.World->GetSubsystem<UURLabRenderServerWorldSubsystem>();
	if(!TestNotNull(TEXT("render workflow exists"),Workflow))return false;
	FString Error;if(!TestTrue(TEXT("render workflow activates"),Workflow->Activate(Error))){AddError(Error);return false;}
	auto Scope=MakeScope(TEXT("scene-patch-owner"));
	const auto Attached=Workflow->AttachOwner(Scope);
	if(!TestTrue(TEXT("scene owner attaches"),Attached.IsOk()))return false;
	const auto Installed=Workflow->InstallNeutralScene(Attached.Proof,FString(),
		PatchableScene(TEXT("prepared")),FRenderSceneInstallOptions());
	if(!TestTrue(TEXT("initial scene installs"),Installed.bCommitted)){AddError(Installed.Error);return false;}
	AURLabRenderScene* Before=nullptr;
	for(TActorIterator<AURLabRenderScene> It(Fixture.World);It;++It)
		if(It->GetScene().SceneRevision==Installed.SceneRevision)Before=*It;
	if(!TestNotNull(TEXT("installed scene actor resolves"),Before))return false;
	const double InitialLight=Before->GetScene().Lights[0].Intensity;	TWeakObjectPtr<AURLabRenderScene> BeforeWeak(Before);

	FURLabRenderScenePatch Invalid;
	FURLabRenderMaterialPatch ValidMaterial;ValidMaterial.Index=0;ValidMaterial.Roughness=0.2f;
	Invalid.Materials.Add(ValidMaterial);
	FURLabRenderLightPatch ValidLight;ValidLight.Name=TEXT("key");ValidLight.Intensity=42.0f;
	Invalid.Lights.Add(ValidLight);
	FURLabRenderLightPatch MissingLight;MissingLight.Name=TEXT("missing");MissingLight.Intensity=7.0f;
	Invalid.Lights.Add(MissingLight);
	const auto Rejected=Workflow->PatchScene(Attached.Proof,Installed.SceneRevision,Invalid);
	TestFalse(TEXT("invalid last item rejects the entire patch"),Rejected.bCommitted);
	AURLabRenderScene* StillActive=nullptr;
	for(TActorIterator<AURLabRenderScene> It(Fixture.World);It;++It)
		if(It->GetScene().SceneRevision==Installed.SceneRevision)StillActive=*It;
	TestEqual(TEXT("failed patch preserves active actor"),StillActive,Before);
	TestEqual(TEXT("failed patch preserves source revision"),Before->GetScene().SceneRevision,Installed.SceneRevision);
	TestEqual(TEXT("failed patch preserves earlier valid light update"),
		static_cast<double>(Before->GetScene().Lights[0].Intensity),InitialLight);

	FURLabRenderScenePatch Accepted;
	FURLabRenderMaterialPatch Material;Material.Index=0;Material.Roughness=0.35f;
	Material.Rgba=FLinearColor(0.2f,0.4f,0.6f,1.0f);Accepted.Materials.Add(Material);
	FURLabRenderLightPatch Light;Light.Name=TEXT("key");Light.Intensity=250.0f;Accepted.Lights.Add(Light);
	FURLabRenderCameraPatch Camera;Camera.Name=TEXT("front");Camera.Mode=ERenderCameraMode::Depth;
	Camera.DepthNearCm=5.0f;Camera.DepthFarCm=900.0f;Accepted.Cameras.Add(Camera);
	FURLabRenderGeomPosePatch Pose;Pose.Index=0;Pose.BodyLocalTransform=FTransform(FVector(25.0,0.0,0.0));
	Accepted.GeomPoses.Add(Pose);
	FURLabRenderEnvironmentPatch Environment;Environment.AmbientIntensity=0.4f;
	Environment.SkyTop=FLinearColor(0.1f,0.2f,0.3f,1.0f);Accepted.Environment=Environment;
	Accepted.VisualizationFeatures=TArray<FString>{TEXT("sites")};
	const auto Patched=Workflow->PatchScene(Attached.Proof,Installed.SceneRevision,Accepted);
	if(!TestTrue(TEXT("valid patch commits"),Patched.bCommitted)){AddError(Patched.Error);return false;}
	TestNotEqual(TEXT("patch advances revision"),Patched.SceneRevision,Installed.SceneRevision);
	AURLabRenderScene* After=nullptr;
	for(TActorIterator<AURLabRenderScene> It(Fixture.World);It;++It)
		if(It->GetScene().SceneRevision==Patched.SceneRevision)After=*It;
	if(!TestNotNull(TEXT("replacement scene actor resolves"),After))return false;
	TestFalse(TEXT("staged scene replacement retires predecessor"),BeforeWeak.IsValid());
	TestEqual(TEXT("light edit persists in installed scene"),After->GetScene().Lights[0].Intensity,250.0f);
	TestEqual(TEXT("camera mode edit persists"),After->GetScene().Cameras[0].Capture.Mode,ERenderCameraMode::Depth);
	TestEqual(TEXT("geom-local transform edit persists"),After->GetScene().Geoms[0].BodyLocalTransform.GetLocation().X,25.0);
	TestEqual(TEXT("environment edit persists"),After->GetScene().Environment.AmbientIntensity,0.4f);
	TestTrue(TEXT("named visualization selection persists"),After->GetScene().Visualization.Features.Contains(TEXT("sites")));
	const auto Stale=Workflow->PatchScene(Attached.Proof,Installed.SceneRevision,Accepted);
	TestFalse(TEXT("stale expected revision rejects"),Stale.bCommitted);
	TestEqual(TEXT("stale patch leaves new revision installed"),After->GetScene().SceneRevision,Patched.SceneRevision);
	return !HasAnyErrors();
}

#endif
