#include "Misc/AutomationTest.h"

#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Render/RenderWorldSubsystem.h"
#include "Workflows/ViewerWorldSubsystem.h"

#if WITH_DEV_AUTOMATION_TESTS
namespace URLabViewerWorkflowTest
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
		if(UURLabViewerWorldSubsystem* Viewer=World->GetSubsystem<UURLabViewerWorldSubsystem>())
			Viewer->ShutdownWorkflow();
		World->DestroyWorld(false); GEngine->DestroyWorldContext(World);
	}
};

FURLabRenderOwnerSessionScopePtr Scope(const TCHAR* Name)
{
	FString Error; return FURLabRenderOwnerSessionScope::Create(Name,Error);
}

FRenderSceneInstallPayload ScenePayload(const FString& Revision,double X,bool bWithLight=false)
{
	FRenderScene Scene; Scene.SceneRevision=Revision;
	Scene.Bodies.AddDefaulted_GetRef().Name=TEXT("body");
	FRenderSceneResources Resources; Resources.SceneRevision=Revision;
	FRenderFrame Frame; Frame.Identity.SceneRevision=Revision; Frame.Identity.FrameId=1;
	Frame.BodySceneTransforms.Add(FTransform(FVector(X,0,0)));
	if(bWithLight)
	{
		FRenderLightDesc& Light=Scene.Lights.AddDefaulted_GetRef();
		Light.Name=TEXT("key");Light.Kind=ERenderLightKind::Point;Light.Intensity=500.0f;
		Light.RangeCm=500.0f;
		Frame.LightSceneTransforms.Add(FTransform(FVector(0,0,200)));
	}
	return FRenderSceneInstallPayload(MoveTemp(Scene),MoveTemp(Resources),MoveTemp(Frame));
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabViewerAuthorityStreamTest,
	"URLab.Workflows.Viewer.AuthoritySourceBindingAndStaleFreeze",
	EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)

bool FURLabViewerAuthorityStreamTest::RunTest(const FString&)
{
	using namespace URLabViewerWorkflowTest;
	FWorldFixture Fixture;
	if(!TestTrue(TEXT("world creates"),Fixture.Create()))return false;
	UURLabViewerWorldSubsystem* Viewer=Fixture.World->GetSubsystem<UURLabViewerWorldSubsystem>();
	if(!TestNotNull(TEXT("viewer subsystem exists"),Viewer))return false;
	TestFalse(TEXT("construction is inert"),Viewer->IsActive());
	FString Error;
	if(!TestTrue(TEXT("viewer activates"),Viewer->Activate(Error))){AddError(Error);return false;}
	auto OwnerScope=Scope(TEXT("viewer-owner"));
	auto Attached=Viewer->AttachOwner(OwnerScope);
	if(!TestTrue(TEXT("owner attaches"),Attached.IsOk()))return false;
	auto Installed=Viewer->InstallNeutralScene(Attached.Proof,FString(),
		ScenePayload(TEXT("local-r1"),0.0,true),FRenderSceneInstallOptions());
	if(!TestTrue(TEXT("owner installs source"),Installed.bCommitted)){AddError(Installed.Error);return false;}
	FURLabViewerStreamDescriptor Descriptor;
	Descriptor.ProducerBootId=TEXT("producer-boot"); Descriptor.StreamId=TEXT("poses");
	Descriptor.StreamGeneration=3; Descriptor.SourceRevision=TEXT("producer-r8");
	if(!TestTrue(TEXT("owner binds producer stream"),
		Viewer->BindStream(Attached.Proof,Installed.SceneRevision,Descriptor,Error)))
	{AddError(Error);return false;}
	FRenderFrame Frame; Frame.Identity.SceneRevision=Installed.SceneRevision;
	Frame.Identity.FrameId=9; Frame.Identity.ClockEpoch=2;
	Frame.BodySceneTransforms.Add(FTransform(FVector(50,0,0)));
	Frame.LightSceneTransforms.Add(FTransform(FVector(0,100,200)));
	FURLabViewerStreamFrameHeader Header;
	Header.ProducerBootId=Descriptor.ProducerBootId; Header.StreamId=Descriptor.StreamId;
	Header.StreamGeneration=Descriptor.StreamGeneration; Header.SourceRevision=Descriptor.SourceRevision;
	Header.Sequence=1; Header.FrameId=9; Header.ClockEpoch=2;
	TestTrue(TEXT("matching frame applies"),Viewer->ApplyStreamFrame(Attached.Proof,Header,Frame).bAccepted);
	TestFalse(TEXT("duplicate sequence rejects"),Viewer->ApplyStreamFrame(Attached.Proof,Header,Frame).bAccepted);
	Header.Sequence=2; Header.SourceRevision=TEXT("stale");
	TestFalse(TEXT("stale producer revision rejects"),Viewer->ApplyStreamFrame(Attached.Proof,Header,Frame).bAccepted);
	TestTrue(TEXT("same-generation malformed identity does not retire binding"),
		Viewer->Status().bStreamBound);
	Header.StreamGeneration=Descriptor.StreamGeneration-1;
	Header.SourceRevision=Descriptor.SourceRevision;
	TestFalse(TEXT("out-of-order old generation rejects"),
		Viewer->ApplyStreamFrame(Attached.Proof,Header,Frame).bAccepted);
	TestTrue(TEXT("old generation does not retire binding"),Viewer->Status().bStreamBound);
	Header.StreamGeneration=Descriptor.StreamGeneration+1;
	Header.SourceRevision=TEXT("producer-r9");
	TestFalse(TEXT("authoritative successor identity is a barrier"),
		Viewer->ApplyStreamFrame(Attached.Proof,Header,Frame).bAccepted);
	TestFalse(TEXT("successor retires exact stream binding"),Viewer->Status().bStreamBound);
	TestTrue(TEXT("successor preserves installed scene as stale"),
		Viewer->Status().bHasInstalledSource && Viewer->Status().bDisplayStale);
	const auto Released=Viewer->ReleaseOwnerCredentials(OwnerScope,Attached.Proof.GetGeneration(),
		Attached.Proof.GetOpaqueToken());
	TestTrue(TEXT("exact owner releases"),Released.IsOk());
	const FURLabViewerStatus Status=Viewer->Status();
	TestTrue(TEXT("release retains installed source"),Status.bHasInstalledSource);
	TestTrue(TEXT("release freezes stale display"),Status.bDisplayStale);
	TestFalse(TEXT("release clears stream binding"),Status.bStreamBound);
	TestEqual(TEXT("release retains scene revision"),Status.SceneRevision,Installed.SceneRevision);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabViewerReplacementTest,
	"URLab.Workflows.Viewer.ReplacementClearsBindingAndRejectsOldStream",
	EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)

bool FURLabViewerReplacementTest::RunTest(const FString&)
{
	using namespace URLabViewerWorkflowTest;
	FWorldFixture Fixture;
	if(!Fixture.Create())return false;
	auto* Viewer=Fixture.World->GetSubsystem<UURLabViewerWorldSubsystem>(); FString Error;
	if(!Viewer||!Viewer->Activate(Error))return false;
	auto S=Scope(TEXT("replace-owner")); auto A=Viewer->AttachOwner(S);
	if(!A.IsOk())return false;
	auto First=Viewer->InstallNeutralScene(A.Proof,FString(),ScenePayload(TEXT("r1"),0),{});
	FURLabViewerStreamDescriptor D; D.ProducerBootId=TEXT("boot"); D.StreamId=TEXT("stream");
	D.StreamGeneration=1; D.SourceRevision=TEXT("source-1");
	if(!First.bCommitted||!Viewer->BindStream(A.Proof,First.SceneRevision,D,Error))return false;
	auto Second=Viewer->InstallNeutralScene(A.Proof,First.SceneRevision,ScenePayload(TEXT("r2"),10),{});
	TestTrue(TEXT("replacement commits"),Second.bCommitted);
	TestFalse(TEXT("replacement clears prior stream"),Viewer->Status().bStreamBound);
	FRenderFrame Frame; Frame.Identity.SceneRevision=Second.SceneRevision; Frame.Identity.FrameId=2;
	Frame.BodySceneTransforms.Add(FTransform::Identity);
	FURLabViewerStreamFrameHeader H; H.ProducerBootId=D.ProducerBootId; H.StreamId=D.StreamId;
	H.StreamGeneration=D.StreamGeneration; H.SourceRevision=D.SourceRevision; H.Sequence=1; H.FrameId=2;
	TestFalse(TEXT("old stream cannot publish after replacement"),Viewer->ApplyStreamFrame(A.Proof,H,Frame).bAccepted);
	return !HasAnyErrors();
}
#endif
