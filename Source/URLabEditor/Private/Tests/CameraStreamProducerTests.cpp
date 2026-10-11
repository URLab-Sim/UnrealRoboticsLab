#include "CoreMinimal.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformProcess.h"
#include "Misc/AutomationTest.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "Misc/ScopeLock.h"
#include "Transport/CameraStreamFrameCodec.h"
#include "Transport/CameraStreamProducer.h"
#include "Transport/LockedLatestFrame.h"
#include "Transport/PoseStreamZmqReader.h"

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabCameraStreamProducerTest,
	"URLab.Transport.CameraStream.CodecAndProducer",
	EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FURLabCameraStreamProducerTest::RunTest(const FString&)
{
#if !PLATFORM_LINUX
	AddWarning(TEXT("camera stream exact-region test requires Linux"));return true;
#else
	const FString Base=FPaths::Combine(FPaths::ProjectSavedDir(),TEXT("URLabTests"),
		TEXT("CameraProducer-")+FGuid::NewGuid().ToString(EGuidFormats::Digits));
	FURLabShmExactRegionPtr Region;if(!TestTrue(TEXT("region creates"),
		FURLabShmExactRegion::Create(TEXT("camera-region"),Base,Region).IsOk()))return false;
	FURLabCameraStreamProducerConfig Config;Config.BindEndpoint=TEXT("tcp://127.0.0.1:*");
	Config.MaxFrameBytes=4096;Config.ShmRegion=Region;FString Error;
	TUniquePtr<FURLabCameraStreamProducer> Producer=FURLabCameraStreamProducer::Create(Config,Error);
	if(!TestTrue(TEXT("producer starts"),Producer.IsValid())){AddError(Error);return false;}
	FURLabCameraStreamDescriptor Descriptor;FURLabCameraStreamDelivery Delivery;
	TestFalse(TEXT("describe requires committed source"),Producer->Describe(true,Descriptor,Delivery,Error));
	FURLabCameraStreamConfiguration Camera;Camera.SourceRevision=TEXT("scene-a");
	Camera.CameraName=TEXT("overview");Camera.CameraConfigurationRevision=3;
	Camera.Encoding=TEXT("rgba8");Camera.Width=2;Camera.Height=1;Camera.StrideBytes=8;
	FURLabCameraStreamDescriptor First;TestTrue(TEXT("camera commits"),Producer->CommitCamera(Camera,First,Error));
	FURLabCameraStreamDescriptor Same;TestTrue(TEXT("same camera commit is idempotent"),
		Producer->CommitCamera(Camera,Same,Error));
	TestEqual(TEXT("same camera retains generation"),Same.StreamGeneration,First.StreamGeneration);
	TestTrue(TEXT("eligible deliveries describe"),Producer->Describe(true,Descriptor,Delivery,Error));
	TestFalse(TEXT("descriptor has no foreign SHM without proof"),([&]{FURLabCameraStreamDelivery D;FURLabCameraStreamDescriptor S;return Producer->Describe(false,S,D,Error)&&D.ShmObject.IsSet();})());
	FURLabShmLiveObjectPtr Consumer;if(!TestTrue(TEXT("live object opens"),Region->OpenLiveObject(Delivery.ShmObject.GetValue(),Consumer).IsOk()))return false;
	auto Reader=FURLabLockedLatestFrameReader::Create(Consumer,4096,Error);
	TArray<uint8>Zmq;FCriticalSection Mutex;auto ZmqReader=FURLabPoseStreamZmqReader::Create(
		Delivery.ZmqEndpoint,Delivery.ZmqTopic,4096,[&](TArray<uint8>&&B){FScopeLock L(&Mutex);Zmq=MoveTemp(B);},Error);
	if(!TestTrue(TEXT("ZMQ reader starts"),ZmqReader.IsValid()))return false;
	FPlatformProcess::SleepNoStats(0.05f);
	for(int32 I=0;I<3;++I)
	{
		auto F=MakeShared<FRenderCameraFrame,ESPMode::ThreadSafe>();
		F->Identity.SceneRevision=TEXT("scene-a");F->Identity.FrameId=7;
		F->Identity.ClockEpoch=2;F->Identity.bHasSourceTime=true;F->Identity.SourceTime=.25;
		F->Mode=ERenderCameraMode::Real;F->CameraConfigurationRevision=3;
		F->CaptureSerial=4+I;F->Width=2;F->Height=1;F->CaptureUnixTime=10.;
		F->Color={FColor::Red,FColor::Blue};
		TestTrue(TEXT("completed frame submission is accepted"),Producer->SubmitCompleted(F,Error));
	}
	TArray<uint8>Bytes;EURLabLatestFrameRead Read=EURLabLatestFrameRead::NoFrame;
	for(int32 I=0;I<200&&Read==EURLabLatestFrameRead::NoFrame;++I){Read=Reader->ReadLatest(Bytes,Error);FPlatformProcess::SleepNoStats(.001f);}
	FURLabCameraStreamFrame Decoded;TestEqual(TEXT("SHM has complete frame"),Read,EURLabLatestFrameRead::Frame);
	TestTrue(TEXT("SHM frame decodes"),FURLabCameraStreamFrameCodec::Decode(Bytes,Decoded,Error,8));
	TestEqual(TEXT("pixels preserved"),Decoded.Pixels,TArray<uint8>({255,0,0,255,0,0,255,255}));
	for(int32 I=0;I<200;++I){{FScopeLock L(&Mutex);if(!Zmq.IsEmpty())break;}FPlatformProcess::SleepNoStats(.001f);}
	{FScopeLock L(&Mutex);TestTrue(TEXT("ZMQ coexists"),!Zmq.IsEmpty()&&FURLabCameraStreamFrameCodec::Decode(Zmq,Decoded,Error,8));}
	auto RacingOld=MakeShared<FRenderCameraFrame,ESPMode::ThreadSafe>();
	RacingOld->Identity.SceneRevision=TEXT("scene-a");RacingOld->Identity.FrameId=70;
	RacingOld->Identity.ClockEpoch=2;RacingOld->Mode=ERenderCameraMode::Real;
	RacingOld->CameraConfigurationRevision=3;RacingOld->CaptureSerial=70;
	RacingOld->Width=2;RacingOld->Height=1;RacingOld->Color={FColor::Red,FColor::Red};
	TestTrue(TEXT("old generation may be in flight at replacement"),
		Producer->SubmitCompleted(RacingOld,Error));
	Camera.CameraConfigurationRevision=4;FURLabCameraStreamDescriptor Replaced;
	TestTrue(TEXT("configuration replacement commits"),Producer->CommitCamera(Camera,Replaced,Error));
	TestTrue(TEXT("configuration replacement advances generation"),
		Replaced.StreamGeneration>First.StreamGeneration);
	Bytes.Reset();
	TestEqual(TEXT("replacement publishes no stale SHM generation"),
		Reader->ReadLatest(Bytes,Error),EURLabLatestFrameRead::NoFrame);
	auto Current=MakeShared<FRenderCameraFrame,ESPMode::ThreadSafe>();
	Current->Identity.SceneRevision=TEXT("scene-a");Current->Identity.FrameId=8;
	Current->Identity.ClockEpoch=2;Current->Mode=ERenderCameraMode::Real;
	Current->CameraConfigurationRevision=4;Current->CaptureSerial=9;
	Current->Width=2;Current->Height=1;Current->Color={FColor::Green,FColor::Blue};
	TestTrue(TEXT("current generation frame submits"),Producer->SubmitCompleted(Current,Error));
	Read=EURLabLatestFrameRead::NoFrame;
	for(int32 I=0;I<200&&Read==EURLabLatestFrameRead::NoFrame;++I)
	{Read=Reader->ReadLatest(Bytes,Error);FPlatformProcess::SleepNoStats(.001f);}
	TestEqual(TEXT("current generation replaces cleared SHM"),Read,EURLabLatestFrameRead::Frame);
	TestTrue(TEXT("current generation decodes"),
		FURLabCameraStreamFrameCodec::Decode(Bytes,Decoded,Error,8));
	TestEqual(TEXT("SHM exposes only replacement generation"),
		Decoded.StreamGeneration,Replaced.StreamGeneration);
	ZmqReader->Stop();Reader.Reset();Consumer.Reset();Producer->Stop();Producer.Reset();Region.Reset();
	IFileManager::Get().DeleteDirectory(*Base,false,true);return !HasAnyErrors();
#endif
}
#endif
