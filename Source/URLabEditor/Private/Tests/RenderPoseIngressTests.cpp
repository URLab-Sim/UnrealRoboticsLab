// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Misc/ByteSwap.h"

#include "Bridge/RenderPoseIngress.h"
#include "Transport/ShmExactPayload.h"
#include "Utils/CoordinateConversion.h"

#if WITH_DEV_AUTOMATION_TESTS
namespace URLabRenderPoseIngressTest
{
FURLabPayloadStoreLimits Limits()
{
	FURLabPayloadStoreLimits Out;
	Out.MaxBlobBytes = 1024 * 1024;
	Out.MaxChunkBytes = 16;
	Out.MaxChunksPerBlob = 65536;
	Out.MaxSessionBytes = 4 * 1024 * 1024;
	Out.MaxGlobalBytes = 4 * 1024 * 1024;
	Out.MaxBlobCount = 16;
	Out.MaxSessionBlobCount = 16;
	Out.MaxSessionPhysicalObjectCount = 16;
	Out.MaxActiveSessions = 4;
	return Out;
}

bool Open(FAutomationTestBase& Test, FURLabPayloadStore& Store,
	const FString& Boot, const FString& SessionId, FURLabPayloadOwner& Out)
{
	FURLabPayloadSessionPtr Session;
	const FURLabPayloadStoreStatus Status = Store.OpenSession(Boot, SessionId, Session);
	if (!Test.TestTrue(TEXT("pose payload session opens"), Status.IsOk())) return false;
	Out = FURLabPayloadOwner::FromSession(Session);
	return true;
}

TArray<uint8> Doubles(std::initializer_list<double> Values)
{
	TArray<uint8> Out;
	Out.SetNumUninitialized(static_cast<int32>(Values.size() * sizeof(double)));
	int32 Index = 0;
	for (double Value : Values)
	{
		uint64 Bits = 0;
		FMemory::Memcpy(&Bits, &Value, sizeof(Bits));
		Bits = INTEL_ORDER64(Bits);
		FMemory::Memcpy(Out.GetData() + Index * sizeof(double), &Bits, sizeof(Bits));
		++Index;
	}
	return Out;
}

bool Put(FAutomationTestBase& Test, FURLabPayloadStore& Store,
	const FURLabPayloadOwner& Owner, TArray<uint8> Bytes,
	const FURLabShmExactRegionPtr& Region, FURLabRenderSourceBlobRef& Out)
{
	FURLabPayloadProducerPtr Producer;
	FURLabPayloadStoreStatus Status = Region
		? Store.BeginShmProduce(Owner, Region, Bytes.Num(), Producer)
		: Store.BeginProduce(Owner, Bytes.Num(), Producer);
	if (!Test.TestTrue(TEXT("pose producer admits"), Status.IsOk())) return false;
	FURLabStoredPayloadDescriptor Descriptor;
	FURLabPayloadPin Pin;
	Status = Store.SealProduced(Owner, Producer, MoveTemp(Bytes), Descriptor, Pin);
	if (!Test.TestTrue(TEXT("pose payload seals"), Status.IsOk())) return false;
	Out.BlobId = Descriptor.BlobId;
	Out.Sha256 = Descriptor.Sha256;
	return true;
}

FURLabRenderSourceDescription Description(bool bQpos, int32 Width)
{
	FURLabRenderSourceDescription Out;
	Out.SceneRevision = TEXT("pose-scene");
	Out.SceneGeneration = 7;
	Out.BodyNames = {TEXT("world"), TEXT("body")};
	FRenderCameraEffectiveConfiguration Camera;
	Camera.CanonicalName = TEXT("model/camera");
	Out.ModelCameras.Add(Camera);
	Out.UserCamera = FRenderCameraEffectiveConfiguration();
	Out.UserCamera->CanonicalName = TEXT("user");
	Out.bSupportsQpos = bQpos;
	Out.QposWidth = Width;
	return Out;
}

FRenderFrameIdentity Identity()
{
	FRenderFrameIdentity Out;
	Out.SceneRevision = TEXT("pose-scene");
	Out.FrameId = 11;
	Out.SourceTime = 1.25;
	Out.bHasSourceTime = true;
	Out.ClockEpoch = 3;
	return Out;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderPoseQposTest,
	"URLab.Workflows.RenderPoseIngress.QposIdentityAndZeroWidth",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderPoseQposTest::RunTest(const FString&)
{
	using namespace URLabRenderPoseIngressTest;
	auto Store = MakeShared<FURLabPayloadStore, ESPMode::ThreadSafe>(TEXT("pose-qpos"), Limits());
	FURLabPayloadOwner Owner;
	if (!Open(*this, *Store, TEXT("pose-qpos"), TEXT("owner"), Owner)) return false;
	FURLabRenderSourceBlobRef Ref;
	if (!Put(*this, *Store, Owner, Doubles({0.25, -0.5}), {}, Ref)) return false;
	const FURLabPayloadStoreStats Baseline = Store->GetStats();
	FURLabRenderPoseRequest Request;
	Request.Kind = EURLabRenderPoseKind::Qpos;
	Request.Payload = Ref;
	Request.Identity = Identity();
	FURLabDecodedRenderPose Pose;
	FString Error;
	if (!TestTrue(TEXT("exact qpos decodes"), FURLabRenderPoseIngress::Decode(
		Store, Owner, Description(true, 2), Request, Pose, Error))) return false;
	TestEqual(TEXT("qpos width is exact"), Pose.Qpos.Num(), 2);
	TestEqual(TEXT("qpos first scalar is exact"), Pose.Qpos[0], 0.25);
	TestTrue(TEXT("qpos retains presentation capacity"), Pose.PresentationLease.IsValid());
	Pose.Reset();
	TestEqual(TEXT("qpos reset frees its array allocation"),
		Pose.Qpos.GetAllocatedSize(), SIZE_T(0));
	TestEqual(TEXT("qpos release restores charged bytes"), Store->GetStats().PhysicalBytes,
		Baseline.PhysicalBytes);

	FURLabRenderPoseRequest Empty;
	Empty.Kind = EURLabRenderPoseKind::Qpos;
	Empty.Identity = Identity();
	TestTrue(TEXT("true zero-width FK accepts absent payload"),
		FURLabRenderPoseIngress::Decode(Store, Owner, Description(true, 0),
			Empty, Pose, Error));
	TestTrue(TEXT("zero-width qpos is empty"), Pose.Qpos.IsEmpty());
	TestFalse(TEXT("neutral source rejects absent qpos payload"),
		FURLabRenderPoseIngress::Decode(Store, Owner, Description(false, 0),
			Empty, Pose, Error));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderPoseQposExtrasTest,
	"URLab.Workflows.RenderPoseIngress.QposMocapAndUserCamera",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderPoseQposExtrasTest::RunTest(const FString&)
{
	using namespace URLabRenderPoseIngressTest;
	auto Store = MakeShared<FURLabPayloadStore, ESPMode::ThreadSafe>(TEXT("pose-extras"), Limits());
	FURLabPayloadOwner Owner;
	if (!Open(*this, *Store, TEXT("pose-extras"), TEXT("owner"), Owner)) return false;
	FURLabRenderSourceDescription Source = Description(true, 1);
	Source.MocapCount = 1;
	FURLabRenderSourceBlobRef Full;
	if (!Put(*this, *Store, Owner, Doubles({0.5, 1, 2, 3, 1, 0, 0, 0, 4, 5, 6, 1, 0, 0, 0}),
		{}, Full)) return false;
	FURLabRenderPoseRequest Request;
	Request.Kind = EURLabRenderPoseKind::Qpos;
	Request.Payload = Full;
	Request.Identity = Identity();
	Request.bMocap = true;
	Request.bUserCamera = true;
	FURLabDecodedRenderPose Pose;
	FString Error;
	if (!TestTrue(TEXT("qpos with mocap and user camera decodes"), FURLabRenderPoseIngress::Decode(
		Store, Owner, Source, Request, Pose, Error))) return false;
	TestEqual(TEXT("qpos prefix is exact"), Pose.Qpos.Num(), 1);
	TestEqual(TEXT("mocap follows qpos"), Pose.Mocap.Num(), 7);
	TestEqual(TEXT("mocap position is RHS metres"), Pose.Mocap[0], 1.0);
	if (TestTrue(TEXT("user camera pose decodes"), Pose.UserCameraSceneTransform.IsSet()))
	{
		const double Position[3] = {4, 5, 6};
		const double Wxyz[4] = {1, 0, 0, 0};
		TestTrue(TEXT("user camera uses the optical convention"),
			Pose.UserCameraSceneTransform->Equals(
				URLabCoordinateConversion::RightHandedOpticalPoseToUnreal(Position, Wxyz)));
	}

	Request.bUserCamera = false;
	TestFalse(TEXT("payload length must match the selected extras"),
		FURLabRenderPoseIngress::Decode(Store, Owner, Source, Request, Pose, Error));
	FURLabRenderSourceBlobRef MocapOnly;
	if (!Put(*this, *Store, Owner, Doubles({0.5, 1, 2, 3, 1, 0, 0, 0}), {}, MocapOnly)) return false;
	Request.Payload = MocapOnly;
	TestTrue(TEXT("mocap without user camera decodes"),
		FURLabRenderPoseIngress::Decode(Store, Owner, Source, Request, Pose, Error));
	TestFalse(TEXT("mocap-only frame carries no user camera"), Pose.UserCameraSceneTransform.IsSet());

	FURLabRenderSourceBlobRef BadQuat;
	if (!Put(*this, *Store, Owner, Doubles({0.5, 1, 2, 3, 2, 0, 0, 0}), {}, BadQuat)) return false;
	Request.Payload = BadQuat;
	TestFalse(TEXT("non-unit mocap quaternion is rejected"),
		FURLabRenderPoseIngress::Decode(Store, Owner, Source, Request, Pose, Error));

	FURLabRenderSourceDescription NoUser = Source;
	NoUser.UserCamera.Reset();
	Request.Payload = Full;
	Request.bUserCamera = true;
	TestFalse(TEXT("user camera extra requires an installed user camera"),
		FURLabRenderPoseIngress::Decode(Store, Owner, NoUser, Request, Pose, Error));
	Request.Kind = EURLabRenderPoseKind::Cartesian;
	TestFalse(TEXT("extras are qpos only"),
		FURLabRenderPoseIngress::Decode(Store, Owner, Source, Request, Pose, Error));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderPoseCartesianTest,
	"URLab.Workflows.RenderPoseIngress.CartesianShmValidationAndRollback",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderPoseCartesianTest::RunTest(const FString&)
{
	using namespace URLabRenderPoseIngressTest;
	auto Store = MakeShared<FURLabPayloadStore, ESPMode::ThreadSafe>(TEXT("pose-cart"), Limits());
	FURLabPayloadOwner Owner;
	if (!Open(*this, *Store, TEXT("pose-cart"), TEXT("owner"), Owner)) return false;
	FURLabShmExactRegionPtr Region;
	if (!TestTrue(TEXT("pose SHM region creates"),
		FURLabShmExactRegion::CreateForBoot(TEXT("pose-cart"), Region).IsOk())) return false;
	// world, body, model camera, user camera; each is xyz metres then wxyz.
	TArray<uint8> Bytes = Doubles({
		1, 2, 3, 1, 0, 0, 0,
		4, 5, 6, 1, 0, 0, 0,
		7, 8, 9, 1, 0, 0, 0,
		10, 11, 12, 1.0000005, 0, 0, 0});
	FURLabRenderSourceBlobRef Ref;
	if (!Put(*this, *Store, Owner, MoveTemp(Bytes), Region, Ref)) return false;
	const FURLabPayloadStoreStats Baseline = Store->GetStats();
	FURLabRenderPoseRequest Request;
	Request.Kind = EURLabRenderPoseKind::Cartesian;
	Request.Payload = Ref;
	Request.Identity = Identity();
	FURLabDecodedRenderPose Pose;
	FString Error;
	if (!TestTrue(TEXT("chunked SHM Cartesian pose decodes"),
		FURLabRenderPoseIngress::Decode(Store, Owner, Description(false, 0),
			Request, Pose, Error))) return false;
	TestEqual(TEXT("body order is exact"), Pose.Cartesian.BodySceneTransforms.Num(), 2);
	TestEqual(TEXT("camera order is exact"), Pose.Cartesian.CameraSceneTransforms.Num(), 1);
	TestTrue(TEXT("user transform is present"), Pose.Cartesian.UserCameraSceneTransform.IsSet());
	TestEqual(TEXT("RHS metres convert to UE centimetres with handedness"),
		Pose.Cartesian.BodySceneTransforms[0].GetTranslation(), FVector(100, -200, 300));
	TestTrue(TEXT("body identity remains the generic RH identity"),
		Pose.Cartesian.BodySceneTransforms[0].GetRotation().Equals(FQuat::Identity));
	const FQuat CameraRotation = Pose.Cartesian.CameraSceneTransforms[0].GetRotation();
	TestTrue(TEXT("optical -Z becomes Unreal camera +X forward"),
		CameraRotation.RotateVector(FVector::ForwardVector).Equals(
			FVector(0, 0, -1), KINDA_SMALL_NUMBER));
	TestTrue(TEXT("optical +Y becomes Unreal camera +Z up"),
		CameraRotation.RotateVector(FVector::UpVector).Equals(
			FVector(0, -1, 0), KINDA_SMALL_NUMBER));
	const double Origin[3] = {0, 0, 0};
	const double IdentityWxyz[4] = {1, 0, 0, 0};
	TestTrue(TEXT("Cartesian camera matches the qpos adapter's shared optical conversion"),
		CameraRotation.Equals(URLabCoordinateConversion::RightHandedOpticalPoseToUnreal(
			Origin, IdentityWxyz).GetRotation(), KINDA_SMALL_NUMBER));
	TestTrue(TEXT("user camera uses the same optical convention"),
		Pose.Cartesian.UserCameraSceneTransform->GetRotation().Equals(
			CameraRotation, KINDA_SMALL_NUMBER));
	TestTrue(TEXT("near-unit quaternion is normalized"),
		Pose.Cartesian.UserCameraSceneTransform->GetRotation().IsNormalized());
	Pose.Reset();
	TestEqual(TEXT("Cartesian reset frees body array allocation"),
		Pose.Cartesian.BodySceneTransforms.GetAllocatedSize(), SIZE_T(0));
	TestEqual(TEXT("Cartesian reset frees camera array allocation"),
		Pose.Cartesian.CameraSceneTransforms.GetAllocatedSize(), SIZE_T(0));
	TestEqual(TEXT("Cartesian release restores charged bytes"), Store->GetStats().PhysicalBytes,
		Baseline.PhysicalBytes);

	FURLabRenderSourceBlobRef BadRef;
	if (!Put(*this, *Store, Owner, Doubles({
		0,0,0, 2,0,0,0, 0,0,0, 1,0,0,0,
		0,0,0, 1,0,0,0, 0,0,0, 1,0,0,0}), {}, BadRef)) return false;
	Request.Payload = BadRef;
	const FURLabPayloadStoreStats BeforeFailure = Store->GetStats();
	TestFalse(TEXT("nonunit quaternion is rejected"), FURLabRenderPoseIngress::Decode(
		Store, Owner, Description(false, 0), Request, Pose, Error));
	TestEqual(TEXT("failed decode rolls back charged bytes"), Store->GetStats().PhysicalBytes,
		BeforeFailure.PhysicalBytes);
	return !HasAnyErrors();
}
#endif
