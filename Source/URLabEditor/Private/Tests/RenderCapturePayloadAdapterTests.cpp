// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Misc/AutomationTest.h"

#include "Bridge/RenderCapturePayloadAdapter.h"
#include "Transport/ShmExactPayload.h"
#include "Utils/AssetCache.h"

#include <limits>

#if WITH_DEV_AUTOMATION_TESTS
namespace URLabRenderCapturePayloadAdapterTest
{
struct FPlanAuthority
{
	UWorld* World = nullptr;
	UURLabRenderWorldSubsystem* Coordinator = nullptr;
	FURLabRenderProducerRegistration Producer;

	bool Create(FString& OutError)
	{
		World = UWorld::CreateWorld(EWorldType::Game, false);
		if (!World || !GEngine) return false;
		FWorldContext& Context = GEngine->CreateNewWorldContext(EWorldType::Game);
		Context.SetCurrentWorld(World);
		Coordinator = World->GetSubsystem<UURLabRenderWorldSubsystem>();
		return Coordinator && Coordinator->RegisterProducer(Producer, OutError);
	}

	~FPlanAuthority()
	{
		if (!World) return;
		if (Coordinator && Coordinator->IsRegistrationCurrent(Producer))
		{
			FString Error;
			Coordinator->ReleaseProducer(Producer, Error);
		}
		World->DestroyWorld(false);
		if (GEngine) GEngine->DestroyWorldContext(World);
	}
};

FURLabPayloadStoreLimits Limits(int64 Bytes = 1024)
{
	FURLabPayloadStoreLimits Out;
	Out.MaxBlobBytes = Bytes;
	Out.MaxChunkBytes = static_cast<int32>(Bytes);
	Out.MaxChunksPerBlob = 4;
	Out.MaxSessionBytes = Bytes;
	Out.MaxGlobalBytes = Bytes;
	Out.MaxBlobCount = 4;
	Out.MaxSessionBlobCount = 4;
	Out.MaxSessionPhysicalObjectCount = 4;
	Out.MaxActiveSessions = 2;
	return Out;
}

bool Open(FAutomationTestBase& Test, FURLabPayloadStore& Store,
	const TCHAR* Boot, const TCHAR* SessionId, FURLabPayloadOwner& OutOwner,
	FURLabPayloadSessionPtr* OutSession = nullptr)
{
	FURLabPayloadSessionPtr Session;
	const FURLabPayloadStoreStatus Status = Store.OpenSession(Boot, SessionId, Session);
	if (!Test.TestTrue(TEXT("payload session opens"), Status.IsOk())) return false;
	OutOwner = FURLabPayloadOwner::FromSession(Session);
	if (OutSession) *OutSession = Session;
	return true;
}

FRenderFrameIdentity Identity(uint64 FrameId = 41)
{
	FRenderFrameIdentity Out;
	Out.SceneRevision = TEXT("payload-scene");
	Out.FrameId = FrameId;
	Out.SourceTime = 4.25;
	Out.ClockEpoch = 9;
	Out.bHasSourceTime = true;
	return Out;
}

FURLabRenderExactCapturePlan Plan(
	const FURLabRenderProducerRegistration& Producer,
	std::initializer_list<TPair<const TCHAR*, ERenderCameraMode>> Cameras)
{
	FURLabRenderExactCapturePlan Out;
	Out.Producer = Producer;
	Out.SceneRevision = TEXT("payload-scene");
	Out.SceneGeneration = 3;
	for (const TPair<const TCHAR*, ERenderCameraMode>& Camera : Cameras)
	{
		FRenderCameraConfigurationSnapshot& Snapshot = Out.Cameras.AddDefaulted_GetRef();
		Snapshot.CanonicalName = Camera.Key;
		Snapshot.Mode = Camera.Value;
		Snapshot.Resolution = FIntPoint(2, 1);
		Snapshot.ConfigurationRevision = static_cast<uint64>(Out.Cameras.Num() + 10);
		int32 Elements = 0;
		FString Error;
		RenderCheckedCameraBufferSizing(Snapshot.Mode, Snapshot.Resolution, Elements,
			Snapshot.LogicalPixelBytes, Snapshot.PhysicalArrayBytes, Error);
		Out.AggregateLogicalPixelBytes += Snapshot.LogicalPixelBytes;
		Out.AggregatePhysicalArrayBytes += Snapshot.PhysicalArrayBytes;
	}
	return Out;
}

TSharedPtr<const FRenderCameraFrame> Frame(
	const FRenderCameraConfigurationSnapshot& Config, uint64 Serial)
{
	auto Out = MakeShared<FRenderCameraFrame>();
	Out->Identity = Identity();
	Out->Mode = Config.Mode;
	Out->CameraConfigurationRevision = Config.ConfigurationRevision;
	Out->CaptureSerial = Serial;
	Out->Width = Config.Resolution.X;
	Out->Height = Config.Resolution.Y;
	if (Config.Mode == ERenderCameraMode::Real)
	{
		Out->Color = {FColor(1, 2, 3, 4), FColor(5, 6, 7, 8)};
	}
	else if (Config.Mode == ERenderCameraMode::Depth)
	{
		// Native frames retain UE centimetres. Materialization clamps to the
		// effective plan and encodes typed payload depth in metres.
		Out->Depth = {1.0f, std::numeric_limits<float>::infinity()};
	}
	else
	{
		Out->Seg = {10, 11, -12, 13};
	}
	return Out;
}

FURLabRenderCaptureJobSnapshot Snapshot(const FURLabRenderExactCapturePlan& Plan)
{
	FURLabRenderCaptureJobSnapshot Out;
	Out.State = EURLabRenderCaptureJobState::Succeeded;
	Out.SceneRevision = Plan.SceneRevision;
	for (int32 Index = 0; Index < Plan.Cameras.Num(); ++Index)
	{
		Out.CameraNames.Add(Plan.Cameras[Index].CanonicalName);
		Out.Frames.Add(Frame(Plan.Cameras[Index], static_cast<uint64>(100 + Index)));
	}
	return Out;
}

TArray<uint8> Read(const TSharedRef<FURLabPayloadStore, ESPMode::ThreadSafe>& Store,
	const FURLabPayloadPin& Pin)
{
	TConstArrayView<uint8> Network;
	if (Pin && Pin->TryGetNetworkBytes(Network))
	{
		TArray<uint8> Out;
		Out.Append(Network.GetData(), Network.Num());
		return Out;
	}
	FURLabPayloadCopyPin Copy;
	if (!Pin || !Store->CopyPinnedRange(Pin, 0, Pin->Num(), Copy).IsOk()) return {};
	return Copy->GetBytes();
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabCapturePayloadNetworkEncodingTest,
	"URLab.Workflows.RenderCapturePayload.NetworkMixedEncodingAndIdempotence",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabCapturePayloadNetworkEncodingTest::RunTest(const FString&)
{
	using namespace URLabRenderCapturePayloadAdapterTest;
	FPlanAuthority Authority;
	FString AuthorityError;
	if (!TestTrue(TEXT("plan authority creates"), Authority.Create(AuthorityError))) return false;
	auto Store = MakeShared<FURLabPayloadStore, ESPMode::ThreadSafe>(TEXT("payload-boot"), Limits());
	FURLabPayloadOwner Owner;
	if (!Open(*this, *Store, TEXT("payload-boot"), TEXT("network"), Owner)) return false;
	const FURLabRenderExactCapturePlan CapturePlan = Plan(Authority.Producer, {
		{TEXT("rgba"), ERenderCameraMode::Real},
		{TEXT("depth"), ERenderCameraMode::Depth},
		{TEXT("seg"), ERenderCameraMode::InstanceSegmentation}});
	FURLabCapturePayloadAdmissionResult Admission = FURLabCapturePayloadAdmission::Admit(
		Store, Owner, EURLabPayloadBackingKind::NetworkBytes, {}, CapturePlan);
	if (!TestTrue(TEXT("network capture capacity admits"), Admission.bAccepted)) return false;
	FURLabRenderCaptureJobSnapshot Complete = Snapshot(CapturePlan);
	FURLabCapturePayloadMaterialization First =
		Admission.Admission->Materialize(Identity(), Complete);
	if (!TestTrue(TEXT("mixed capture materializes"), First.bSucceeded)) return false;
	const TArray<uint8> Bytes = Read(Store, First.Pin);
	TestEqual(TEXT("one blob has the planned logical size"), Bytes.Num(), 32);
	TestEqual(TEXT("descriptor hashes exact encoded bytes"), First.Descriptor.Sha256,
		FURLabAssetCache::Sha256Hex(Bytes));
	TestEqual(TEXT("network backing remains explicit"), First.Descriptor.BackingKind,
		EURLabPayloadBackingKind::NetworkBytes);
	TestEqual(TEXT("submitted scene identity is retained"), First.Identity.SceneRevision,
		Identity().SceneRevision);
	TestEqual(TEXT("submitted frame identity is retained"), First.Identity.FrameId,
		Identity().FrameId);
	TestEqual(TEXT("three ordered camera regions are described"), First.Cameras.Num(), 3);
	if (First.Cameras.Num() == 3)
	{
		TestEqual(TEXT("RGBA offset"), First.Cameras[0].ByteOffset, 0ll);
		TestEqual(TEXT("RGBA bytes"), First.Cameras[0].ByteLength, 8ll);
		TestEqual(TEXT("depth offset"), First.Cameras[1].ByteOffset, 8ll);
		TestEqual(TEXT("depth bytes"), First.Cameras[1].ByteLength, 8ll);
		TestEqual(TEXT("seg offset"), First.Cameras[2].ByteOffset, 16ll);
		TestEqual(TEXT("seg bytes"), First.Cameras[2].ByteLength, 16ll);
		TestEqual(TEXT("capture serial retained"), First.Cameras[2].CaptureSerial, 102ull);
		TestEqual(TEXT("camera configuration identity retained"),
			First.Cameras[2].ConfigurationRevision,
			CapturePlan.Cameras[2].ConfigurationRevision);
	}
	const TArray<uint8> Expected = {
		1,2,3,4,5,6,7,8,
		205,204,204,61,0,0,200,66,
		10,0,0,0,11,0,0,0,244,255,255,255,13,0,0,0};
	TestEqual(TEXT("RGBA8, depth-f32-LE and seg-i32x2-LE are exact"), Bytes, Expected);
	TestEqual(TEXT("depth result metadata retains effective near bound in centimetres"),
		First.Cameras[1].DepthNearCm, CapturePlan.Cameras[1].DepthNearCm);
	TestEqual(TEXT("depth result metadata retains effective far bound in centimetres"),
		First.Cameras[1].DepthFarCm, CapturePlan.Cameras[1].DepthFarCm);
	FURLabCapturePayloadMaterialization Again =
		Admission.Admission->Materialize(Identity(), Complete);
	TestTrue(TEXT("repeat materialization succeeds"), Again.bSucceeded);
	TestEqual(TEXT("repeat returns the same blob"), Again.Descriptor.BlobId,
		First.Descriptor.BlobId);
	TestTrue(TEXT("repeat returns the same immutable payload"), Again.Pin == First.Pin);
	FURLabPayloadPin ExternalPin = First.Pin;
	First.Pin.Reset();
	Again.Pin.Reset();
	Admission.Admission->Abort();
	Admission.ResultLease.Reset();
	Admission.Admission.Reset();
	TestEqual(TEXT("external result pin survives abort and remains charged"),
		Store->GetStats().PhysicalBytes, 64ll);
	ExternalPin.Reset();
	TestEqual(TEXT("dropping external and cached result pins leaves only pixel capacity"),
		Store->GetStats().PhysicalBytes, 32ll);
	Admission.PixelLease.Reset();
	TestEqual(TEXT("dropping the explicitly held pixel lease returns all capacity"),
		Store->GetStats().PhysicalBytes, 0ll);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabCapturePayloadShmBackingTest,
	"URLab.Workflows.RenderCapturePayload.ShmMixedRoundTripNoFallback",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabCapturePayloadShmBackingTest::RunTest(const FString&)
{
	using namespace URLabRenderCapturePayloadAdapterTest;
#if !PLATFORM_LINUX
	AddInfo(TEXT("exact SHM capture backing is Linux-only"));
	return true;
#else
	FPlanAuthority Authority;
	FString AuthorityError;
	if (!TestTrue(TEXT("plan authority creates"), Authority.Create(AuthorityError))) return false;
	auto Store = MakeShared<FURLabPayloadStore, ESPMode::ThreadSafe>(TEXT("payload-shm"), Limits());
	FURLabPayloadOwner Owner;
	if (!Open(*this, *Store, TEXT("payload-shm"), TEXT("shm"), Owner)) return false;
	FURLabShmExactRegionPtr Region;
	if (!TestTrue(TEXT("SHM region creates"),
		FURLabShmExactRegion::CreateForBoot(TEXT("payload-shm"), Region).IsOk())) return false;
	const FURLabRenderExactCapturePlan CapturePlan = Plan(Authority.Producer, {
		{TEXT("rgba"), ERenderCameraMode::Real},
		{TEXT("seg"), ERenderCameraMode::SemanticSegmentation}});
	FURLabCapturePayloadAdmissionResult Admission = FURLabCapturePayloadAdmission::Admit(
		Store, Owner, EURLabPayloadBackingKind::ShmSealedFile, Region, CapturePlan);
	if (!TestTrue(TEXT("explicit SHM capture capacity admits"), Admission.bAccepted)) return false;
	const FURLabCapturePayloadMaterialization Result =
		Admission.Admission->Materialize(Identity(), Snapshot(CapturePlan));
	if (!TestTrue(TEXT("SHM capture materializes"), Result.bSucceeded)) return false;
	TestEqual(TEXT("SHM never falls back to network"), Result.Descriptor.BackingKind,
		EURLabPayloadBackingKind::ShmSealedFile);
	TestFalse(TEXT("sealed SHM descriptor names its object"),
		Result.Descriptor.ObjectName.IsEmpty());
	const TArray<uint8> Expected = {
		1,2,3,4,5,6,7,8,10,0,0,0,11,0,0,0,244,255,255,255,13,0,0,0};
	TestEqual(TEXT("sealed SHM bytes round-trip through its retained pin"),
		Read(Store, Result.Pin), Expected);
	return !HasAnyErrors();
#endif
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabCapturePayloadAtomicValidationTest,
	"URLab.Workflows.RenderCapturePayload.AtomicMismatchAndFailure",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabCapturePayloadAtomicValidationTest::RunTest(const FString&)
{
	using namespace URLabRenderCapturePayloadAdapterTest;
	FPlanAuthority Authority;
	FString AuthorityError;
	if (!TestTrue(TEXT("plan authority creates"), Authority.Create(AuthorityError))) return false;
	auto Store = MakeShared<FURLabPayloadStore, ESPMode::ThreadSafe>(TEXT("payload-atomic"), Limits());
	FURLabPayloadOwner Owner;
	if (!Open(*this, *Store, TEXT("payload-atomic"), TEXT("atomic"), Owner)) return false;
	const FURLabRenderExactCapturePlan CapturePlan = Plan(Authority.Producer, {
		{TEXT("rgba"), ERenderCameraMode::Real}, {TEXT("depth"), ERenderCameraMode::Depth}});
	auto Admit = [&]() { return FURLabCapturePayloadAdmission::Admit(Store, Owner,
		EURLabPayloadBackingKind::NetworkBytes, {}, CapturePlan); };
	FURLabCapturePayloadAdmissionResult Mismatch = Admit();
	FURLabRenderCaptureJobSnapshot Bad = Snapshot(CapturePlan);
	auto WrongIdentity = MakeShared<FRenderCameraFrame>(*Bad.Frames[1]);
	WrongIdentity->Identity.FrameId = 99;
	Bad.Frames[1] = WrongIdentity;
	const FURLabCapturePayloadMaterialization BadResult =
		Mismatch.Admission->Materialize(Identity(), Bad);
	TestFalse(TEXT("one mismatched camera rejects the whole payload"), BadResult.bSucceeded);
	TestTrue(TEXT("mismatch publishes no descriptor"), BadResult.Descriptor.BlobId.IsEmpty());
	TestFalse(TEXT("mismatch publishes no partial pin"), BadResult.Pin.IsValid());
	TestEqual(TEXT("mismatch seals no partial blob"), Store->GetStats().SealedCount, 0);
	Mismatch.Admission->Abort();

	FURLabCapturePayloadAdmissionResult Failed = Admit();
	Bad = Snapshot(CapturePlan);
	Bad.State = EURLabRenderCaptureJobState::Failed;
	Bad.Frames.SetNum(1);
	const FURLabCapturePayloadMaterialization FailedResult =
		Failed.Admission->Materialize(Identity(), Bad);
	TestFalse(TEXT("one-camera capture failure rejects materialization"),
		FailedResult.bSucceeded);
	TestTrue(TEXT("failed capture publishes no descriptor"),
		FailedResult.Descriptor.BlobId.IsEmpty());
	TestEqual(TEXT("failed capture seals no partial blob"), Store->GetStats().SealedCount, 0);
	Failed.Admission->Abort();
	Failed.PixelLease.Reset();
	Failed.ResultLease.Reset();
	Failed.Admission.Reset();

	for (const float InvalidDepth : {std::numeric_limits<float>::quiet_NaN(), -1.0f,
		-std::numeric_limits<float>::infinity()})
	{
		FURLabCapturePayloadAdmissionResult Invalid = Admit();
		if (!TestTrue(TEXT("invalid-depth fixture admits"), Invalid.bAccepted)) return false;
		FURLabRenderCaptureJobSnapshot InvalidSnapshot = Snapshot(CapturePlan);
		auto InvalidFrame = MakeShared<FRenderCameraFrame>(*InvalidSnapshot.Frames[1]);
		InvalidFrame->Depth[0] = InvalidDepth;
		InvalidSnapshot.Frames[1] = InvalidFrame;
		const FURLabCapturePayloadMaterialization InvalidResult =
			Invalid.Admission->Materialize(Identity(), InvalidSnapshot);
		TestFalse(TEXT("invalid native depth fails materialization"), InvalidResult.bSucceeded);
		TestTrue(TEXT("invalid native depth publishes no descriptor"),
			InvalidResult.Descriptor.BlobId.IsEmpty());
		TestFalse(TEXT("invalid native depth publishes no pin"), InvalidResult.Pin.IsValid());
		Invalid.Admission->Abort();
		Invalid.PixelLease.Reset();
		Invalid.ResultLease.Reset();
		Invalid.Admission.Reset();
	}

	FURLabCapturePayloadAdmissionResult Cached = Admit();
	if (!TestTrue(TEXT("cached-success fixture admits"), Cached.bAccepted)) return false;
	FURLabCapturePayloadMaterialization Success =
		Cached.Admission->Materialize(Identity(), Snapshot(CapturePlan));
	if (!TestTrue(TEXT("cached-success fixture seals"), Success.bSucceeded)) return false;
	Success.Pin.Reset();
	int32 Removed = 0;
	TestTrue(TEXT("cached-success session revokes"),
		Store->RevokeSession(Owner, Removed).IsOk());
	const FURLabCapturePayloadMaterialization RevokedCached =
		Cached.Admission->Materialize(Identity(), Snapshot(CapturePlan));
	TestFalse(TEXT("revocation invalidates a cached materialization"),
		RevokedCached.bSucceeded);
	TestTrue(TEXT("revoked cached result exposes no descriptor"),
		RevokedCached.Descriptor.BlobId.IsEmpty());
	TestFalse(TEXT("revoked cached result exposes no pin"), RevokedCached.Pin.IsValid());
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabCapturePayloadCapacityLifetimeTest,
	"URLab.Workflows.RenderCapturePayload.CapacityRollbackAbortAndRevoke",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabCapturePayloadCapacityLifetimeTest::RunTest(const FString&)
{
	using namespace URLabRenderCapturePayloadAdapterTest;
	FPlanAuthority Authority;
	FString AuthorityError;
	if (!TestTrue(TEXT("plan authority creates"), Authority.Create(AuthorityError))) return false;
	const FURLabRenderExactCapturePlan CapturePlan = Plan(Authority.Producer, {
		{TEXT("rgba"), ERenderCameraMode::Real}, {TEXT("depth"), ERenderCameraMode::Depth}});
	const int64 OneCaptureBytes = CapturePlan.AggregatePhysicalArrayBytes
		+ CapturePlan.AggregateLogicalPixelBytes;
	auto Store = MakeShared<FURLabPayloadStore, ESPMode::ThreadSafe>(
		TEXT("payload-life"), Limits(OneCaptureBytes));
	FURLabPayloadOwner Owner;
	FURLabPayloadSessionPtr Session;
	if (!Open(*this, *Store, TEXT("payload-life"), TEXT("life"), Owner, &Session)) return false;
	FURLabCapturePayloadAdmissionResult First = FURLabCapturePayloadAdmission::Admit(Store,
		Owner, EURLabPayloadBackingKind::NetworkBytes, {}, CapturePlan);
	if (!TestTrue(TEXT("first exact capacity admits"), First.bAccepted)) return false;
	FURLabCapturePayloadAdmissionResult Rejected = FURLabCapturePayloadAdmission::Admit(Store,
		Owner, EURLabPayloadBackingKind::NetworkBytes, {}, CapturePlan);
	TestFalse(TEXT("preallocation quota rejects a second capture"), Rejected.bAccepted);
	TestFalse(TEXT("rejection returns no pixel lease"), Rejected.PixelLease.IsValid());
	TestFalse(TEXT("rejection returns no result lease"), Rejected.ResultLease.IsValid());
	First.Admission->Abort();
	First.PixelLease.Reset();
	First.ResultLease.Reset();
	First.Admission.Reset();
	TestEqual(TEXT("abort returns every physical reservation"),
		Store->GetStats().PhysicalBytes, 0ll);

	FURLabCapturePayloadAdmissionResult Revoked = FURLabCapturePayloadAdmission::Admit(Store,
		Owner, EURLabPayloadBackingKind::NetworkBytes, {}, CapturePlan);
	if (!TestTrue(TEXT("capacity can be readmitted after abort"), Revoked.bAccepted)) return false;
	int32 Removed = 0;
	TestTrue(TEXT("session revoke succeeds"), Store->RevokeSession(Owner, Removed).IsOk());
	const FURLabCapturePayloadMaterialization AfterRevoke =
		Revoked.Admission->Materialize(Identity(), Snapshot(CapturePlan));
	TestFalse(TEXT("revoked session cannot publish"), AfterRevoke.bSucceeded);
	TestTrue(TEXT("revoked session publishes no descriptor"),
		AfterRevoke.Descriptor.BlobId.IsEmpty());
	Revoked.Admission->Abort();
	Revoked.PixelLease.Reset();
	Revoked.ResultLease.Reset();
	Revoked.Admission.Reset();
	TestEqual(TEXT("revoke plus abort releases admitted capacity"),
		Store->GetStats().PhysicalBytes, 0ll);

	double Now = 10.0;
	FURLabPayloadStoreLimits ExpiryLimits = Limits(64);
	ExpiryLimits.SealedTtlSeconds = 1.0;
	auto ExpiringStore = MakeShared<FURLabPayloadStore, ESPMode::ThreadSafe>(
		TEXT("payload-expiry"), ExpiryLimits, [&Now]() { return Now; });
	FURLabPayloadOwner ExpiringOwner;
	if (!Open(*this, *ExpiringStore, TEXT("payload-expiry"), TEXT("expiry"),
		ExpiringOwner)) return false;
	FURLabCapturePayloadAdmissionResult Expiring = FURLabCapturePayloadAdmission::Admit(
		ExpiringStore, ExpiringOwner, EURLabPayloadBackingKind::NetworkBytes, {},
		CapturePlan);
	if (!TestTrue(TEXT("expiry fixture admits"), Expiring.bAccepted)) return false;
	FURLabCapturePayloadMaterialization BeforeExpiry =
		Expiring.Admission->Materialize(Identity(), Snapshot(CapturePlan));
	if (!TestTrue(TEXT("expiry fixture initially seals"), BeforeExpiry.bSucceeded)) return false;
	BeforeExpiry.Pin.Reset();
	Now = 10.5;
	FURLabCapturePayloadMaterialization Refreshed =
		Expiring.Admission->Materialize(Identity(), Snapshot(CapturePlan));
	TestTrue(TEXT("live cached descriptor remains materializable"), Refreshed.bSucceeded);
	TestTrue(TEXT("cached descriptor reports its current remaining lifetime"),
		Refreshed.Descriptor.ExpiresInSeconds < BeforeExpiry.Descriptor.ExpiresInSeconds);
	Refreshed.Pin.Reset();
	Now = 12.0;
	const FURLabCapturePayloadMaterialization ExpiredCached =
		Expiring.Admission->Materialize(Identity(), Snapshot(CapturePlan));
	TestFalse(TEXT("descriptor expiry invalidates cached success"),
		ExpiredCached.bSucceeded);
	TestTrue(TEXT("expired cached result exposes no descriptor"),
		ExpiredCached.Descriptor.BlobId.IsEmpty());
	TestFalse(TEXT("expired cached result exposes no pin"), ExpiredCached.Pin.IsValid());
	return !HasAnyErrors();
}
#endif
