// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/AutomationTest.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

#include "Bridge/RenderSourceIngress.h"
#include "Transport/ShmExactPayload.h"
#include "Utils/AssetCache.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

#if WITH_DEV_AUTOMATION_TESTS
namespace URLabRenderSourceIngressTest
{
const char* SimpleXml = R"(<mujoco model="ingress"><worldbody><body><freejoint/><geom type="sphere" size="0.1" mass="1"/></body></worldbody></mujoco>)";
const char* AssetXml = R"(<mujoco model="ingress_asset"><asset><mesh name="tetra" file="tetra.obj"/></asset><worldbody><body><freejoint/><geom type="mesh" mesh="tetra" mass="1"/></body></worldbody></mujoco>)";
const char* TetraObj = R"(v 0 0 0
v 0.1 0 0
v 0 0.1 0
v 0 0 0.1
f 1 3 2
f 1 2 4
f 2 3 4
f 3 1 4
)";

TArray<uint8> Bytes(const char* Text)
{
	TArray<uint8> Out;
	Out.Append(reinterpret_cast<const uint8*>(Text), FCStringAnsi::Strlen(Text));
	return Out;
}

FURLabPayloadStoreLimits Limits(int64 MaxBytes = 64ll * 1024 * 1024)
{
	FURLabPayloadStoreLimits Out;
	Out.MaxBlobBytes = MaxBytes;
	Out.MaxChunkBytes = static_cast<int32>(FMath::Min<int64>(MaxBytes, 64));
	Out.MaxChunksPerBlob = static_cast<int32>(FMath::Max<int64>(1, MaxBytes / Out.MaxChunkBytes + 1));
	Out.MaxSessionBytes = MaxBytes;
	Out.MaxGlobalBytes = MaxBytes;
	Out.MaxBlobCount = 64;
	Out.MaxSessionBlobCount = 64;
	Out.MaxSessionPhysicalObjectCount = 64;
	Out.MaxActiveSessions = 4;
	return Out;
}

bool Open(FAutomationTestBase& Test, FURLabPayloadStore& Store,
	const FString& Boot, const FString& SessionId, FURLabPayloadOwner& Out)
{
	FURLabPayloadSessionPtr Session;
	const FURLabPayloadStoreStatus Status = Store.OpenSession(Boot, SessionId, Session);
	if (!Test.TestTrue(TEXT("payload session opens"), Status.IsOk())) return false;
	Out = FURLabPayloadOwner::FromSession(Session);
	return true;
}

bool Put(FAutomationTestBase& Test, FURLabPayloadStore& Store,
	const FURLabPayloadOwner& Owner, const TArray<uint8>& Source,
	const FURLabShmExactRegionPtr& Region, FURLabRenderSourceBlobRef& Out)
{
	FURLabPayloadProducerPtr Producer;
	FURLabPayloadStoreStatus Status = Region
		? Store.BeginShmProduce(Owner, Region, Source.Num(), Producer)
		: Store.BeginProduce(Owner, Source.Num(), Producer);
	if (!Test.TestTrue(TEXT("source producer admits"), Status.IsOk())) return false;
	TArray<uint8> Copy = Source;
	FURLabStoredPayloadDescriptor Descriptor;
	FURLabPayloadPin Pin;
	Status = Store.SealProduced(Owner, Producer, MoveTemp(Copy), Descriptor, Pin);
	if (!Test.TestTrue(TEXT("source payload seals"), Status.IsOk())) return false;
	Out.BlobId = Descriptor.BlobId;
	Out.Sha256 = Descriptor.Sha256;
	return true;
}

bool MakeMjb(TArray<uint8>& Out, FString& OutError)
{
	FMjOwnedModelCandidate Model;
	if (!FMjOwnedModelCandidate::FromBytes(EMjModelSourceKind::XmlWithAssets,
		Bytes(SimpleXml), {}, Model, OutError, TEXT("ingress MJB seed"))) return false;
	return Model.VisitPrepared([&Out](const mjModel* Prepared, const mjData*)
	{
		const mjtSize Size = mj_sizeModel(Prepared);
		if (Size <= 0 || Size > MAX_int32) return false;
		Out.SetNumUninitialized(static_cast<int32>(Size));
		mj_saveModel(Prepared, nullptr, Out.GetData(), Out.Num());
		return true;
	});
}

bool LoadMjz(TArray<uint8>& Out)
{
	const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("UnrealRoboticsLab"));
	return Plugin && FFileHelper::LoadFileToArray(Out, *FPaths::Combine(
		Plugin->GetBaseDir(), TEXT("third_party/MuJoCo/src/test/testdata/model.mjz")));
}

bool Prepare(FAutomationTestBase& Test,
	const TSharedPtr<FURLabPayloadStore, ESPMode::ThreadSafe>& Store,
	const FURLabPayloadOwner& Owner, const FURLabRenderSourceRequest& Request,
	const FString& Revision, FMjRenderKinematicsCandidate& Out)
{
	FString Error;
	const bool bOk = FURLabRenderSourceIngress::Prepare(Store, Owner, Request,
		Revision, FMjRenderKinematicsOptions(), Out, Error);
	if (!Test.TestTrue(TEXT("source ingress prepares"), bOk)) Test.AddError(Error);
	return bOk;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderSourceIngressPrimaryKindsTest,
	"URLab.Workflows.RenderSourceIngress.AllPrimaryKindsNetwork",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderSourceIngressPrimaryKindsTest::RunTest(const FString&)
{
	using namespace URLabRenderSourceIngressTest;
	auto Store = MakeShared<FURLabPayloadStore, ESPMode::ThreadSafe>(TEXT("ingress-kinds"), Limits());
	FURLabPayloadOwner Owner;
	if (!Open(*this, *Store, TEXT("ingress-kinds"), TEXT("owner"), Owner)) return false;

	FURLabRenderSourceRequest Xml;
	Xml.Kind = EMjModelSourceKind::XmlWithAssets;
	Xml.Label = TEXT("xml asset source");
	if (!Put(*this, *Store, Owner, Bytes(AssetXml), {}, Xml.Primary)) return false;
	FURLabRenderSourceAssetRef Asset;
	Asset.Name = TEXT("tetra.obj");
	if (!Put(*this, *Store, Owner, Bytes(TetraObj), {}, Asset.Payload)) return false;
	Xml.Assets.Add(Asset);
	FMjRenderKinematicsCandidate Candidate;
	if (!Prepare(*this, Store, Owner, Xml, TEXT("xml-revision"), Candidate)) return false;
	TestEqual(TEXT("XML candidate revision"), Candidate.GetSceneRevision(), FString(TEXT("xml-revision")));

	TArray<uint8> MjbBytes;
	FString Error;
	if (!TestTrue(TEXT("MJB fixture creates"), MakeMjb(MjbBytes, Error))) return false;
	FURLabRenderSourceRequest Mjb;
	Mjb.Kind = EMjModelSourceKind::Mjb;
	if (!Put(*this, *Store, Owner, MjbBytes, {}, Mjb.Primary)) return false;
	if (!Prepare(*this, Store, Owner, Mjb, TEXT("mjb-revision"), Candidate)) return false;

	TArray<uint8> MjzBytes;
	if (!TestTrue(TEXT("MJZ fixture reads"), LoadMjz(MjzBytes))) return false;
	FURLabRenderSourceRequest Mjz;
	Mjz.Kind = EMjModelSourceKind::Mjz;
	if (!Put(*this, *Store, Owner, MjzBytes, {}, Mjz.Primary)) return false;
	return Prepare(*this, Store, Owner, Mjz, TEXT("mjz-revision"), Candidate)
		&& !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderSourceIngressShmTest,
	"URLab.Workflows.RenderSourceIngress.XmlAssetsExactShm",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderSourceIngressShmTest::RunTest(const FString&)
{
	using namespace URLabRenderSourceIngressTest;
	auto Store = MakeShared<FURLabPayloadStore, ESPMode::ThreadSafe>(TEXT("ingress-shm"), Limits());
	FURLabPayloadOwner Owner;
	if (!Open(*this, *Store, TEXT("ingress-shm"), TEXT("owner"), Owner)) return false;
	FURLabShmExactRegionPtr Region;
	if (!TestTrue(TEXT("SHM region creates"),
		FURLabShmExactRegion::CreateForBoot(TEXT("ingress-shm"), Region).IsOk())) return false;
	FURLabRenderSourceRequest Request;
	Request.Kind = EMjModelSourceKind::XmlWithAssets;
	if (!Put(*this, *Store, Owner, Bytes(AssetXml), Region, Request.Primary)) return false;
	FURLabRenderSourceAssetRef Asset;
	Asset.Name = TEXT("tetra.obj");
	if (!Put(*this, *Store, Owner, Bytes(TetraObj), Region, Asset.Payload)) return false;
	Request.Assets.Add(Asset);
	FMjRenderKinematicsCandidate Candidate;
	return Prepare(*this, Store, Owner, Request, TEXT("shm-revision"), Candidate)
		&& !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderSourceIngressValidationTest,
	"URLab.Workflows.RenderSourceIngress.IdentityAndAtomicFailure",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderSourceIngressValidationTest::RunTest(const FString&)
{
	using namespace URLabRenderSourceIngressTest;
	auto Store = MakeShared<FURLabPayloadStore, ESPMode::ThreadSafe>(TEXT("ingress-validation"), Limits());
	FURLabPayloadOwner Owner;
	FURLabPayloadOwner Foreign;
	if (!Open(*this, *Store, TEXT("ingress-validation"), TEXT("owner"), Owner)
		|| !Open(*this, *Store, TEXT("ingress-validation"), TEXT("foreign"), Foreign)) return false;
	FURLabRenderSourceRequest Request;
	Request.Kind = EMjModelSourceKind::XmlWithAssets;
	if (!Put(*this, *Store, Owner, Bytes(SimpleXml), {}, Request.Primary)) return false;
	FMjRenderKinematicsCandidate Candidate;
	if (!Prepare(*this, Store, Owner, Request, TEXT("preserved"), Candidate)) return false;

	FString Error;
	FURLabRenderSourceRequest BadHash = Request;
	BadHash.Primary.Sha256[0] = BadHash.Primary.Sha256[0] == TCHAR('0') ? TCHAR('1') : TCHAR('0');
	TestFalse(TEXT("mismatched hash rejected"), FURLabRenderSourceIngress::Prepare(
		Store, Owner, BadHash, TEXT("bad-hash"), {}, Candidate, Error));
	TestEqual(TEXT("hash failure preserves candidate"), Candidate.GetSceneRevision(), FString(TEXT("preserved")));

	FURLabRenderSourceRequest Duplicate = Request;
	Duplicate.Assets = {{TEXT("same"), Request.Primary}, {TEXT("same"), Request.Primary}};
	TestFalse(TEXT("duplicate asset names rejected"), FURLabRenderSourceIngress::Prepare(
		Store, Owner, Duplicate, TEXT("duplicate"), {}, Candidate, Error));
	TestFalse(TEXT("foreign session cannot pin source"), FURLabRenderSourceIngress::Prepare(
		Store, Foreign, Request, TEXT("foreign"), {}, Candidate, Error));
	TestEqual(TEXT("all failures preserve candidate"), Candidate.GetSceneRevision(), FString(TEXT("preserved")));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderSourceIngressTransientCopyTest,
	"URLab.Workflows.RenderSourceIngress.TransientCopyIsNotCharged",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderSourceIngressTransientCopyTest::RunTest(const FString&)
{
	using namespace URLabRenderSourceIngressTest;
	const TArray<uint8> XmlBytes = Bytes(SimpleXml);
	auto Store = MakeShared<FURLabPayloadStore, ESPMode::ThreadSafe>(
		TEXT("ingress-transient"), Limits(XmlBytes.Num()));
	FURLabPayloadOwner Owner;
	if (!Open(*this, *Store, TEXT("ingress-transient"), TEXT("owner"), Owner)) return false;
	FURLabRenderSourceRequest Request;
	Request.Kind = EMjModelSourceKind::XmlWithAssets;
	if (!Put(*this, *Store, Owner, XmlBytes, {}, Request.Primary)) return false;
	const FURLabPayloadStoreStats Before = Store->GetStats();
	FMjRenderKinematicsCandidate Candidate;
	if (!Prepare(*this, Store, Owner, Request, TEXT("transient"), Candidate)) return false;
	const FURLabPayloadStoreStats After = Store->GetStats();
	TestEqual(TEXT("prepare leaves physical bytes unchanged"), After.PhysicalBytes, Before.PhysicalBytes);
	TestEqual(TEXT("prepare leaves physical objects unchanged"), After.PhysicalBlobCount, Before.PhysicalBlobCount);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderSourceIngressAssetLimitTest,
	"URLab.Workflows.RenderSourceIngress.AdvertisedAssetLimit",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderSourceIngressAssetLimitTest::RunTest(const FString&)
{
	using namespace URLabRenderSourceIngressTest;
	const auto Install = [this](int32 AssetCount, FString& OutError)
	{
		const FString Boot = FString::Printf(TEXT("ingress-limit-%d"), AssetCount);
		auto Store = MakeShared<FURLabPayloadStore, ESPMode::ThreadSafe>(Boot, FURLabPayloadStoreLimits());
		FURLabPayloadOwner Owner;
		if (!Open(*this, *Store, Boot, TEXT("owner"), Owner)) return false;
		FString Xml = TEXT("<mujoco model=\"many\"><asset>");
		FString Geoms;
		FURLabRenderSourceRequest Request;
		Request.Kind = EMjModelSourceKind::XmlWithAssets;
		for (int32 Index = 0; Index < AssetCount; ++Index)
		{
			Xml += FString::Printf(TEXT("<mesh name=\"m%d\" file=\"tetra_%d.obj\"/>"), Index, Index);
			Geoms += FString::Printf(TEXT("<geom type=\"mesh\" mesh=\"m%d\" pos=\"%d 0 0\"/>"), Index, Index);
			FURLabRenderSourceAssetRef& Asset = Request.Assets.AddDefaulted_GetRef();
			Asset.Name = FString::Printf(TEXT("tetra_%d.obj"), Index);
			if (!Put(*this, *Store, Owner, Bytes(TetraObj), {}, Asset.Payload)) return false;
		}
		Xml += FString::Printf(TEXT("</asset><worldbody>%s</worldbody></mujoco>"), *Geoms);
		if (!Put(*this, *Store, Owner, Bytes(TCHAR_TO_UTF8(*Xml)), {}, Request.Primary)) return false;
		FMjRenderKinematicsCandidate Candidate;
		const bool bOk = FURLabRenderSourceIngress::Prepare(Store, Owner, Request,
			TEXT("limit"), FMjRenderKinematicsOptions(), Candidate, OutError);
		TestEqual(TEXT("prepare does not retain extra physical objects"),
			Store->GetStats().PhysicalBlobCount, bOk ? AssetCount + 1 : 0);
		return bOk;
	};

	FString Error;
	if (!TestTrue(TEXT("40-asset XML install prepares"), Install(40, Error))) AddError(Error);
	if (!TestTrue(TEXT("advertised maximum asset count prepares"),
		Install(FURLabRenderSourceIngress::MaxAssets, Error))) AddError(Error);
	TestTrue(TEXT("maximum fits the default session object budget"),
		FURLabRenderSourceIngress::MaxAssets + 1 <= FURLabPayloadStoreLimits().MaxSessionPhysicalObjectCount);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabRenderSourceIngressFailedReleaseTest,
	"URLab.Workflows.RenderSourceIngress.FailedPrepareReleasesPayloads",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabRenderSourceIngressFailedReleaseTest::RunTest(const FString&)
{
	using namespace URLabRenderSourceIngressTest;
	auto Store = MakeShared<FURLabPayloadStore, ESPMode::ThreadSafe>(TEXT("ingress-release"), Limits());
	FURLabPayloadOwner Owner;
	FURLabPayloadOwner Foreign;
	if (!Open(*this, *Store, TEXT("ingress-release"), TEXT("owner"), Owner)
		|| !Open(*this, *Store, TEXT("ingress-release"), TEXT("foreign"), Foreign)) return false;
	FURLabRenderSourceRequest Request;
	Request.Kind = EMjModelSourceKind::XmlWithAssets;
	if (!Put(*this, *Store, Owner, Bytes(AssetXml), {}, Request.Primary)) return false;
	FURLabRenderSourceAssetRef Asset;
	Asset.Name = TEXT("wrong_name.obj");
	if (!Put(*this, *Store, Owner, Bytes(TetraObj), {}, Asset.Payload)) return false;
	Request.Assets.Add(Asset);

	FMjRenderKinematicsCandidate Candidate;
	FString Error;
	TestFalse(TEXT("foreign session cannot prepare"), FURLabRenderSourceIngress::Prepare(
		Store, Foreign, Request, TEXT("foreign"), {}, Candidate, Error));
	FURLabPayloadPin Pin;
	TestTrue(TEXT("foreign failure does not release the owner's payloads"),
		Store->Pin(Owner, Request.Primary.BlobId, Pin).IsOk());
	Pin.Reset();

	TestFalse(TEXT("source with an unresolvable asset fails"), FURLabRenderSourceIngress::Prepare(
		Store, Owner, Request, TEXT("invalid"), {}, Candidate, Error));
	TestTrue(TEXT("failure names the missing asset"), Error.Contains(TEXT("tetra.obj")));
	TestFalse(TEXT("failed install releases the primary payload"),
		Store->Pin(Owner, Request.Primary.BlobId, Pin).IsOk());
	TestFalse(TEXT("failed install releases the asset payloads"),
		Store->Pin(Owner, Asset.Payload.BlobId, Pin).IsOk());
	TestEqual(TEXT("failed install leaves no physical objects behind"), Store->GetStats().PhysicalBlobCount, 0);
	TestFalse(TEXT("failed prepare leaves candidate invalid"), Candidate.IsValid());
	return !HasAnyErrors();
}
#endif
