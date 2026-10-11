// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "MjEditorOpHandlers.h"

#include "Async/Async.h"
#include "Async/TaskGraphInterfaces.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/BridgeSubsystem.h"
#include "Bridge/RenderServiceBinding.h"
#include "Bridge/ViewerServiceBinding.h"
#include "Bridge/ViewerProducerSession.h"
#include "Bridge/PoseStreamServiceBinding.h"
#include "Bridge/RpcDispatcher.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HAL/PlatformProcess.h"
#include "Transport/ZmqRpcTransport.h"
#include "Transport/PoseStreamProducer.h"
#include "Utils/AssetCache.h"
#include "Wire/RequestEnvelope.h"
#include "Workflows/RenderServerWorldSubsystem.h"
#include "Workflows/ViewerWorldSubsystem.h"

#if WITH_DEV_AUTOMATION_TESTS
namespace URLabTypedRenderServiceTest
{
using V = FURLabWireValue;

V Map(std::initializer_list<TPair<FString, V>> Fields)
{
	TArray<FString> Keys;
	TArray<V> Values;
	for (const TPair<FString, V>& Field : Fields)
	{
		Keys.Add(Field.Key);
		Values.Add(Field.Value);
	}
	return V::MakeMap(MoveTemp(Keys), MoveTemp(Values));
}

TArray<uint8> Bytes(const char* Text)
{
	TArray<uint8> Out;
	Out.Append(reinterpret_cast<const uint8*>(Text), FCStringAnsi::Strlen(Text));
	return Out;
}

bool Connect(UURLabZmqRpcTransport& Transport, FURLabConnectReply& Out)
{
	FURLabConnectRequest Connect;
	Connect.ProtocolVersion = 2;
	Connect.DataPolicy = TEXT("network");
	TArray<uint8> Request;
	TArray<uint8> Reply;
	return FURLabRequestEnvelopeCodec::EncodeConnectRequest(Connect, Request).IsOk()
		&& Transport.ProcessRequestBytes(Request, Reply)
		&& FURLabRequestEnvelopeCodec::DecodeConnectReply(Reply, Out).IsOk();
}

bool Disconnect(UURLabZmqRpcTransport& Transport, const FURLabConnectReply& Connection)
{
	TArray<uint8> Request;
	TArray<uint8> ReplyBytes;
	FURLabDisconnectReply Reply;
	return FURLabRequestEnvelopeCodec::EncodeDisconnectRequest(
		{2, Connection.BootId, Connection.SessionId}, Request).IsOk()
		&& Transport.ProcessRequestBytes(Request, ReplyBytes)
		&& FURLabRequestEnvelopeCodec::DecodeDisconnectReply(ReplyBytes, Reply).IsOk()
		&& Reply.bDisconnected;
}

bool Encode(const FURLabConnectReply& Connection, uint64 RequestId,
	const FString& Op, V Body, TArray<uint8>& Out)
{
	FURLabRequestEnvelope Request;
	Request.ProtocolVersion = 2;
	Request.BootId = Connection.BootId;
	Request.SessionId = Connection.SessionId;
	Request.RequestId = RequestId;
	Request.Op = Op;
	Request.Body = MoveTemp(Body);
	return FURLabRequestEnvelopeCodec::Encode(Request, Out).IsOk();
}

bool Dispatch(UURLabZmqRpcTransport& Transport, const TArray<uint8>& Request,
	TArray<uint8>& OutBytes, FURLabReplyEnvelope& OutReply)
{
	return Transport.ProcessRequestBytes(Request, OutBytes)
		&& FURLabRequestEnvelopeCodec::DecodeReply(OutBytes, OutReply).IsOk();
}

bool DispatchQueued(UURLabZmqRpcTransport& Transport, const TArray<uint8>& Request,
	TArray<uint8>& OutTerminal, FURLabReplyEnvelope& OutReply)
{
	TFuture<TArray<uint8>> Attempt = Async(EAsyncExecution::ThreadPool,
		[TransportPtr = TWeakObjectPtr<UURLabZmqRpcTransport>(&Transport), Request]()
		{
			TArray<uint8> Reply;
			if (TransportPtr.IsValid()) TransportPtr->ProcessRequestBytes(Request, Reply);
			return Reply;
		});
	const TArray<uint8> PendingBytes = Attempt.Get();
	FURLabReplyEnvelope Pending;
	if (!FURLabRequestEnvelopeCodec::DecodeReply(PendingBytes, Pending).IsOk()
		|| Pending.Kind != EURLabReplyKind::Pending) return false;
	FTaskGraphInterface::Get().ProcessThreadUntilIdle(ENamedThreads::GameThread);
	return Dispatch(Transport, Request, OutTerminal, OutReply)
		&& OutReply.Kind != EURLabReplyKind::Pending;
}

struct FFixture
{
	UWorld* World = nullptr;
	UURLabRenderServerWorldSubsystem* Workflow = nullptr;
	UURLabBridgeServer* Server = nullptr;
	UURLabZmqRpcTransport* Transport = nullptr;
	FURLabRpcDispatcher::FRenderServiceRegistration Registration;

	bool Init(bool bBind, FString& OutError)
	{
		UURLabBridgeSubsystem::SetUnboundForTest(true);
		World = UWorld::CreateWorld(EWorldType::Game, false);
		if (!World || !GEngine) return false;
		FWorldContext& Context = GEngine->CreateNewWorldContext(EWorldType::Game);
		Context.SetCurrentWorld(World);
		Workflow = World->GetSubsystem<UURLabRenderServerWorldSubsystem>();
		if (!Workflow || !Workflow->Activate(OutError)) return false;
		Server = NewObject<UURLabBridgeServer>();
		Server->Start(TEXT(""));
		FMjCapabilitySet Grants;
		Grants.Set(EMjCapability::CameraRpc, true);
		Grants.Set(EMjCapability::AcceptModelLoad, true);
		Server->SetGrants(Grants);
		TSharedPtr<FURLabRenderServiceBinding, ESPMode::ThreadSafe> Binding;
		if (bBind) Binding = FURLabRenderServiceBinding::Create(*Workflow, OutError);
		if (bBind && (!Binding || !Server->GetDispatcher()->AttachRenderService(
			Binding.ToSharedRef(), Registration, OutError)))
		{
			return false;
		}
		Transport = NewObject<UURLabZmqRpcTransport>(Server);
		Transport->SetOwningBridge(Server);
		return true;
	}

	bool Bind(FString& OutError)
	{
		if (!Server || !Workflow) return false;
		const TSharedPtr<FURLabRenderServiceBinding, ESPMode::ThreadSafe> Binding =
			FURLabRenderServiceBinding::Create(*Workflow, OutError);
		return Binding && Server->GetDispatcher()->AttachRenderService(
			Binding.ToSharedRef(), Registration, OutError);
	}

	~FFixture()
	{
		if (Server) Server->Stop();
		if (Workflow) Workflow->ShutdownWorkflow();
		if (World)
		{
			World->DestroyWorld(false);
			if (GEngine) GEngine->DestroyWorldContext(World);
		}
		UURLabBridgeSubsystem::SetUnboundForTest(false);
	}
};

bool Send(UURLabZmqRpcTransport& Transport, const FURLabConnectReply& Connection,
	uint64 RequestId, const FString& Op, V Body, TArray<uint8>& OutBytes,
	FURLabReplyEnvelope& OutReply)
{
	TArray<uint8> Request;
	return Encode(Connection, RequestId, Op, MoveTemp(Body), Request)
		&& Dispatch(Transport, Request, OutBytes, OutReply);
}

bool SendAwait(UURLabZmqRpcTransport& Transport, const FURLabConnectReply& Connection,
	uint64 RequestId, const FString& Op, V Body, TArray<uint8>& OutBytes,
	FURLabReplyEnvelope& OutReply)
{
	TArray<uint8> Request;
	if (!Encode(Connection, RequestId, Op, MoveTemp(Body), Request)) return false;
	for (int32 Attempt = 0; Attempt < 400; ++Attempt)
	{
		if (!Dispatch(Transport, Request, OutBytes, OutReply)) return false;
		if (OutReply.Kind != EURLabReplyKind::Pending) return true;
		FPlatformProcess::SleepNoStats(0.005f);
	}
	return false;
}

bool ReadOwner(const FURLabReplyEnvelope& Reply, uint64& OutGeneration, FString& OutToken)
{
	if (Reply.Kind != EURLabReplyKind::Result) return false;
	const V* Owner = Reply.Result.FindMapValue(TEXT("owner"));
	const V* Generation = Owner ? Owner->FindMapValue(TEXT("generation")) : nullptr;
	const V* Token = Owner ? Owner->FindMapValue(TEXT("token")) : nullptr;
	if (!Generation || !Token || Generation->GetInt() <= 0 || Token->GetString().IsEmpty())
		return false;
	OutGeneration = static_cast<uint64>(Generation->GetInt());
	OutToken = Token->GetString();
	return true;
}

V Credentials(uint64 Generation, const FString& Token,
	std::initializer_list<TPair<FString, V>> Extra = {})
{
	TArray<FString> Keys({TEXT("generation"), TEXT("owner_token")});
	TArray<V> Values({V::MakeInt(static_cast<int64>(Generation)), V::MakeString(Token)});
	for (const TPair<FString, V>& Field : Extra)
	{
		Keys.Add(Field.Key);
		Values.Add(Field.Value);
	}
	return V::MakeMap(MoveTemp(Keys), MoveTemp(Values));
}

bool Upload(UURLabZmqRpcTransport& Transport, const FURLabConnectReply& Connection,
	uint64& NextId, const TArray<uint8>& Data, FString& OutBlob, FString& OutSha,
	FString& OutError)
{
	OutError.Reset();
	OutSha = FURLabAssetCache::Sha256Hex(Data);
	TArray<uint8> ReplyBytes;
	FURLabReplyEnvelope Reply;
	if (!SendAwait(Transport, Connection, NextId++, TEXT("payload_begin"), Map({
		{TEXT("transfer"), V::MakeString(TEXT("network"))},
		{TEXT("byte_length"), V::MakeInt(Data.Num())},
		{TEXT("sha256"), V::MakeString(OutSha)}}), ReplyBytes, Reply)
		|| Reply.Kind != EURLabReplyKind::Result)
	{
		OutError = FString::Printf(TEXT("payload_begin failed: %s %s"),
			*Reply.Error.Code, *Reply.Error.Message);
		return false;
	}
	const V* Blob = Reply.Result.FindMapValue(TEXT("blob_id"));
	if (!Blob) return false;
	OutBlob = Blob->GetString();
	if (!SendAwait(Transport, Connection, NextId++, TEXT("payload_write"), Map({
		{TEXT("blob_id"), V::MakeString(OutBlob)},
		{TEXT("offset"), V::MakeInt(0)},
		{TEXT("data"), V::MakeBinary(Data)}}), ReplyBytes, Reply)
		|| Reply.Kind != EURLabReplyKind::Result)
	{
		OutError = FString::Printf(TEXT("payload_write failed: %s %s"),
			*Reply.Error.Code, *Reply.Error.Message);
		return false;
	}
	const bool bCommitted = SendAwait(Transport, Connection, NextId++,
		TEXT("payload_commit"), Map({
		{TEXT("blob_id"), V::MakeString(OutBlob)}}), ReplyBytes, Reply)
		&& Reply.Kind == EURLabReplyKind::Result;
	if (!bCommitted) OutError = FString::Printf(TEXT("payload_commit failed: %s %s"),
		*Reply.Error.Code, *Reply.Error.Message);
	return bCommitted;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedRenderOwnerServiceTest,
	"URLab.TypedRenderService.BindingOwnerReplayAndRebind",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabTypedRenderOwnerServiceTest::RunTest(const FString&)
{
	using namespace URLabTypedRenderServiceTest;
	FFixture Fixture;
	FString Error;
	if (!TestTrue(TEXT("render workflow activates"), Fixture.Init(false, Error))) return false;
	FURLabConnectReply Passive;
	if (!TestTrue(TEXT("passive bridge connects"), Connect(*Fixture.Transport, Passive))) return false;
	TestFalse(TEXT("passive bootstrap does not advertise render"),
		Passive.Services.Contains(TEXT("render")));
	TArray<uint8> BytesOut;
	FURLabReplyEnvelope Reply;
	TestTrue(TEXT("unbound status request replies"), Send(*Fixture.Transport, Passive, 1,
		TEXT("render_owner_status"), Map({}), BytesOut, Reply));
	TestEqual(TEXT("unbound status reports unavailable"), Reply.Error.Code,
		FString(TEXT("service_unavailable")));
	if (!TestTrue(TEXT("explicit render binding attaches"), Fixture.Bind(Error))) return false;
	FURLabConnectReply A, B;
	if (!Connect(*Fixture.Transport, A) || !Connect(*Fixture.Transport, B)) return false;
	TestTrue(TEXT("bound bootstrap advertises render"), A.Services.Contains(TEXT("render")));
	Fixture.Server->SetGrants(FMjCapabilitySet());
	TestTrue(TEXT("disabled owner permission replies"), Send(*Fixture.Transport, A, 1,
		TEXT("render_owner_attach"), Map({{TEXT("ttl_s"), V::MakeFloat(20.0)}}),
		BytesOut, Reply));
	TestEqual(TEXT("owner mutation honors permission policy"), Reply.Error.Code,
		FString(TEXT("capability_disabled")));
	FMjCapabilitySet Grants;
	Grants.Set(EMjCapability::CameraRpc, true);
	Grants.Set(EMjCapability::AcceptModelLoad, true);
	Fixture.Server->SetGrants(Grants);

	TArray<uint8> AttachRequest;
	if (!Encode(A, 2, TEXT("render_owner_attach"),
		Map({{TEXT("ttl_s"), V::MakeFloat(20.0)}}), AttachRequest)) return false;
	TArray<uint8> AttachedBytes;
	if (!Dispatch(*Fixture.Transport, AttachRequest, AttachedBytes, Reply)) return false;
	uint64 Generation = 0;
	FString Token;
	if (!TestTrue(TEXT("owner attach returns opaque credentials"),
		ReadOwner(Reply, Generation, Token))) return false;
	TArray<uint8> Duplicate;
	TestTrue(TEXT("duplicate attach dispatches"),
		Dispatch(*Fixture.Transport, AttachRequest, Duplicate, Reply));
	TestEqual(TEXT("ledger replays exact attach without duplicate authority"),
		Duplicate, AttachedBytes);
	TestTrue(TEXT("conflicting session receives a reply"), Send(*Fixture.Transport, B, 1,
		TEXT("render_owner_attach"), Map({{TEXT("ttl_s"), V::MakeFloat(20.0)}}),
		BytesOut, Reply));
	TestEqual(TEXT("second live owner conflicts"), Reply.Error.Code,
		FString(TEXT("owner_unavailable")));
	TestTrue(TEXT("exact owner renews"), Send(*Fixture.Transport, A, 3,
		TEXT("render_owner_renew"), Credentials(Generation, Token,
			{{TEXT("ttl_s"), V::MakeFloat(25.0)}}), BytesOut, Reply));
	TestEqual(TEXT("renew returns a result"), Reply.Kind, EURLabReplyKind::Result);
	TestTrue(TEXT("exact owner releases"), Send(*Fixture.Transport, A, 4,
		TEXT("render_owner_release"), Credentials(Generation, Token), BytesOut, Reply));
	TestEqual(TEXT("release returns a result"), Reply.Kind, EURLabReplyKind::Result);
	TestTrue(TEXT("released session can attach a fresh scope"), Send(*Fixture.Transport, A, 5,
		TEXT("render_owner_attach"), Map({{TEXT("ttl_s"), V::MakeFloat(20.0)}}),
		BytesOut, Reply));
	if (!ReadOwner(Reply, Generation, Token)) return false;

	FURLabRpcDispatcher* Dispatcher = Fixture.Server->GetDispatcher();
	TArray<uint8> QueuedRenew;
	if (!Encode(A, 6, TEXT("render_owner_renew"), Credentials(Generation, Token,
		{{TEXT("ttl_s"), V::MakeFloat(28.0)}}), QueuedRenew)) return false;
	TFuture<TArray<uint8>> QueuedAttempt = Async(EAsyncExecution::ThreadPool,
		[Transport = TWeakObjectPtr<UURLabZmqRpcTransport>(Fixture.Transport), QueuedRenew]()
		{
			TArray<uint8> Out;
			if (Transport.IsValid()) Transport->ProcessRequestBytes(QueuedRenew, Out);
			return Out;
		});
	FURLabReplyEnvelope Pending;
	const TArray<uint8> PendingBytes = QueuedAttempt.Get();
	if (!TestTrue(TEXT("worker render mutation returns pending"),
		FURLabRequestEnvelopeCodec::DecodeReply(PendingBytes, Pending).IsOk())
		|| !TestEqual(TEXT("render mutation is queued"), Pending.Kind,
			EURLabReplyKind::Pending)) return false;
	TestTrue(TEXT("exact render registration detaches"),
		Dispatcher->DetachRenderService(Fixture.Registration));
	FTaskGraphInterface::Get().ProcessThreadUntilIdle(ENamedThreads::GameThread);
	if (!TestTrue(TEXT("same world rebinds"), Fixture.Bind(Error))) return false;
	TestTrue(TEXT("queued old-binding request reaches a retained terminal"),
		Dispatch(*Fixture.Transport, QueuedRenew, BytesOut, Reply));
	TestEqual(TEXT("queued old authority cannot run after detach"), Reply.Error.Code,
		FString(TEXT("service_unavailable")));
	FURLabConnectReply Successor;
	if (!Connect(*Fixture.Transport, Successor)) return false;
	TestTrue(TEXT("successor attaches after old binding retirement"),
		Send(*Fixture.Transport, Successor, 1, TEXT("render_owner_attach"),
			Map({{TEXT("ttl_s"), V::MakeFloat(20.0)}}), BytesOut, Reply)
		&& Reply.Kind == EURLabReplyKind::Result);
	uint64 SuccessorGeneration = 0;
	FString SuccessorToken;
	TestTrue(TEXT("successor credentials are fresh"),
		ReadOwner(Reply, SuccessorGeneration, SuccessorToken)
		&& (SuccessorGeneration != Generation || SuccessorToken != Token));
	TestTrue(TEXT("old session request replies after rebind"), Send(*Fixture.Transport, A, 7,
		TEXT("render_owner_renew"), Credentials(Generation, Token,
			{{TEXT("ttl_s"), V::MakeFloat(20.0)}}), BytesOut, Reply));
	TestEqual(TEXT("old scope and token cannot affect successor"), Reply.Error.Code,
		FString(TEXT("owner_invalid")));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedRenderSourceServiceTest,
	"URLab.TypedRenderService.XmlInstallReplayAndReplacementAtomicity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabTypedRenderSourceServiceTest::RunTest(const FString&)
{
	using namespace URLabTypedRenderServiceTest;
	FFixture Fixture;
	FString Error;
	if (!TestTrue(TEXT("bound render fixture activates"), Fixture.Init(true, Error))) return false;
	FURLabConnectReply Session;
	if (!Connect(*Fixture.Transport, Session)) return false;
	TArray<uint8> ReplyBytes;
	FURLabReplyEnvelope Reply;
	if (!Send(*Fixture.Transport, Session, 1, TEXT("render_owner_attach"),
		Map({{TEXT("ttl_s"), V::MakeFloat(30.0)}}), ReplyBytes, Reply)) return false;
	uint64 Generation = 0;
	FString Token;
	if (!ReadOwner(Reply, Generation, Token)) return false;
	const char* Xml = "<mujoco model='typed_render'><worldbody><body name='b'><geom type='sphere' size='0.1' mass='1'/><camera name='c'/></body></worldbody></mujoco>";
	uint64 NextId = 2;
	FString Blob, Sha;
	FString UploadError;
	if (!TestTrue(TEXT("XML uploads through negotiated payload service"),
		Upload(*Fixture.Transport, Session, NextId, Bytes(Xml), Blob, Sha,
			UploadError)))
	{
		AddError(UploadError);
		return false;
	}
	auto InstallBody = [&](const FString& Expected, const FString& SourceBlob,
		const FString& SourceSha)
	{
		return Credentials(Generation, Token, {
			{TEXT("expected_revision"), V::MakeString(Expected)},
			{TEXT("source"), Map({
				{TEXT("format"), V::MakeString(TEXT("xml"))},
				{TEXT("primary"), Map({
					{TEXT("blob_id"), V::MakeString(SourceBlob)},
					{TEXT("sha256"), V::MakeString(SourceSha)}})},
				{TEXT("assets"), V::MakeArray(TArray<V>())},
				{TEXT("label"), V::MakeString(TEXT("typed render fixture"))}})},
			{TEXT("options"), Map({
				{TEXT("visible_group_mask"), V::MakeInt(7)}})}});
	};
	TArray<uint8> InstallRequest;
	if (!Encode(Session, NextId++, TEXT("render_source_install"),
		InstallBody(FString(), Blob, Sha), InstallRequest)) return false;
	TArray<uint8> InstalledBytes;
	if (!TestTrue(TEXT("real XML source install dispatches"), Dispatch(*Fixture.Transport,
		InstallRequest, InstalledBytes, Reply))
		|| !TestEqual(TEXT("real XML source commits"), Reply.Kind, EURLabReplyKind::Result))
		return false;
	const V* RevisionValue = Reply.Result.FindMapValue(TEXT("scene_revision"));
	if (!TestNotNull(TEXT("install returns a revision"), RevisionValue)) return false;
	const FString Revision = RevisionValue->GetString();
	TestFalse(TEXT("installed revision is fresh and nonempty"), Revision.IsEmpty());
	TArray<uint8> ReplayBytes;
	TestTrue(TEXT("duplicate install dispatches"),
		Dispatch(*Fixture.Transport, InstallRequest, ReplayBytes, Reply));
	TestEqual(TEXT("ledger replays exact install without duplicate action"),
		ReplayBytes, InstalledBytes);

	FString BadBlob, BadSha;
	if (!Upload(*Fixture.Transport, Session, NextId, Bytes("not xml"), BadBlob,
		BadSha, UploadError))
	{
		AddError(UploadError);
		return false;
	}
	TestTrue(TEXT("invalid replacement replies"), Send(*Fixture.Transport, Session,
		NextId++, TEXT("render_source_install"), InstallBody(Revision, BadBlob, BadSha),
		ReplyBytes, Reply));
	TestEqual(TEXT("invalid replacement is rejected"), Reply.Error.Code,
		FString(TEXT("source_invalid")));
	TestTrue(TEXT("failed replacement preserves the old revision for a valid replacement"),
		Send(*Fixture.Transport, Session, NextId++, TEXT("render_source_install"),
			InstallBody(Revision, Blob, Sha), ReplyBytes, Reply)
		&& Reply.Kind == EURLabReplyKind::Result);
	if (Reply.Kind == EURLabReplyKind::Result)
	{
		const V* NewRevision = Reply.Result.FindMapValue(TEXT("scene_revision"));
		TestTrue(TEXT("successful replacement returns another fresh revision"),
			NewRevision && NewRevision->GetString() != Revision);
	}
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedNeutralRenderSourceServiceTest,
	"URLab.TypedRenderService.NeutralInstallDescribeReplayAndRollback",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabTypedNeutralRenderSourceServiceTest::RunTest(const FString&)
{
	using namespace URLabTypedRenderServiceTest;
	FFixture Fixture;
	FString Error;
	if (!TestTrue(TEXT("bound neutral render fixture activates"), Fixture.Init(true, Error)))
		return false;
	FURLabConnectReply Session;
	if (!TestTrue(TEXT("neutral fixture connects"), Connect(*Fixture.Transport, Session)))
		return false;
	TArray<uint8> ReplyBytes;
	FURLabReplyEnvelope Reply;
	if (!Send(*Fixture.Transport, Session, 1, TEXT("render_owner_attach"),
		Map({{TEXT("ttl_s"), V::MakeFloat(30.0)}}), ReplyBytes, Reply)) return false;
	uint64 Generation = 0;
	FString Token;
	if (!ReadOwner(Reply, Generation, Token)) return false;

	double InitialPose[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
	TArray<uint8> PoseBytes;
	PoseBytes.Append(reinterpret_cast<const uint8*>(InitialPose), sizeof(InitialPose));
	uint64 NextId = 2;
	FString PoseBlob, PoseSha, UploadError;
	if (!TestTrue(TEXT("neutral initial pose uploads"), Upload(*Fixture.Transport, Session,
		NextId, PoseBytes, PoseBlob, PoseSha, UploadError)))
	{
		AddError(UploadError);
		return false;
	}
	auto Pose = []()
	{
		return Map({
			{TEXT("position_m"), V::MakeArray({V::MakeFloat(0.0), V::MakeFloat(0.0),
				V::MakeFloat(0.0)})},
			{TEXT("rotation_wxyz"), V::MakeArray({V::MakeFloat(1.0), V::MakeFloat(0.0),
				V::MakeFloat(0.0), V::MakeFloat(0.0)})}});
	};
	auto Source = [&](const FString& Sha)
	{
		return Map({
			{TEXT("format"), V::MakeString(TEXT("neutral"))},
			{TEXT("bodies"), V::MakeArray({Map({{TEXT("name"), V::MakeString(TEXT("body"))}})})},
			{TEXT("geoms"), V::MakeArray({Map({
				{TEXT("body_index"), V::MakeInt(0)},
				{TEXT("name"), V::MakeString(TEXT("sphere"))},
				{TEXT("group"), V::MakeInt(0)},
				{TEXT("kind"), V::MakeString(TEXT("sphere"))},
				{TEXT("body_local_pose"), Pose()},
				{TEXT("size"), V::MakeArray({V::MakeFloat(0.1)})},
				{TEXT("surface"), Map({
					{TEXT("rgba"), V::MakeArray({V::MakeFloat(1.0), V::MakeFloat(0.0),
						V::MakeFloat(0.0), V::MakeFloat(1.0)})},
					{TEXT("texuniform_scale"), V::MakeArray({V::MakeFloat(0.0),
						V::MakeFloat(0.0)})}})}})})},
			{TEXT("meshes"), V::MakeArray({})},
			{TEXT("textures"), V::MakeArray({})},
			{TEXT("materials"), V::MakeArray({})},
			{TEXT("lights"), V::MakeArray({})},
			{TEXT("model_cameras"), V::MakeArray({})},
			{TEXT("initial_frame"), Map({
				{TEXT("frame_id"), V::MakeInt(7)},
				{TEXT("clock_epoch"), V::MakeInt(2)},
				{TEXT("has_source_time"), V::MakeBool(false)},
				{TEXT("poses"), Map({
					{TEXT("payload"), Map({{TEXT("blob_id"), V::MakeString(PoseBlob)},
						{TEXT("sha256"), V::MakeString(Sha)}})},
					{TEXT("byte_offset"), V::MakeInt(0)},
					{TEXT("byte_length"), V::MakeInt(PoseBytes.Num())}})}})}});
	};
	auto InstallBody = [&](const FString& Expected, const FString& Sha)
	{
		return Credentials(Generation, Token, {
			{TEXT("expected_revision"), V::MakeString(Expected)},
			{TEXT("source"), Source(Sha)},
			{TEXT("options"), Map({{TEXT("visible_group_mask"), V::MakeInt(1)}})}});
	};

	TArray<uint8> InstallRequest;
	if (!Encode(Session, NextId++, TEXT("render_source_install"),
		InstallBody(FString(), PoseSha), InstallRequest)) return false;
	TArray<uint8> InstalledBytes;
	if (!TestTrue(TEXT("neutral source install dispatches"), Dispatch(*Fixture.Transport,
		InstallRequest, InstalledBytes, Reply))
		|| !TestEqual(TEXT("neutral source commits"), Reply.Kind, EURLabReplyKind::Result))
		return false;
	const V* RevisionValue = Reply.Result.FindMapValue(TEXT("scene_revision"));
	if (!TestNotNull(TEXT("neutral install returns revision"), RevisionValue)) return false;
	const FString Revision = RevisionValue->GetString();
	TestFalse(TEXT("neutral revision is fresh"), Revision.IsEmpty());
	TArray<uint8> ReplayBytes;
	TestTrue(TEXT("neutral replay dispatches"),
		Dispatch(*Fixture.Transport, InstallRequest, ReplayBytes, Reply));
	TestEqual(TEXT("neutral replay is byte exact"), ReplayBytes, InstalledBytes);

	TestTrue(TEXT("neutral describe replies"), Send(*Fixture.Transport, Session, NextId++,
		TEXT("render_scene_describe"), Credentials(Generation, Token, {
			{TEXT("expected_revision"), V::MakeString(Revision)}}), ReplyBytes, Reply));
	if (Reply.Kind == EURLabReplyKind::Result)
	{
		TestFalse(TEXT("neutral source does not advertise qpos"),
			Reply.Result.FindMapValue(TEXT("supports_qpos"))->GetBool());
		TestEqual(TEXT("neutral body identity is described"),
			Reply.Result.FindMapValue(TEXT("body_names"))->GetArray()[0].GetString(),
			FString(TEXT("body")));
	}

	TestTrue(TEXT("forged neutral replacement replies"), Send(*Fixture.Transport, Session,
		NextId++, TEXT("render_source_install"), InstallBody(Revision, FString::ChrN(64, TEXT('0'))),
		ReplyBytes, Reply));
	TestEqual(TEXT("forged neutral payload is rejected"), Reply.Error.Code,
		FString(TEXT("source_invalid")));
	TestTrue(TEXT("failed neutral replacement preserves revision"), Send(*Fixture.Transport,
		Session, NextId++, TEXT("render_scene_describe"), Credentials(Generation, Token, {
			{TEXT("expected_revision"), V::MakeString(Revision)}}), ReplyBytes, Reply)
		&& Reply.Kind == EURLabReplyKind::Result);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedRenderUnboundDisconnectTest,
	"URLab.TypedRenderService.UnboundDisconnectRetiresPayloadSession",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabTypedRenderUnboundDisconnectTest::RunTest(const FString&)
{
	using namespace URLabTypedRenderServiceTest;
	FFixture Fixture;
	FString Error;
	if (!Fixture.Init(false, Error)) return false;
	FURLabConnectReply Session;
	if (!Connect(*Fixture.Transport, Session)) return false;
	uint64 NextId = 1;
	FString Blob, Sha;
	FString UploadError;
	if (!TestTrue(TEXT("unbound session owns a real payload"), Upload(*Fixture.Transport,
		Session, NextId, Bytes("payload"), Blob, Sha, UploadError)))
	{
		AddError(UploadError);
		return false;
	}
	const TSharedRef<FURLabPayloadStore, ESPMode::ThreadSafe> Store =
		Fixture.Server->GetDispatcher()->GetPayloadStoreForTest();
	TestEqual(TEXT("unbound protocol session is live"),
		Store->GetStats().ActiveSessionCount, 1);
	TestEqual(TEXT("unbound session catalog contains its payload"),
		Store->GetStats().SealedCount, 1);
	TestTrue(TEXT("unbound session disconnects"), Disconnect(*Fixture.Transport, Session));
	FTaskGraphInterface::Get().ProcessThreadUntilIdle(ENamedThreads::GameThread);
	TestEqual(TEXT("null render binding does not stall session retirement"),
		Store->GetStats().ActiveSessionCount, 0);
	TestEqual(TEXT("disconnect reclaims the unbound session catalog"),
		Store->GetStats().SealedCount, 0);
	TestEqual(TEXT("disconnect releases all unbound payload backing"),
		Store->GetStats().PhysicalBytes, 0ll);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabViewerProducerCommandUriTest,
	"URLab.TypedViewerService.ProducerCommandUriValidation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FURLabViewerProducerCommandUriTest::RunTest(const FString&)
{
	FString Error;
	TestTrue(TEXT("IPv4/hostname tcp endpoint accepted"),
		UURLabViewerProducerSession::ValidateCommandUri(TEXT("tcp://producer.local:5571"),Error));
	TestTrue(TEXT("bracketed IPv6 tcp endpoint accepted"),
		UURLabViewerProducerSession::ValidateCommandUri(TEXT("tcp://[::1]:5571"),Error));
	for(const FString& Invalid:{FString(TEXT("udp://host:5571")),FString(TEXT("tcp://host")),
		FString(TEXT("tcp://host:0")),FString(TEXT("tcp://user@host:5571")),
		FString(TEXT("tcp://host:5571/path"))})
		TestFalse(*FString::Printf(TEXT("reject %s"),*Invalid),
			UURLabViewerProducerSession::ValidateCommandUri(Invalid,Error));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedViewerWireLifecycleTest,
	"URLab.TypedViewerService.AuthorityInstallBindAndSessionRetirement",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabTypedViewerWireLifecycleTest::RunTest(const FString&)
{
	using namespace URLabTypedRenderServiceTest;
	UURLabBridgeSubsystem::SetUnboundForTest(true);
	UWorld* World=UWorld::CreateWorld(EWorldType::Game,false);
	if(!TestNotNull(TEXT("viewer test world"),World)||!GEngine)return false;
	FWorldContext& Context=GEngine->CreateNewWorldContext(EWorldType::Game); Context.SetCurrentWorld(World);
	auto* Viewer=World->GetSubsystem<UURLabViewerWorldSubsystem>(); FString Error;
	if(!TestTrue(TEXT("viewer activates"),Viewer&&Viewer->Activate(Error))){AddError(Error);return false;}
	auto* Server=NewObject<UURLabBridgeServer>(); Server->Start(TEXT(""));
	FMjCapabilitySet Grants; Grants.Set(EMjCapability::AcceptInput,true); Grants.Set(EMjCapability::AcceptModelLoad,true); Server->SetGrants(Grants);
	auto Binding=FURLabViewerServiceBinding::Create(*Viewer,Error); FURLabRpcDispatcher::FViewerServiceRegistration Registration;
	if(!TestTrue(TEXT("viewer binding attaches"),Binding&&Server->GetDispatcher()->AttachViewerService(Binding.ToSharedRef(),Registration,Error))){AddError(Error);return false;}
	auto* Transport=NewObject<UURLabZmqRpcTransport>(Server); Transport->SetOwningBridge(Server); FURLabConnectReply Session;
	if(!TestTrue(TEXT("viewer client connects"),Connect(*Transport,Session)))return false;
	TestTrue(TEXT("viewer service advertised only while explicitly attached"),Session.Services.Contains(TEXT("viewer")));
	TArray<uint8> ReplyBytes; FURLabReplyEnvelope Reply;
	if(!TestTrue(TEXT("viewer owner attaches"),Send(*Transport,Session,1,TEXT("viewer_owner_attach"),Map({{TEXT("ttl_s"),V::MakeFloat(30.0)}}),ReplyBytes,Reply)))return false;
	uint64 Generation=0; FString Token; if(!TestTrue(TEXT("viewer proof returned"),ReadOwner(Reply,Generation,Token)))return false;
	auto Descriptor=[](){return Map({{TEXT("kind"),V::MakeString(TEXT("cartesian_pose"))},{TEXT("schema_version"),V::MakeInt(2)},{TEXT("producer_boot_id"),V::MakeString(TEXT("producer"))},{TEXT("stream_id"),V::MakeString(TEXT("poses"))},{TEXT("stream_generation"),V::MakeInt(1)},{TEXT("source_revision"),V::MakeString(TEXT("source-r1"))},{TEXT("payload_format"),V::MakeString(TEXT("dense_rh_m_wxyz_f64le_dynamic_mesh_v1"))}});};
	auto BindBody=[&](const FString& Revision,const FString& TokenValue){return Map({{TEXT("generation"),V::MakeInt(Generation)},{TEXT("owner_token"),V::MakeString(TokenValue)},{TEXT("expected_revision"),V::MakeString(Revision)},{TEXT("stream"),Descriptor()}});};
	auto EmptySource=Map({{TEXT("format"),V::MakeString(TEXT("neutral"))},{TEXT("bodies"),V::MakeArray({})},{TEXT("geoms"),V::MakeArray({})},{TEXT("meshes"),V::MakeArray({})},{TEXT("textures"),V::MakeArray({})},{TEXT("materials"),V::MakeArray({})},{TEXT("model_cameras"),V::MakeArray({})},{TEXT("initial_frame"),Map({{TEXT("frame_id"),V::MakeInt(0)},{TEXT("clock_epoch"),V::MakeInt(0)},{TEXT("has_source_time"),V::MakeBool(false)}})}});
	auto Install=Credentials(Generation,Token,{{TEXT("expected_revision"),V::MakeString(TEXT(""))},{TEXT("source"),EmptySource},{TEXT("options"),Map({{TEXT("visible_group_mask"),V::MakeInt(1)}})}});
	TArray<uint8> InstallRequest; Encode(Session,3,TEXT("viewer_source_install"),Install,InstallRequest); TArray<uint8> InstalledBytes;
	TestTrue(TEXT("viewer neutral install dispatches"),Dispatch(*Transport,InstallRequest,InstalledBytes,Reply));
	if(!TestEqual(TEXT("viewer source install succeeds"),Reply.Kind,EURLabReplyKind::Result))return false;
	const FString Revision=Reply.Result.FindMapValue(TEXT("scene_revision"))->GetString();
	TArray<uint8> ReplayBytes; TestTrue(TEXT("viewer install replay dispatches"),Dispatch(*Transport,InstallRequest,ReplayBytes,Reply)); TestEqual(TEXT("viewer source acknowledgement replay is exact"),ReplayBytes,InstalledBytes);
	TestTrue(TEXT("session disconnects"),Disconnect(*Transport,Session)); FTaskGraphInterface::Get().ProcessThreadUntilIdle(ENamedThreads::GameThread);
	const auto Status=Viewer->Status(); TestFalse(TEXT("session retirement releases exact owner"),Status.Owner.bActive); TestFalse(TEXT("session retirement clears binding"),Status.bStreamBound); TestTrue(TEXT("installed display remains stale"),Status.bDisplayStale);
	Server->Stop(); Viewer->ShutdownWorkflow(); World->DestroyWorld(false); GEngine->DestroyWorldContext(World); UURLabBridgeSubsystem::SetUnboundForTest(false);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedViewerLiveStreamTest,
	"URLab.TypedViewerService.LiveProducerNetworkAndShm",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabTypedViewerLiveStreamTest::RunTest(const FString&)
{
	using namespace URLabTypedRenderServiceTest;
	UURLabBridgeSubsystem::SetUnboundForTest(true);
	UWorld* World=UWorld::CreateWorld(EWorldType::Game,false);
	if(!TestNotNull(TEXT("viewer live world"),World)||!GEngine)return false;
	FWorldContext& Context=GEngine->CreateNewWorldContext(EWorldType::Game);Context.SetCurrentWorld(World);
	auto* Viewer=World->GetSubsystem<UURLabViewerWorldSubsystem>();FString Error;
	if(!TestTrue(TEXT("viewer activates"),Viewer&&Viewer->Activate(Error))){AddError(Error);return false;}

	auto* ProducerServer=NewObject<UURLabBridgeServer>();ProducerServer->Start(TEXT("tcp://127.0.0.1:45991"));
	TArray<FString> CommandEndpoints;ProducerServer->CollectCommandEndpoints(CommandEndpoints);
	if(!TestEqual(TEXT("producer exposes one command endpoint"),CommandEndpoints.Num(),1))return false;
	FURLabPoseStreamProducerConfig ProducerConfig;ProducerConfig.BindEndpoint=TEXT("tcp://127.0.0.1:45992");
	ProducerConfig.AdvertisedEndpoint=ProducerConfig.BindEndpoint;ProducerConfig.MaxPayloadBytes=4096;
	ProducerConfig.ShmRegion=ProducerServer->GetDispatcher()->GetShmRegionForLocalProducer();
	TUniquePtr<FURLabPoseStreamProducer> Created=FURLabPoseStreamProducer::Create(ProducerConfig,Error);
	if(!TestTrue(TEXT("pose producer creates"),Created.IsValid())){AddError(Error);return false;}
	TSharedPtr<FURLabPoseStreamProducer,ESPMode::ThreadSafe> Producer=MakeShareable(Created.Release());auto StreamBinding=MakeShared<FURLabPoseStreamServiceBinding,ESPMode::ThreadSafe>(Producer.ToSharedRef());
	FURLabRpcDispatcher::FPoseStreamServiceRegistration StreamRegistration;
	if(!TestTrue(TEXT("streams service attaches"),ProducerServer->GetDispatcher()->AttachPoseStreamService(StreamBinding,StreamRegistration,Error))){AddError(Error);return false;}
	FURLabPoseStreamDescriptor ProducerDescriptor;if(!TestTrue(TEXT("producer source commits"),Producer->CommitSource(TEXT("producer-source"),ProducerDescriptor,Error))){AddError(Error);return false;}

	auto* ViewerServer=NewObject<UURLabBridgeServer>();ViewerServer->Start(TEXT(""));FMjCapabilitySet Grants;Grants.Set(EMjCapability::AcceptInput,true);Grants.Set(EMjCapability::AcceptModelLoad,true);ViewerServer->SetGrants(Grants);
	auto ViewerBinding=FURLabViewerServiceBinding::Create(*Viewer,Error);FURLabRpcDispatcher::FViewerServiceRegistration ViewerRegistration;
	if(!TestTrue(TEXT("viewer service attaches"),ViewerBinding&&ViewerServer->GetDispatcher()->AttachViewerService(ViewerBinding.ToSharedRef(),ViewerRegistration,Error))){AddError(Error);return false;}
	auto* Transport=NewObject<UURLabZmqRpcTransport>(ViewerServer);Transport->SetOwningBridge(ViewerServer);

	auto DescriptorValue=[&](){return Map({{TEXT("kind"),V::MakeString(ProducerDescriptor.Kind)},{TEXT("schema_version"),V::MakeInt(ProducerDescriptor.SchemaVersion)},{TEXT("producer_boot_id"),V::MakeString(ProducerDescriptor.ProducerBootId)},{TEXT("stream_id"),V::MakeString(ProducerDescriptor.StreamId)},{TEXT("stream_generation"),V::MakeInt(ProducerDescriptor.StreamGeneration)},{TEXT("source_revision"),V::MakeString(ProducerDescriptor.SourceRevision)},{TEXT("payload_format"),V::MakeString(ProducerDescriptor.PayloadFormat)}});};
		auto OneBodySource=[](const FString& Blob,const FString& Sha,int64 Length){return Map({{TEXT("format"),V::MakeString(TEXT("neutral"))},{TEXT("bodies"),V::MakeArray({Map({{TEXT("name"),V::MakeString(TEXT("body"))}})})},{TEXT("geoms"),V::MakeArray({})},{TEXT("meshes"),V::MakeArray({})},{TEXT("textures"),V::MakeArray({})},{TEXT("materials"),V::MakeArray({})},{TEXT("model_cameras"),V::MakeArray({})},{TEXT("lights"),V::MakeArray({})},{TEXT("initial_frame"),Map({{TEXT("frame_id"),V::MakeInt(0)},{TEXT("clock_epoch"),V::MakeInt(0)},{TEXT("has_source_time"),V::MakeBool(false)},{TEXT("poses"),Map({{TEXT("payload"),Map({{TEXT("blob_id"),V::MakeString(Blob)},{TEXT("sha256"),V::MakeString(Sha)}})},{TEXT("byte_offset"),V::MakeInt(0)},{TEXT("byte_length"),V::MakeInt(Length)}})}})}});};
	auto RunPolicy=[&](const FString& Policy,uint64 Base,bool bTestDeath)->bool
	{
		auto Fail=[&](const FString& Step,const FURLabReplyEnvelope* Reply=nullptr){FString Detail=Step;if(Reply&&Reply->Kind==EURLabReplyKind::Error)Detail+=TEXT(": ")+Reply->Error.Code+TEXT(" ")+Reply->Error.Message;AddError(Policy+TEXT(" ")+Detail);return false;};
		FURLabConnectReply Session;if(!Connect(*Transport,Session))return Fail(TEXT("viewer connect"));TArray<uint8> BytesOut;FURLabReplyEnvelope Reply;
		if(!Send(*Transport,Session,Base,TEXT("viewer_owner_attach"),Map({{TEXT("ttl_s"),V::MakeFloat(30.0)}}),BytesOut,Reply)||Reply.Kind!=EURLabReplyKind::Result)return Fail(TEXT("owner attach"),&Reply);
		uint64 Generation=0;FString Token;if(!ReadOwner(Reply,Generation,Token))return Fail(TEXT("owner proof"),&Reply);
		double InitialPose[7]={0,0,0,1,0,0,0};TArray<uint8> PoseBytes;PoseBytes.Append(reinterpret_cast<const uint8*>(InitialPose),sizeof(InitialPose));uint64 NextId=Base+1;FString PoseBlob,PoseSha,UploadError;if(!Upload(*Transport,Session,NextId,PoseBytes,PoseBlob,PoseSha,UploadError)){AddError(Policy+TEXT(" pose upload: ")+UploadError);return false;}
		const FString PriorRevision=Viewer->Status().SceneRevision;auto Install=Credentials(Generation,Token,{{TEXT("expected_revision"),V::MakeString(PriorRevision)},{TEXT("source"),OneBodySource(PoseBlob,PoseSha,PoseBytes.Num())},{TEXT("options"),Map({{TEXT("visible_group_mask"),V::MakeInt(1)}})}});
		if(!Send(*Transport,Session,NextId++,TEXT("viewer_source_install"),MoveTemp(Install),BytesOut,Reply)||Reply.Kind!=EURLabReplyKind::Result)return Fail(TEXT("source install"),&Reply);
		const FString Revision=Reply.Result.FindMapValue(TEXT("scene_revision"))->GetString();
		auto Bind=Credentials(Generation,Token,{{TEXT("expected_revision"),V::MakeString(Revision)},{TEXT("stream"),DescriptorValue()},{TEXT("producer_connection"),Map({{TEXT("command_uri"),V::MakeString(CommandEndpoints[0])},{TEXT("data"),V::MakeString(Policy)}})}});
		TArray<uint8> Request;if(!Encode(Session,NextId++,TEXT("viewer_stream_bind"),MoveTemp(Bind),Request))return Fail(TEXT("bind encode"));
		if(!Dispatch(*Transport,Request,BytesOut,Reply))return Fail(TEXT("bind dispatch"));
		for(int32 Attempt=0;Attempt<500&&Reply.Kind==EURLabReplyKind::Pending;++Attempt){FPlatformProcess::SleepNoStats(0.002f);FTaskGraphInterface::Get().ProcessThreadUntilIdle(ENamedThreads::GameThread);if(!Dispatch(*Transport,Request,BytesOut,Reply))return Fail(TEXT("bind poll"));}
		if(Reply.Kind!=EURLabReplyKind::Result)return Fail(TEXT("bind terminal"),&Reply);
		const TArray<double> Pose={0,0,0,1,0,0,0};if(!Producer->Publish(Base,0.0,false,1,Pose,Error)){AddError(Policy+TEXT(" publish: ")+Error);return false;}
		for(int32 Attempt=0;Attempt<500&&Viewer->Status().bDisplayStale;++Attempt){FPlatformProcess::SleepNoStats(0.002f);Viewer->Tick(0.0f);}
		if(Viewer->Status().bDisplayStale)return Fail(TEXT("frame apply timeout"));
		if(bTestDeath)
		{
			const double QuietUntil=FPlatformTime::Seconds()+2.2;
			while(FPlatformTime::Seconds()<QuietUntil){FPlatformProcess::SleepNoStats(0.01f);Viewer->Tick(0.0f);}
			if(!Viewer->Status().bStreamBound)return Fail(TEXT("paused producer was retired"));
			ProducerServer->Stop();
			const double DeathDeadline=FPlatformTime::Seconds()+5.0;
			while(FPlatformTime::Seconds()<DeathDeadline&&Viewer->Status().bStreamBound){FPlatformProcess::SleepNoStats(0.01f);Viewer->Tick(0.0f);}
			if(Viewer->Status().bStreamBound||!Viewer->Status().bDisplayStale)return Fail(TEXT("dead producer was not retired stale"));
			return true;
		}
		if(!Disconnect(*Transport,Session))return Fail(TEXT("viewer disconnect"));FTaskGraphInterface::Get().ProcessThreadUntilIdle(ENamedThreads::GameThread);
		if(Viewer->Status().bStreamBound)return Fail(TEXT("binding retained after disconnect"));
		return true;
	};
	const bool bShm=RunPolicy(TEXT("shm"),100,false);TestTrue(TEXT("SHM producer independently proves, binds, applies, and retires"),bShm);
	const bool bNetwork=bShm&&RunPolicy(TEXT("network"),200,true);TestTrue(TEXT("network paused producer survives and command death retires"),bNetwork);
	ViewerServer->Stop();StreamBinding->Deactivate();Producer->Stop();ProducerServer->Stop();Viewer->ShutdownWorkflow();World->DestroyWorld(false);GEngine->DestroyWorldContext(World);UURLabBridgeSubsystem::SetUnboundForTest(false);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedEditorServiceRegistration,
	"URLab.TypedEditorService.RegistrationGrantAndRead",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabTypedEditorServiceRegistration::RunTest(const FString&)
{
	using namespace URLabTypedRenderServiceTest;
	auto* Server=NewObject<UURLabBridgeServer>();Server->Start(TEXT(""));
	FMjCapabilitySet Grants;Server->SetGrants(Grants);
	auto* Transport=NewObject<UURLabZmqRpcTransport>(Server);Transport->SetOwningBridge(Server);
	FURLabConnectReply Session;if(!TestTrue(TEXT("editor session connects"),Connect(*Transport,Session))){Server->Stop();return false;}
	TestTrue(TEXT("editor service advertised only with URLabEditor registration"),Session.Services.Contains(TEXT("editor")));
	TArray<uint8> BytesOut;FURLabReplyEnvelope Reply;
	TestTrue(TEXT("read-only editor status dispatches"),SendAwait(*Transport,Session,1,TEXT("editor_current_level"),Map({}),BytesOut,Reply));
	TestEqual(TEXT("read-only editor status succeeds"),Reply.Kind,EURLabReplyKind::Result);
	TestTrue(TEXT("editor mutation dispatches denial"),Send(*Transport,Session,2,TEXT("editor_create_level"),Map({{TEXT("name"),V::MakeString(TEXT("Denied"))},{TEXT("force_overwrite"),V::MakeBool(false)}}),BytesOut,Reply));
	TestEqual(TEXT("editor mutation needs separate grant"),Reply.Kind,EURLabReplyKind::Error);
	if(Reply.Kind==EURLabReplyKind::Error)TestEqual(TEXT("permission error"),Reply.Error.Code,FString(TEXT("capability_disabled")));

	Grants.Set(EMjCapability::AcceptEditorCommands,true);Server->SetGrants(Grants);
	const FString ActorId=FString::Printf(TEXT("typed_editor_test_%llu"),static_cast<unsigned long long>(FPlatformTime::Cycles64()));
	auto Vec3=[](double X,double Y,double Z){return V::MakeArray({V::MakeFloat(X),V::MakeFloat(Y),V::MakeFloat(Z)});};
	TestTrue(TEXT("granted spawn reaches terminal result"),SendAwait(*Transport,Session,3,TEXT("editor_spawn_light"),Map({
		{TEXT("kind"),V::MakeString(TEXT("point"))},{TEXT("actor_id"),V::MakeString(ActorId)},
		{TEXT("location"),Vec3(0,0,0)},{TEXT("rotation_euler"),Vec3(0,0,0)},
		{TEXT("scale"),Vec3(1,1,1)},{TEXT("intensity"),V::MakeFloat(10)},
		{TEXT("color"),Vec3(1,1,1)}}),BytesOut,Reply));
	TestEqual(TEXT("granted spawn succeeds"),Reply.Kind,EURLabReplyKind::Result);
	TestTrue(TEXT("spawn result uses exact actor name"),Reply.Kind==EURLabReplyKind::Result
		&& Reply.Result.FindMapValue(TEXT("actor_name"))!=nullptr);
	TestTrue(TEXT("same-ticket replay returns retained spawn result"),SendAwait(*Transport,Session,3,TEXT("editor_spawn_light"),Map({
		{TEXT("kind"),V::MakeString(TEXT("point"))},{TEXT("actor_id"),V::MakeString(ActorId)},
		{TEXT("location"),Vec3(0,0,0)},{TEXT("rotation_euler"),Vec3(0,0,0)},
		{TEXT("scale"),Vec3(1,1,1)},{TEXT("intensity"),V::MakeFloat(10)},
		{TEXT("color"),Vec3(1,1,1)}}),BytesOut,Reply));
	TestEqual(TEXT("same-ticket replay remains successful"),Reply.Kind,EURLabReplyKind::Result);
	TestTrue(TEXT("spawned actor is inspectable"),SendAwait(*Transport,Session,4,TEXT("editor_get_actor_bounds"),Map({
		{TEXT("target"),V::MakeString(ActorId)},{TEXT("components_only"),V::MakeBool(false)}}),BytesOut,Reply));
	TestEqual(TEXT("inspect succeeds"),Reply.Kind,EURLabReplyKind::Result);
	TestTrue(TEXT("spawned actor is editable"),SendAwait(*Transport,Session,5,TEXT("editor_set_actor_transform"),Map({
		{TEXT("target"),V::MakeString(ActorId)},{TEXT("location"),Vec3(0.1,0.2,0.3)}}),BytesOut,Reply));
	TestEqual(TEXT("edit succeeds"),Reply.Kind,EURLabReplyKind::Result);
	TestTrue(TEXT("temporary actor is deleted"),SendAwait(*Transport,Session,6,TEXT("editor_remove_actor"),Map({
		{TEXT("target"),V::MakeString(ActorId)}}),BytesOut,Reply));
	TestEqual(TEXT("delete succeeds"),Reply.Kind,EURLabReplyKind::Result);
	Disconnect(*Transport,Session);Server->Stop();

	URLabEditorOpHandlers::UnregisterAll();
	ON_SCOPE_EXIT{URLabEditorOpHandlers::RegisterAll();};
	auto* RuntimeServer=NewObject<UURLabBridgeServer>();RuntimeServer->Start(TEXT(""));
	auto* RuntimeTransport=NewObject<UURLabZmqRpcTransport>(RuntimeServer);RuntimeTransport->SetOwningBridge(RuntimeServer);
	FURLabConnectReply RuntimeSession;
	if(TestTrue(TEXT("runtime-shaped session connects"),Connect(*RuntimeTransport,RuntimeSession)))
	{
		TestFalse(TEXT("dispatcher without URLabEditor handlers does not advertise editor"),RuntimeSession.Services.Contains(TEXT("editor")));
		Disconnect(*RuntimeTransport,RuntimeSession);
	}
	RuntimeServer->Stop();return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedEditorPieLifecycle,
	"URLab.TypedEditorService.PieLifecycle",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

class FURLabTypedEditorPieLifecycleCommand final : public IAutomationLatentCommand
{
public:
	explicit FURLabTypedEditorPieLifecycleCommand(FAutomationTestBase* InTest):Test(InTest){}
	virtual bool Update() override
	{
		using namespace URLabTypedRenderServiceTest;
		if(Phase==0)
		{
			Server=NewObject<UURLabBridgeServer>();Server->AddToRoot();Server->Start(TEXT(""));
			FMjCapabilitySet Grants;Grants.Set(EMjCapability::AcceptEditorCommands,true);Server->SetGrants(Grants);
			Transport=NewObject<UURLabZmqRpcTransport>(Server);Transport->AddToRoot();Transport->SetOwningBridge(Server);
			const bool bConnected=Connect(*Transport,Session);
			Test->TestTrue(TEXT("PIE editor session connects"),bConnected);
			if(!bConnected||!Encode(Session,1,TEXT("editor_begin_pie"),Map({{TEXT("timeout_s"),V::MakeFloat(5.0)}}),Request)
				||!Dispatch(*Transport,Request,ReplyBytes,Reply))return Finish(false);
			Test->TestEqual(TEXT("PIE begin is retained asynchronous work"),Reply.Kind,EURLabReplyKind::Pending);
			Deadline=FPlatformTime::Seconds()+10.0;Phase=1;return false;
		}
		if(Phase==1)
		{
			if(!Dispatch(*Transport,Request,ReplyBytes,Reply))return Finish(false);
			if(Reply.Kind==EURLabReplyKind::Pending&&FPlatformTime::Seconds()<Deadline)return false;
			Test->TestEqual(TEXT("PIE begin reaches a result"),Reply.Kind,EURLabReplyKind::Result);
			if(Reply.Kind==EURLabReplyKind::Result)
			{
				const V* State=Reply.Result.FindMapValue(TEXT("state"));
				const FString LifecycleState=State?State->GetString():FString();
				Test->TestEqual(TEXT("PIE lifecycle reports running independently of model readiness"),LifecycleState,FString(TEXT("running")));
				Test->TestNull(TEXT("PIE result has no embedded legacy handshake"),Reply.Result.FindMapValue(TEXT("handshake_payload")));
			}
			if(!Encode(Session,2,TEXT("editor_pie_status"),Map({}),Request)
				||!Dispatch(*Transport,Request,ReplyBytes,Reply))return Finish(false);
			Test->TestEqual(TEXT("PIE status succeeds without authority side effects"),Reply.Kind,EURLabReplyKind::Result);
			if(Reply.Kind==EURLabReplyKind::Result)
			{
				const V* StatusState=Reply.Result.FindMapValue(TEXT("state"));
				Test->TestTrue(TEXT("PIE status reports running without a manager"),StatusState&&StatusState->GetString()==TEXT("running"));
			}
			if(!Encode(Session,3,TEXT("editor_stop_pie"),Map({}),Request)
				||!Dispatch(*Transport,Request,ReplyBytes,Reply))return Finish(false);
			Test->TestEqual(TEXT("PIE stop is retained asynchronous work"),Reply.Kind,EURLabReplyKind::Pending);
			Deadline=FPlatformTime::Seconds()+15.0;Phase=2;return false;
		}
		if(!Dispatch(*Transport,Request,ReplyBytes,Reply))return Finish(false);
		if(Reply.Kind==EURLabReplyKind::Pending&&FPlatformTime::Seconds()<Deadline)return false;
		Test->TestEqual(TEXT("PIE stop succeeds"),Reply.Kind,EURLabReplyKind::Result);
		return Finish(true);
	}
private:
	bool Finish(bool bNormal)
	{
		if(Transport&&!Session.SessionId.IsEmpty())URLabTypedRenderServiceTest::Disconnect(*Transport,Session);
		if(Server)Server->Stop();
		if(Transport){Transport->RemoveFromRoot();Transport=nullptr;}
		if(Server){Server->RemoveFromRoot();Server=nullptr;}
		if(!bNormal)Test->AddError(TEXT("PIE lifecycle fixture failed before terminal cleanup"));
		return true;
	}
	FAutomationTestBase* Test=nullptr;UURLabBridgeServer* Server=nullptr;UURLabZmqRpcTransport* Transport=nullptr;
	FURLabConnectReply Session;TArray<uint8> Request,ReplyBytes;FURLabReplyEnvelope Reply;int32 Phase=0;double Deadline=0;
};

bool FURLabTypedEditorPieLifecycle::RunTest(const FString&)
{
	ADD_LATENT_AUTOMATION_COMMAND(FURLabTypedEditorPieLifecycleCommand(this));
	return true;
}
#endif
