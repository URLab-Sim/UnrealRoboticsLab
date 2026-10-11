// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "Misc/AutomationTest.h"

#include "Async/Async.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/RpcDispatcher.h"
#include "HAL/PlatformProcess.h"
#include "HAL/Event.h"
#include "HAL/PlatformFileManager.h"
#include "Misc/FileHelper.h"
#include "MuJoCo/Runtime/MjSimulationRuntime.h"
#include "Utils/AssetCache.h"
#include "Wire/OperationSchema.h"

#include <limits>

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
using FTypedDataValue = FURLabWireValue;

FTypedDataValue TypedDataMap(
	std::initializer_list<TPair<FString, FTypedDataValue>> Fields)
{
	TArray<FString> Keys;
	TArray<FTypedDataValue> Values;
	for (const TPair<FString, FTypedDataValue>& Field : Fields)
	{
		Keys.Add(Field.Key);
		Values.Add(Field.Value);
	}
	return FTypedDataValue::MakeMap(MoveTemp(Keys), MoveTemp(Values));
}

TArray<uint8> TypedDataBytes(std::initializer_list<uint8> Values)
{
	TArray<uint8> Out;
	Out.Append(Values.begin(), static_cast<int32>(Values.size()));
	return Out;
}

TArray<uint8> TypedDataXmlBytes(const char* Text)
{
	TArray<uint8> Out;
	Out.Append(reinterpret_cast<const uint8*>(Text), FCStringAnsi::Strlen(Text));
	return Out;
}

bool TypedDataConnectPolicy(FURLabRpcDispatcher& Dispatcher, const FString& Adapter,
	const FString& Policy, FURLabConnectReply& Out)
{
	TArray<uint8> Request;
	TArray<uint8> Reply;
	FURLabConnectRequest Connect;
	Connect.ProtocolVersion = 2;
	Connect.DataPolicy = Policy;
	return FURLabRequestEnvelopeCodec::EncodeConnectRequest(Connect, Request).IsOk()
		&& Dispatcher.DispatchBytes(Request, Adapter, Reply)
		&& FURLabRequestEnvelopeCodec::DecodeConnectReply(Reply, Out).IsOk();
}

bool TypedDataConnect(FURLabRpcDispatcher& Dispatcher, const FString& Adapter,
	FURLabConnectReply& Out)
{
	return TypedDataConnectPolicy(Dispatcher, Adapter, TEXT("network"), Out);
}

bool TypedDataEncode(const FURLabConnectReply& Connection, uint64 Id,
	const FString& Op, FTypedDataValue Body, TArray<uint8>& Out)
{
	FURLabRequestEnvelope Request;
	Request.ProtocolVersion = 2;
	Request.BootId = Connection.BootId;
	Request.SessionId = Connection.SessionId;
	Request.RequestId = Id;
	Request.Op = Op;
	Request.Body = MoveTemp(Body);
	return FURLabRequestEnvelopeCodec::Encode(Request, Out).IsOk();
}

bool TypedDataDispatch(FURLabRpcDispatcher& Dispatcher, const FString& Adapter,
	const TArray<uint8>& Request, TArray<uint8>& OutBytes, FURLabReplyEnvelope& Out)
{
	return Dispatcher.DispatchBytes(Request, Adapter, OutBytes)
		&& FURLabRequestEnvelopeCodec::DecodeReply(OutBytes, Out).IsOk();
}

bool TypedDataAwait(FURLabRpcDispatcher& Dispatcher, const FString& Adapter,
	const TArray<uint8>& Request, TArray<uint8>& OutBytes, FURLabReplyEnvelope& Out)
{
	for (int32 Attempt = 0; Attempt < 400; ++Attempt)
	{
		if (!TypedDataDispatch(Dispatcher, Adapter, Request, OutBytes, Out)) return false;
		if (Out.Kind != EURLabReplyKind::Pending) return true;
		FPlatformProcess::SleepNoStats(0.005f);
	}
	return false;
}

bool TypedDataUpload(FURLabRpcDispatcher& Dispatcher, const FString& Adapter,
	const FURLabConnectReply& Connection, uint64& NextId, const TArray<uint8>& Bytes,
	FString& OutBlob, TArray<uint8>* OutBeginTerminal = nullptr)
{
	const FString Sha = FURLabAssetCache::Sha256Hex(Bytes);
	TArray<uint8> Request;
	if (!TypedDataEncode(Connection, NextId++, TEXT("payload_begin"), TypedDataMap({
		{TEXT("transfer"), FTypedDataValue::MakeString(TEXT("network"))},
		{TEXT("byte_length"), FTypedDataValue::MakeInt(Bytes.Num())},
		{TEXT("sha256"), FTypedDataValue::MakeString(Sha.ToUpper())}}), Request)) return false;
	TArray<uint8> ReplyBytes;
	FURLabReplyEnvelope Reply;
	if (!TypedDataAwait(Dispatcher, Adapter, Request, ReplyBytes, Reply)
		|| Reply.Kind != EURLabReplyKind::Result) return false;
	if (OutBeginTerminal) *OutBeginTerminal = ReplyBytes;
	const FTypedDataValue* Blob = Reply.Result.FindMapValue(TEXT("blob_id"));
	if (!Blob) return false;
	OutBlob = Blob->GetString();
	for (int32 Offset = 0; Offset < Bytes.Num(); Offset += 3)
	{
		const int32 Count = FMath::Min(3, Bytes.Num() - Offset);
		TArray<uint8> Chunk;
		Chunk.Append(Bytes.GetData() + Offset, Count);
		if (!TypedDataEncode(Connection, NextId++, TEXT("payload_write"), TypedDataMap({
			{TEXT("blob_id"), FTypedDataValue::MakeString(OutBlob)},
			{TEXT("offset"), FTypedDataValue::MakeInt(Offset)},
			{TEXT("data"), FTypedDataValue::MakeBinary(MoveTemp(Chunk))}}), Request)
			|| !TypedDataAwait(Dispatcher, Adapter, Request, ReplyBytes, Reply)
			|| Reply.Kind != EURLabReplyKind::Result) return false;
	}
	return TypedDataEncode(Connection, NextId++, TEXT("payload_commit"), TypedDataMap({
			{TEXT("blob_id"), FTypedDataValue::MakeString(OutBlob)}}), Request)
		&& TypedDataAwait(Dispatcher, Adapter, Request, ReplyBytes, Reply)
		&& Reply.Kind == EURLabReplyKind::Result
		&& Reply.Result.FindMapValue(TEXT("payload")) != nullptr;
}

const char* TypedDataClockXml = R"(
<mujoco model="typed_data_clock">
  <option timestep="0.01" gravity="0 0 0"/>
  <worldbody><body><joint type="hinge"/><geom type="sphere" size="0.1" mass="1"/></body></worldbody>
</mujoco>)";
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedDataGeneratedSchemaTest,
	"URLab.TypedDataDispatch.GeneratedTaggedValidation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabTypedDataGeneratedSchemaTest::RunTest(const FString&)
{
	const FTypedDataValue Begin = TypedDataMap({
		{TEXT("transfer"), FTypedDataValue::MakeString(TEXT("network"))},
		{TEXT("byte_length"), FTypedDataValue::MakeInt(4)},
		{TEXT("sha256"), FTypedDataValue::MakeString(FString::ChrN(64, TEXT('a')))}});
	TestTrue(TEXT("known request tag selects exact shape"),
		FURLabOperationSchema::ValidateRequestBody(TEXT("payload_begin"), Begin).IsOk());
	TestTrue(TEXT("SHM request tag selects its exact shape"),
		FURLabOperationSchema::ValidateRequestBody(TEXT("payload_begin"), TypedDataMap({
			{TEXT("transfer"), FTypedDataValue::MakeString(TEXT("shm"))},
			{TEXT("byte_length"), FTypedDataValue::MakeInt(4)},
			{TEXT("sha256"), FTypedDataValue::MakeString(FString::ChrN(64, TEXT('a')))}})).IsOk());
	TestFalse(TEXT("unknown request tag fails closed"),
		FURLabOperationSchema::ValidateRequestBody(TEXT("payload_begin"), TypedDataMap({
			{TEXT("transfer"), FTypedDataValue::MakeString(TEXT("mapped"))},
			{TEXT("byte_length"), FTypedDataValue::MakeInt(4)},
			{TEXT("sha256"), FTypedDataValue::MakeString(FString::ChrN(64, TEXT('a')))}})).IsOk());
	TestTrue(TEXT("network selection is its one-field exact variant"),
		FURLabOperationSchema::ValidateRequestBody(TEXT("data_select"), TypedDataMap({
			{TEXT("transfer"), FTypedDataValue::MakeString(TEXT("network"))}})).IsOk());
	TestFalse(TEXT("network selection rejects SHM-only fields"),
		FURLabOperationSchema::ValidateRequestBody(TEXT("data_select"), TypedDataMap({
			{TEXT("transfer"), FTypedDataValue::MakeString(TEXT("network"))},
			{TEXT("region_id"), FTypedDataValue::MakeString(TEXT("extra"))}})).IsOk());
	TestTrue(TEXT("SHM selection accepts the full exact proof shape"),
		FURLabOperationSchema::ValidateRequestBody(TEXT("data_select"), TypedDataMap({
			{TEXT("transfer"), FTypedDataValue::MakeString(TEXT("shm"))},
			{TEXT("region_id"), FTypedDataValue::MakeString(TEXT("region"))},
			{TEXT("region_generation"), FTypedDataValue::MakeInt(1)},
			{TEXT("challenge_name"), FTypedDataValue::MakeString(TEXT("challenge"))},
			{TEXT("challenge_nonce"), FTypedDataValue::MakeBinary(TypedDataBytes({1}))}})).IsOk());
	TestFalse(TEXT("SHM selection rejects a missing nonce"),
		FURLabOperationSchema::ValidateRequestBody(TEXT("data_select"), TypedDataMap({
			{TEXT("transfer"), FTypedDataValue::MakeString(TEXT("shm"))},
			{TEXT("region_id"), FTypedDataValue::MakeString(TEXT("region"))},
			{TEXT("region_generation"), FTypedDataValue::MakeInt(1)},
			{TEXT("challenge_name"), FTypedDataValue::MakeString(TEXT("challenge"))}})).IsOk());
	TestFalse(TEXT("malformed map storage returns a validation error"),
		FURLabOperationSchema::ValidateRequestBody(TEXT("payload_begin"),
			FTypedDataValue::MakeMap({TEXT("transfer")}, {})).IsOk());
	TestTrue(TEXT("true result literal validates"),
		FURLabOperationSchema::ValidateResultBody(TEXT("payload_release"),
			TypedDataMap({{TEXT("released"), FTypedDataValue::MakeBool(true)}})).IsOk());
	TestFalse(TEXT("false result literal is rejected"),
		FURLabOperationSchema::ValidateResultBody(TEXT("payload_release"),
			TypedDataMap({{TEXT("released"), FTypedDataValue::MakeBool(false)}})).IsOk());
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedDataNetworkRoundTripTest,
	"URLab.TypedDataDispatch.NetworkRoundTripIsolationAndRetention",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabTypedDataNetworkRoundTripTest::RunTest(const FString&)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->Start(TEXT(""));
	FURLabRpcDispatcher* Dispatcher = Server->GetDispatcher();
	FURLabConnectReply ZmqA, ZmqB, Grpc;
	if (!TestTrue(TEXT("ZMQ A connects"), TypedDataConnect(*Dispatcher, TEXT("zmq"), ZmqA))
		|| !TestTrue(TEXT("ZMQ B connects"), TypedDataConnect(*Dispatcher, TEXT("zmq"), ZmqB))
		|| !TestTrue(TEXT("gRPC connects"), TypedDataConnect(*Dispatcher, TEXT("grpc"), Grpc)))
	{
		Server->Stop();
		return false;
	}
	TestTrue(TEXT("model-free ZMQ advertises data"), ZmqA.Services.Contains(TEXT("data")));
	TestFalse(TEXT("model-free ZMQ does not invent simulation"), ZmqA.Services.Contains(TEXT("simulation")));
	TestTrue(TEXT("gRPC advertises data independently"), Grpc.Services.Contains(TEXT("data")));
	TestNotNull(TEXT("live max blob limit advertised"),
		ZmqA.Limits.FindMapValue(TEXT("payload_max_blob_bytes")));

	const TArray<uint8> Payload = TypedDataBytes({0, 1, 2, 3, 4, 0xff, 7});
	uint64 NextA = 1;
	FString BlobA;
	TArray<uint8> BeginTerminal;
	if (!TestTrue(TEXT("multi-chunk ZMQ upload commits"),
		TypedDataUpload(*Dispatcher, TEXT("zmq"), ZmqA, NextA, Payload, BlobA, &BeginTerminal)))
	{
		Server->Stop();
		return false;
	}

	TArray<uint8> ReadRequest;
	TypedDataEncode(ZmqA, NextA++, TEXT("payload_read"), TypedDataMap({
		{TEXT("blob_id"), FTypedDataValue::MakeString(BlobA)},
		{TEXT("offset"), FTypedDataValue::MakeInt(1)},
		{TEXT("byte_length"), FTypedDataValue::MakeInt(5)}}), ReadRequest);
	TArray<uint8> ReadTerminal, Duplicate;
	FURLabReplyEnvelope ReadReply, DuplicateReply;
	if (!TestTrue(TEXT("bounded read reaches terminal"),
		TypedDataAwait(*Dispatcher, TEXT("zmq"), ReadRequest, ReadTerminal, ReadReply))
		|| !TestEqual(TEXT("bounded read succeeds"), ReadReply.Kind, EURLabReplyKind::Result)
		|| !TestNotNull(TEXT("bounded read returns data"),
			ReadReply.Result.FindMapValue(TEXT("data"))))
	{
		Server->Stop();
		return false;
	}
	const TArray<uint8> ExpectedRead = TypedDataBytes({1, 2, 3, 4, 0xff});
	TestEqual(TEXT("read bytes are exact"),
		ReadReply.Result.FindMapValue(TEXT("data"))->GetBinary(), ExpectedRead);
	TestTrue(TEXT("duplicate read dispatches"),
		TypedDataDispatch(*Dispatcher, TEXT("zmq"), ReadRequest, Duplicate, DuplicateReply));
	TestEqual(TEXT("duplicate terminal is byte exact"), Duplicate, ReadTerminal);

	TArray<uint8> ForeignRequest, ForeignBytes;
	FURLabReplyEnvelope ForeignReply;
	TypedDataEncode(ZmqB, 1, TEXT("payload_read"), TypedDataMap({
		{TEXT("blob_id"), FTypedDataValue::MakeString(BlobA)},
		{TEXT("offset"), FTypedDataValue::MakeInt(0)},
		{TEXT("byte_length"), FTypedDataValue::MakeInt(1)}}), ForeignRequest);
	TestTrue(TEXT("foreign read reaches terminal"),
		TypedDataAwait(*Dispatcher, TEXT("zmq"), ForeignRequest, ForeignBytes, ForeignReply));
	TestEqual(TEXT("cross-session blob identity rejected"), ForeignReply.Error.Code,
		FString(TEXT("payload_identity_mismatch")));

	uint64 NextGrpc = 1;
	FString GrpcBlob;
	TestTrue(TEXT("gRPC uses the same data service"),
		TypedDataUpload(*Dispatcher, TEXT("grpc"), Grpc, NextGrpc, Payload, GrpcBlob));

	TArray<uint8> ReleaseRequest, ReleaseTerminal;
	FURLabReplyEnvelope ReleaseReply;
	TypedDataEncode(ZmqA, NextA++, TEXT("payload_release"), TypedDataMap({
		{TEXT("blob_id"), FTypedDataValue::MakeString(BlobA)}}), ReleaseRequest);
	if (!TestTrue(TEXT("release reaches terminal"),
		TypedDataAwait(*Dispatcher, TEXT("zmq"), ReleaseRequest, ReleaseTerminal, ReleaseReply))
		|| !TestEqual(TEXT("release succeeds"), ReleaseReply.Kind, EURLabReplyKind::Result)
		|| !TestNotNull(TEXT("release result is shaped"),
			ReleaseReply.Result.FindMapValue(TEXT("released"))))
	{
		Server->Stop();
		return false;
	}
	TestTrue(TEXT("release literal true"),
		ReleaseReply.Result.FindMapValue(TEXT("released"))->GetBool());
	TestTrue(TEXT("release duplicate dispatches"),
		TypedDataDispatch(*Dispatcher, TEXT("zmq"), ReleaseRequest, Duplicate, DuplicateReply));
	TestEqual(TEXT("release duplicate is byte exact"), Duplicate, ReleaseTerminal);
	Server->Stop();
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedDataShmNegotiationTest,
	"URLab.TypedDataDispatch.ShmNegotiationSelectionAndPayload",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabTypedDataShmNegotiationTest::RunTest(const FString&)
{
#if !PLATFORM_LINUX
	AddInfo(TEXT("Exact SHM data negotiation is Linux-only"));
	return true;
#else
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->Start(TEXT(""));
	FURLabRpcDispatcher* Dispatcher = Server->GetDispatcher();
	FURLabConnectReply Connection;
	if (!TestTrue(TEXT("ZMQ auto connect prepares SHM"),
		TypedDataConnectPolicy(*Dispatcher, TEXT("zmq"), TEXT("auto"), Connection)))
	{
		Server->Stop();
		return false;
	}
	if (!TestEqual(TEXT("connect is provisional SHM"),
		Connection.DataNegotiation.Transfer, FString(TEXT("shm")))
		|| !TestEqual(TEXT("challenge is exact length"),
			Connection.DataNegotiation.ChallengeByteLength, int64(32)))
	{
		Server->Stop();
		return false;
	}
	const FString ChallengePath = FPaths::Combine(
		Connection.DataNegotiation.DirectoryPath,
		Connection.DataNegotiation.ChallengeName);
	TArray<uint8> Nonce;
	if (!TestTrue(TEXT("client reads owner-bound challenge"),
		FFileHelper::LoadFileToArray(Nonce, *ChallengePath))
		|| !TestEqual(TEXT("challenge read is exact"), Nonce.Num(), 32))
	{
		Server->Stop();
		return false;
	}

	TArray<uint8> Request, ReplyBytes;
	FURLabReplyEnvelope Reply;
	TypedDataEncode(Connection, 1, TEXT("payload_begin"), TypedDataMap({
		{TEXT("transfer"), FTypedDataValue::MakeString(TEXT("network"))},
		{TEXT("byte_length"), FTypedDataValue::MakeInt(4)},
		{TEXT("sha256"), FTypedDataValue::MakeString(FString::ChrN(64, TEXT('a')))}}), Request);
	TestTrue(TEXT("ordinary provisional request replies"),
		TypedDataDispatch(*Dispatcher, TEXT("zmq"), Request, ReplyBytes, Reply));
	TestEqual(TEXT("ordinary provisional request is rejected"), Reply.Error.Code,
		FString(TEXT("data_negotiation_required")));

	TArray<uint8> WrongNonce = Nonce;
	WrongNonce[0] ^= 0xff;
	auto SelectionBody = [&Connection](TArray<uint8> Proof)
	{
		return TypedDataMap({
			{TEXT("transfer"), FTypedDataValue::MakeString(TEXT("shm"))},
			{TEXT("region_id"), FTypedDataValue::MakeString(
				Connection.DataNegotiation.RegionId)},
			{TEXT("region_generation"), FTypedDataValue::MakeInt(
				static_cast<int64>(Connection.DataNegotiation.RegionGeneration))},
			{TEXT("challenge_name"), FTypedDataValue::MakeString(
				Connection.DataNegotiation.ChallengeName)},
			{TEXT("challenge_nonce"), FTypedDataValue::MakeBinary(MoveTemp(Proof))}});
	};
	TypedDataEncode(Connection, 2, TEXT("data_select"),
		SelectionBody(MoveTemp(WrongNonce)), Request);
	TestTrue(TEXT("wrong proof reaches terminal"),
		TypedDataAwait(*Dispatcher, TEXT("zmq"), Request, ReplyBytes, Reply));
	TestEqual(TEXT("wrong proof cannot select"), Reply.Error.Code,
		FString(TEXT("shm_challenge_invalid")));

	TypedDataEncode(Connection, 3, TEXT("data_select"), SelectionBody(Nonce), Request);
	TArray<uint8> SelectionTerminal;
	if (!TestTrue(TEXT("exact proof reaches terminal"),
		TypedDataAwait(*Dispatcher, TEXT("zmq"), Request, SelectionTerminal, Reply))
		|| !TestEqual(TEXT("exact proof selects SHM"), Reply.Kind, EURLabReplyKind::Result))
	{
		Server->Stop();
		return false;
	}
	TestEqual(TEXT("selection returns immutable transfer"),
		Reply.Result.FindMapValue(TEXT("transfer"))->GetString(), FString(TEXT("shm")));
	TArray<uint8> Duplicate;
	FURLabReplyEnvelope DuplicateReply;
	TestTrue(TEXT("same selection ticket is recoverable"),
		TypedDataDispatch(*Dispatcher, TEXT("zmq"), Request, Duplicate, DuplicateReply));
	TestEqual(TEXT("same selection terminal is byte exact"), Duplicate, SelectionTerminal);
	TestFalse(TEXT("selection retires challenge path"),
		FPlatformFileManager::Get().GetPlatformFile().FileExists(*ChallengePath));

	TypedDataEncode(Connection, 4, TEXT("data_select"), TypedDataMap({
		{TEXT("transfer"), FTypedDataValue::MakeString(TEXT("network"))}}), Request);
	TestTrue(TEXT("new selection reaches terminal"),
		TypedDataAwait(*Dispatcher, TEXT("zmq"), Request, ReplyBytes, Reply));
	TestEqual(TEXT("selected backend is immutable"), Reply.Error.Code,
		FString(TEXT("data_selection_conflict")));

	const TArray<uint8> Payload = TypedDataBytes({5, 4, 3, 2, 1});
	const FString Sha = FURLabAssetCache::Sha256Hex(Payload);
	TypedDataEncode(Connection, 5, TEXT("payload_begin"), TypedDataMap({
		{TEXT("transfer"), FTypedDataValue::MakeString(TEXT("network"))},
		{TEXT("byte_length"), FTypedDataValue::MakeInt(Payload.Num())},
		{TEXT("sha256"), FTypedDataValue::MakeString(Sha)}}), Request);
	TestTrue(TEXT("wrong backing begin reaches terminal"),
		TypedDataAwait(*Dispatcher, TEXT("zmq"), Request, ReplyBytes, Reply));
	TestEqual(TEXT("wrong backing begin is explicit"), Reply.Error.Code,
		FString(TEXT("data_transfer_mismatch")));

	TypedDataEncode(Connection, 6, TEXT("payload_begin"), TypedDataMap({
		{TEXT("transfer"), FTypedDataValue::MakeString(TEXT("shm"))},
		{TEXT("byte_length"), FTypedDataValue::MakeInt(Payload.Num())},
		{TEXT("sha256"), FTypedDataValue::MakeString(Sha)}}), Request);
	if (!TestTrue(TEXT("SHM begin reaches terminal"),
		TypedDataAwait(*Dispatcher, TEXT("zmq"), Request, ReplyBytes, Reply))
		|| !TestEqual(TEXT("SHM begin succeeds"), Reply.Kind, EURLabReplyKind::Result))
	{
		Server->Stop();
		return false;
	}
	const FString BlobId = Reply.Result.FindMapValue(TEXT("blob_id"))->GetString();
	const FString ObjectName = Reply.Result.FindMapValue(TEXT("object_name"))->GetString();
	const FString StagingPath = FPaths::Combine(
		Connection.DataNegotiation.DirectoryPath, ObjectName);
	TUniquePtr<IFileHandle> Staging(
		FPlatformFileManager::Get().GetPlatformFile().OpenWrite(*StagingPath, false, false));
	if (!TestNotNull(TEXT("client opens granted staging object"), Staging.Get())
		|| !TestTrue(TEXT("client writes exact staging bytes"),
			Staging && Staging->Write(Payload.GetData(), Payload.Num())))
	{
		Server->Stop();
		return false;
	}
	Staging.Reset();
	TypedDataEncode(Connection, 7, TEXT("payload_commit"), TypedDataMap({
		{TEXT("blob_id"), FTypedDataValue::MakeString(BlobId)}}), Request);
	if (!TestTrue(TEXT("SHM commit reaches terminal"),
		TypedDataAwait(*Dispatcher, TEXT("zmq"), Request, ReplyBytes, Reply))
		|| !TestEqual(TEXT("SHM commit succeeds"), Reply.Kind, EURLabReplyKind::Result))
	{
		Server->Stop();
		return false;
	}
	const FTypedDataValue* Descriptor = Reply.Result.FindMapValue(TEXT("payload"));
	TestNotNull(TEXT("SHM commit returns descriptor"), Descriptor);
	if (Descriptor)
	{
		TestEqual(TEXT("descriptor transfer is SHM"),
			Descriptor->FindMapValue(TEXT("transfer"))->GetString(), FString(TEXT("shm")));
		TestEqual(TEXT("descriptor digest is exact"),
			Descriptor->FindMapValue(TEXT("sha256"))->GetString(), Sha);
		TestEqual(TEXT("descriptor generation is selected generation"),
			Descriptor->FindMapValue(TEXT("region_generation"))->GetInt(),
			static_cast<int64>(Connection.DataNegotiation.RegionGeneration));
	}

	TypedDataEncode(Connection, 8, TEXT("payload_read"), TypedDataMap({
		{TEXT("blob_id"), FTypedDataValue::MakeString(BlobId)},
		{TEXT("offset"), FTypedDataValue::MakeInt(0)},
		{TEXT("byte_length"), FTypedDataValue::MakeInt(1)}}), Request);
	TestTrue(TEXT("network read on SHM reaches terminal"),
		TypedDataAwait(*Dispatcher, TEXT("zmq"), Request, ReplyBytes, Reply));
	TestEqual(TEXT("network read on SHM is explicit"), Reply.Error.Code,
		FString(TEXT("data_transfer_mismatch")));
	Server->Stop();
	return !HasAnyErrors();
#endif
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedDataShmExpiryPolicyTest,
	"URLab.TypedDataDispatch.ShmRequiredAdapterAndChallengeExpiry",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabTypedDataShmExpiryPolicyTest::RunTest(const FString&)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->Start(TEXT(""));
	FURLabRpcDispatcher* Dispatcher = Server->GetDispatcher();
	FURLabConnectRequest Required;
	Required.ProtocolVersion = 2;
	Required.DataPolicy = TEXT("shm");
	TArray<uint8> Request, ReplyBytes;
	FURLabRequestEnvelopeCodec::EncodeConnectRequest(Required, Request);
	FURLabProtocolError ProtocolError;
	TestTrue(TEXT("gRPC required-SHM connect replies"),
		Dispatcher->DispatchBytes(Request, TEXT("grpc"), ReplyBytes));
	TestTrue(TEXT("gRPC required-SHM reply is protocol error"),
		FURLabRequestEnvelopeCodec::DecodeProtocolError(ReplyBytes, ProtocolError).IsOk());
	TestEqual(TEXT("gRPC never offers required SHM"), ProtocolError.Code,
		FString(TEXT("shm_unavailable")));
#if PLATFORM_LINUX
	FURLabConnectReply DeclineConnection;
	if (!TestTrue(TEXT("auto ZMQ prepares optional SHM"),
		TypedDataConnectPolicy(*Dispatcher, TEXT("zmq"), TEXT("auto"), DeclineConnection)))
	{
		Server->Stop();
		return false;
	}
	const FString DeclineChallengePath = FPaths::Combine(
		DeclineConnection.DataNegotiation.DirectoryPath,
		DeclineConnection.DataNegotiation.ChallengeName);
	TypedDataEncode(DeclineConnection, 1, TEXT("data_select"), TypedDataMap({
		{TEXT("transfer"), FTypedDataValue::MakeString(TEXT("network"))}}), Request);
	FURLabReplyEnvelope Reply;
	TestTrue(TEXT("auto client may decline before proof"),
		TypedDataAwait(*Dispatcher, TEXT("zmq"), Request, ReplyBytes, Reply));
	const FTypedDataValue* DeclinedTransfer = Reply.Kind == EURLabReplyKind::Result
		? Reply.Result.FindMapValue(TEXT("transfer")) : nullptr;
	TestNotNull(TEXT("decline returns selected transfer"), DeclinedTransfer);
	if (DeclinedTransfer)
	{
		TestEqual(TEXT("decline selects network"), DeclinedTransfer->GetString(),
			FString(TEXT("network")));
	}
	TestFalse(TEXT("decline retires challenge"),
		FPlatformFileManager::Get().GetPlatformFile().FileExists(*DeclineChallengePath));

	FURLabConnectReply ClosingConnection;
	if (!TestTrue(TEXT("second auto ZMQ prepares provisional session"),
		TypedDataConnectPolicy(*Dispatcher, TEXT("zmq"), TEXT("auto"), ClosingConnection)))
	{
		Server->Stop();
		return false;
	}
	const FString ClosingChallengePath = FPaths::Combine(
		ClosingConnection.DataNegotiation.DirectoryPath,
		ClosingConnection.DataNegotiation.ChallengeName);
	FURLabDisconnectRequest Disconnect{2, ClosingConnection.BootId,
		ClosingConnection.SessionId};
	FURLabRequestEnvelopeCodec::EncodeDisconnectRequest(Disconnect, Request);
	TestTrue(TEXT("provisional disconnect replies"),
		Dispatcher->DispatchBytes(Request, TEXT("zmq"), ReplyBytes));
	FURLabDisconnectReply Disconnected;
	TestTrue(TEXT("provisional disconnect succeeds"),
		FURLabRequestEnvelopeCodec::DecodeDisconnectReply(ReplyBytes, Disconnected).IsOk());
	TestFalse(TEXT("provisional disconnect retires challenge"),
		FPlatformFileManager::Get().GetPlatformFile().FileExists(*ClosingChallengePath));

	FURLabConnectReply FailedRetirementConnection;
	if (!TestTrue(TEXT("retirement-failure fixture connects"),
		TypedDataConnectPolicy(*Dispatcher, TEXT("zmq"), TEXT("auto"),
			FailedRetirementConnection)))
	{
		Server->Stop();
		return false;
	}
	const FString FailedChallengePath = FPaths::Combine(
		FailedRetirementConnection.DataNegotiation.DirectoryPath,
		FailedRetirementConnection.DataNegotiation.ChallengeName);
	TArray<uint8> FailedNonce;
	if (!TestTrue(TEXT("retirement-failure fixture reads proof"),
		FFileHelper::LoadFileToArray(FailedNonce, *FailedChallengePath))
		|| !TestTrue(TEXT("fixture removes challenge backing before server retirement"),
			FPlatformFileManager::Get().GetPlatformFile().DeleteFile(*FailedChallengePath)))
	{
		Server->Stop();
		return false;
	}
	auto FailedSelectionBody = [&FailedRetirementConnection, &FailedNonce]()
	{
		return TypedDataMap({
			{TEXT("transfer"), FTypedDataValue::MakeString(TEXT("shm"))},
			{TEXT("region_id"), FTypedDataValue::MakeString(
				FailedRetirementConnection.DataNegotiation.RegionId)},
			{TEXT("region_generation"), FTypedDataValue::MakeInt(static_cast<int64>(
				FailedRetirementConnection.DataNegotiation.RegionGeneration))},
			{TEXT("challenge_name"), FTypedDataValue::MakeString(
				FailedRetirementConnection.DataNegotiation.ChallengeName)},
			{TEXT("challenge_nonce"), FTypedDataValue::MakeBinary(FailedNonce)}});
	};
	TypedDataEncode(FailedRetirementConnection, 1, TEXT("data_select"),
		FailedSelectionBody(), Request);
	TArray<uint8> FailedSelectionTerminal, FailedSelectionDuplicate;
	TestTrue(TEXT("failed retirement reaches terminal"), TypedDataAwait(
		*Dispatcher, TEXT("zmq"), Request, FailedSelectionTerminal, Reply));
	TestEqual(TEXT("failed retirement is backing failure"), Reply.Error.Code,
		FString(TEXT("payload_backing_failure")));
	FURLabReplyEnvelope FailedDuplicateReply;
	TestTrue(TEXT("failed retirement original ticket remains recoverable"),
		TypedDataDispatch(*Dispatcher, TEXT("zmq"), Request,
			FailedSelectionDuplicate, FailedDuplicateReply));
	TestEqual(TEXT("failed retirement recovery is byte exact"),
		FailedSelectionDuplicate, FailedSelectionTerminal);
	TypedDataEncode(FailedRetirementConnection, 2, TEXT("payload_begin"), TypedDataMap({
		{TEXT("transfer"), FTypedDataValue::MakeString(TEXT("shm"))},
		{TEXT("byte_length"), FTypedDataValue::MakeInt(1)},
		{TEXT("sha256"), FTypedDataValue::MakeString(FString::ChrN(64, TEXT('a')))}}), Request);
	TestTrue(TEXT("failed selection cannot use payload service"),
		TypedDataDispatch(*Dispatcher, TEXT("zmq"), Request, ReplyBytes, Reply));
	TestEqual(TEXT("failed selection remains gated"), Reply.Error.Code,
		FString(TEXT("data_negotiation_required")));
	TypedDataEncode(FailedRetirementConnection, 3, TEXT("data_select"),
		FailedSelectionBody(), Request);
	TestTrue(TEXT("new selection after retirement failure reaches terminal"),
		TypedDataAwait(*Dispatcher, TEXT("zmq"), Request, ReplyBytes, Reply));
	TestEqual(TEXT("retirement failure permanently disables selection"), Reply.Error.Code,
		FString(TEXT("payload_backing_failure")));

	Dispatcher->SetSessionClockForTest(100.0);
	FURLabConnectReply Connection;
	if (!TestTrue(TEXT("required SHM prepares on Linux ZMQ"),
		TypedDataConnectPolicy(*Dispatcher, TEXT("zmq"), TEXT("shm"), Connection)))
	{
		Server->Stop();
		return false;
	}
	TArray<uint8> Nonce;
	const FString ChallengePath = FPaths::Combine(
		Connection.DataNegotiation.DirectoryPath,
		Connection.DataNegotiation.ChallengeName);
	if (!TestTrue(TEXT("expiry fixture reads challenge"),
		FFileHelper::LoadFileToArray(Nonce, *ChallengePath)))
	{
		Server->Stop();
		return false;
	}
	Dispatcher->SetSessionClockForTest(131.0);
	TypedDataEncode(Connection, 1, TEXT("simulation_describe"), TypedDataMap({}), Request);
	TestTrue(TEXT("ordinary keepalive after challenge deadline replies"),
		TypedDataDispatch(*Dispatcher, TEXT("zmq"), Request, ReplyBytes, Reply));
	TestEqual(TEXT("expired provisional session remains negotiation-gated"), Reply.Error.Code,
		FString(TEXT("data_negotiation_required")));
	TestFalse(TEXT("ordinary keepalive lazily retires expired challenge"),
		FPlatformFileManager::Get().GetPlatformFile().FileExists(*ChallengePath));
	TypedDataEncode(Connection, 2, TEXT("data_select"), TypedDataMap({
		{TEXT("transfer"), FTypedDataValue::MakeString(TEXT("shm"))},
		{TEXT("region_id"), FTypedDataValue::MakeString(Connection.DataNegotiation.RegionId)},
		{TEXT("region_generation"), FTypedDataValue::MakeInt(
			static_cast<int64>(Connection.DataNegotiation.RegionGeneration))},
		{TEXT("challenge_name"), FTypedDataValue::MakeString(
			Connection.DataNegotiation.ChallengeName)},
		{TEXT("challenge_nonce"), FTypedDataValue::MakeBinary(MoveTemp(Nonce))}}), Request);
	TestTrue(TEXT("expired proof reaches terminal"),
		TypedDataAwait(*Dispatcher, TEXT("zmq"), Request, ReplyBytes, Reply));
	TestEqual(TEXT("expired proof has exact error"), Reply.Error.Code,
		FString(TEXT("shm_challenge_expired")));
	TestFalse(TEXT("expired challenge remains retired"),
		FPlatformFileManager::Get().GetPlatformFile().FileExists(*ChallengePath));
#endif
	Server->Stop();
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedDataShutdownAndReopenTest,
	"URLab.TypedDataDispatch.ShutdownCancellationAndExactScopeReopen",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabTypedDataShutdownAndReopenTest::RunTest(const FString&)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->Start(TEXT(""));
	FURLabRpcDispatcher* Dispatcher = Server->GetDispatcher();
	Dispatcher->SetPayloadQueueAutoDrainForTest(false);
	FURLabConnectReply Connection;
	if (!TestTrue(TEXT("connect"), TypedDataConnect(*Dispatcher, TEXT("zmq"), Connection)))
	{
		Server->Stop();
		return false;
	}
	const TArray<uint8> Data = TypedDataBytes({1, 2, 3});
	TArray<uint8> Request;
	TypedDataEncode(Connection, 1, TEXT("payload_begin"), TypedDataMap({
		{TEXT("transfer"), FTypedDataValue::MakeString(TEXT("network"))},
		{TEXT("byte_length"), FTypedDataValue::MakeInt(Data.Num())},
		{TEXT("sha256"), FTypedDataValue::MakeString(FURLabAssetCache::Sha256Hex(Data))}}), Request);
	TArray<uint8> PendingBytes;
	FURLabReplyEnvelope Pending;
	TestTrue(TEXT("payload work queues"),
		TypedDataDispatch(*Dispatcher, TEXT("zmq"), Request, PendingBytes, Pending));
	TestEqual(TEXT("queued work reports pending"), Pending.Kind, EURLabReplyKind::Pending);
	const TSharedRef<FURLabSerialCommandQueue, ESPMode::ThreadSafe> Queue =
		Dispatcher->GetPayloadCommandQueueForTest();
	const TSharedRef<FURLabRequestLedger, ESPMode::ThreadSafe> Ledger =
		Dispatcher->GetRequestLedgerForTest();
	FEvent* Gate = FPlatformProcess::GetSynchEventFromPool(true);
	TFuture<void> Posted = Async(EAsyncExecution::ThreadPool, [Queue, Gate]()
	{
		Gate->Wait();
		Queue->Drain();
	});
	Server->Stop();
	TestEqual(TEXT("shutdown releases pending ledger quota"), Ledger->GetStats().Pending, 0);
	Gate->Trigger();
	Posted.Get();
	FPlatformProcess::ReturnSynchEventToPool(Gate);

	double Now = 1.0;
	FURLabPayloadStore Store(TEXT("boot"), FURLabPayloadStoreLimits(), [&Now]() { return Now; });
	FURLabPayloadSessionPtr OldSession;
	TestTrue(TEXT("old scope opens"), Store.OpenSession(TEXT("boot"), TEXT("same"), OldSession).IsOk());
	const FURLabPayloadOwner OldOwner = FURLabPayloadOwner::FromSession(OldSession);
	OldSession->Deactivate();
	FURLabPayloadSessionPtr PrematureReplacement;
	TestEqual(TEXT("inactive scope cannot masquerade as a fresh session"),
		Store.OpenSession(TEXT("boot"), TEXT("same"), PrematureReplacement).Code,
		EURLabPayloadStoreError::SessionRevoked);
	Now = std::numeric_limits<double>::quiet_NaN();
	int32 Removed = 0;
	TestTrue(TEXT("revoke remains possible with invalid operation clock"),
		Store.RevokeSession(OldOwner, Removed).IsOk());
	Now = 2.0;
	FURLabPayloadSessionPtr NewSession;
	TestTrue(TEXT("same textual id gets a fresh exact scope"),
		Store.OpenSession(TEXT("boot"), TEXT("same"), NewSession).IsOk());
	TestNotEqual(TEXT("scope pointer changes"), OldSession.Get(), NewSession.Get());
	TestEqual(TEXT("stale scope cannot revoke successor"), Store.RevokeSession(OldOwner, Removed).Code,
		EURLabPayloadStoreError::SessionRevoked);
	TestTrue(TEXT("successor remains active"), NewSession->IsActive());
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedClockWireTest,
	"URLab.TypedDataDispatch.RemoteClockStatusAuthorityAndRetention",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabTypedClockWireTest::RunTest(const FString&)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->Start(TEXT(""));
	FMjCapabilitySet Grants;
	Grants.Set(EMjCapability::AcceptInput, true);
	Server->SetGrants(Grants);
	const TSharedRef<FMjSimulationRuntime, ESPMode::ThreadSafe> Runtime =
		MakeShared<FMjSimulationRuntime, ESPMode::ThreadSafe>();
	FString Error;
	FMjOwnedModelCandidate Candidate;
	if (!TestTrue(TEXT("clock model decodes"), FMjOwnedModelCandidate::FromBytes(
		EMjModelSourceKind::XmlWithAssets, TypedDataXmlBytes(TypedDataClockXml), {},
		Candidate, Error, TEXT("typed clock wire"))))
	{
		AddError(Error);
		Server->Stop();
		return false;
	}
	const FMjSimulationInstallResult Installed = Runtime->Install(MoveTemp(Candidate));
	FURLabRpcDispatcher::FSimulationRuntimeRegistration Registration;
	if (!TestTrue(TEXT("clock model installs"), Installed.bCommitted)
		|| !TestTrue(TEXT("runtime attaches"), Server->GetDispatcher()->AttachSimulationRuntime(
			Runtime, Registration, Error)))
	{
		AddError(Error);
		Server->Stop();
		return false;
	}
	FURLabConnectReply Connection;
	if (!TestTrue(TEXT("clock client connects"),
		TypedDataConnect(*Server->GetDispatcher(), TEXT("zmq"), Connection)))
	{
		Server->Stop();
		return false;
	}
	TArray<uint8> Request, ReplyBytes;
	FURLabReplyEnvelope Reply;
	TypedDataEncode(Connection, 1, TEXT("simulation_clock_status"), TypedDataMap({}), Request);
	if (!TestTrue(TEXT("passive status succeeds"), TypedDataAwait(
		*Server->GetDispatcher(), TEXT("zmq"), Request, ReplyBytes, Reply))
		|| !TestEqual(TEXT("passive status is a result"), Reply.Kind, EURLabReplyKind::Result)
		|| !TestNotNull(TEXT("clock status contains mode"),
			Reply.Result.FindMapValue(TEXT("mode"))))
	{
		Server->Stop();
		return false;
	}
	TestEqual(TEXT("clock defaults stepped"), Reply.Result.FindMapValue(TEXT("mode"))->GetString(), FString(TEXT("stepped")));

	TypedDataEncode(Connection, 2, TEXT("simulation_claim_control"), TypedDataMap({
		{TEXT("model_generation"), FTypedDataValue::MakeInt(Installed.Generation)},
		{TEXT("ttl_s"), FTypedDataValue::MakeFloat(30.0)}}), Request);
	if (!TestTrue(TEXT("control claim completes"), TypedDataAwait(
		*Server->GetDispatcher(), TEXT("zmq"), Request, ReplyBytes, Reply))
		|| !TestEqual(TEXT("control claim succeeds"), Reply.Kind, EURLabReplyKind::Result))
	{
		Server->Stop();
		return false;
	}
	const FTypedDataValue* Control = Reply.Result.FindMapValue(TEXT("control"));
	const FTypedDataValue* TokenValue = Control
		? Control->FindMapValue(TEXT("token")) : nullptr;
	if (!TestNotNull(TEXT("claim returns control token"), TokenValue))
	{
		Server->Stop();
		return false;
	}
	const FString Token = TokenValue->GetString();
	TypedDataEncode(Connection, 3, TEXT("simulation_set_clock"), TypedDataMap({
		{TEXT("model_generation"), FTypedDataValue::MakeInt(Installed.Generation)},
		{TEXT("control_token"), FTypedDataValue::MakeString(Token)},
		{TEXT("mode"), FTypedDataValue::MakeString(TEXT("freerun"))}}), Request);
	TArray<uint8> FirstTerminal, Duplicate;
	if (!TestTrue(TEXT("authorized clock switch succeeds"),
		TypedDataAwait(*Server->GetDispatcher(), TEXT("zmq"), Request, FirstTerminal, Reply))
		|| !TestEqual(TEXT("clock switch is a result"), Reply.Kind, EURLabReplyKind::Result)
		|| !TestNotNull(TEXT("clock switch contains changed"),
			Reply.Result.FindMapValue(TEXT("changed"))))
	{
		Server->Stop();
		return false;
	}
	TestTrue(TEXT("clock switch committed"), Reply.Result.FindMapValue(TEXT("changed"))->GetBool());
	TestTrue(TEXT("clock duplicate dispatches"),
		Server->GetDispatcher()->DispatchBytes(Request, TEXT("zmq"), Duplicate));
	TestEqual(TEXT("clock duplicate is byte exact"), Duplicate, FirstTerminal);
	Server->Stop();
	return !HasAnyErrors();
}

#endif // WITH_DEV_AUTOMATION_TESTS
