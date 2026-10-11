#include "Misc/AutomationTest.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/RpcDispatcher.h"
#include "MuJoCo/Runtime/MjSimulationRuntime.h"
#include "Transport/ZmqRpcTransport.h"
#include "Wire/RequestEnvelope.h"
#include "Async/Async.h"
#include "Async/TaskGraphInterfaces.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
const char* DynamicRuntimeXml = R"(
<mujoco model="typed_dispatch_runtime">
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="falling" pos="0 0 1">
      <freejoint name="root"/>
      <geom name="ball" type="sphere" size="0.05" mass="1"/>
    </body>
  </worldbody>
</mujoco>)";

TArray<uint8> TypedDispatchBytesOf(const char* Text)
{
	TArray<uint8> Out;
	Out.Append(reinterpret_cast<const uint8*>(Text), FCStringAnsi::Strlen(Text));
	return Out;
}

bool InstallRuntime(const TSharedRef<FMjSimulationRuntime, ESPMode::ThreadSafe>& Runtime,
	FString& OutError)
{
	FMjOwnedModelCandidate Candidate;
	if (!FMjOwnedModelCandidate::FromBytes(EMjModelSourceKind::XmlWithAssets,
		TypedDispatchBytesOf(DynamicRuntimeXml), {}, Candidate, OutError, TEXT("typed dispatch fixture")))
	{
		return false;
	}
	const FMjSimulationInstallResult Result = Runtime->Install(MoveTemp(Candidate));
	OutError = Result.Error;
	return Result.bCommitted;
}

FURLabWireValue EmptyBody()
{
	return FURLabWireValue::MakeMap({}, {});
}

bool Connect(UURLabZmqRpcTransport* Transport, FURLabConnectReply& Out)
{
	TArray<uint8> Request;
	TArray<uint8> Reply;
	if (!FURLabRequestEnvelopeCodec::EncodeConnectRequest({2}, Request)) return false;
	if (!Transport->ProcessRequestBytes(Request, Reply)) return false;
	return FURLabRequestEnvelopeCodec::DecodeConnectReply(Reply, Out).IsOk();
}

FURLabRequestEnvelope RequestFor(const FURLabConnectReply& Connection, int64 Id,
	const FString& Op, FURLabWireValue Body = EmptyBody())
{
	FURLabRequestEnvelope Request;
	Request.ProtocolVersion = 2;
	Request.BootId = Connection.BootId;
	Request.SessionId = Connection.SessionId;
	Request.RequestId = static_cast<uint64>(Id);
	Request.Op = Op;
	Request.Body = MoveTemp(Body);
	return Request;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedDispatchProductionBytes,
	"URLab.TypedDispatch.ProductionBytesAndRetention",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FURLabTypedDispatchProductionBytes::RunTest(const FString&)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->Start(TEXT(""));
	UURLabZmqRpcTransport* Transport = NewObject<UURLabZmqRpcTransport>();
	Transport->SetOwningBridge(Server);
	FURLabConnectReply Connection;
	TestTrue(TEXT("strict connect uses production byte path"), Connect(Transport, Connection));
	TestEqual(TEXT("model-free connection advertises only data"), Connection.Services.Num(), 1);
	TestTrue(TEXT("model-free data service is advertised"), Connection.Services.Contains(TEXT("data")));

	FURLabRequestEnvelope Request = RequestFor(Connection, 1, TEXT("simulation_observe"));
	TArray<uint8> Bytes, First, Duplicate;
	TestTrue(TEXT("encode regular envelope"), FURLabRequestEnvelopeCodec::Encode(Request, Bytes).IsOk());
	TestTrue(TEXT("dispatch accepted read"), Transport->ProcessRequestBytes(Bytes, First));
	TestTrue(TEXT("duplicate dispatch"), Transport->ProcessRequestBytes(Bytes, Duplicate));
	TestEqual(TEXT("terminal failure is retained byte exactly"), Duplicate, First);
	FURLabReplyEnvelope Reply;
	TestTrue(TEXT("terminal reply decodes"), FURLabRequestEnvelopeCodec::DecodeReply(First, Reply).IsOk());
	TestEqual(TEXT("missing model is admitted terminal"), Reply.Kind, EURLabReplyKind::Error);
	TestEqual(TEXT("terminal execution classification"), Reply.Error.Execution, EURLabExecution::Terminal);
	Server->GetDispatcher()->SetDraining(true);
	TArray<uint8> DrainingBytes;
	if (!TestTrue(TEXT("draining recovery reply"), Transport->ProcessRequestBytes(Bytes, DrainingBytes)) ||
		!TestTrue(TEXT("draining recovery decodes"), FURLabRequestEnvelopeCodec::DecodeReply(DrainingBytes, Reply).IsOk()))
	{
		Server->Stop();
		return false;
	}
	TestEqual(TEXT("draining cannot disprove earlier execution"), Reply.Error.Execution, EURLabExecution::Unknown);
	Server->GetDispatcher()->SetDraining(false);

	FURLabDisconnectRequest Disconnect{2, Connection.BootId, Connection.SessionId};
	TArray<uint8> DisconnectBytes, DisconnectReply;
	FURLabRequestEnvelopeCodec::EncodeDisconnectRequest(Disconnect, DisconnectBytes);
	TestTrue(TEXT("disconnect"), Transport->ProcessRequestBytes(DisconnectBytes, DisconnectReply));
	TArray<uint8> ClosedReplyBytes;
	Transport->ProcessRequestBytes(Bytes, ClosedReplyBytes);
	FURLabReplyEnvelope ClosedReply;
	TestTrue(TEXT("closed reply decodes"), FURLabRequestEnvelopeCodec::DecodeReply(ClosedReplyBytes, ClosedReply).IsOk());
	TestEqual(TEXT("closed session cannot imply non-execution"), ClosedReply.Error.Execution, EURLabExecution::Unknown);
	Server->Stop();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedInteractionRoutesToStreamsHandler,
	"URLab.TypedDispatch.InteractionSimulationQueueRoutesToStreamsHandler",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FURLabTypedInteractionRoutesToStreamsHandler::RunTest(const FString&)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->Start(TEXT(""));
	FMjCapabilitySet Grants;
	Grants.Set(EMjCapability::AcceptInput, true);
	Server->SetGrants(Grants);
	UURLabZmqRpcTransport* Transport = NewObject<UURLabZmqRpcTransport>();
	Transport->SetOwningBridge(Server);
	FURLabConnectReply Connection;
	if (!TestTrue(TEXT("connect"), Connect(Transport, Connection))) return false;
	auto Map = [](std::initializer_list<TPair<FString, FURLabWireValue>> Fields)
	{
		TArray<FString> Keys;
		TArray<FURLabWireValue> Values;
		for (const auto& Field : Fields) { Keys.Add(Field.Key); Values.Add(Field.Value); }
		return FURLabWireValue::MakeMap(MoveTemp(Keys), MoveTemp(Values));
	};
	auto Vec3 = []
	{
		return FURLabWireValue::MakeArray({FURLabWireValue::MakeFloat(0.0),
			FURLabWireValue::MakeFloat(0.0), FURLabWireValue::MakeFloat(0.0)});
	};
	FURLabWireValue Source = Map({
		{TEXT("producer_boot_id"), FURLabWireValue::MakeString(TEXT("producer"))},
		{TEXT("stream_id"), FURLabWireValue::MakeString(TEXT("poses"))},
		{TEXT("stream_generation"), FURLabWireValue::MakeInt(1)},
		{TEXT("source_revision"), FURLabWireValue::MakeString(TEXT("scene"))}});
	FURLabWireValue Grant = Map({
		{TEXT("grant_token"), FURLabWireValue::MakeString(TEXT("grant"))},
		{TEXT("source"), MoveTemp(Source)},
		{TEXT("viewer_boot_id"), FURLabWireValue::MakeString(TEXT("viewer"))},
		{TEXT("clock_epoch"), FURLabWireValue::MakeInt(1)},
		{TEXT("expires_in_s"), FURLabWireValue::MakeFloat(5.0)}});
	FURLabWireValue Wrench = Map({
		{TEXT("body_index"), FURLabWireValue::MakeInt(1)},
		{TEXT("force_n"), Vec3()}, {TEXT("torque_nm"), Vec3()}});
	FURLabRequestEnvelope Request = RequestFor(Connection, 41, TEXT("interaction_submit"),
		Map({{TEXT("grant"), MoveTemp(Grant)}, {TEXT("wrench"), MoveTemp(Wrench)}}));
	TArray<uint8> RequestBytes, ReplyBytes;
	FURLabRequestEnvelopeCodec::Encode(Request, RequestBytes);
	TestTrue(TEXT("dispatch"), Transport->ProcessRequestBytes(RequestBytes, ReplyBytes));
	FURLabReplyEnvelope Reply;
	if (!TestTrue(TEXT("reply decodes"),
		FURLabRequestEnvelopeCodec::DecodeReply(ReplyBytes, Reply).IsOk())) return false;
	TestEqual(TEXT("request reached streams handler without a bound producer"),
		Reply.Error.Code, FString(TEXT("unsupported")));
	Server->Stop();
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedDispatchRejectsBeforeAdmission,
	"URLab.TypedDispatch.MalformedAdapterAndConflict",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FURLabTypedDispatchRejectsBeforeAdmission::RunTest(const FString&)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->Start(TEXT(""));
	UURLabZmqRpcTransport* Transport = NewObject<UURLabZmqRpcTransport>();
	Transport->SetOwningBridge(Server);
	FURLabConnectReply Connection;
	TestTrue(TEXT("connect"), Connect(Transport, Connection));

	FURLabRequestEnvelope BadBody = RequestFor(Connection, 7, TEXT("simulation_describe"),
		FURLabWireValue::MakeMap({TEXT("extra")}, {FURLabWireValue::MakeInt(1)}));
	TArray<uint8> BadBytes, BadReply;
	if (!TestTrue(TEXT("encode malformed body"),
		FURLabRequestEnvelopeCodec::Encode(BadBody, BadBytes).IsOk()) ||
		!TestTrue(TEXT("dispatch malformed body"),
			Transport->ProcessRequestBytes(BadBytes, BadReply)))
	{
		Server->Stop();
		return false;
	}
	FURLabReplyEnvelope Decoded;
	if (!TestTrue(TEXT("decode malformed-body rejection"),
		FURLabRequestEnvelopeCodec::DecodeReply(BadReply, Decoded).IsOk()))
	{
		Server->Stop();
		return false;
	}
	TestEqual(TEXT("shape error is rejected"), Decoded.Error.Execution, EURLabExecution::Rejected);

	FURLabRequestEnvelope Good = RequestFor(Connection, 7, TEXT("simulation_describe"));
	TArray<uint8> GoodBytes, GoodReply;
	if (!TestTrue(TEXT("encode valid request"),
		FURLabRequestEnvelopeCodec::Encode(Good, GoodBytes).IsOk()) ||
		!TestTrue(TEXT("dispatch valid request"),
			Transport->ProcessRequestBytes(GoodBytes, GoodReply)) ||
		!TestTrue(TEXT("decode valid terminal"),
			FURLabRequestEnvelopeCodec::DecodeReply(GoodReply, Decoded).IsOk()))
	{
		Server->Stop();
		return false;
	}
	TestEqual(TEXT("rejected malformed body did not consume id"), Decoded.Error.Execution, EURLabExecution::Terminal);
	FURLabRequestEnvelope Changed = RequestFor(Connection, 7, TEXT("simulation_observe"));
	TArray<uint8> ChangedBytes, ConflictBytes;
	if (!TestTrue(TEXT("encode changed identity"), FURLabRequestEnvelopeCodec::Encode(Changed, ChangedBytes).IsOk()) ||
		!TestTrue(TEXT("dispatch conflict"), Transport->ProcessRequestBytes(ChangedBytes, ConflictBytes)) ||
		!TestTrue(TEXT("decode conflict"), FURLabRequestEnvelopeCodec::DecodeReply(ConflictBytes, Decoded).IsOk()))
	{
		Server->Stop();
		return false;
	}
	TestEqual(TEXT("same id with changed digest conflicts"), Decoded.Error.Code, FString(TEXT("request_id_conflict")));
	for (int64 Id = 8; Id <= 1032; ++Id)
	{
		FURLabRequestEnvelope Advancing = RequestFor(Connection, Id, TEXT("simulation_describe"));
		TArray<uint8> AdvancingBytes, AdvancingReply;
		if (!FURLabRequestEnvelopeCodec::Encode(Advancing, AdvancingBytes) ||
			!Transport->ProcessRequestBytes(AdvancingBytes, AdvancingReply))
		{
			AddError(FString::Printf(TEXT("failed to advance ledger at id %lld"), Id));
			Server->Stop();
			return false;
		}
	}
	TArray<uint8> RetiredReply;
	if (!TestTrue(TEXT("dispatch retired identity"), Transport->ProcessRequestBytes(GoodBytes, RetiredReply)) ||
		!TestTrue(TEXT("decode retired identity"), FURLabRequestEnvelopeCodec::DecodeReply(RetiredReply, Decoded).IsOk()))
	{
		Server->Stop();
		return false;
	}
	TestEqual(TEXT("retired identity outcome is unknown"), Decoded.Error.Execution, EURLabExecution::Unknown);
	TestEqual(TEXT("retired identity code"), Decoded.Error.Code, FString(TEXT("request_retired")));

	TArray<uint8> GrpcReply;
	if (!TestTrue(TEXT("dispatch gRPC adapter rejection"),
		Server->GetDispatcher()->DispatchBytes(GoodBytes, TEXT("grpc"), GrpcReply, &Good.Op)) ||
		!TestTrue(TEXT("decode gRPC adapter rejection"),
			FURLabRequestEnvelopeCodec::DecodeReply(GrpcReply, Decoded).IsOk()))
	{
		Server->Stop();
		return false;
	}
	TestEqual(TEXT("cross-adapter ticket outcome is unknown"), Decoded.Error.Execution, EURLabExecution::Unknown);

	FURLabDisconnectRequest CrossDisconnect{2, Connection.BootId, Connection.SessionId};
	TArray<uint8> CrossDisconnectBytes, CrossDisconnectReply;
	TestTrue(TEXT("encode cross-adapter disconnect"), FURLabRequestEnvelopeCodec::EncodeDisconnectRequest(CrossDisconnect, CrossDisconnectBytes).IsOk());
	TestTrue(TEXT("cross-adapter disconnect rejected"), Server->GetDispatcher()->DispatchBytes(
		CrossDisconnectBytes, TEXT("grpc"), CrossDisconnectReply, nullptr));
	FURLabProtocolError DisconnectError;
	TestTrue(TEXT("cross-adapter disconnect protocol error"),
		FURLabRequestEnvelopeCodec::DecodeProtocolError(CrossDisconnectReply, DisconnectError).IsOk());
	TestEqual(TEXT("cross-adapter disconnect cannot close session"), DisconnectError.Code,
		FString(TEXT("session_expired")));
	TArray<uint8> StillActive;
	TestTrue(TEXT("original adapter session remains active"), Transport->ProcessRequestBytes(GoodBytes, StillActive));

	TArray<uint8> GrpcConnectBytes, GrpcConnectReplyBytes;
	TestTrue(TEXT("encode grpc connect"), FURLabRequestEnvelopeCodec::EncodeConnectRequest({2}, GrpcConnectBytes).IsOk());
	TestTrue(TEXT("grpc connect dispatch"), Server->GetDispatcher()->DispatchBytes(
		GrpcConnectBytes, TEXT("grpc"), GrpcConnectReplyBytes, nullptr));
	FURLabConnectReply GrpcConnection;
	TestTrue(TEXT("decode grpc connect"), FURLabRequestEnvelopeCodec::DecodeConnectReply(GrpcConnectReplyBytes, GrpcConnection).IsOk());
	FURLabRequestEnvelope FreshGrpc = RequestFor(GrpcConnection, 1, TEXT("simulation_describe"));
	TArray<uint8> FreshGrpcBytes, FreshGrpcReply;
	TestTrue(TEXT("encode fresh grpc request"), FURLabRequestEnvelopeCodec::Encode(FreshGrpc, FreshGrpcBytes).IsOk());
	TestTrue(TEXT("fresh grpc dispatch"), Server->GetDispatcher()->DispatchBytes(
		FreshGrpcBytes, TEXT("grpc"), FreshGrpcReply, &FreshGrpc.Op));
	TestTrue(TEXT("fresh grpc rejection decodes"), FURLabRequestEnvelopeCodec::DecodeReply(FreshGrpcReply, Decoded).IsOk());
	TestEqual(TEXT("fresh grpc simulation rejection"), Decoded.Error.Execution, EURLabExecution::Rejected);
	TestEqual(TEXT("fresh grpc simulation reports wrong_transport"), Decoded.Error.Code, FString(TEXT("wrong_transport")));
	Server->Stop();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedDispatchRetainsCoherentObservation,
	"URLab.TypedDispatch.CoherentObservationRetainedAfterRuntimeStep",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FURLabTypedDispatchRetainsCoherentObservation::RunTest(const FString&)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->Start(TEXT(""));
	const TSharedRef<FMjSimulationRuntime, ESPMode::ThreadSafe> Runtime =
		MakeShared<FMjSimulationRuntime, ESPMode::ThreadSafe>();
	FString Error;
	if (!TestTrue(TEXT("independent runtime fixture"), InstallRuntime(Runtime, Error)))
	{
		AddError(Error);
		Server->Stop();
		return false;
	}
	FURLabRpcDispatcher::FSimulationRuntimeRegistration Registration;
	if (!TestTrue(TEXT("attach independent runtime"),
		Server->GetDispatcher()->AttachSimulationRuntime(Runtime, Registration, Error)))
	{
		AddError(Error);
		Server->Stop();
		return false;
	}
	UURLabZmqRpcTransport* Transport = NewObject<UURLabZmqRpcTransport>();
	Transport->SetOwningBridge(Server);
	FURLabConnectReply Connection;
	TestTrue(TEXT("connect"), Connect(Transport, Connection));
	TestTrue(TEXT("simulation service is callable"),
		Connection.Services.Contains(TEXT("simulation")));
	TestTrue(TEXT("data service remains independent"),
		Connection.Services.Contains(TEXT("data")));

	FURLabRequestEnvelope Request = RequestFor(Connection, 23, TEXT("simulation_observe"));
	TArray<uint8> RequestBytes, First, Duplicate;
	TestTrue(TEXT("encode observation"), FURLabRequestEnvelopeCodec::Encode(Request, RequestBytes).IsOk());
	TestTrue(TEXT("first observation"), Transport->ProcessRequestBytes(RequestBytes, First));
	FURLabReplyEnvelope FirstReply;
	if (!TestTrue(TEXT("first observation decodes"), FURLabRequestEnvelopeCodec::DecodeReply(First, FirstReply).IsOk()) ||
		!TestEqual(TEXT("first observation is terminal result"), FirstReply.Kind, EURLabReplyKind::Result)) return false;
	const FURLabWireValue* FirstQpos = FirstReply.Result.FindMapValue(TEXT("qpos"));
	if (!TestNotNull(TEXT("first qpos"), FirstQpos) ||
		!TestTrue(TEXT("free-joint qpos is present"), FirstQpos->GetArray().Num() >= 3)) return false;
	const double FirstSample = FirstQpos->GetArray()[2].GetFloat();
	TestTrue(TEXT("runtime advances explicitly"), Runtime->Step(1, Error));
	FURLabRequestEnvelope NewRequest = RequestFor(Connection, 24, TEXT("simulation_observe"));
	TArray<uint8> NewBytes, NewReplyBytes;
	TestTrue(TEXT("encode new observation"), FURLabRequestEnvelopeCodec::Encode(NewRequest, NewBytes).IsOk());
	TestTrue(TEXT("new observation"), Transport->ProcessRequestBytes(NewBytes, NewReplyBytes));
	FURLabReplyEnvelope NewReply;
	if (!TestTrue(TEXT("new observation decodes"), FURLabRequestEnvelopeCodec::DecodeReply(NewReplyBytes, NewReply).IsOk()) ||
		!TestEqual(TEXT("new observation is result"), NewReply.Kind, EURLabReplyKind::Result)) return false;
	const FURLabWireValue* NewQpos = NewReply.Result.FindMapValue(TEXT("qpos"));
	if (!TestNotNull(TEXT("new qpos"), NewQpos)) return false;
	TestTrue(TEXT("new request sees the falling body after one runtime step"),
		NewQpos->GetArray()[2].GetFloat() < FirstSample - 1.0e-10);
	TestTrue(TEXT("duplicate observation"), Transport->ProcessRequestBytes(RequestBytes, Duplicate));
	TestEqual(TEXT("duplicate returns the exact retained coherent sample"), Duplicate, First);
	Server->Stop();
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabTypedDispatchCompletionSurvivesClosure,
	"URLab.TypedDispatch.AcceptedCompletionSurvivesRuntimeDetach",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FURLabTypedDispatchCompletionSurvivesClosure::RunTest(const FString&)
{
	UURLabBridgeServer* Server = NewObject<UURLabBridgeServer>();
	Server->Start(TEXT(""));
	const TSharedRef<FMjSimulationRuntime, ESPMode::ThreadSafe> Runtime =
		MakeShared<FMjSimulationRuntime, ESPMode::ThreadSafe>();
	FString Error;
	if (!TestTrue(TEXT("independent runtime fixture"), InstallRuntime(Runtime, Error)))
	{
		AddError(Error);
		Server->Stop();
		return false;
	}
	FURLabRpcDispatcher* Dispatcher = Server->GetDispatcher();
	FURLabRpcDispatcher::FSimulationRuntimeRegistration Registration;
	if (!TestTrue(TEXT("attach independent runtime"),
		Dispatcher->AttachSimulationRuntime(Runtime, Registration, Error)))
	{
		AddError(Error);
		Server->Stop();
		return false;
	}
	const TSharedRef<FURLabRequestLedger, ESPMode::ThreadSafe> Ledger = Dispatcher->GetRequestLedgerForTest();
	UURLabZmqRpcTransport* Transport = NewObject<UURLabZmqRpcTransport>();
	Transport->SetOwningBridge(Server);
	FURLabConnectReply Connection;
	if (!TestTrue(TEXT("connect"), Connect(Transport, Connection))) return false;
	FURLabRequestEnvelope Request = RequestFor(Connection, 81, TEXT("simulation_observe"));
	TArray<uint8> RequestBytes;
	if (!TestTrue(TEXT("encode"), FURLabRequestEnvelopeCodec::Encode(Request, RequestBytes).IsOk())) return false;

	TFuture<TArray<uint8>> Attempt = Async(EAsyncExecution::ThreadPool, [Dispatcher, RequestBytes]()
	{
		TArray<uint8> Reply;
		Dispatcher->DispatchBytes(RequestBytes, TEXT("zmq"), Reply);
		return Reply;
	});
	const TArray<uint8> PendingBytes = Attempt.Get();
	FURLabReplyEnvelope Pending;
	if (!TestTrue(TEXT("pending decodes"), FURLabRequestEnvelopeCodec::DecodeReply(PendingBytes, Pending).IsOk()) ||
		!TestEqual(TEXT("worker request returns pending"), Pending.Kind, EURLabReplyKind::Pending)) return false;
	TestEqual(TEXT("accepted request is globally pending"), Ledger->GetStats().Pending, 1);

	TestTrue(TEXT("exact registration detaches"),
		Dispatcher->DetachSimulationRuntime(Registration));
	FTaskGraphInterface::Get().ProcessThreadUntilIdle(ENamedThreads::GameThread);
	TestEqual(TEXT("captured runtime completion releases pending credit"), Ledger->GetStats().Pending, 0);
	TArray<uint8> RetainedBytes;
	TestTrue(TEXT("completed request remains queryable after detach"),
		Transport->ProcessRequestBytes(RequestBytes, RetainedBytes));
	FURLabReplyEnvelope Retained;
	if (!TestTrue(TEXT("retained completion decodes"),
		FURLabRequestEnvelopeCodec::DecodeReply(RetainedBytes, Retained).IsOk()))
	{
		Server->Stop();
		return false;
	}
	TestEqual(TEXT("accepted work completed against captured runtime"),
		Retained.Kind, EURLabReplyKind::Result);

	FURLabRequestEnvelope AfterDetach = RequestFor(Connection, 82, TEXT("simulation_observe"));
	TArray<uint8> AfterDetachRequest, AfterDetachReply;
	FURLabRequestEnvelopeCodec::Encode(AfterDetach, AfterDetachRequest);
	TestTrue(TEXT("new read is answered after detach"),
		Transport->ProcessRequestBytes(AfterDetachRequest, AfterDetachReply));
	FURLabReplyEnvelope Unavailable;
	TestTrue(TEXT("post-detach reply decodes"),
		FURLabRequestEnvelopeCodec::DecodeReply(AfterDetachReply, Unavailable).IsOk());
	TestEqual(TEXT("new work cannot reuse detached runtime"),
		Unavailable.Error.Code, FString(TEXT("service_unavailable")));
	Server->Stop();
	return true;
}

#endif
