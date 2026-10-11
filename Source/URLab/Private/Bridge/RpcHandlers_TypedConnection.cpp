// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "Bridge/RpcDispatcher.h"
#include "Bridge/BridgeServer.h"
#include "Bridge/BridgeServerConfig.h"
#include "Wire/OperationSchema.h"
#include "Wire/UrlabWire.gen.h"

namespace
{
using FConnectionValue = FURLabWireValue;

FConnectionValue ConnectionMap(std::initializer_list<TPair<FString, FConnectionValue>> Fields)
{
	TArray<FString> Keys;
	TArray<FConnectionValue> Values;
	for (const TPair<FString, FConnectionValue>& Field : Fields)
	{
		Keys.Add(Field.Key);
		Values.Add(Field.Value);
	}
	return FConnectionValue::MakeMap(MoveTemp(Keys), MoveTemp(Values));
}
}

bool FURLabRpcDispatcher::DispatchBytes(TConstArrayView<uint8> RequestBytes,
	const FString& Adapter, TArray<uint8>& OutReplyBytes, const FString* OuterOp)
{
	static_assert(URLabWire::Limits::MaxSessions == kMaxActiveSessions);
	static_assert(URLabWire::Limits::SessionIdleTimeoutS == kSessionIdleTimeoutSeconds);
	static_assert(URLabWire::Limits::DataNegotiationTimeoutS == 30.0);
	OutReplyBytes.Reset();
	FURLabWireValue Root;
	FURLabWireStatus Status = FURLabRequestEnvelopeCodec::DecodeValue(RequestBytes, Root);
	auto ProtocolError = [&OutReplyBytes](const FString& Code, const FString& Message)
	{
		return FURLabRequestEnvelopeCodec::EncodeProtocolError({Code, Message}, OutReplyBytes).IsOk();
	};
	if (!Status || Root.GetType() != EURLabWireValueType::Map)
		return ProtocolError(TEXT("bad_request"), Status.Detail.IsEmpty()
			? TEXT("Request must be one MessagePack map") : Status.Detail);
	const FURLabWireValue* OpValue = Root.FindMapValue(TEXT("op"));
	if (!OpValue || OpValue->GetType() != EURLabWireValueType::String)
		return ProtocolError(TEXT("bad_request"), TEXT("Request op must be a string"));
	const FString Op = OpValue->GetString();
	if (OuterOp && *OuterOp != Op)
		return ProtocolError(TEXT("bad_request"), TEXT("Outer transport op does not match request op"));
	if (Adapter != TEXT("zmq") && Adapter != TEXT("grpc"))
		return ProtocolError(TEXT("wrong_transport"), TEXT("Protocol-v2 commands require ZMQ or gRPC"));

	if (Op == TEXT("connect"))
	{
		if (IsDraining())
			return ProtocolError(TEXT("server_draining"), TEXT("Bridge server is shutting down"));
		FURLabConnectRequest Request;
		Status = FURLabRequestEnvelopeCodec::DecodeConnectRequest(RequestBytes, Request);
		if (!Status) return ProtocolError(TEXT("bad_request"), Status.Detail);
		UURLabBridgeServer* Bridge = OwningBridge.Get();
		if (!Bridge) return ProtocolError(TEXT("not_ready"), TEXT("Bridge server is unavailable"));
		EURLabDataPolicy DataPolicy = EURLabDataPolicy::Network;
		if (Request.DataPolicy == TEXT("auto")) DataPolicy = EURLabDataPolicy::Auto;
		else if (Request.DataPolicy == TEXT("shm")) DataPolicy = EURLabDataPolicy::Shm;
		FString SessionId, RegisterErrorCode, RegisterError;
		FURLabDataSessionSnapshot Data;
		if (!RegisterNewSession(Adapter, DataPolicy, SessionId, Data,
			RegisterErrorCode, RegisterError))
		{
			return ProtocolError(RegisterErrorCode.IsEmpty() ? TEXT("not_ready") : RegisterErrorCode,
				RegisterError.IsEmpty() ? TEXT("Session preparation failed") : RegisterError);
		}
		const FURLabBridgeServerConfig& Config = Bridge->ResolveInstanceConfig();
		const FURLabPayloadStoreLimits& PayloadLimits = PayloadStore->GetLimits();
		FURLabConnectReply Reply;
		Reply.ProtocolVersion = 2;
		Reply.InstanceId = Config.InstanceId.IsEmpty() ? TEXT("live") : Config.InstanceId;
		Reply.BootId = BootId;
		Reply.SessionId = SessionId;
		Reply.URLabVersion = URLabVersion;
		Bridge->CollectCommandEndpoints(Reply.CommandEndpoints);
		if (HasSimulationRuntime() && URLabWire::OpAdapter::Allows(TEXT("simulation_describe"), Adapter))
			Reply.Services.Add(TEXT("simulation"));
		if (HasRenderService() && URLabWire::OpAdapter::Allows(TEXT("render_owner_status"), Adapter))
			Reply.Services.Add(TEXT("render"));
		if (HasViewerService() && URLabWire::OpAdapter::Allows(TEXT("viewer_owner_status"), Adapter))
			Reply.Services.Add(TEXT("viewer"));
		if (HasPoseStreamService() && URLabWire::OpAdapter::Allows(TEXT("pose_stream_describe"), Adapter))
			Reply.Services.Add(TEXT("streams"));
		if (HasEditorService() && URLabWire::OpAdapter::Allows(TEXT("editor_current_level"), Adapter))
			Reply.Services.Add(TEXT("editor"));
		if (URLabWire::OpAdapter::Allows(TEXT("payload_begin"), Adapter))
			Reply.Services.Add(TEXT("data"));
		Bridge->CollectGrantTokens(Reply.Permissions);
		Reply.DataOffers.Add(TEXT("network"));
		if (Adapter == TEXT("zmq") && ShmRegion)
		{
			Reply.DataOffers.Add(TEXT("shm"));
		}
		if (Data.Transfer == EURLabDataTransfer::Shm
			|| Data.Transfer == EURLabDataTransfer::Provisional)
		{
			Reply.DataNegotiation.Transfer = TEXT("shm");
			Reply.DataNegotiation.RegionId = Data.Region.RegionId;
			Reply.DataNegotiation.RegionGeneration = Data.Region.Generation;
			Reply.DataNegotiation.DirectoryPath = Data.Region.DirectoryPath;
			Reply.DataNegotiation.ChallengeName = Data.Challenge.ChallengeName;
			Reply.DataNegotiation.ChallengeByteLength = Data.Challenge.ByteLength;
			Reply.DataNegotiation.ExpiresInSeconds = Data.ExpiresInSeconds;
		}
		else
		{
			Reply.DataNegotiation.Transfer = TEXT("network");
		}
		Reply.Limits = ConnectionMap({
			{TEXT("max_sessions"), FConnectionValue::MakeInt(kMaxActiveSessions)},
			{TEXT("session_idle_timeout_s"), FConnectionValue::MakeFloat(kSessionIdleTimeoutSeconds)},
			{TEXT("request_window"), FConnectionValue::MakeInt(FURLabRequestLedger::WindowWidth)},
			{TEXT("max_session_pending"), FConnectionValue::MakeInt(FURLabRequestLedger::MaxPendingPerSession)},
			{TEXT("max_global_pending"), FConnectionValue::MakeInt(FURLabRequestLedger::MaxPendingGlobal)},
			{TEXT("max_inline_reply_bytes"), FConnectionValue::MakeInt(FURLabRequestLedger::MaxInlineReplyBytes)},
			{TEXT("data_negotiation_timeout_s"), FConnectionValue::MakeFloat(
				URLabWire::Limits::DataNegotiationTimeoutS)},
			{URLabWire::ConfigurableLimit::PayloadMaxBlobBytes, FConnectionValue::MakeInt(PayloadLimits.MaxBlobBytes)},
			{URLabWire::ConfigurableLimit::PayloadMaxChunkBytes, FConnectionValue::MakeInt(PayloadLimits.MaxChunkBytes)},
			{URLabWire::ConfigurableLimit::PayloadMaxChunksPerBlob, FConnectionValue::MakeInt(PayloadLimits.MaxChunksPerBlob)},
			{URLabWire::ConfigurableLimit::PayloadMaxSessionBytes, FConnectionValue::MakeInt(PayloadLimits.MaxSessionBytes)},
			{URLabWire::ConfigurableLimit::PayloadMaxGlobalBytes, FConnectionValue::MakeInt(PayloadLimits.MaxGlobalBytes)},
			{URLabWire::ConfigurableLimit::PayloadMaxBlobCount, FConnectionValue::MakeInt(PayloadLimits.MaxBlobCount)},
			{URLabWire::ConfigurableLimit::PayloadMaxSessionBlobCount, FConnectionValue::MakeInt(PayloadLimits.MaxSessionBlobCount)},
			{URLabWire::ConfigurableLimit::PayloadMaxSessionPhysicalObjectCount, FConnectionValue::MakeInt(PayloadLimits.MaxSessionPhysicalObjectCount)},
			{URLabWire::ConfigurableLimit::PayloadMaxActiveSessions, FConnectionValue::MakeInt(PayloadLimits.MaxActiveSessions)},
			{URLabWire::ConfigurableLimit::PayloadStagingTtlS, FConnectionValue::MakeFloat(PayloadLimits.StagingTtlSeconds)},
			{URLabWire::ConfigurableLimit::PayloadSealedTtlS, FConnectionValue::MakeFloat(PayloadLimits.SealedTtlSeconds)},
			{URLabWire::ConfigurableLimit::PayloadExpiredTombstoneTtlS, FConnectionValue::MakeFloat(PayloadLimits.ExpiredTombstoneTtlSeconds)}});
		Status = FURLabRequestEnvelopeCodec::EncodeConnectReply(Reply, OutReplyBytes);
		if (!Status) RemoveSession(SessionId, Adapter);
		return Status.IsOk();
	}

	if (Op == TEXT("disconnect"))
	{
		FURLabDisconnectRequest Request;
		Status = FURLabRequestEnvelopeCodec::DecodeDisconnectRequest(RequestBytes, Request);
		if (!Status) return ProtocolError(TEXT("bad_request"), Status.Detail);
		if (Request.BootId != BootId || !RemoveSession(Request.SessionId, Adapter))
			return ProtocolError(TEXT("session_expired"), TEXT("Session is not active on this boot"));
		FURLabDisconnectReply Reply;
		Reply.ProtocolVersion = 2;
		Reply.BootId = BootId;
		Reply.SessionId = Request.SessionId;
		Reply.bDisconnected = true;
		return FURLabRequestEnvelopeCodec::EncodeDisconnectReply(Reply, OutReplyBytes).IsOk();
	}

	FURLabRequestEnvelope Request;
	Status = FURLabRequestEnvelopeCodec::Decode(RequestBytes, Request);
	if (!Status) return ProtocolError(TEXT("bad_request"), Status.Detail);
	auto ReplyError = [&Request, &OutReplyBytes](const TCHAR* Code, const FString& Message,
		EURLabExecution Execution)
	{
		return FURLabRequestEnvelopeCodec::EncodeReply(
			FURLabReplyEnvelope::MakeError(Request, Code, Message, Execution), OutReplyBytes).IsOk();
	};
	if (IsDraining())
		return ReplyError(TEXT("server_draining"), TEXT("Bridge server is shutting down"), EURLabExecution::Unknown);
	if (Request.BootId != BootId)
		return ReplyError(TEXT("outcome_unknown"), TEXT("Request boot does not match this process"), EURLabExecution::Unknown);
	TSharedPtr<FURLabRequestLedger::FSession, ESPMode::ThreadSafe> Session;
	TSharedPtr<FMjSimulationControlSessionLifetime, ESPMode::ThreadSafe> ControlSession;
	FURLabRenderOwnerSessionScopePtr RenderOwnerScope;
	FURLabRenderOwnerSessionScopePtr ViewerOwnerScope;
	FURLabPayloadSessionPtr PayloadSession;
	FURLabDataSessionPtr DataSession;
	if (!FindAndTouchSession(Request.SessionId, Adapter, Session, ControlSession, RenderOwnerScope, ViewerOwnerScope,
		PayloadSession, DataSession))
		return ReplyError(TEXT("outcome_unknown"), TEXT("Session is unavailable on this command adapter"), EURLabExecution::Unknown);
	if (Request.Op != TEXT("data_select") && !DataSession->IsSelected())
		return ReplyError(TEXT("data_negotiation_required"),
			TEXT("Complete data-plane selection before issuing commands"),
			EURLabExecution::Rejected);
	const URLabWire::TypedOp::FDecl* Decl = URLabWire::TypedOp::Find(Request.Op);
	if (!Decl)
		return ReplyError(TEXT("unknown_op"), TEXT("Operation is not active on the typed command path"), EURLabExecution::Rejected);
	if (!URLabWire::OpAdapter::Allows(Request.Op, Adapter))
		return ReplyError(TEXT("wrong_transport"), TEXT("Operation is unavailable on this command adapter"), EURLabExecution::Rejected);
	Status = FURLabOperationSchema::ValidateRequestBody(Request.Op, Request.Body);
	if (!Status) return ReplyError(TEXT("bad_request"), Status.Detail, EURLabExecution::Rejected);

	const FURLabPayloadStoreLimits& PayloadLimits = PayloadStore->GetLimits();
	if (Request.Op == TEXT("payload_begin")
		&& Request.Body.FindMapValue(TEXT("byte_length"))->GetInt() > PayloadLimits.MaxBlobBytes)
		return ReplyError(TEXT("payload_too_large"), TEXT("Declared payload exceeds max blob bytes"), EURLabExecution::Rejected);
	if (Request.Op == TEXT("payload_write"))
	{
		const int32 Bytes = Request.Body.FindMapValue(TEXT("data"))->GetBinary().Num();
		if (Bytes <= 0) return ReplyError(TEXT("bad_request"), TEXT("Payload chunk must not be empty"), EURLabExecution::Rejected);
		if (Bytes > PayloadLimits.MaxChunkBytes)
			return ReplyError(TEXT("payload_chunk_too_large"), TEXT("Payload chunk exceeds max chunk bytes"), EURLabExecution::Rejected);
	}
	if (Request.Op == TEXT("payload_read")
		&& Request.Body.FindMapValue(TEXT("byte_length"))->GetInt() > PayloadLimits.MaxChunkBytes)
		return ReplyError(TEXT("payload_chunk_too_large"), TEXT("Requested payload range exceeds max chunk bytes"), EURLabExecution::Rejected);

	FURLabRequestDigest Digest;
	Status = FURLabRequestEnvelopeCodec::Digest(Request.Op, Request.Body, Digest);
	if (!Status) return ReplyError(TEXT("bad_request"), Status.Detail, EURLabExecution::Rejected);
	const FString Service = Decl->Service ? Decl->Service : TEXT("");
	const FString Execution = Decl->Execution ? Decl->Execution : TEXT("");
	if (Service == TEXT("simulation") && Execution == TEXT("simulation_queue"))
		return DispatchTypedSimulation(Request, Digest, Session.ToSharedRef(), ControlSession.ToSharedRef(), OutReplyBytes);
	if (Service == TEXT("data") && Execution == TEXT("payload_queue"))
		return DispatchTypedPayload(Request, Digest, Session.ToSharedRef(), DataSession, OutReplyBytes);
	if (Service == TEXT("render") && Execution == TEXT("render_queue"))
		return DispatchTypedRender(Request, Digest, Session.ToSharedRef(), RenderOwnerScope,
			DataSession, OutReplyBytes);
	if (Service == TEXT("viewer") && Execution == TEXT("render_queue"))
		return DispatchTypedViewer(Request, Digest, Session.ToSharedRef(), ViewerOwnerScope,
			DataSession, OutReplyBytes);
	if (Service == TEXT("streams")
		&& (Execution == TEXT("render_queue") || Execution == TEXT("simulation_queue")))
		return DispatchTypedStreams(Request, Digest, Session.ToSharedRef(), DataSession,
			OutReplyBytes);
	if (Service == TEXT("editor") && Execution == TEXT("editor_queue"))
	{
		if (Decl->Permission && FCString::Strcmp(Decl->Permission, TEXT("accept_editor_commands")) == 0
			&& !GrantsPermission(EMjCapability::AcceptEditorCommands))
			return ReplyError(TEXT("capability_disabled"),
				TEXT("Editor mutation requires accept_editor_commands"), EURLabExecution::Rejected);
		return DispatchTypedEditor(Request, Digest, Session.ToSharedRef(), OutReplyBytes);
	}
	return ReplyError(TEXT("server_schema_error"),
		TEXT("Active operation has unsupported service or execution metadata"), EURLabExecution::Rejected);
}
