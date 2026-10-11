#include "Bridge/ViewerProducerSession.h"
#include "Transport/RpcClientTransport.h"
#include "Transport/ShmExactPayload.h"

namespace
{
const TCHAR* PolicyToken(EURLabViewerProducerDataPolicy Policy)
{
	switch (Policy) { case EURLabViewerProducerDataPolicy::Auto:return TEXT("auto");
	case EURLabViewerProducerDataPolicy::Shm:return TEXT("shm"); default:return TEXT("network"); }
}
}

bool UURLabViewerProducerSession::ValidateCommandUri(const FString& Uri, FString& OutError)
{
	OutError.Reset();
	if (!Uri.StartsWith(TEXT("tcp://"), ESearchCase::CaseSensitive))
	{ OutError=TEXT("producer command_uri must use the exact tcp:// scheme"); return false; }
	const FString Authority=Uri.Mid(6);
	if (Authority.IsEmpty() || Authority.Contains(TEXT("/")) || Authority.Contains(TEXT("?"))
		|| Authority.Contains(TEXT("#")) || Authority.Contains(TEXT("@")))
	{ OutError=TEXT("producer command_uri must contain only a host and port"); return false; }
	FString Host,PortText;
	if (Authority.StartsWith(TEXT("[")))
	{
		int32 End=INDEX_NONE; if(!Authority.FindChar(TEXT(']'),End) || End<=1
			|| End+2>=Authority.Len() || Authority[End+1]!=TEXT(':'))
		{OutError=TEXT("producer command_uri has an invalid bracketed host");return false;}
		Host=Authority.Mid(1,End-1); PortText=Authority.Mid(End+2);
	}
	else
	{
		int32 Colon=INDEX_NONE;if(!Authority.FindLastChar(TEXT(':'),Colon)||Colon<=0
			|| Authority.Left(Colon).Contains(TEXT(":")))
		{OutError=TEXT("producer command_uri must include one host and port");return false;}
		Host=Authority.Left(Colon);PortText=Authority.Mid(Colon+1);
	}
	if(Host.IsEmpty()||PortText.IsEmpty()) {OutError=TEXT("producer command_uri host or port is empty");return false;}
	for(TCHAR C:PortText) if(!FChar::IsDigit(C)){OutError=TEXT("producer command_uri port is not numeric");return false;}
	const int64 Port=FCString::Atoi64(*PortText);if(Port<1||Port>65535){OutError=TEXT("producer command_uri port is out of range");return false;}
	return true;
}

bool UURLabViewerProducerSession::RoundTrip(const TArray<uint8>& Bytes,int32 TimeoutMs,
	TArray<uint8>& Out,FString& OutError)
{
	if(!Transport || TimeoutMs<=0 || !Transport->Request(Bytes,Out,TimeoutMs))
	{OutError=TEXT("producer command round trip timed out or failed");return false;}
	return true;
}

bool UURLabViewerProducerSession::Initialize(const FURLabViewerProducerConnection& Connection,FString& OutError)
{
	check(IsInGameThread()); Reset();
	if(!ValidateCommandUri(Connection.CommandUri,OutError))return false;
	Transport=UURLabRpcClientTransport::Create(this,Connection.CommandUri);
	if(!Transport){OutError=TEXT("producer command transport could not connect");return false;}
	PendingPolicy=Connection.DataPolicy; OutError.Reset(); return true;
}

bool UURLabViewerProducerSession::Connect(const FURLabViewerProducerConnection& Connection,int32 TimeoutMs,FString& OutError)
{
	return Initialize(Connection,OutError) && CompleteConnect(TimeoutMs,OutError);
}

bool UURLabViewerProducerSession::CompleteConnect(int32 TimeoutMs,FString& OutError)
{
	if(!Transport){OutError=TEXT("producer command transport is not initialized");return false;}
	const auto ConnectionPolicy=PendingPolicy;
	FURLabConnectRequest ConnectRequest;ConnectRequest.ProtocolVersion=2;ConnectRequest.DataPolicy=PolicyToken(ConnectionPolicy);
	TArray<uint8> Encoded,ReplyBytes;
	FURLabWireStatus Status=FURLabRequestEnvelopeCodec::EncodeConnectRequest(ConnectRequest,Encoded);
	if(!Status||!RoundTrip(Encoded,TimeoutMs,ReplyBytes,OutError)){Reset();return false;}
	FURLabConnectReply Reply;Status=FURLabRequestEnvelopeCodec::DecodeConnectReply(ReplyBytes,Reply);
	if(!Status){FURLabProtocolError Protocol;if(FURLabRequestEnvelopeCodec::DecodeProtocolError(ReplyBytes,Protocol))OutError=Protocol.Code+TEXT(": ")+Protocol.Message;else OutError=Status.Detail;Reset();return false;}
	if(Reply.ProtocolVersion!=2||Reply.BootId.IsEmpty()||Reply.SessionId.IsEmpty())
	{OutError=TEXT("producer returned an invalid protocol-v2 identity");Reset();return false;}
	const FString Transfer=Reply.DataNegotiation.Transfer;
	if(ConnectionPolicy==EURLabViewerProducerDataPolicy::Network && Transfer!=TEXT("network"))
	{OutError=TEXT("producer did not select required network data");Reset();return false;}
	ConnectReply=Reply; BootId=Reply.BootId; SessionId=Reply.SessionId; NextRequestId=1;
	if(Transfer==TEXT("network")){SelectedTransfer=TEXT("network");OutError.Reset();return true;}
	if(Transfer!=TEXT("shm") || Reply.DataNegotiation.RegionId.IsEmpty()
		|| Reply.DataNegotiation.RegionGeneration==0 || Reply.DataNegotiation.ChallengeName.IsEmpty())
	{OutError=TEXT("producer returned invalid provisional SHM negotiation");Reset();return false;}
	const FURLabWireValue* MaxValue=Reply.Limits.FindMapValue(TEXT("payload_max_blob_bytes"));
	const int64 MaxBlob=MaxValue?MaxValue->GetInt():0; TSharedPtr<FURLabShmRegionAccess,ESPMode::ThreadSafe> Access;
	auto OpenStatus=FURLabShmRegionAccess::Open(Reply.DataNegotiation.DirectoryPath,Reply.BootId,
		Reply.DataNegotiation.RegionId,Reply.DataNegotiation.RegionGeneration,MaxBlob,Access);
	TArray<uint8> Nonce; if(OpenStatus)OpenStatus=Access->ReadChallenge(Reply.DataNegotiation.ChallengeName,
		Reply.DataNegotiation.ChallengeByteLength,Nonce);
	auto Select=[&](FURLabWireValue Selection,FURLabReplyEnvelope& Out)->bool{FString E;if(!Request(TEXT("data_select"),MoveTemp(Selection),TimeoutMs,Out,E)||Out.Kind!=EURLabReplyKind::Result){OutError=E.IsEmpty()?Out.Error.Code+TEXT(": ")+Out.Error.Message:E;return false;}return true;};
	if(!OpenStatus)
	{
		if(ConnectionPolicy!=EURLabViewerProducerDataPolicy::Auto){OutError=OpenStatus.Message;Reset();return false;}
		FURLabReplyEnvelope Selected;if(!Select(FURLabWireValue::MakeMap({TEXT("transfer")},{FURLabWireValue::MakeString(TEXT("network"))}),Selected)){Reset();return false;}
		SelectedTransfer=TEXT("network");OutError.Reset();return true;
	}
	FURLabReplyEnvelope Selected;
	FURLabWireValue Selection=FURLabWireValue::MakeMap(
		{TEXT("transfer"),TEXT("region_id"),TEXT("region_generation"),TEXT("challenge_name"),TEXT("challenge_nonce")},
		{FURLabWireValue::MakeString(TEXT("shm")),FURLabWireValue::MakeString(Reply.DataNegotiation.RegionId),FURLabWireValue::MakeInt(Reply.DataNegotiation.RegionGeneration),FURLabWireValue::MakeString(Reply.DataNegotiation.ChallengeName),FURLabWireValue::MakeBinary(MoveTemp(Nonce))});
	if(!Select(MoveTemp(Selection),Selected)){Reset();return false;}
	const auto* SelectedTransferValue=Selected.Result.FindMapValue(TEXT("transfer"));
	const auto* SelectedRegion=Selected.Result.FindMapValue(TEXT("region_id"));
	const auto* SelectedGeneration=Selected.Result.FindMapValue(TEXT("region_generation"));
	if(!SelectedTransferValue||SelectedTransferValue->GetString()!=TEXT("shm")||!SelectedRegion
		||SelectedRegion->GetString()!=Reply.DataNegotiation.RegionId||!SelectedGeneration
		||static_cast<uint64>(SelectedGeneration->GetInt())!=Reply.DataNegotiation.RegionGeneration)
	{OutError=TEXT("producer selected a different SHM region identity");Reset();return false;}
	ShmAccess=MoveTemp(Access);SelectedTransfer=TEXT("shm");OutError.Reset();return true;
}

bool UURLabViewerProducerSession::Request(const FString& Op,FURLabWireValue Body,int32 TimeoutMs,
	FURLabReplyEnvelope& OutReply,FString& OutError)
{
	FURLabViewerProducerRequest RequestState;
	if(!BeginRequest(Op,MoveTemp(Body),RequestState,OutError))return false;
	for(int32 Attempt=0;Attempt<64;++Attempt)
	{
		const auto State=PollRequest(RequestState,TimeoutMs,OutReply,OutError);
		if(State==EURLabViewerProducerRequestState::Terminal)return true;
		if(State==EURLabViewerProducerRequestState::Failed)return false;
	}
	OutError=TEXT("producer request remained pending after 64 exact-ticket polls");return false;
}

bool UURLabViewerProducerSession::BeginRequest(const FString& Op,FURLabWireValue Body,
	FURLabViewerProducerRequest& OutRequest,FString& OutError)
{
	if(!IsConnected()||Op.IsEmpty()||NextRequestId==0){OutError=TEXT("producer session is unavailable");return false;}
	FURLabRequestEnvelope R;R.ProtocolVersion=2;R.BootId=BootId;R.SessionId=SessionId;
	R.RequestId=NextRequestId++;R.Op=Op;R.Body=MoveTemp(Body);
	OutRequest={};OutRequest.Op=Op;OutRequest.RequestId=R.RequestId;
	const auto Status=FURLabRequestEnvelopeCodec::Encode(R,OutRequest.Bytes);
	if(!Status){OutError=Status.Detail;OutRequest={};return false;}
	OutError.Reset();return true;
}

EURLabViewerProducerRequestState UURLabViewerProducerSession::PollRequest(
	FURLabViewerProducerRequest& RequestState,int32 TimeoutMs,FURLabReplyEnvelope& OutReply,
	FString& OutError)
{
	if(RequestState.RequestId==0||RequestState.Op.IsEmpty()||RequestState.Bytes.IsEmpty()
		||RequestState.Attempts>=64){OutError=TEXT("producer request ticket is invalid or exhausted");return EURLabViewerProducerRequestState::Failed;}
	++RequestState.Attempts;TArray<uint8> ReplyBytes;
	if(!RoundTrip(RequestState.Bytes,TimeoutMs,ReplyBytes,OutError))return EURLabViewerProducerRequestState::Failed;
	FURLabReplyEnvelope Reply;const auto Status=FURLabRequestEnvelopeCodec::DecodeReply(ReplyBytes,Reply);
	if(!Status){OutError=Status.Detail;return EURLabViewerProducerRequestState::Failed;}
	if(Reply.RequestId!=RequestState.RequestId||Reply.Op!=RequestState.Op){OutError=TEXT("producer reply ticket does not match the request");return EURLabViewerProducerRequestState::Failed;}
	if(Reply.Kind==EURLabReplyKind::Pending){OutError.Reset();return EURLabViewerProducerRequestState::Pending;}
	OutReply=MoveTemp(Reply);OutError.Reset();return EURLabViewerProducerRequestState::Terminal;
}

bool UURLabViewerProducerSession::Disconnect(int32 TimeoutMs,FString& OutError)
{
	if(!IsConnected()){Reset();OutError.Reset();return true;}FURLabDisconnectRequest R{2,BootId,SessionId};TArray<uint8>B,O;auto S=FURLabRequestEnvelopeCodec::EncodeDisconnectRequest(R,B);if(!S||!RoundTrip(B,TimeoutMs,O,OutError)){Reset();return false;}FURLabDisconnectReply Reply;S=FURLabRequestEnvelopeCodec::DecodeDisconnectReply(O,Reply);const bool Ok=S&&Reply.bDisconnected&&Reply.BootId==BootId&&Reply.SessionId==SessionId;if(!Ok)OutError=S?TEXT("producer disconnect identity mismatch"):S.Detail;Reset();return Ok;
}
void UURLabViewerProducerSession::Reset(){ShmAccess.Reset();if(Transport)Transport->TransportShutdown();Transport=nullptr;ConnectReply={};BootId.Reset();SessionId.Reset();SelectedTransfer.Reset();NextRequestId=1;}
void UURLabViewerProducerSession::Abort(){Reset();}
void UURLabViewerProducerSession::BeginDestroy(){Reset();Super::BeginDestroy();}
