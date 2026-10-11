#include "Bridge/ViewerStreamConnection.h"
#include "Workflows/ViewerWorldSubsystem.h"
#include "Transport/LockedLatestFrame.h"
#include "Transport/PoseStreamZmqReader.h"
#include "Transport/ShmExactPayload.h"
#include "Wire/RequestEnvelope.h"
#include "Wire/OperationSchema.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"

namespace { using V=FURLabWireValue;
bool ParseDescriptor(const V& S,FURLabViewerStreamDescriptor& D){const V* K=S.FindMapValue(TEXT("kind"));const V* Vsn=S.FindMapValue(TEXT("schema_version"));const V* B=S.FindMapValue(TEXT("producer_boot_id"));const V* I=S.FindMapValue(TEXT("stream_id"));const V* G=S.FindMapValue(TEXT("stream_generation"));const V* R=S.FindMapValue(TEXT("source_revision"));const V* F=S.FindMapValue(TEXT("payload_format"));if(!K||!Vsn||!B||!I||!G||!R||!F||K->GetString()!=TEXT("cartesian_pose"))return false;D.SchemaVersion=Vsn->GetInt();D.ProducerBootId=B->GetString();D.StreamId=I->GetString();D.StreamGeneration=G->GetInt();D.SourceRevision=R->GetString();D.PayloadFormat=F->GetString();FString E;return D.IsValid(E);}
bool Same(const FURLabViewerStreamDescriptor&A,const FURLabViewerStreamDescriptor&B){return A.SchemaVersion==B.SchemaVersion&&A.ProducerBootId==B.ProducerBootId&&A.StreamId==B.StreamId&&A.StreamGeneration==B.StreamGeneration&&A.SourceRevision==B.SourceRevision&&A.PayloadFormat==B.PayloadFormat;}}
FURLabViewerStreamConnection::FURLabViewerStreamConnection(TStrongObjectPtr<UURLabViewerProducerSession>&& S):Session(MoveTemp(S)){}
FURLabViewerStreamConnection::~FURLabViewerStreamConnection(){Shutdown();}
TSharedPtr<FURLabViewerStreamConnection,ESPMode::ThreadSafe> FURLabViewerStreamConnection::Create(UObject* Outer,const FURLabViewerProducerConnection&C,FString&Error){check(IsInGameThread());TStrongObjectPtr<UURLabViewerProducerSession>S(NewObject<UURLabViewerProducerSession>(Outer));if(!S||!S->Initialize(C,Error))return nullptr;return MakeShareable(new FURLabViewerStreamConnection(MoveTemp(S)));}
void FURLabViewerStreamConnection::StoreLatest(TArray<uint8>&&B){FScopeLock L(&InboxMutex);Latest=MoveTemp(B);}
bool FURLabViewerStreamConnection::PopLatest(TArray<uint8>&Out){FScopeLock L(&InboxMutex);if(Latest.IsEmpty())return false;Out=MoveTemp(Latest);Latest.Reset();return true;}
bool FURLabViewerStreamConnection::Prepare(const FURLabViewerStreamDescriptor&Expected,int32 TimeoutMs,FString&Error)
{
	if(!Session||!Session->CompleteConnect(TimeoutMs,Error))return false;FURLabReplyEnvelope Reply;if(!Session->Request(TEXT("pose_stream_describe"),V::MakeMap({},{}),TimeoutMs,Reply,Error)||Reply.Kind!=EURLabReplyKind::Result){if(Error.IsEmpty())Error=Reply.Error.Code+TEXT(": ")+Reply.Error.Message;return false;}
	const FURLabWireStatus Shape=FURLabOperationSchema::ValidateResultBody(TEXT("pose_stream_describe"),Reply.Result);if(!Shape){Error=Shape.Detail;return false;}
	const V* Stream=Reply.Result.FindMapValue(TEXT("stream"));const V* Deliveries=Reply.Result.FindMapValue(TEXT("deliveries"));FURLabViewerStreamDescriptor Actual;if(!Stream||!Deliveries||!ParseDescriptor(*Stream,Actual)||!Same(Expected,Actual)||Deliveries->GetArray().IsEmpty()){Error=TEXT("producer describe identity or deliveries do not match the requested stream");return false;}ExpectedDescriptor=MakeUnique<FURLabViewerStreamDescriptor>(Expected);
	const FString Transfer=Session->GetSelectedTransfer();for(const V&D:Deliveries->GetArray()){const V* Kind=D.FindMapValue(TEXT("kind"));if(!Kind)continue;if(Transfer==TEXT("network")&&Kind->GetString()==TEXT("zmq")){const V* Endpoint=D.FindMapValue(TEXT("endpoint"));const V* Topic=D.FindMapValue(TEXT("topic"));if(!Endpoint||!Topic)continue;const V* Max=Session->GetConnectReply().Limits.FindMapValue(TEXT("payload_max_blob_bytes"));const int64 MaxBytes=Max?Max->GetInt():0;TWeakPtr<FURLabViewerStreamConnection,ESPMode::ThreadSafe> Weak=AsShared();ZmqReader=FURLabPoseStreamZmqReader::Create(Endpoint->GetString(),Topic->GetBinary(),MaxBytes,[Weak](TArray<uint8>&&B){if(auto P=Weak.Pin())P->StoreLatest(MoveTemp(B));},Error);if(ZmqReader)return StartCommandWorker(Error);}
		if(Transfer==TEXT("shm")&&Kind->GetString()==TEXT("shm_latest")){const V* Region=D.FindMapValue(TEXT("region_id"));const V* Generation=D.FindMapValue(TEXT("region_generation"));const V* Object=D.FindMapValue(TEXT("object_name"));const V* Layout=D.FindMapValue(TEXT("layout_version"));const V* Max=D.FindMapValue(TEXT("max_payload_bytes"));if(!Region||!Generation||!Object||!Layout||!Max||Layout->GetInt()!=1||Max->GetInt()<=0||Max->GetInt()>MAX_uint32)continue;auto Access=Session->GetShmAccess();if(!Access||Access->GetRegionId()!=Region->GetString()||Access->GetRegionGeneration()!=static_cast<uint64>(Generation->GetInt()))continue;FURLabShmLiveObjectDescriptor Desc;Desc.RegionId=Region->GetString();Desc.RegionGeneration=Generation->GetInt();Desc.ObjectName=Object->GetString();Desc.ByteLength=48+Max->GetInt();FURLabShmLiveObjectPtr Live;auto Status=Access->OpenLiveObject(Desc,Live);if(!Status){Error=Status.Message;return false;}ShmReader=FURLabLockedLatestFrameReader::Create(Live,static_cast<uint32>(Max->GetInt()),Error);if(!ShmReader)return false;bStop.store(false);bShmExited.store(false);if(!ShmWorker.Start(TEXT("URLab.Viewer.ShmLatest"),[this](){RunShm();},[this](){bStop.store(true);})){bShmExited.store(true);Error=TEXT("viewer SHM reader worker could not start");return false;}return StartCommandWorker(Error);}}
	Error=TEXT("producer did not advertise a delivery compatible with negotiated data");return false;
}
void FURLabViewerStreamConnection::RunShm(){while(!bStop.load()){TArray<uint8>B;FString E;const auto R=ShmReader->ReadLatest(B,E);if(R==EURLabLatestFrameRead::Frame)StoreLatest(MoveTemp(B));else if(R==EURLabLatestFrameRead::Error)break;FPlatformProcess::SleepNoStats(0.001f);}bShmExited.store(true,std::memory_order_release);}
bool FURLabViewerStreamConnection::StartCommandWorker(FString& Error){if(CommandWorker.IsRunning())return true;bCommandExited.store(false);if(!CommandWorker.Start(TEXT("URLab.Viewer.Commands"),[this](){RunCommands();},[this](){bStop.store(true);})){bCommandExited.store(true);Error=TEXT("viewer producer command worker could not start");return false;}return true;}
bool FURLabViewerStreamConnection::ForwardInteraction(V Body,int32 TimeoutMs,FInteractionCallback&&Completion,FString&Error){if(!Session||!CommandWorker.IsRunning()||bStop.load()||TimeoutMs<=0||!Completion){Error=TEXT("viewer producer command connection is unavailable");return false;}int32 Prior=PendingCommands.fetch_add(1);if(Prior>=32){PendingCommands.fetch_sub(1);Error=TEXT("viewer producer command queue capacity is exhausted");return false;}FCommand C;C.Body=MoveTemp(Body);C.TimeoutMs=TimeoutMs;C.Completion=MoveTemp(Completion);Commands.Enqueue(MoveTemp(C));Error.Reset();return true;}
bool FURLabViewerStreamConnection::ProbeProducer()
{
	if(!Session||!ExpectedDescriptor)return false;
	FURLabViewerProducerRequest Ticket;FString Error;FURLabReplyEnvelope Reply;
	if(!Session->BeginRequest(TEXT("pose_stream_describe"),V::MakeMap({},{}),Ticket,Error)
		||Session->PollRequest(Ticket,1000,Reply,Error)!=EURLabViewerProducerRequestState::Terminal)
		return false;
	if(Reply.Kind!=EURLabReplyKind::Result||!FURLabOperationSchema::ValidateResultBody(TEXT("pose_stream_describe"),Reply.Result))
	{Health.store(static_cast<uint8>(EHealth::Failed));return true;}
	const V* Stream=Reply.Result.FindMapValue(TEXT("stream"));FURLabViewerStreamDescriptor Actual;
	if(!Stream||!ParseDescriptor(*Stream,Actual)){Health.store(static_cast<uint8>(EHealth::Failed));return true;}
	if(Same(*ExpectedDescriptor,Actual)){Health.store(static_cast<uint8>(EHealth::Healthy));return true;}
	if(Actual.ProducerBootId==ExpectedDescriptor->ProducerBootId&&Actual.StreamId==ExpectedDescriptor->StreamId
		&&Actual.StreamGeneration>ExpectedDescriptor->StreamGeneration)
	{Health.store(static_cast<uint8>(EHealth::Successor));return true;}
	Health.store(static_cast<uint8>(EHealth::Failed));return true;
}

void FURLabViewerStreamConnection::RunCommands()
{
	double NextProbe=FPlatformTime::Seconds()+1.0;int32 ProbeFailures=0;
	TOptional<FCommand> Active;TOptional<FURLabViewerProducerRequest> Ticket;
	while(!bStop.load())
	{
		const double Now=FPlatformTime::Seconds();
		if(Now>=NextProbe)
		{
			const bool Completed=ProbeProducer();
			if(Health.load()!=static_cast<uint8>(EHealth::Healthy))break;
			ProbeFailures=Completed?0:ProbeFailures+1;
			if(ProbeFailures>=2){Health.store(static_cast<uint8>(EHealth::Failed));break;}
			NextProbe=FPlatformTime::Seconds()+1.0;
		}
		if(!Active)
		{
			FCommand C;if(Commands.Dequeue(C))
			{
				FURLabViewerProducerRequest Started;FString Error;
				if(Session&&Session->BeginRequest(TEXT("interaction_submit"),MoveTemp(C.Body),Started,Error))
				{Active.Emplace(MoveTemp(C));Ticket.Emplace(MoveTemp(Started));}
				else {PendingCommands.fetch_sub(1);if(C.Completion){FURLabReplyEnvelope R;C.Completion(MoveTemp(R),MoveTemp(Error));}}
			}
		}
		if(Active&&Ticket)
		{
			FURLabReplyEnvelope Reply;FString Error;const int32 Slice=FMath::Clamp(Active->TimeoutMs,1,1000);
			const auto State=Session->PollRequest(*Ticket,Slice,Reply,Error);
			if(State!=EURLabViewerProducerRequestState::Pending)
			{FCommand Done=MoveTemp(*Active);Active.Reset();Ticket.Reset();PendingCommands.fetch_sub(1);if(Done.Completion)Done.Completion(MoveTemp(Reply),MoveTemp(Error));}
		}
		else FPlatformProcess::SleepNoStats(.001f);
	}
	if(Active){PendingCommands.fetch_sub(1);if(Active->Completion){FURLabReplyEnvelope R;Active->Completion(MoveTemp(R),TEXT("viewer producer connection retired"));}}
	FCommand C;while(Commands.Dequeue(C)){PendingCommands.fetch_sub(1);if(C.Completion){FURLabReplyEnvelope R;C.Completion(MoveTemp(R),TEXT("viewer producer connection retired"));}}
	bStop.store(true);bCommandExited.store(true,std::memory_order_release);
}
void FURLabViewerStreamConnection::BeginRetirement(){bStop.store(true);if(ZmqReader)ZmqReader->RequestStop();}
bool FURLabViewerStreamConnection::CanFinalizeRetirement() const{return bCommandExited.load(std::memory_order_acquire)&&bShmExited.load(std::memory_order_acquire)&&(!ZmqReader||ZmqReader->HasExited());}
void FURLabViewerStreamConnection::FinalizeRetirement(){if(!CanFinalizeRetirement())return;CommandWorker.Stop();if(ZmqReader)ZmqReader->Stop();ZmqReader.Reset();ShmWorker.Stop();ShmReader.Reset();if(Session){Session->Abort();Session.Reset();}FScopeLock L(&InboxMutex);Latest.Reset();}
void FURLabViewerStreamConnection::Shutdown(){bStop.store(true);CommandWorker.Stop();if(ZmqReader)ZmqReader->Stop();ZmqReader.Reset();ShmWorker.Stop();ShmReader.Reset();if(Session){FString E;Session->Disconnect(100,E);Session.Reset();}FScopeLock L(&InboxMutex);Latest.Reset();}
