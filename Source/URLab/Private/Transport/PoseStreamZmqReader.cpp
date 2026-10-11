#include "Transport/PoseStreamZmqReader.h"
#include "Transport/ZmqContext.h"
#include "HAL/PlatformProcess.h"
#include "Misc/ScopeExit.h"
#include "zmq.h"

FURLabPoseStreamZmqReader::FURLabPoseStreamZmqReader(FString&& E,TArray<uint8>&& T,int64 M,FOnFrame&& C)
	:Endpoint(MoveTemp(E)),Topic(MoveTemp(T)),MaxFrameBytes(M),Callback(MoveTemp(C)){}
TUniquePtr<FURLabPoseStreamZmqReader> FURLabPoseStreamZmqReader::Create(const FString&E,TArray<uint8>T,int64 M,FOnFrame C,FString&Error)
{
	if(!E.StartsWith(TEXT("tcp://"))||T.IsEmpty()||T.Num()>1024||M<=0||M>MAX_int32||!C){Error=TEXT("invalid pose stream ZMQ reader configuration");return nullptr;}
	auto R=TUniquePtr<FURLabPoseStreamZmqReader>(new FURLabPoseStreamZmqReader(FString(E),MoveTemp(T),M,MoveTemp(C)));R->ReadyEvent=FPlatformProcess::GetSynchEventFromPool(true);
	if(!R->Worker.Start(TEXT("URLab.PoseStream.Zmq"),[P=R.Get()](){P->Run();},[P=R.Get()](){P->bStop.store(true,std::memory_order_release);})){Error=TEXT("pose stream ZMQ worker could not start");return nullptr;}
	if(!R->ReadyEvent->Wait(2000)||!R->bReady.load()){Error=TEXT("pose stream ZMQ reader could not connect and subscribe");R->Stop();return nullptr;}
	Error.Reset();return R;
}
FURLabPoseStreamZmqReader::~FURLabPoseStreamZmqReader(){Stop();if(ReadyEvent){FPlatformProcess::ReturnSynchEventToPool(ReadyEvent);ReadyEvent=nullptr;}}
void FURLabPoseStreamZmqReader::Stop(){bStop.store(true,std::memory_order_release);Worker.Stop();}
void FURLabPoseStreamZmqReader::Run()
{
	ON_SCOPE_EXIT { bExited.store(true,std::memory_order_release); };
	void* Ctx=FURLabZmqContext::Acquire();void* S=Ctx?zmq_socket(Ctx,ZMQ_SUB):nullptr;if(!S){if(ReadyEvent)ReadyEvent->Trigger();FURLabZmqContext::Release(Ctx);return;}
	const int Timeout=50,Linger=0,Hwm=4;zmq_setsockopt(S,ZMQ_RCVTIMEO,&Timeout,sizeof Timeout);zmq_setsockopt(S,ZMQ_LINGER,&Linger,sizeof Linger);zmq_setsockopt(S,ZMQ_RCVHWM,&Hwm,sizeof Hwm);
	if(zmq_setsockopt(S,ZMQ_SUBSCRIBE,Topic.GetData(),Topic.Num())!=0||zmq_connect(S,TCHAR_TO_UTF8(*Endpoint))!=0){if(ReadyEvent)ReadyEvent->Trigger();zmq_close(S);FURLabZmqContext::Release(Ctx);return;}
	bReady.store(true);if(ReadyEvent)ReadyEvent->Trigger();
	auto Receive=[&](int Flags,TArray<uint8>&Out)->bool{zmq_msg_t A,B;zmq_msg_init(&A);zmq_msg_init(&B);const int N=zmq_msg_recv(&A,S,Flags);if(N<0){zmq_msg_close(&A);zmq_msg_close(&B);return false;}int More=0;size_t L=sizeof More;zmq_getsockopt(S,ZMQ_RCVMORE,&More,&L);bool Ok=More&&N==Topic.Num()&&!FMemory::Memcmp(zmq_msg_data(&A),Topic.GetData(),Topic.Num());const int P=More?zmq_msg_recv(&B,S,Flags):-1;zmq_getsockopt(S,ZMQ_RCVMORE,&More,&L);Ok=Ok&&P>=0&&!More&&P<=MaxFrameBytes;if(Ok){Out.SetNumUninitialized(P);if(P)FMemory::Memcpy(Out.GetData(),zmq_msg_data(&B),P);}zmq_msg_close(&A);zmq_msg_close(&B);return Ok;};
	while(!bStop.load(std::memory_order_acquire)){TArray<uint8>Newest,Incoming;if(!Receive(0,Newest))continue;while(!bStop.load(std::memory_order_acquire)&&Receive(ZMQ_DONTWAIT,Incoming))Newest=MoveTemp(Incoming);if(!Newest.IsEmpty()&&Callback)Callback(MoveTemp(Newest));}
	zmq_close(S);FURLabZmqContext::Release(Ctx);
}
