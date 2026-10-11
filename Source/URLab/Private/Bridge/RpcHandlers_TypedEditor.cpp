#include "Bridge/RpcDispatcher.h"
#include "Bridge/OpRegistry.h"
#include "Bridge/RetainedCompletion.h"
#include "Bridge/GameThreadCommandQueue.h"
#include "Wire/UrlabWire.gen.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Async/Async.h"

namespace
{
TSharedPtr<FJsonValue> WireToJson(const FURLabWireValue& V)
{
 switch(V.GetType()) {
 case EURLabWireValueType::Null:return MakeShared<FJsonValueNull>();
 case EURLabWireValueType::Bool:return MakeShared<FJsonValueBoolean>(V.GetBool());
 case EURLabWireValueType::Int64:return MakeShared<FJsonValueNumber>((double)V.GetInt());
 case EURLabWireValueType::Float64:return MakeShared<FJsonValueNumber>(V.GetFloat());
 case EURLabWireValueType::String:return MakeShared<FJsonValueString>(V.GetString());
 case EURLabWireValueType::Array:{TArray<TSharedPtr<FJsonValue>> A;for(const auto& X:V.GetArray())A.Add(WireToJson(X));return MakeShared<FJsonValueArray>(MoveTemp(A));}
 case EURLabWireValueType::Map:{auto O=MakeShared<FJsonObject>();for(int32 I=0;I<V.GetMapSize();++I)O->SetField(V.GetMapKey(I),WireToJson(V.GetMapValue(I)));return MakeShared<FJsonValueObject>(O);}
 default:return MakeShared<FJsonValueNull>();
 }
}
bool JsonToWire(const TSharedPtr<FJsonValue>& J,const FString& Type,FURLabWireValue& Out,FString& Error)
{
 if(Type==TEXT("string")||Type==TEXT("nonempty_string")||Type==TEXT("literal")){FString X;if(!J||!J->TryGetString(X)){Error=TEXT("expected string");return false;}Out=FURLabWireValue::MakeString(MoveTemp(X));return true;}
 if(Type==TEXT("bool")){bool X=false;if(!J||!J->TryGetBool(X)){Error=TEXT("expected bool");return false;}Out=FURLabWireValue::MakeBool(X);return true;}
 if(Type==TEXT("finite_float")){double X=0;if(!J||!J->TryGetNumber(X)){Error=TEXT("expected float");return false;}Out=FURLabWireValue::MakeFloat(X);return true;}
 if(Type==TEXT("int64")||Type==TEXT("nonnegative_int64")||Type==TEXT("positive_int64")||Type==TEXT("positive_uint32")){double X=0;if(!J||!J->TryGetNumber(X)){Error=TEXT("expected integer");return false;}Out=FURLabWireValue::MakeInt((int64)X);return true;}
 if(Type.StartsWith(TEXT("array<"))&&Type.EndsWith(TEXT(">"))){const TArray<TSharedPtr<FJsonValue>>* A=nullptr;if(!J||!J->TryGetArray(A)||!A){Error=TEXT("expected array");return false;}const FString E=Type.Mid(6,Type.Len()-7);TArray<FURLabWireValue> W;for(const auto& X:*A){FURLabWireValue V;if(!JsonToWire(X,E,V,Error))return false;W.Add(MoveTemp(V));}Out=FURLabWireValue::MakeArray(MoveTemp(W));return true;}
 const URLabWire::TypedResult::FShape* S=URLabWire::TypedResult::Find(Type);const TSharedPtr<FJsonObject>* O=nullptr;if(!S||!J||!J->TryGetObject(O)||!O||!O->IsValid()){Error=TEXT("expected generated result object");return false;}TArray<FString>K;TArray<FURLabWireValue>V;for(int32 I=0;I<S->Num;++I){const auto& F=S->Fields[I];const TSharedPtr<FJsonValue>* P=(*O)->Values.Find(F.Name);if(!P){if(F.Required){Error=FString::Printf(TEXT("missing result field %s"),F.Name);return false;}continue;}FURLabWireValue X;if(!JsonToWire(*P,F.Type,X,Error))return false;K.Add(F.Name);V.Add(MoveTemp(X));}Out=FURLabWireValue::MakeMap(MoveTemp(K),MoveTemp(V));return true;
}
void FlattenNode(const TSharedPtr<FJsonObject>& Node,int64 Parent,TArray<TSharedPtr<FJsonValue>>& Out)
{
 const int64 Index=Out.Num();auto Flat=MakeShared<FJsonObject>();FString S;Node->TryGetStringField(TEXT("name"),S);Flat->SetStringField(TEXT("name"),S);Node->TryGetStringField(TEXT("class"),S);Flat->SetStringField(TEXT("class"),S);Flat->SetNumberField(TEXT("parent_index"),(double)Parent);Flat->SetBoolField(TEXT("is_root"),Parent<0);const TArray<TSharedPtr<FJsonValue>>* L=nullptr;if(Node->TryGetArrayField(TEXT("location"),L)&&L)Flat->SetArrayField(TEXT("location"),*L);else Flat->SetArrayField(TEXT("location"),{});Out.Add(MakeShared<FJsonValueObject>(Flat));const TArray<TSharedPtr<FJsonValue>>* C=nullptr;if(Node->TryGetArrayField(TEXT("children"),C)&&C)for(const auto& Child:*C){const TSharedPtr<FJsonObject>* Obj=nullptr;if(Child->TryGetObject(Obj)&&Obj&&Obj->IsValid())FlattenNode(*Obj,Index,Out);}
}

void ExecuteEditorRequest(
	FURLabRpcDispatcher* Dispatcher,
	const TSharedRef<FURLabRetainedCompletion, ESPMode::ThreadSafe>& Completion)
{
	const FString Legacy = Completion->Request.Op.Mid(7);
	auto Handler = URLabOpRegistry::GetHandler(TEXT("__typed_editor_") + Legacy);
	if (!Handler)
	{
		Completion->Error(TEXT("service_unavailable"), TEXT("Editor service is not registered"));
		return;
	}
	auto Json = WireToJson(Completion->Request.Body);
	const TSharedPtr<FJsonObject>* Body = nullptr;
	if (!Json->TryGetObject(Body) || !Body || !Body->IsValid())
	{
		Completion->Error(TEXT("bad_request"), TEXT("Editor request body is not a map"));
		return;
	}
	auto Reply = Handler(*Dispatcher, *Body);
	if (!Reply)
	{
		Completion->Error(TEXT("editor_failed"), TEXT("Editor handler returned no result"));
		return;
	}
	FString ReplyOp;
	if (Reply->TryGetStringField(TEXT("op"), ReplyOp) && ReplyOp == TEXT("error"))
	{
		FString Code, Message;
		Reply->TryGetStringField(TEXT("code"), Code);
		Reply->TryGetStringField(TEXT("message"), Message);
		Completion->Error(Code.IsEmpty() ? TEXT("editor_failed") : Code, Message);
		return;
	}
	if (Completion->Request.Op == TEXT("editor_actor_hierarchy"))
	{
		const TSharedPtr<FJsonObject>* Root = nullptr;
		if (Reply->TryGetObjectField(TEXT("root"), Root) && Root && Root->IsValid())
		{
			TArray<TSharedPtr<FJsonValue>> Nodes;
			FlattenNode(*Root, -1, Nodes);
			Reply->SetArrayField(TEXT("nodes"), MoveTemp(Nodes));
		}
	}
	FURLabWireValue Result;
	FString Error;
	if (!JsonToWire(MakeShared<FJsonValueObject>(Reply), Completion->Request.Op, Result, Error))
	{
		Completion->Error(TEXT("encoding_error"), Error);
		return;
	}
	Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request, MoveTemp(Result)));
}
}

bool FURLabRpcDispatcher::HasEditorService() const{return URLabOpRegistry::GetHandler(TEXT("__typed_editor_current_level"))!=nullptr;}

bool FURLabRpcDispatcher::DispatchTypedEditor(const FURLabRequestEnvelope& Request,const FURLabRequestDigest& Digest,const FURLabRequestLedger::FSessionRef& Session,TArray<uint8>& Out)
{
 FURLabRequestLedger::FAdmissionResult A;bool Start=false;auto C=MakeShared<FURLabRetainedCompletion,ESPMode::ThreadSafe>(RequestLedger,Session,Request,Digest);{FScopeLock L(&DispatchMutex);A=RequestLedger->Admit(Session,Request.RequestId,Digest);if(A.Status==FURLabRequestLedger::Accepted){const bool bWorkerLifecycle=Request.Op==TEXT("editor_begin_pie")||Request.Op==TEXT("editor_stop_pie");if(bWorkerLifecycle){Start=true;Async(EAsyncExecution::ThreadPool,[this,C](){ExecuteEditorRequest(this,C);});}else{FURLabGameThreadCommandQueue::FWork W;W.Execute=[this,C](){ExecuteEditorRequest(this,C);};W.Cancel=[C](){C->Error(TEXT("service_unavailable"),TEXT("Editor command cancelled"));};Start=RenderCommandQueue->Enqueue(MoveTemp(W));}}}
 auto E=FURLabRetainedCompletion::EncodeAdmission(A,Request,Out);if(E!=EURLabRetainedAdmissionReply::Accepted)return E==EURLabRetainedAdmissionReply::Replied;if(IsInGameThread())RenderCommandQueue->Drain();else if(Start){auto Q=RenderCommandQueue;AsyncTask(ENamedThreads::GameThread,[Q](){Q->Drain();});}auto D=RequestLedger->Admit(Session,Request.RequestId,Digest);if(D.Terminal==FURLabRequestLedger::Completed&&D.ReplyBytes){Out=*D.ReplyBytes;return true;}FURLabRequestEnvelopeCodec::EncodeReply(FURLabReplyEnvelope::MakePending(Request),Out);return true;
}
