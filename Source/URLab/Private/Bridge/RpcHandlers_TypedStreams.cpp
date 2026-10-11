#include "Bridge/RpcDispatcher.h"

#include "Async/Async.h"
#include "Bridge/PoseStreamServiceBinding.h"
#include "Bridge/CameraStreamServiceBinding.h"
#include "Bridge/RetainedCompletion.h"
#include "Bridge/InteractionWire.h"
#include "MuJoCo/Runtime/MjSimulationInteractionOwner.h"
#include "Transport/PoseStreamProducer.h"

namespace
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

V StreamValue(const FURLabPoseStreamDescriptor& Stream)
{
	return Map({
		{TEXT("kind"), V::MakeString(Stream.Kind)},
		{TEXT("schema_version"), V::MakeInt(Stream.SchemaVersion)},
		{TEXT("producer_boot_id"), V::MakeString(Stream.ProducerBootId)},
		{TEXT("stream_id"), V::MakeString(Stream.StreamId)},
		{TEXT("stream_generation"), V::MakeInt(static_cast<int64>(Stream.StreamGeneration))},
		{TEXT("source_revision"), V::MakeString(Stream.SourceRevision)},
		{TEXT("payload_format"), V::MakeString(Stream.PayloadFormat)}});
}

V ResultValue(const FURLabPoseStreamDescription& Description)
{
	TArray<V> Deliveries;
	if (Description.Zmq.IsSet())
	{
		const FURLabPoseStreamZmqDelivery& Zmq = Description.Zmq.GetValue();
		Deliveries.Add(Map({
			{TEXT("kind"), V::MakeString(TEXT("zmq"))},
			{TEXT("endpoint"), V::MakeString(Zmq.Endpoint)},
			{TEXT("topic"), V::MakeBinary(Zmq.Topic)}}));
	}
	if (Description.Shm.IsSet())
	{
		const FURLabPoseStreamShmDelivery& Shm = Description.Shm.GetValue();
		Deliveries.Add(Map({
			{TEXT("kind"), V::MakeString(TEXT("shm_latest"))},
			{TEXT("region_id"), V::MakeString(Shm.Object.RegionId)},
			{TEXT("region_generation"), V::MakeInt(static_cast<int64>(Shm.Object.RegionGeneration))},
			{TEXT("object_name"), V::MakeString(Shm.Object.ObjectName)},
			{TEXT("layout_version"), V::MakeInt(Shm.LayoutVersion)},
			{TEXT("max_payload_bytes"), V::MakeInt(Shm.MaxPayloadBytes)}}));
	}
	return Map({{TEXT("stream"), StreamValue(Description.Stream)},
		{TEXT("deliveries"), V::MakeArray(MoveTemp(Deliveries))}});
}

V CameraStreamValue(const FURLabCameraStreamDescriptor& Stream)
{
	return Map({{TEXT("kind"), V::MakeString(TEXT("camera_frame"))},
		{TEXT("schema_version"), V::MakeInt(1)},
		{TEXT("producer_boot_id"), V::MakeString(Stream.ProducerBootId)},
		{TEXT("stream_id"), V::MakeString(Stream.StreamId)},
		{TEXT("stream_generation"), V::MakeInt(Stream.StreamGeneration)},
		{TEXT("source_revision"), V::MakeString(Stream.SourceRevision)},
		{TEXT("camera_name"), V::MakeString(Stream.CameraName)},
		{TEXT("camera_configuration_revision"), V::MakeInt(Stream.CameraConfigurationRevision)},
		{TEXT("encoding"), V::MakeString(Stream.Encoding)},
		{TEXT("width"), V::MakeInt(Stream.Width)}, {TEXT("height"), V::MakeInt(Stream.Height)},
		{TEXT("stride_bytes"), V::MakeInt(Stream.StrideBytes)}});
}

V CameraResultValue(const FURLabCameraStreamDescriptor& Stream,
	const FURLabCameraStreamDelivery& Delivery, bool bSelectShm)
{
	V Selected;
	if (bSelectShm && Delivery.ShmObject.IsSet())
	{
		const auto& Shm = Delivery.ShmObject.GetValue();
		Selected = Map({{TEXT("kind"), V::MakeString(TEXT("shm_latest"))},
			{TEXT("region_id"), V::MakeString(Shm.RegionId)},
			{TEXT("region_generation"), V::MakeInt(Shm.RegionGeneration)},
			{TEXT("object_name"), V::MakeString(Shm.ObjectName)},
			{TEXT("layout_version"), V::MakeInt(Delivery.ShmLayoutVersion)},
			{TEXT("max_payload_bytes"), V::MakeInt(Delivery.MaxFrameBytes)}});
	}
	else
	{
		Selected = Map({{TEXT("kind"), V::MakeString(TEXT("zmq"))},
			{TEXT("endpoint"), V::MakeString(Delivery.ZmqEndpoint)},
			{TEXT("topic"), V::MakeBinary(Delivery.ZmqTopic)}});
	}
	return Map({{TEXT("stream"), CameraStreamValue(Stream)},
		{TEXT("selected_delivery"), MoveTemp(Selected)}});
}
}

bool FURLabRpcDispatcher::DispatchTypedStreams(const FURLabRequestEnvelope& Request,
	const FURLabRequestDigest& Digest,
	const FURLabRequestLedger::FSessionRef& Session,
	const FURLabDataSessionPtr& DataSession, TArray<uint8>& OutReplyBytes)
{
	FURLabRequestLedger::FAdmissionResult Admission;
	TSharedPtr<FURLabPoseStreamServiceBinding, ESPMode::ThreadSafe> Binding;
	{
		FScopeLock Lock(&DispatchMutex);
		Binding = PoseStreamService;
		Admission = RequestLedger->Admit(Session, Request.RequestId, Digest);
	}
	const EURLabRetainedAdmissionReply AdmissionReply =
		FURLabRetainedCompletion::EncodeAdmission(Admission, Request, OutReplyBytes);
	if (AdmissionReply != EURLabRetainedAdmissionReply::Accepted)
		return AdmissionReply == EURLabRetainedAdmissionReply::Replied;

	const TSharedRef<FURLabRetainedCompletion, ESPMode::ThreadSafe> Completion =
		MakeShared<FURLabRetainedCompletion, ESPMode::ThreadSafe>(
			RequestLedger, Session, Request, Digest);
	const TSharedPtr<FURLabPoseStreamProducer, ESPMode::ThreadSafe> Producer =
		Binding ? Binding->ResolveProducer() : nullptr;
	if (Request.Op == TEXT("interaction_submit"))
	{
		const TSharedPtr<FMjSimulationInteractionOwner, ESPMode::ThreadSafe> Owner =
			Binding ? Binding->ResolveInteractionOwner() : nullptr;
		if (!Owner)
		{
			Completion->Error(TEXT("unsupported"),
				TEXT("Pose producer has no interaction handler"));
		}
		else
		{
			FURLabInteractionWrenchRequest Work;
			const FURLabWireValue& Grant = *Request.Body.FindMapValue(TEXT("grant"));
			const FURLabWireValue& Wrench = *Request.Body.FindMapValue(TEXT("wrench"));
			Work.Ticket = Request.SessionId + TEXT(":")
				+ FString::Printf(TEXT("%llu"), Request.RequestId);
			if (!URLabInteractionWire::ParseGrant(Grant, Work)
				|| !URLabInteractionWire::ParseWrench(Wrench, Work))
			{
				Completion->Error(TEXT("bad_request"),
					TEXT("Interaction grant or wrench is invalid"));
			}
			else
			{
				auto Sink = MakeShared<URLabInteractionWire::FCompletionSink,
					ESPMode::ThreadSafe>(Completion);
				const FURLabInteractionAdmissionResult Result = Owner->Enqueue(
					Work, MoveTemp(Sink));
				if (Result.Admission != EURLabInteractionAdmission::Accepted)
				{
					const TCHAR* Code = Result.Admission == EURLabInteractionAdmission::Capacity
						? TEXT("service_unavailable")
						: Result.Admission == EURLabInteractionAdmission::Invalid
						? TEXT("bad_request") : TEXT("grant_invalid");
					Completion->Error(Code, Result.Error);
				}
			}
		}
	}
	else if (Request.Op == TEXT("camera_stream_describe"))
	{
		TSharedPtr<FURLabCameraStreamServiceBinding, ESPMode::ThreadSafe> CameraBinding;
		{ FScopeLock Lock(&DispatchMutex); CameraBinding = CameraStreamService; }
		const FURLabDataSessionSnapshot Data = DataSession
			? DataSession->Snapshot(SessionNow()) : FURLabDataSessionSnapshot{};
		const bool bShm = Data.bActive && Data.Adapter == TEXT("zmq")
			&& Data.Transfer == EURLabDataTransfer::Shm;
		const FString CameraName = Request.Body.FindMapValue(TEXT("camera_name"))->GetString();
		FURLabGameThreadCommandQueue::FWork Work;
		Work.Execute = [Completion, CameraBinding, CameraName, bShm]()
		{
			FURLabCameraStreamDescriptor Stream; FURLabCameraStreamDelivery Delivery; FString Error;
			if (!CameraBinding || !CameraBinding->Describe(CameraName, bShm, Stream, Delivery, Error))
			{
				Completion->Error(TEXT("stream_unavailable"), Error.IsEmpty()
					? TEXT("Camera stream is unavailable") : Error); return;
			}
			Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request,
				CameraResultValue(Stream, Delivery, bShm)));
		};
		Work.Cancel = [Completion]() { Completion->Error(TEXT("service_unavailable"),
			TEXT("Camera stream describe was cancelled")); };
		const bool bStart = RenderCommandQueue->Enqueue(MoveTemp(Work));
		if (IsInGameThread()) RenderCommandQueue->Drain();
		else if (bStart) { auto Queue = RenderCommandQueue;
			AsyncTask(ENamedThreads::GameThread, [Queue]() { Queue->Drain(); }); }
	}
	else if (!Producer || !DataSession || !DataSession->IsActive())
	{
		Completion->Error(TEXT("service_unavailable"), TEXT("Pose stream service is unavailable"));
	}
	else
	{
		const FURLabDataSessionSnapshot Data = DataSession->Snapshot(SessionNow());
		const bool bShmEligible = Data.bActive && Data.Adapter == TEXT("zmq")
			&& Data.Transfer == EURLabDataTransfer::Shm;
		FURLabPoseStreamDescription Description;
		FString Error;
		if (!Producer->DescribeForSession(bShmEligible, Description, Error))
		{
			Completion->Error(TEXT("service_unavailable"), Error);
		}
		else
		{
			Completion->Publish(FURLabReplyEnvelope::MakeResult(
				Request, ResultValue(Description)));
		}
	}
	const FURLabRequestLedger::FAdmissionResult Done =
		RequestLedger->Admit(Session, Request.RequestId, Digest);
	if (Done.Terminal == FURLabRequestLedger::Completed && Done.ReplyBytes)
	{
		OutReplyBytes = *Done.ReplyBytes;
		return true;
	}
	return FURLabRequestEnvelopeCodec::EncodeReply(
		FURLabReplyEnvelope::MakePending(Request), OutReplyBytes).IsOk();
}
