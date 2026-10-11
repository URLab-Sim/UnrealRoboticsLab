// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "Bridge/RpcDispatcher.h"
#include "Bridge/RetainedCompletion.h"
#include "Wire/UrlabWire.gen.h"
#include "Async/Async.h"

namespace
{
using FPayloadValue = FURLabWireValue;

FPayloadValue PayloadMap(std::initializer_list<TPair<FString, FPayloadValue>> Fields)
{
	TArray<FString> Keys;
	TArray<FPayloadValue> Values;
	for (const TPair<FString, FPayloadValue>& Field : Fields)
	{
		Keys.Add(Field.Key);
		Values.Add(Field.Value);
	}
	return FPayloadValue::MakeMap(MoveTemp(Keys), MoveTemp(Values));
}

const TCHAR* PayloadErrorCode(EURLabPayloadStoreError Error)
{
	switch (Error)
	{
	case EURLabPayloadStoreError::InvalidLimits: return TEXT("payload_configuration_invalid");
	case EURLabPayloadStoreError::InvalidClock: return TEXT("payload_clock_invalid");
	case EURLabPayloadStoreError::InvalidIdentity:
	case EURLabPayloadStoreError::IdentityMismatch: return TEXT("payload_identity_mismatch");
	case EURLabPayloadStoreError::SessionRevoked: return TEXT("session_expired");
	case EURLabPayloadStoreError::InvalidDeclaration:
	case EURLabPayloadStoreError::InvalidChunk: return TEXT("bad_request");
	case EURLabPayloadStoreError::LimitExceeded: return TEXT("payload_quota_exceeded");
	case EURLabPayloadStoreError::NotFound: return TEXT("payload_not_found");
	case EURLabPayloadStoreError::Expired: return TEXT("payload_expired");
	case EURLabPayloadStoreError::ChunkTooLarge: return TEXT("payload_chunk_too_large");
	case EURLabPayloadStoreError::ChunkLimitExceeded: return TEXT("payload_chunk_limit_exceeded");
	case EURLabPayloadStoreError::OutOfBounds: return TEXT("payload_out_of_bounds");
	case EURLabPayloadStoreError::Overlap: return TEXT("payload_overlap");
	case EURLabPayloadStoreError::Incomplete: return TEXT("payload_incomplete");
	case EURLabPayloadStoreError::IntegrityMismatch: return TEXT("payload_integrity_mismatch");
	case EURLabPayloadStoreError::BackingFailure: return TEXT("payload_backing_failure");
	default: return TEXT("internal_error");
	}
}

const TCHAR* DataSelectionErrorCode(EURLabDataSelectionError Error)
{
	switch (Error)
	{
	case EURLabDataSelectionError::Inactive: return TEXT("session_expired");
	case EURLabDataSelectionError::NegotiationExpired: return TEXT("shm_challenge_expired");
	case EURLabDataSelectionError::SelectionConflict: return TEXT("data_selection_conflict");
	case EURLabDataSelectionError::ChallengeInvalid: return TEXT("shm_challenge_invalid");
	case EURLabDataSelectionError::InvalidClock:
	case EURLabDataSelectionError::BackingFailure: return TEXT("payload_backing_failure");
	default: return TEXT("internal_error");
	}
}

FPayloadValue PayloadDescriptor(const FURLabStoredPayloadDescriptor& Descriptor)
{
	if (Descriptor.BackingKind == EURLabPayloadBackingKind::ShmSealedFile)
	{
		return PayloadMap({
			{TEXT("transfer"), FPayloadValue::MakeString(TEXT("shm"))},
			{TEXT("blob_id"), FPayloadValue::MakeString(Descriptor.BlobId)},
			{TEXT("boot_id"), FPayloadValue::MakeString(Descriptor.BootId)},
			{TEXT("session_id"), FPayloadValue::MakeString(Descriptor.SessionId)},
			{TEXT("byte_length"), FPayloadValue::MakeInt(Descriptor.ByteLength)},
			{TEXT("sha256"), FPayloadValue::MakeString(Descriptor.Sha256)},
			{TEXT("expires_in_s"), FPayloadValue::MakeFloat(Descriptor.ExpiresInSeconds)},
			{TEXT("region_id"), FPayloadValue::MakeString(Descriptor.RegionId)},
			{TEXT("region_generation"), FPayloadValue::MakeInt(
				static_cast<int64>(Descriptor.RegionGeneration))},
			{TEXT("object_name"), FPayloadValue::MakeString(Descriptor.ObjectName)}});
	}
	return PayloadMap({
		{TEXT("transfer"), FPayloadValue::MakeString(TEXT("network"))},
		{TEXT("blob_id"), FPayloadValue::MakeString(Descriptor.BlobId)},
		{TEXT("boot_id"), FPayloadValue::MakeString(Descriptor.BootId)},
		{TEXT("session_id"), FPayloadValue::MakeString(Descriptor.SessionId)},
		{TEXT("byte_length"), FPayloadValue::MakeInt(Descriptor.ByteLength)},
		{TEXT("sha256"), FPayloadValue::MakeString(Descriptor.Sha256)},
		{TEXT("expires_in_s"), FPayloadValue::MakeFloat(Descriptor.ExpiresInSeconds)}});
}

bool ValidatePayloadMetadata(const FString& Op, FString& OutError)
{
	const URLabWire::TypedOp::FDecl* Decl = URLabWire::TypedOp::Find(Op);
	if (!Decl || !Decl->Service || !Decl->Execution || !Decl->Authority || !Decl->Permission
		|| FString(Decl->Service) != TEXT("data")
		|| FString(Decl->Execution) != TEXT("payload_queue")
		|| FString(Decl->Authority) != TEXT("session")
		|| FString(Decl->Permission) != TEXT("none"))
	{
		OutError = TEXT("operation has missing or invalid payload service metadata");
		return false;
	}
	return true;
}

void ExecuteTypedPayload(
	const TSharedRef<FURLabRetainedCompletion, ESPMode::ThreadSafe>& Completion,
	const TSharedRef<FURLabPayloadStore, ESPMode::ThreadSafe>& Store,
	const FURLabDataSessionPtr& DataSession, double NowSeconds,
	const FString& MetadataError)
{
	if (!MetadataError.IsEmpty())
	{
		Completion->Error(TEXT("server_schema_error"), MetadataError);
		return;
	}
	const FString& Op = Completion->Request.Op;
	const FURLabWireValue& Body = Completion->Request.Body;
	const FURLabPayloadOwner& Owner = DataSession->GetOwner();
	FURLabPayloadStoreStatus Status;
	if (Op == TEXT("data_select"))
	{
		const FString& Transfer = Body.FindMapValue(TEXT("transfer"))->GetString();
		FURLabDataSelectionCommit Commit;
		FURLabShmExactChallengePtr RetiredChallenge;
		FURLabDataSelectionStatus Selection;
		if (Transfer == TEXT("network"))
		{
			Selection = DataSession->PrepareSelectNetwork(
				NowSeconds, Commit, RetiredChallenge);
		}
		else
		{
			FURLabShmExactChallengeProof Proof;
			Proof.RegionId = Body.FindMapValue(TEXT("region_id"))->GetString();
			Proof.RegionGeneration = static_cast<uint64>(
				Body.FindMapValue(TEXT("region_generation"))->GetInt());
			Proof.ChallengeName = Body.FindMapValue(TEXT("challenge_name"))->GetString();
			Proof.Nonce = Body.FindMapValue(TEXT("challenge_nonce"))->GetBinary();
			Selection = DataSession->PrepareSelectShm(
				NowSeconds, Proof, Commit, RetiredChallenge);
		}
		FURLabShmExactStatus Retire;
		if (RetiredChallenge)
		{
			if (DataSession->GetRegion())
			{
				Retire = DataSession->GetRegion()->RetireChallenge(
					Owner, RetiredChallenge);
			}
			else
			{
				Retire.Code = EURLabShmExactError::IoFailure;
				Retire.Message = TEXT("SHM challenge region is unavailable during retirement");
			}
		}
		if (!Selection)
		{
			if (RetiredChallenge && !Retire)
			{
				Completion->Error(TEXT("payload_backing_failure"), Retire.Message);
				return;
			}
			Completion->Error(DataSelectionErrorCode(Selection.Code), Selection.Message);
			return;
		}
		Selection = DataSession->CompleteSelection(Commit,
			!RetiredChallenge || Retire.IsOk(), Retire.Message);
		if (!Selection)
		{
			Completion->Error(DataSelectionErrorCode(Selection.Code), Selection.Message);
			return;
		}
		if (Transfer == TEXT("shm"))
		{
			const FURLabShmExactRegionDescriptor& Region =
				DataSession->GetRegion()->GetDescriptor();
			Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request,
				PayloadMap({
					{TEXT("transfer"), FPayloadValue::MakeString(TEXT("shm"))},
					{TEXT("region_id"), FPayloadValue::MakeString(Region.RegionId)},
					{TEXT("region_generation"), FPayloadValue::MakeInt(
						static_cast<int64>(Region.Generation))}})));
		}
		else
		{
			Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request,
				PayloadMap({{TEXT("transfer"),
					FPayloadValue::MakeString(TEXT("network"))}})));
		}
		return;
	}

	const FURLabDataSessionSnapshot Data = DataSession->Snapshot(NowSeconds);
	if (!Data.bActive)
	{
		Completion->Error(TEXT("session_expired"), TEXT("Data session is inactive"));
		return;
	}
	if (Data.Transfer == EURLabDataTransfer::Provisional)
	{
		Completion->Error(TEXT("data_negotiation_required"),
			TEXT("Complete data-plane selection before payload operations"));
		return;
	}
	if (Op == TEXT("payload_begin"))
	{
		const FString& Requested = Body.FindMapValue(TEXT("transfer"))->GetString();
		const bool bShm = Data.Transfer == EURLabDataTransfer::Shm;
		if ((bShm && Requested != TEXT("shm")) || (!bShm && Requested != TEXT("network")))
		{
			Completion->Error(TEXT("data_transfer_mismatch"),
				TEXT("Payload transfer differs from the immutable session selection"));
			return;
		}
		if (bShm)
		{
			FURLabPayloadShmStagingGrant Grant;
			Status = Store->BeginShmWrite(Owner, DataSession->GetRegion(),
				Body.FindMapValue(TEXT("byte_length"))->GetInt(),
				Body.FindMapValue(TEXT("sha256"))->GetString(), Grant);
			if (Status)
			{
				Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request,
					PayloadMap({
						{TEXT("transfer"), FPayloadValue::MakeString(TEXT("shm"))},
						{TEXT("blob_id"), FPayloadValue::MakeString(Grant.BlobId)},
						{TEXT("region_id"), FPayloadValue::MakeString(Grant.RegionId)},
						{TEXT("region_generation"), FPayloadValue::MakeInt(
							static_cast<int64>(Grant.RegionGeneration))},
						{TEXT("object_name"), FPayloadValue::MakeString(Grant.ObjectName)},
						{TEXT("byte_length"), FPayloadValue::MakeInt(Grant.ByteLength)},
						{TEXT("sha256"), FPayloadValue::MakeString(Grant.Sha256)},
						{TEXT("expires_in_s"), FPayloadValue::MakeFloat(
							Grant.ExpiresInSeconds)}})));
				return;
			}
		}
		else
		{
			FString BlobId;
			Status = Store->BeginWrite(Owner,
				Body.FindMapValue(TEXT("byte_length"))->GetInt(),
				Body.FindMapValue(TEXT("sha256"))->GetString(), BlobId);
			if (!Status)
			{
				Completion->Error(PayloadErrorCode(Status.Code), Status.Message);
				return;
			}
			const FURLabPayloadStoreLimits& Limits = Store->GetLimits();
			Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request, PayloadMap({
				{TEXT("transfer"), FPayloadValue::MakeString(TEXT("network"))},
				{TEXT("blob_id"), FPayloadValue::MakeString(BlobId)},
				{TEXT("max_chunk_bytes"), FPayloadValue::MakeInt(Limits.MaxChunkBytes)},
				{TEXT("expires_in_s"), FPayloadValue::MakeFloat(Limits.StagingTtlSeconds)}})));
			return;
		}
	}
	else if (Op == TEXT("payload_write"))
	{
		if (Data.Transfer != EURLabDataTransfer::Network)
		{
			Completion->Error(TEXT("data_transfer_mismatch"),
				TEXT("Network payload writes are unavailable for SHM data"));
			return;
		}
		const TArray<uint8>& Bytes = Body.FindMapValue(TEXT("data"))->GetBinary();
		const int64 Offset = Body.FindMapValue(TEXT("offset"))->GetInt();
		Status = Store->WriteChunk(Owner, Body.FindMapValue(TEXT("blob_id"))->GetString(),
			Offset, Bytes);
		if (Status)
		{
			Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request, PayloadMap({
				{TEXT("blob_id"), FPayloadValue::MakeString(Body.FindMapValue(TEXT("blob_id"))->GetString())},
				{TEXT("offset"), FPayloadValue::MakeInt(Offset)},
				{TEXT("byte_length"), FPayloadValue::MakeInt(Bytes.Num())}})));
			return;
		}
	}
	else if (Op == TEXT("payload_commit"))
	{
		FURLabStoredPayloadDescriptor Descriptor;
		FURLabPayloadPin Pin;
		Status = Store->Commit(Owner, Body.FindMapValue(TEXT("blob_id"))->GetString(), Descriptor, Pin);
		if (Status)
		{
			Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request,
				PayloadMap({{TEXT("payload"), PayloadDescriptor(Descriptor)}})));
			return;
		}
	}
	else if (Op == TEXT("payload_read"))
	{
		if (Data.Transfer != EURLabDataTransfer::Network)
		{
			Completion->Error(TEXT("data_transfer_mismatch"),
				TEXT("Network payload reads are unavailable for SHM data"));
			return;
		}
		const FString& BlobId = Body.FindMapValue(TEXT("blob_id"))->GetString();
		const int64 Offset = Body.FindMapValue(TEXT("offset"))->GetInt();
		FURLabPayloadCopyPin Copy;
		Status = Store->CopyRange(Owner, BlobId, Offset,
			Body.FindMapValue(TEXT("byte_length"))->GetInt(), Copy);
		if (Status)
		{
			TArray<uint8> Bytes = Copy->GetBytes();
			Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request, PayloadMap({
				{TEXT("blob_id"), FPayloadValue::MakeString(BlobId)},
				{TEXT("offset"), FPayloadValue::MakeInt(Offset)},
				{TEXT("data"), FPayloadValue::MakeBinary(MoveTemp(Bytes))}})));
			return;
		}
	}
	else if (Op == TEXT("payload_ack"))
	{
		bool bNewlyAcknowledged = false;
		Status = Store->Acknowledge(Owner, Body.FindMapValue(TEXT("blob_id"))->GetString(),
			Body.FindMapValue(TEXT("sha256"))->GetString(), bNewlyAcknowledged);
		if (Status)
		{
			Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request, PayloadMap({
				{TEXT("acknowledged"), FPayloadValue::MakeBool(true)},
				{TEXT("newly_acknowledged"), FPayloadValue::MakeBool(bNewlyAcknowledged)}})));
			return;
		}
	}
	else if (Op == TEXT("payload_release"))
	{
		Status = Store->Release(Owner, Body.FindMapValue(TEXT("blob_id"))->GetString());
		if (Status)
		{
			Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request,
				PayloadMap({{TEXT("released"), FPayloadValue::MakeBool(true)}})));
			return;
		}
	}
	else
	{
		Completion->Error(TEXT("server_schema_error"),
			TEXT("active payload operation has no store handler"));
		return;
	}

	Completion->Error(PayloadErrorCode(Status.Code), Status.Message);
}
} // namespace

bool FURLabRpcDispatcher::DispatchTypedPayload(const FURLabRequestEnvelope& Request,
	const FURLabRequestDigest& Digest, const FURLabRequestLedger::FSessionRef& Session,
	const FURLabDataSessionPtr& DataSession, TArray<uint8>& OutReplyBytes)
{
	FURLabRequestLedger::FAdmissionResult Admission;
	bool bStartDrain = false;
	const TSharedRef<FURLabRetainedCompletion, ESPMode::ThreadSafe> Completion =
		MakeShared<FURLabRetainedCompletion, ESPMode::ThreadSafe>(
			RequestLedger, Session, Request, Digest);
	{
		// Admission and enqueue share the dispatcher lock, establishing accepted
		// order while retaining only dispatcher-independent work state.
		FScopeLock Lock(&DispatchMutex);
		Admission = RequestLedger->Admit(Session, static_cast<int64>(Request.RequestId), Digest);
		if (Admission.Status == FURLabRequestLedger::Accepted)
		{
			FString MetadataError;
			ValidatePayloadMetadata(Request.Op, MetadataError);
			const TSharedRef<FURLabPayloadStore, ESPMode::ThreadSafe> Store = PayloadStore;
			const TSharedRef<std::atomic<double>, ESPMode::ThreadSafe> SessionClock =
				SessionClockOverrideForTest;
			FURLabSerialCommandQueue::FWork Work;
			Work.Execute = [Completion, Store, DataSession, SessionClock,
				MetadataError = MoveTemp(MetadataError)]()
			{
				const double Override = SessionClock->load(std::memory_order_acquire);
				ExecuteTypedPayload(Completion, Store, DataSession,
					Override >= 0.0 ? Override : FPlatformTime::Seconds(), MetadataError);
			};
			Work.Cancel = [Completion]()
			{
				Completion->Error(TEXT("service_unavailable"),
					TEXT("Payload command was cancelled during service shutdown"));
			};
			bStartDrain = PayloadCommandQueue->Enqueue(MoveTemp(Work));
		}
	}
	const EURLabRetainedAdmissionReply AdmissionReply =
		FURLabRetainedCompletion::EncodeAdmission(Admission, Request, OutReplyBytes);
	if (AdmissionReply != EURLabRetainedAdmissionReply::Accepted)
		return AdmissionReply == EURLabRetainedAdmissionReply::Replied;

	if (bStartDrain && bPayloadQueueAutoDrainForTest.load(std::memory_order_acquire))
	{
		const TSharedRef<FURLabSerialCommandQueue, ESPMode::ThreadSafe> Queue = PayloadCommandQueue;
		Async(EAsyncExecution::ThreadPool, [Queue]() { Queue->Drain(); });
	}
	return FURLabRequestEnvelopeCodec::EncodeReply(
		FURLabReplyEnvelope::MakePending(Request), OutReplyBytes).IsOk();
}
