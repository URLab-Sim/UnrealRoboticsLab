// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "Bridge/RetainedCompletion.h"

#include "Wire/OperationSchema.h"

FURLabRetainedCompletion::FURLabRetainedCompletion(
	TSharedRef<FURLabRequestLedger, ESPMode::ThreadSafe> InLedger,
	FURLabRequestLedger::FSessionRef InSession,
	FURLabRequestEnvelope InRequest, FURLabRequestDigest InDigest)
	: Ledger(MoveTemp(InLedger)), Session(MoveTemp(InSession)),
	  Request(MoveTemp(InRequest)), Digest(MoveTemp(InDigest))
{
}

void FURLabRetainedCompletion::Publish(FURLabReplyEnvelope Reply,
	const FString& ResultFailureCode) const
{
	if (Reply.Kind == EURLabReplyKind::Result)
	{
		const FURLabWireStatus Validation =
			FURLabOperationSchema::ValidateResultBody(Request.Op, Reply.Result);
		if (!Validation)
		{
			Reply = FURLabReplyEnvelope::MakeError(Request, ResultFailureCode,
				TEXT("The produced result did not match its generated schema"),
				EURLabExecution::Terminal);
		}
	}
	TArray<uint8> Bytes;
	FURLabWireLimits Limits;
	Limits.MaxOutputBytes = FURLabRequestLedger::MaxInlineReplyBytes;
	FURLabWireStatus Status = FURLabRequestEnvelopeCodec::EncodeReply(Reply, Bytes, Limits);
	if (!Status)
	{
		const bool bOversize = Status.Code == EURLabWireError::OutputTooLarge;
		Reply = FURLabReplyEnvelope::MakeError(Request,
			ResultFailureCode == TEXT("encoding_error")
				? (bOversize ? TEXT("result_unavailable") : TEXT("encoding_error"))
				: ResultFailureCode,
			bOversize ? TEXT("The terminal result exceeded the inline reply limit")
				: TEXT("The terminal result could not be represented on the wire"),
			EURLabExecution::Terminal);
		Bytes.Reset();
		Status = FURLabRequestEnvelopeCodec::EncodeReply(Reply, Bytes, Limits);
	}
	checkf(Status.IsOk(), TEXT("bounded correlated terminal error must encode: %s"),
		*Status.Detail);
	Ledger->Complete(Session, static_cast<int64>(Request.RequestId), Digest, MoveTemp(Bytes));
}

void FURLabRetainedCompletion::Error(const FString& Code, const FString& Message) const
{
	Publish(FURLabReplyEnvelope::MakeError(
		Request, Code, Message, EURLabExecution::Terminal));
}

EURLabRetainedAdmissionReply FURLabRetainedCompletion::EncodeAdmission(
	const FURLabRequestLedger::FAdmissionResult& Admission,
	const FURLabRequestEnvelope& Request, TArray<uint8>& OutReplyBytes)
{
	if (Admission.Status == FURLabRequestLedger::Accepted)
	{
		return EURLabRetainedAdmissionReply::Accepted;
	}
	FURLabReplyEnvelope Reply;
	if (Admission.Status == FURLabRequestLedger::ExistingTerminal)
	{
		if (Admission.Terminal == FURLabRequestLedger::Completed && Admission.ReplyBytes)
		{
			OutReplyBytes = *Admission.ReplyBytes;
			return EURLabRetainedAdmissionReply::Replied;
		}
		Reply = FURLabReplyEnvelope::MakeError(Request, TEXT("result_unavailable"),
			TEXT("The retained result exceeded the inline reply limit"),
			EURLabExecution::Terminal);
	}
	else if (Admission.Status == FURLabRequestLedger::ExistingPending)
	{
		Reply = FURLabReplyEnvelope::MakePending(Request);
	}
	else
	{
		const TCHAR* Code = Admission.Status == FURLabRequestLedger::Conflict
			? TEXT("request_id_conflict")
			: Admission.Status == FURLabRequestLedger::Retired ? TEXT("request_retired")
			: Admission.Status == FURLabRequestLedger::Closed ? TEXT("outcome_unknown")
			: TEXT("request_capacity");
		const EURLabExecution Execution = Admission.Status == FURLabRequestLedger::Closed
			|| Admission.Status == FURLabRequestLedger::Retired
			? EURLabExecution::Unknown : EURLabExecution::Rejected;
		Reply = FURLabReplyEnvelope::MakeError(Request, Code,
			TEXT("The request ledger rejected this request identity"), Execution);
	}
	return FURLabRequestEnvelopeCodec::EncodeReply(Reply, OutReplyBytes).IsOk()
		? EURLabRetainedAdmissionReply::Replied
		: EURLabRetainedAdmissionReply::EncodingFailed;
}
