// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#pragma once

#include "CoreMinimal.h"
#include "Bridge/RequestLedger.h"
#include "Wire/RequestEnvelope.h"

/** Result of translating a request-ledger admission into an immediate wire reply. */
enum class EURLabRetainedAdmissionReply : uint8
{
	Accepted,
	Replied,
	EncodingFailed
};

/**
 * Owned completion for one admitted typed operation.
 *
 * The object carries no dispatcher pointer. A domain queue may retain it across
 * disconnect or dispatcher destruction; completion updates only the exact ledger
 * session and validates the result against the generated schema before encoding.
 */
class URLAB_API FURLabRetainedCompletion
{
public:
	FURLabRetainedCompletion(
		TSharedRef<FURLabRequestLedger, ESPMode::ThreadSafe> InLedger,
		FURLabRequestLedger::FSessionRef InSession,
		FURLabRequestEnvelope InRequest, FURLabRequestDigest InDigest);

	void Publish(FURLabReplyEnvelope Reply,
		const FString& ResultFailureCode = TEXT("encoding_error")) const;
	void Error(const FString& Code, const FString& Message) const;

	/** Encode every non-Accepted ledger disposition without executing domain work. */
	static EURLabRetainedAdmissionReply EncodeAdmission(
		const FURLabRequestLedger::FAdmissionResult& Admission,
		const FURLabRequestEnvelope& Request, TArray<uint8>& OutReplyBytes);

	TSharedRef<FURLabRequestLedger, ESPMode::ThreadSafe> Ledger;
	FURLabRequestLedger::FSessionRef Session;
	FURLabRequestEnvelope Request;
	FURLabRequestDigest Digest;
};
