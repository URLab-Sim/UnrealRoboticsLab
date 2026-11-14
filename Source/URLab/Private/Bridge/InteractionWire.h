#pragma once

#include "Bridge/RetainedCompletion.h"
#include "Workflows/InteractionInbox.h"

namespace URLabInteractionWire
{
using V = FURLabWireValue;

inline bool ParseString(const V& Value, const TCHAR* Name, FString& Out)
{
	const V* Field = Value.FindMapValue(Name);
	if (!Field) return false;
	Out = Field->GetString();
	return true;
}

inline bool ParseUInt(const V& Value, const TCHAR* Name, uint64& Out)
{
	const V* Field = Value.FindMapValue(Name);
	if (!Field || Field->GetInt() < 0) return false;
	Out = static_cast<uint64>(Field->GetInt());
	return true;
}

inline bool ParseVector(const V* Field, FVector3d& Out)
{
	if (!Field) return false;
	const TArray<V>& Items = Field->GetArray();
	if (Items.Num() != 3) return false;
	Out = FVector3d(Items[0].GetFloat(), Items[1].GetFloat(), Items[2].GetFloat());
	return true;
}

inline bool ParseSource(const V& Value, FURLabInteractionSourceIdentity& Out)
{
	return ParseString(Value, TEXT("producer_boot_id"), Out.ProducerBootId)
		&& ParseString(Value, TEXT("stream_id"), Out.StreamId)
		&& ParseUInt(Value, TEXT("stream_generation"), Out.StreamGeneration)
		&& ParseString(Value, TEXT("source_revision"), Out.SourceRevision)
		&& Out.IsValid();
}

inline V Source(const FURLabInteractionSourceIdentity& Value)
{
	return V::MakeMap(
		{TEXT("producer_boot_id"), TEXT("stream_id"), TEXT("stream_generation"),
			TEXT("source_revision")},
		{V::MakeString(Value.ProducerBootId), V::MakeString(Value.StreamId),
			V::MakeInt(static_cast<int64>(Value.StreamGeneration)),
			V::MakeString(Value.SourceRevision)});
}

inline bool ParseGrant(const V& Value, FURLabInteractionWrenchRequest& Out)
{
	const V* SourceValue = Value.FindMapValue(TEXT("source"));
	return SourceValue && ParseString(Value, TEXT("grant_token"), Out.GrantToken)
		&& ParseString(Value, TEXT("viewer_boot_id"), Out.ViewerBootId)
		&& ParseUInt(Value, TEXT("clock_epoch"), Out.ClockEpoch)
		&& ParseSource(*SourceValue, Out.Source);
}

/** Parses the wrench body, including its optional frame, point, hold and identity. */
inline bool ParseWrench(const V& Value, FURLabInteractionWrenchRequest& Out)
{
	const V* Body = Value.FindMapValue(TEXT("body_index"));
	if (!Body || Body->GetInt() < 0 || Body->GetInt() > MAX_int32) return false;
	Out.BodyIndex = static_cast<int32>(Body->GetInt());
	if (!ParseVector(Value.FindMapValue(TEXT("force_n")), Out.ForceNewton)
		|| !ParseVector(Value.FindMapValue(TEXT("torque_nm")), Out.TorqueNewtonMetre))
	{
		return false;
	}
	if (const V* Frame = Value.FindMapValue(TEXT("body_frame"))) Out.bBodyFrame = Frame->GetBool();
	if (const V* Point = Value.FindMapValue(TEXT("point_m")))
	{
		if (!ParseVector(Point, Out.PointMetre)) return false;
		Out.bHasPoint = true;
	}
	if (const V* Duration = Value.FindMapValue(TEXT("duration_s")))
		Out.DurationSeconds = Duration->GetFloat();
	if (const V* Channel = Value.FindMapValue(TEXT("channel"))) Out.Channel = Channel->GetString();
	if (const V* Id = Value.FindMapValue(TEXT("interaction_id"))) Out.InteractionId = Id->GetString();
	return true;
}

/** Wire error code for an admission refusal; empty for Accepted/Existing*. */
inline const TCHAR* AdmissionCode(EURLabInteractionAdmission Admission)
{
	switch (Admission)
	{
	case EURLabInteractionAdmission::Unsupported: return TEXT("unsupported");
	case EURLabInteractionAdmission::Invalid: return TEXT("bad_request");
	case EURLabInteractionAdmission::StaleGrant: return TEXT("grant_invalid");
	case EURLabInteractionAdmission::ExpiredGrant: return TEXT("grant_expired");
	case EURLabInteractionAdmission::StaleSource: return TEXT("source_revision_conflict");
	case EURLabInteractionAdmission::Conflict: return TEXT("interaction_conflict");
	case EURLabInteractionAdmission::Capacity: return TEXT("service_unavailable");
	default: return TEXT("");
	}
}

inline const TCHAR* TerminalCode(EURLabInteractionTerminal Terminal)
{
	switch (Terminal)
	{
	case EURLabInteractionTerminal::RejectedTarget: return TEXT("target_invalid");
	case EURLabInteractionTerminal::RejectedStale: return TEXT("grant_invalid");
	case EURLabInteractionTerminal::RejectedExpired: return TEXT("grant_expired");
	case EURLabInteractionTerminal::RejectedSource: return TEXT("source_revision_conflict");
	default: return TEXT("apply_failed");
	}
}

inline V Grant(const FURLabInteractionGrant& Value, double ExpiresInSeconds)
{
	return V::MakeMap(
		{TEXT("grant_token"), TEXT("source"), TEXT("viewer_boot_id"),
			TEXT("clock_epoch"), TEXT("expires_in_s")},
		{V::MakeString(Value.Token), Source(Value.Source), V::MakeString(Value.ViewerBootId),
			V::MakeInt(static_cast<int64>(Value.ClockEpoch)), V::MakeFloat(ExpiresInSeconds)});
}

class FCompletionSink final : public IURLabInteractionCompletionSink
{
public:
	explicit FCompletionSink(
		TSharedRef<FURLabRetainedCompletion, ESPMode::ThreadSafe> InCompletion)
		: Completion(MoveTemp(InCompletion))
	{
	}

	virtual void CompleteInteraction(const FString&,
		const FURLabInteractionResult& Result) override
	{
		if (Result.Terminal == EURLabInteractionTerminal::Applied)
		{
			Completion->Publish(FURLabReplyEnvelope::MakeResult(Completion->Request,
				V::MakeMap(
					{TEXT("status"), TEXT("frame_id"), TEXT("clock_epoch"),
						TEXT("has_source_time"), TEXT("source_time")},
					{V::MakeString(TEXT("applied")),
						V::MakeInt(static_cast<int64>(Result.AppliedFrame.FrameId)),
						V::MakeInt(static_cast<int64>(Result.AppliedFrame.ClockEpoch)),
						V::MakeBool(Result.AppliedFrame.bHasSourceTime),
							V::MakeFloat(Result.AppliedFrame.SourceTime)})));
			return;
		}
		Completion->Error(TerminalCode(Result.Terminal), Result.Error);
	}

private:
	TSharedRef<FURLabRetainedCompletion, ESPMode::ThreadSafe> Completion;
};
}
