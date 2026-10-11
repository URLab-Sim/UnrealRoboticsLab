#pragma once

#include "Bridge/RetainedCompletion.h"
#include "Workflows/InteractionInbox.h"

namespace URLabInteractionWire
{
using V = FURLabWireValue;

inline bool ParseSource(const V& Value, FURLabInteractionSourceIdentity& Out)
{
	Out.ProducerBootId = Value.FindMapValue(TEXT("producer_boot_id"))->GetString();
	Out.StreamId = Value.FindMapValue(TEXT("stream_id"))->GetString();
	Out.StreamGeneration = Value.FindMapValue(TEXT("stream_generation"))->GetInt();
	Out.SourceRevision = Value.FindMapValue(TEXT("source_revision"))->GetString();
	return Out.IsValid();
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
	Out.GrantToken = Value.FindMapValue(TEXT("grant_token"))->GetString();
	Out.ViewerBootId = Value.FindMapValue(TEXT("viewer_boot_id"))->GetString();
	Out.ClockEpoch = Value.FindMapValue(TEXT("clock_epoch"))->GetInt();
	return ParseSource(*Value.FindMapValue(TEXT("source")), Out.Source);
}

inline bool ParseWrench(const V& Value, FURLabInteractionWrenchRequest& Out)
{
	Out.BodyIndex = Value.FindMapValue(TEXT("body_index"))->GetInt();
	const TArray<V>& Force = Value.FindMapValue(TEXT("force_n"))->GetArray();
	const TArray<V>& Torque = Value.FindMapValue(TEXT("torque_nm"))->GetArray();
	if (Force.Num() != 3 || Torque.Num() != 3) return false;
	Out.ForceNewton = FVector3d(Force[0].GetFloat(), Force[1].GetFloat(), Force[2].GetFloat());
	Out.TorqueNewtonMetre = FVector3d(
		Torque[0].GetFloat(), Torque[1].GetFloat(), Torque[2].GetFloat());
	return true;
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
		const TCHAR* Code = Result.Terminal == EURLabInteractionTerminal::RejectedTarget
			? TEXT("target_invalid") : Result.Terminal == EURLabInteractionTerminal::RejectedStale
			? TEXT("grant_invalid") : TEXT("interaction_failed");
		Completion->Error(Code, Result.Error);
	}

private:
	TSharedRef<FURLabRetainedCompletion, ESPMode::ThreadSafe> Completion;
};
}
