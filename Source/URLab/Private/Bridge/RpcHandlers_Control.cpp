// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#include "Bridge/RpcDispatcher.h"
#include "Bridge/RpcErrorCodes.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "State/MjStateTypes.h"

namespace
{
// Parse one inbound user-channel value into an IR channel. The inferred kind only
// distinguishes the text family from the numeric family; the declaring component
// stores the value under its own declared kind. Returns false with a reason for a
// shape no input kind accepts. A bare object without pos/quat (a generic struct)
// is rejected: only declared typed channels accept input.
bool JsonValueToUserChannel(const TSharedPtr<FJsonValue>& Value, FMjUserChannel& Out,
	FString& OutReason)
{
	if (!Value.IsValid())
	{
		OutReason = TEXT("null_value");
		return false;
	}
	switch (Value->Type)
	{
		case EJson::Boolean:
			Out.Kind = EMjUserChannelKind::Bool;
			Out.Values = {Value->AsBool() ? 1.0 : 0.0};
			return true;
		case EJson::Number:
			Out.Kind = EMjUserChannelKind::Scalar;
			Out.Values = {Value->AsNumber()};
			return true;
		case EJson::String:
			Out.Kind = EMjUserChannelKind::String;
			Out.Text = Value->AsString();
			return true;
		case EJson::Array:
		{
			Out.Kind = EMjUserChannelKind::Array;
			for (const TSharedPtr<FJsonValue>& E : Value->AsArray())
				Out.Values.Add(E.IsValid() ? E->AsNumber() : 0.0);
			return true;
		}
		case EJson::Object:
		{
			// {pos:[3], quat:[4]} is a transform; anything else is unsupported input.
			const TSharedPtr<FJsonObject> Obj = Value->AsObject();
			const TArray<TSharedPtr<FJsonValue>>* Pos = nullptr;
			const TArray<TSharedPtr<FJsonValue>>* Quat = nullptr;
			if (Obj.IsValid() && Obj->TryGetArrayField(TEXT("pos"), Pos)
				&& Obj->TryGetArrayField(TEXT("quat"), Quat))
			{
				Out.Kind = EMjUserChannelKind::Transform;
				Out.Values.SetNumZeroed(7);
				for (int32 i = 0; i < 3 && i < Pos->Num(); ++i)
					Out.Values[i] = (*Pos)[i].IsValid() ? (*Pos)[i]->AsNumber() : 0.0;
				for (int32 i = 0; i < 4 && i < Quat->Num(); ++i)
					Out.Values[3 + i] = (*Quat)[i].IsValid() ? (*Quat)[i]->AsNumber() : 0.0;
				return true;
			}
			OutReason = TEXT("unsupported_object");
			return false;
		}
		default:
			OutReason = TEXT("unsupported_value");
			return false;
	}
}
} // namespace

FString FURLabRpcDispatcher::ResolveControlSource(const TSharedPtr<FJsonObject>& Req) const
{
	FString Source;
	if (Req->TryGetStringField(TEXT("source"), Source) && !Source.IsEmpty())
		return Source;
	Req->TryGetStringField(TEXT("session_id"), Source);
	return Source;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::RejectIfNotControlOwner(FName ArtKey,
	const TSharedPtr<FJsonObject>& Req)
{
	const FString Source = ResolveControlSource(Req);
	FString CurrentOwner;
	if (ControlOwnership.CheckWrite(ArtKey, Source, CurrentOwner)
		== FMjControlOwnership::EWriteCheck::Ok)
	{
		return nullptr;
	}

	TSharedPtr<FJsonObject> Err = MakeError(TEXT("not_control_owner"),
		FString::Printf(TEXT("%s owned by %s"), *ArtKey.ToString(), *CurrentOwner));
	Err->SetStringField(TEXT("owner"), CurrentOwner);
	return Err;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleClaimControl(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr)
		return MakeError(URLabError::NotReady, TEXT("Manager missing"));

	FString ArtName;
	if (!Req->TryGetStringField(TEXT("articulation"), ArtName))
		return MakeError(URLabError::MissingField, TEXT("claim_control requires 'articulation'"));

	AMjArticulation* Art = Mgr->GetArticulation(ArtName);
	if (!Art)
		return MakeError(URLabError::UnknownArticulation, ArtName);

	const FName Key(*Art->GetName());
	const FString Source = ResolveControlSource(Req);

	double Ttl = 0.0;
	Req->TryGetNumberField(TEXT("ttl_s"), Ttl);
	bool bForce = false;
	Req->TryGetBoolField(TEXT("force"), bForce);

	FString CurrentOwner;
	if (ControlOwnership.Claim(Key, Source, Ttl, bForce, CurrentOwner)
		== FMjControlOwnership::EClaimResult::AlreadyOwned)
	{
		TSharedPtr<FJsonObject> Err = MakeError(TEXT("control_claimed"),
			FString::Printf(TEXT("%s already owned by %s"), *Key.ToString(), *CurrentOwner));
		Err->SetStringField(TEXT("owner"), CurrentOwner);
		return Err;
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("claim_control_ok"));
	Reply->SetStringField(TEXT("articulation"), Art->GetName());
	Reply->SetStringField(TEXT("owner"), Source);
	Reply->SetNumberField(TEXT("ttl_s"), Ttl);
	return Reply;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleReleaseControl(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr)
		return MakeError(URLabError::NotReady, TEXT("Manager missing"));

	FString ArtName;
	if (!Req->TryGetStringField(TEXT("articulation"), ArtName))
		return MakeError(URLabError::MissingField, TEXT("release_control requires 'articulation'"));

	AMjArticulation* Art = Mgr->GetArticulation(ArtName);
	if (!Art)
		return MakeError(URLabError::UnknownArticulation, ArtName);

	const FName Key(*Art->GetName());
	const FString Source = ResolveControlSource(Req);

	if (!ControlOwnership.Release(Key, Source))
	{
		FString CurrentOwner;
		ControlOwnership.CheckWrite(Key, Source, CurrentOwner);
		TSharedPtr<FJsonObject> Err = MakeError(TEXT("not_control_owner"),
			FString::Printf(TEXT("%s owned by %s"), *Key.ToString(), *CurrentOwner));
		Err->SetStringField(TEXT("owner"), CurrentOwner);
		return Err;
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("release_control_ok"));
	Reply->SetStringField(TEXT("articulation"), Art->GetName());
	return Reply;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleSetUserChannels(const TSharedPtr<FJsonObject>& Req)
{
	AAMjManager* Mgr = OwnerMgr.Get();
	if (!Mgr)
		return MakeError(URLabError::NotReady, TEXT("Manager missing"));

	int32 Applied = 0;
	TSharedPtr<FJsonObject> Rejected = MakeShared<FJsonObject>();

	// One scope's channel map: {channel: value}. ArtOrNone is the canonical art
	// segment (None for scene). Undeclared names and unparseable / kind-mismatched
	// values land in `rejected` rather than failing the whole batch.
	auto ApplyScope = [&](const TSharedPtr<FJsonObject>& Channels, FName ArtOrNone,
						  const FString& RejectPrefix) {
		if (!Channels.IsValid())
			return;
		for (const TPair<FString, TSharedPtr<FJsonValue>>& Pair : Channels->Values)
		{
			const FName Channel(*Pair.Key);
			FMjUserChannel Value;
			FString Reason;
			if (!JsonValueToUserChannel(Pair.Value, Value, Reason))
			{
				Rejected->SetStringField(RejectPrefix + Pair.Key, Reason);
				continue;
			}
			if (Mgr->ApplyUserChannelInput(ArtOrNone, Channel, Value))
				++Applied;
			else
				Rejected->SetStringField(RejectPrefix + Pair.Key,
					TEXT("undeclared_or_kind_mismatch"));
		}
	};

	const TSharedPtr<FJsonObject>* ArtsObj = nullptr;
	if (Req->TryGetObjectField(TEXT("arts"), ArtsObj))
	{
		for (const TPair<FString, TSharedPtr<FJsonValue>>& ArtPair : (*ArtsObj)->Values)
		{
			const TSharedPtr<FJsonObject>* ArtChannels = nullptr;
			if (ArtPair.Value.IsValid() && ArtPair.Value->TryGetObject(ArtChannels))
				ApplyScope(*ArtChannels, FName(*ArtPair.Key), ArtPair.Key + TEXT("/"));
		}
	}

	const TSharedPtr<FJsonObject>* SceneObj = nullptr;
	if (Req->TryGetObjectField(TEXT("scene"), SceneObj))
		ApplyScope(*SceneObj, NAME_None, TEXT("scene/"));

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("set_user_channels_ok"));
	Reply->SetNumberField(TEXT("applied"), Applied);
	Reply->SetObjectField(TEXT("rejected"), Rejected);
	return Reply;
}
