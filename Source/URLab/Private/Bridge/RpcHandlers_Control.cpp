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
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"

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
		return MakeError(TEXT("not_ready"), TEXT("Manager missing"));

	FString ArtName;
	if (!Req->TryGetStringField(TEXT("articulation"), ArtName))
		return MakeError(TEXT("missing_field"), TEXT("claim_control requires 'articulation'"));

	AMjArticulation* Art = Mgr->GetArticulation(ArtName);
	if (!Art)
		return MakeError(TEXT("unknown_articulation"), ArtName);

	const FName Key(*Art->GetName());
	const FString Source = ResolveControlSource(Req);

	double Ttl = 0.0;
	Req->TryGetNumberField(TEXT("ttl_s"), Ttl);
	bool bExclusive = true;
	Req->TryGetBoolField(TEXT("exclusive"), bExclusive);
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
		return MakeError(TEXT("not_ready"), TEXT("Manager missing"));

	FString ArtName;
	if (!Req->TryGetStringField(TEXT("articulation"), ArtName))
		return MakeError(TEXT("missing_field"), TEXT("release_control requires 'articulation'"));

	AMjArticulation* Art = Mgr->GetArticulation(ArtName);
	if (!Art)
		return MakeError(TEXT("unknown_articulation"), ArtName);

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
