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
#include "Bridge/BridgeServer.h"

// Cooperative render-farm lease ops. A lease is a claim, not a security
// boundary: one client leases an instance so a pool won't hand the same
// editor process to two clients. One lease per process. Auto-release is
// TTL-based (an idle lease past its TTL frees on the next check); socket-level
// disconnect detection is a Phase 5 hardening item, deferred.

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleAcquireLease(const TSharedPtr<FJsonObject>& Req)
{
	UURLabBridgeServer* Bridge = OwningBridge.Get();
	if (!Bridge)
	{
		return MakeError(URLabError::NotReady,
			TEXT("lease state is unavailable (no bridge server)"));
	}

	FString Owner;
	if (Req.IsValid())
		Req->TryGetStringField(TEXT("owner"), Owner);

	double TtlSeconds = 60.0;
	if (Req.IsValid())
		Req->TryGetNumberField(TEXT("ttl_s"), TtlSeconds);

	FString AcquiredLeaseId;
	FString ExistingLeaseId;
	if (!Bridge->TryAcquireLease(Owner, TtlSeconds, AcquiredLeaseId, ExistingLeaseId))
	{
		TSharedPtr<FJsonObject> Err = MakeError(TEXT("busy"),
			TEXT("instance is already leased"));
		Err->SetStringField(TEXT("lease_id"), ExistingLeaseId);
		return Err;
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("acquire_lease_ok"));
	Reply->SetStringField(TEXT("lease_id"), AcquiredLeaseId);
	Reply->SetNumberField(TEXT("ttl_s"), TtlSeconds);
	return Reply;
}

TSharedPtr<FJsonObject> FURLabRpcDispatcher::HandleReleaseLease(const TSharedPtr<FJsonObject>& Req)
{
	UURLabBridgeServer* Bridge = OwningBridge.Get();
	if (!Bridge)
	{
		return MakeError(URLabError::NotReady,
			TEXT("lease state is unavailable (no bridge server)"));
	}

	FString LeaseId;
	if (!Req.IsValid() || !Req->TryGetStringField(TEXT("lease_id"), LeaseId) || LeaseId.IsEmpty())
	{
		return MakeError(URLabError::BadRequest, TEXT("release_lease requires a non-empty lease_id"));
	}

	if (!Bridge->ReleaseLease(LeaseId))
	{
		return MakeError(URLabError::BadRequest,
			TEXT("lease_id does not match the held lease (or no lease is held)"));
	}

	TSharedPtr<FJsonObject> Reply = MakeShared<FJsonObject>();
	Reply->SetStringField(TEXT("op"), TEXT("release_lease_ok"));
	return Reply;
}
