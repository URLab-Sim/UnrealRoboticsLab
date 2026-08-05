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

#include "Bridge/ControlOwnership.h"
#include "HAL/PlatformTime.h"

double FMjControlOwnership::Now() const
{
	return ClockOverrideForTest >= 0.0 ? ClockOverrideForTest : FPlatformTime::Seconds();
}

bool FMjControlOwnership::IsExpired(const FMjControlClaim& Claim, double NowSeconds)
{
	return Claim.TtlSeconds > 0.0 && (NowSeconds - Claim.LastActivitySeconds) > Claim.TtlSeconds;
}

FMjControlOwnership::EClaimResult FMjControlOwnership::Claim(FName Art, const FString& Source,
	double TtlSeconds, bool bForce, FString& OutCurrentOwner)
{
	FScopeLock Lock(&Mutex);
	const double N = Now();

	if (FMjControlClaim* Existing = Claims.Find(Art))
	{
		// An expired claim is free for the taking; a live one held by someone
		// else blocks unless the caller forces the steal.
		if (!IsExpired(*Existing, N) && !bForce && !Existing->Owner.Equals(Source))
		{
			OutCurrentOwner = Existing->Owner;
			return EClaimResult::AlreadyOwned;
		}
	}

	FMjControlClaim& Held = Claims.FindOrAdd(Art);
	Held.Owner = Source;
	Held.TtlSeconds = TtlSeconds;
	Held.LastActivitySeconds = N;
	OutCurrentOwner = Source;
	return EClaimResult::Ok;
}

bool FMjControlOwnership::Release(FName Art, const FString& Source)
{
	FScopeLock Lock(&Mutex);
	const double N = Now();

	FMjControlClaim* Existing = Claims.Find(Art);
	if (!Existing)
		return false;
	if (IsExpired(*Existing, N))
	{
		Claims.Remove(Art);
		return false;
	}
	if (!Existing->Owner.Equals(Source))
		return false;

	Claims.Remove(Art);
	return true;
}

FMjControlOwnership::EWriteCheck FMjControlOwnership::CheckWrite(FName Art, const FString& Source,
	FString& OutCurrentOwner)
{
	FScopeLock Lock(&Mutex);
	const double N = Now();

	FMjControlClaim* Existing = Claims.Find(Art);
	if (Existing && IsExpired(*Existing, N))
	{
		Claims.Remove(Art);
		Existing = nullptr;
	}

	if (!Existing)
	{
		OutCurrentOwner = TEXT("(unclaimed)");
		return EWriteCheck::NotOwner;
	}
	if (!Existing->Owner.Equals(Source))
	{
		OutCurrentOwner = Existing->Owner;
		return EWriteCheck::NotOwner;
	}

	Existing->LastActivitySeconds = N;
	OutCurrentOwner = Existing->Owner;
	return EWriteCheck::Ok;
}

TMap<FName, FString> FMjControlOwnership::GetActiveOwners()
{
	FScopeLock Lock(&Mutex);
	const double N = Now();

	TMap<FName, FString> Owners;
	for (auto It = Claims.CreateIterator(); It; ++It)
	{
		if (IsExpired(It->Value, N))
		{
			It.RemoveCurrent();
			continue;
		}
		Owners.Add(It->Key, It->Value.Owner);
	}
	return Owners;
}

void FMjControlOwnership::Reset()
{
	FScopeLock Lock(&Mutex);
	Claims.Empty();
}

void FMjControlOwnership::SetClockOverrideForTest(double NowSeconds)
{
	FScopeLock Lock(&Mutex);
	ClockOverrideForTest = NowSeconds;
}
