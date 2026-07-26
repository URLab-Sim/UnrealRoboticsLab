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

#pragma once

#include "CoreMinimal.h"

/** One articulation's control claim: who owns it, its TTL, and the last time
 *  the owner touched it. A zero TTL never expires. */
struct FMjControlClaim
{
	FString Owner;                 // source id
	bool bExclusive = true;
	double TtlSeconds = 0.0;       // 0 = no expiry
	double LastActivitySeconds = 0.0;
};

/**
 * Per-articulation control arbitration. Every control write (ctrl, xfrc,
 * twist, qpos, mocap, control-source flip) carries a source id and must own
 * the target articulation before it applies; observation is never gated.
 *
 * An unclaimed articulation rejects control writes — control must always be
 * explicitly owned so RPC, Python, and ROS surfaces coexist without silently
 * fighting over the same robot. Claims are cooperative, not a security
 * boundary: a TTL auto-frees a crashed owner, and `bForce` steals for operator
 * override / crash recovery.
 *
 * Multiple RPC transport threads (ZMQ + SHM, and later a ROS executor) reach
 * this concurrently, so every operation takes the mutex. Expiry is lazy —
 * evaluated on the next Claim / CheckWrite for an articulation rather than on a
 * timer.
 */
class URLAB_API FMjControlOwnership
{
public:
	enum class EClaimResult : uint8 { Ok, AlreadyOwned };

	/** Claim `Art` for `Source`. `bForce` steals an existing claim (operator
	 *  override). Returns AlreadyOwned and fills OutCurrentOwner when the art is
	 *  held by another live source and force is not set. */
	EClaimResult Claim(FName Art, const FString& Source, double TtlSeconds,
		bool bForce, FString& OutCurrentOwner);

	/** Release `Art` when `Source` owns it. Returns false when the source is
	 *  not the current owner (including an already-expired or absent claim). */
	bool Release(FName Art, const FString& Source);

	enum class EWriteCheck : uint8 { Ok, NotOwner };

	/** The gate for every control write. Ok refreshes LastActivity (the
	 *  heartbeat). Unclaimed art => NotOwner (unclaimed = external control
	 *  ignored). Fills OutCurrentOwner on NotOwner when another source holds it. */
	EWriteCheck CheckWrite(FName Art, const FString& Source, FString& OutCurrentOwner);

	/** Snapshot of currently-held (non-expired) claims, art -> owner. Lazily
	 *  expires stale claims first. Used by the global control-source flip, which
	 *  must own every claimed art (or none may be claimed). */
	TMap<FName, FString> GetActiveOwners();

	/** Drop every claim. Called on OnManagerGone / Shutdown: claims are per-PIE
	 *  because the articulations die with the world. */
	void Reset();

	/** Pin the clock to a fixed value so TTL expiry is deterministic without
	 *  sleeping. A negative value restores the wall clock. */
	void SetClockOverrideForTest(double NowSeconds);

private:
	/** Current time: the test override when set (>= 0), else the wall clock.
	 *  Callers hold Mutex. */
	double Now() const;

	/** True when a claim has a live TTL and has been idle past it. */
	static bool IsExpired(const FMjControlClaim& Claim, double NowSeconds);

	FCriticalSection Mutex;             // RPC threads: ZMQ + SHM (+ ROS later)
	TMap<FName, FMjControlClaim> Claims;
	double ClockOverrideForTest = -1.0;
};
