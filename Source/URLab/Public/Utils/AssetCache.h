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

/**
 * @class FURLabAssetCache
 * @brief Content-addressed, on-disk blob store shared across farm instances.
 *
 * A blob lives at `{root}/blobs/{sha[:2]}/{sha}`; its content is its key, so
 * reads need no lock and are immutable. Writes are atomic (temp file in the
 * same directory, then rename), which makes two processes storing the same
 * hash safe — the bytes are identical, so whichever rename wins, the result
 * is correct. The manifest step of the upload protocol resolves each asset
 * hash here to skip re-sending bytes an instance already holds.
 *
 * The default root comes from `URLAB_ASSET_CACHE`, else
 * `%LOCALAPPDATA%/URLab/cache` on Windows and `~/.cache/URLab/cache`
 * (honouring `XDG_CACHE_HOME`) on Linux.
 */
class URLAB_API FURLabAssetCache
{
public:
	/** Cache rooted at an explicit directory (used by tests). */
	explicit FURLabAssetCache(const FString& InRoot);

	/** Process-wide cache rooted at ResolveCacheRoot(). */
	static FURLabAssetCache& Get();

	/** True if a verified blob for this hash is already stored. Lock-free. */
	bool Has(const FString& Sha256Hex) const;

	/** Resolve a stored blob to its on-disk path. Returns false if absent. */
	bool GetPath(const FString& Sha256Hex, FString& OutPath) const;

	/** Store bytes under their (already-computed) content hash. Atomic: writes
	 *  a temp file in the blob's directory then renames it into place, so a
	 *  concurrent writer of the same hash is harmless. Returns false only on a
	 *  filesystem error. */
	bool Put(const FString& Sha256Hex, const uint8* Data, int32 Size);
	bool Put(const FString& Sha256Hex, const TArray<uint8>& Data)
	{
		return Put(Sha256Hex, Data.GetData(), Data.Num());
	}

	/** Root directory of this cache instance. */
	const FString& GetRoot() const { return Root; }

	/** Best-effort LRU eviction: if the total blob size exceeds MaxBytes, delete
	 *  the least-recently-used blobs (by mtime) until back under the cap. A
	 *  MaxBytes <= 0 reads URLAB_ASSET_CACHE_MAX_BYTES (0/unset = no eviction).
	 *  Returns the number of bytes freed. */
	int64 EvictToBudget(int64 MaxBytes = 0);

	/** Default cache root (env override, else per-OS user cache dir). */
	static FString ResolveCacheRoot();

	/** Lowercase-hex SHA-256 of a byte range. */
	static FString Sha256Hex(const uint8* Data, int32 Size);
	static FString Sha256Hex(const TArray<uint8>& Data)
	{
		return Sha256Hex(Data.GetData(), Data.Num());
	}

	/** True if the string is a syntactically valid lowercase SHA-256 hex digest
	 *  (64 chars, [0-9a-f]). */
	static bool IsValidSha256Hex(const FString& Candidate);

private:
	FString BlobPath(const FString& Sha256Hex) const;

	FString Root;
};
