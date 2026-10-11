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

#include "Utils/AssetCache.h"

#include "HAL/FileManager.h"
#include "HAL/PlatformMisc.h"
#include "HAL/PlatformProcess.h"
#include "Misc/DateTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"

namespace
{
// --- SHA-256 (FIPS 180-4), self-contained -----------------------------------
// UE's Core exposes SHA-1 (FSHA1) but no public SHA-256; the upload protocol is
// content-addressed on SHA-256, so a small standard implementation lives here.
// It hashes raw bytes only (no secrets), so this is a plain digest, not a MAC.
struct FSha256
{
	uint32 State[8];
	uint64 BitLen = 0;
	uint8 Block[64];
	int32 BlockLen = 0;

	FSha256() { Reset(); }

	void Reset()
	{
		State[0] = 0x6a09e667u;
		State[1] = 0xbb67ae85u;
		State[2] = 0x3c6ef372u;
		State[3] = 0xa54ff53au;
		State[4] = 0x510e527fu;
		State[5] = 0x9b05688cu;
		State[6] = 0x1f83d9abu;
		State[7] = 0x5be0cd19u;
		BitLen = 0;
		BlockLen = 0;
	}

	static uint32 Rotr(uint32 X, uint32 N) { return (X >> N) | (X << (32 - N)); }

	void Transform()
	{
		static const uint32 K[64] = {
			0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u,
			0x923f82a4u, 0xab1c5ed5u, 0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u,
			0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u, 0xe49b69c1u, 0xefbe4786u,
			0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
			0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u,
			0x06ca6351u, 0x14292967u, 0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u,
			0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u, 0xa2bfe8a1u, 0xa81a664bu,
			0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
			0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au,
			0x5b9cca4fu, 0x682e6ff3u, 0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
			0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u};

		uint32 W[64];
		for (int32 i = 0; i < 16; ++i)
		{
			W[i] = (uint32(Block[i * 4]) << 24) | (uint32(Block[i * 4 + 1]) << 16)
				 | (uint32(Block[i * 4 + 2]) << 8) | uint32(Block[i * 4 + 3]);
		}
		for (int32 i = 16; i < 64; ++i)
		{
			const uint32 S0 = Rotr(W[i - 15], 7) ^ Rotr(W[i - 15], 18) ^ (W[i - 15] >> 3);
			const uint32 S1 = Rotr(W[i - 2], 17) ^ Rotr(W[i - 2], 19) ^ (W[i - 2] >> 10);
			W[i] = W[i - 16] + S0 + W[i - 7] + S1;
		}

		uint32 A = State[0], B = State[1], C = State[2], D = State[3];
		uint32 E = State[4], F = State[5], G = State[6], H = State[7];
		for (int32 i = 0; i < 64; ++i)
		{
			const uint32 S1 = Rotr(E, 6) ^ Rotr(E, 11) ^ Rotr(E, 25);
			const uint32 Ch = (E & F) ^ (~E & G);
			const uint32 T1 = H + S1 + Ch + K[i] + W[i];
			const uint32 S0 = Rotr(A, 2) ^ Rotr(A, 13) ^ Rotr(A, 22);
			const uint32 Maj = (A & B) ^ (A & C) ^ (B & C);
			const uint32 T2 = S0 + Maj;
			H = G;
			G = F;
			F = E;
			E = D + T1;
			D = C;
			C = B;
			B = A;
			A = T1 + T2;
		}
		State[0] += A;
		State[1] += B;
		State[2] += C;
		State[3] += D;
		State[4] += E;
		State[5] += F;
		State[6] += G;
		State[7] += H;
	}

	void Update(const uint8* Data, int32 Size)
	{
		for (int32 i = 0; i < Size; ++i)
		{
			Block[BlockLen++] = Data[i];
			if (BlockLen == 64)
			{
				Transform();
				BitLen += 512;
				BlockLen = 0;
			}
		}
	}

	FString Finalize()
	{
		const uint64 TotalBits = BitLen + uint64(BlockLen) * 8;
		Block[BlockLen++] = 0x80;
		if (BlockLen > 56)
		{
			while (BlockLen < 64)
				Block[BlockLen++] = 0;
			Transform();
			BlockLen = 0;
		}
		while (BlockLen < 56)
			Block[BlockLen++] = 0;
		for (int32 i = 7; i >= 0; --i)
			Block[BlockLen++] = uint8((TotalBits >> (i * 8)) & 0xff);
		Transform();

		FString Hex;
		Hex.Reserve(64);
		const TCHAR* Digits = TEXT("0123456789abcdef");
		for (int32 i = 0; i < 8; ++i)
		{
			for (int32 Shift = 28; Shift >= 0; Shift -= 4)
				Hex.AppendChar(Digits[(State[i] >> Shift) & 0xf]);
		}
		return Hex;
	}
};
} // namespace

FURLabAssetCache::FURLabAssetCache(const FString& InRoot)
	: Root(InRoot)
{
}

FURLabAssetCache& FURLabAssetCache::Get()
{
	static FURLabAssetCache Instance(ResolveCacheRoot());
	return Instance;
}

FString FURLabAssetCache::ResolveCacheRoot()
{
	const FString Override = FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_ASSET_CACHE"));
	if (!Override.IsEmpty())
		return Override;

#if PLATFORM_WINDOWS
	FString Base = FPlatformMisc::GetEnvironmentVariable(TEXT("LOCALAPPDATA"));
	if (Base.IsEmpty())
		Base = FPlatformProcess::UserSettingsDir();
#else
	FString Base = FPlatformMisc::GetEnvironmentVariable(TEXT("XDG_CACHE_HOME"));
	if (Base.IsEmpty())
		Base = FPaths::Combine(FPlatformMisc::GetEnvironmentVariable(TEXT("HOME")), TEXT(".cache"));
#endif
	return FPaths::Combine(Base, TEXT("URLab"), TEXT("cache"));
}

FString FURLabAssetCache::Sha256Hex(const uint8* Data, int32 Size)
{
	FSha256 Ctx;
	if (Data && Size > 0)
		Ctx.Update(Data, Size);
	return Ctx.Finalize();
}

bool FURLabAssetCache::IsValidSha256Hex(const FString& Candidate)
{
	if (Candidate.Len() != 64)
		return false;
	for (const TCHAR C : Candidate)
	{
		const bool bHex = (C >= '0' && C <= '9') || (C >= 'a' && C <= 'f');
		if (!bHex)
			return false;
	}
	return true;
}

FString FURLabAssetCache::BlobPath(const FString& Sha256Hex) const
{
	return FPaths::Combine(Root, TEXT("blobs"), Sha256Hex.Left(2), Sha256Hex);
}

bool FURLabAssetCache::Has(const FString& Sha256Hex) const
{
	if (!IsValidSha256Hex(Sha256Hex))
		return false;
	return IFileManager::Get().FileExists(*BlobPath(Sha256Hex));
}

bool FURLabAssetCache::GetPath(const FString& Sha256Hex, FString& OutPath) const
{
	if (!Has(Sha256Hex))
		return false;
	OutPath = BlobPath(Sha256Hex);
	return true;
}

bool FURLabAssetCache::Put(const FString& Sha256Hex, const uint8* Data, int32 Size)
{
	if (!IsValidSha256Hex(Sha256Hex))
		return false;

	const FString Final = BlobPath(Sha256Hex);
	if (IFileManager::Get().FileExists(*Final))
		return true; // immutable content already present

	const FString Dir = FPaths::GetPath(Final);
	IFileManager::Get().MakeDirectory(*Dir, /*Tree=*/true);

	// Temp file in the SAME directory so the rename is an atomic same-volume
	// move. A distinct GUID per writer keeps concurrent writers of the same
	// hash from clobbering each other's temp file.
	const FString Temp = Final + TEXT(".") + FGuid::NewGuid().ToString(EGuidFormats::Digits) + TEXT(".tmp");

	TArray<uint8> Payload;
	if (Data && Size > 0)
		Payload.Append(Data, Size);
	if (!FFileHelper::SaveArrayToFile(Payload, *Temp))
		return false;

	// bReplace=true: content is identical across writers, so replacing a blob a
	// racing process just created is harmless and keeps this call idempotent.
	if (!IFileManager::Get().Move(*Final, *Temp, /*bReplace=*/true))
	{
		// A concurrent writer may have won the rename; treat an existing final
		// blob as success and drop our temp copy.
		IFileManager::Get().Delete(*Temp, /*RequireExists=*/false, /*EvenReadOnly=*/true);
		return IFileManager::Get().FileExists(*Final);
	}
	return true;
}

int64 FURLabAssetCache::EvictToBudget(int64 MaxBytes)
{
	if (MaxBytes <= 0)
	{
		const FString Env = FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_ASSET_CACHE_MAX_BYTES"));
		MaxBytes = Env.IsEmpty() ? 0 : FCString::Atoi64(*Env);
	}
	if (MaxBytes <= 0)
		return 0; // eviction disabled

	const FString BlobsDir = FPaths::Combine(Root, TEXT("blobs"));

	struct FBlob
	{
		FString Path;
		int64 Size = 0;
		FDateTime Modified;
	};
	TArray<FBlob> Blobs;
	int64 Total = 0;
	IFileManager::Get().IterateDirectoryStatRecursively(*BlobsDir,
		[&Blobs, &Total](const TCHAR* Path, const FFileStatData& Stat) {
			if (!Stat.bIsDirectory && Stat.FileSize > 0)
			{
				Blobs.Add({Path, Stat.FileSize, Stat.ModificationTime});
				Total += Stat.FileSize;
			}
			return true;
		});

	if (Total <= MaxBytes)
		return 0;

	Blobs.Sort([](const FBlob& A, const FBlob& B) { return A.Modified < B.Modified; });

	int64 Freed = 0;
	for (const FBlob& B : Blobs)
	{
		if (Total - Freed <= MaxBytes)
			break;
		if (IFileManager::Get().Delete(*B.Path, /*RequireExists=*/false, /*EvenReadOnly=*/true))
			Freed += B.Size;
	}
	return Freed;
}
