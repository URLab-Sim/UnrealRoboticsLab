// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#pragma once

#include "CoreMinimal.h"

/** Owned canonical SHA-256 digest for a validated protocol-2 request. */
struct URLAB_API FURLabRequestDigest
{
	static constexpr int32 NumBytes = 32;

	uint8 Bytes[NumBytes]{};

	bool operator==(const FURLabRequestDigest& Other) const
	{
		return FMemory::Memcmp(Bytes, Other.Bytes, NumBytes) == 0;
	}

	bool operator!=(const FURLabRequestDigest& Other) const
	{
		return !(*this == Other);
	}

	/** Lowercase hexadecimal for logs and fixtures. Raw bytes remain primary. */
	FString ToHex() const;
};

inline uint32 GetTypeHash(const FURLabRequestDigest& Digest)
{
	uint32 Hash = 0;
	for (const uint8 Byte : Digest.Bytes)
	{
		Hash = HashCombineFast(Hash, ::GetTypeHash(Byte));
	}
	return Hash;
}
