// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#pragma once

#include "CoreMinimal.h"

/** Output of an encoded payload_read reply. */
struct URLAB_API FURLabPayloadReadEncoded
{
	/** Concatenated, independently decodable compressed chunks. */
	TArray<uint8> Bytes;
	/** Compressed size of each chunk, in order. */
	TArray<int64> ChunkSizes;
};

/**
 * Optional lossless transfer encodings for network payload_read replies.
 *
 * "zstd_delta32": the range is cut into ChunkBytes-sized chunks (last may be
 * shorter). Each chunk is transformed and zstd-compressed independently so both
 * ends can work on all chunks in parallel:
 *   1. view the chunk's first floor(n/4)*4 bytes as little-endian uint32 words
 *      and delta-code them: d[0] = w[0], d[k] = w[k] - w[k-1] (mod 2^32), so the
 *      delta restarts in every chunk; the 0..3 tail bytes follow raw;
 *   2. zstd level 1.
 * For RGBA8 frames a word is one pixel, so this is a previous-pixel predictor:
 * ~2.6x on rendered images, and the inverse is a plain uint32 prefix sum, which
 * NumPy runs at memory speed (a byte-plane variant compressed ~10% better but
 * decoded ~12x slower in Python).
 */
namespace URLabPayloadReadEncoding
{
inline const TCHAR* const ZstdDelta32 = TEXT("zstd_delta32");
inline constexpr int32 ChunkBytes = 4 * 1024 * 1024;
/** Ranges smaller than this are always returned raw. */
inline constexpr int32 MinEncodeBytes = 256 * 1024;

/** Encodes Raw. Returns false (caller should send raw) when the range is small,
 *  compression fails, or the result would not be smaller. */
URLAB_API bool EncodeZstdDelta32(const TArray<uint8>& Raw, FURLabPayloadReadEncoded& Out);
/** Exact inverse, used by tests. */
URLAB_API bool DecodeZstdDelta32(TConstArrayView<uint8> Encoded, TConstArrayView<int64> ChunkSizes,
	int64 RawByteLength, TArray<uint8>& OutRaw);
}
