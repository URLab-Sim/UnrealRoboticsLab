// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "Transport/PayloadReadEncoding.h"

#include "Async/ParallelFor.h"
#include <atomic>

THIRD_PARTY_INCLUDES_START
#include "zstd.h"
THIRD_PARTY_INCLUDES_END

namespace
{
void Delta32(const uint8* Src, int64 N, uint8* Dst)
{
	const int64 Words = N / 4;
	uint32 Prev = 0;
	for (int64 K = 0; K < Words; ++K)
	{
		uint32 Word;
		FMemory::Memcpy(&Word, Src + 4 * K, 4);
		Word = INTEL_ORDER32(Word);
		const uint32 Delta = INTEL_ORDER32(Word - Prev);
		FMemory::Memcpy(Dst + 4 * K, &Delta, 4);
		Prev = Word;
	}
	for (int64 T = Words * 4; T < N; ++T)
	{
		Dst[T] = Src[T];
	}
}

void UndoDelta32(const uint8* Src, int64 N, uint8* Dst)
{
	const int64 Words = N / 4;
	uint32 Acc = 0;
	for (int64 K = 0; K < Words; ++K)
	{
		uint32 Delta;
		FMemory::Memcpy(&Delta, Src + 4 * K, 4);
		Acc += INTEL_ORDER32(Delta);
		const uint32 Out = INTEL_ORDER32(Acc);
		FMemory::Memcpy(Dst + 4 * K, &Out, 4);
	}
	for (int64 T = Words * 4; T < N; ++T)
	{
		Dst[T] = Src[T];
	}
}
} // namespace

bool URLabPayloadReadEncoding::EncodeZstdDelta32(const TArray<uint8>& Raw, FURLabPayloadReadEncoded& Out)
{
	Out.Bytes.Reset();
	Out.ChunkSizes.Reset();
	const int64 Total = Raw.Num();
	if (Total < MinEncodeBytes)
	{
		return false;
	}
	const int32 NumChunks = static_cast<int32>((Total + ChunkBytes - 1) / ChunkBytes);
	TArray<TArray<uint8>> Compressed;
	Compressed.SetNum(NumChunks);
	std::atomic<bool> bOk{true};
	ParallelFor(NumChunks, [&](int32 Index)
	{
		const int64 Begin = static_cast<int64>(Index) * ChunkBytes;
		const int64 N = FMath::Min<int64>(ChunkBytes, Total - Begin);
		TArray<uint8> Scratch;
		Scratch.SetNumUninitialized(static_cast<int32>(N));
		Delta32(Raw.GetData() + Begin, N, Scratch.GetData());
		const size_t Bound = ZSTD_compressBound(static_cast<size_t>(N));
		TArray<uint8>& Dst = Compressed[Index];
		Dst.SetNumUninitialized(static_cast<int32>(Bound));
		const size_t Written = ZSTD_compress(Dst.GetData(), Bound, Scratch.GetData(),
			static_cast<size_t>(N), /*compressionLevel=*/1);
		if (ZSTD_isError(Written))
		{
			bOk.store(false, std::memory_order_relaxed);
			return;
		}
		Dst.SetNum(static_cast<int32>(Written), EAllowShrinking::No);
	});
	if (!bOk.load(std::memory_order_relaxed))
	{
		return false;
	}
	int64 Sum = 0;
	for (const TArray<uint8>& Chunk : Compressed) Sum += Chunk.Num();
	if (Sum >= Total)
	{
		return false;
	}
	Out.Bytes.Reserve(static_cast<int32>(Sum));
	Out.ChunkSizes.Reserve(NumChunks);
	for (const TArray<uint8>& Chunk : Compressed)
	{
		Out.Bytes.Append(Chunk);
		Out.ChunkSizes.Add(Chunk.Num());
	}
	return true;
}

bool URLabPayloadReadEncoding::DecodeZstdDelta32(TConstArrayView<uint8> Encoded,
	TConstArrayView<int64> ChunkSizes, int64 RawByteLength, TArray<uint8>& OutRaw)
{
	OutRaw.SetNumUninitialized(static_cast<int32>(RawByteLength));
	TArray<uint8> Scratch;
	int64 In = 0;
	int64 OutPos = 0;
	for (const int64 Size : ChunkSizes)
	{
		const int64 N = FMath::Min<int64>(ChunkBytes, RawByteLength - OutPos);
		if (Size < 0 || In + Size > Encoded.Num() || N <= 0) return false;
		Scratch.SetNumUninitialized(static_cast<int32>(N));
		const size_t Got = ZSTD_decompress(Scratch.GetData(), static_cast<size_t>(N),
			Encoded.GetData() + In, static_cast<size_t>(Size));
		if (ZSTD_isError(Got) || static_cast<int64>(Got) != N) return false;
		UndoDelta32(Scratch.GetData(), N, OutRaw.GetData() + OutPos);
		In += Size;
		OutPos += N;
	}
	return In == Encoded.Num() && OutPos == RawByteLength;
}
