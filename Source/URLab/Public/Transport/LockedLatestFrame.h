// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#pragma once

#include "CoreMinimal.h"
#include "Transport/ShmExactPayload.h"

/** URLVSTRM layout: 48-byte header whose last u64 is a writer-only sequence lock. */
inline constexpr uint32 URLabLatestFrameLayoutVersion = 2;

enum class EURLabLatestFrameRead : uint8
{
	NoFrame,
	Frame,
	Error
};

/** Nonblocking coalescing writer for URLVSTRM layout 2. Readers never delay it. */
class URLAB_API FURLabLockedLatestFrameWriter
{
public:
	static TUniquePtr<FURLabLockedLatestFrameWriter> Create(
		FURLabShmLiveObjectPtr Object, uint32 MaxPayloadBytes, FString& OutError);
	~FURLabLockedLatestFrameWriter();

	bool Submit(TSharedRef<const TArray<uint8>, ESPMode::ThreadSafe> ImmutableFrame,
		FString& OutError);
	/** Generation barrier: discard old work and atomically publish no frame. Never waits for readers. */
	bool Clear(FString& OutError);
	void Stop();

private:
	struct FImpl;
	explicit FURLabLockedLatestFrameWriter(TUniquePtr<FImpl>&& InImpl);
	TUniquePtr<FImpl> Impl;
};

/** Bounded nonblocking copied reader for URLVSTRM layout 2. */
class URLAB_API FURLabLockedLatestFrameReader
{
public:
	static TUniquePtr<FURLabLockedLatestFrameReader> Create(
		FURLabShmLiveObjectPtr Object, uint32 MaxPayloadBytes, FString& OutError);
	~FURLabLockedLatestFrameReader();

	EURLabLatestFrameRead ReadLatest(TArray<uint8>& OutFrame, FString& OutError);

private:
	struct FImpl;
	explicit FURLabLockedLatestFrameReader(TUniquePtr<FImpl>&& InImpl);
	TUniquePtr<FImpl> Impl;
};
