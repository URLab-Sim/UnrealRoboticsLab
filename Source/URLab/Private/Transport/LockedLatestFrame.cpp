// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#include "Transport/LockedLatestFrame.h"

#include <condition_variable>
#include <mutex>
#include <thread>

#if PLATFORM_LINUX
#include <cerrno>
#include <unistd.h>
#elif PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#include <Windows.h>
#include "Windows/HideWindowsPlatformTypes.h"
#endif

namespace
{
constexpr uint32 HeaderBytes = 48;
constexpr uint32 FieldBytes = 40;
constexpr uint64 SeqlockOffset = 40;
constexpr int32 ReadAttempts = 4;
constexpr uint8 Magic[8] = {'U','R','L','V','S','T','R','M'};

void Put32(uint8* Out, uint32 Value)
{
	for (int32 Index = 0; Index < 4; ++Index) Out[Index] = uint8(Value >> (Index * 8));
}

void Put64(uint8* Out, uint64 Value)
{
	for (int32 Index = 0; Index < 8; ++Index) Out[Index] = uint8(Value >> (Index * 8));
}

uint32 Get32(const uint8* In)
{
	uint32 Value = 0;
	for (int32 Index = 0; Index < 4; ++Index) Value |= uint32(In[Index]) << (Index * 8);
	return Value;
}

uint64 Get64(const uint8* In)
{
	uint64 Value = 0;
	for (int32 Index = 0; Index < 8; ++Index) Value |= uint64(In[Index]) << (Index * 8);
	return Value;
}

bool WriteExactAt(UPTRINT NativeHandle, const uint8* Data, uint32 Length,
	uint64 Offset, FString& Error)
{
#if PLATFORM_LINUX
	uint32 Done = 0;
	while (Done < Length)
	{
		const ssize_t Result = ::pwrite(static_cast<int>(NativeHandle), Data + Done,
			Length - Done, static_cast<off_t>(Offset + Done));
		if (Result < 0 && errno == EINTR) continue;
		if (Result <= 0) { Error = TEXT("short latest-frame write"); return false; }
		Done += static_cast<uint32>(Result);
	}
	return true;
#elif PLATFORM_WINDOWS
	OVERLAPPED Overlapped = {};
	Overlapped.Offset = static_cast<DWORD>(Offset);
	Overlapped.OffsetHigh = static_cast<DWORD>(Offset >> 32);
	DWORD Written = 0;
	if (!::WriteFile(reinterpret_cast<HANDLE>(NativeHandle), Data, Length, &Written, &Overlapped)
		|| Written != Length) { Error = TEXT("short latest-frame write"); return false; }
	return true;
#else
	Error = TEXT("latest-frame writes are unsupported on this platform"); return false;
#endif
}

bool ReadExactAt(UPTRINT NativeHandle, uint8* Data, uint32 Length,
	uint64 Offset, FString& Error)
{
#if PLATFORM_LINUX
	uint32 Done = 0;
	while (Done < Length)
	{
		const ssize_t Result = ::pread(static_cast<int>(NativeHandle), Data + Done,
			Length - Done, static_cast<off_t>(Offset + Done));
		if (Result < 0 && errno == EINTR) continue;
		if (Result <= 0) { Error = TEXT("short latest-frame read"); return false; }
		Done += static_cast<uint32>(Result);
	}
	return true;
#elif PLATFORM_WINDOWS
	OVERLAPPED Overlapped = {};
	Overlapped.Offset = static_cast<DWORD>(Offset);
	Overlapped.OffsetHigh = static_cast<DWORD>(Offset >> 32);
	DWORD Read = 0;
	if (!::ReadFile(reinterpret_cast<HANDLE>(NativeHandle), Data, Length, &Read, &Overlapped)
		|| Read != Length) { Error = TEXT("short latest-frame read"); return false; }
	return true;
#else
	Error = TEXT("latest-frame reads are unsupported on this platform"); return false;
#endif
}

bool ValidLiveObject(const FURLabShmLiveObjectPtr& Object, uint32 MaxPayloadBytes)
{
	return Object && MaxPayloadBytes != 0
		&& Object->GetDescriptor().RegionGeneration != 0
		&& Object->GetDescriptor().RegionGeneration <= static_cast<uint64>(MAX_int64)
		&& Object->GetDescriptor().ByteLength == int64(HeaderBytes) + MaxPayloadBytes;
}
}

struct FURLabLockedLatestFrameWriter::FImpl
{
	struct FPending
	{
		TSharedPtr<const TArray<uint8>, ESPMode::ThreadSafe> Frame;
		uint64 Epoch = 0;
	};
	FURLabShmLiveObjectPtr Object;
	uint32 Capacity = 0;
	uint64 Sequence = 0;
	uint64 Seqlock = 0;
	uint64 Epoch = 1;
	std::mutex PendingMutex;
	std::mutex WriteMutex;
	std::condition_variable Wakeup;
	TOptional<FPending> Pending;
	bool bStopped = false;
	std::thread Worker;

	FImpl(FURLabShmLiveObjectPtr InObject, uint32 InCapacity)
		: Object(MoveTemp(InObject)), Capacity(InCapacity)
	{
	}

	/** Seqlock publication; the caller holds WriteMutex. Readers never block it. */
	bool Publish(const uint8* Fields, const uint8* Payload, uint32 PayloadBytes, FString& Error)
	{
		uint8 Lock[8];
		Put64(Lock, ++Seqlock);
		if (!WriteExactAt(Object->NativeHandle, Lock, sizeof(Lock), SeqlockOffset, Error)
			|| (Payload && !WriteExactAt(Object->NativeHandle, Payload, PayloadBytes,
				HeaderBytes, Error))
			|| !WriteExactAt(Object->NativeHandle, Fields, FieldBytes, 0, Error))
			return false;
		Put64(Lock, ++Seqlock);
		return WriteExactAt(Object->NativeHandle, Lock, sizeof(Lock), SeqlockOffset, Error);
	}

	void Run()
	{
		for (;;)
		{
			FPending Candidate;
			{
				std::unique_lock<std::mutex> Lock(PendingMutex);
				Wakeup.wait(Lock, [this] { return bStopped || Pending.IsSet(); });
				if (bStopped) { Pending.Reset(); return; }
				Candidate = MoveTemp(Pending.GetValue());
				Pending.Reset();
			}
			std::lock_guard<std::mutex> WriteLock(WriteMutex);
			{
				std::lock_guard<std::mutex> Lock(PendingMutex);
				if (bStopped || Candidate.Epoch != Epoch) continue;
			}
			if (Sequence == static_cast<uint64>(MAX_int64))
			{
				StopFromWorker();
				continue;
			}
			const uint64 NextSequence = Sequence + 1;
			uint8 Fields[FieldBytes] = {};
			FMemory::Memcpy(Fields, Magic, sizeof(Magic));
			Put32(Fields + 8, URLabLatestFrameLayoutVersion);
			Put32(Fields + 12, HeaderBytes);
			Put64(Fields + 16, Object->GetDescriptor().RegionGeneration);
			Put64(Fields + 24, NextSequence);
			Put32(Fields + 32, Candidate.Frame->Num());
			Put32(Fields + 36, Capacity);
			FString Error;
			if (!Publish(Fields, Candidate.Frame->GetData(), Candidate.Frame->Num(), Error))
				StopFromWorker();
			else
				Sequence = NextSequence;
		}
	}

	void StopFromWorker()
	{
		std::lock_guard<std::mutex> Lock(PendingMutex);
		bStopped = true;
		Pending.Reset();
	}
};

struct FURLabLockedLatestFrameReader::FImpl
{
	FURLabShmLiveObjectPtr Object;
	uint32 Capacity = 0;
	uint64 LastSequence = 0;

	FImpl(FURLabShmLiveObjectPtr InObject, uint32 InCapacity)
		: Object(MoveTemp(InObject)), Capacity(InCapacity)
	{
	}
};

TUniquePtr<FURLabLockedLatestFrameWriter> FURLabLockedLatestFrameWriter::Create(
	FURLabShmLiveObjectPtr Object, uint32 MaxPayloadBytes, FString& OutError)
{
	OutError.Reset();
	if (!ValidLiveObject(Object, MaxPayloadBytes))
	{
		OutError = TEXT("latest-frame live object does not match layout capacity");
		return nullptr;
	}
	uint8 Lock[8] = {};
	if (!ReadExactAt(Object->NativeHandle, Lock, sizeof(Lock), SeqlockOffset, OutError))
		return nullptr;
	TUniquePtr<FImpl> NewImpl = MakeUnique<FImpl>(MoveTemp(Object), MaxPayloadBytes);
	NewImpl->Seqlock = Get64(Lock) + (Get64(Lock) & 1);
	NewImpl->Worker = std::thread([Raw = NewImpl.Get()] { Raw->Run(); });
	return TUniquePtr<FURLabLockedLatestFrameWriter>(
		new FURLabLockedLatestFrameWriter(MoveTemp(NewImpl)));
}

FURLabLockedLatestFrameWriter::FURLabLockedLatestFrameWriter(TUniquePtr<FImpl>&& InImpl)
	: Impl(MoveTemp(InImpl))
{
}

FURLabLockedLatestFrameWriter::~FURLabLockedLatestFrameWriter()
{
	Stop();
}

bool FURLabLockedLatestFrameWriter::Submit(
	TSharedRef<const TArray<uint8>, ESPMode::ThreadSafe> ImmutableFrame, FString& OutError)
{
	OutError.Reset();
	if (!Impl || ImmutableFrame->IsEmpty() || ImmutableFrame->Num() > Impl->Capacity)
	{
		OutError = TEXT("latest-frame snapshot is empty or exceeds capacity");
		return false;
	}
	std::lock_guard<std::mutex> Lock(Impl->PendingMutex);
	if (Impl->bStopped)
	{
		OutError = TEXT("latest-frame writer is stopped");
		return false;
	}
	FImpl::FPending Pending;
	Pending.Frame = ImmutableFrame;
	Pending.Epoch = Impl->Epoch;
	Impl->Pending = MoveTemp(Pending);
	Impl->Wakeup.notify_one();
	return true;
}

bool FURLabLockedLatestFrameWriter::Clear(FString& OutError)
{
	OutError.Reset();
	if (!Impl) { OutError = TEXT("latest-frame writer is unavailable"); return false; }
	std::lock_guard<std::mutex> WriteLock(Impl->WriteMutex);
	{
		std::lock_guard<std::mutex> Lock(Impl->PendingMutex);
		if (Impl->bStopped || Impl->Epoch == MAX_uint64)
		{
			OutError = TEXT("latest-frame writer cannot advance its generation barrier");
			return false;
		}
		++Impl->Epoch;
		Impl->Pending.Reset();
	}
	const uint8 Fields[FieldBytes] = {};
	return Impl->Publish(Fields, nullptr, 0, OutError);
}

void FURLabLockedLatestFrameWriter::Stop()
{
	if (!Impl) return;
	{
		std::lock_guard<std::mutex> Lock(Impl->PendingMutex);
		Impl->bStopped = true;
		Impl->Pending.Reset();
	}
	Impl->Wakeup.notify_all();
	if (Impl->Worker.joinable()) Impl->Worker.join();
}

TUniquePtr<FURLabLockedLatestFrameReader> FURLabLockedLatestFrameReader::Create(
	FURLabShmLiveObjectPtr Object, uint32 MaxPayloadBytes, FString& OutError)
{
	OutError.Reset();
	if (!ValidLiveObject(Object, MaxPayloadBytes))
	{
		OutError = TEXT("latest-frame live object does not match layout capacity");
		return nullptr;
	}
	return TUniquePtr<FURLabLockedLatestFrameReader>(new FURLabLockedLatestFrameReader(
		MakeUnique<FImpl>(MoveTemp(Object), MaxPayloadBytes)));
}

FURLabLockedLatestFrameReader::FURLabLockedLatestFrameReader(TUniquePtr<FImpl>&& InImpl)
	: Impl(MoveTemp(InImpl))
{
}

FURLabLockedLatestFrameReader::~FURLabLockedLatestFrameReader() = default;

EURLabLatestFrameRead FURLabLockedLatestFrameReader::ReadLatest(
	TArray<uint8>& OutFrame, FString& OutError)
{
	OutError.Reset();
	if (!Impl) return EURLabLatestFrameRead::NoFrame;
	const UPTRINT Handle = Impl->Object->NativeHandle;
	const uint64 RegionGeneration = Impl->Object->GetDescriptor().RegionGeneration;
	for (int32 Attempt = 0; Attempt < ReadAttempts; ++Attempt)
	{
		uint8 Header[HeaderBytes] = {};
		if (!ReadExactAt(Handle, Header, HeaderBytes, 0, OutError))
			return EURLabLatestFrameRead::Error;
		const uint64 Lock = Get64(Header + SeqlockOffset);
		if (Lock & 1) continue;
		bool bEmpty = true;
		for (uint32 Index = 0; Index < FieldBytes; ++Index) bEmpty &= Header[Index] == 0;
		if (bEmpty) return EURLabLatestFrameRead::NoFrame;
		const uint64 Generation = Get64(Header + 16);
		const uint64 Sequence = Get64(Header + 24);
		const uint32 Length = Get32(Header + 32);
		const uint32 Capacity = Get32(Header + 36);
		const bool bHeaderValid = FMemory::Memcmp(Header, Magic, sizeof(Magic)) == 0
			&& Get32(Header + 8) == URLabLatestFrameLayoutVersion
			&& Get32(Header + 12) == HeaderBytes && Generation == RegionGeneration
			&& Sequence > 0 && Sequence <= static_cast<uint64>(MAX_int64)
			&& Capacity == Impl->Capacity && Length > 0 && Length <= Capacity;
		if (bHeaderValid && Sequence <= Impl->LastSequence)
			return EURLabLatestFrameRead::NoFrame;
		TArray<uint8> Copy;
		if (bHeaderValid)
		{
			Copy.SetNumUninitialized(Length);
			if (!ReadExactAt(Handle, Copy.GetData(), Length, HeaderBytes, OutError))
				return EURLabLatestFrameRead::Error;
		}
		uint8 After[8] = {};
		if (!ReadExactAt(Handle, After, sizeof(After), SeqlockOffset, OutError))
			return EURLabLatestFrameRead::Error;
		if (Get64(After) != Lock) continue;
		if (!bHeaderValid)
		{
			OutError = Generation != RegionGeneration
				? TEXT("stale latest-frame region generation")
				: TEXT("invalid latest-frame header or payload bounds");
			return EURLabLatestFrameRead::Error;
		}
		Impl->LastSequence = Sequence;
		OutFrame = MoveTemp(Copy);
		return EURLabLatestFrameRead::Frame;
	}
	return EURLabLatestFrameRead::NoFrame;
}
