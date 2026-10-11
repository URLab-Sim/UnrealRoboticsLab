// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#include "Transport/LockedLatestFrame.h"

#include "HAL/PlatformProcess.h"
#include "Misc/ScopeLock.h"

#include <condition_variable>
#include <mutex>
#include <thread>

#if PLATFORM_LINUX
#include <cerrno>
#include <sys/file.h>
#include <unistd.h>
#elif PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#include <Windows.h>
#include "Windows/HideWindowsPlatformTypes.h"
#endif

namespace
{
constexpr uint32 LayoutVersion = 1;
constexpr uint32 HeaderBytes = 48;
constexpr uint8 Magic[8] = {'U','R','L','V','S','T','R','M'};

FCriticalSection MappingLocksMutex;
TMap<FString, TWeakPtr<FCriticalSection, ESPMode::ThreadSafe>> MappingLocks;

TSharedRef<FCriticalSection, ESPMode::ThreadSafe> MappingLock(
	const FURLabShmLiveObjectDescriptor& Descriptor)
{
	const FString Key = Descriptor.RegionId + TEXT(":") + Descriptor.ObjectName;
	FScopeLock Guard(&MappingLocksMutex);
	if (TSharedPtr<FCriticalSection, ESPMode::ThreadSafe> Existing = MappingLocks.FindRef(Key).Pin())
		return Existing.ToSharedRef();
	TSharedRef<FCriticalSection, ESPMode::ThreadSafe> Created =
		MakeShared<FCriticalSection, ESPMode::ThreadSafe>();
	MappingLocks.Add(Key, Created);
	return Created;
}

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

enum class ELockResult : uint8 { Acquired, Contended, Error };

ELockResult TryFileLock(UPTRINT NativeHandle, bool bExclusive, FString& Error)
{
#if PLATFORM_LINUX
	if (::flock(static_cast<int>(NativeHandle),
		(bExclusive ? LOCK_EX : LOCK_SH) | LOCK_NB) == 0) return ELockResult::Acquired;
	if (errno == EWOULDBLOCK || errno == EAGAIN) return ELockResult::Contended;
	Error = FString::Printf(TEXT("latest-frame flock failed (errno=%d)"), errno);
	return ELockResult::Error;
#elif PLATFORM_WINDOWS
	OVERLAPPED Overlapped = {};
	const DWORD Flags = LOCKFILE_FAIL_IMMEDIATELY
		| (bExclusive ? LOCKFILE_EXCLUSIVE_LOCK : 0);
	if (::LockFileEx(reinterpret_cast<HANDLE>(NativeHandle), Flags, 0,
		MAXDWORD, MAXDWORD, &Overlapped)) return ELockResult::Acquired;
	const DWORD Code = ::GetLastError();
	if (Code == ERROR_LOCK_VIOLATION) return ELockResult::Contended;
	Error = FString::Printf(TEXT("latest-frame LockFileEx failed (error=%lu)"), Code);
	return ELockResult::Error;
#else
	Error = TEXT("latest-frame file locking is unsupported on this platform");
	return ELockResult::Error;
#endif
}

bool UnlockFile(UPTRINT NativeHandle, FString& Error)
{
#if PLATFORM_LINUX
	if (::flock(static_cast<int>(NativeHandle), LOCK_UN) == 0) return true;
	Error = FString::Printf(TEXT("latest-frame unlock failed (errno=%d)"), errno);
	return false;
#elif PLATFORM_WINDOWS
	OVERLAPPED Overlapped = {};
	if (::UnlockFileEx(reinterpret_cast<HANDLE>(NativeHandle), 0,
		MAXDWORD, MAXDWORD, &Overlapped)) return true;
	Error = FString::Printf(TEXT("latest-frame UnlockFileEx failed (error=%lu)"), ::GetLastError());
	return false;
#else
	Error = TEXT("latest-frame file unlocking is unsupported on this platform");
	return false;
#endif
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
	LARGE_INTEGER Position; Position.QuadPart = Offset;
	if (!::SetFilePointerEx(reinterpret_cast<HANDLE>(NativeHandle), Position, nullptr, FILE_BEGIN))
	{ Error = TEXT("latest-frame seek failed"); return false; }
	DWORD Written = 0;
	if (!::WriteFile(reinterpret_cast<HANDLE>(NativeHandle), Data, Length, &Written, nullptr)
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
	LARGE_INTEGER Position; Position.QuadPart = Offset;
	if (!::SetFilePointerEx(reinterpret_cast<HANDLE>(NativeHandle), Position, nullptr, FILE_BEGIN))
	{ Error = TEXT("latest-frame seek failed"); return false; }
	DWORD Read = 0;
	if (!::ReadFile(reinterpret_cast<HANDLE>(NativeHandle), Data, Length, &Read, nullptr)
		|| Read != Length) { Error = TEXT("short latest-frame read"); return false; }
	return true;
#else
	Error = TEXT("latest-frame reads are unsupported on this platform"); return false;
#endif
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
	TSharedRef<FCriticalSection, ESPMode::ThreadSafe> LocalLock;
	uint32 Capacity = 0;
	uint64 Sequence = 0;
	uint64 Epoch = 1;
	std::mutex PendingMutex;
	std::mutex WriteMutex;
	std::condition_variable Wakeup;
	TOptional<FPending> Pending;
	bool bStopped = false;
	std::thread Worker;

	FImpl(FURLabShmLiveObjectPtr InObject, uint32 InCapacity)
		: Object(MoveTemp(InObject)), LocalLock(MappingLock(Object->GetDescriptor())),
		  Capacity(InCapacity)
	{
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
			std::unique_lock<std::mutex> WriteLock(WriteMutex);
			{
				std::lock_guard<std::mutex> Lock(PendingMutex);
				if (bStopped || Candidate.Epoch != Epoch) continue;
			}
			if (!LocalLock->TryLock())
			{
				WriteLock.unlock();
				Requeue(MoveTemp(Candidate));
				continue;
			}
			FString Error;
			const ELockResult LockResult = TryFileLock(Object->NativeHandle, true, Error);
			if (LockResult != ELockResult::Acquired)
			{
				LocalLock->Unlock();
				WriteLock.unlock();
				if (LockResult == ELockResult::Contended) Requeue(MoveTemp(Candidate));
				else StopFromWorker();
				continue;
			}
			if (Sequence == static_cast<uint64>(MAX_int64))
			{
				UnlockFile(Object->NativeHandle, Error);
				LocalLock->Unlock();
				StopFromWorker();
				continue;
			}
			const uint64 NextSequence = Sequence + 1;
			uint8 Header[HeaderBytes] = {};
			FMemory::Memcpy(Header, Magic, sizeof(Magic));
			Put32(Header + 8, LayoutVersion);
			Put32(Header + 12, HeaderBytes);
			Put64(Header + 16, Object->GetDescriptor().RegionGeneration);
			Put64(Header + 24, NextSequence);
			Put32(Header + 32, Candidate.Frame->Num());
			Put32(Header + 36, Capacity);
			const bool bWritten = WriteExactAt(Object->NativeHandle, Candidate.Frame->GetData(),
				Candidate.Frame->Num(), HeaderBytes, Error)
				&& WriteExactAt(Object->NativeHandle, Header, HeaderBytes, 0, Error);
			UnlockFile(Object->NativeHandle, Error);
			LocalLock->Unlock();
			if (!bWritten) StopFromWorker();
			else Sequence = NextSequence;
		}
	}

	void Requeue(FPending&& Candidate)
	{
		{
			std::lock_guard<std::mutex> Lock(PendingMutex);
			if (bStopped || Candidate.Epoch != Epoch) return;
			if (!Pending.IsSet()) Pending = MoveTemp(Candidate);
		}
		FPlatformProcess::SleepNoStats(0.001f);
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
	TSharedRef<FCriticalSection, ESPMode::ThreadSafe> LocalLock;
	uint32 Capacity = 0;
	uint64 LastSequence = 0;

	FImpl(FURLabShmLiveObjectPtr InObject, uint32 InCapacity)
		: Object(MoveTemp(InObject)), LocalLock(MappingLock(Object->GetDescriptor())),
		  Capacity(InCapacity)
	{
	}
};

TUniquePtr<FURLabLockedLatestFrameWriter> FURLabLockedLatestFrameWriter::Create(
	FURLabShmLiveObjectPtr Object, uint32 MaxPayloadBytes, FString& OutError)
{
	OutError.Reset();
	if (!Object || MaxPayloadBytes == 0
		|| Object->GetDescriptor().RegionGeneration == 0
		|| Object->GetDescriptor().RegionGeneration > static_cast<uint64>(MAX_int64)
		|| Object->GetDescriptor().ByteLength != int64(HeaderBytes) + MaxPayloadBytes)
	{
		OutError = TEXT("latest-frame live object does not match layout capacity");
		return nullptr;
	}
	TUniquePtr<FImpl> NewImpl = MakeUnique<FImpl>(MoveTemp(Object), MaxPayloadBytes);
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
	std::unique_lock<std::mutex> WriteLock(Impl->WriteMutex);
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
	Impl->LocalLock->Lock();
	ELockResult LockResult = ELockResult::Contended;
	for (int32 Attempt = 0; Attempt < 2000 && LockResult == ELockResult::Contended; ++Attempt)
	{
		LockResult = TryFileLock(Impl->Object->NativeHandle, true, OutError);
		if (LockResult == ELockResult::Contended) FPlatformProcess::SleepNoStats(0.001f);
	}
	bool bCleared = false;
	if (LockResult == ELockResult::Acquired)
	{
		uint8 Header[HeaderBytes] = {};
		bCleared = WriteExactAt(Impl->Object->NativeHandle, Header, HeaderBytes, 0, OutError);
		UnlockFile(Impl->Object->NativeHandle, OutError);
	}
	else if (OutError.IsEmpty()) OutError = TEXT("latest-frame generation barrier lock timed out");
	Impl->LocalLock->Unlock();
	return bCleared;
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
	if (!Object || MaxPayloadBytes == 0
		|| Object->GetDescriptor().RegionGeneration == 0
		|| Object->GetDescriptor().RegionGeneration > static_cast<uint64>(MAX_int64)
		|| Object->GetDescriptor().ByteLength != int64(HeaderBytes) + MaxPayloadBytes)
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
	if (!Impl || !Impl->LocalLock->TryLock()) return EURLabLatestFrameRead::NoFrame;
	const ELockResult LockResult = TryFileLock(Impl->Object->NativeHandle, false, OutError);
	if (LockResult != ELockResult::Acquired)
	{
		Impl->LocalLock->Unlock();
		return LockResult == ELockResult::Contended
			? EURLabLatestFrameRead::NoFrame : EURLabLatestFrameRead::Error;
	}
	uint8 Header[HeaderBytes] = {};
	if (!ReadExactAt(Impl->Object->NativeHandle, Header, HeaderBytes, 0, OutError))
	{
		UnlockFile(Impl->Object->NativeHandle, OutError);
		Impl->LocalLock->Unlock();
		return EURLabLatestFrameRead::Error;
	}
	bool bAllZero = true;
	for (uint8 Byte : Header) bAllZero &= Byte == 0;
	if (bAllZero)
	{
		UnlockFile(Impl->Object->NativeHandle, OutError);
		Impl->LocalLock->Unlock();
		return EURLabLatestFrameRead::NoFrame;
	}
	const uint64 Generation = Get64(Header + 16);
	const uint64 Sequence = Get64(Header + 24);
	const uint32 Length = Get32(Header + 32);
	const uint32 Capacity = Get32(Header + 36);
	const bool bHeaderValid = FMemory::Memcmp(Header, Magic, sizeof(Magic)) == 0
		&& Get32(Header + 8) == LayoutVersion && Get32(Header + 12) == HeaderBytes
		&& Get64(Header + 40) == 0 && Generation == Impl->Object->GetDescriptor().RegionGeneration
		&& Generation <= static_cast<uint64>(MAX_int64) && Sequence > 0
		&& Sequence <= static_cast<uint64>(MAX_int64)
		&& Capacity == Impl->Capacity && Length > 0 && Length <= Capacity;
	if (!bHeaderValid)
	{
		OutError = Generation != Impl->Object->GetDescriptor().RegionGeneration
			? TEXT("stale latest-frame region generation")
			: TEXT("invalid latest-frame header or payload bounds");
		UnlockFile(Impl->Object->NativeHandle, OutError);
		Impl->LocalLock->Unlock();
		return EURLabLatestFrameRead::Error;
	}
	if (Sequence <= Impl->LastSequence)
	{
		UnlockFile(Impl->Object->NativeHandle, OutError);
		Impl->LocalLock->Unlock();
		return EURLabLatestFrameRead::NoFrame;
	}
	TArray<uint8> Copy;
	Copy.SetNumUninitialized(Length);
	const bool bRead = ReadExactAt(Impl->Object->NativeHandle, Copy.GetData(),
		Length, HeaderBytes, OutError);
	UnlockFile(Impl->Object->NativeHandle, OutError);
	Impl->LocalLock->Unlock();
	if (!bRead) return EURLabLatestFrameRead::Error;
	Impl->LastSequence = Sequence;
	OutFrame = MoveTemp(Copy);
	return EURLabLatestFrameRead::Frame;
}
