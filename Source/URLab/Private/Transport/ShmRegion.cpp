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

#include "Transport/ShmRegion.h"
#include "HAL/FileManager.h"
#include "Misc/Paths.h"
#include "Utils/URLabLogging.h"

#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#include <windows.h>
#include "Windows/HideWindowsPlatformTypes.h"
#else
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

FMjShmRegion::~FMjShmRegion()
{
	Close(/*bDeleteFile=*/false);
}

uint8* FMjShmRegion::GetSlot(uint32 Idx) const
{
	if (!MappedAddr || Idx >= NumBuffers)
		return nullptr;
	return static_cast<uint8*>(MappedAddr) + sizeof(FMjShmHeader) + Idx * BufferStride;
}

bool FMjShmRegion::Open(const FString& Path, uint32 InBufferStride, uint32 InNBuffers)
{
	if (MappedAddr != nullptr)
	{
		UE_LOG(LogURLabNet, Warning, TEXT("FMjShmRegion::Open: already open at %s"), *FilePath);
		return false;
	}
	if (InBufferStride < sizeof(uint32))
	{
		UE_LOG(LogURLabNet, Error, TEXT("FMjShmRegion::Open: stride must be >= 4"));
		return false;
	}

	// Make sure parent directory exists.
	const FString ParentDir = FPaths::GetPath(Path);
	if (!ParentDir.IsEmpty())
	{
		IFileManager::Get().MakeDirectory(*ParentDir, /*Tree=*/true);
	}

	const uint64 TotalSize = sizeof(FMjShmHeader) + uint64(InBufferStride) * uint64(InNBuffers);

#if PLATFORM_WINDOWS
	// Create / truncate the backing file.
	HANDLE hFile = ::CreateFileW(*Path,
		GENERIC_READ | GENERIC_WRITE,
		FILE_SHARE_READ | FILE_SHARE_WRITE,
		nullptr,
		CREATE_ALWAYS,
		FILE_ATTRIBUTE_NORMAL,
		nullptr);
	if (hFile == INVALID_HANDLE_VALUE)
	{
		UE_LOG(LogURLabNet, Error, TEXT("FMjShmRegion::Open: CreateFileW failed for %s (err=%lu)"),
			*Path, ::GetLastError());
		return false;
	}

	// Set file size to TotalSize via SetFilePointerEx + SetEndOfFile.
	LARGE_INTEGER NewPos;
	NewPos.QuadPart = static_cast<LONGLONG>(TotalSize);
	if (!::SetFilePointerEx(hFile, NewPos, nullptr, FILE_BEGIN) || !::SetEndOfFile(hFile))
	{
		UE_LOG(LogURLabNet, Error, TEXT("FMjShmRegion::Open: SetEndOfFile failed (err=%lu)"),
			::GetLastError());
		::CloseHandle(hFile);
		return false;
	}

	HANDLE hMap = ::CreateFileMappingW(hFile, nullptr, PAGE_READWRITE,
		static_cast<DWORD>(TotalSize >> 32), static_cast<DWORD>(TotalSize & 0xFFFFFFFFu),
		nullptr);
	if (!hMap)
	{
		UE_LOG(LogURLabNet, Error, TEXT("FMjShmRegion::Open: CreateFileMappingW failed (err=%lu)"),
			::GetLastError());
		::CloseHandle(hFile);
		return false;
	}

	void* Addr = ::MapViewOfFile(hMap, FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, 0);
	if (!Addr)
	{
		UE_LOG(LogURLabNet, Error, TEXT("FMjShmRegion::Open: MapViewOfFile failed (err=%lu)"),
			::GetLastError());
		::CloseHandle(hMap);
		::CloseHandle(hFile);
		return false;
	}

	PlatformFileHandle = hFile;
	PlatformMappingHandle = hMap;
#else
	int fd = ::open(TCHAR_TO_UTF8(*Path),
		O_RDWR | O_CREAT | O_TRUNC,
		S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
	if (fd < 0)
	{
		UE_LOG(LogURLabNet, Error, TEXT("FMjShmRegion::Open: open() failed for %s (errno=%d)"),
			*Path, errno);
		return false;
	}

	if (::ftruncate(fd, static_cast<off_t>(TotalSize)) != 0)
	{
		UE_LOG(LogURLabNet, Error, TEXT("FMjShmRegion::Open: ftruncate() failed (errno=%d)"), errno);
		::close(fd);
		return false;
	}

	void* Addr = ::mmap(nullptr, static_cast<size_t>(TotalSize),
		PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (Addr == MAP_FAILED)
	{
		UE_LOG(LogURLabNet, Error, TEXT("FMjShmRegion::Open: mmap() failed (errno=%d)"), errno);
		::close(fd);
		return false;
	}

	PlatformFileHandle = reinterpret_cast<void*>(static_cast<intptr_t>(fd));
	PlatformMappingHandle = nullptr;
#endif

	FilePath = Path;
	MappedAddr = Addr;
	MappedSize = TotalSize;
	BufferStride = InBufferStride;
	NumBuffers = InNBuffers;

	// Initialise header. Atomics are placement-new'd via direct ctor calls
	// (no constructor runs over an already-mapped page).
	FMjShmHeader* Hdr = static_cast<FMjShmHeader*>(MappedAddr);
	Hdr->Magic = URLAB_SHM_MAGIC;
	Hdr->ProtocolVersion = URLAB_SHM_PROTOCOL_VERSION;
	Hdr->BufferStride = BufferStride;
	Hdr->NBuffers = NumBuffers;
	new (&Hdr->Sequence) std::atomic<uint64_t>(0);
	new (&Hdr->LatestIdx) std::atomic<uint32_t>(0);
	FMemory::Memzero(Hdr->Reserved, sizeof(Hdr->Reserved));

	// Zero the buffer slots so consumers reading before the first write get
	// a defined size=0 (rather than whatever was on disk).
	FMemory::Memzero(static_cast<uint8*>(MappedAddr) + sizeof(FMjShmHeader),
		static_cast<SIZE_T>(uint64(BufferStride) * NumBuffers));

	UE_LOG(LogURLabNet, Log, TEXT("FMjShmRegion: opened %s (stride=%u, n=%u, total=%llu bytes)"),
		*FilePath, BufferStride, NumBuffers, (unsigned long long)TotalSize);
	return true;
}

void FMjShmRegion::Close(bool bDeleteFile)
{
	if (MappedAddr == nullptr)
		return;

#if PLATFORM_WINDOWS
	::UnmapViewOfFile(MappedAddr);
	if (PlatformMappingHandle)
		::CloseHandle(static_cast<HANDLE>(PlatformMappingHandle));
	if (PlatformFileHandle)
		::CloseHandle(static_cast<HANDLE>(PlatformFileHandle));
#else
	::munmap(MappedAddr, static_cast<size_t>(MappedSize));
	if (PlatformFileHandle)
	{
		const int fd = static_cast<int>(reinterpret_cast<intptr_t>(PlatformFileHandle));
		::close(fd);
	}
#endif

	if (bDeleteFile && !FilePath.IsEmpty())
	{
		IFileManager::Get().Delete(*FilePath, /*RequireExists=*/false, /*EvenReadOnly=*/false, /*Quiet=*/true);
	}

	MappedAddr = nullptr;
	PlatformFileHandle = nullptr;
	PlatformMappingHandle = nullptr;
	MappedSize = 0;
	BufferStride = 0;
	NumBuffers = 0;
	FilePath.Reset();
}
