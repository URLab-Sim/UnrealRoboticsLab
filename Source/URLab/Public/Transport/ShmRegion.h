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
#include <atomic>

/**
 * @file MjShmRegion.h
 *
 * Cross-platform shared-memory region for the URLab SHM transport: a
 * fixed-size, file-backed mmap region with a header and N buffer slots.
 * Producers double-buffer (n_buffers=2) and bump a sequence counter after
 * each write; consumers poll the sequence to detect new data.
 *
 * The implementation calls into the platform's mmap directly (`::MapViewOfFile`
 * on Windows, `::mmap` on Linux/macOS). UE's IMappedFileHandle is read-only,
 * which doesn't fit our writer use case, so this lower-level wrapper is
 * justified. Both branches keep the surface tiny (Open / Close / GetData /
 * GetSize) so the publisher logic above stays portable.
 */

/** Magic number identifying a URLab SHM region. ASCII "URLB" little-endian. */
inline constexpr uint32 URLAB_SHM_MAGIC = 0x42'4C'52'55;
inline constexpr uint32 URLAB_SHM_PROTOCOL_VERSION = 1;

/** Fixed header layout. Must be exactly 64 bytes (one cache line) for atomic
 *  alignment safety and to make the C side and Python side trivially agree.
 *
 *  Layout (offsets in bytes):
 *    0  uint32 magic
 *    4  uint32 protocol_version
 *    8  uint32 buffer_stride
 *    12 uint32 n_buffers
 *    16 atomic<uint64> sequence
 *    24 atomic<uint32> latest_idx
 *    28 uint8[36] reserved
 *
 *  Atomic alignment notes:
 *   - sequence at offset 16 -> 8-byte aligned. Good.
 *   - latest_idx at offset 24 -> 4-byte aligned. Good.
 *   - All atomics are over primitive integer types, ABI-stable across
 *     the major x86_64 / arm64 compilers we target.
 */
struct FMjShmHeader
{
    uint32 Magic = 0;
    uint32 ProtocolVersion = 0;
    uint32 BufferStride = 0;
    uint32 NBuffers = 0;
    std::atomic<uint64_t> Sequence{0};
    std::atomic<uint32_t> LatestIdx{0};
    uint8 Reserved[36] = {};
};
static_assert(sizeof(FMjShmHeader) == 64, "FMjShmHeader must fit one cache line");

/**
 * @brief Owning wrapper around a file-backed mmap region.
 *
 * Lifetime: Open() truncates / creates the file, mmaps the full size, and
 * writes the initial FMjShmHeader. Close() unmaps and (optionally) deletes
 * the file. Move-only; no copies.
 *
 * Thread safety: GetData() returns a raw pointer that is valid until Close()
 * is called. Concurrent reads / writes through that pointer are the caller's
 * responsibility (the publisher uses the double-buffer pattern below).
 */
class URLAB_API FMjShmRegion
{
public:
    FMjShmRegion() = default;
    ~FMjShmRegion();

    FMjShmRegion(const FMjShmRegion&) = delete;
    FMjShmRegion& operator=(const FMjShmRegion&) = delete;

    /** Create / truncate the file at `Path`, mmap header + n_buffers slots,
     *  and initialise the header. Returns false on platform / IO error.
     *
     *  Total mapped size = sizeof(FMjShmHeader) + NBuffers * BufferStride.
     */
    bool Open(const FString& Path, uint32 BufferStride, uint32 NBuffers);

    /** Unmap, close handles, optionally remove the backing file. */
    void Close(bool bDeleteFile = false);

    bool IsOpen() const { return MappedAddr != nullptr; }

    /** Pointer to the start of the mapped region (header lives at offset 0). */
    void* GetData() const { return MappedAddr; }

    /** Pointer to buffer slot index `Idx` (0 .. NBuffers-1). */
    uint8* GetSlot(uint32 Idx) const;

    uint32 GetBufferStride() const { return BufferStride; }
    uint32 GetNumBuffers() const { return NumBuffers; }

    FString GetPath() const { return FilePath; }

private:
    FString FilePath;
    void* MappedAddr = nullptr;
    uint64 MappedSize = 0;
    uint32 BufferStride = 0;
    uint32 NumBuffers = 0;

    // Platform-specific handles. Stored as void* so the header doesn't drag
    // <windows.h> / <sys/mman.h> into every TU that includes us.
    void* PlatformFileHandle = nullptr;     // Win: HANDLE; Posix: int (cast)
    void* PlatformMappingHandle = nullptr;  // Win: HANDLE; Posix: unused
};
