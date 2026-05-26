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
#include "Transport/ShmRegion.h"

/**
 * @class FCameraShmWriter
 * @brief Per-camera SHM publisher.
 *
 * Writes raw pixel frames into a double-buffered SHM region named
 * `cam_<prefix>_<name>.shm`. Same producer pattern as the state snapshot
 * publisher: write into !latest, flip latest, bump sequence. No
 * background thread -- the camera readback callback feeds this directly
 * on the game / render-thread sidecar.
 *
 * Wire layout per slot: `[u32 size][bytes...]`. The byte format depends
 * on the camera mode and is documented out-of-band via the handshake:
 *  - Real: width * height * 4 bytes BGRA8 (FColor).
 *  - Depth: width * height * 4 bytes float32 (single channel).
 *  - Semantic / Instance: width * height * 4 bytes BGRA8 with a
 *    per-class / per-instance color tint baked in.
 *
 * All four modes share the same 4 bytes-per-pixel stride, so the SHM
 * region size is independent of mode.
 */
class URLAB_API FCameraShmWriter
{
public:
    FCameraShmWriter() = default;

    /** Open / truncate the SHM file. The slot stride is sized to hold
     *  Resolution.X * Resolution.Y * 4 bytes plus the size prefix --
     *  works for BGRA8 and float32 single-channel alike. */
    bool Open(const FString& Path, FIntPoint Resolution);

    /** Unmap; optionally remove the backing file. */
    void Close(bool bDeleteFile = true);

    bool IsOpen() const { return Region.IsOpen(); }

    /** Push one frame's raw bytes. Silently drops if `ByteCount` doesn't
     *  match the configured pixel count * 4 -- a partial frame can't be
     *  decoded by the consumer anyway. */
    void PushFrame(const void* Data, uint32 ByteCount);

    /** Convenience overload for color frames. */
    void PushFrame(const TArray<FColor>& Pixels)
    {
        PushFrame(Pixels.GetData(), static_cast<uint32>(Pixels.Num()) * sizeof(FColor));
    }

    /** Convenience overload for single-channel float frames (depth). */
    void PushFrame(const TArray<float>& Pixels)
    {
        PushFrame(Pixels.GetData(), static_cast<uint32>(Pixels.Num()) * sizeof(float));
    }

    FString GetPath() const { return Region.GetPath(); }

private:
    FMjShmRegion Region;
    int32 ExpectedPixels = 0;
};
