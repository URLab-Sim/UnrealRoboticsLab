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

#include "MuJoCo/Components/Sensors/CameraShmWriter.h"
#include "Utils/URLabLogging.h"

bool FCameraShmWriter::Open(const FString& Path, FIntPoint Resolution)
{
	ExpectedPixels = Resolution.X * Resolution.Y;
	if (ExpectedPixels <= 0)
	{
		UE_LOG(LogURLabNet, Warning,
			TEXT("FCameraShmWriter::Open: bad resolution %dx%d"),
			Resolution.X, Resolution.Y);
		return false;
	}

	// 4 bytes per pixel (FColor) + 4-byte size prefix + frame meta per slot.
	const uint32 Stride = static_cast<uint32>(ExpectedPixels) * 4u
		+ sizeof(uint32) + sizeof(FMjCameraFrameMeta);
	return Region.Open(Path, Stride, /*NBuffers=*/2);
}

void FCameraShmWriter::Close(bool bDeleteFile)
{
	Region.Close(bDeleteFile);
	ExpectedPixels = 0;
}

void FCameraShmWriter::PushFrame(const void* Data, uint32 ByteCount, const FMjCameraFrameMeta& Meta)
{
	if (!Region.IsOpen() || !Data)
		return;
	if (ByteCount != static_cast<uint32>(ExpectedPixels) * 4u)
		return;

	FMjShmHeader* Hdr = static_cast<FMjShmHeader*>(Region.GetData());
	if (!Hdr)
		return;

	const uint32 CurrentLatest = Hdr->LatestIdx.load(std::memory_order_acquire);
	const uint32 NBuffers = Hdr->NBuffers > 0 ? Hdr->NBuffers : 1;
	const uint32 Target = (CurrentLatest + 1) % NBuffers;
	uint8* Slot = Region.GetSlot(Target);
	if (!Slot)
		return;

	// Slot layout: [u32 payload_size][FMjCameraFrameMeta][pixels]. payload_size
	// covers meta + pixels so the consumer reads one length then splits.
	const uint32 PayloadSize = sizeof(FMjCameraFrameMeta) + ByteCount;
	FMemory::Memcpy(Slot, &PayloadSize, sizeof(uint32));
	FMemory::Memcpy(Slot + sizeof(uint32), &Meta, sizeof(FMjCameraFrameMeta));
	FMemory::Memcpy(Slot + sizeof(uint32) + sizeof(FMjCameraFrameMeta), Data, ByteCount);

	Hdr->LatestIdx.store(Target, std::memory_order_release);
	Hdr->Sequence.fetch_add(1, std::memory_order_release);
}
