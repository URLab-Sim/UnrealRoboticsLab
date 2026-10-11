// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#pragma once

#include "CoreMinimal.h"
#include "Render/RenderFrame.h"

struct URLAB_API FURLabPoseStreamFrame
{
	FString ProducerBootId;
	FString StreamId;
	uint64 StreamGeneration = 0;
	FString SourceRevision;
	uint64 Sequence = 0;
	uint64 FrameId = 0;
	bool bHasSourceTime = false;
	double SourceTime = 0.0;
	uint64 ClockEpoch = 0;
	TArray<uint8> Payload;
	TArray<FRenderFrame::FDeformable> Deformables;
	TArray<FRenderFrame::FDebugMesh> DebugMeshes;
};

/** Exact MessagePack codec for dense RH-metre/wxyz Cartesian pose frames. */
class URLAB_API FURLabPoseStreamFrameCodec
{
public:
	static bool Encode(const FURLabPoseStreamFrame& Frame, TArray<uint8>& OutBytes,
		FString& OutError);
	static bool Decode(TConstArrayView<uint8> Bytes, FURLabPoseStreamFrame& OutFrame,
		FString& OutError, int64 ExpectedPayloadBytes = INDEX_NONE);
};
