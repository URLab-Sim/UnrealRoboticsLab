// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#pragma once

#include "CoreMinimal.h"
#include "Transport/ShmExactPayload.h"
#include "Render/RenderFrame.h"

struct URLAB_API FURLabPoseStreamProducerConfig
{
	FString BindEndpoint;
	/** Explicit dialable endpoint advertised after BindEndpoint successfully binds. */
	FString AdvertisedEndpoint;
	int64 MaxPayloadBytes = 4 * 1024 * 1024;
	FURLabShmExactRegionPtr ShmRegion;
};

struct URLAB_API FURLabPoseStreamDescriptor
{
	FString Kind = TEXT("cartesian_pose");
	int32 SchemaVersion = 2;
	FString ProducerBootId;
	FString StreamId;
	uint64 StreamGeneration = 0;
	FString SourceRevision;
	FString PayloadFormat = TEXT("dense_rh_m_wxyz_f64le_dynamic_mesh_v1");
};

struct URLAB_API FURLabPoseStreamZmqDelivery
{
	FString Endpoint;
	TArray<uint8> Topic;
};

struct URLAB_API FURLabPoseStreamShmDelivery
{
	FURLabShmLiveObjectDescriptor Object;
	uint32 LayoutVersion = 1;
	uint32 MaxPayloadBytes = 0;
};

struct URLAB_API FURLabPoseStreamDescription
{
	FURLabPoseStreamDescriptor Stream;
	TOptional<FURLabPoseStreamZmqDelivery> Zmq;
	TOptional<FURLabPoseStreamShmDelivery> Shm;
};

/** Transport-only Cartesian producer. Callers retain all source and authority policy. */
class URLAB_API FURLabPoseStreamProducer
{
public:
	static TUniquePtr<FURLabPoseStreamProducer> Create(
		const FURLabPoseStreamProducerConfig& Config, FString& OutError);
	~FURLabPoseStreamProducer();

	bool CommitSource(const FString& SourceRevision,
		FURLabPoseStreamDescriptor& OutDescriptor, FString& OutError);
	bool Publish(uint64 FrameId, double SourceTime, bool bHasSourceTime,
		uint64 ClockEpoch, TConstArrayView<double> RightHandedMetresWxyz,
		FString& OutError);
	bool Publish(uint64 FrameId, double SourceTime, bool bHasSourceTime,
		uint64 ClockEpoch, TConstArrayView<double> RightHandedMetresWxyz,
		const FRenderFrame& DynamicFrame, FString& OutError);
	bool DescribeForSession(bool bShmEligible,
		FURLabPoseStreamDescription& OutDescription, FString& OutError) const;
	void Stop();

private:
	struct FImpl;
	explicit FURLabPoseStreamProducer(TUniquePtr<FImpl>&& InImpl);
	TUniquePtr<FImpl> Impl;
};
