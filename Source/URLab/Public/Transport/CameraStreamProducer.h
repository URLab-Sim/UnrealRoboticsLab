#pragma once

#include "CoreMinimal.h"
#include "Transport/CameraStreamFrameCodec.h"
#include "Transport/ShmExactPayload.h"

struct URLAB_API FURLabCameraStreamProducerConfig
{
	FString BindEndpoint;
	FString AdvertisedEndpoint;
	int64 MaxFrameBytes = 16 * 1024 * 1024;
	FURLabShmExactRegionPtr ShmRegion;
};

struct URLAB_API FURLabCameraStreamDescriptor
{
	FString ProducerBootId;
	FString StreamId;
	uint64 StreamGeneration = 0;
	FString SourceRevision;
	FString CameraName;
	uint64 CameraConfigurationRevision = 0;
	FString Encoding;
	int32 Width = 0;
	int32 Height = 0;
	int32 StrideBytes = 0;
};

/** Exact immutable camera/source configuration for one producer generation. */
struct URLAB_API FURLabCameraStreamConfiguration
{
	FString SourceRevision;
	FString CameraName;
	uint64 CameraConfigurationRevision = 0;
	FString Encoding;
	int32 Width = 0;
	int32 Height = 0;
	int32 StrideBytes = 0;
	float DepthNearCm = 0.0f;
	float DepthFarCm = 0.0f;
};

struct URLAB_API FURLabCameraStreamDelivery
{
	FString ZmqEndpoint;
	TArray<uint8> ZmqTopic;
	TOptional<FURLabShmLiveObjectDescriptor> ShmObject;
	uint32 ShmLayoutVersion = 1;
	uint32 MaxFrameBytes = 0;
};

/** Transport-only completed-frame fan-out. It owns no workflow or authority state. */
class URLAB_API FURLabCameraStreamProducer
{
public:
	static TUniquePtr<FURLabCameraStreamProducer> Create(
		const FURLabCameraStreamProducerConfig& Config, FString& OutError);
	~FURLabCameraStreamProducer();
	bool CommitCamera(const FURLabCameraStreamConfiguration& Configuration,
		FURLabCameraStreamDescriptor& OutDescriptor, FString& OutError);
	bool SubmitCompleted(
		TSharedPtr<const struct FRenderCameraFrame, ESPMode::ThreadSafe> Frame,
		FString& OutError);
	bool Describe(bool bShmEligible, FURLabCameraStreamDescriptor& OutDescriptor,
		FURLabCameraStreamDelivery& OutDelivery, FString& OutError) const;
	void Stop();
private:
	struct FImpl;
	explicit FURLabCameraStreamProducer(TUniquePtr<FImpl>&& InImpl);
	TUniquePtr<FImpl> Impl;
};
