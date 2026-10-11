#pragma once

#include "CoreMinimal.h"
#include "Render/RenderFrame.h"

/** One immutable, completed camera frame. Pixel bytes use Encoding exactly. */
struct URLAB_API FURLabCameraStreamFrame
{
	FString ProducerBootId;
	FString StreamId;
	uint64 StreamGeneration = 0;
	FString SourceRevision;
	uint64 Sequence = 0;
	FString CameraName;
	FRenderFrameIdentity Identity;
	double AcquisitionUnixTime = 0.0;
	FString Encoding;
	int32 Width = 0;
	int32 Height = 0;
	int32 StrideBytes = 0;
	uint64 CameraConfigurationRevision = 0;
	uint64 CaptureSerial = 0;
	TArray<uint8> Pixels;
};

class URLAB_API FURLabCameraStreamFrameCodec
{
public:
	static bool Encode(const FURLabCameraStreamFrame& Frame, TArray<uint8>& OutBytes,
		FString& OutError);
	static bool Decode(TConstArrayView<uint8> Bytes, FURLabCameraStreamFrame& OutFrame,
		FString& OutError, int64 MaxPixelBytes = MAX_int32);
};
