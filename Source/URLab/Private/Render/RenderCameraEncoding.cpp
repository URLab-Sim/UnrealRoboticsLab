#include "Render/RenderCameraEncoding.h"

namespace
{
void WriteU32Le(TArray<uint8>& Bytes, uint32 Value)
{
	Bytes.Add(static_cast<uint8>(Value)); Bytes.Add(static_cast<uint8>(Value >> 8));
	Bytes.Add(static_cast<uint8>(Value >> 16)); Bytes.Add(static_cast<uint8>(Value >> 24));
}
}

bool RenderEncodeCameraFrame(const FRenderCameraFrame& Frame,
	const FRenderCameraEffectiveConfiguration& Configuration,
	FString& OutEncoding, int32& OutStrideBytes,
	TArray<uint8>& OutBytes, FString& OutError)
{
	OutEncoding.Reset(); OutStrideBytes = 0; OutBytes.Reset(); OutError.Reset();
	if (Frame.Mode != Configuration.Mode || Frame.Width != Configuration.Resolution.X
		|| Frame.Height != Configuration.Resolution.Y
		|| Frame.CameraConfigurationRevision != Configuration.ConfigurationRevision
		|| Frame.Width <= 0 || Frame.Height <= 0)
	{
		OutError = TEXT("camera frame does not match its effective configuration"); return false;
	}
	const int64 Pixels = static_cast<int64>(Frame.Width) * Frame.Height;
	if (Configuration.LogicalPixelBytes <= 0 || Configuration.LogicalPixelBytes > MAX_int32)
	{
		OutError = TEXT("camera frame encoded size is invalid"); return false;
	}
	OutBytes.Reserve(static_cast<int32>(Configuration.LogicalPixelBytes));
	if (Frame.Mode == ERenderCameraMode::Real && Frame.Color.Num() == Pixels)
	{
		OutEncoding = TEXT("rgba8"); OutStrideBytes = Frame.Width * 4;
		for (const FColor& C : Frame.Color)
		{
			OutBytes.Add(C.R); OutBytes.Add(C.G); OutBytes.Add(C.B); OutBytes.Add(C.A);
		}
	}
	else if (Frame.Mode == ERenderCameraMode::Depth && Frame.Depth.Num() == Pixels
		&& FMath::IsFinite(Configuration.DepthNearCm)
		&& FMath::IsFinite(Configuration.DepthFarCm)
		&& Configuration.DepthNearCm > 0.0f
		&& Configuration.DepthFarCm > Configuration.DepthNearCm)
	{
		OutEncoding = TEXT("depth_f32le"); OutStrideBytes = Frame.Width * 4;
		for (float Cm : Frame.Depth)
		{
			if (FMath::IsNaN(Cm) || Cm < 0.0f
				|| (!FMath::IsFinite(Cm) && Cm < 0.0f))
			{
				OutError = TEXT("depth frame contains NaN, negative, or negative-infinite pixels");
				OutBytes.Reset(); return false;
			}
			const float Metres = (FMath::IsFinite(Cm)
				? FMath::Clamp(Cm, Configuration.DepthNearCm, Configuration.DepthFarCm)
				: Configuration.DepthFarCm) / 100.0f;
			uint32 Bits = 0; FMemory::Memcpy(&Bits, &Metres, sizeof(Bits)); WriteU32Le(OutBytes, Bits);
		}
	}
	else if ((Frame.Mode == ERenderCameraMode::SemanticSegmentation
		|| Frame.Mode == ERenderCameraMode::InstanceSegmentation)
		&& Frame.Seg.Num() == Pixels * 2)
	{
		OutEncoding = TEXT("seg_i32x2le"); OutStrideBytes = Frame.Width * 8;
		for (int32 Value : Frame.Seg) WriteU32Le(OutBytes, static_cast<uint32>(Value));
	}
	else
	{
		OutError = TEXT("camera frame plane count does not match its mode"); return false;
	}
	if (OutBytes.Num() != Configuration.LogicalPixelBytes)
	{
		OutError = TEXT("camera frame encoding does not match its planned byte bound");
		OutBytes.Reset(); return false;
	}
	return true;
}
