#include "Render/RenderCameraEncoding.h"
#include "Async/ParallelFor.h"
#if PLATFORM_CPU_X86_FAMILY
#include <tmmintrin.h>
#endif

static_assert(PLATFORM_LITTLE_ENDIAN, "typed camera wire encodings are little-endian");

bool RenderEncodeCameraFrame(const FRenderCameraFrame& Frame,
	const FRenderCameraEffectiveConfiguration& Configuration,
	FString& OutEncoding, int32& OutStrideBytes,
	TArrayView<uint8> OutBytes, FString& OutError)
{
	OutEncoding.Reset(); OutStrideBytes = 0; OutError.Reset();
	if (Frame.Mode != Configuration.Mode || Frame.Width != Configuration.Resolution.X
		|| Frame.Height != Configuration.Resolution.Y
		|| Frame.CameraConfigurationRevision != Configuration.ConfigurationRevision
		|| Frame.Width <= 0 || Frame.Height <= 0)
	{
		OutError = TEXT("camera frame does not match its effective configuration"); return false;
	}
	const int64 Pixels = static_cast<int64>(Frame.Width) * Frame.Height;
	if (Configuration.LogicalPixelBytes <= 0 || Configuration.LogicalPixelBytes > MAX_int32
		|| OutBytes.Num() != Configuration.LogicalPixelBytes)
	{
		OutError = TEXT("camera frame encoded size is invalid"); return false;
	}
	uint8* Out = OutBytes.GetData();
	if (Frame.Mode == ERenderCameraMode::Real && Frame.Color.Num() == Pixels)
	{
		if (Pixels * 4 != Configuration.LogicalPixelBytes)
		{
			OutError = TEXT("camera frame encoding does not match its planned byte bound"); return false;
		}
		OutEncoding = TEXT("rgba8"); OutStrideBytes = Frame.Width * 4;
		const uint32* Source = reinterpret_cast<const uint32*>(Frame.Color.GetData());
		uint32* Dest = reinterpret_cast<uint32*>(Out);
		const int32 Width = Frame.Width;
		const int32 Height = Frame.Height;
		constexpr int32 RowsPerStrip = 64;
		const int32 NumStrips = FMath::DivideAndRoundUp(Height, RowsPerStrip);
		ParallelFor(NumStrips, [Source, Dest, Width, Height](int32 StripIdx)
		{
			const int32 StartRow = StripIdx * RowsPerStrip;
			const int32 EndRow = FMath::Min(Height, StartRow + RowsPerStrip);
			const int64 StartPixel = static_cast<int64>(StartRow) * Width;
			const int64 NumPixels = static_cast<int64>(EndRow - StartRow) * Width;
			const uint32* SrcPtr = Source + StartPixel;
			uint32* DstPtr = Dest + StartPixel;
			int64 Index = 0;
#if PLATFORM_CPU_X86_FAMILY
			const __m128i ShuffleMask = _mm_setr_epi8(
				2, 1, 0, 3, 6, 5, 4, 7, 10, 9, 8, 11, 14, 13, 12, 15);
			for (; Index + 4 <= NumPixels; Index += 4)
			{
				const __m128i BgraVec = _mm_loadu_si128(
					reinterpret_cast<const __m128i*>(SrcPtr + Index));
				const __m128i RgbaVec = _mm_shuffle_epi8(BgraVec, ShuffleMask);
				_mm_storeu_si128(reinterpret_cast<__m128i*>(DstPtr + Index), RgbaVec);
			}
#endif
			for (; Index < NumPixels; ++Index)
			{
				const uint32 Bgra = SrcPtr[Index];
				DstPtr[Index] = (Bgra & 0xff00ff00u) | ((Bgra >> 16) & 0xffu) | ((Bgra & 0xffu) << 16);
			}
		}, Pixels < 65536 ? EParallelForFlags::ForceSingleThread : EParallelForFlags::None);
	}
	else if (Frame.Mode == ERenderCameraMode::Depth && Frame.Depth.Num() == Pixels
		&& FMath::IsFinite(Configuration.DepthNearCm)
		&& FMath::IsFinite(Configuration.DepthFarCm)
		&& Configuration.DepthNearCm > 0.0f
		&& Configuration.DepthFarCm > Configuration.DepthNearCm)
	{
		if (Pixels * 4 != Configuration.LogicalPixelBytes)
		{
			OutError = TEXT("camera frame encoding does not match its planned byte bound"); return false;
		}
		OutEncoding = TEXT("depth_f32le"); OutStrideBytes = Frame.Width * 4;
		const float* Source = Frame.Depth.GetData();
		for (int64 Index = 0; Index < Pixels; ++Index)
		{
			const float Cm = Source[Index];
			if (FMath::IsNaN(Cm) || Cm < 0.0f)
			{
				OutError = TEXT("depth frame contains NaN, negative, or negative-infinite pixels");
				return false;
			}
			const float Metres = (FMath::IsFinite(Cm)
				? FMath::Clamp(Cm, Configuration.DepthNearCm, Configuration.DepthFarCm)
				: Configuration.DepthFarCm) / 100.0f;
			FMemory::Memcpy(Out + Index * 4, &Metres, sizeof(Metres));
		}
	}
	else if ((Frame.Mode == ERenderCameraMode::SemanticSegmentation
		|| Frame.Mode == ERenderCameraMode::InstanceSegmentation)
		&& Frame.Seg.Num() == Pixels * 2)
	{
		if (Pixels * 8 != Configuration.LogicalPixelBytes)
		{
			OutError = TEXT("camera frame encoding does not match its planned byte bound"); return false;
		}
		OutEncoding = TEXT("seg_i32x2le"); OutStrideBytes = Frame.Width * 8;
		FMemory::Memcpy(Out, Frame.Seg.GetData(), Pixels * 8);
	}
	else
	{
		OutError = TEXT("camera frame plane count does not match its mode"); return false;
	}
	return true;
}
