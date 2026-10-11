// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#include "Transport/CameraStreamProducer.h"

#include "Misc/Guid.h"
#include "Misc/ScopeLock.h"
#include "Render/RenderCameraTypes.h"
#include "Transport/LockedLatestFrame.h"
#include "Transport/CameraStreamFrameCodec.h"
#include "Transport/ZmqContext.h"

#include <condition_variable>
#include <atomic>
#include <mutex>
#include <thread>

#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#endif
#include "zmq.h"
#if PLATFORM_WINDOWS
#include "Windows/HideWindowsPlatformTypes.h"
#endif

namespace
{
FString MintId()
{
	return FGuid::NewGuid().ToString(EGuidFormats::Digits).ToLower();
}

bool IsValidConfiguration(const FURLabCameraStreamConfiguration& Configuration)
{
	int32 BytesPerPixel = 0;
	if (Configuration.Encoding == TEXT("rgba8") || Configuration.Encoding == TEXT("depth_f32le"))
	{
		BytesPerPixel = 4;
	}
	else if (Configuration.Encoding == TEXT("seg_i32x2le"))
	{
		BytesPerPixel = 8;
	}
	return !Configuration.SourceRevision.IsEmpty() && !Configuration.CameraName.IsEmpty()
		&& Configuration.CameraConfigurationRevision > 0 && BytesPerPixel > 0
		&& Configuration.Width > 0 && Configuration.Height > 0
		&& Configuration.Width <= MAX_int32 / BytesPerPixel
		&& Configuration.StrideBytes == Configuration.Width * BytesPerPixel
		&& (Configuration.Encoding != TEXT("depth_f32le")
			|| (FMath::IsFinite(Configuration.DepthNearCm)
				&& FMath::IsFinite(Configuration.DepthFarCm)
				&& Configuration.DepthNearCm >= 0.0f
				&& Configuration.DepthFarCm > Configuration.DepthNearCm));
}

bool Matches(const FURLabCameraStreamDescriptor& Descriptor,
	const FURLabCameraStreamConfiguration& Configuration)
{
	return Descriptor.SourceRevision == Configuration.SourceRevision
		&& Descriptor.CameraName == Configuration.CameraName
		&& Descriptor.CameraConfigurationRevision == Configuration.CameraConfigurationRevision
		&& Descriptor.Encoding == Configuration.Encoding
		&& Descriptor.Width == Configuration.Width
		&& Descriptor.Height == Configuration.Height
		&& Descriptor.StrideBytes == Configuration.StrideBytes;
}

void WriteU32Le(TArray<uint8>& Bytes, uint32 Value)
{
	Bytes.Add(static_cast<uint8>(Value));
	Bytes.Add(static_cast<uint8>(Value >> 8));
	Bytes.Add(static_cast<uint8>(Value >> 16));
	Bytes.Add(static_cast<uint8>(Value >> 24));
}

struct FPendingCameraFrame
{
	TSharedPtr<const FRenderCameraFrame, ESPMode::ThreadSafe> Frame;
	FURLabCameraStreamDescriptor Descriptor;
	uint64 Sequence = 0;
	float DepthNearCm = 0.0f;
	float DepthFarCm = 0.0f;
};

bool BuildWireFrame(const FPendingCameraFrame& Pending, uint64 Sequence,
	FURLabCameraStreamFrame& Out, FString& OutError)
{
	const FRenderCameraFrame& Frame = *Pending.Frame;
	const FURLabCameraStreamDescriptor& Descriptor = Pending.Descriptor;
	if (Frame.Width != Descriptor.Width || Frame.Height != Descriptor.Height
		|| Frame.CameraConfigurationRevision != Descriptor.CameraConfigurationRevision
		|| Frame.Identity.SceneRevision != Descriptor.SourceRevision)
	{
		OutError = TEXT("completed camera frame does not match committed stream configuration");
		return false;
	}
	const int64 PixelCount = static_cast<int64>(Frame.Width) * Frame.Height;
	Out = {};
	Out.ProducerBootId = Descriptor.ProducerBootId;
	Out.StreamId = Descriptor.StreamId;
	Out.StreamGeneration = Descriptor.StreamGeneration;
	Out.SourceRevision = Descriptor.SourceRevision;
	Out.Sequence = Sequence;
	Out.CameraName = Descriptor.CameraName;
	Out.Identity = Frame.Identity;
	Out.AcquisitionUnixTime = Frame.CaptureUnixTime;
	Out.Encoding = Descriptor.Encoding;
	Out.Width = Descriptor.Width;
	Out.Height = Descriptor.Height;
	Out.StrideBytes = Descriptor.StrideBytes;
	Out.CameraConfigurationRevision = Descriptor.CameraConfigurationRevision;
	Out.CaptureSerial = Frame.CaptureSerial;
	if (Out.Encoding == TEXT("rgba8") && Frame.Mode == ERenderCameraMode::Real
		&& Frame.Color.Num() == PixelCount)
	{
		Out.Pixels.Reserve(static_cast<int32>(PixelCount * 4));
		for (const FColor& Color : Frame.Color)
		{
			Out.Pixels.Add(Color.R); Out.Pixels.Add(Color.G);
			Out.Pixels.Add(Color.B); Out.Pixels.Add(Color.A);
		}
		return true;
	}
	if (Out.Encoding == TEXT("depth_f32le") && Frame.Mode == ERenderCameraMode::Depth
		&& Frame.Depth.Num() == PixelCount)
	{
		Out.Pixels.Reserve(static_cast<int32>(PixelCount * 4));
		for (float Cm : Frame.Depth)
		{
			if (FMath::IsNaN(Cm) || Cm < 0.0f)
			{
				OutError = TEXT("completed depth frame contains an invalid sample");
				return false;
			}
			const float Metres = (FMath::IsFinite(Cm)
				? FMath::Clamp(Cm, Pending.DepthNearCm, Pending.DepthFarCm)
				: Pending.DepthFarCm) / 100.0f;
			uint32 Bits = 0; FMemory::Memcpy(&Bits, &Metres, sizeof(Bits));
			WriteU32Le(Out.Pixels, Bits);
		}
		return true;
	}
	if (Out.Encoding == TEXT("seg_i32x2le") && RenderIsSegmentationMode(Frame.Mode)
		&& Frame.Seg.Num() == PixelCount * 2)
	{
		Out.Pixels.Reserve(static_cast<int32>(PixelCount * 8));
		for (int32 Value : Frame.Seg) WriteU32Le(Out.Pixels, static_cast<uint32>(Value));
		return true;
	}
	OutError = TEXT("completed camera frame payload does not match committed stream mode");
	return false;
}

}

struct FURLabCameraStreamProducer::FImpl
{
	mutable FCriticalSection StateMutex;
	FURLabCameraStreamDescriptor Descriptor;
	float DepthNearCm = 0.0f;
	float DepthFarCm = 0.0f;
	uint64 Sequence = 0;
	uint32 Capacity = 0;
	FURLabShmLiveObjectPtr ShmObject;
	TUniquePtr<FURLabLockedLatestFrameWriter> ShmWriter;
	FString ZmqEndpoint;
	TArray<uint8> ZmqTopic;
	void* ZmqContext = nullptr;
	void* ZmqSocket = nullptr;
	std::mutex PendingMutex;
	std::condition_variable Wakeup;
	TSharedPtr<FPendingCameraFrame, ESPMode::ThreadSafe> Pending;
	std::atomic<bool> bStopped{false};
	bool bReady = false;
	FString StartupError;
	FString BindEndpoint;
	FString AdvertisedEndpoint;
	std::thread Worker;

	void Run()
	{
		if (!BindEndpoint.IsEmpty())
		{
			ZmqContext = FURLabZmqContext::Acquire();
			ZmqSocket = ZmqContext ? zmq_socket(ZmqContext, ZMQ_PUB) : nullptr;
			if (!ZmqSocket) { Ready(TEXT("camera-stream ZMQ socket creation failed")); return; }
			const int Linger = 0, Hwm = 1;
			zmq_setsockopt(ZmqSocket, ZMQ_LINGER, &Linger, sizeof(Linger));
			zmq_setsockopt(ZmqSocket, ZMQ_SNDHWM, &Hwm, sizeof(Hwm));
			if (zmq_bind(ZmqSocket, TCHAR_TO_UTF8(*BindEndpoint)) != 0)
			{
				Ready(TEXT("camera-stream ZMQ bind failed")); CloseSocket(); return;
			}
			char Endpoint[512] = {};
			size_t EndpointBytes = sizeof(Endpoint);
			if (zmq_getsockopt(ZmqSocket, ZMQ_LAST_ENDPOINT, Endpoint, &EndpointBytes) != 0
				|| EndpointBytes <= 1)
			{
				Ready(TEXT("camera-stream ZMQ endpoint could not be resolved")); CloseSocket(); return;
			}
			ZmqEndpoint = AdvertisedEndpoint.IsEmpty()
				? FString(UTF8_TO_TCHAR(Endpoint)) : AdvertisedEndpoint;
		}
		Ready(FString());
		for (;;)
		{
			TSharedPtr<FPendingCameraFrame, ESPMode::ThreadSafe> Candidate;
			{
				std::unique_lock<std::mutex> Lock(PendingMutex);
				Wakeup.wait(Lock, [this] { return bStopped.load() || Pending.IsValid(); });
				if (bStopped.load()) break;
				Candidate = MoveTemp(Pending);
			}
			FURLabCameraStreamFrame Wire;
			TArray<uint8> Encoded;
			FString Error;
			if (!BuildWireFrame(*Candidate, Candidate->Sequence, Wire, Error)
				|| !FURLabCameraStreamFrameCodec::Encode(Wire, Encoded, Error)
				|| Encoded.Num() > Capacity)
			{
				continue;
			}
			TSharedRef<const TArray<uint8>, ESPMode::ThreadSafe> Snapshot =
				MakeShared<TArray<uint8>, ESPMode::ThreadSafe>(MoveTemp(Encoded));
			FScopeLock StateLock(&StateMutex);
			if (Candidate->Descriptor.StreamGeneration != Descriptor.StreamGeneration)
				continue;
			if (ShmWriter) ShmWriter->Submit(Snapshot, Error);
			if (ZmqSocket && zmq_send(ZmqSocket, ZmqTopic.GetData(), ZmqTopic.Num(),
				ZMQ_SNDMORE | ZMQ_DONTWAIT) >= 0)
			{
				zmq_send(ZmqSocket, Snapshot->GetData(), Snapshot->Num(), ZMQ_DONTWAIT);
			}
		}
		CloseSocket();
	}

	void Ready(FString Error)
	{
		std::lock_guard<std::mutex> Lock(PendingMutex);
		StartupError = MoveTemp(Error);
		bReady = true;
		Wakeup.notify_all();
	}

	void CloseSocket()
	{
		if (ZmqSocket) zmq_close(ZmqSocket);
		ZmqSocket = nullptr;
		if (ZmqContext) FURLabZmqContext::Release(ZmqContext);
		ZmqContext = nullptr;
	}
};

TUniquePtr<FURLabCameraStreamProducer> FURLabCameraStreamProducer::Create(
	const FURLabCameraStreamProducerConfig& Config, FString& OutError)
{
	OutError.Reset();
	if (Config.MaxFrameBytes <= 0 || Config.MaxFrameBytes > MAX_int32
		|| (Config.BindEndpoint.IsEmpty() && !Config.ShmRegion))
	{
		OutError = TEXT("camera-stream producer configuration is invalid");
		return nullptr;
	}
	TUniquePtr<FImpl> NewImpl = MakeUnique<FImpl>();
	NewImpl->Capacity = static_cast<uint32>(Config.MaxFrameBytes);
	NewImpl->Descriptor.ProducerBootId = MintId();
	NewImpl->Descriptor.StreamId = MintId();
	if (Config.ShmRegion)
	{
		FURLabShmExactStatus Status = Config.ShmRegion->CreateLiveObject(
			48 + Config.MaxFrameBytes, NewImpl->ShmObject);
		if (!Status)
		{
			OutError = Status.Message;
			return nullptr;
		}
		NewImpl->ShmWriter = FURLabLockedLatestFrameWriter::Create(
			NewImpl->ShmObject, NewImpl->Capacity, OutError);
		if (!NewImpl->ShmWriter) return nullptr;
	}
	if (!Config.BindEndpoint.IsEmpty())
	{
		NewImpl->BindEndpoint = Config.BindEndpoint;
		NewImpl->AdvertisedEndpoint = Config.AdvertisedEndpoint;
		FTCHARToUTF8 Topic(*MintId());
		NewImpl->ZmqTopic.Append(
			reinterpret_cast<const uint8*>(Topic.Get()), Topic.Length());
	}
	NewImpl->Worker = std::thread([Raw = NewImpl.Get()] { Raw->Run(); });
	std::unique_lock<std::mutex> Lock(NewImpl->PendingMutex);
	NewImpl->Wakeup.wait(Lock, [&NewImpl] { return NewImpl->bReady; });
	if (!NewImpl->StartupError.IsEmpty())
	{
		OutError = NewImpl->StartupError;
		NewImpl->bStopped.store(true);
		Lock.unlock();
		NewImpl->Wakeup.notify_all();
		if (NewImpl->Worker.joinable()) NewImpl->Worker.join();
		return nullptr;
	}
	return TUniquePtr<FURLabCameraStreamProducer>(
		new FURLabCameraStreamProducer(MoveTemp(NewImpl)));
}

FURLabCameraStreamProducer::FURLabCameraStreamProducer(TUniquePtr<FImpl>&& InImpl)
	: Impl(MoveTemp(InImpl))
{
}

FURLabCameraStreamProducer::~FURLabCameraStreamProducer()
{
	Stop();
}

bool FURLabCameraStreamProducer::CommitCamera(
	const FURLabCameraStreamConfiguration& Configuration,
	FURLabCameraStreamDescriptor& OutDescriptor, FString& OutError)
{
	OutError.Reset();
	if (!Impl || Impl->bStopped.load() || !IsValidConfiguration(Configuration))
	{
		OutError = TEXT("camera-stream configuration is invalid");
		return false;
	}
	FScopeLock Lock(&Impl->StateMutex);
	if (!Matches(Impl->Descriptor, Configuration)
		|| Impl->DepthNearCm != Configuration.DepthNearCm
		|| Impl->DepthFarCm != Configuration.DepthFarCm)
	{
		if (Impl->Descriptor.StreamGeneration == static_cast<uint64>(MAX_int64))
		{
			OutError = TEXT("camera-stream generation exhausted signed64 range");
			return false;
		}
		{
			std::lock_guard<std::mutex> PendingLock(Impl->PendingMutex);
			Impl->Pending.Reset();
		}
		if (Impl->ShmWriter && !Impl->ShmWriter->Clear(OutError)) return false;
		FURLabCameraStreamDescriptor Next = Impl->Descriptor;
		Next.SourceRevision = Configuration.SourceRevision;
		Next.CameraName = Configuration.CameraName;
		Next.CameraConfigurationRevision = Configuration.CameraConfigurationRevision;
		Next.Encoding = Configuration.Encoding;
		Next.Width = Configuration.Width;
		Next.Height = Configuration.Height;
		Next.StrideBytes = Configuration.StrideBytes;
		++Next.StreamGeneration;
		Impl->Descriptor = MoveTemp(Next);
		Impl->DepthNearCm = Configuration.DepthNearCm;
		Impl->DepthFarCm = Configuration.DepthFarCm;
		Impl->Sequence = 0;
	}
	OutDescriptor = Impl->Descriptor;
	return true;
}

bool FURLabCameraStreamProducer::SubmitCompleted(
	TSharedPtr<const FRenderCameraFrame, ESPMode::ThreadSafe> Frame, FString& OutError)
{
	OutError.Reset();
	if (!Impl || Impl->bStopped.load() || !Frame)
	{
		OutError = TEXT("camera-stream frame input is invalid");
		return false;
	}
	TSharedRef<FPendingCameraFrame, ESPMode::ThreadSafe> Candidate =
		MakeShared<FPendingCameraFrame, ESPMode::ThreadSafe>();
	{
		FScopeLock Lock(&Impl->StateMutex);
		if (Impl->Descriptor.SourceRevision.IsEmpty())
		{
			OutError = TEXT("camera-stream source is not committed");
			return false;
		}
		if (Impl->Sequence == static_cast<uint64>(MAX_int64))
		{
			OutError = TEXT("camera-stream sequence exhausted signed64 range");
			return false;
		}
		if (Frame->Identity.SceneRevision != Impl->Descriptor.SourceRevision
			|| Frame->CameraConfigurationRevision != Impl->Descriptor.CameraConfigurationRevision
			|| Frame->Width != Impl->Descriptor.Width || Frame->Height != Impl->Descriptor.Height)
		{
			OutError = TEXT("camera-stream frame does not match the committed camera configuration");
			return false;
		}
		Candidate->Frame = MoveTemp(Frame);
		Candidate->Descriptor = Impl->Descriptor;
		Candidate->Sequence = ++Impl->Sequence;
		Candidate->DepthNearCm = Impl->DepthNearCm;
		Candidate->DepthFarCm = Impl->DepthFarCm;
	}
	{
		std::lock_guard<std::mutex> Lock(Impl->PendingMutex);
		if (Impl->bStopped.load())
		{
			OutError = TEXT("camera-stream producer is stopped");
			return false;
		}
		Impl->Pending = Candidate;
		Impl->Wakeup.notify_all();
	}
	return true;
}

bool FURLabCameraStreamProducer::Describe(bool bShmEligible,
	FURLabCameraStreamDescriptor& OutDescriptor,FURLabCameraStreamDelivery& OutDelivery,
	FString& OutError) const
{
	OutError.Reset();
	if (!Impl || Impl->bStopped.load()) { OutError = TEXT("camera-stream producer is unavailable"); return false; }
	FURLabCameraStreamDescriptor Descriptor;
	FURLabCameraStreamDelivery Delivery;
	{
		FScopeLock Lock(&Impl->StateMutex);
		if (Impl->Descriptor.SourceRevision.IsEmpty())
		{
			OutError = TEXT("camera-stream source is not committed");
			return false;
		}
		Descriptor = Impl->Descriptor;
	}
	Delivery.ZmqEndpoint=Impl->ZmqEndpoint;Delivery.ZmqTopic=Impl->ZmqTopic;
	if (bShmEligible && Impl->ShmObject)
	{
		Delivery.ShmObject=Impl->ShmObject->GetDescriptor();
	}
	Delivery.MaxFrameBytes=Impl->Capacity;
	if (Delivery.ZmqEndpoint.IsEmpty() && !Delivery.ShmObject.IsSet())
	{
		OutError = TEXT("camera-stream has no eligible ready delivery");
		return false;
	}
	OutDescriptor=MoveTemp(Descriptor);OutDelivery=MoveTemp(Delivery);
	return true;
}

void FURLabCameraStreamProducer::Stop()
{
	if (!Impl) return;
	if (Impl->ShmWriter) Impl->ShmWriter->Stop();
	{
		std::lock_guard<std::mutex> Lock(Impl->PendingMutex);
		Impl->bStopped.store(true);
		Impl->Pending.Reset();
	}
	Impl->Wakeup.notify_all();
	if (Impl->Worker.joinable()) Impl->Worker.join();
}
