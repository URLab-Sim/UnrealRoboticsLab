// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#include "Transport/PoseStreamProducer.h"

#include "Misc/Guid.h"
#include "Misc/ScopeLock.h"
#include "Transport/LockedLatestFrame.h"
#include "Transport/PoseStreamFrameCodec.h"
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

void PutDoubleLe(uint8* Out, double Value)
{
	uint64 Bits = 0;
	FMemory::Memcpy(&Bits, &Value, sizeof(Bits));
	for (int32 Index = 0; Index < 8; ++Index) Out[Index] = uint8(Bits >> (Index * 8));
}
}

struct FURLabPoseStreamProducer::FImpl
{
	mutable FCriticalSection StateMutex;
	FURLabPoseStreamDescriptor Descriptor;
	uint64 Sequence = 0;
	uint32 Capacity = 0;
	FURLabShmLiveObjectPtr ShmObject;
	TUniquePtr<FURLabLockedLatestFrameWriter> ShmWriter;
	FURLabPoseStreamZmqDelivery ZmqDelivery;
	void* ZmqContext = nullptr;
	void* ZmqSocket = nullptr;
	std::mutex PendingMutex;
	std::condition_variable Wakeup;
	TSharedPtr<const TArray<uint8>, ESPMode::ThreadSafe> Pending;
	std::atomic<bool> bStopped{false};
	bool bReady = false;
	FString StartupError;
	FString BindEndpoint;
	FString AdvertisedEndpoint;
	std::thread Worker;

	void Run()
	{
		ZmqContext = FURLabZmqContext::Acquire();
		ZmqSocket = ZmqContext ? zmq_socket(ZmqContext, ZMQ_PUB) : nullptr;
		if (!ZmqSocket)
		{
			Ready(TEXT("pose-stream ZMQ socket creation failed"));
			return;
		}
		const int Linger = 0, Hwm = 1;
		zmq_setsockopt(ZmqSocket, ZMQ_LINGER, &Linger, sizeof(Linger));
		zmq_setsockopt(ZmqSocket, ZMQ_SNDHWM, &Hwm, sizeof(Hwm));
		if (zmq_bind(ZmqSocket, TCHAR_TO_UTF8(*BindEndpoint)) != 0)
		{
			Ready(TEXT("pose-stream ZMQ bind failed"));
			CloseSocket();
			return;
		}
		char Endpoint[512] = {};
		size_t EndpointBytes = sizeof(Endpoint);
		if (zmq_getsockopt(ZmqSocket, ZMQ_LAST_ENDPOINT, Endpoint, &EndpointBytes) != 0
			|| EndpointBytes <= 1)
		{
			Ready(TEXT("pose-stream ZMQ endpoint could not be resolved"));
			CloseSocket();
			return;
		}
		ZmqDelivery.Endpoint = AdvertisedEndpoint.IsEmpty()
			? FString(UTF8_TO_TCHAR(Endpoint)) : AdvertisedEndpoint;
		Ready(FString());
		for (;;)
		{
			TSharedPtr<const TArray<uint8>, ESPMode::ThreadSafe> Frame;
			{
				std::unique_lock<std::mutex> Lock(PendingMutex);
				Wakeup.wait(Lock, [this] { return bStopped.load() || Pending.IsValid(); });
				if (bStopped.load()) break;
				Frame = MoveTemp(Pending);
			}
			if (zmq_send(ZmqSocket, ZmqDelivery.Topic.GetData(), ZmqDelivery.Topic.Num(),
				ZMQ_SNDMORE | ZMQ_DONTWAIT) >= 0)
			{
				zmq_send(ZmqSocket, Frame->GetData(), Frame->Num(), ZMQ_DONTWAIT);
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

TUniquePtr<FURLabPoseStreamProducer> FURLabPoseStreamProducer::Create(
	const FURLabPoseStreamProducerConfig& Config, FString& OutError)
{
	OutError.Reset();
	if (Config.MaxPayloadBytes <= 0 || Config.MaxPayloadBytes > MAX_int32
		|| (Config.BindEndpoint.IsEmpty() && !Config.ShmRegion))
	{
		OutError = TEXT("pose-stream producer configuration is invalid");
		return nullptr;
	}
	TUniquePtr<FImpl> NewImpl = MakeUnique<FImpl>();
	NewImpl->Capacity = static_cast<uint32>(Config.MaxPayloadBytes);
	NewImpl->Descriptor.ProducerBootId = MintId();
	NewImpl->Descriptor.StreamId = MintId();
	if (Config.ShmRegion)
	{
		FURLabShmExactStatus Status = Config.ShmRegion->CreateLiveObject(
			48 + Config.MaxPayloadBytes, NewImpl->ShmObject);
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
		NewImpl->ZmqDelivery.Topic.Append(
			reinterpret_cast<const uint8*>(Topic.Get()), Topic.Length());
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
	}
	return TUniquePtr<FURLabPoseStreamProducer>(
		new FURLabPoseStreamProducer(MoveTemp(NewImpl)));
}

FURLabPoseStreamProducer::FURLabPoseStreamProducer(TUniquePtr<FImpl>&& InImpl)
	: Impl(MoveTemp(InImpl))
{
}

FURLabPoseStreamProducer::~FURLabPoseStreamProducer()
{
	Stop();
}

bool FURLabPoseStreamProducer::CommitSource(const FString& SourceRevision,
	FURLabPoseStreamDescriptor& OutDescriptor, FString& OutError)
{
	OutError.Reset();
	if (!Impl || Impl->bStopped.load() || SourceRevision.IsEmpty())
	{
		OutError = TEXT("pose-stream source revision must be non-empty");
		return false;
	}
	FScopeLock Lock(&Impl->StateMutex);
	if (Impl->Descriptor.SourceRevision != SourceRevision)
	{
		if (Impl->Descriptor.StreamGeneration == static_cast<uint64>(MAX_int64))
		{
			OutError = TEXT("pose-stream generation exhausted signed64 range");
			return false;
		}
		Impl->Descriptor.SourceRevision = SourceRevision;
		++Impl->Descriptor.StreamGeneration;
		Impl->Sequence = 0;
	}
	OutDescriptor = Impl->Descriptor;
	return true;
}

bool FURLabPoseStreamProducer::Publish(uint64 FrameId, double SourceTime,
	bool bHasSourceTime, uint64 ClockEpoch,
	TConstArrayView<double> RightHandedMetresWxyz, FString& OutError)
{
	const FRenderFrame Empty;
	return Publish(FrameId,SourceTime,bHasSourceTime,ClockEpoch,RightHandedMetresWxyz,Empty,OutError);
}

bool FURLabPoseStreamProducer::Publish(uint64 FrameId, double SourceTime,
	bool bHasSourceTime, uint64 ClockEpoch,
	TConstArrayView<double> RightHandedMetresWxyz,const FRenderFrame& DynamicFrame,FString& OutError)
{
	OutError.Reset();
	if (!Impl || Impl->bStopped.load() || RightHandedMetresWxyz.IsEmpty()
		|| RightHandedMetresWxyz.Num() % 7 != 0 || FrameId > static_cast<uint64>(MAX_int64)
		|| ClockEpoch > static_cast<uint64>(MAX_int64)
		|| (bHasSourceTime && !FMath::IsFinite(SourceTime)))
	{
		OutError = TEXT("pose-stream frame input is invalid");
		return false;
	}
	for (double Value : RightHandedMetresWxyz)
		if (!FMath::IsFinite(Value)) { OutError = TEXT("pose-stream transforms must be finite"); return false; }
	const int64 PayloadBytes = int64(RightHandedMetresWxyz.Num()) * sizeof(double);
	if (PayloadBytes > Impl->Capacity)
	{
		OutError = TEXT("pose-stream frame exceeds configured capacity");
		return false;
	}
	FURLabPoseStreamFrame Frame;
	{
		FScopeLock Lock(&Impl->StateMutex);
		if (Impl->Descriptor.SourceRevision.IsEmpty())
		{
			OutError = TEXT("pose-stream source is not committed");
			return false;
		}
		if (Impl->Sequence == static_cast<uint64>(MAX_int64))
		{
			OutError = TEXT("pose-stream sequence exhausted signed64 range");
			return false;
		}
		Frame.ProducerBootId = Impl->Descriptor.ProducerBootId;
		Frame.StreamId = Impl->Descriptor.StreamId;
		Frame.StreamGeneration = Impl->Descriptor.StreamGeneration;
		Frame.SourceRevision = Impl->Descriptor.SourceRevision;
		Frame.Sequence = ++Impl->Sequence;
	}
	Frame.FrameId = FrameId;
	Frame.bHasSourceTime = bHasSourceTime;
	Frame.SourceTime = SourceTime;
	Frame.ClockEpoch = ClockEpoch;
	Frame.Deformables=DynamicFrame.Deformables;
	Frame.DebugMeshes=DynamicFrame.DebugMeshes;
	Frame.Payload.SetNumUninitialized(PayloadBytes);
	for (int32 Index = 0; Index < RightHandedMetresWxyz.Num(); ++Index)
		PutDoubleLe(Frame.Payload.GetData() + Index * sizeof(double), RightHandedMetresWxyz[Index]);
	TArray<uint8> Encoded;
	if (!FURLabPoseStreamFrameCodec::Encode(Frame, Encoded, OutError)) return false;
	if (Encoded.Num() > Impl->Capacity)
	{
		OutError = TEXT("encoded pose-stream frame exceeds configured capacity");
		return false;
	}
	TSharedRef<const TArray<uint8>, ESPMode::ThreadSafe> Snapshot =
		MakeShared<TArray<uint8>, ESPMode::ThreadSafe>(MoveTemp(Encoded));
	if (Impl->ShmWriter && !Impl->ShmWriter->Submit(Snapshot, OutError)) return false;
	if (!Impl->ZmqDelivery.Endpoint.IsEmpty())
	{
		std::lock_guard<std::mutex> Lock(Impl->PendingMutex);
		if (Impl->bStopped.load())
		{
			OutError = TEXT("pose-stream producer is stopped");
			return false;
		}
		Impl->Pending = Snapshot;
		Impl->Wakeup.notify_all();
	}
	return true;
}

bool FURLabPoseStreamProducer::DescribeForSession(bool bShmEligible,
	FURLabPoseStreamDescription& OutDescription, FString& OutError) const
{
	OutError.Reset();
	if (!Impl || Impl->bStopped.load()) { OutError = TEXT("pose-stream producer is unavailable"); return false; }
	FURLabPoseStreamDescription Local;
	{
		FScopeLock Lock(&Impl->StateMutex);
		if (Impl->Descriptor.SourceRevision.IsEmpty())
		{
			OutError = TEXT("pose-stream source is not committed");
			return false;
		}
		Local.Stream = Impl->Descriptor;
	}
	if (!Impl->ZmqDelivery.Endpoint.IsEmpty()) Local.Zmq = Impl->ZmqDelivery;
	if (bShmEligible && Impl->ShmObject)
	{
		FURLabPoseStreamShmDelivery Shm;
		Shm.Object = Impl->ShmObject->GetDescriptor();
		Shm.MaxPayloadBytes = Impl->Capacity;
		Local.Shm = MoveTemp(Shm);
	}
	if (!Local.Zmq.IsSet() && !Local.Shm.IsSet())
	{
		OutError = TEXT("pose-stream has no eligible ready delivery");
		return false;
	}
	OutDescription = MoveTemp(Local);
	return true;
}

void FURLabPoseStreamProducer::Stop()
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
