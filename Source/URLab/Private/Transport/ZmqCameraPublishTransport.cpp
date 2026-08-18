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
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#include "Transport/ZmqCameraPublishTransport.h"

#include "Containers/Queue.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "HAL/ThreadSafeBool.h"

#include "MuJoCo/Capture/MjCameraTypes.h"
#include "MuJoCo/Elements/MjCamera.h"
#include "Utils/URLabLogging.h"

#include "zmq.h"

// ---------------------------------------------------------------------------
// Publisher thread
// ---------------------------------------------------------------------------

/**
 * The background thread that publishes camera frames over ZeroMQ.
 *
 * One per streaming camera, owning its own context and PUB socket. The camera
 * pushes finished frames onto a queue; this thread drains to the freshest one
 * and sends it as [topic][meta + pixels], the same layout the SHM transport
 * writes, so both consumers parse identically.
 */
class FCameraZmqPublisher final : public FRunnable
{
public:
	FCameraZmqPublisher(const FString& InEndpoint, const FString& InTopic, FIntPoint InResolution)
		: RequestedEndpoint(InEndpoint)
		, BoundEndpoint(InEndpoint)
		, Topic(InTopic)
		, Resolution(InResolution)
	{
	}

	virtual ~FCameraZmqPublisher() override { Stop(); }

	virtual bool Init() override
	{
		ZmqContext = zmq_ctx_new();
		if (!ZmqContext)
		{
			UE_LOG(LogURLabNet, Error, TEXT("CameraZmqPublisher: zmq_ctx_new failed"));
			return false;
		}
		ZmqPublisher = zmq_socket(ZmqContext, ZMQ_PUB);
		if (!ZmqPublisher)
		{
			UE_LOG(LogURLabNet, Error, TEXT("CameraZmqPublisher: zmq_socket failed"));
			zmq_ctx_term(ZmqContext);
			ZmqContext = nullptr;
			return false;
		}

		// A live feed only cares about the FRESHEST frame, so keep the send queue
		// shallow: at HWM=1 the socket holds at most one frame in flight and a slow
		// consumer gets a near-latest frame instead of draining a backlog.
		int Hwm = 1;
		zmq_setsockopt(ZmqPublisher, ZMQ_SNDHWM, &Hwm, sizeof(Hwm));
		// LINGER=0, so a connected-but-not-reading subscriber can never block
		// zmq_ctx_term at shutdown; libzmq's default there is infinite.
		int Linger = 0;
		zmq_setsockopt(ZmqPublisher, ZMQ_LINGER, &Linger, sizeof(Linger));

		// Auto-increment the port on a bind conflict, so co-located cameras (and
		// co-located editor processes) do not fight over one port.
		FString BaseAddr = TEXT("tcp://0.0.0.0:");
		int32 Port = 5558;
		if (RequestedEndpoint.Contains(TEXT(":")))
		{
			FString Left, Right;
			RequestedEndpoint.Split(TEXT(":"), &Left, &Right, ESearchCase::IgnoreCase, ESearchDir::FromEnd);
			if (Right.IsNumeric())
			{
				Port = FCString::Atoi(*Right);
				BaseAddr = Left + TEXT(":");
			}
		}

		int Rc = -1;
		for (int32 i = 0; i < 10; ++i)
		{
			const FString TryEndpoint = FString::Printf(TEXT("%s%d"), *BaseAddr, Port + i);
			Rc = zmq_bind(ZmqPublisher, TCHAR_TO_UTF8(*TryEndpoint));
			if (Rc == 0)
			{
				BoundEndpoint = TryEndpoint;
				break;
			}
		}

		if (Rc != 0)
		{
			UE_LOG(LogURLabNet, Error,
				TEXT("CameraZmqPublisher failed to bind ZMQ after 10 retries, starting at %s"),
				*RequestedEndpoint);
			// Release the half-open socket and context, so a failed Init leaks nothing.
			zmq_close(ZmqPublisher);
			ZmqPublisher = nullptr;
			zmq_ctx_term(ZmqContext);
			ZmqContext = nullptr;
			return false;
		}

		UE_LOG(LogURLabNet, Log, TEXT("CameraZmqPublisher bound at %s [Topic: %s]"), *BoundEndpoint, *Topic);
		return true;
	}

	virtual uint32 Run() override
	{
		const int32 ExpectedPixels = Resolution.X * Resolution.Y;
		while (!bStopThread)
		{
			bool bSent = false;

			// Drain to the FRESHEST frame: if the producer outran us, skip the
			// backlog rather than send stale frames FIFO.
			FQueuedColorFrame ColorFrame;
			bool bHaveColor = false;
			while (FrameQueue.Dequeue(ColorFrame))
			{
				bHaveColor = true;
			}
			if (bHaveColor)
			{
				if (ColorFrame.Pixels.Num() == ExpectedPixels)
				{
					SendFrame(ColorFrame.Meta, ColorFrame.Pixels.GetData(),
						ColorFrame.Pixels.Num() * sizeof(FColor));
				}
				bSent = true;
			}

			FQueuedFloatFrame FloatFrame;
			bool bHaveFloat = false;
			while (FloatFrameQueue.Dequeue(FloatFrame))
			{
				bHaveFloat = true;
			}
			if (bHaveFloat)
			{
				if (FloatFrame.Pixels.Num() == ExpectedPixels)
				{
					SendFrame(FloatFrame.Meta, FloatFrame.Pixels.GetData(),
						FloatFrame.Pixels.Num() * sizeof(float));
				}
				bSent = true;
			}

			if (!bSent)
			{
				FPlatformProcess::Sleep(0.002f);
			}
		}
		return 0;
	}

	virtual void Stop() override { bStopThread = true; }

	virtual void Exit() override
	{
		if (ZmqPublisher)
		{
			zmq_close(ZmqPublisher);
			ZmqPublisher = nullptr;
		}
		if (ZmqContext)
		{
			zmq_ctx_term(ZmqContext);
			ZmqContext = nullptr;
		}
	}

	// Enqueue unconditionally: Run drains to the latest frame before sending and
	// the socket runs at HWM=1, so a backlog here collapses to the freshest frame
	// rather than streaming FIFO.
	void PushFrame(const TArray<FColor>& Pixels, const FMjCameraFrameMeta& Meta)
	{
		FrameQueue.Enqueue(FQueuedColorFrame{Meta, Pixels});
	}

	void PushFrame(const TArray<float>& Pixels, const FMjCameraFrameMeta& Meta)
	{
		FloatFrameQueue.Enqueue(FQueuedFloatFrame{Meta, Pixels});
	}

	const FString& GetBoundEndpoint() const { return BoundEndpoint; }

private:
	/**
	 * Publish one message as [topic][meta + pixels]. The metadata header is
	 * prepended to the pixel bytes in a single payload frame, so the ZMQ and SHM
	 * consumers parse an identical (meta, pixels) layout.
	 */
	void SendFrame(const FMjCameraFrameMeta& Meta, const void* Pixels, size_t PixelBytes)
	{
		if (FCameraZmqWorker::bPublishersPaused.load(std::memory_order_acquire))
		{
			return;
		}
		TArray<uint8> Payload;
		Payload.SetNumUninitialized(sizeof(FMjCameraFrameMeta) + static_cast<int32>(PixelBytes));
		FMemory::Memcpy(Payload.GetData(), &Meta, sizeof(FMjCameraFrameMeta));
		FMemory::Memcpy(Payload.GetData() + sizeof(FMjCameraFrameMeta), Pixels, PixelBytes);

		const FString TopicSpace = Topic + TEXT(" ");
		const FTCHARToUTF8 TopicUtf8(*TopicSpace);
		if (zmq_send(ZmqPublisher, TopicUtf8.Get(), TopicUtf8.Length(), ZMQ_SNDMORE) < 0)
		{
			// Topic frame dropped, e.g. at the high-water mark. Skip the body too,
			// or the multipart message desyncs.
			return;
		}
		// Best-effort feed: a body dropped under HWM=1 just means this frame is
		// skipped, and the next push carries a fresher one.
		zmq_send(ZmqPublisher, Payload.GetData(), Payload.Num(), 0);
	}

	// Each queued frame carries its metadata header, so Run can prepend it to the
	// published bytes and the client can associate the frame with the step that
	// produced it.
	struct FQueuedColorFrame
	{
		FMjCameraFrameMeta Meta;
		TArray<FColor> Pixels;
	};
	struct FQueuedFloatFrame
	{
		FMjCameraFrameMeta Meta;
		TArray<float> Pixels;
	};

	// Two queues, one per pixel format. Real and segmentation cameras drive the
	// colour queue, depth cameras the float one. A camera's CaptureMode never
	// changes after streaming starts, so only one is ever active per publisher.
	TQueue<FQueuedColorFrame, EQueueMode::Spsc> FrameQueue;
	TQueue<FQueuedFloatFrame, EQueueMode::Spsc> FloatFrameQueue;

	FString RequestedEndpoint;
	FString BoundEndpoint;
	FString Topic;
	FIntPoint Resolution = FIntPoint::ZeroValue;

	void* ZmqContext = nullptr;
	void* ZmqPublisher = nullptr;

	FThreadSafeBool bStopThread{false};
};

// ---------------------------------------------------------------------------
// Transport
// ---------------------------------------------------------------------------

UURLabZmqCameraPublishTransport::~UURLabZmqCameraPublishTransport()
{
	TransportShutdown();
}

void UURLabZmqCameraPublishTransport::Configure(const FString& InEndpoint, FIntPoint InResolution)
{
	Endpoint = InEndpoint;
	Resolution = InResolution;
}

void UURLabZmqCameraPublishTransport::OpenCameraChannel(int32 CameraIndex,
	const FString& CanonicalName, int32 StreamPortIndex)
{
	if (Channels.Contains(CameraIndex))
	{
		return;
	}

	FChannel Channel;
	Channel.Publisher = new FCameraZmqPublisher(Endpoint, CanonicalName, Resolution);
	Channel.Thread = FRunnableThread::Create(
		Channel.Publisher, TEXT("CameraZmqPublisherThread"), 0, TPri_BelowNormal);
	if (!Channel.Thread)
	{
		// Drop the publisher rather than hold a runnable nothing drives.
		UE_LOG(LogURLabNet, Error,
			TEXT("[ZmqCameraPublishTransport] failed to create ZMQ publisher thread for '%s'; "
				 "disabling ZMQ broadcast for this camera"),
			*CanonicalName);
		delete Channel.Publisher;
		return;
	}

	Channels.Add(CameraIndex, Channel);
}

void UURLabZmqCameraPublishTransport::PublishCameraFrame(const FMjCameraWireFrame& Frame)
{
	const FChannel* Found = Channels.Find(Frame.CameraIndex);
	if (!Found || !Found->Publisher)
	{
		return;
	}

	FMjCameraFrameMeta Meta;
	Meta.FrameId = Frame.FrameId;
	Meta.SimTime = Frame.SimTime;
	Meta.Width = static_cast<uint32>(Frame.Width);
	Meta.Height = static_cast<uint32>(Frame.Height);
	Meta.CaptureUnixTime = Frame.CaptureUnixSeconds;

	if (Frame.Dtype == FName(TEXT("float32")))
	{
		const int32 Count = Frame.NumBytes / static_cast<int32>(sizeof(float));
		TArray<float> Pixels;
		Pixels.SetNumUninitialized(Count);
		FMemory::Memcpy(Pixels.GetData(), Frame.Bytes, Count * sizeof(float));
		Found->Publisher->PushFrame(Pixels, Meta);
	}
	else
	{
		const int32 Count = Frame.NumBytes / static_cast<int32>(sizeof(FColor));
		TArray<FColor> Pixels;
		Pixels.SetNumUninitialized(Count);
		FMemory::Memcpy(Pixels.GetData(), Frame.Bytes, Count * sizeof(FColor));
		Found->Publisher->PushFrame(Pixels, Meta);
	}
}

void UURLabZmqCameraPublishTransport::CloseCameraChannel(int32 CameraIndex)
{
	FChannel Channel;
	if (!Channels.RemoveAndCopyValue(CameraIndex, Channel))
	{
		return;
	}
	if (Channel.Thread)
	{
		Channel.Thread->Kill(true);
		delete Channel.Thread;
	}
	delete Channel.Publisher;
}

void UURLabZmqCameraPublishTransport::TransportShutdown()
{
	TArray<int32> Open;
	Channels.GetKeys(Open);
	for (int32 CameraIndex : Open)
	{
		CloseCameraChannel(CameraIndex);
	}
}

FString UURLabZmqCameraPublishTransport::GetBoundEndpoint(int32 CameraIndex) const
{
	if (const FChannel* Found = Channels.Find(CameraIndex))
	{
		if (Found->Publisher)
		{
			return Found->Publisher->GetBoundEndpoint();
		}
	}
	return FString();
}
