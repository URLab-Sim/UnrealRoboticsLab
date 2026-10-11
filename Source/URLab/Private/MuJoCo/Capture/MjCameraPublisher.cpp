// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
#include "MuJoCo/Capture/MjCameraPublisher.h"

#include "MuJoCo/Capture/MjCameraFrameBus.h"
#include "Render/RenderCameraComponent.h"
#include "Transport/CameraPublishTransport.h"
#include "Transport/MjExternalTransportProvider.h"
#include "Transport/ShmCameraPublishTransport.h"
#include "Transport/ZmqCameraPublishTransport.h"
#include "Wire/UrlabWire.gen.h"

std::atomic<bool> FCameraZmqWorker::bPublishersPaused{false};

void UMjCameraPublisher::Configure(URenderCameraComponent* InCamera, int32 InChannelIndex,
	const FString& InEndpoint, const FURLabShmSession& InSession)
{
	Shutdown();
	Camera = InCamera;
	ChannelIndex = InChannelIndex;
	Endpoint = InEndpoint;
	Session = InSession;
	if (InCamera) InCamera->OnFrameCompleted.AddUObject(this, &UMjCameraPublisher::HandleFrame);
}

void UMjCameraPublisher::SetBroadcastEnabled(bool bZmq, bool bShm)
{
	if (bEnableZmq == bZmq && bEnableShm == bShm) return;
	Shutdown();
	bEnableZmq = bZmq;
	bEnableShm = bShm;
	if (Camera.IsValid())
	{
		Camera->OnFrameCompleted.AddUObject(this, &UMjCameraPublisher::HandleFrame);
		if (bEnableZmq || bEnableShm) Open();
		Camera->bContinuousCapture = bEnableZmq || bEnableShm;
		Camera->SetStreamingEnabled(bEnableZmq || bEnableShm || Camera->IsCaptureActive());
	}
}

void UMjCameraPublisher::Open()
{
	URenderCameraComponent* RenderCamera = Camera.Get();
	if (!RenderCamera) return;
	if (bEnableZmq)
	{
		UURLabZmqCameraPublishTransport* Zmq = NewObject<UURLabZmqCameraPublishTransport>(this);
		Zmq->Configure(Endpoint, RenderCamera->CaptureResolution());
		Publishers.Add(Zmq);
	}
	if (bEnableShm)
	{
		UURLabShmCameraPublishTransport* Shm = NewObject<UURLabShmCameraPublishTransport>(this);
		Shm->Configure(RenderCamera->CaptureResolution(), Session);
		Publishers.Add(Shm);
	}
	if (FMjExternalTransportProvider::MakeCameraPublishTransport.IsBound())
	{
		if (UURLabCameraPublishTransport* External =
			FMjExternalTransportProvider::MakeCameraPublishTransport.Execute(this))
			Publishers.Add(External);
	}
	for (UURLabCameraPublishTransport* Publisher : Publishers)
	{
		if (!Publisher) continue;
		Publisher->TransportInit();
		Publisher->OpenCameraChannel(ChannelIndex, RenderCamera->GetCanonicalName());
	}
}

void UMjCameraPublisher::Shutdown()
{
	if (URenderCameraComponent* RenderCamera = Camera.Get())
	{
		RenderCamera->OnFrameCompleted.RemoveAll(this);
		RenderCamera->bContinuousCapture = false;
	}
	for (UURLabCameraPublishTransport* Publisher : Publishers)
	{
		if (!Publisher) continue;
		Publisher->CloseCameraChannel(ChannelIndex);
		Publisher->TransportShutdown();
	}
	if (Publishers.Num() > 0) FMjCameraFrameBus::Get().OnStreamStopped.Broadcast(GetCanonicalName());
	Publishers.Reset();
}

FString UMjCameraPublisher::GetCanonicalName() const
{
	return Camera.IsValid() ? Camera->GetCanonicalName() : FString();
}

FString UMjCameraPublisher::GetActualZmqEndpoint() const
{
	for (const UURLabCameraPublishTransport* Publisher : Publishers)
	{
		if (const UURLabZmqCameraPublishTransport* Zmq =
			Cast<UURLabZmqCameraPublishTransport>(Publisher))
		{
			const FString Bound = Zmq->GetBoundEndpoint(ChannelIndex);
			if (!Bound.IsEmpty()) return Bound;
		}
	}
	return Endpoint;
}

void UMjCameraPublisher::HandleFrame(const FRenderCameraFrame& Frame)
{
	URenderCameraComponent* RenderCamera = Camera.Get();
	if (!RenderCamera || FCameraZmqWorker::bPublishersPaused.load(std::memory_order_acquire)) return;
	FMjCameraWireFrame Wire;
	Wire.CameraIndex = ChannelIndex;
	Wire.Width = Frame.Width;
	Wire.Height = Frame.Height;
	Wire.FrameId = Frame.Identity.FrameId;
	Wire.SimTime = Frame.Identity.bHasSourceTime ? Frame.Identity.SourceTime : 0.0;
	Wire.CaptureUnixSeconds = Frame.CaptureUnixTime;
	Wire.Channels = URenderCameraComponent::ChannelsForMode(Frame.Mode);
	if (Frame.Mode == ERenderCameraMode::Depth)
	{
		if (Frame.Depth.IsEmpty()) return;
		Wire.Dtype = FName(URLabWire::PixelDtype::Float32);
		Wire.Bytes = reinterpret_cast<const uint8*>(Frame.Depth.GetData());
		Wire.NumBytes = Frame.Depth.Num() * sizeof(float);
	}
	else if (RenderIsSegmentationMode(Frame.Mode))
	{
		if (Frame.Seg.IsEmpty()) return;
		Wire.Dtype = FName(URLabWire::PixelDtype::Int32);
		Wire.Bytes = reinterpret_cast<const uint8*>(Frame.Seg.GetData());
		Wire.NumBytes = Frame.Seg.Num() * sizeof(int32);
	}
	else
	{
		if (Frame.Color.IsEmpty()) return;
		Wire.Dtype = FName(URLabWire::PixelDtype::Bgra8);
		Wire.Bytes = reinterpret_cast<const uint8*>(Frame.Color.GetData());
		Wire.NumBytes = Frame.Color.Num() * sizeof(FColor);
	}
	for (UURLabCameraPublishTransport* Publisher : Publishers)
		if (Publisher) Publisher->PublishCameraFrame(Wire);

	FMjCameraFramePayload Payload;
	Payload.CanonicalName = RenderCamera->GetCanonicalName();
	Payload.Width = Frame.Width;
	Payload.Height = Frame.Height;
	Payload.Mode = Frame.Mode;
	Payload.SimTime = Wire.SimTime;
	Payload.FrameId = Wire.FrameId;
	Payload.Data = Wire.Bytes;
	Payload.DataNumBytes = Wire.NumBytes;
	Payload.RowStrideBytes = Frame.Height > 0 ? Wire.NumBytes / Frame.Height : 0;
	if (IsInGameThread())
	{
		FMjCameraFrameBus::Get().OnFrameReady.Broadcast(Payload);
	}
}
