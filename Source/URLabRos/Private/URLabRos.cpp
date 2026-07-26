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

#include "URLabRos.h"

#include "Transport/MjExternalTransportProvider.h"
#include "Transport/RpcTransport.h"
#include "Transport/PublishTransport.h"
#include "Transport/RosRpcTransport.h"
#include "Transport/RosPublishTransport.h"
#include "Bridge/BridgeServer.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Components/Sensors/MjCameraFrameBus.h"
#include "URLabRosLog.h"

#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2
#include "Transport/RosContext.h"
#include "Ros/UrlabRclCore.h"
#endif

DEFINE_LOG_CATEGORY(LogURLabRos);

// Opaque rcl image publisher handle. Forward-declared unconditionally so the
// per-camera pointer map compiles in an ROS-off build too; the handle is only
// created / used inside the URLAB_WITH_ROS2 fence.
struct UrlabRclImagePub;

namespace
{
/** Factory for the ROS control RPC transport, installed as the core's external
 *  control-RPC hook. Creates the transport with the bridge as outer and wires
 *  ownership; the core calls TransportInit and stores the result. */
UURLabRpcTransport* MakeRosControlRpcTransport(UURLabBridgeServer* Bridge)
{
	UURLabRosRpcTransport* Ros = NewObject<UURLabRosRpcTransport>(Bridge, NAME_None);
	Ros->SetOwningBridge(Bridge);
	return Ros;
}

/** Factory for the ROS state publish transport, installed as the core's external
 *  state-publish hook. Creates the transport with the manager as outer; the
 *  transport registers itself as an IMjStateConsumer in TransportInit. */
UURLabPublishTransport* MakeRosStatePublishTransport(AAMjManager* Manager)
{
	return NewObject<UURLabRosPublishTransport>(Manager, NAME_None);
}

/**
 * @class FRosCameraImageSink
 * @brief Publishes camera frames as ROS `sensor_msgs/Image`, subscribing to the
 *        core's transport-neutral FMjCameraFrameBus.
 *
 * One rcl Image publisher per camera, keyed by canonical name, created lazily on
 * the first frame (topic `/<art>/<part>/image`) and released when the camera
 * stops streaming. Runs on the game thread (the bus fires there), matching the
 * former in-camera image sink. In an ROS-off build the handlers are no-ops.
 */
class FRosCameraImageSink
{
public:
	void Install()
	{
		FrameHandle = FMjCameraFrameBus::Get().OnFrameReady.AddRaw(
			this, &FRosCameraImageSink::OnFrameReady);
		StopHandle = FMjCameraFrameBus::Get().OnStreamStopped.AddRaw(
			this, &FRosCameraImageSink::OnStreamStopped);
	}

	void Uninstall()
	{
		FMjCameraFrameBus::Get().OnFrameReady.Remove(FrameHandle);
		FMjCameraFrameBus::Get().OnStreamStopped.Remove(StopHandle);
		FrameHandle.Reset();
		StopHandle.Reset();
#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2
		for (TPair<FString, UrlabRclImagePub*>& Pair : Pubs)
		{
			UrlabRcl_DestroyImagePub(Pair.Value);
		}
#endif
		Pubs.Reset();
	}

private:
	void OnFrameReady(const FMjCameraFramePayload& Frame)
	{
#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2
		if (!FURLabRosContext::Get().IsAvailable() || Frame.Data == nullptr)
		{
			return;
		}
		UrlabRclImagePub* Pub = FindOrCreatePub(Frame);
		if (!Pub)
		{
			return;
		}
		const int64 SimTimeNs = static_cast<int64>(Frame.SimTime * 1.0e9);
		UrlabRcl_PublishImage(Pub, Frame.Data, Frame.RowStrideBytes, SimTimeNs);
#endif
	}

	void OnStreamStopped(const FString& CanonicalName)
	{
#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2
		if (UrlabRclImagePub** Found = Pubs.Find(CanonicalName))
		{
			UrlabRcl_DestroyImagePub(*Found);
			Pubs.Remove(CanonicalName);
		}
#endif
	}

#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2
	UrlabRclImagePub* FindOrCreatePub(const FMjCameraFramePayload& Frame)
	{
		if (UrlabRclImagePub** Found = Pubs.Find(Frame.CanonicalName))
		{
			return *Found;
		}
		UrlabRclContext* Ctx = FURLabRosContext::Get().GetHandle();
		if (!Ctx)
		{
			return nullptr;
		}
		const FString Topic = FString::Printf(TEXT("/%s/image"), *Frame.CanonicalName);
		// FColor pixels are BGRA8; depth is single-channel float32.
		const char* Encoding = Frame.bDepth ? "32FC1" : "bgra8";
		UrlabRclImagePub* Pub = UrlabRcl_CreateImagePub(Ctx, TCHAR_TO_UTF8(*Topic),
			TCHAR_TO_UTF8(*Frame.CanonicalName), Frame.Width, Frame.Height, Encoding);
		if (!Pub)
		{
			UE_LOG(LogURLabRos, Warning,
				TEXT("[URLabRos] '%s' ROS image publisher create failed (%hs)"),
				*Frame.CanonicalName, UrlabRcl_LastError());
			return nullptr;
		}
		UE_LOG(LogURLabRos, Log, TEXT("[URLabRos] '%s' ROS image broadcast at %s"),
			*Frame.CanonicalName, *Topic);
		Pubs.Add(Frame.CanonicalName, Pub);
		return Pub;
	}
#endif

	TMap<FString, UrlabRclImagePub*> Pubs;
	FDelegateHandle FrameHandle;
	FDelegateHandle StopHandle;
};

FRosCameraImageSink GCameraImageSink;
}  // namespace

void FURLabRosModule::StartupModule()
{
	FMjExternalTransportProvider::MakeControlRpcTransport.BindStatic(&MakeRosControlRpcTransport);
	FMjExternalTransportProvider::MakeStatePublishTransport.BindStatic(&MakeRosStatePublishTransport);
	GCameraImageSink.Install();
}

void FURLabRosModule::ShutdownModule()
{
	GCameraImageSink.Uninstall();
	FMjExternalTransportProvider::MakeControlRpcTransport.Unbind();
	FMjExternalTransportProvider::MakeStatePublishTransport.Unbind();
}

IMPLEMENT_MODULE(FURLabRosModule, URLabRos)
