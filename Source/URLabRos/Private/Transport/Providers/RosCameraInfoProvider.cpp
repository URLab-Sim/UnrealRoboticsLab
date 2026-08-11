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

#include "Transport/RosOutputProvider.h"
#include "Transport/RosStateEstimation.h"
#include "State/MjStateTypes.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Elements/MjCamera.h"

// sensor_msgs/CameraInfo on /<art>/<camera>/camera_info, one publisher per camera,
// carrying the pinhole intrinsics a vision node needs to interpret the paired
// image stream. The topic base and frame id are the camera's canonical identity
// ("<art>/<part>", the same key the image stream uses), so camera_info sits
// alongside the image.
//
// Intrinsic derivation: a MuJoCo camera is specified either by a vertical FOV or
// by intrinsics, and DerivedFovy is whichever of the two resolved, so it is what
// this reads rather than the `fovy` attribute. The standard pinhole K is
// fy = (height/2) / tan(fovy/2), fx = fy (square pixels; the horizontal FOV
// follows from the width), principal point at the image centre. If the camera
// stores an explicit pixel focal length (focalpixel, MuJoCo's intrinsic
// override), that is used verbatim instead of the fovy derivation.
// Intrinsics are constant, so K / width / height / frame are fixed at create and
// Publish only restamps.
class FMjRosCameraInfoProvider : public IMjRosOutputProvider
{
public:
	virtual FName GetProviderName() const override { return TEXT("camera_info"); }

	virtual void Build(FMjRosPublisherFactory& Factory, const FMjStateSnapshot& /*Snapshot*/) override
	{
		Pubs.Reset();
		const AAMjManager* Manager = Factory.GetManager();
		if (!Manager)
		{
			return;
		}

		// One publisher per distinct camera (canonical name), first writer wins so a
		// sanitize collision cannot hide a distinct camera - mirrors the camera name
		// map the RPC / streaming paths build.
		TSet<FString> Seen;
		auto AddCamera = [this, &Factory, &Seen](UMjCamera* Cam) {
			if (!Cam)
			{
				return;
			}
			const FString Canonical = Cam->GetCanonicalName();
			if (Canonical.IsEmpty() || Seen.Contains(Canonical))
			{
				return;
			}
			Seen.Add(Canonical);

			const FIntPoint Res = Cam->CaptureResolution();
			double K[9];
			DeriveK(Cam, Res.X, Res.Y, K);

			const FString Topic = FString::Printf(TEXT("/%s/camera_info"), *Canonical);
			FMjRosPub Pub = Factory.CreateCameraInfo(Topic, Canonical, Res.X, Res.Y, K);
			if (Pub.IsValid())
			{
				Pubs.Add(MoveTemp(Pub));
			}
		};

		for (AMjArticulation* Art : Manager->GetAllArticulations())
		{
			if (!Art)
			{
				continue;
			}
			TArray<UMjCamera*> Cameras;
			Art->GetComponents<UMjCamera>(Cameras);
			for (UMjCamera* Cam : Cameras)
			{
				AddCamera(Cam);
			}
		}
		// Manager-owned (global) cameras not attached to any articulation.
		TArray<UMjCamera*> GlobalCameras;
		Manager->GetComponents<UMjCamera>(GlobalCameras);
		for (UMjCamera* Cam : GlobalCameras)
		{
			AddCamera(Cam);
		}
	}

	virtual void Publish(const FMjStateSnapshot& /*Snapshot*/, int64 SimTimeNs) override
	{
		for (FMjRosPub& Pub : Pubs)
		{
			Pub.PublishCameraInfo(SimTimeNs);
		}
	}

	virtual int32 GetPublisherCountForTest() const override { return Pubs.Num(); }

private:
	static void DeriveK(const UMjCamera* Cam, int32 Width, int32 Height, double OutK[9])
	{
		MjRosStateEstimation::PinholeKFromFovy(Cam->DerivedFovy, Width, Height, OutK);
		// Explicit pixel focal length overrides the fovy-derived focal length when
		// the camera stores one (MuJoCo's focalpixel intrinsic).
		if (Cam->HasFocalpixel())
		{
			const TArray<float> FocalPixel = Cam->GetFocalpixel();
			if (FocalPixel.Num() >= 2 && FocalPixel[0] > 0 && FocalPixel[1] > 0)
			{
				OutK[0] = static_cast<double>(FocalPixel[0]); // fx
				OutK[4] = static_cast<double>(FocalPixel[1]); // fy
			}
		}
	}

	TArray<FMjRosPub> Pubs;
};

REGISTER_MJ_ROS_OUTPUT_PROVIDER("camera_info", FMjRosCameraInfoProvider);
