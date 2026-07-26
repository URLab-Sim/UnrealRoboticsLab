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

// The whole body is fenced so that when ROS is not linked (URLAB_WITH_ROS2
// undefined or 0) this is an empty translation unit and no rcl symbol is
// referenced. Every caller of FURLabRosContext is fenced the same way, so the
// out-of-line definitions below are only needed when the feature is on.
#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2

#include "Transport/RosContext.h"
#include "Utils/URLabLogging.h"

#include "Ros/UrlabRclCore.h"

FURLabRosContext& FURLabRosContext::Get()
{
	static FURLabRosContext Instance;
	return Instance;
}

FURLabRosContext::~FURLabRosContext()
{
	Shutdown();
}

bool FURLabRosContext::Initialize()
{
	FScopeLock Lock(&Mutex);
	if (bInitAttempted)
	{
		return Context != nullptr;
	}
	bInitAttempted = true;

	Context = UrlabRcl_Init("urlab", "", -1);
	if (Context == nullptr)
	{
		UE_LOG(LogURLab, Warning,
			TEXT("ROS 2 unavailable: rcl context init failed (%hs). ROS publishing is disabled."),
			UrlabRcl_LastError());
		return false;
	}

	UE_LOG(LogURLab, Log, TEXT("ROS 2 context up (distro %hs, node 'urlab')."),
		UrlabRcl_DistroName());
	return true;
}

bool FURLabRosContext::IsAvailable() const
{
	FScopeLock Lock(&Mutex);
	return Context != nullptr;
}

void FURLabRosContext::Shutdown()
{
	FScopeLock Lock(&Mutex);
	if (Context != nullptr)
	{
		UrlabRcl_Shutdown(Context);
		Context = nullptr;
	}
	// Clear the attempt latch so an explicit teardown can be followed by a fresh
	// Initialize(). The latch exists only to stop a failed auto-init from being
	// retried every step, not to make shutdown terminal.
	bInitAttempted = false;
}

#endif  // URLAB_WITH_ROS2
