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
#include "State/MjStateTypes.h"

// rosgraph_msgs/Clock on /clock, one process-wide publisher. The transport
// projects the IR clock to nanoseconds once per step and passes it to every
// provider, so this one just forwards it.
class FMjRosClockProvider : public IMjRosOutputProvider
{
public:
	virtual FName GetProviderName() const override { return TEXT("clock"); }

	virtual void Build(FMjRosPublisherFactory& Factory, const FMjStateSnapshot& /*Snapshot*/) override
	{
		ClockPub = Factory.CreateClock();
	}

	virtual void Publish(const FMjStateSnapshot& /*Snapshot*/, int64 SimTimeNs) override
	{
		ClockPub.PublishClock(SimTimeNs);
	}

	virtual int32 GetPublisherCountForTest() const override { return ClockPub.IsValid() ? 1 : 0; }

private:
	FMjRosPub ClockPub;
};

REGISTER_MJ_ROS_OUTPUT_PROVIDER("clock", FMjRosClockProvider);
