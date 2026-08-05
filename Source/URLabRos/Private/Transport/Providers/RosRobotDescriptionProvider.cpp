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
#include "MuJoCo/Core/AMjManager.h"

// Latched std_msgs/String on /<art>/robot_description carrying the URDF the
// manager exported on compile. Published once at Build (transient-local QoS
// delivers it to late-joining rviz / MoveIt), so Publish is a no-op.
class FMjRosRobotDescriptionProvider : public IMjRosOutputProvider
{
public:
	virtual FName GetProviderName() const override { return TEXT("robot_description"); }

	virtual void Build(FMjRosPublisherFactory& Factory, const FMjStateSnapshot& Snapshot) override
	{
		Pubs.Reset();
		const AAMjManager* Manager = Factory.GetManager();
		if (!Manager)
		{
			return;
		}
		const TMap<FName, FString>& Descriptions = Manager->GetRobotDescriptions();
		for (const FMjArticulationState& Art : Snapshot.Articulations)
		{
			const FString* Urdf = Descriptions.Find(Art.Name);
			if (!Urdf)
			{
				continue;
			}
			const FString Topic = FString::Printf(TEXT("/%s/robot_description"), *Art.Name.ToString());
			FMjRosPub Pub = Factory.CreateString(Topic);
			if (Pub.IsValid())
			{
				Pub.PublishString(*Urdf);
				Pubs.Add(MoveTemp(Pub));
			}
		}
	}

	virtual void Publish(const FMjStateSnapshot& /*Snapshot*/, int64 /*SimTimeNs*/) override
	{
		// Latched at Build; nothing to publish per step.
	}

	virtual int32 GetPublisherCountForTest() const override { return Pubs.Num(); }

private:
	TArray<FMjRosPub> Pubs;
};

REGISTER_MJ_ROS_OUTPUT_PROVIDER("robot_description", FMjRosRobotDescriptionProvider);
