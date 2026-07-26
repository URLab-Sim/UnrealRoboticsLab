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
#include "Transport/RosPublishTransport.h"
#include "State/MjStateTypes.h"

// geometry_msgs/TwistStamped on /<art>/cmd_twist, one publisher per articulation
// that carries a twist command.
class FMjRosTwistProvider : public IMjRosOutputProvider
{
public:
	virtual FName GetProviderName() const override { return TEXT("cmd_twist"); }

	virtual void Build(FMjRosPublisherFactory& Factory, const FMjStateSnapshot& Snapshot) override
	{
		Entries.Reset();
		for (int32 i = 0; i < Snapshot.Articulations.Num(); ++i)
		{
			const FMjArticulationState& Art = Snapshot.Articulations[i];
			if (!Art.Twist.IsSet())
			{
				continue;
			}
			const FString ArtName = Art.Name.ToString();
			const FString Topic = FString::Printf(TEXT("/%s/cmd_twist"), *ArtName);
			FMjRosPub Pub = Factory.CreateTwistStamped(Topic, ArtName);
			if (Pub.IsValid())
			{
				Entries.Add({i, MoveTemp(Pub)});
			}
		}
	}

	virtual void Publish(const FMjStateSnapshot& Snapshot, int64 SimTimeNs) override
	{
		for (FEntry& Entry : Entries)
		{
			if (!Snapshot.Articulations.IsValidIndex(Entry.ArtIndex))
			{
				continue;
			}
			const FMjArticulationState& Art = Snapshot.Articulations[Entry.ArtIndex];
			double Lin[3];
			double Ang[3];
			if (UURLabRosPublishTransport::FillTwistStamped(Art, Lin, Ang))
			{
				Entry.Pub.PublishTwistStamped(Lin, Ang, SimTimeNs);
			}
		}
	}

	virtual int32 GetPublisherCountForTest() const override { return Entries.Num(); }

private:
	struct FEntry
	{
		int32 ArtIndex = 0;
		FMjRosPub Pub;
	};
	TArray<FEntry> Entries;
};

REGISTER_MJ_ROS_OUTPUT_PROVIDER("cmd_twist", FMjRosTwistProvider);
