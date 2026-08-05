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

// sensor_msgs/JointState on /<art>/joint_states, one publisher per articulation.
// The IR -> parallel-array transform stays in the tested pure UURLabRosPublish
// Transport::FillJointState; this provider only owns the per-art publishers.
class FMjRosJointStateProvider : public IMjRosOutputProvider
{
public:
	virtual FName GetProviderName() const override { return TEXT("joint_state"); }

	virtual void Build(FMjRosPublisherFactory& Factory, const FMjStateSnapshot& Snapshot) override
	{
		Entries.Reset();
		Entries.Reserve(Snapshot.Articulations.Num());
		for (int32 i = 0; i < Snapshot.Articulations.Num(); ++i)
		{
			const FMjArticulationState& Art = Snapshot.Articulations[i];

			TArray<FString> Names;
			TArray<double> Positions;
			TArray<double> Velocities;
			TArray<double> Efforts;
			UURLabRosPublishTransport::FillJointState(Art, Names, Positions, Velocities, Efforts);

			const FString Topic = FString::Printf(TEXT("/%s/joint_states"), *Art.Name.ToString());
			FMjRosPub Pub = Factory.CreateJointState(Topic, Names);
			if (Pub.IsValid())
			{
				Entries.Add({i, MoveTemp(Pub)});
			}
		}
	}

	virtual void Publish(const FMjStateSnapshot& Snapshot, int64 SimTimeNs) override
	{
		static constexpr int64 PublishIntervalNs = 20'000'000; // 50 Hz
		if (LastPublishNs != 0 && (SimTimeNs - LastPublishNs) < PublishIntervalNs)
		{
			return;
		}
		LastPublishNs = SimTimeNs;

		for (FEntry& Entry : Entries)
		{
			if (!Snapshot.Articulations.IsValidIndex(Entry.ArtIndex))
			{
				continue;
			}
			const FMjArticulationState& Art = Snapshot.Articulations[Entry.ArtIndex];

			TArray<FString> Names;
			TArray<double> Positions;
			TArray<double> Velocities;
			TArray<double> Efforts;
			UURLabRosPublishTransport::FillJointState(Art, Names, Positions, Velocities, Efforts);
			Entry.Pub.PublishJointState(Positions.GetData(), Velocities.GetData(),
				Efforts.Num() > 0 ? Efforts.GetData() : nullptr, Names.Num(), SimTimeNs);
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
	int64 LastPublishNs = 0;
};

REGISTER_MJ_ROS_OUTPUT_PROVIDER("joint_state", FMjRosJointStateProvider);
