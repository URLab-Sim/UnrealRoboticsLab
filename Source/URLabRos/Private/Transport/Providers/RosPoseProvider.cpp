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

// geometry_msgs/PoseWithCovarianceStamped on /<art>/pose, one publisher per
// free-base articulation. This is the ground-truth counterpart of amcl_pose: the
// robot's exact base pose in the map frame (frame_id = "map"), with a ground-truth
// covariance diagonal fixed at create. Fixed-base arts do not localise, so they
// are skipped (matching the odometry provider).
class FMjRosPoseProvider : public IMjRosOutputProvider
{
public:
	virtual FName GetProviderName() const override { return TEXT("pose"); }

	virtual void Build(FMjRosPublisherFactory& Factory, const FMjStateSnapshot& Snapshot) override
	{
		Entries.Reset();
		for (int32 i = 0; i < Snapshot.Articulations.Num(); ++i)
		{
			const FMjArticulationState& Art = Snapshot.Articulations[i];
			MjRosStateEstimation::FMjFreeBaseState State;
			if (!MjRosStateEstimation::ComputeFreeBaseState(Art, State))
			{
				continue;
			}
			const FString Topic = FString::Printf(TEXT("/%s/pose"), *Art.Name.ToString());
			FMjRosPub Pub = Factory.CreatePoseWithCovariance(Topic, TEXT("map"));
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
			MjRosStateEstimation::FMjFreeBaseState State;
			if (MjRosStateEstimation::ComputeFreeBaseState(
					Snapshot.Articulations[Entry.ArtIndex], State))
			{
				Entry.Pub.PublishPoseWithCovariance(State.Position, State.OrientationXyzw, SimTimeNs);
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

REGISTER_MJ_ROS_OUTPUT_PROVIDER("pose", FMjRosPoseProvider);
