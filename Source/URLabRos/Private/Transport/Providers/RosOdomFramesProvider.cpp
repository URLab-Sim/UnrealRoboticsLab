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

// REP-105 ground-truth frame chain on /tf_static: map -> odom -> world, both
// identity. The existing tf provider already publishes world -> <art>/<body> with
// the exact FK pose on /tf, so this pair completes the standard
// map -> odom -> base_link tree a Nav2 / MoveIt stack expects, WITHOUT an AMCL:
// with ground truth there is no localisation error, so map, odom and the sim root
// "world" are all coincident. The two static edges are published rather than a
// literal odom -> base_link edge precisely so the base body keeps its single
// parent ("world") in the flat tf tree - a second parent would corrupt the tree.
// odom -> base_link is still exactly recoverable (odom -> world identity composed
// with the dynamic, exact world -> base_link).
class FMjRosOdomFramesProvider : public IMjRosOutputProvider
{
public:
	virtual FName GetProviderName() const override { return TEXT("rep105_frames"); }

	virtual void Build(FMjRosPublisherFactory& Factory, const FMjStateSnapshot& /*Snapshot*/) override
	{
		StaticTfPub = Factory.CreateTf(/*bStatic=*/true);
		if (!StaticTfPub.IsValid())
		{
			return;
		}
		const TArray<FString> Parents = { TEXT("map"), TEXT("odom") };
		const TArray<FString> Children = { TEXT("odom"), TEXT("world") };
		const TArray<double> Translations = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		const TArray<double> RotationsXyzw = { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
		// Latched (transient-local) static transforms; publish once at Build.
		StaticTfPub.PublishTf(Parents, Children, Translations, RotationsXyzw, /*SimTimeNs=*/0);
	}

	virtual void Publish(const FMjStateSnapshot& /*Snapshot*/, int64 /*SimTimeNs*/) override
	{
		// Latched at Build; the identity chain never changes.
	}

	virtual int32 GetPublisherCountForTest() const override { return StaticTfPub.IsValid() ? 1 : 0; }

private:
	FMjRosPub StaticTfPub;
};

REGISTER_MJ_ROS_OUTPUT_PROVIDER("rep105_frames", FMjRosOdomFramesProvider);
