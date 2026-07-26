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

// tf2_msgs/TFMessage on /tf, one process-wide publisher carrying every body's
// world pose (parent "world", child "<art>/<body>"). The flatten stays in the
// tested pure UURLabRosPublishTransport::FillTf.
class FMjRosTfProvider : public IMjRosOutputProvider
{
public:
	virtual FName GetProviderName() const override { return TEXT("tf"); }

	virtual void Build(FMjRosPublisherFactory& Factory, const FMjStateSnapshot& /*Snapshot*/) override
	{
		TfPub = Factory.CreateTf(/*bStatic=*/false);
	}

	virtual void Publish(const FMjStateSnapshot& Snapshot, int64 SimTimeNs) override
	{
		if (!TfPub.IsValid())
		{
			return;
		}
		TArray<FString> Parents;
		TArray<FString> Children;
		TArray<double> Translations;
		TArray<double> Rotations;
		UURLabRosPublishTransport::FillTf(Snapshot, Parents, Children, Translations, Rotations);
		TfPub.PublishTf(Parents, Children, Translations, Rotations, SimTimeNs);
	}

	virtual int32 GetPublisherCountForTest() const override { return TfPub.IsValid() ? 1 : 0; }

private:
	FMjRosPub TfPub;
};

REGISTER_MJ_ROS_OUTPUT_PROVIDER("tf", FMjRosTfProvider);
