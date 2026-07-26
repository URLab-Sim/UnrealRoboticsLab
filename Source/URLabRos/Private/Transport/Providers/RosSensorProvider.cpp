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
#include "Transport/RosSensorRouting.h"
#include "State/MjStateTypes.h"
#include "Transport/Providers/RosProviderCommon.h"

// Total sensor -> ROS routing. Every sensor on every articulation reaches ROS via
// its RouteForSemantic message: Force+Torque -> WrenchStamped, Rangefinder ->
// Range, Magnetometer -> MagneticField, Velocimeter -> TwistStamped, and anything
// with no standard typed message -> Float64MultiArray on /<art>/sensors/<name>.
// Gyro/Accel (the Imu route) are owned by the Imu provider and skipped here.
class FMjRosSensorProvider : public IMjRosOutputProvider
{
public:
	virtual FName GetProviderName() const override { return TEXT("sensors"); }

	virtual void Build(FMjRosPublisherFactory& Factory, const FMjStateSnapshot& Snapshot) override
	{
		Entries.Reset();
		for (int32 i = 0; i < Snapshot.Articulations.Num(); ++i)
		{
			const FMjArticulationState& Art = Snapshot.Articulations[i];
			const FString ArtName = Art.Name.ToString();

			// Force + Torque are paired into WrenchStamped; build those first, then
			// route the remaining sensors one message per sensor.
			TArray<FMjWrenchPair> Pairs;
			MjRosSensorRouting::GatherWrenchPairs(Art, Pairs);
			for (const FMjWrenchPair& Pair : Pairs)
			{
				const FString Topic = MjRosSensorRouting::TopicFor(ArtName,
					Pair.TopicName.ToString(), ERosSensorRoute::Wrench);
				FMjRosPub Pub = Factory.CreateWrench(Topic, ArtName);
				if (Pub.IsValid())
				{
					FEntry Entry;
					Entry.ArtIndex = i;
					Entry.Route = ERosSensorRoute::Wrench;
					Entry.PrimaryName = Pair.ForceSensor;
					Entry.SecondaryName = Pair.TorqueSensor;
					Entry.Pub = MoveTemp(Pub);
					Entries.Add(MoveTemp(Entry));
				}
			}

			for (const FMjSensorState& Sensor : Art.Sensors)
			{
				const ERosSensorRoute Route = RouteForSemantic(Sensor.Semantic);
				// Imu is owned by the Imu provider; Wrench was handled by the pairing
				// pass above.
				if (Route == ERosSensorRoute::Imu || Route == ERosSensorRoute::Wrench)
				{
					continue;
				}
				const FString SensorName = Sensor.Name.ToString();
				const FString Topic = MjRosSensorRouting::TopicFor(ArtName, SensorName, Route);
				FMjRosPub Pub = CreateForRoute(Factory, Route, Topic, ArtName);
				if (Pub.IsValid())
				{
					FEntry Entry;
					Entry.ArtIndex = i;
					Entry.Route = Route;
					Entry.PrimaryName = Sensor.Name;
					Entry.Pub = MoveTemp(Pub);
					Entries.Add(MoveTemp(Entry));
				}
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

			switch (Entry.Route)
			{
			case ERosSensorRoute::Wrench:
			{
				double Force[3] = {0.0, 0.0, 0.0};
				double Torque[3] = {0.0, 0.0, 0.0};
				if (const FMjSensorState* F = MjRosProvider::FindSensor(Art, Entry.PrimaryName))
				{
					MjRosProvider::FirstThree(F->Values, Force);
				}
				if (const FMjSensorState* T = MjRosProvider::FindSensor(Art, Entry.SecondaryName))
				{
					MjRosProvider::FirstThree(T->Values, Torque);
				}
				Entry.Pub.PublishWrench(Force, Torque, SimTimeNs);
				break;
			}
			case ERosSensorRoute::Range:
			{
				const FMjSensorState* S = MjRosProvider::FindSensor(Art, Entry.PrimaryName);
				const double Reading = (S && S->Values.Num() > 0) ? S->Values[0] : 0.0;
				Entry.Pub.PublishRange(Reading, SimTimeNs);
				break;
			}
			case ERosSensorRoute::MagneticField:
			{
				double Field[3] = {0.0, 0.0, 0.0};
				if (const FMjSensorState* S = MjRosProvider::FindSensor(Art, Entry.PrimaryName))
				{
					MjRosProvider::FirstThree(S->Values, Field);
				}
				Entry.Pub.PublishMagneticField(Field, SimTimeNs);
				break;
			}
			case ERosSensorRoute::Twist:
			{
				double Linear[3] = {0.0, 0.0, 0.0};
				const double Angular[3] = {0.0, 0.0, 0.0};
				if (const FMjSensorState* S = MjRosProvider::FindSensor(Art, Entry.PrimaryName))
				{
					MjRosProvider::FirstThree(S->Values, Linear);
				}
				Entry.Pub.PublishTwistStamped(Linear, Angular, SimTimeNs);
				break;
			}
			case ERosSensorRoute::MultiArray:
			{
				if (const FMjSensorState* S = MjRosProvider::FindSensor(Art, Entry.PrimaryName))
				{
					Entry.Pub.PublishFloat64MultiArray(S->Values.GetData(), S->Values.Num());
				}
				break;
			}
			case ERosSensorRoute::Imu:
				break;  // owned by the Imu provider
			}
		}
	}

	virtual int32 GetPublisherCountForTest() const override { return Entries.Num(); }

private:
	struct FEntry
	{
		int32 ArtIndex = 0;
		ERosSensorRoute Route = ERosSensorRoute::MultiArray;
		FName PrimaryName;    // the source sensor (Wrench: the force sensor, may be None)
		FName SecondaryName;  // Wrench only: the torque sensor (may be None)
		FMjRosPub Pub;
	};

	static FMjRosPub CreateForRoute(FMjRosPublisherFactory& Factory, ERosSensorRoute Route,
		const FString& Topic, const FString& ArtName)
	{
		switch (Route)
		{
		case ERosSensorRoute::Range:
			// No FOV / range bounds in the IR; publish the reading with neutral
			// constants (infrared, unbounded) so consumers still get the distance.
			return Factory.CreateRange(Topic, ArtName, /*RadiationType=*/1,
				/*FieldOfView=*/0.0f, /*MinRange=*/0.0f, /*MaxRange=*/TNumericLimits<float>::Max());
		case ERosSensorRoute::MagneticField:
			return Factory.CreateMagneticField(Topic, ArtName);
		case ERosSensorRoute::Twist:
			return Factory.CreateTwistStamped(Topic, ArtName);
		case ERosSensorRoute::MultiArray:
			return Factory.CreateFloat64MultiArray(Topic);
		case ERosSensorRoute::Wrench:
		case ERosSensorRoute::Imu:
		default:
			return FMjRosPub();
		}
	}

	TArray<FEntry> Entries;
};

REGISTER_MJ_ROS_OUTPUT_PROVIDER("sensors", FMjRosSensorProvider);
