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

#include "Transport/RosPublishTransport.h"
#include "State/MjStateTypes.h"
#include "State/MjCanonicalName.h"

#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2
#include "Transport/RosContext.h"
#include "Transport/RosOutputProvider.h"
#include "URLabRosLog.h"
#include "MuJoCo/Core/AMjManager.h"
#endif

// ConsumeState is the IMjStateConsumer entry point the manager fan-out calls; it
// forwards to PublishState (which has an absent-ROS no-op stub), so it links in
// every configuration.
void UURLabRosPublishTransport::ConsumeState(const FMjStateSnapshot& Snapshot)
{
	PublishState(Snapshot);
}

// The Fill* functions are pure IR -> arrays transforms with no rcl dependency, so
// they compile in every configuration (and are exercised by the fill-correctness
// tests whether or not ROS is linked).

void UURLabRosPublishTransport::FillJointState(const FMjArticulationState& Art,
	TArray<FString>& OutNames, TArray<double>& OutPositions,
	TArray<double>& OutVelocities, TArray<double>& OutEfforts)
{
	OutNames.Reset();
	OutPositions.Reset();
	OutVelocities.Reset();
	OutEfforts.Reset();
	OutNames.Reserve(Art.Joints.Num());
	OutPositions.Reserve(Art.Joints.Num());
	OutVelocities.Reserve(Art.Joints.Num());
	OutEfforts.Reserve(Art.Joints.Num());

	// Map joint -> actuator force by the joint each actuator drives (its target
	// joint), which is the name JointState reports the joint under. An actuator's
	// own name need not match the joint it drives (e.g. 'actuator1' drives 'joint1'),
	// so keying by the target joint is what makes effort line up. Joints with no
	// joint-transmission actuator report zero.
	TMap<FName, double> ForceByJoint;
	ForceByJoint.Reserve(Art.Actuators.Num());
	for (const FMjActuatorState& Actuator : Art.Actuators)
	{
		if (!Actuator.TargetJoint.IsNone())
		{
			ForceByJoint.Add(Actuator.TargetJoint, Actuator.Force);
		}
	}

	bool bAnyEffort = false;
	for (const FMjJointState& Joint : Art.Joints)
	{
		// sensor_msgs/JointState is parallel scalar arrays. Only hinge / slide joints
		// are scalar (1 qpos / 1 qvel); free (7/6) and ball (4/3) joints are not URDF
		// joints and reach ROS through /tf, so they are not JointState entries.
		if (Joint.Type != EMjJointType::hinge && Joint.Type != EMjJointType::slide)
		{
			continue;
		}

		OutNames.Add(Joint.Name.ToString());
		// Emit qpos - qpos0 so the ROS zero pose matches the exported URDF (whose
		// joint limits are shifted by qpos0). RefPos is filled only for the 1-DOF
		// joints the URDF exposes; an empty slice means no shift.
		const double Pos = Joint.QPos.Num() > 0 ? Joint.QPos[0] : 0.0;
		const double Ref = Joint.RefPos.Num() > 0 ? Joint.RefPos[0] : 0.0;
		OutPositions.Add(Pos - Ref);
		OutVelocities.Add(Joint.QVel.Num() > 0 ? Joint.QVel[0] : 0.0);

		if (const double* Force = ForceByJoint.Find(Joint.Name))
		{
			OutEfforts.Add(*Force);
			bAnyEffort = true;
		}
		else
		{
			OutEfforts.Add(0.0);
		}
	}

	// Distinguish "no effort data" (no actuator drives any joint) from a genuine
	// all-zero effort by leaving the array empty in the former case.
	if (!bAnyEffort)
	{
		OutEfforts.Reset();
	}
}

bool UURLabRosPublishTransport::FillImu(const FMjArticulationState& Art,
	double OutAngularVel[3], bool& bOutHasAngularVel,
	double OutLinearAccel[3], bool& bOutHasLinearAccel)
{
	bOutHasAngularVel = false;
	bOutHasLinearAccel = false;
	for (int32 i = 0; i < 3; ++i)
	{
		OutAngularVel[i] = 0.0;
		OutLinearAccel[i] = 0.0;
	}

	// Pair the first gyro with the first accel found on the art. A gyro without an
	// accel still yields an Imu with angular velocity only, and vice versa.
	for (const FMjSensorState& Sensor : Art.Sensors)
	{
		if (!bOutHasAngularVel && Sensor.Semantic == EMjSensorSemantic::Gyro
			&& Sensor.Values.Num() >= 3)
		{
			OutAngularVel[0] = Sensor.Values[0];
			OutAngularVel[1] = Sensor.Values[1];
			OutAngularVel[2] = Sensor.Values[2];
			bOutHasAngularVel = true;
		}
		else if (!bOutHasLinearAccel && Sensor.Semantic == EMjSensorSemantic::Accel
				 && Sensor.Values.Num() >= 3)
		{
			OutLinearAccel[0] = Sensor.Values[0];
			OutLinearAccel[1] = Sensor.Values[1];
			OutLinearAccel[2] = Sensor.Values[2];
			bOutHasLinearAccel = true;
		}
	}

	return bOutHasAngularVel || bOutHasLinearAccel;
}

bool UURLabRosPublishTransport::FillTwistStamped(const FMjArticulationState& Art,
	double OutLinear[3], double OutAngular[3])
{
	if (!Art.Twist.IsSet())
	{
		return false;
	}
	const FMjTwistState& Twist = Art.Twist.GetValue();
	for (int32 i = 0; i < 3; ++i)
	{
		OutLinear[i] = Twist.Linear[i];
		OutAngular[i] = Twist.Angular[i];
	}
	return true;
}

void UURLabRosPublishTransport::FillTf(const FMjStateSnapshot& Snapshot,
	TArray<FString>& OutParents, TArray<FString>& OutChildren,
	TArray<double>& OutTranslations, TArray<double>& OutRotationsXyzw)
{
	OutParents.Reset();
	OutChildren.Reset();
	OutTranslations.Reset();
	OutRotationsXyzw.Reset();

	for (const FMjArticulationState& Art : Snapshot.Articulations)
	{
		for (const FMjBodyState& Body : Art.Bodies)
		{
			OutParents.Add(TEXT("world"));
			OutChildren.Add(FMjCanonicalName::Full(Art.Name, Body.Name));
			OutTranslations.Add(Body.Xpos[0]);
			OutTranslations.Add(Body.Xpos[1]);
			OutTranslations.Add(Body.Xpos[2]);
			// MuJoCo stores quaternions wxyz; the core (and ROS) expect xyzw.
			OutRotationsXyzw.Add(Body.Xquat[1]);
			OutRotationsXyzw.Add(Body.Xquat[2]);
			OutRotationsXyzw.Add(Body.Xquat[3]);
			OutRotationsXyzw.Add(Body.Xquat[0]);
		}
	}
}

int64 UURLabRosPublishTransport::FillClock(const FMjClock& Clock)
{
	return static_cast<int64>(Clock.SimSec) * 1000000000LL
		 + static_cast<int64>(Clock.SimNsec);
}

// Reports the joint_state provider's per-articulation publisher count (one per
// art), preserving the historical "art publisher count" test seam. Compiles in
// every configuration; Providers is empty when ROS is not linked.
int32 UURLabRosPublishTransport::GetArtPublisherCountForTest() const
{
	for (const TUniquePtr<IMjRosOutputProvider>& Provider : Providers)
	{
		if (Provider && Provider->GetProviderName() == TEXT("joint_state"))
		{
			return Provider->GetPublisherCountForTest();
		}
	}
	return 0;
}

#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2

bool UURLabRosPublishTransport::TransportInit()
{
	if (!FURLabRosContext::Get().Initialize())
	{
		return false;
	}
	// Register with the owning manager as a typed state consumer so the post-step
	// fan-out drives ConsumeState. Null-safe: a transport created without a manager
	// outer (isolated publish test) simply is not registered and is driven directly.
	if (AAMjManager* Manager = GetTypedOuter<AAMjManager>())
	{
		Manager->RegisterStateConsumer(this, this);
	}
	return true;
}

void UURLabRosPublishTransport::TransportShutdown()
{
	if (AAMjManager* Manager = GetTypedOuter<AAMjManager>())
	{
		Manager->UnregisterStateConsumer(this);
	}
	// Destroying each provider releases the publishers it owns.
	Providers.Reset();
	bProvidersBuilt = false;
}

void UURLabRosPublishTransport::RebuildProviders(const FMjStateSnapshot& Snapshot)
{
	// Destroy the previous provider set (releasing its publishers) before building
	// a fresh one for the current structure.
	Providers.Reset();
	bProvidersBuilt = false;

	UrlabRclContext* Ctx = FURLabRosContext::Get().GetHandle();
	if (Ctx == nullptr)
	{
		return;
	}

	FMjRosPublisherFactory Factory(Ctx, GetTypedOuter<AAMjManager>());
	FMjRosOutputRegistry::Get().InstantiateAll(Providers);
	for (const TUniquePtr<IMjRosOutputProvider>& Provider : Providers)
	{
		if (Provider)
		{
			Provider->Build(Factory, Snapshot);
		}
	}

	CachedStructureVersion = Snapshot.StructureVersion;
	bProvidersBuilt = true;
}

void UURLabRosPublishTransport::PublishState(const FMjStateSnapshot& Snapshot)
{
	if (!FURLabRosContext::Get().IsAvailable())
	{
		return;
	}
	if (!bProvidersBuilt || Snapshot.StructureVersion != CachedStructureVersion)
	{
		RebuildProviders(Snapshot);
	}
	++PublishStateCount;

	const int64 SimTimeNs = FillClock(Snapshot.Clock);
	for (const TUniquePtr<IMjRosOutputProvider>& Provider : Providers)
	{
		if (Provider)
		{
			Provider->Publish(Snapshot, SimTimeNs);
		}
	}
}

#else // URLAB_WITH_ROS2

// Absent-ROS stubs so the class links; every real caller is fenced off too.
bool UURLabRosPublishTransport::TransportInit()
{
	return false;
}
void UURLabRosPublishTransport::TransportShutdown() {}
void UURLabRosPublishTransport::PublishState(const FMjStateSnapshot& /*Snapshot*/) {}
void UURLabRosPublishTransport::RebuildProviders(const FMjStateSnapshot& /*Snapshot*/) {}

#endif // URLAB_WITH_ROS2
