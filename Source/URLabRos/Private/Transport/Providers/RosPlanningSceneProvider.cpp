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

#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2

#include "Ros/UrlabRclCore.h"
#include "Transport/RosContext.h"

// moveit_msgs/PlanningScene (is_diff) on /planning_scene: the sim's non-robot
// collision geometry, so MoveIt plans around obstacles and can manipulate them.
// The object set is fixed per StructureVersion (Build creates it with shapes);
// Publish streams live world poses, throttled to ~10 Hz. This provider talks to
// the rcl core directly (moveit_msgs is a MoveIt-only dependency, kept out of the
// generic publisher factory).
class FMjRosPlanningSceneProvider : public IMjRosOutputProvider
{
public:
	virtual FName GetProviderName() const override { return TEXT("planning_scene"); }

	virtual void Build(FMjRosPublisherFactory& Factory, const FMjStateSnapshot& Snapshot) override
	{
		Destroy();
		Count = Snapshot.WorldGeoms.Num();
		if (Count == 0)
		{
			return;
		}
		UrlabRclContext* Ctx = FURLabRosContext::Get().GetHandle();
		if (!Ctx)
		{
			Count = 0;
			return;
		}

		// Stable UTF-8 id storage + the pointer array the C ABI takes.
		IdBytes.SetNum(Count);
		TArray<const char*> IdPtrs;
		IdPtrs.SetNum(Count);
		TArray<int32> PrimTypes;
		PrimTypes.SetNum(Count);
		TArray<double> Dims;
		Dims.SetNumZeroed(Count * 3);

		for (int32 i = 0; i < Count; ++i)
		{
			const FMjWorldGeom& G = Snapshot.WorldGeoms[i];
			FTCHARToUTF8 Conv(*G.Name.ToString());
			TArray<ANSICHAR>& Bytes = IdBytes[i];
			Bytes.Append(reinterpret_cast<const ANSICHAR*>(Conv.Get()), Conv.Length());
			Bytes.Add('\0');
			IdPtrs[i] = Bytes.GetData();

			// shape_msgs/SolidPrimitive: BOX=1 dims[x,y,z] full; SPHERE=2 dims[r];
			// CYLINDER=3 dims[height, radius]. MuJoCo sizes are half-extents / radius.
			switch (G.Shape)
			{
				case EMjWorldGeomShape::Sphere:
					PrimTypes[i] = 2;
					Dims[i * 3 + 0] = G.Size[0];
					break;
				case EMjWorldGeomShape::Cylinder:
					PrimTypes[i] = 3;
					Dims[i * 3 + 0] = 2.0 * G.Size[1];  // height
					Dims[i * 3 + 1] = G.Size[0];        // radius
					break;
				case EMjWorldGeomShape::Box:
				default:
					PrimTypes[i] = 1;
					Dims[i * 3 + 0] = 2.0 * G.Size[0];
					Dims[i * 3 + 1] = 2.0 * G.Size[1];
					Dims[i * 3 + 2] = 2.0 * G.Size[2];
					break;
			}
		}

		Pub = UrlabRcl_CreatePlanningScenePub(Ctx, "/planning_scene", "world",
			IdPtrs.GetData(), PrimTypes.GetData(), Dims.GetData(), Count);
		if (!Pub)
		{
			Count = 0;
		}
	}

	virtual void Publish(const FMjStateSnapshot& Snapshot, int64 SimTimeNs) override
	{
		if (!Pub || Count == 0)
		{
			return;
		}
		// ~10 Hz is plenty for a planning scene and avoids re-adding geometry every
		// physics step.
		if (LastPublishNs != 0 && (SimTimeNs - LastPublishNs) < 100'000'000)
		{
			return;
		}
		LastPublishNs = SimTimeNs;

		const int32 N = FMath::Min(Count, Snapshot.WorldGeoms.Num());
		Poses.SetNumUninitialized(N * 7, EAllowShrinking::No);
		for (int32 i = 0; i < N; ++i)
		{
			const FMjWorldGeom& G = Snapshot.WorldGeoms[i];
			double* P = &Poses[i * 7];
			P[0] = G.Xpos[0];
			P[1] = G.Xpos[1];
			P[2] = G.Xpos[2];
			P[3] = G.Xquat[1];  // x (mj wxyz -> ros xyzw)
			P[4] = G.Xquat[2];  // y
			P[5] = G.Xquat[3];  // z
			P[6] = G.Xquat[0];  // w
		}
		UrlabRcl_PublishPlanningScene(Pub, Poses.GetData(), N, SimTimeNs);
	}

	virtual int32 GetPublisherCountForTest() const override { return Pub ? 1 : 0; }

	virtual ~FMjRosPlanningSceneProvider() { Destroy(); }

private:
	void Destroy()
	{
		if (Pub)
		{
			UrlabRcl_DestroyPlanningScenePub(Pub);
			Pub = nullptr;
		}
		Count = 0;
		LastPublishNs = 0;
		IdBytes.Reset();
	}

	UrlabRclPlanningScenePub* Pub = nullptr;
	int32 Count = 0;
	int64 LastPublishNs = 0;
	TArray<TArray<ANSICHAR>> IdBytes;
	TArray<double> Poses;
};

REGISTER_MJ_ROS_OUTPUT_PROVIDER("planning_scene", FMjRosPlanningSceneProvider);

#endif  // URLAB_WITH_ROS2
