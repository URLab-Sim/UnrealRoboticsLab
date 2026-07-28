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
// Publish streams live world poses for dynamic objects only, throttled to ~10 Hz.
// Static objects (worldbody-fixed) get their initial pose in Build and are not
// republished.
class FMjRosPlanningSceneProvider : public IMjRosOutputProvider
{
public:
	virtual FName GetProviderName() const override { return TEXT("planning_scene"); }

	virtual void Build(FMjRosPublisherFactory& Factory, const FMjStateSnapshot& Snapshot) override
	{
		Destroy();
		const int32 TotalCount = Snapshot.WorldGeoms.Num();
		if (TotalCount == 0)
		{
			return;
		}
		UrlabRclContext* Ctx = FURLabRosContext::Get().GetHandle();
		if (!Ctx)
		{
			return;
		}

		// Partition dynamic objects to the front so that Publish can send only
		// their poses (the C ABI updates collision objects 0..M-1 in order).
		TArray<int32> Order;
		Order.Reserve(TotalCount);
		for (int32 i = 0; i < TotalCount; ++i)
		{
			if (!Snapshot.WorldGeoms[i].bStatic)
			{
				Order.Add(i);
			}
		}
		DynamicCount = Order.Num();
		for (int32 i = 0; i < TotalCount; ++i)
		{
			if (Snapshot.WorldGeoms[i].bStatic)
			{
				Order.Add(i);
			}
		}
		Count = TotalCount;

		// Stable UTF-8 id storage + the pointer array the C ABI takes.
		IdBytes.SetNum(Count);
		TArray<const char*> IdPtrs;
		IdPtrs.SetNum(Count);
		TArray<int32> PrimTypes;
		PrimTypes.SetNum(Count);
		TArray<double> Dims;
		Dims.SetNumZeroed(Count * 3);

		// Mesh geometry, flattened across all objects (0 counts for primitives).
		TArray<int32> MeshVertCounts;
		MeshVertCounts.SetNumZeroed(Count);
		TArray<int32> MeshTriCounts;
		MeshTriCounts.SetNumZeroed(Count);
		TArray<double> MeshVerts;
		TArray<int32> MeshTris;

		for (int32 j = 0; j < Count; ++j)
		{
			const int32 i = Order[j];
			const FMjWorldGeom& G = Snapshot.WorldGeoms[i];
			FTCHARToUTF8 Conv(*G.Name.ToString());
			TArray<ANSICHAR>& Bytes = IdBytes[j];
			Bytes.Append(reinterpret_cast<const ANSICHAR*>(Conv.Get()), Conv.Length());
			Bytes.Add('\0');
			IdPtrs[j] = Bytes.GetData();

			// shape_msgs/SolidPrimitive: BOX=1 dims[x,y,z] full; SPHERE=2 dims[r];
			// CYLINDER=3 dims[height, radius]; 4=MESH (geometry in the mesh arrays).
			// MuJoCo sizes are half-extents / radius.
			switch (G.Shape)
			{
				case EMjWorldGeomShape::Sphere:
					PrimTypes[j] = 2;
					Dims[j * 3 + 0] = G.Size[0];
					break;
				case EMjWorldGeomShape::Cylinder:
					PrimTypes[j] = 3;
					Dims[j * 3 + 0] = 2.0 * G.Size[1]; // height
					Dims[j * 3 + 1] = G.Size[0];       // radius
					break;
				case EMjWorldGeomShape::Mesh:
					if (G.Mesh.IsValid())
					{
						PrimTypes[j] = 4;
						const FMjWorldMesh& Me = *G.Mesh;
						MeshVertCounts[j] = Me.Verts.Num();
						MeshTriCounts[j] = Me.Tris.Num() / 3;
						MeshVerts.Reserve(MeshVerts.Num() + Me.Verts.Num() * 3);
						for (const FVector3f& V : Me.Verts)
						{
							MeshVerts.Add(V.X);
							MeshVerts.Add(V.Y);
							MeshVerts.Add(V.Z);
						}
						MeshTris.Append(Me.Tris);
					}
					else
					{
						PrimTypes[j] = 1; // degrade to a null box rather than crash
					}
					break;
				case EMjWorldGeomShape::Box:
				default:
					PrimTypes[j] = 1;
					Dims[j * 3 + 0] = 2.0 * G.Size[0];
					Dims[j * 3 + 1] = 2.0 * G.Size[1];
					Dims[j * 3 + 2] = 2.0 * G.Size[2];
					break;
			}
		}

		Pub = UrlabRcl_CreatePlanningScenePub(Ctx, "/planning_scene", "world",
			IdPtrs.GetData(), PrimTypes.GetData(), Dims.GetData(),
			MeshVertCounts.GetData(), MeshVerts.GetData(),
			MeshTriCounts.GetData(), MeshTris.GetData(), Count);
		if (!Pub)
		{
			Count = 0;
			DynamicCount = 0;
		}
	}

	virtual void Publish(const FMjStateSnapshot& Snapshot, int64 SimTimeNs) override
	{
		if (!Pub || DynamicCount == 0)
		{
			return;
		}
		// ~10 Hz is plenty for a planning scene and avoids re-adding geometry every
		// physics step. Dynamic objects are at the front of the collision-object
		// list; static objects at the tail keep their initial (never-changing) pose.
		static constexpr int64 PublishIntervalNs = 100'000'000;
		if (LastPublishNs != 0 && (SimTimeNs - LastPublishNs) < PublishIntervalNs)
		{
			return;
		}
		LastPublishNs = SimTimeNs;

		const int32 N = FMath::Min(DynamicCount, Snapshot.WorldGeoms.Num());
		Poses.SetNumUninitialized(N * 7, EAllowShrinking::No);
		for (int32 i = 0; i < N; ++i)
		{
			const FMjWorldGeom& G = Snapshot.WorldGeoms[i];
			double* P = &Poses[i * 7];
			P[0] = G.Xpos[0];
			P[1] = G.Xpos[1];
			P[2] = G.Xpos[2];
			P[3] = G.Xquat[1]; // x (mj wxyz -> ros xyzw)
			P[4] = G.Xquat[2]; // y
			P[5] = G.Xquat[3]; // z
			P[6] = G.Xquat[0]; // w
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
		DynamicCount = 0;
		LastPublishNs = 0;
		IdBytes.Reset();
	}

	UrlabRclPlanningScenePub* Pub = nullptr;
	int32 Count = 0;
	int32 DynamicCount = 0;
	int64 LastPublishNs = 0;
	TArray<TArray<ANSICHAR>> IdBytes;
	TArray<double> Poses;
};

REGISTER_MJ_ROS_OUTPUT_PROVIDER("planning_scene", FMjRosPlanningSceneProvider);

#endif // URLAB_WITH_ROS2
