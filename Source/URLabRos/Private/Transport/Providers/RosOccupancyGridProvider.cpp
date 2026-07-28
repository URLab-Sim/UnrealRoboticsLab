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

namespace
{
constexpr double GResolution = 0.05;
constexpr int32 GWidth = 400;
constexpr int32 GHeight = 400;
constexpr double GZSliceMin = -0.1;
constexpr double GZSliceMax = 2.0;
constexpr int32 GCellCount = GWidth * GHeight;

constexpr double GOriginX = -static_cast<double>(GWidth) * 0.5 * GResolution;
constexpr double GOriginY = -static_cast<double>(GHeight) * 0.5 * GResolution;

// Transform a local point by the geom's world pose.
void TransformPoint(const double* Xpos, const double* Xquat,
	double& X, double& Y, double& Z)
{
	const double Qw = Xquat[0], Qx = Xquat[1], Qy = Xquat[2], Qz = Xquat[3];
	const double Xx = Qx * Qx, Yy = Qy * Qy, Zz = Qz * Qz;
	const double Xy = Qx * Qy, Xz = Qx * Qz, Yz = Qy * Qz;
	const double Wx = Qw * Qx, Wy = Qw * Qy, Wz = Qw * Qz;

	const double R00 = 1.0 - 2.0 * (Yy + Zz);
	const double R01 = 2.0 * (Xy - Wz);
	const double R02 = 2.0 * (Xz + Wy);
	const double R10 = 2.0 * (Xy + Wz);
	const double R11 = 1.0 - 2.0 * (Xx + Zz);
	const double R12 = 2.0 * (Yz - Wx);
	const double R20 = 2.0 * (Xz - Wy);
	const double R21 = 2.0 * (Yz + Wx);
	const double R22 = 1.0 - 2.0 * (Xx + Yy);

	const double Lx = X, Ly = Y, Lz = Z;
	X = R00 * Lx + R01 * Ly + R02 * Lz + Xpos[0];
	Y = R10 * Lx + R11 * Ly + R12 * Lz + Xpos[1];
	Z = R20 * Lx + R21 * Ly + R22 * Lz + Xpos[2];
}

// Compute a conservative world-space AABB for a geom by transforming the 8
// corners of its local AABB.
void ComputeWorldAABB(const FMjWorldGeom& G, double& MinX, double& MinY,
	double& MaxX, double& MaxY)
{
	double HalfX, HalfY, HalfZ;
	switch (G.Shape)
	{
		case EMjWorldGeomShape::Box:
			HalfX = G.Size[0]; HalfY = G.Size[1]; HalfZ = G.Size[2];
			break;
		case EMjWorldGeomShape::Sphere:
			HalfX = HalfY = HalfZ = G.Size[0];
			break;
		case EMjWorldGeomShape::Cylinder:
			HalfX = HalfY = G.Size[0]; HalfZ = G.Size[1];
			break;
		case EMjWorldGeomShape::Mesh:
			if (G.Mesh.IsValid())
			{
				const FMjWorldMesh& Me = *G.Mesh;
				MinX = MinY = MaxX = MaxY = 0.0;
				bool bFirst = true;
				for (const FVector3f& V : Me.Verts)
				{
					double Px = V.X, Py = V.Y, Pz = V.Z;
					TransformPoint(G.Xpos, G.Xquat, Px, Py, Pz);
					if (bFirst) { MinX = MaxX = Px; MinY = MaxY = Py; bFirst = false; }
					else
					{
						MinX = FMath::Min(MinX, Px); MaxX = FMath::Max(MaxX, Px);
						MinY = FMath::Min(MinY, Py); MaxY = FMath::Max(MaxY, Py);
					}
				}
				return;
			}
			return;
		default:
			return;
	}

	MinX = MinY = MaxX = MaxY = 0.0;
	bool bFirst = true;
	for (int32 iz = 0; iz < 2; ++iz)
	{
		const double Z = (iz == 0) ? -HalfZ : HalfZ;
		for (int32 iy = 0; iy < 2; ++iy)
		{
			const double Y = (iy == 0) ? -HalfY : HalfY;
			for (int32 ix = 0; ix < 2; ++ix)
			{
				const double X = (ix == 0) ? -HalfX : HalfX;
				double Px = X, Py = Y, Pz = Z;
				TransformPoint(G.Xpos, G.Xquat, Px, Py, Pz);
				if (bFirst) { MinX = MaxX = Px; MinY = MaxY = Py; bFirst = false; }
				else
				{
					MinX = FMath::Min(MinX, Px); MaxX = FMath::Max(MaxX, Px);
					MinY = FMath::Min(MinY, Py); MaxY = FMath::Max(MaxY, Py);
				}
			}
		}
	}
}

// Rasterize a 2D bounding box into the grid, marking cells as occupied.
void RasterizeAABB(const int8_t* InGrid, int8_t* OutGrid,
	double MinX, double MinY, double MaxX, double MaxY)
{
	const int32 X0 = FMath::Clamp(
		FMath::FloorToInt32((MinX - GOriginX) / GResolution), 0, GWidth - 1);
	const int32 X1 = FMath::Clamp(
		FMath::FloorToInt32((MaxX - GOriginX) / GResolution), 0, GWidth - 1);
	const int32 Y0 = FMath::Clamp(
		FMath::FloorToInt32((MinY - GOriginY) / GResolution), 0, GHeight - 1);
	const int32 Y1 = FMath::Clamp(
		FMath::FloorToInt32((MaxY - GOriginY) / GResolution), 0, GHeight - 1);

	for (int32 Y = Y0; Y <= Y1; ++Y)
	{
		for (int32 X = X0; X <= X1; ++X)
		{
			OutGrid[Y * GWidth + X] = 100;
		}
	}
}
} // namespace

// nav_msgs/OccupancyGrid on /map, world frame, latched. Rasterises world-geom
// AABBs into a fixed-resolution 2D grid. Publishes once on Build and again
// whenever StructureVersion changes.
class FMjRosOccupancyGridProvider : public IMjRosOutputProvider
{
public:
	virtual FName GetProviderName() const override { return TEXT("map"); }

	virtual void Build(FMjRosPublisherFactory& /*Factory*/, const FMjStateSnapshot& Snapshot) override
	{
		Destroy();
		UrlabRclContext* Ctx = FURLabRosContext::Get().GetHandle();
		if (!Ctx)
		{
			return;
		}
		Pub = UrlabRcl_CreateOccupancyGridPub(Ctx, "/map", "world",
			GResolution, GWidth, GHeight, GOriginX, GOriginY);
		if (!Pub)
		{
			return;
		}
		LastStructureVersion = Snapshot.StructureVersion;
		Grid.SetNumUninitialized(GCellCount);
		FMemory::Memzero(Grid.GetData(), GCellCount);

		for (const FMjWorldGeom& G : Snapshot.WorldGeoms)
		{
			double MinX, MinY, MaxX, MaxY;
			ComputeWorldAABB(G, MinX, MinY, MaxX, MaxY);
			RasterizeAABB(Grid.GetData(), Grid.GetData(), MinX, MinY, MaxX, MaxY);
		}

		TArray<int8> PackageData;
		PackageData.SetNumUninitialized(GCellCount);
		FMemory::Memcpy(PackageData.GetData(), Grid.GetData(), GCellCount);
		UrlabRcl_PublishOccupancyGrid(Pub, PackageData.GetData(), 0);
	}

	virtual void Publish(const FMjStateSnapshot& Snapshot, int64 SimTimeNs) override
	{
		if (!Pub)
		{
			return;
		}
		// Latched: only republish when the structure changes because occupancy
		// is ground-truth static geometry; the grid origin and resolution are
		// fixed at create.
		if (Snapshot.StructureVersion == LastStructureVersion)
		{
			return;
		}
		LastStructureVersion = Snapshot.StructureVersion;

		FMemory::Memzero(Grid.GetData(), GCellCount);
		for (const FMjWorldGeom& G : Snapshot.WorldGeoms)
		{
			double MinX, MinY, MaxX, MaxY;
			ComputeWorldAABB(G, MinX, MinY, MaxX, MaxY);
			RasterizeAABB(Grid.GetData(), Grid.GetData(), MinX, MinY, MaxX, MaxY);
		}

		TArray<int8> PackageData;
		PackageData.SetNumUninitialized(GCellCount);
		FMemory::Memcpy(PackageData.GetData(), Grid.GetData(), GCellCount);
		UrlabRcl_PublishOccupancyGrid(Pub, PackageData.GetData(), SimTimeNs);
	}

	virtual int32 GetPublisherCountForTest() const override { return Pub ? 1 : 0; }

	virtual ~FMjRosOccupancyGridProvider() { Destroy(); }

private:
	void Destroy()
	{
		if (Pub)
		{
			UrlabRcl_DestroyOccupancyGridPub(Pub);
			Pub = nullptr;
		}
		LastStructureVersion = 0;
	}

	UrlabRclOccupancyGridPub* Pub = nullptr;
	uint32 LastStructureVersion = 0;
	TArray<int8> Grid;
};

REGISTER_MJ_ROS_OUTPUT_PROVIDER("map", FMjRosOccupancyGridProvider);

#endif // URLAB_WITH_ROS2
