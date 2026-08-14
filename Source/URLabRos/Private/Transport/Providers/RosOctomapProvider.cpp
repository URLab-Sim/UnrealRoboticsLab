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
constexpr int32 GMaxDepth = 16;

constexpr float GOccupiedLogOdds = 2.0f;
constexpr float GFreeLogOdds = -2.0f;

// Transform a local point by the geom's world pose (wxyz quaternion).
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

// Compute a conservative world-space AABB by transforming the 8 corners of
// the local AABB. Returns the result as an FBox.
FBox ComputeWorldAABB(const FMjWorldGeom& G)
{
	double HalfX, HalfY, HalfZ;
	switch (G.Shape)
	{
		case EMjWorldGeomShape::Box:
			HalfX = G.Size[0];
			HalfY = G.Size[1];
			HalfZ = G.Size[2];
			break;
		case EMjWorldGeomShape::Sphere:
			HalfX = HalfY = HalfZ = G.Size[0];
			break;
		case EMjWorldGeomShape::Cylinder:
			HalfX = HalfY = G.Size[0];
			HalfZ = G.Size[1];
			break;
		case EMjWorldGeomShape::Mesh:
			if (G.Mesh.IsValid())
			{
				const FMjWorldMesh& Me = *G.Mesh;
				FBox Out;
				for (const FVector3f& V : Me.Verts)
				{
					double Px = V.X, Py = V.Y, Pz = V.Z;
					TransformPoint(G.Xpos, G.Xquat, Px, Py, Pz);
					Out += FVector(Px, Py, Pz);
				}
				return Out;
			}
			return FBox();
		default:
			return FBox();
	}

	FBox Out;
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
				Out += FVector(Px, Py, Pz);
			}
		}
	}
	return Out;
}

// Compute the global AABB enclosing all world geoms, expanded to a
// power-of-2 cube so it cleanly subdivides to the leaf resolution.
FBox ComputeGlobalCube(const TArray<FMjWorldGeom>& Geoms, double& OutHalfSize)
{
	if (Geoms.Num() == 0)
	{
		OutHalfSize = 1.0;
		return FBox(FVector(-1.0), FVector(1.0));
	}
	FBox Global;
	for (const FMjWorldGeom& G : Geoms)
	{
		FBox B = ComputeWorldAABB(G);
		if (B.IsValid)
		{
			Global += B.Min;
			Global += B.Max;
		}
	}
	if (!Global.IsValid)
	{
		OutHalfSize = 1.0;
		return FBox(FVector(-1.0), FVector(1.0));
	}

	const FVector Center = Global.GetCenter();
	const FVector Extent = Global.GetExtent();
	const double MaxExt = FMath::Max3(Extent.X, Extent.Y, Extent.Z);

	// Double until the cube side is at least MaxExt * 2 (to fully contain the
	// AABB), but cap at a size that keeps the depth reasonable.
	double HalfSize = 1.0;
	while (HalfSize < MaxExt && HalfSize < 1.0e6)
	{
		HalfSize *= 2.0;
	}
	OutHalfSize = HalfSize;
	return FBox(Center - FVector(HalfSize), Center + FVector(HalfSize));
}

// Test whether the node box (at arbitrary depth) intersects any geom AABB.
bool NodeIntersectsGeoms(const FBox& NodeBox, const TArray<FBox>& GeomBoxes)
{
	for (const FBox& Gb : GeomBoxes)
	{
		if (NodeBox.Intersect(Gb))
		{
			return true;
		}
	}
	return false;
}

// Test whether the node box is fully contained within some geom AABB.
bool NodeInsideGeom(const FBox& NodeBox, const TArray<FBox>& GeomBoxes)
{
	for (const FBox& Gb : GeomBoxes)
	{
		if (NodeBox.Min.X >= Gb.Min.X && NodeBox.Max.X <= Gb.Max.X && NodeBox.Min.Y >= Gb.Min.Y && NodeBox.Max.Y <= Gb.Max.Y && NodeBox.Min.Z >= Gb.Min.Z && NodeBox.Max.Z <= Gb.Max.Z)
		{
			return true;
		}
	}
	return false;
}

void ClassifyNode(const FBox& NodeBox, const TArray<FBox>& GeomBoxes,
	bool& bOutIntersects, bool& bOutFullyInside)
{
	bOutIntersects = NodeIntersectsGeoms(NodeBox, GeomBoxes);
	bOutFullyInside = bOutIntersects && NodeInsideGeom(NodeBox, GeomBoxes);
}

// Count nodes for the size header in a pre-pass.
int32 CountNodes(const FBox& Box, int32 Depth, int32 MaxDepth,
	const TArray<FBox>& GeomBoxes)
{
	bool bIntersects, bFullyInside;
	ClassifyNode(Box, GeomBoxes, bIntersects, bFullyInside);
	if (!bIntersects || bFullyInside || Depth >= MaxDepth)
	{
		return 1; // leaf
	}
	int32 Count = 1; // this inner node
	const FVector C = Box.GetCenter();
	const FVector HalfExt = Box.GetExtent() * 0.5;
	for (int32 i = 0; i < 8; ++i)
	{
		FVector Corner = C;
		Corner.X += (i & 1) ? HalfExt.X : -HalfExt.X;
		Corner.Y += (i & 2) ? HalfExt.Y : -HalfExt.Y;
		Corner.Z += (i & 4) ? HalfExt.Z : -HalfExt.Z;
		FBox Child(Corner - HalfExt, Corner + HalfExt);
		if (NodeIntersectsGeoms(Child, GeomBoxes))
		{
			Count += CountNodes(Child, Depth + 1, MaxDepth, GeomBoxes);
		}
	}
	return Count;
}

// Serialize the ocTree in DFS order into OutBinary. Inner nodes get a
// placeholder children mask that is back-patched after recursion.
void SerializeNodes(const FBox& Box, int32 Depth, int32 MaxDepth,
	const TArray<FBox>& GeomBoxes, TArray<uint8>& OutBinary)
{
	bool bIntersects, bFullyInside;
	ClassifyNode(Box, GeomBoxes, bIntersects, bFullyInside);

	if (!bIntersects)
	{
		float V = GFreeLogOdds;
		OutBinary.Append(reinterpret_cast<const uint8*>(&V), sizeof(float));
		OutBinary.Add(0); // children mask = 0
		return;
	}
	if (bFullyInside || Depth >= MaxDepth)
	{
		float V = GOccupiedLogOdds;
		OutBinary.Append(reinterpret_cast<const uint8*>(&V), sizeof(float));
		OutBinary.Add(0); // children mask = 0
		return;
	}

	// Inner node: logOdds = 0 (unknown at this resolution level), children follow.
	const float InnerLogOdds = 0.0f;
	OutBinary.Append(reinterpret_cast<const uint8*>(&InnerLogOdds), sizeof(float));
	const int32 MaskPos = OutBinary.Num();
	OutBinary.Add(0); // placeholder
	uint8 Mask = 0;

	const FVector C = Box.GetCenter();
	const FVector HalfExt = Box.GetExtent() * 0.5;
	for (int32 i = 0; i < 8; ++i)
	{
		FVector Corner = C;
		Corner.X += (i & 1) ? HalfExt.X : -HalfExt.X;
		Corner.Y += (i & 2) ? HalfExt.Y : -HalfExt.Y;
		Corner.Z += (i & 4) ? HalfExt.Z : -HalfExt.Z;
		FBox Child(Corner - HalfExt, Corner + HalfExt);
		if (NodeIntersectsGeoms(Child, GeomBoxes))
		{
			Mask |= static_cast<uint8>(1 << i);
			SerializeNodes(Child, Depth + 1, MaxDepth, GeomBoxes, OutBinary);
		}
	}
	OutBinary[MaskPos] = Mask;
}

void AppendBinaryLE(TArray<uint8>& Out, uint16_t Val)
{
	Out.Add(static_cast<uint8>(Val & 0xFF));
	Out.Add(static_cast<uint8>((Val >> 8) & 0xFF));
}

// Builds the full serialised Octomap message payload: text header line
// followed by the binary tree data.
TArray<uint8> BuildOctomapPayload(const TArray<FMjWorldGeom>& Geoms,
	double Resolution, int32 MaxDepth)
{
	// Collect per-geom world-space AABBs.
	TArray<FBox> GeomBoxes;
	GeomBoxes.Reserve(Geoms.Num());
	for (const FMjWorldGeom& G : Geoms)
	{
		FBox B = ComputeWorldAABB(G);
		if (B.IsValid)
		{
			GeomBoxes.Add(B);
		}
	}
	if (GeomBoxes.Num() == 0)
	{
		return TArray<uint8>();
	}

	double HalfSize;
	FBox RootBox = ComputeGlobalCube(Geoms, HalfSize);

	const int32 TotalNodes = CountNodes(RootBox, 0, MaxDepth, GeomBoxes);

	// Build text header.
	const FString HeaderLine = FString::Printf(
		TEXT("# Octomap OcTree\nid OcTree\nsize %d\nres %.4f\ndata\n"),
		TotalNodes, Resolution);
	FTCHARToUTF8 HeaderUtf8(*HeaderLine);

	TArray<uint8> Payload;
	Payload.Reserve(HeaderUtf8.Length() + 2 + TotalNodes * 7);
	Payload.Append(reinterpret_cast<const uint8*>(HeaderUtf8.Get()),
		HeaderUtf8.Length());

	// Binary data: uint16_t node count, then node records.
	AppendBinaryLE(Payload, static_cast<uint16_t>(TotalNodes > 65535 ? 65535 : TotalNodes));
	SerializeNodes(RootBox, 0, MaxDepth, GeomBoxes, Payload);

	return Payload;
}
} // namespace

// octomap_msgs/Octomap on /octomap_binary, world frame. Builds a minimal
// OcTree from world-geom AABBs using recursive subdivision to the configured
// resolution. Publishes at ~5 Hz.
class FMjRosOctomapProvider : public IMjRosOutputProvider
{
public:
	virtual FName GetProviderName() const override { return TEXT("octomap"); }

	virtual void Build(FMjRosPublisherFactory& /*Factory*/, const FMjStateSnapshot& Snapshot) override
	{
		Destroy();
		UrlabRclContext* Ctx = FURLabRosContext::Get().GetHandle();
		if (!Ctx)
		{
			return;
		}
		Pub = UrlabRcl_CreateOctomapPub(Ctx, "/octomap_binary", "world", GResolution);
		LastStructureVersion = Snapshot.StructureVersion;
	}

	virtual void Publish(const FMjStateSnapshot& Snapshot, int64 SimTimeNs) override
	{
		if (!Pub)
		{
			return;
		}
		static constexpr int64 PublishIntervalNs = 200'000'000; // 5 Hz
		if (LastPublishNs != 0 && (SimTimeNs - LastPublishNs) < PublishIntervalNs)
		{
			// Within rate limit, but republish if structure changed.
			if (Snapshot.StructureVersion == LastStructureVersion)
			{
				return;
			}
		}
		LastPublishNs = SimTimeNs;
		LastStructureVersion = Snapshot.StructureVersion;

		const TArray<uint8> Payload = BuildOctomapPayload(
			Snapshot.WorldGeoms, GResolution, GMaxDepth);
		UrlabRcl_PublishOctomap(Pub, Payload.GetData(),
			static_cast<int32>(Payload.Num()), SimTimeNs);
	}

	virtual int32 GetPublisherCountForTest() const override { return Pub ? 1 : 0; }

	virtual ~FMjRosOctomapProvider() { Destroy(); }

private:
	void Destroy()
	{
		if (Pub)
		{
			UrlabRcl_DestroyOctomapPub(Pub);
			Pub = nullptr;
		}
		LastPublishNs = 0;
		LastStructureVersion = 0;
	}

	UrlabRclOctomapPub* Pub = nullptr;
	int64 LastPublishNs = 0;
	uint32 LastStructureVersion = 0;
};

REGISTER_MJ_ROS_OUTPUT_PROVIDER("octomap", FMjRosOctomapProvider);

#endif // URLAB_WITH_ROS2
