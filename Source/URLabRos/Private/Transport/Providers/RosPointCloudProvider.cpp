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
constexpr double GResolution = 0.02; // metres between sample points
constexpr double GTwoPi = 6.283185307179586;

// Apply quaternion (wxyz) rotation then translation to a point in-place.
void ApplyWorldTransform(const double* Xpos, const double* Xquat,
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

void EmitPoint(TArray<float>& Out, double X, double Y, double Z)
{
	Out.Add(static_cast<float>(X));
	Out.Add(static_cast<float>(Y));
	Out.Add(static_cast<float>(Z));
}

void SampleBox(const FMjWorldGeom& G, TArray<float>& Out)
{
	const double Sx = G.Size[0], Sy = G.Size[1], Sz = G.Size[2];
	const int32 Nx = FMath::Max(2, FMath::CeilToInt32(2.0 * Sx / GResolution) + 1);
	const int32 Ny = FMath::Max(2, FMath::CeilToInt32(2.0 * Sy / GResolution) + 1);
	const int32 Nz = FMath::Max(2, FMath::CeilToInt32(2.0 * Sz / GResolution) + 1);

	// +X / -X faces (perpendicular to x, grid in y,z)
	for (int32 iy = 0; iy < Ny; ++iy)
	{
		const double Y = -Sy + (2.0 * Sy * iy) / (Ny - 1);
		for (int32 iz = 0; iz < Nz; ++iz)
		{
			const double Z = -Sz + (2.0 * Sz * iz) / (Nz - 1);
			double Px = Sx, Py = Y, Pz = Z;
			ApplyWorldTransform(G.Xpos, G.Xquat, Px, Py, Pz);
			EmitPoint(Out, Px, Py, Pz);
			Px = -Sx;
			Py = Y;
			Pz = Z;
			ApplyWorldTransform(G.Xpos, G.Xquat, Px, Py, Pz);
			EmitPoint(Out, Px, Py, Pz);
		}
	}
	// +Y / -Y faces (perpendicular to y, grid in x,z)
	for (int32 ix = 0; ix < Nx; ++ix)
	{
		const double X = -Sx + (2.0 * Sx * ix) / (Nx - 1);
		for (int32 iz = 0; iz < Nz; ++iz)
		{
			const double Z = -Sz + (2.0 * Sz * iz) / (Nz - 1);
			double Px = X, Py = Sy, Pz = Z;
			ApplyWorldTransform(G.Xpos, G.Xquat, Px, Py, Pz);
			EmitPoint(Out, Px, Py, Pz);
			Px = X;
			Py = -Sy;
			Pz = Z;
			ApplyWorldTransform(G.Xpos, G.Xquat, Px, Py, Pz);
			EmitPoint(Out, Px, Py, Pz);
		}
	}
	// +Z / -Z faces (perpendicular to z, grid in x,y)
	for (int32 ix = 0; ix < Nx; ++ix)
	{
		const double X = -Sx + (2.0 * Sx * ix) / (Nx - 1);
		for (int32 iy = 0; iy < Ny; ++iy)
		{
			const double Y = -Sy + (2.0 * Sy * iy) / (Ny - 1);
			double Px = X, Py = Y, Pz = Sz;
			ApplyWorldTransform(G.Xpos, G.Xquat, Px, Py, Pz);
			EmitPoint(Out, Px, Py, Pz);
			Px = X;
			Py = Y;
			Pz = -Sz;
			ApplyWorldTransform(G.Xpos, G.Xquat, Px, Py, Pz);
			EmitPoint(Out, Px, Py, Pz);
		}
	}
}

void SampleSphere(const FMjWorldGeom& G, TArray<float>& Out)
{
	const double R = G.Size[0];
	const int32 N = FMath::Max(20, FMath::CeilToInt32(4.0 * UE_PI * R * R / (GResolution * GResolution)));

	// Fibonacci lattice on the unit sphere. The irrational angle step
	// (golden-angle conjugate) distributes points evenly.
	constexpr double GGoldenConjugate = 0.6180339887498948482;
	for (int32 i = 0; i < N; ++i)
	{
		const double Y = 1.0 - (2.0 * static_cast<double>(i) / static_cast<double>(N - 1));
		const double RadiusAtY = FMath::Sqrt(1.0 - Y * Y);
		const double Theta = GTwoPi * static_cast<double>(i) * GGoldenConjugate;
		const double Lx = RadiusAtY * FMath::Cos(Theta) * R;
		const double Ly = Y * R;
		const double Lz = RadiusAtY * FMath::Sin(Theta) * R;
		double Px = Lx, Py = Ly, Pz = Lz;
		ApplyWorldTransform(G.Xpos, G.Xquat, Px, Py, Pz);
		EmitPoint(Out, Px, Py, Pz);
	}
}

void SampleCylinder(const FMjWorldGeom& G, TArray<float>& Out)
{
	const double R = G.Size[0];
	const double HalfH = G.Size[1];
	const int32 Nh = FMath::Max(2, FMath::CeilToInt32(2.0 * HalfH / GResolution) + 1);
	const double AngRes = GResolution / R;
	const int32 Na = FMath::Max(8, FMath::CeilToInt32(GTwoPi / AngRes));

	// Curved surface: grid in height x angle.
	for (int32 ih = 0; ih < Nh; ++ih)
	{
		const double Z = -HalfH + (2.0 * HalfH * ih) / (Nh - 1);
		for (int32 ia = 0; ia < Na; ++ia)
		{
			const double A = (GTwoPi * ia) / Na;
			double Px = R * FMath::Cos(A);
			double Py = R * FMath::Sin(A);
			double Pz = Z;
			ApplyWorldTransform(G.Xpos, G.Xquat, Px, Py, Pz);
			EmitPoint(Out, Px, Py, Pz);
		}
	}
	// Caps: concentric rings at z = +/-HalfH.
	const int32 Nr = FMath::Max(2, FMath::CeilToInt32(R / GResolution) + 1);
	for (int32 cap = 0; cap < 2; ++cap)
	{
		const double Z = (cap == 0) ? HalfH : -HalfH;
		for (int32 ir = 0; ir < Nr; ++ir)
		{
			const double Cr = (R * ir) / (Nr - 1);
			const int32 RingN = FMath::Max(1, FMath::CeilToInt32(GTwoPi * Cr / GResolution));
			for (int32 ia = 0; ia < RingN; ++ia)
			{
				const double A = (GTwoPi * ia) / RingN;
				double Px = Cr * FMath::Cos(A);
				double Py = Cr * FMath::Sin(A);
				double Pz = Z;
				ApplyWorldTransform(G.Xpos, G.Xquat, Px, Py, Pz);
				EmitPoint(Out, Px, Py, Pz);
			}
		}
	}
}

void SampleMesh(const FMjWorldGeom& G, TArray<float>& Out)
{
	if (!G.Mesh.IsValid())
	{
		return;
	}
	const FMjWorldMesh& Me = *G.Mesh;
	const int32 VertCount = Me.Verts.Num();
	const int32 TriCount = Me.Tris.Num() / 3;
	if (TriCount == 0)
	{
		return;
	}

	// Emit all vertices.
	for (int32 vi = 0; vi < VertCount; ++vi)
	{
		const FVector3f& V = Me.Verts[vi];
		double Px = V.X, Py = V.Y, Pz = V.Z;
		ApplyWorldTransform(G.Xpos, G.Xquat, Px, Py, Pz);
		EmitPoint(Out, Px, Py, Pz);
	}

	// Compute per-triangle area and total area.
	TArray<double> TriAreas;
	TriAreas.SetNum(TriCount);
	double TotalArea = 0.0;
	for (int32 ti = 0; ti < TriCount; ++ti)
	{
		const int32 I0 = Me.Tris[ti * 3 + 0];
		const int32 I1 = Me.Tris[ti * 3 + 1];
		const int32 I2 = Me.Tris[ti * 3 + 2];
		const FVector3f& V0 = Me.Verts[I0];
		const FVector3f& V1 = Me.Verts[I1];
		const FVector3f& V2 = Me.Verts[I2];
		const FVector E1(V1.X - V0.X, V1.Y - V0.Y, V1.Z - V0.Z);
		const FVector E2(V2.X - V0.X, V2.Y - V0.Y, V2.Z - V0.Z);
		const double Area = 0.5 * FVector::CrossProduct(E1, E2).Size();
		TriAreas[ti] = Area;
		TotalArea += Area;
	}
	if (TotalArea <= 0.0)
	{
		return;
	}

	const int32 TotalTarget = FMath::Max(0, FMath::CeilToInt32(TotalArea / (GResolution * GResolution)));
	if (TotalTarget <= 0)
	{
		return;
	}

	// Distribute samples across triangles proportional to area.
	Out.Reserve(Out.Num() + TotalTarget * 3);
	int32 Emitted = 0;
	for (int32 ti = 0; ti < TriCount && Emitted < TotalTarget; ++ti)
	{
		const int32 TargetForTri = FMath::Max(0,
			FMath::RoundToInt32(static_cast<double>(TotalTarget - Emitted) * TriAreas[ti] / TotalArea));
		if (TargetForTri <= 0)
		{
			continue;
		}
		const int32 I0 = Me.Tris[ti * 3 + 0];
		const int32 I1 = Me.Tris[ti * 3 + 1];
		const int32 I2 = Me.Tris[ti * 3 + 2];
		const FVector3f& V0 = Me.Verts[I0];
		const FVector3f& V1 = Me.Verts[I1];
		const FVector3f& V2 = Me.Verts[I2];
		const double V0x = V0.X, V0y = V0.Y, V0z = V0.Z;
		const double E1x = V1.X - V0x, E1y = V1.Y - V0y, E1z = V1.Z - V0z;
		const double E2x = V2.X - V0x, E2y = V2.Y - V0y, E2z = V2.Z - V0z;

		for (int32 s = 0; s < TargetForTri && Emitted < TotalTarget; ++s, ++Emitted)
		{
			double U = FMath::FRand();
			double V = FMath::FRand();
			if (U + V > 1.0)
			{
				U = 1.0 - U;
				V = 1.0 - V;
			}
			double Px = V0x + U * E1x + V * E2x;
			double Py = V0y + U * E1y + V * E2y;
			double Pz = V0z + U * E1z + V * E2z;
			ApplyWorldTransform(G.Xpos, G.Xquat, Px, Py, Pz);
			EmitPoint(Out, Px, Py, Pz);
		}
		TotalArea -= TriAreas[ti];
	}
}

void SampleWorldGeom(const FMjWorldGeom& G, TArray<float>& Out)
{
	switch (G.Shape)
	{
		case EMjWorldGeomShape::Box:
			SampleBox(G, Out);
			break;
		case EMjWorldGeomShape::Sphere:
			SampleSphere(G, Out);
			break;
		case EMjWorldGeomShape::Cylinder:
			SampleCylinder(G, Out);
			break;
		case EMjWorldGeomShape::Mesh:
			SampleMesh(G, Out);
			break;
	}
}
} // namespace

// sensor_msgs/PointCloud2 on /urlab/obstacle_cloud, world frame. Surface-samples
// every non-robot world geom at a fixed resolution and publishes at ~10 Hz.
class FMjRosPointCloudProvider : public IMjRosOutputProvider
{
public:
	virtual FName GetProviderName() const override { return TEXT("obstacle_cloud"); }

	virtual void Build(FMjRosPublisherFactory& /*Factory*/, const FMjStateSnapshot& /*Snapshot*/) override
	{
		UrlabRclContext* Ctx = FURLabRosContext::Get().GetHandle();
		if (!Ctx)
		{
			return;
		}
		Pub = UrlabRcl_CreatePointCloud2Pub(Ctx, "/urlab/obstacle_cloud", "world", 0);
	}

	virtual void Publish(const FMjStateSnapshot& Snapshot, int64 SimTimeNs) override
	{
		if (!Pub)
		{
			return;
		}
		static constexpr int64 PublishIntervalNs = 100'000'000;
		if (LastPublishNs != 0 && (SimTimeNs - LastPublishNs) < PublishIntervalNs)
		{
			return;
		}
		LastPublishNs = SimTimeNs;

		if (Snapshot.WorldGeoms.Num() == 0)
		{
			UrlabRcl_PublishPointCloud2(Pub, nullptr, 0, SimTimeNs);
			return;
		}

		Points.Reset();
		for (const FMjWorldGeom& G : Snapshot.WorldGeoms)
		{
			SampleWorldGeom(G, Points);
		}
		const int32 N = Points.Num() / 3;
		UrlabRcl_PublishPointCloud2(Pub, Points.GetData(), N, SimTimeNs);
	}

	virtual int32 GetPublisherCountForTest() const override { return Pub ? 1 : 0; }

	virtual ~FMjRosPointCloudProvider() { Destroy(); }

private:
	void Destroy()
	{
		if (Pub)
		{
			UrlabRcl_DestroyPointCloud2Pub(Pub);
			Pub = nullptr;
		}
		LastPublishNs = 0;
	}

	UrlabRclPointCloud2Pub* Pub = nullptr;
	int64 LastPublishNs = 0;
	TArray<float> Points;
};

REGISTER_MJ_ROS_OUTPUT_PROVIDER("obstacle_cloud", FMjRosPointCloudProvider);

#endif // URLAB_WITH_ROS2
