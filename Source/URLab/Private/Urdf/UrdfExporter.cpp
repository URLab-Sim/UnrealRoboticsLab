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

#include "Urdf/UrdfExporter.h"

#include "State/MjCanonicalName.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

#include "mujoco/mujoco.h"

#include <cmath>

namespace
{
// Match the Python prototype's %.9g formatting so numbers round-trip against the
// validated reference URDF.
FString FmtNum(double X)
{
	return FString::Printf(TEXT("%.9g"), X);
}

FString Xyz(const FVector& V)
{
	return FString::Printf(TEXT("%s %s %s"), *FmtNum(V.X), *FmtNum(V.Y), *FmtNum(V.Z));
}

// MuJoCo quaternion (w,x,y,z) -> URDF rpy (fixed-axis / extrinsic XYZ), matching
// urdfdom's Rotation::getRPY.
FVector QuatWxyzToRpy(const mjtNum* Q)
{
	double W = Q[0], X = Q[1], Y = Q[2], Z = Q[3];
	const double Nrm = std::sqrt(W * W + X * X + Y * Y + Z * Z);
	if (Nrm == 0.0)
		return FVector::ZeroVector;
	W /= Nrm; X /= Nrm; Y /= Nrm; Z /= Nrm;

	const double SinrCosp = 2.0 * (W * X + Y * Z);
	const double CosrCosp = 1.0 - 2.0 * (X * X + Y * Y);
	const double Roll = std::atan2(SinrCosp, CosrCosp);

	const double Sinp = 2.0 * (W * Y - Z * X);
	const double Pitch = (std::abs(Sinp) >= 1.0)
		? std::copysign(PI / 2.0, Sinp)
		: std::asin(Sinp);

	const double SinyCosp = 2.0 * (W * Z + X * Y);
	const double CosyCosp = 1.0 - 2.0 * (Y * Y + Z * Z);
	const double Yaw = std::atan2(SinyCosp, CosyCosp);
	return FVector(Roll, Pitch, Yaw);
}

// Rotate a vector by a MuJoCo (w,x,y,z) quaternion, no normalization (mjModel
// stores unit quaternions), matching the prototype's quat_rotate.
FVector RotateByQuatWxyz(const mjtNum* Q, const FVector& V)
{
	const double W = Q[0], X = Q[1], Y = Q[2], Z = Q[3];
	const double Vx = V.X, Vy = V.Y, Vz = V.Z;
	const double Tx = 2.0 * (Y * Vz - Z * Vy);
	const double Ty = 2.0 * (Z * Vx - X * Vz);
	const double Tz = 2.0 * (X * Vy - Y * Vx);
	return FVector(
		Vx + W * Tx + (Y * Tz - Z * Ty),
		Vy + W * Ty + (Z * Tx - X * Tz),
		Vz + W * Tz + (X * Ty - Y * Tx));
}

double AbsMax(double A, double B)
{
	return FMath::Max(FMath::Abs(A), FMath::Abs(B));
}

bool IsPlainMotor(const mjModel* M, int A)
{
	// force = gain * ctrl: fixed gain, no bias, no activation dynamics.
	return M->actuator_biastype[A] == mjBIAS_NONE
		&& M->actuator_gaintype[A] == mjGAIN_FIXED
		&& M->actuator_dyntype[A] == mjDYN_NONE;
}

bool IsVelocityActuator(const mjModel* M, int A)
{
	// mjcf <velocity>: affine bias with biasprm = [0, 0, -kv], so ctrl is a
	// velocity target. A position servo's affine bias is [0, -kp, -kv] instead.
	if (M->actuator_biastype[A] != mjBIAS_AFFINE)
		return false;
	const mjtNum* Bp = &M->actuator_biasprm[A * mjNBIAS];
	return std::abs(Bp[1]) < 1e-12 && std::abs(Bp[2]) > 0.0;
}

// Effort ladder: actuator force range limit, else summed bounded
// joint-transmission actuator contributions, else the config default.
double JointEffort(const mjModel* M, int J, const FUrdfExportConfig& Cfg,
	const FString& JointName, TArray<FString>& Warnings)
{
	if (M->jnt_actfrclimited[J])
		return AbsMax(M->jnt_actfrcrange[2 * J + 0], M->jnt_actfrcrange[2 * J + 1]);

	double Total = 0.0;
	bool bFound = false;
	bool bUnbounded = false;
	for (int A = 0; A < M->nu; ++A)
	{
		if (M->actuator_trntype[A] != mjTRN_JOINT || M->actuator_trnid[2 * A + 0] != J)
			continue;
		bFound = true;
		const double Gear = std::abs(M->actuator_gear[A * 6 + 0]);
		double B;
		if (M->actuator_forcelimited[A])
		{
			B = AbsMax(M->actuator_forcerange[2 * A + 0], M->actuator_forcerange[2 * A + 1]);
		}
		else if (M->actuator_ctrllimited[A] && IsPlainMotor(M, A))
		{
			B = M->actuator_gainprm[A * mjNGAIN + 0]
				* AbsMax(M->actuator_ctrlrange[2 * A + 0], M->actuator_ctrlrange[2 * A + 1]);
		}
		else
		{
			bUnbounded = true;
			continue;
		}
		Total += Gear * B;
	}
	if (bFound && !bUnbounded && Total > 0.0)
		return Total;

	Warnings.Add(FString::Printf(
		TEXT("joint '%s': effort defaulted to %s (no bounded joint-transmission "
			 "actuator; tendon/site transmissions do not count)"),
		*JointName, *FmtNum(Cfg.DefaultEffort)));
	return Cfg.DefaultEffort;
}

// Velocity ladder: override map, else a velocity-actuator ctrl range, else the
// type-specific config default. MuJoCo has no joint velocity limit, so a
// position-servo arm falls through to the default.
double JointVelocity(const mjModel* M, int J, bool bIsSlide,
	const FUrdfExportConfig& Cfg, const FString& JointName, TArray<FString>& Warnings)
{
	if (const double* Override = Cfg.VelocityOverrides.Find(JointName))
		return *Override;

	for (int A = 0; A < M->nu; ++A)
	{
		if (M->actuator_trntype[A] != mjTRN_JOINT || M->actuator_trnid[2 * A + 0] != J)
			continue;
		if (IsVelocityActuator(M, A) && M->actuator_ctrllimited[A])
			return AbsMax(M->actuator_ctrlrange[2 * A + 0], M->actuator_ctrlrange[2 * A + 1]);
	}

	const double Default = bIsSlide ? Cfg.DefaultVelocityLinear : Cfg.DefaultVelocityAngular;
	Warnings.Add(FString::Printf(
		TEXT("joint '%s': velocity defaulted to %s (MuJoCo has no joint velocity "
			 "limit; no velocity actuator found)"),
		*JointName, *FmtNum(Default)));
	return Default;
}
}  // namespace

FString FUrdfExporter::MeshBaseName(const mjModel* M, int32 MeshId)
{
	const char* Raw = mj_id2name(const_cast<mjModel*>(M), mjOBJ_MESH, MeshId);
	const FString Name = Raw ? FString(UTF8_TO_TCHAR(Raw)) : FString::Printf(TEXT("mesh%d"), MeshId);
	return FMjCanonicalName::Sanitize(Name);
}

TArray<int32> FUrdfExporter::BodyIdsForArt(const mjModel* M, const FString& ArtRawName)
{
	TArray<int32> Out;
	if (!M)
		return Out;
	const FString Prefix = ArtRawName + TEXT("_");
	for (int32 i = 1; i < M->nbody; ++i)
	{
		if (ArtRawName.IsEmpty())
		{
			Out.Add(i);
			continue;
		}
		const char* Raw = mj_id2name(const_cast<mjModel*>(M), mjOBJ_BODY, i);
		const FString Name = Raw ? FString(UTF8_TO_TCHAR(Raw)) : FString();
		if (Name == ArtRawName || Name.StartsWith(Prefix))
			Out.Add(i);
	}
	return Out;
}

FUrdfModel FUrdfExporter::Build(const mjModel* M, const FString& RobotName,
	const FString& ArtRawName, const TArray<int32>& BodyIds,
	const FString& MeshUriDir, const FUrdfExportConfig& Cfg)
{
	FUrdfModel Model;
	Model.RobotName = RobotName;
	if (!M)
		return Model;

	const FString Prefix = ArtRawName.IsEmpty() ? FString() : (ArtRawName + TEXT("_"));

	// Compiled name -> canonical URDF segment (art prefix stripped, sanitized),
	// mirroring FMjCanonicalName::PartSegment without needing the AMjArticulation.
	auto LocalName = [&Prefix](const char* Raw) -> FString
	{
		FString Local = Raw ? FString(UTF8_TO_TCHAR(Raw)) : FString();
		if (!Prefix.IsEmpty() && Local.StartsWith(Prefix))
			Local = Local.Mid(Prefix.Len());
		return FMjCanonicalName::Sanitize(Local);
	};
	auto BodyName = [M, &LocalName](int i) -> FString
	{
		return LocalName(mj_id2name(const_cast<mjModel*>(M), mjOBJ_BODY, i));
	};

	// Absolute file:// URI for a mesh id; also records the mesh id for STL export.
	auto MeshUri = [&](int32 MeshId) -> FString
	{
		Model.MeshIds.AddUnique(MeshId);
		const FString File = MeshBaseName(M, MeshId) + TEXT(".stl");
		FString Full = FPaths::ConvertRelativePathToFull(FPaths::Combine(MeshUriDir, File));
		Full.ReplaceInline(TEXT("\\"), TEXT("/"));
		return FString(TEXT("file://")) + Full;
	};

	const TSet<int32> BodySet(BodyIds);

	// Per-body frame offset from the jnt_pos anchor shift (single-joint case).
	TArray<FVector> Offset;
	Offset.Init(FVector::ZeroVector, M->nbody);

	int32 RootCount = 0;
	for (int32 i : BodyIds)
		if (M->body_parentid[i] == 0)
			++RootCount;
	if (BodyIds.Num() > 0 && RootCount != 1)
	{
		Model.Warnings.Add(FString::Printf(
			TEXT("%d world-rooted bodies; single-URDF export handles the first, a "
				 "synthetic-root welded ensemble is needed for multi-root arts"),
			RootCount));
	}

	TArray<FString> LinkXml;
	TArray<FString> JointXml;

	// Emit one joint (real, fixed, or a chained dummy segment) into JointXml and
	// Model.Joints. ForceFixed synthesizes the parentless-child fixed joint.
	auto EmitJoint = [&](int J, const FString& ParentLink, const FString& ChildLink,
		const FVector& OriginXyz, const FVector& OriginRpy, bool bForceFixed)
	{
		FUrdfJoint Joint;
		Joint.Parent = ParentLink;
		Joint.Child = ChildLink;
		Joint.OriginPos = OriginXyz;
		Joint.OriginRpy = OriginRpy;

		if (bForceFixed)
		{
			Joint.Type = TEXT("fixed");
			Joint.Name = FString::Printf(TEXT("%s__to__%s"), *ParentLink, *ChildLink);
		}
		else
		{
			Joint.MjJointId = J;
			const char* Raw = mj_id2name(const_cast<mjModel*>(M), mjOBJ_JOINT, J);
			Joint.Name = LocalName(Raw);
			const int JType = M->jnt_type[J];
			if (JType == mjJNT_HINGE)
			{
				Joint.Type = M->jnt_limited[J] ? TEXT("revolute") : TEXT("continuous");
			}
			else if (JType == mjJNT_SLIDE)
			{
				Joint.Type = TEXT("prismatic");
			}
			else if (JType == mjJNT_BALL)
			{
				Model.Warnings.Add(FString::Printf(
					TEXT("joint '%s': ball joint exported as fixed (URDF has no ball)"), *Joint.Name));
				Joint.Type = TEXT("fixed");
			}
			else if (JType == mjJNT_FREE)
			{
				Model.Warnings.Add(FString::Printf(
					TEXT("joint '%s': free joint omitted; floating base rides tf2"), *Joint.Name));
				Joint.Type = TEXT("fixed");
			}
			else
			{
				Joint.Type = TEXT("fixed");
			}
		}

		TArray<FString> Lines;
		Lines.Add(FString::Printf(TEXT("  <joint name=\"%s\" type=\"%s\">"), *Joint.Name, *Joint.Type));
		Lines.Add(FString::Printf(TEXT("    <parent link=\"%s\"/>"), *ParentLink));
		Lines.Add(FString::Printf(TEXT("    <child link=\"%s\"/>"), *ChildLink));
		Lines.Add(FString::Printf(TEXT("    <origin xyz=\"%s\" rpy=\"%s %s %s\"/>"),
			*Xyz(OriginXyz), *FmtNum(OriginRpy.X), *FmtNum(OriginRpy.Y), *FmtNum(OriginRpy.Z)));

		if (Joint.Type == TEXT("revolute") || Joint.Type == TEXT("prismatic")
			|| Joint.Type == TEXT("continuous"))
		{
			const mjtNum* Ax = &M->jnt_axis[3 * J];
			Joint.Axis = FVector(Ax[0], Ax[1], Ax[2]);
			Lines.Add(FString::Printf(TEXT("    <axis xyz=\"%s %s %s\"/>"),
				*FmtNum(Ax[0]), *FmtNum(Ax[1]), *FmtNum(Ax[2])));

			const bool bIsSlide = (M->jnt_type[J] == mjJNT_SLIDE);
			Joint.Effort = JointEffort(M, J, Cfg, Joint.Name, Model.Warnings);
			Joint.Velocity = JointVelocity(M, J, bIsSlide, Cfg, Joint.Name, Model.Warnings);

			if (Joint.Type == TEXT("continuous"))
			{
				Lines.Add(FString::Printf(TEXT("    <limit effort=\"%s\" velocity=\"%s\"/>"),
					*FmtNum(Joint.Effort), *FmtNum(Joint.Velocity)));
			}
			else
			{
				const double Q0 = M->qpos0[M->jnt_qposadr[J]];
				Joint.Lower = M->jnt_range[2 * J + 0] - Q0;
				Joint.Upper = M->jnt_range[2 * J + 1] - Q0;
				Joint.bHasLimit = true;
				Lines.Add(FString::Printf(
					TEXT("    <limit lower=\"%s\" upper=\"%s\" effort=\"%s\" velocity=\"%s\"/>"),
					*FmtNum(Joint.Lower), *FmtNum(Joint.Upper),
					*FmtNum(Joint.Effort), *FmtNum(Joint.Velocity)));
			}
		}
		Lines.Add(TEXT("  </joint>"));
		JointXml.Add(FString::Join(Lines, TEXT("\n")));
		Model.Joints.Add(MoveTemp(Joint));
	};

	for (int32 i : BodyIds)
	{
		const int NJnt = M->body_jntnum[i];
		const int Parent = M->body_parentid[i];

		if (NJnt == 1)
		{
			const int J = M->body_jntadr[i];
			if (M->jnt_type[J] == mjJNT_HINGE || M->jnt_type[J] == mjJNT_SLIDE)
			{
				const mjtNum* Jp = &M->jnt_pos[3 * J];
				Offset[i] = FVector(Jp[0], Jp[1], Jp[2]);
			}
		}

		// --- link ---
		FUrdfLink Link;
		Link.Name = BodyName(i);

		TArray<FString> Parts;
		Parts.Add(FString::Printf(TEXT("  <link name=\"%s\">"), *Link.Name));
		if (M->body_mass[i] > 0.0)
		{
			const mjtNum* Ip = &M->body_ipos[3 * i];
			const FVector IpV = FVector(Ip[0], Ip[1], Ip[2]) - Offset[i];
			const FVector Rpy = QuatWxyzToRpy(&M->body_iquat[4 * i]);
			const mjtNum* In = &M->body_inertia[3 * i];
			Parts.Add(TEXT("    <inertial>"));
			Parts.Add(FString::Printf(TEXT("      <origin xyz=\"%s\" rpy=\"%s %s %s\"/>"),
				*Xyz(IpV), *FmtNum(Rpy.X), *FmtNum(Rpy.Y), *FmtNum(Rpy.Z)));
			Parts.Add(FString::Printf(TEXT("      <mass value=\"%s\"/>"), *FmtNum(M->body_mass[i])));
			Parts.Add(FString::Printf(
				TEXT("      <inertia ixx=\"%s\" ixy=\"0\" ixz=\"0\" iyy=\"%s\" iyz=\"0\" izz=\"%s\"/>"),
				*FmtNum(In[0]), *FmtNum(In[1]), *FmtNum(In[2])));
			Parts.Add(TEXT("    </inertial>"));
		}

		for (int g = 0; g < M->ngeom; ++g)
		{
			if (M->geom_bodyid[g] != i)
				continue;

			FString GeoXml;
			int32 UsedMeshId = -1;
			const int GType = M->geom_type[g];
			const mjtNum* Sz = &M->geom_size[3 * g];
			if (GType == mjGEOM_SPHERE)
			{
				GeoXml = FString::Printf(TEXT("<sphere radius=\"%s\"/>"), *FmtNum(Sz[0]));
			}
			else if (GType == mjGEOM_BOX)
			{
				GeoXml = FString::Printf(TEXT("<box size=\"%s %s %s\"/>"),
					*FmtNum(2 * Sz[0]), *FmtNum(2 * Sz[1]), *FmtNum(2 * Sz[2]));
			}
			else if (GType == mjGEOM_CYLINDER)
			{
				GeoXml = FString::Printf(TEXT("<cylinder radius=\"%s\" length=\"%s\"/>"),
					*FmtNum(Sz[0]), *FmtNum(2 * Sz[1]));
			}
			else if (GType == mjGEOM_CAPSULE)
			{
				Model.Warnings.Add(FString::Printf(
					TEXT("geom %d: capsule approximated as a cylinder"), g));
				GeoXml = FString::Printf(TEXT("<cylinder radius=\"%s\" length=\"%s\"/>"),
					*FmtNum(Sz[0]), *FmtNum(2 * Sz[1]));
			}
			else if (GType == mjGEOM_MESH)
			{
				UsedMeshId = M->geom_dataid[g];
				GeoXml = FString::Printf(TEXT("<mesh filename=\"%s\" scale=\"1 1 1\"/>"),
					*MeshUri(UsedMeshId));
			}
			else
			{
				Model.Warnings.Add(FString::Printf(
					TEXT("geom %d: type %d has no URDF equivalent; dropped"), g, GType));
				continue;
			}

			const mjtNum* Gp = &M->geom_pos[3 * g];
			const FVector GpV = FVector(Gp[0], Gp[1], Gp[2]) - Offset[i];
			const FVector Rpy = QuatWxyzToRpy(&M->geom_quat[4 * g]);

			FUrdfGeomFrame Frame;
			Frame.MjGeomId = g;
			Frame.LocalPos = GpV;
			Frame.LocalRpy = Rpy;
			Link.Geoms.Add(Frame);

			const bool bCollision = M->geom_contype[g] != 0 || M->geom_conaffinity[g] != 0;
			const TCHAR* Tag = bCollision ? TEXT("collision") : TEXT("visual");
			Parts.Add(FString::Printf(TEXT("    <%s>"), Tag));
			Parts.Add(FString::Printf(TEXT("      <origin xyz=\"%s\" rpy=\"%s %s %s\"/>"),
				*Xyz(GpV), *FmtNum(Rpy.X), *FmtNum(Rpy.Y), *FmtNum(Rpy.Z)));
			Parts.Add(FString::Printf(TEXT("      <geometry>%s</geometry>"), *GeoXml));
			if (!bCollision)
			{
				const float* Rgba = &M->geom_rgba[4 * g];
				Parts.Add(FString::Printf(
					TEXT("      <material name=\"%s_mat_%d\"><color rgba=\"%s %s %s %s\"/></material>"),
					*Link.Name, g, *FmtNum(Rgba[0]), *FmtNum(Rgba[1]), *FmtNum(Rgba[2]), *FmtNum(Rgba[3])));
			}
			Parts.Add(FString::Printf(TEXT("    </%s>"), Tag));
		}
		Parts.Add(TEXT("  </link>"));
		LinkXml.Add(FString::Join(Parts, TEXT("\n")));
		Model.Links.Add(MoveTemp(Link));

		if (Parent == 0)
			continue;  // subtree root: no joint to world, placement rides tf2

		const FVector ParentOff = Offset[Parent];
		const mjtNum* Bp = &M->body_pos[3 * i];
		const FVector BodyPos(Bp[0], Bp[1], Bp[2]);
		const mjtNum* Bq = &M->body_quat[4 * i];
		const FVector ChildLinkInParentBody = BodyPos + RotateByQuatWxyz(Bq, Offset[i]);
		const FVector OriginXyz = ChildLinkInParentBody - ParentOff;
		const FVector OriginRpy = QuatWxyzToRpy(Bq);

		const FString ChildLink = BodyName(i);
		const FString ParentLink = BodyName(Parent);

		if (NJnt == 0)
		{
			EmitJoint(-1, ParentLink, ChildLink, OriginXyz, OriginRpy, /*bForceFixed=*/true);
		}
		else if (NJnt == 1)
		{
			EmitJoint(M->body_jntadr[i], ParentLink, ChildLink, OriginXyz, OriginRpy, false);
		}
		else
		{
			// Multi-joint body: k-1 zero-inertia dummy links carry the first k-1
			// joints; the real link is carried by the last. Successive joints share
			// the body frame, so intermediate origins are identity.
			Model.Warnings.Add(FString::Printf(
				TEXT("body '%s': %d joints -> %d dummy links inserted"), *ChildLink, NJnt, NJnt - 1));
			FString PrevLink = ParentLink;
			FVector CurXyz = OriginXyz;
			FVector CurRpy = OriginRpy;
			for (int k = 0; k < NJnt; ++k)
			{
				const int J = M->body_jntadr[i] + k;
				if (k < NJnt - 1)
				{
					const FString DLink = FString::Printf(TEXT("%s__j%d"), *ChildLink, k);
					LinkXml.Add(FString::Printf(TEXT("  <link name=\"%s\"/>"), *DLink));
					FUrdfLink Dummy;
					Dummy.Name = DLink;
					Model.Links.Add(MoveTemp(Dummy));
					EmitJoint(J, PrevLink, DLink, CurXyz, CurRpy, false);
					PrevLink = DLink;
					CurXyz = FVector::ZeroVector;
					CurRpy = FVector::ZeroVector;
				}
				else
				{
					EmitJoint(J, PrevLink, ChildLink, CurXyz, CurRpy, false);
				}
			}
		}
	}

	if (M->ntendon > 0)
	{
		Model.Warnings.Add(FString::Printf(
			TEXT("%d tendon(s) dropped (no URDF representation)"), M->ntendon));
	}
	if (M->neq > 0)
	{
		Model.Warnings.Add(FString::Printf(
			TEXT("%d equality constraint(s) dropped (URDF is a tree; coupling like "
				 "finger mimic is lost)"), M->neq));
	}

	TArray<FString> Doc;
	Doc.Add(TEXT("<?xml version=\"1.0\"?>"));
	Doc.Add(FString::Printf(TEXT("<robot name=\"%s\">"), *RobotName));
	Doc.Append(LinkXml);
	Doc.Append(JointXml);
	Doc.Add(TEXT("</robot>"));
	Model.Xml = FString::Join(Doc, TEXT("\n")) + TEXT("\n");
	return Model;
}

bool FUrdfExporter::WriteMeshStl(const mjModel* M, int32 MeshId, const FString& Dir,
	FString& OutFilename)
{
	if (!M || MeshId < 0 || MeshId >= M->nmesh)
		return false;

	const int VertAdr = M->mesh_vertadr[MeshId];
	const int VertNum = M->mesh_vertnum[MeshId];
	const int FaceAdr = M->mesh_faceadr[MeshId];
	const int FaceNum = M->mesh_facenum[MeshId];

	TArray<uint8> Bytes;
	Bytes.Reserve(84 + FaceNum * 50);

	auto AppendU16 = [&Bytes](uint16 V) {
		Bytes.Append(reinterpret_cast<const uint8*>(&V), sizeof(V));
	};
	auto AppendU32 = [&Bytes](uint32 V) {
		Bytes.Append(reinterpret_cast<const uint8*>(&V), sizeof(V));
	};
	auto AppendF32 = [&Bytes](float V) {
		Bytes.Append(reinterpret_cast<const uint8*>(&V), sizeof(V));
	};

	// 80-byte zero header + triangle count.
	for (int i = 0; i < 80; ++i)
		Bytes.Add(0);
	AppendU32(static_cast<uint32>(FaceNum));

	auto Vert = [M, VertAdr](int Local) -> FVector {
		const float* V = &M->mesh_vert[3 * (VertAdr + Local)];
		return FVector(V[0], V[1], V[2]);
	};

	for (int Tri = 0; Tri < FaceNum; ++Tri)
	{
		const int* F = &M->mesh_face[3 * (FaceAdr + Tri)];
		const FVector V0 = Vert(F[0]);
		const FVector V1 = Vert(F[1]);
		const FVector V2 = Vert(F[2]);
		FVector N = FVector::CrossProduct(V1 - V0, V2 - V0);
		const double Len = N.Size();
		N = (Len > 0.0) ? (N / Len) : FVector::ZeroVector;

		AppendF32(static_cast<float>(N.X));
		AppendF32(static_cast<float>(N.Y));
		AppendF32(static_cast<float>(N.Z));
		for (const FVector& V : {V0, V1, V2})
		{
			AppendF32(static_cast<float>(V.X));
			AppendF32(static_cast<float>(V.Y));
			AppendF32(static_cast<float>(V.Z));
		}
		AppendU16(0);
	}

	OutFilename = MeshBaseName(M, MeshId) + TEXT(".stl");
	const FString Path = FPaths::Combine(Dir, OutFilename);
	return FFileHelper::SaveArrayToFile(Bytes, *Path);
}

FUrdfModel FUrdfExporter::ExportToDir(const mjModel* M, const FString& RobotName,
	const FString& ArtRawName, const FString& OutDir, const FUrdfExportConfig& Cfg)
{
	FUrdfModel Model;
	if (!M)
		return Model;

	const FString MeshDir = FPaths::Combine(OutDir, TEXT("meshes"));
	IFileManager::Get().MakeDirectory(*MeshDir, /*Tree=*/true);

	const TArray<int32> BodyIds = BodyIdsForArt(M, ArtRawName);
	Model = Build(M, RobotName, ArtRawName, BodyIds, MeshDir, Cfg);

	for (int32 MeshId : Model.MeshIds)
	{
		FString Unused;
		if (!WriteMeshStl(M, MeshId, MeshDir, Unused))
		{
			Model.Warnings.Add(FString::Printf(TEXT("mesh %d: STL write failed"), MeshId));
		}
	}

	const FString UrdfPath = FPaths::Combine(OutDir, TEXT("model.urdf"));
	FFileHelper::SaveStringToFile(Model.Xml, *UrdfPath);
	return Model;
}
