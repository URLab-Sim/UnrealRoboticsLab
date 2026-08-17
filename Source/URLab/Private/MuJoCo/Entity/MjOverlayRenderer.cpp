// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Entity/MjOverlayRenderer.h"

#include "DrawDebugHelpers.h"
#include "Engine/World.h"

#include "MuJoCo/Core/MjRenderSnapshot.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "MuJoCo/Utils/URLabAxisConv.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace
{
// A flag set in a runtime-sized array; absent/short arrays read as off.
bool FlagSet(const TArray<uint8>& Flags, int32 Index)
{
	return Flags.IsValidIndex(Index) && Flags[Index] != 0;
}

// A group's visibility. An unsized mask (nothing authored yet) shows every group,
// so a freshly bound renderer draws rather than hiding everything by default.
bool GroupVisible(const TArray<uint8>& Mask, int32 Group)
{
	if (Mask.Num() == 0)
	{
		return true;
	}
	return Mask.IsValidIndex(Group) && Mask[Group] != 0;
}

// Column c (0..2) of a row-major 3x3 world rotation, i.e. that frame axis in
// world coordinates: element (r, c) is Mat[r * 3 + c].
void MatColumn(const mjtNum* Mat, int32 c, mjtNum Out[3])
{
	Out[0] = Mat[0 * 3 + c];
	Out[1] = Mat[1 * 3 + c];
	Out[2] = Mat[2 * 3 + c];
}
} // namespace

void UMjOverlayRenderer::SetModel(mjModel_* InModel)
{
	Model = InModel;
}

void UMjOverlayRenderer::DrawOverlays(const FMjRenderSnapshot& Snap) const
{
	if (!Model || !GetWorld())
	{
		return;
	}
	if (FlagSet(Flags.VisFlags, mjVIS_CONVEXHULL))
	{
		DrawCollision(Snap);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_JOINT))
	{
		DrawJoints(Snap);
	}
	if (bDrawSites)
	{
		DrawSites(Snap);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_COM))
	{
		DrawCom(Snap);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_INERTIA))
	{
		DrawInertia(Snap);
	}
	const bool bContactPoints = FlagSet(Flags.VisFlags, mjVIS_CONTACTPOINT);
	const bool bContactForces = FlagSet(Flags.VisFlags, mjVIS_CONTACTFORCE);
	const bool bContactSplit = FlagSet(Flags.VisFlags, mjVIS_CONTACTSPLIT);
	if (bContactPoints || bContactForces)
	{
		DrawContacts(Snap, bContactPoints, bContactForces, bContactSplit);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_PERTFORCE) || FlagSet(Flags.VisFlags, mjVIS_PERTOBJ))
	{
		DrawPerturb(Snap);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_CAMERA))
	{
		DrawCameras(Snap);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_LIGHT))
	{
		DrawLights(Snap);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_ACTUATOR))
	{
		DrawActuators(Snap);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_TENDON))
	{
		DrawTendons(Snap);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_RANGEFINDER))
	{
		DrawRangefinders(Snap);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_CONSTRAINT))
	{
		DrawConstraints(Snap);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_STATIC))
	{
		DrawStaticBodies(Snap);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_AUTOCONNECT))
	{
		DrawAutoConnect(Snap);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_TRANSPARENT))
	{
		DrawGeomBounds(Snap);
	}
}

void UMjOverlayRenderer::DrawCollision(const FMjRenderSnapshot& Snap) const
{
	UWorld* World = GetWorld();
	const mjModel* M = Model;
	for (int32 G = 0; G < static_cast<int32>(M->ngeom); ++G)
	{
		if (!GroupVisible(Flags.GeomGroup, M->geom_group[G]))
		{
			continue;
		}
		if (!Snap.GeomXPos.IsValidIndex(G * 3 + 2) || !Snap.GeomXMat.IsValidIndex(G * 9 + 8))
		{
			continue;
		}
		MjUtils::DrawDebugGeom(World, M, G, &Snap.GeomXPos[G * 3], &Snap.GeomXMat[G * 9],
			FColor::Magenta, 100.0f);
	}
}

void UMjOverlayRenderer::DrawJoints(const FMjRenderSnapshot& Snap) const
{
	UWorld* World = GetWorld();
	const mjModel* M = Model;
	for (int32 J = 0; J < static_cast<int32>(M->njnt); ++J)
	{
		const int32 Type = M->jnt_type[J];
		if (Type != mjJNT_HINGE && Type != mjJNT_SLIDE)
		{
			continue;
		}
		if (!Snap.JntXAnchor.IsValidIndex(J * 3 + 2) || !Snap.JntXAxis.IsValidIndex(J * 3 + 2))
		{
			continue;
		}

		const FVector Anchor = URLabAxisConv::MjPositionToUe(&Snap.JntXAnchor[J * 3]) + SceneOrigin;
		const FVector Axis = URLabAxisConv::MjDirectionToUe(&Snap.JntXAxis[J * 3]);

		float RangeMin = static_cast<float>(M->jnt_range[J * 2 + 0]);
		float RangeMax = static_cast<float>(M->jnt_range[J * 2 + 1]);
		const bool bLimited = RangeMin != 0.0f || RangeMax != 0.0f;

		const int32 QAdr = M->jnt_qposadr[J];
		float CurrentPos = Snap.QPos.IsValidIndex(QAdr) ? static_cast<float>(Snap.QPos[QAdr]) : NAN;
		float RefPos = static_cast<float>(M->qpos0[QAdr]);

		// MuJoCo stores a slide's travel in metres; the draw helper wants cm.
		if (Type == mjJNT_SLIDE)
		{
			RangeMin *= 100.0f;
			RangeMax *= 100.0f;
			if (!FMath::IsNaN(CurrentPos))
			{
				CurrentPos *= 100.0f;
			}
			RefPos *= 100.0f;
		}

		MjUtils::DrawDebugJoint(World, Anchor, Axis, Type, bLimited, RangeMin, RangeMax, CurrentPos, RefPos);
	}
}

void UMjOverlayRenderer::DrawSites(const FMjRenderSnapshot& Snap) const
{
	UWorld* World = GetWorld();
	const mjModel* M = Model;
	for (int32 S = 0; S < static_cast<int32>(M->nsite); ++S)
	{
		if (!GroupVisible(Flags.SiteGroup, M->site_group[S]))
		{
			continue;
		}
		if (!Snap.SiteXPos.IsValidIndex(S * 3 + 2))
		{
			continue;
		}

		const FVector Pos = URLabAxisConv::MjPositionToUe(&Snap.SiteXPos[S * 3]) + SceneOrigin;
		const float* Rgba = &M->site_rgba[S * 4];
		FColor Color(static_cast<uint8>(Rgba[0] * 255.0), static_cast<uint8>(Rgba[1] * 255.0),
			static_cast<uint8>(Rgba[2] * 255.0), 200);

		const float Radius = FMath::Max(static_cast<float>(M->site_size[S * 3]) * 100.0f, 0.5f);
		const float CrossSize = FMath::Max(Radius * 2.0f, 2.0f);
		DrawDebugPoint(World, Pos, 6.0f, Color, false, -1);
		DrawDebugLine(World, Pos - FVector(CrossSize, 0, 0), Pos + FVector(CrossSize, 0, 0), Color, false, -1, 0, 1.0f);
		DrawDebugLine(World, Pos - FVector(0, CrossSize, 0), Pos + FVector(0, CrossSize, 0), Color, false, -1, 0, 1.0f);
		DrawDebugLine(World, Pos - FVector(0, 0, CrossSize), Pos + FVector(0, 0, CrossSize), Color, false, -1, 0, 1.0f);
	}
}

void UMjOverlayRenderer::DrawCom(const FMjRenderSnapshot& Snap) const
{
	UWorld* World = GetWorld();
	const mjModel* M = Model;
	// Body 0 is the world; skip it, matching simulate's per-body subtree markers.
	for (int32 B = 1; B < static_cast<int32>(M->nbody); ++B)
	{
		if (!Snap.SubtreeCom.IsValidIndex(B * 3 + 2))
		{
			continue;
		}
		const FVector Pos = URLabAxisConv::MjPositionToUe(&Snap.SubtreeCom[B * 3]) + SceneOrigin;
		DrawDebugSphere(World, Pos, 3.0f, 8, FColor(255, 128, 255), false, -1, 0, 0.5f);
	}
}

void UMjOverlayRenderer::DrawInertia(const FMjRenderSnapshot& Snap) const
{
	UWorld* World = GetWorld();
	const mjModel* M = Model;
	for (int32 B = 1; B < static_cast<int32>(M->nbody); ++B)
	{
		const mjtNum Mass = M->body_mass[B];
		if (Mass <= 0.0)
		{
			continue;
		}
		if (!Snap.XiPos.IsValidIndex(B * 3 + 2) || !Snap.XiMat.IsValidIndex(B * 9 + 8))
		{
			continue;
		}

		// Equivalent inertia box: solve the uniform-box moments for the full edge
		// lengths, then halve for the draw extent. Same box simulate renders.
		const mjtNum* I = &M->body_inertia[B * 3];
		const mjtNum T = 6.0 * (I[0] + I[1] + I[2]) / Mass;
		const mjtNum Ex = FMath::Sqrt(FMath::Max(0.0, T - 12.0 * I[0] / Mass));
		const mjtNum Ey = FMath::Sqrt(FMath::Max(0.0, T - 12.0 * I[1] / Mass));
		const mjtNum Ez = FMath::Sqrt(FMath::Max(0.0, T - 12.0 * I[2] / Mass));

		const FVector Pos = URLabAxisConv::MjPositionToUe(&Snap.XiPos[B * 3]) + SceneOrigin;
		mjtNum Quat[4];
		mju_mat2Quat(Quat, &Snap.XiMat[B * 9]);
		const FQuat Rot = URLabAxisConv::MjQuatToUe(Quat);

		// Metres -> cm half-extent: full edge * 100 / 2 = * 50.
		const FVector Extent(Ex * 50.0, Ey * 50.0, Ez * 50.0);
		DrawDebugBox(World, Pos, Extent, Rot, FColor(120, 180, 255), false, -1, 0, 0.3f);
	}
}

void UMjOverlayRenderer::DrawContacts(const FMjRenderSnapshot& Snap, bool bPoints, bool bForces, bool bSplit) const
{
	UWorld* World = GetWorld();
	for (const FMjContactViz& C : Snap.Contacts)
	{
		const FVector Pos = URLabAxisConv::MjPositionToUe(C.Pos) + SceneOrigin;
		if (bPoints)
		{
			DrawDebugPoint(World, Pos, 8.0f, FColor::Cyan, false, -1);
		}
		if (!bForces)
		{
			continue;
		}

		auto DrawWrench = [&](const mjtNum InForce[3], const FColor& Colour, float Thickness) {
			// Rotate a contact-frame force into world: the frame's rows are its
			// world-space axes, so world_f = sum_i frame_row_i * force_i.
			mjtNum FW[3] = {0, 0, 0};
			for (int32 k = 0; k < 3; ++k)
			{
				FW[k] = C.Frame[0 * 3 + k] * InForce[0] + C.Frame[1 * 3 + k] * InForce[1]
						+ C.Frame[2 * 3 + k] * InForce[2];
			}
			const FVector Dir = URLabAxisConv::MjDirectionToUe(FW);
			// Newtons -> cm at a fixed visual scale; clamp so a big impulse stays on screen.
			const float LenCm = FMath::Clamp(static_cast<float>(Dir.Size()) * 2.0f, 0.0f, 200.0f);
			if (LenCm > 0.5f)
			{
				const FVector End = Pos + Dir.GetSafeNormal() * LenCm;
				DrawDebugDirectionalArrow(World, Pos, End, 8.0f, Colour, false, -1, 0, Thickness);
			}
		};

		if (bSplit)
		{
			// Contact frame row 0 is the normal, rows 1-2 the tangents; the force
			// vector's first component is normal, the other two are friction. Draw
			// each subspace on its own so the split reads at a glance.
			const mjtNum Normal[3] = {C.Force[0], 0, 0};
			const mjtNum Tangent[3] = {0, C.Force[1], C.Force[2]};
			DrawWrench(Normal, FColor::Red, 1.5f);
			DrawWrench(Tangent, FColor(64, 160, 255), 1.0f);
		}
		else
		{
			DrawWrench(C.Force, FColor::Red, 1.0f);
		}
	}
}

void UMjOverlayRenderer::DrawPerturb(const FMjRenderSnapshot& Snap) const
{
	UWorld* World = GetWorld();
	if (PerturbBodyId < 0 || !Snap.XPos.IsValidIndex(PerturbBodyId * 3 + 2))
	{
		return;
	}
	const FVector BodyPos = URLabAxisConv::MjPositionToUe(&Snap.XPos[PerturbBodyId * 3]) + SceneOrigin;

	if (FlagSet(Flags.VisFlags, mjVIS_PERTOBJ))
	{
		DrawDebugSphere(World, BodyPos, 6.0f, 10, FColor::Yellow, false, -1, 0, 1.0f);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_PERTFORCE))
	{
		const FVector Dir = URLabAxisConv::MjDirectionToUe(PerturbForce);
		const float LenCm = FMath::Clamp(static_cast<float>(Dir.Size()) * 2.0f, 0.0f, 200.0f);
		if (LenCm > 0.5f)
		{
			const FVector End = BodyPos + Dir.GetSafeNormal() * LenCm;
			DrawDebugDirectionalArrow(World, BodyPos, End, 10.0f, FColor::Orange, false, -1, 0, 1.5f);
		}
	}
}

void UMjOverlayRenderer::DrawCameras(const FMjRenderSnapshot& Snap) const
{
	UWorld* World = GetWorld();
	const mjModel* M = Model;
	// A little frustum: apex at the camera, opening along its -z (MuJoCo cameras
	// look down local -z), sized by the vertical fovy and the sensor aspect.
	const mjtNum Depth = 0.15; // metres, purely a visual reach
	for (int32 C = 0; C < static_cast<int32>(M->ncam); ++C)
	{
		if (!Snap.CamXPos.IsValidIndex(C * 3 + 2) || !Snap.CamXMat.IsValidIndex(C * 9 + 8))
		{
			continue;
		}
		const mjtNum* P = &Snap.CamXPos[C * 3];
		const mjtNum* R = &Snap.CamXMat[C * 9];
		mjtNum X[3], Y[3], Z[3];
		MatColumn(R, 0, X);
		MatColumn(R, 1, Y);
		MatColumn(R, 2, Z);

		double Aspect = 1.0;
		if (M->cam_resolution && M->cam_resolution[C * 2 + 1] > 0)
		{
			Aspect = static_cast<double>(M->cam_resolution[C * 2 + 0]) / static_cast<double>(M->cam_resolution[C * 2 + 1]);
		}
		const double Fovy = M->cam_fovy ? FMath::DegreesToRadians(M->cam_fovy[C]) : FMath::DegreesToRadians(45.0);
		const double HalfH = Depth * FMath::Tan(Fovy * 0.5);
		const double HalfW = HalfH * Aspect;

		auto ToUe = [&](const mjtNum W[3]) { return URLabAxisConv::MjPositionToUe(W) + SceneOrigin; };
		const FVector Apex = ToUe(P);

		FVector Corners[4];
		const double Signs[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
		for (int32 i = 0; i < 4; ++i)
		{
			mjtNum WPt[3];
			for (int32 k = 0; k < 3; ++k)
			{
				WPt[k] = P[k] - Z[k] * Depth + X[k] * (Signs[i][0] * HalfW) + Y[k] * (Signs[i][1] * HalfH);
			}
			Corners[i] = ToUe(WPt);
		}

		const FColor Colour(80, 200, 255);
		DrawDebugPoint(World, Apex, 8.0f, Colour, false, -1);
		for (int32 i = 0; i < 4; ++i)
		{
			DrawDebugLine(World, Apex, Corners[i], Colour, false, -1, 0, 1.0f);
			DrawDebugLine(World, Corners[i], Corners[(i + 1) % 4], Colour, false, -1, 0, 1.0f);
		}
	}
}

void UMjOverlayRenderer::DrawLights(const FMjRenderSnapshot& Snap) const
{
	UWorld* World = GetWorld();
	const mjModel* M = Model;
	for (int32 L = 0; L < static_cast<int32>(M->nlight); ++L)
	{
		const int32 Bid = M->light_bodyid ? M->light_bodyid[L] : -1;
		if (Bid < 0 || !Snap.XPos.IsValidIndex(Bid * 3 + 2) || !Snap.XQuat.IsValidIndex(Bid * 4 + 3))
		{
			continue;
		}
		const mjtNum* Bp = &Snap.XPos[Bid * 3];
		const mjtNum* Bq = &Snap.XQuat[Bid * 4];

		// Light pos / dir are stored in the parent body frame; compose with the
		// body's world pose to place them, matching how the engine positions lights.
		mjtNum LocalPos[3] = {0, 0, 0};
		mjtNum LocalDir[3] = {0, 0, -1};
		if (M->light_pos)
		{
			for (int32 k = 0; k < 3; ++k)
				LocalPos[k] = M->light_pos[L * 3 + k];
		}
		if (M->light_dir)
		{
			for (int32 k = 0; k < 3; ++k)
				LocalDir[k] = M->light_dir[L * 3 + k];
		}

		mjtNum RotPos[3], WorldPos[3], WorldDir[3];
		mju_rotVecQuat(RotPos, LocalPos, Bq);
		for (int32 k = 0; k < 3; ++k)
			WorldPos[k] = Bp[k] + RotPos[k];
		mju_rotVecQuat(WorldDir, LocalDir, Bq);

		const FVector Pos = URLabAxisConv::MjPositionToUe(WorldPos) + SceneOrigin;
		const FVector Dir = URLabAxisConv::MjDirectionToUe(WorldDir).GetSafeNormal();

		const FColor Colour(255, 240, 120);
		DrawDebugSphere(World, Pos, 4.0f, 8, Colour, false, -1, 0, 1.0f);
		if (!Dir.IsNearlyZero())
		{
			DrawDebugDirectionalArrow(World, Pos, Pos + Dir * 20.0f, 6.0f, Colour, false, -1, 0, 1.5f);
		}
	}
}

void UMjOverlayRenderer::DrawActuators(const FMjRenderSnapshot& Snap) const
{
	UWorld* World = GetWorld();
	const mjModel* M = Model;
	for (int32 A = 0; A < static_cast<int32>(M->nu); ++A)
	{
		if (!GroupVisible(Flags.ActuatorGroup, M->actuator_group ? M->actuator_group[A] : 0))
		{
			continue;
		}

		// Anchor + a direction to grow the bar along, resolved from the
		// transmission target: a joint's world anchor/axis, or a site's origin
		// and local z. Other transmission types have no single world point to
		// mark, so they are skipped.
		FVector Anchor;
		FVector Axis;
		const int32 Trn = M->actuator_trntype ? M->actuator_trntype[A] : -1;
		if (Trn == mjTRN_JOINT || Trn == mjTRN_JOINTINPARENT)
		{
			const int32 Jid = M->actuator_trnid[A * 2];
			if (Jid < 0 || !Snap.JntXAnchor.IsValidIndex(Jid * 3 + 2) || !Snap.JntXAxis.IsValidIndex(Jid * 3 + 2))
				continue;
			Anchor = URLabAxisConv::MjPositionToUe(&Snap.JntXAnchor[Jid * 3]) + SceneOrigin;
			Axis = URLabAxisConv::MjDirectionToUe(&Snap.JntXAxis[Jid * 3]).GetSafeNormal();
		}
		else if (Trn == mjTRN_SITE)
		{
			const int32 Sid = M->actuator_trnid[A * 2];
			if (Sid < 0 || !Snap.SiteXPos.IsValidIndex(Sid * 3 + 2) || !Snap.SiteXMat.IsValidIndex(Sid * 9 + 8))
				continue;
			Anchor = URLabAxisConv::MjPositionToUe(&Snap.SiteXPos[Sid * 3]) + SceneOrigin;
			mjtNum ZAxis[3];
			MatColumn(&Snap.SiteXMat[Sid * 9], 2, ZAxis);
			Axis = URLabAxisConv::MjDirectionToUe(ZAxis).GetSafeNormal();
		}
		else
		{
			continue;
		}

		if (Axis.IsNearlyZero())
			Axis = FVector::UpVector;

		// Activation ratio in [0, 1]: control against its range where limited,
		// else the force against its range, else a neutral half. Drives both the
		// bar length and its blue-to-red colour.
		double Norm = 0.5;
		const double Ctrl = Snap.Ctrl.IsValidIndex(A) ? Snap.Ctrl[A] : 0.0;
		if (M->actuator_ctrllimited && M->actuator_ctrllimited[A] && M->actuator_ctrlrange)
		{
			const double Lo = M->actuator_ctrlrange[A * 2 + 0];
			const double Hi = M->actuator_ctrlrange[A * 2 + 1];
			if (Hi > Lo)
				Norm = FMath::Clamp((Ctrl - Lo) / (Hi - Lo), 0.0, 1.0);
		}
		else if (M->actuator_forcerange && Snap.ActuatorForce.IsValidIndex(A))
		{
			const double Lo = M->actuator_forcerange[A * 2 + 0];
			const double Hi = M->actuator_forcerange[A * 2 + 1];
			if (Hi > Lo)
				Norm = FMath::Clamp((Snap.ActuatorForce[A] - Lo) / (Hi - Lo), 0.0, 1.0);
		}

		const float LenCm = 3.0f + 12.0f * static_cast<float>(Norm);
		const FColor Colour = FLinearColor::LerpUsingHSV(
			FLinearColor(0.1f, 0.3f, 1.0f), FLinearColor(1.0f, 0.15f, 0.05f), static_cast<float>(Norm))
								  .ToFColor(false);
		DrawDebugLine(World, Anchor, Anchor + Axis * LenCm, Colour, false, -1, 0, 3.0f);
		DrawDebugPoint(World, Anchor, 6.0f, Colour, false, -1);
	}
}

void UMjOverlayRenderer::DrawTendons(const FMjRenderSnapshot& Snap) const
{
	// Needs the per-step wrap path (d->wrap_xpos et al). Empty until the engine's
	// PushRenderState fills WrapXPos / WrapObj / TenWrapAdr / TenWrapNum, so this is
	// a safe no-op meanwhile rather than reaching into live mjData off-thread.
	if (Snap.TenWrapAdr.Num() == 0 || Snap.WrapXPos.Num() == 0)
	{
		return;
	}
	UWorld* World = GetWorld();
	const mjModel* M = Model;
	const FColor Colour(255, 160, 64);
	for (int32 T = 0; T < static_cast<int32>(M->ntendon); ++T)
	{
		if (!GroupVisible(Flags.TendonGroup, M->tendon_group ? M->tendon_group[T] : 0))
		{
			continue;
		}
		if (!Snap.TenWrapAdr.IsValidIndex(T) || !Snap.TenWrapNum.IsValidIndex(T))
		{
			continue;
		}
		const int32 Adr = Snap.TenWrapAdr[T];
		const int32 Num = Snap.TenWrapNum[T];
		for (int32 j = Adr; j + 1 < Adr + Num; ++j)
		{
			// A pulley wrap point (-2) breaks the path; MuJoCo skips the segment
			// on either side of it.
			if (Snap.WrapObj.IsValidIndex(j) && Snap.WrapObj[j] == -2)
				continue;
			if (Snap.WrapObj.IsValidIndex(j + 1) && Snap.WrapObj[j + 1] == -2)
				continue;
			if (!Snap.WrapXPos.IsValidIndex((j + 1) * 3 + 2))
				continue;
			const FVector A = URLabAxisConv::MjPositionToUe(&Snap.WrapXPos[j * 3]) + SceneOrigin;
			const FVector B = URLabAxisConv::MjPositionToUe(&Snap.WrapXPos[(j + 1) * 3]) + SceneOrigin;
			DrawDebugLine(World, A, B, Colour, false, -1, 0, 2.0f);
		}
	}
}

void UMjOverlayRenderer::DrawRangefinders(const FMjRenderSnapshot& Snap) const
{
	UWorld* World = GetWorld();
	const mjModel* M = Model;
	for (int32 S = 0; S < static_cast<int32>(M->nsensor); ++S)
	{
		if (M->sensor_type[S] != mjSENS_RANGEFINDER || M->sensor_objtype[S] != mjOBJ_SITE)
		{
			continue;
		}
		const int32 Sid = M->sensor_objid[S];
		if (Sid < 0 || !Snap.SiteXPos.IsValidIndex(Sid * 3 + 2) || !Snap.SiteXMat.IsValidIndex(Sid * 9 + 8))
		{
			continue;
		}
		// A rangefinder casts along its site's local +z; the reading is the metric
		// distance to the first geom, or negative when nothing is in range.
		const int32 Adr = M->sensor_adr[S];
		const double Dist = Snap.SensorData.IsValidIndex(Adr) ? Snap.SensorData[Adr] : -1.0;

		const mjtNum* Op = &Snap.SiteXPos[Sid * 3];
		mjtNum Zaxis[3];
		MatColumn(&Snap.SiteXMat[Sid * 9], 2, Zaxis);
		const FVector Origin = URLabAxisConv::MjPositionToUe(Op) + SceneOrigin;

		if (Dist >= 0.0)
		{
			mjtNum Hit[3];
			for (int32 k = 0; k < 3; ++k)
				Hit[k] = Op[k] + Zaxis[k] * Dist;
			const FVector End = URLabAxisConv::MjPositionToUe(Hit) + SceneOrigin;
			DrawDebugLine(World, Origin, End, FColor(64, 255, 64), false, -1, 0, 1.0f);
			DrawDebugPoint(World, End, 8.0f, FColor(64, 255, 64), false, -1);
		}
		else
		{
			// No hit: a short faded ray in the cast direction so the sensor still reads.
			const FVector Dir = URLabAxisConv::MjDirectionToUe(Zaxis).GetSafeNormal();
			DrawDebugLine(World, Origin, Origin + Dir * 10.0f, FColor(64, 128, 64), false, -1, 0, 1.0f);
		}
	}
}

void UMjOverlayRenderer::DrawConstraints(const FMjRenderSnapshot& Snap) const
{
	UWorld* World = GetWorld();
	const mjModel* M = Model;
	const FColor Colour(255, 64, 255);
	for (int32 E = 0; E < static_cast<int32>(M->neq); ++E)
	{
		const int32 Type = M->eq_type[E];
		if (Type != mjEQ_CONNECT && Type != mjEQ_WELD)
		{
			continue; // joint / tendon equalities have no single spatial marker
		}
		if (M->eq_objtype && M->eq_objtype[E] != mjOBJ_BODY)
		{
			continue;
		}
		const int32 B1 = M->eq_obj1id[E];
		const int32 B2 = M->eq_obj2id[E];

		// A body id < 0 means the world body; anchor it at the scene origin.
		auto BodyPoint = [&](int32 B, FVector& Out) -> bool {
			if (B <= 0)
			{
				Out = SceneOrigin;
				return true;
			}
			if (!Snap.XPos.IsValidIndex(B * 3 + 2))
				return false;
			Out = URLabAxisConv::MjPositionToUe(&Snap.XPos[B * 3]) + SceneOrigin;
			return true;
		};

		FVector P1, P2;
		if (!BodyPoint(B1, P1) || !BodyPoint(B2, P2))
			continue;
		DrawDebugLine(World, P1, P2, Colour, false, -1, 0, 1.5f);
		DrawDebugSphere(World, P1, 3.0f, 8, Colour, false, -1, 0, 1.0f);
		DrawDebugSphere(World, P2, 3.0f, 8, Colour, false, -1, 0, 1.0f);
	}
}

void UMjOverlayRenderer::DrawStaticBodies(const FMjRenderSnapshot& Snap) const
{
	UWorld* World = GetWorld();
	const mjModel* M = Model;
	for (int32 B = 1; B < static_cast<int32>(M->nbody); ++B)
	{
		if (M->body_dofnum && M->body_dofnum[B] != 0)
		{
			continue; // dynamic body: not a static marker
		}
		if (!Snap.XPos.IsValidIndex(B * 3 + 2))
		{
			continue;
		}
		const FVector Pos = URLabAxisConv::MjPositionToUe(&Snap.XPos[B * 3]) + SceneOrigin;
		DrawDebugBox(World, Pos, FVector(2.0f), FColor(160, 160, 160), false, -1, 0, 0.5f);
	}
}

void UMjOverlayRenderer::DrawAutoConnect(const FMjRenderSnapshot& Snap) const
{
	UWorld* World = GetWorld();
	const mjModel* M = Model;
	const FColor Colour(120, 120, 255);
	for (int32 B = 1; B < static_cast<int32>(M->nbody); ++B)
	{
		const int32 Parent = M->body_parentid ? M->body_parentid[B] : -1;
		if (Parent < 0 || !Snap.XPos.IsValidIndex(B * 3 + 2) || !Snap.XPos.IsValidIndex(Parent * 3 + 2))
		{
			continue;
		}
		const FVector Child = URLabAxisConv::MjPositionToUe(&Snap.XPos[B * 3]) + SceneOrigin;
		const FVector Par = URLabAxisConv::MjPositionToUe(&Snap.XPos[Parent * 3]) + SceneOrigin;
		DrawDebugLine(World, Par, Child, Colour, false, -1, 0, 1.0f);
	}
}

void UMjOverlayRenderer::DrawGeomBounds(const FMjRenderSnapshot& Snap) const
{
	// The overlay half of mjVIS_TRANSPARENT: a bounding marker so a dynamic geom
	// stays locatable even when the material path makes it see-through. True geom
	// transparency is a material property, owned by the renderer, not drawn here.
	UWorld* World = GetWorld();
	const mjModel* M = Model;
	const FColor Colour(200, 200, 255);
	for (int32 G = 0; G < static_cast<int32>(M->ngeom); ++G)
	{
		const int32 Bid = M->geom_bodyid ? M->geom_bodyid[G] : -1;
		if (Bid < 0 || (M->body_dofnum && M->body_dofnum[Bid] == 0))
		{
			continue; // only dynamic geoms, matching simulate's transparent set
		}
		if (!GroupVisible(Flags.GeomGroup, M->geom_group[G]))
		{
			continue;
		}
		if (!Snap.GeomXPos.IsValidIndex(G * 3 + 2))
		{
			continue;
		}
		const double RBound = M->geom_rbound ? M->geom_rbound[G] : 0.0;
		if (RBound <= 0.0)
		{
			continue;
		}
		const FVector Pos = URLabAxisConv::MjPositionToUe(&Snap.GeomXPos[G * 3]) + SceneOrigin;
		DrawDebugSphere(World, Pos, static_cast<float>(RBound * 100.0), 10, Colour, false, -1, 0, 0.3f);
	}
}
