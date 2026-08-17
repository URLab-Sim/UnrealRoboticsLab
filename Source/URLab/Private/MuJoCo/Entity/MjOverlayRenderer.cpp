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
	if (bContactPoints || bContactForces)
	{
		DrawContacts(Snap, bContactPoints, bContactForces);
	}
	if (FlagSet(Flags.VisFlags, mjVIS_PERTFORCE) || FlagSet(Flags.VisFlags, mjVIS_PERTOBJ))
	{
		DrawPerturb(Snap);
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

void UMjOverlayRenderer::DrawContacts(const FMjRenderSnapshot& Snap, bool bPoints, bool bForces) const
{
	UWorld* World = GetWorld();
	for (const FMjContactViz& C : Snap.Contacts)
	{
		const FVector Pos = URLabAxisConv::MjPositionToUe(C.Pos) + SceneOrigin;
		if (bPoints)
		{
			DrawDebugPoint(World, Pos, 8.0f, FColor::Cyan, false, -1);
		}
		if (bForces)
		{
			// Rotate the contact-frame wrench into world: the frame's rows are its
			// world-space axes, so world_f = sum_i frame_row_i * force_i.
			mjtNum FW[3] = {0, 0, 0};
			for (int32 k = 0; k < 3; ++k)
			{
				FW[k] = C.Frame[0 * 3 + k] * C.Force[0] + C.Frame[1 * 3 + k] * C.Force[1]
						+ C.Frame[2 * 3 + k] * C.Force[2];
			}
			const FVector Dir = URLabAxisConv::MjDirectionToUe(FW);
			// Newtons -> cm at a fixed visual scale; clamp so a big impulse stays on screen.
			const float LenCm = FMath::Clamp(static_cast<float>(Dir.Size()) * 2.0f, 0.0f, 200.0f);
			if (LenCm > 0.5f)
			{
				const FVector End = Pos + Dir.GetSafeNormal() * LenCm;
				DrawDebugDirectionalArrow(World, Pos, End, 8.0f, FColor::Red, false, -1, 0, 1.0f);
			}
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
