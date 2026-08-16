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
