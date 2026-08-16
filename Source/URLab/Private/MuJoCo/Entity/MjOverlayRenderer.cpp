// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.
// UnrealRoboticsLab is independent and not affiliated with Epic Games. See ThirdPartyNotices.txt.

#include "MuJoCo/Entity/MjOverlayRenderer.h"

#include "DrawDebugHelpers.h"
#include "Engine/World.h"

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

void UMjOverlayRenderer::SetModel(mjModel_* InModel, mjData_* InData)
{
	Model = InModel;
	Data = InData;
}

void UMjOverlayRenderer::DrawOverlays() const
{
	if (!Model || !Data || !GetWorld())
	{
		return;
	}
	if (FlagSet(Flags.VisFlags, mjVIS_CONVEXHULL))
	{
		DrawCollision();
	}
	if (FlagSet(Flags.VisFlags, mjVIS_JOINT))
	{
		DrawJoints();
	}
	DrawSites();
}

void UMjOverlayRenderer::DrawCollision() const
{
	UWorld* World = GetWorld();
	const mjModel* M = Model;
	for (int32 G = 0; G < static_cast<int32>(M->ngeom); ++G)
	{
		if (!GroupVisible(Flags.GeomGroup, M->geom_group[G]))
		{
			continue;
		}
		MjUtils::DrawDebugGeom(World, M, Data, G, FColor::Magenta, 100.0f);
	}
}

void UMjOverlayRenderer::DrawJoints() const
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

		const FVector Anchor = URLabAxisConv::MjPositionToUe(Data->xanchor + 3 * J) + SceneOrigin;
		const FVector Axis = URLabAxisConv::MjDirectionToUe(Data->xaxis + 3 * J);

		float RangeMin = static_cast<float>(M->jnt_range[J * 2 + 0]);
		float RangeMax = static_cast<float>(M->jnt_range[J * 2 + 1]);
		const bool bLimited = RangeMin != 0.0f || RangeMax != 0.0f;

		float CurrentPos = static_cast<float>(Data->qpos[M->jnt_qposadr[J]]);
		float RefPos = static_cast<float>(M->qpos0[M->jnt_qposadr[J]]);

		// MuJoCo stores a slide's travel in metres; the draw helper wants cm.
		if (Type == mjJNT_SLIDE)
		{
			RangeMin *= 100.0f;
			RangeMax *= 100.0f;
			CurrentPos *= 100.0f;
			RefPos *= 100.0f;
		}

		MjUtils::DrawDebugJoint(World, Anchor, Axis, Type, bLimited, RangeMin, RangeMax, CurrentPos, RefPos);
	}
}

void UMjOverlayRenderer::DrawSites() const
{
	UWorld* World = GetWorld();
	const mjModel* M = Model;
	for (int32 S = 0; S < static_cast<int32>(M->nsite); ++S)
	{
		if (!GroupVisible(Flags.SiteGroup, M->site_group[S]))
		{
			continue;
		}

		const FVector Pos = URLabAxisConv::MjPositionToUe(Data->site_xpos + 3 * S) + SceneOrigin;
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
