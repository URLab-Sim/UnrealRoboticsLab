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

#include "MuJoCo/Utils/MjUtils.h"
#include "MuJoCo/Utils/URLabAxisConv.h"
#include "DrawDebugHelpers.h"

FString MjUtils::MjToString(const char* text)
{
	if (!text)
		return FString();
	return FString(UTF8_TO_TCHAR(text));
}

void MjUtils::StringToMj(const FString& text, char* buffer, int bufferSize)
{
	if (!buffer || bufferSize <= 0)
		return;

	FTCHARToUTF8 Converter(*text);
	int32 Length = Converter.Length();

	if (Length >= bufferSize)
	{
		// Truncate if too long (reserving space for null)
		FMemory::Memcpy(buffer, Converter.Get(), bufferSize - 1);
		buffer[bufferSize - 1] = '\0';
	}
	else
	{
		FMemory::Memcpy(buffer, Converter.Get(), Length);
		buffer[Length] = '\0';
	}
}

bool MjUtils::ParseFromTo(const FString& FromToStr, FVector& OutStart, FVector& OutEnd)
{
	if (FromToStr.IsEmpty())
		return false;

	TArray<FString> Parts;
	FromToStr.ParseIntoArray(Parts, TEXT(" "), true);

	if (Parts.Num() >= 6)
	{
		double Start[3] = {FCString::Atod(*Parts[0]), FCString::Atod(*Parts[1]), FCString::Atod(*Parts[2])};
		double End[3] = {FCString::Atod(*Parts[3]), FCString::Atod(*Parts[4]), FCString::Atod(*Parts[5])};

		OutStart = URLabAxisConv::MjPositionToUe(Start);
		OutEnd = URLabAxisConv::MjPositionToUe(End);
		return true;
	}
	return false;
}

void MjUtils::DrawDebugGeom(UWorld* World, const mjModel* m, const mjData* d, int32 GeomId,
	const FColor& DrawColor, float Multiplier)
{
	if (!World || !m || !d || GeomId < 0 || GeomId >= m->ngeom)
		return;

	const mjtNum* pos = &d->geom_xpos[GeomId * 3];
	const mjtNum* mat = &d->geom_xmat[GeomId * 9];
	const mjtNum* size = &m->geom_size[GeomId * 3];

	// Draw if group 3 (collision convention) OR if both contype and conaffinity are non-zero (active collider)
	int group = m->geom_group[GeomId];
	int contype = m->geom_contype[GeomId];
	int conaffinity = m->geom_conaffinity[GeomId];
	bool isCollisionGeom = (group == 3) || (contype != 0 && conaffinity != 0);
	if (!isCollisionGeom)
	{
		return;
	}

	FVector Position = URLabAxisConv::MjPositionToUe(pos);

	mjtNum _quat[4];
	mju_mat2Quat(_quat, mat);
	FQuat Rotation = URLabAxisConv::MjQuatToUe(_quat);

	switch (m->geom_type[GeomId])
	{
		case mjGEOM_BOX:
		{
			FColor Color = DrawColor != FColor::Magenta ? DrawColor : FColor::Cyan;
			FVector Extent(size[0] * Multiplier, size[1] * Multiplier, size[2] * Multiplier);
			DrawDebugBox(World, Position, Extent, Rotation, Color, false, -1, 0, 0.15f);
			break;
		}
		case mjGEOM_SPHERE:
		{
			FColor Color = DrawColor != FColor::Magenta ? DrawColor : FColor::Green;
			float Radius = size[0] * Multiplier;
			DrawDebugSphere(World, Position, Radius, 16, Color, false, -1, 0, 0.15f);
			break;
		}
		case mjGEOM_CAPSULE:
		{
			FColor Color = DrawColor != FColor::Magenta ? DrawColor : FColor::Yellow;
			float Radius = size[0] * Multiplier;
			float HalfHeight = size[1] * Multiplier;
			DrawDebugCapsule(World, Position, HalfHeight + Radius, Radius, Rotation, Color, false, -1, 0, 0.15f);
			break;
		}
		case mjGEOM_CYLINDER:
		{
			FColor Color = DrawColor != FColor::Magenta ? DrawColor : FColor::Orange;
			float Radius = size[0] * Multiplier;
			float HalfHeight = size[1] * Multiplier;
			FVector UpVector = Rotation.GetAxisZ();
			FVector Start = Position - UpVector * HalfHeight;
			FVector End = Position + UpVector * HalfHeight;
			DrawDebugCylinder(World, Start, End, Radius, 16, Color, false, -1, 0, 0.15f);
			break;
		}
		case mjGEOM_MESH:
		{
			FColor Color = DrawColor != FColor::Magenta ? DrawColor : FColor::Magenta;
			int meshId = m->geom_dataid[GeomId];
			if (meshId < 0)
				break;

			float* vertices = m->mesh_vert + m->mesh_vertadr[meshId] * 3;

			// Prefer convex hull data (what MuJoCo actually uses for collision)
			// over the full mesh triangles (which are the visual mesh)
			if (m->mesh_graphadr[meshId] != -1)
			{
				int graphStart = m->mesh_graphadr[meshId];
				int* graphData = m->mesh_graph + graphStart;
				int numVert = graphData[0];
				int numFace = graphData[1];
				int* edgeLocalId = &graphData[2 + numVert * 2];
				int* faceGlobalId = &edgeLocalId[numVert + 3 * numFace];

				for (int j = 0; j < numFace; j++)
				{
					int v1_idx = faceGlobalId[3 * j];
					int v2_idx = faceGlobalId[3 * j + 1];
					int v3_idx = faceGlobalId[3 * j + 2];

					FVector v1 = URLabAxisConv::MjPositionToUe(&vertices[3 * v1_idx]);
					FVector v2 = URLabAxisConv::MjPositionToUe(&vertices[3 * v2_idx]);
					FVector v3 = URLabAxisConv::MjPositionToUe(&vertices[3 * v3_idx]);

					v1 = Rotation.RotateVector(v1) + Position;
					v2 = Rotation.RotateVector(v2) + Position;
					v3 = Rotation.RotateVector(v3) + Position;

					DrawDebugLine(World, v1, v2, Color, false, -1, 0, 0.15f);
					DrawDebugLine(World, v2, v3, Color, false, -1, 0, 0.15f);
					DrawDebugLine(World, v3, v1, Color, false, -1, 0, 0.15f);
				}
			}
			else
			{
				// Fallback: no convex hull data, draw full mesh triangles
				int faceStart = m->mesh_faceadr[meshId];
				int numFace = m->mesh_facenum[meshId];
				int* faceData = m->mesh_face + faceStart * 3;

				for (int j = 0; j < numFace; j++)
				{
					int v1_idx = faceData[3 * j];
					int v2_idx = faceData[3 * j + 1];
					int v3_idx = faceData[3 * j + 2];

					FVector v1 = URLabAxisConv::MjPositionToUe(&vertices[3 * v1_idx]);
					FVector v2 = URLabAxisConv::MjPositionToUe(&vertices[3 * v2_idx]);
					FVector v3 = URLabAxisConv::MjPositionToUe(&vertices[3 * v3_idx]);

					v1 = Rotation.RotateVector(v1) + Position;
					v2 = Rotation.RotateVector(v2) + Position;
					v3 = Rotation.RotateVector(v3) + Position;

					DrawDebugLine(World, v1, v2, Color, false, -1, 0, 0.15f);
					DrawDebugLine(World, v2, v3, Color, false, -1, 0, 0.15f);
					DrawDebugLine(World, v3, v1, Color, false, -1, 0, 0.15f);
				}
			}
			break;
		}
	}
}

void MjUtils::DrawDebugJoint(UWorld* World, const FVector& anchor, const FVector& Axis,
	int JointType, bool bLimited, float RangeMin, float RangeMax,
	float CurrentPos, float RefPos, float ArcRadius)
{
	if (!World)
		return;

	const FColor ArcColor(40, 200, 40);      // Bright green arc outline
	const FColor FillColor(40, 180, 40, 50); // Semi-transparent green fill
	const FColor LimitColor(220, 60, 60);    // Red limit marks
	const FColor PosColor(255, 220, 30);     // Yellow current position
	const FColor RefColor(180, 180, 220);    // Light blue-grey reference position
	FVector AxisDir = Axis.GetSafeNormal();

	// Negate all rotation angles: MuJoCo uses right-hand rule but the
	// axis has been Y-negated for UE's left-hand coordinate system.
	const float Sign = -1.0f;

	if (JointType == mjJNT_HINGE)
	{
		// Find a perpendicular radial direction
		FVector RadialDir = FVector::CrossProduct(AxisDir, FVector::UpVector);
		if (RadialDir.SizeSquared() < 1e-6f)
		{
			RadialDir = FVector::CrossProduct(AxisDir, FVector::RightVector);
		}
		RadialDir.Normalize();

		if (bLimited && (RangeMin != 0.0f || RangeMax != 0.0f))
		{
			// Compute draw angles (swap min/max since we negate)
			float DrawMin = Sign * RangeMax;
			float DrawMax = Sign * RangeMin;
			const int NumSegments = 48;
			float Span = DrawMax - DrawMin;
			float Step = Span / NumSegments;

			// Build arc points for reuse
			TArray<FVector> ArcPoints;
			ArcPoints.Reserve(NumSegments + 1);
			for (int i = 0; i <= NumSegments; ++i)
			{
				float Theta = DrawMin + Step * i;
				ArcPoints.Add(anchor + FQuat(AxisDir, Theta).RotateVector(RadialDir) * ArcRadius);
			}

			// Filled arc (triangle fan from anchor)
			TArray<FVector> FillVerts;
			TArray<int32> FillIndices;
			FillVerts.Add(anchor); // index 0 = center
			for (int i = 0; i <= NumSegments; ++i)
			{
				FillVerts.Add(ArcPoints[i]);
			}
			for (int i = 0; i < NumSegments; ++i)
			{
				FillIndices.Add(0);
				FillIndices.Add(i + 1);
				FillIndices.Add(i + 2);
			}
			DrawDebugMesh(World, FillVerts, FillIndices, FillColor, false, -1, 0);

			// Arc outline
			for (int i = 0; i < NumSegments; ++i)
			{
				DrawDebugLine(World, ArcPoints[i], ArcPoints[i + 1], ArcColor, false, -1, 0, 1.5f);
			}

			// Limit lines from anchor to arc edges
			DrawDebugLine(World, anchor, ArcPoints[0], LimitColor, false, -1, 0, 1.5f);
			DrawDebugLine(World, anchor, ArcPoints.Last(), LimitColor, false, -1, 0, 1.5f);
		}

		// Reference position (qpos0) indicator
		if (!FMath::IsNaN(RefPos))
		{
			FVector RefDir = FQuat(AxisDir, Sign * RefPos).RotateVector(RadialDir);
			DrawDebugLine(World, anchor, anchor + RefDir * ArcRadius * 0.9f, RefColor, false, -1, 0, 1.0f);
		}

		// Current position indicator
		if (!FMath::IsNaN(CurrentPos))
		{
			FVector PosDir = FQuat(AxisDir, Sign * CurrentPos).RotateVector(RadialDir);
			DrawDebugLine(World, anchor, anchor + PosDir * ArcRadius * 1.15f, PosColor, false, -1, 0, 2.5f);
		}
	}
	else if (JointType == mjJNT_SLIDE)
	{
		// All slide values expected in cm (caller converts from meters if needed)
		if (bLimited && (RangeMin != 0.0f || RangeMax != 0.0f))
		{
			FVector MinPos = anchor + AxisDir * RangeMin;
			FVector MaxPos = anchor + AxisDir * RangeMax;

			// Perpendicular tick direction
			FVector TickDir = FVector::CrossProduct(AxisDir, FVector::UpVector);
			if (TickDir.SizeSquared() < 1e-6f)
				TickDir = FVector::CrossProduct(AxisDir, FVector::RightVector);
			TickDir.Normalize();
			float TickSize = 4.0f;

			// range bar
			DrawDebugLine(World, MinPos, MaxPos, ArcColor, false, -1, 0, 1.5f);

			// Tick marks at limits
			DrawDebugLine(World, MinPos - TickDir * TickSize, MinPos + TickDir * TickSize, LimitColor, false, -1, 0, 1.5f);
			DrawDebugLine(World, MaxPos - TickDir * TickSize, MaxPos + TickDir * TickSize, LimitColor, false, -1, 0, 1.5f);
		}

		// Reference position
		if (!FMath::IsNaN(RefPos))
		{
			FVector RefPoint = anchor + AxisDir * RefPos;
			DrawDebugPoint(World, RefPoint, 6.0f, RefColor, false, -1);
		}

		// Current position indicator
		if (!FMath::IsNaN(CurrentPos))
		{
			FVector PosPoint = anchor + AxisDir * CurrentPos;
			DrawDebugPoint(World, PosPoint, 8.0f, PosColor, false, -1);
		}
	}
}

FString MjUtils::PrettifyName(const FString& Name, const FString& PrefixToStrip)
{
	FString Result = Name;

	// 1. Strip the Articulation name prefix if provided
	if (!PrefixToStrip.IsEmpty() && Result.StartsWith(PrefixToStrip))
	{
		Result = Result.RightChop(PrefixToStrip.Len());
		if (Result.StartsWith(TEXT("_")))
			Result = Result.RightChop(1);
	}

	// 2. Strip UAID suffix: everything from the first _UAID_ (common in UE)
	int32 UaidIdx = Result.Find(TEXT("_UAID_"));
	if (UaidIdx != INDEX_NONE)
	{
		Result = Result.Left(UaidIdx);
	}

	// 3. Strip _C_ suffix (common for Blueprint classes)
	int32 CIdx = Result.Find(TEXT("_C_"), ESearchCase::CaseSensitive, ESearchDir::FromEnd);
	if (CIdx != INDEX_NONE && CIdx == Result.Len() - 3)
	{
		Result = Result.Left(CIdx);
	}

	// 4. Final cleanup of leading/trailing underscores
	Result = Result.TrimStartAndEnd().TrimChar('_');

	return Result.IsEmpty() ? Name : Result;
}
