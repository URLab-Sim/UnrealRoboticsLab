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

#include "MJSpecVisualizers.h"

#include "Editor/UnrealEdEngine.h"
#include "MuJoCoSpec/MJSceneComponents.h"
#include "SceneManagement.h"
#include "UnrealEdGlobals.h"

namespace
{
void DrawLocalRgbAxes(FPrimitiveDrawInterface* PDI, const FTransform& Transform, float AxisLengthCm,
	float Thickness = 1.5f)
{
	const FVector Origin = Transform.GetLocation();
	const FVector XAxis = Transform.GetUnitAxis(EAxis::X);
	const FVector YAxis = Transform.GetUnitAxis(EAxis::Y);
	const FVector ZAxis = Transform.GetUnitAxis(EAxis::Z);
	PDI->DrawLine(Origin, Origin + XAxis * AxisLengthCm, FLinearColor::Red, SDPG_Foreground, Thickness);
	PDI->DrawLine(Origin, Origin + YAxis * AxisLengthCm, FLinearColor::Green, SDPG_Foreground, Thickness);
	PDI->DrawLine(Origin, Origin + ZAxis * AxisLengthCm, FLinearColor::Blue, SDPG_Foreground, Thickness);
}
} // namespace

void FMJJointVisualizer::DrawVisualization(const UActorComponent* Component, const FSceneView* View,
	FPrimitiveDrawInterface* PDI)
{
	const UMJJointComponent* Joint = Cast<UMJJointComponent>(Component);
	if (!Joint || !PDI)
	{
		return;
	}

	const FTransform WorldTransform = Joint->GetComponentTransform();
	const FVector Origin = WorldTransform.GetLocation();
	const FVector LocalAxis = Joint->Spec.Axis.IsNearlyZero()
		? FVector::UpVector
		: Joint->Spec.Axis.GetSafeNormal();
	const FVector WorldAxis = WorldTransform.TransformVectorNoScale(LocalAxis).GetSafeNormal();

	const FMatrix ArrowMatrix = FRotationMatrix::MakeFromX(WorldAxis) * FTranslationMatrix(Origin);
	const FLinearColor JointColor(1.0f, 0.75f, 0.1f, 1.0f);
	DrawDirectionalArrow(PDI, ArrowMatrix, JointColor, 20.0f, 3.0f, SDPG_Foreground, 2.0f);

	const EMJJoint JointType = Joint->Spec.bOverride_Type ? Joint->Spec.Type : EMJJoint::Hinge;
	if (JointType == EMJJoint::Hinge)
	{
		FVector PerpA, PerpB;
		WorldAxis.FindBestAxisVectors(PerpA, PerpB);
		DrawCircle(PDI, Origin, PerpA, PerpB, JointColor, 8.0f, 24, SDPG_Foreground, 1.5f);
	}
	else if (JointType == EMJJoint::Slide)
	{
		const FVector BackEnd = Origin - WorldAxis * 12.0f;
		PDI->DrawLine(BackEnd, Origin, JointColor, SDPG_Foreground, 2.0f);
	}
	else if (JointType == EMJJoint::Ball)
	{
		DrawWireSphere(PDI, Origin, JointColor, 6.0f, 16, SDPG_Foreground, 1.5f);
	}
}

void FMJSiteVisualizer::DrawVisualization(const UActorComponent* Component, const FSceneView* View,
	FPrimitiveDrawInterface* PDI)
{
	const UMJSiteComponent* Site = Cast<UMJSiteComponent>(Component);
	if (!Site || !PDI)
	{
		return;
	}

	const FTransform WorldTransform = Site->GetComponentTransform();
	const FVector Origin = WorldTransform.GetLocation();
	const FVector XAxis = WorldTransform.GetUnitAxis(EAxis::X);
	const FVector YAxis = WorldTransform.GetUnitAxis(EAxis::Y);
	const FVector ZAxis = WorldTransform.GetUnitAxis(EAxis::Z);

	DrawLocalRgbAxes(PDI, WorldTransform, 12.0f, 1.5f);

	const FLinearColor SiteColor = Site->Spec.bOverride_Rgba
		? Site->Spec.Rgba
		: FLinearColor(0.2f, 0.9f, 0.9f, 1.0f);
	const TArray<double>& Size = Site->Spec.Size;
	const float R0 = Size.Num() > 0 && Size[0] > 0.0 ? static_cast<float>(Size[0]) : 5.0f;
	const float R1 = Size.Num() > 1 && Size[1] > 0.0 ? static_cast<float>(Size[1]) : R0;
	const float R2 = Size.Num() > 2 && Size[2] > 0.0 ? static_cast<float>(Size[2]) : R0;

	const EMJGeom SiteType = Site->Spec.bOverride_Type ? Site->Spec.Type : EMJGeom::Sphere;
	switch (SiteType)
	{
	case EMJGeom::Box:
		DrawWireBox(PDI, WorldTransform.ToMatrixNoScale(), FBox(FVector(-R0, -R1, -R2), FVector(R0, R1, R2)),
			SiteColor, SDPG_Foreground, 1.0f);
		break;
	case EMJGeom::Capsule:
		DrawWireCapsule(PDI, Origin, XAxis, YAxis, ZAxis, SiteColor, R0, R0 + R1, 16, SDPG_Foreground, 1.0f);
		break;
	case EMJGeom::Cylinder:
		DrawWireCylinder(PDI, Origin, XAxis, YAxis, ZAxis, SiteColor, R0, R1, 16, SDPG_Foreground, 1.0f);
		break;
	case EMJGeom::Sphere:
	default:
		DrawWireSphere(PDI, Origin, SiteColor, R0, 16, SDPG_Foreground, 1.0f);
		break;
	}
}

void FMJFrameVisualizer::DrawVisualization(const UActorComponent* Component, const FSceneView* View,
	FPrimitiveDrawInterface* PDI)
{
	const UMJFrameComponent* Frame = Cast<UMJFrameComponent>(Component);
	if (!Frame || !PDI)
	{
		return;
	}

	DrawLocalRgbAxes(PDI, Frame->GetComponentTransform(), 15.0f, 2.0f);
}

void MJSpecVisualizers::RegisterAll()
{
	if (!GUnrealEd)
	{
		return;
	}
	GUnrealEd->RegisterComponentVisualizer(
		UMJJointComponent::StaticClass()->GetFName(), MakeShared<FMJJointVisualizer>());
	GUnrealEd->RegisterComponentVisualizer(
		UMJSiteComponent::StaticClass()->GetFName(), MakeShared<FMJSiteVisualizer>());
	GUnrealEd->RegisterComponentVisualizer(
		UMJFrameComponent::StaticClass()->GetFName(), MakeShared<FMJFrameVisualizer>());
}

void MJSpecVisualizers::UnregisterAll()
{
	if (!GUnrealEd)
	{
		return;
	}
	GUnrealEd->UnregisterComponentVisualizer(UMJJointComponent::StaticClass()->GetFName());
	GUnrealEd->UnregisterComponentVisualizer(UMJSiteComponent::StaticClass()->GetFName());
	GUnrealEd->UnregisterComponentVisualizer(UMJFrameComponent::StaticClass()->GetFName());
}
