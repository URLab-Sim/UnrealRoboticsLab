// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjTestHelpers.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Authoring/MjAuthoringCompiler.h"
#include "MuJoCoSpec/MJModelComponent.h"
#include "MuJoCoSpec/MJSceneComponents.h"
#include "MuJoCoSpec/MJActorComponents.h"
#include "Engine/World.h"
#include "mujoco/mujoco.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileMinimalBodyTest, "URLab.Compile.MinimalBody",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjCompileMinimalBodyTest::RunTest(const FString&)
{
	UWorld* World = UWorld::CreateWorld(EWorldType::Editor, false);
	TestNotNull(TEXT("World created"), World);
	if (!World) return false;

	AAMjManager* Manager = World->SpawnActor<AAMjManager>();
	AMjArticulation* Art = World->SpawnActor<AMjArticulation>();
	UMJModelComponent* ModelComp = NewObject<UMJModelComponent>(Art, TEXT("MuJoCoModel"));
	ModelComp->RegisterComponent();
	Art->AddInstanceComponent(ModelComp);
	TestNotNull(TEXT("Manager spawned"), Manager);
	TestNotNull(TEXT("Articulation spawned"), Art);

	UMJBodyComponent* Body = NewObject<UMJBodyComponent>(Art, TEXT("Link1"));
	Body->ElementName = TEXT("link1");
	Body->AttachToComponent(Art->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
	Body->RegisterComponent();
	Art->AddInstanceComponent(Body);

	UMJGeomComponent* Geom = NewObject<UMJGeomComponent>(Art, TEXT("BoxGeom"));
	Geom->ElementName = TEXT("box_geom");
	Geom->Spec.bOverride_Type = true;
	Geom->Spec.Type = EMJGeom::Box;
	Geom->Spec.bOverride_Size = true;
	Geom->Spec.Size = {0.1, 0.1, 0.1};
	Geom->AttachToComponent(Body, FAttachmentTransformRules::KeepRelativeTransform);
	Geom->RegisterComponent();
	Art->AddInstanceComponent(Geom);

	UMJJointComponent* Joint = NewObject<UMJJointComponent>(Art, TEXT("HingeJoint"));
	Joint->ElementName = TEXT("hinge1");
	Joint->Spec.bOverride_Type = true;
	Joint->Spec.Type = EMJJoint::Hinge;
	Joint->Spec.bOverride_Limited = true;
	Joint->Spec.Limited = EMJLimited::MjTrue;
	Joint->Spec.bOverride_Range = true;
	Joint->Spec.Range = {-45.0, 45.0};
	Joint->AttachToComponent(Body, FAttachmentTransformRules::KeepRelativeTransform);
	Joint->RegisterComponent();
	Art->AddInstanceComponent(Joint);

	FMjAuthoredSimulationPackage Package = FMjAuthoringCompiler::Compile(*Manager);
	TestTrue(TEXT("Authoring compiler succeeded"), Package.IsValid());
	if (!Package.Error.IsEmpty())
	{
		AddError(Package.Error);
	}

	Package.Candidate.VisitPrepared([&](const mjModel* M, const mjData*) {
		TestTrue(TEXT("Body link1 compiled"), mj_name2id(M, mjOBJ_BODY, "link1") > 0);
		TestTrue(TEXT("Geom box_geom compiled"), mj_name2id(M, mjOBJ_GEOM, "box_geom") >= 0);
		const int JId = mj_name2id(M, mjOBJ_JOINT, "hinge1");
		TestTrue(TEXT("Joint hinge1 compiled"), JId >= 0);
		if (JId >= 0)
		{
			TestTrue(TEXT("Range converted to radians (~pi/4)"),
				FMath::IsNearlyEqual(M->jnt_range[2 * JId + 1], PI * 0.25, 1e-4));
		}
		return true;
	});

	World->DestroyWorld(false);
	return true;
}
