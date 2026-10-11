// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"

#include "MuJoCo/Render/MjRenderScene.h"
#include "MuJoCo/Render/MjRenderSceneShim.h"
#include "Render/RenderScene.h"
#include "Render/RenderPrimitiveBuilder.h"
#include "Tests/MjTestHelpers.h"

namespace
{
struct FPrimitiveWorld
{
	UWorld* World = nullptr;
	AActor* Actor = nullptr;
	bool Open()
	{
		World = UWorld::CreateWorld(EWorldType::Editor, false);
		if (!World) return false;
		Actor = World->SpawnActor<AActor>();
		if (!Actor) return false;
		USceneComponent* Root = NewObject<USceneComponent>(Actor);
		Actor->SetRootComponent(Root);
		Root->RegisterComponent();
		return true;
	}
	~FPrimitiveWorld() { if (World) World->DestroyWorld(false); }
};

FRenderGeomDesc Geom(ERenderGeomKind Kind, double X, double Y, double Z)
{
	FRenderGeomDesc Result;
	Result.Kind = Kind;
	Result.SizeCm = FVector(X, Y, Z);
	return Result;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRenderPrimitiveComponents,
	"URLab.Render.PrimitiveBuilder.Components", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjRenderPrimitiveComponents::RunTest(const FString& Parameters)
{
	FPrimitiveWorld W;
	if (!TestTrue(TEXT("scratch world"), W.Open())) return false;
	int32 MaterialCalls = 0;
	auto Apply = [&MaterialCalls](UPrimitiveComponent*) { ++MaterialCalls; };
	UStaticMeshComponent* Box = Cast<UStaticMeshComponent>(BuildNeutralPrimitive(Geom(ERenderGeomKind::Box, 20, 30, 40), W.Actor, Apply));
	if (!TestNotNull(TEXT("box component"), Box)) return false;
	TestTrue(TEXT("box scale"), Box->GetRelativeScale3D().Equals(FVector(.4, .6, .8), KINDA_SMALL_NUMBER));
	UStaticMeshComponent* Capsule = Cast<UStaticMeshComponent>(BuildNeutralPrimitive(Geom(ERenderGeomKind::Capsule, 20, 50, 0), W.Actor, Apply));
	if (!TestNotNull(TEXT("capsule shaft"), Capsule)) return false;
	TestTrue(TEXT("capsule shaft scale"), Capsule->GetRelativeScale3D().Equals(FVector(.4, .4, 1.0), KINDA_SMALL_NUMBER));
	TArray<USceneComponent*> Children; Capsule->GetChildrenComponents(false, Children);
	TestEqual(TEXT("capsule has two caps"), Children.Num(), 2);
	for (USceneComponent* Child : Children)
	{
		TestTrue(TEXT("cap at shaft end"), FMath::IsNearlyEqual(FMath::Abs(Child->GetRelativeLocation().Z), 50.0));
		TestTrue(TEXT("cap cancels shaft Z scale"), FMath::IsNearlyEqual(Child->GetRelativeScale3D().Z, .4));
	}
	UStaticMeshComponent* ZeroCapsule = Cast<UStaticMeshComponent>(BuildNeutralPrimitive(Geom(ERenderGeomKind::Capsule, 20, 0, 0), W.Actor, Apply));
	if (!TestNotNull(TEXT("zero-length capsule sphere"), ZeroCapsule)) return false;
	TestTrue(TEXT("zero-length capsule has sphere scale"), ZeroCapsule->GetRelativeScale3D().Equals(FVector(.4), KINDA_SMALL_NUMBER));
	TArray<USceneComponent*> ZeroChildren; ZeroCapsule->GetChildrenComponents(false, ZeroChildren);
	TestEqual(TEXT("zero-length capsule has no caps"), ZeroChildren.Num(), 0);
	TestNull(TEXT("invalid sphere rejected"), BuildNeutralPrimitive(Geom(ERenderGeomKind::Sphere, 0, 0, 0), W.Actor, Apply));
	TestNull(TEXT("unsupported rejected"), BuildNeutralPrimitive(Geom(ERenderGeomKind::Unsupported, 100, 100, 100), W.Actor, Apply));
	TestEqual(TEXT("material callback all actual parts"), MaterialCalls, 5);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRenderSceneShimKinds,
	"URLab.Render.SceneShim.PrimitiveKinds", EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjRenderSceneShimKinds::RunTest(const FString& Parameters)
{
	FMjTestSession Session;
	if (!TestTrue(TEXT("compile primitive fixture"), Session.CompileXml(TEXT("<mujoco><worldbody><geom type='box' size='.2 .3 .4'/><geom type='capsule' size='.1 .5'/></worldbody></mujoco>")))) return false;
	FMjRenderScene Scene;
	MjRenderSceneShim::Fill(Scene, Session.m);
	TestEqual(TEXT("geom count"), Scene.Geoms.Num(), 2);
	TestEqual(TEXT("box kind"), Scene.Geoms[0].Kind, EMjSceneGeomKind::Box);
	TestEqual(TEXT("capsule kind"), Scene.Geoms[1].Kind, EMjSceneGeomKind::Capsule);
	TestEqual(TEXT("box X"), Scene.Geoms[0].Size[0], .2);
	TestEqual(TEXT("capsule half length"), Scene.Geoms[1].Size[1], .5);
	return true;
}
