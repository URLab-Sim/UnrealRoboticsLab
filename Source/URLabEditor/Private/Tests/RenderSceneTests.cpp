// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Misc/AutomationTest.h"
#include "Editor.h"
#include "Engine/World.h"
#include "Components/PrimitiveComponent.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Render/URLabRenderScene.h"
#include "ProceduralMeshComponent.h"
#include <limits>

#if WITH_EDITOR
namespace RenderSceneTest
{
void MakeTwoDrawableScene(const FString& Revision, FRenderScene& OutScene,
	FRenderSceneResources& OutResources, FRenderFrame& OutFrame)
{
	OutScene.SceneRevision = Revision;
	OutScene.Bodies.AddDefaulted_GetRef().Name = TEXT("body");
	FRenderMeshResource& Mesh = OutResources.Meshes.AddDefaulted_GetRef();
	Mesh.Positions = {FVector::ZeroVector, FVector(10.0, 0.0, 0.0), FVector(0.0, 10.0, 0.0)};
	Mesh.Normals = {FVector::UpVector, FVector::UpVector, FVector::UpVector};
	Mesh.UVs = {FVector2D::ZeroVector, FVector2D(1.0, 0.0), FVector2D(0.0, 1.0)};
	Mesh.Triangles = {0, 1, 2};
	for (int32 Index = 0; Index < 2; ++Index)
	{
		FRenderGeomDesc& Geom = OutScene.Geoms.AddDefaulted_GetRef();
		Geom.BodyIndex = 0;
		Geom.Kind = Index == 0 ? ERenderGeomKind::Box : ERenderGeomKind::Mesh;
		Geom.SizeCm = FVector(5.0, 5.0, 5.0);
		Geom.MeshResourceIndex = Index == 1 ? 0 : INDEX_NONE;
		Geom.BodyLocalTransform = FTransform(FVector(Index * 20.0, 0.0, 0.0));
		Geom.Name = FString::Printf(TEXT("box_%d"), Index);
		FRenderSurfaceBinding& Surface = OutResources.Geoms.AddDefaulted_GetRef();
		Surface.Rgba = Index == 0 ? FLinearColor::Red : FLinearColor::Green;
	}
	OutResources.SceneRevision = Revision;
	OutFrame.Identity.SceneRevision = Revision;
	OutFrame.BodySceneTransforms.Add(FTransform(FVector(100.0, 0.0, 0.0)));
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNeutralRigidSceneIsTransactional,
	"URLab.Render.NeutralRigidScene_TransactionalInstallAndFrame",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FNeutralRigidSceneIsTransactional::RunTest(const FString& Parameters)
{
	UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
	if (!TestNotNull(TEXT("editor world"), World)) return false;
	AURLabRenderScene* SceneActor = World->SpawnActor<AURLabRenderScene>();
	if (!TestNotNull(TEXT("neutral scene actor"), SceneActor)) return false;

	FRenderScene Scene;
	FRenderSceneResources Resources;
	FRenderFrame Frame;
	RenderSceneTest::MakeTwoDrawableScene(TEXT("neutral-rigid-a"), Scene, Resources, Frame);
	FString Error;
	FRenderSceneInstallOptions Options;
	Options.bPersistentEditorActors = true;
	if (!TestTrue(TEXT("valid neutral scene installs"), SceneActor->InstallScene(
		FRenderSceneInstallPayload(MoveTemp(Scene), MoveTemp(Resources), FRenderFrame(Frame)), Options, Error)))
	{
		AddError(Error);
		SceneActor->Destroy();
		return false;
	}
	TestEqual(TEXT("one component per geom"), SceneActor->NumBuiltGeoms(), 2);
	TestTrue(TEXT("distinct appearances retain distinct component identity"),
		SceneActor->GetGeomComponent(0) != SceneActor->GetGeomComponent(1));
	TestTrue(TEXT("distinct appearances retain distinct material instances"),
		SceneActor->GetGeomComponent(0)->GetMaterial(0)
			!= SceneActor->GetGeomComponent(1)->GetMaterial(0));
	UMaterialInstanceDynamic* ResourceMeshMaterial = Cast<UMaterialInstanceDynamic>(
		SceneActor->GetGeomComponent(1)->GetMaterial(0));
	FLinearColor AppliedBaseColor;
	TestTrue(TEXT("resource mesh receives compiled MuJoCo material"), ResourceMeshMaterial
		&& ResourceMeshMaterial->GetVectorParameterValue(
			FMaterialParameterInfo(TEXT("BaseColor")), AppliedBaseColor)
		&& AppliedBaseColor.Equals(FLinearColor::Green));
	AActor* LiveBody = SceneActor->GetBodyActor(0);
	if (!TestNotNull(TEXT("installed body actor"), LiveBody))
	{
		SceneActor->Destroy();
		return false;
	}
	TestTrue(TEXT("initial body frame applied"),
		LiveBody->GetRootComponent()->GetRelativeLocation().Equals(FVector(100.0, 0.0, 0.0)));

	FRenderFrame InvalidFrame = Frame;
	InvalidFrame.Identity.SceneRevision = TEXT("wrong-revision");
	InvalidFrame.BodySceneTransforms[0].SetTranslation(FVector(999.0, 0.0, 0.0));
	TestFalse(TEXT("wrong-revision frame rejected"), SceneActor->ApplyFrame(InvalidFrame, Error));
	TestTrue(TEXT("failed frame preserves live transform"),
		LiveBody->GetRootComponent()->GetRelativeLocation().Equals(FVector(100.0, 0.0, 0.0)));

	FRenderScene BadScene;
	FRenderSceneResources BadResources;
	FRenderFrame BadFrame;
	RenderSceneTest::MakeTwoDrawableScene(TEXT("neutral-rigid-b"), BadScene, BadResources, BadFrame);
	BadResources.Geoms[0].MaterialIndex = 12;
	TestFalse(TEXT("invalid replacement rejected"), SceneActor->InstallScene(
		FRenderSceneInstallPayload(MoveTemp(BadScene), MoveTemp(BadResources), FRenderFrame(BadFrame)), Options, Error));
	TestTrue(TEXT("failed replacement preserves live tree"),
		SceneActor->GetBodyActor(0) == LiveBody && SceneActor->NumBuiltGeoms() == 2);

	FRenderScene SameContentScene;
	FRenderSceneResources SameContentResources;
	FRenderFrame NewGenerationFrame;
	RenderSceneTest::MakeTwoDrawableScene(TEXT("neutral-rigid-c"), SameContentScene,
		SameContentResources, NewGenerationFrame);
	TestTrue(TEXT("same content installs with a fresh revision"), SceneActor->InstallScene(
		FRenderSceneInstallPayload(MoveTemp(SameContentScene), MoveTemp(SameContentResources), FRenderFrame(NewGenerationFrame)),
		Options, Error));
	TestFalse(TEXT("stale frame from prior same-content install is rejected"),
		SceneActor->ApplyFrame(Frame, Error));
	LiveBody = SceneActor->GetBodyActor(0);

	SceneActor->Destroy();
	TestTrue(TEXT("destroying neutral owner destroys its body tree"),
		!IsValid(LiveBody) || LiveBody->IsActorBeingDestroyed());
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNeutralDynamicSceneFrameTest,
	"URLab.Render.NeutralDynamicScene_FrameAndBounds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FNeutralDynamicSceneFrameTest::RunTest(const FString&)
{
	UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
	if (!TestNotNull(TEXT("editor world"), World)) return false;
	AURLabRenderScene* Actor = World->SpawnActor<AURLabRenderScene>();
	if (!TestNotNull(TEXT("render scene actor"), Actor)) return false;
	FRenderScene Scene;
	Scene.SceneRevision = TEXT("neutral-dynamic-a");
	FRenderDeformableDesc& Surface = Scene.Deformables.AddDefaulted_GetRef();
	Surface.Name = TEXT("cloth"); Surface.VertexCount = 3;
	Surface.Triangles = {0, 1, 2}; Surface.SurfaceIndex = 0;
	Surface.UVs = {FVector2D(0.125, 0.25), FVector2D(0.875, 0.25), FVector2D(0.125, 0.75)};
	FRenderSceneResources Resources;
	Resources.SceneRevision = Scene.SceneRevision;
	Resources.Deformables.AddDefaulted();
	FRenderFrame Initial;
	Initial.Identity.SceneRevision = Scene.SceneRevision;
	FRenderFrame::FDeformable& Dynamic = Initial.Deformables.AddDefaulted_GetRef();
	Dynamic.Name = Surface.Name;
	Dynamic.Positions = {FVector(0, 0, 0), FVector(10, 0, 0), FVector(0, 10, 0)};
	Dynamic.Normals = {FVector::UpVector, FVector::UpVector, FVector::UpVector};
	FString Error;
	FRenderSceneInstallOptions Options;
	FRenderScene BadUVScene = Scene;
	BadUVScene.Deformables[0].UVs.RemoveAt(2);
	TestFalse(TEXT("partial deformable UV array is rejected"), BadUVScene.IsValid(Resources, Options.VisibleGroupMask, Error));
	BadUVScene = Scene;
	BadUVScene.Deformables[0].UVs[1].X = std::numeric_limits<double>::quiet_NaN();
	TestFalse(TEXT("non-finite deformable UV is rejected"), BadUVScene.IsValid(Resources, Options.VisibleGroupMask, Error));
	if (!TestTrue(TEXT("neutral deformable scene installs"), Actor->InstallScene(
		FRenderSceneInstallPayload(MoveTemp(Scene), MoveTemp(Resources),
			FRenderFrame(Initial)), Options, Error)))
	{
		AddError(Error); Actor->Destroy(); return false;
	}
	UProceduralMeshComponent* Mesh = Actor->GetDeformableComponent(0);
	TestNotNull(TEXT("procedural deformable exists"), Mesh);
	if (Mesh && Mesh->GetProcMeshSection(0))
	{
		const auto& Vertices = Mesh->GetProcMeshSection(0)->ProcVertexBuffer;
		TestTrue(TEXT("immutable UV reaches initial procedural section"),
			Vertices[1].UV0.Equals(FVector2D(0.875, 0.25)));
		TestTrue(TEXT("UV basis gives initial +X tangent"),
			FVector(Vertices[0].Tangent.TangentX).Equals(FVector::ForwardVector, 1.e-4));
	}
	FRenderFrame Next = Initial;
	Next.Identity.FrameId = 1;
	Next.Deformables[0].Positions[1].Y = 5;
	Next.Deformables[0].Positions[2].Z = 15;
	FRenderFrame::FDebugMesh& Debug = Next.DebugMeshes.AddDefaulted_GetRef();
	Debug.Name = TEXT("point_marker"); Debug.Color = FLinearColor::Red;
	Debug.Positions = {FVector(0, 0, 20), FVector(2, 0, 20), FVector(0, 2, 20)};
	Debug.Triangles = {0, 1, 2};
	TestTrue(TEXT("dynamic frame applies"), Actor->ApplyFrame(Next, Error));
	TestEqual(TEXT("debug mesh count grows"), Actor->NumDebugMeshComponents(), 1);
	if (Mesh && Mesh->GetProcMeshSection(0))
	{
		TestTrue(TEXT("deformed vertex moves"),
			FVector(Mesh->GetProcMeshSection(0)->ProcVertexBuffer[2].Position)
				.Equals(FVector(0, 10, 15)));
		TestTrue(TEXT("supplied normals reach procedural mesh"),
			FVector(Mesh->GetProcMeshSection(0)->ProcVertexBuffer[2].Normal)
				.Equals(FVector::UpVector));
		TestTrue(TEXT("immutable UV survives deformation update"),
			Mesh->GetProcMeshSection(0)->ProcVertexBuffer[2].UV0.Equals(FVector2D(0.125, 0.75)));
		TestTrue(TEXT("deformation recomputes UV tangent"),
			FVector(Mesh->GetProcMeshSection(0)->ProcVertexBuffer[0].Tangent.TangentX)
				.Equals(FVector(10, 5, 0).GetSafeNormal(), 1.e-4));
	}
	FRenderFrame Oversized = Next;
	Oversized.DebugMeshes.SetNum(4097);
	TestFalse(TEXT("oversized dynamic mesh batch rejected"), Actor->ApplyFrame(Oversized, Error));
	TestEqual(TEXT("rejected batch retains components"), Actor->NumDebugMeshComponents(), 1);
	Actor->Destroy();
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNeutralRigidSceneAcceptsEmptyScene,
	"URLab.Render.NeutralRigidScene_EmptyScene",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FNeutralRigidSceneAcceptsEmptyScene::RunTest(const FString& Parameters)
{
	UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
	if (!TestNotNull(TEXT("editor world"), World)) return false;
	AURLabRenderScene* SceneActor = World->SpawnActor<AURLabRenderScene>();
	if (!TestNotNull(TEXT("neutral scene actor"), SceneActor)) return false;
	FRenderScene Scene;
	Scene.SceneRevision = TEXT("empty-scene");
	FRenderSceneResources Resources;
	Resources.SceneRevision = Scene.SceneRevision;
	FRenderFrame Frame;
	Frame.Identity.SceneRevision = Scene.SceneRevision;
	FRenderSceneInstallOptions Options;
	FString Error;
	TestTrue(TEXT("empty neutral scene installs without a synthetic body"),
		SceneActor->InstallScene(FRenderSceneInstallPayload(MoveTemp(Scene), MoveTemp(Resources), FRenderFrame(Frame)), Options, Error));
	TestEqual(TEXT("empty scene has no body actors"), SceneActor->NumBodyActors(), 0);
	TestEqual(TEXT("empty scene has no geom components"), SceneActor->NumBuiltGeoms(), 0);
	SceneActor->Destroy();
	return !HasAnyErrors();
}
#endif
