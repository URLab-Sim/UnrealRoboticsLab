// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Misc/AutomationTest.h"

#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Editor.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "HAL/FileManager.h"
#include "ImageUtils.h"
#include "Materials/MaterialRenderProxy.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "MaterialShared.h"
#include "Misc/App.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/ScopeExit.h"
#include "RenderingThread.h"
#include "Render/RenderCameraComponent.h"
#include "Render/RenderCameraRig.h"
#include "Render/RenderWorldSubsystem.h"
#include "Render/URLabRenderScene.h"

#if WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS
namespace URLabNeutralRenderWorldGpuTest
{
constexpr int32 CaptureWidth = 256;
constexpr int32 CaptureHeight = 192;
const TCHAR* const SceneRevision = TEXT("neutral-production-gpu-scene");

FRenderFrame MakeFrame(uint64 FrameId, double SourceTime, double PrimitiveY,
	double CameraY)
{
	FRenderFrame Frame;
	Frame.Identity.SceneRevision = SceneRevision;
	Frame.Identity.FrameId = FrameId;
	Frame.Identity.SourceTime = SourceTime;
	Frame.Identity.ClockEpoch = 9;
	Frame.Identity.bHasSourceTime = true;
	Frame.BodySceneTransforms = {
		FTransform(FVector(300.0, PrimitiveY, 0.0)),
		FTransform(FVector(300.0, 80.0, 0.0)),
	};
	Frame.CameraSceneTransforms = {FTransform(FVector(0.0, CameraY, 0.0))};
	return Frame;
}

void MakeScene(FRenderScene& OutScene, FRenderSceneResources& OutResources)
{
	OutScene.SceneRevision = SceneRevision;
	OutResources.SceneRevision = SceneRevision;
	OutScene.Bodies.AddDefaulted_GetRef().Name = TEXT("moving_primitive");
	OutScene.Bodies.AddDefaulted_GetRef().Name = TEXT("owned_mesh");

	FRenderMaterialResource& PrimitiveMaterial =
		OutResources.Materials.AddDefaulted_GetRef();
	PrimitiveMaterial.Emission = 3.0f;
	PrimitiveMaterial.Roughness = 1.0f;

	FRenderGeomDesc& Primitive = OutScene.Geoms.AddDefaulted_GetRef();
	Primitive.BodyIndex = 0;
	Primitive.Kind = ERenderGeomKind::Box;
	Primitive.SizeCm = FVector(40.0, 40.0, 40.0);
	Primitive.Name = TEXT("red_emissive_box");
	FRenderSurfaceBinding& PrimitiveSurface =
		OutResources.Geoms.AddDefaulted_GetRef();
	PrimitiveSurface.MaterialIndex = 0;
	PrimitiveSurface.Rgba = FLinearColor(1.0f, 0.01f, 0.01f, 1.0f);

	FRenderMeshResource& Mesh = OutResources.Meshes.AddDefaulted_GetRef();
	Mesh.Positions = {
		FVector(0.0, -40.0, -40.0), FVector(0.0, 40.0, -40.0),
		FVector(0.0, 40.0, 40.0), FVector(0.0, -40.0, 40.0),
	};
	Mesh.Normals.Init(FVector(-1.0, 0.0, 0.0), 4);
	Mesh.UVs = {
		FVector2D(0.0, 1.0), FVector2D(1.0, 1.0),
		FVector2D(1.0, 0.0), FVector2D(0.0, 0.0),
	};
	Mesh.Triangles = {0, 1, 2, 0, 2, 3};

	FRenderGeomDesc& OwnedMesh = OutScene.Geoms.AddDefaulted_GetRef();
	OwnedMesh.BodyIndex = 1;
	OwnedMesh.Kind = ERenderGeomKind::Mesh;
	OwnedMesh.MeshResourceIndex = 0;
	OwnedMesh.Name = TEXT("green_owned_mesh");
	FRenderMaterialResource& MeshMaterial = OutResources.Materials.AddDefaulted_GetRef();
	MeshMaterial.Rgba = FLinearColor(0.01f, 1.0f, 0.01f, 1.0f);
	MeshMaterial.Emission = 1.0f;
	MeshMaterial.Roughness = 1.0f;
	FRenderSurfaceBinding& MeshSurface = OutResources.Geoms.AddDefaulted_GetRef();
	MeshSurface.MaterialIndex = 0;
	MeshSurface.Rgba = FLinearColor::White;

	FRenderCameraDesc& Camera = OutScene.Cameras.AddDefaulted_GetRef();
	Camera.CanonicalName = TEXT("neutral_content_camera");
	Camera.Optics.Resolution = FIntPoint(CaptureWidth, CaptureHeight);
	Camera.Optics.Projection = ERenderCameraProjection::Orthographic;
	Camera.Optics.OrthographicWidthCm = 400.0f;
	Camera.Optics.NearClipCm = 2.0f;
	Camera.Optics.FarClipCm = 1000.0f;
	Camera.Capture.Mode = ERenderCameraMode::Real;
	Camera.Capture.bManualCaptureOnly = true;
	Camera.Capture.HistoryCapacity = 4;
}

bool Capture(URenderCameraComponent& Camera, uint64 FrameId,
	FRenderCameraFrame& OutFrame)
{
	Camera.ClearHistory();
	FlushRenderingCommands();
	Camera.IssueSyncCapture();
	FlushRenderingCommands();
	Camera.WaitAndHarvestReadbacks(2.0);
	return Camera.GetFrame(FrameId, OutFrame);
}

struct FMaterialFallbackProbe
{
	bool bHasDirectMaterial = false;
	bool bDirectShaderMapComplete = false;
	bool bUsedFallback = false;
};

FMaterialFallbackProbe ProbeMaterialFallback(UMaterialInterface& Material,
	ERHIFeatureLevel::Type FeatureLevel)
{
	FMaterialFallbackProbe Result;
	FMaterialRenderProxy* RenderProxy = Material.GetRenderProxy();
	ENQUEUE_RENDER_COMMAND(URLabProbeNeutralGpuMaterialFallback)(
		[RenderProxy, FeatureLevel, &Result](FRHICommandListImmediate&)
		{
			const FMaterial* Direct = RenderProxy->GetMaterialNoFallback(FeatureLevel);
			Result.bHasDirectMaterial = Direct != nullptr;
			Result.bDirectShaderMapComplete = Direct
				&& Direct->IsRenderingThreadShaderMapComplete();
			const FMaterialRenderProxy* ResolvedProxy = RenderProxy;
			RenderProxy->GetMaterialWithFallback(FeatureLevel, ResolvedProxy);
			Result.bUsedFallback = ResolvedProxy != RenderProxy;
		});
	FlushRenderingCommands();
	return Result;
}

struct FColorMetrics
{
	int32 RedPixels = 0;
	int32 GreenPixels = 0;
	double GreenXSum = 0.0;

	double GreenCentroidX() const
	{
		return GreenPixels > 0 ? GreenXSum / GreenPixels : -1.0;
	}
};

FColorMetrics Measure(const TArray<FColor>& Pixels)
{
	FColorMetrics Result;
	for (int32 Index = 0; Index < Pixels.Num(); ++Index)
	{
		const FColor& Pixel = Pixels[Index];
		if (Pixel.R > 80 && Pixel.R > Pixel.G + 30 && Pixel.R > Pixel.B + 30)
		{
			++Result.RedPixels;
		}
		if (Pixel.G > 80 && Pixel.G > Pixel.R + 30 && Pixel.G > Pixel.B + 30)
		{
			++Result.GreenPixels;
			Result.GreenXSum += Index % CaptureWidth;
		}
	}
	return Result;
}

void RecordCaptureDiagnostics(FAutomationTestBase& Test, const TCHAR* Label,
	const FRenderCameraFrame& Frame)
{
	uint8 MinR = 255, MinG = 255, MinB = 255;
	uint8 MaxR = 0, MaxG = 0, MaxB = 0;
	uint64 SumR = 0, SumG = 0, SumB = 0;
	int32 NonBlack = 0, Chromatic = 0, NearWhite = 0;
	for (const FColor& Pixel : Frame.Color)
	{
		MinR = FMath::Min(MinR, Pixel.R); MaxR = FMath::Max(MaxR, Pixel.R);
		MinG = FMath::Min(MinG, Pixel.G); MaxG = FMath::Max(MaxG, Pixel.G);
		MinB = FMath::Min(MinB, Pixel.B); MaxB = FMath::Max(MaxB, Pixel.B);
		SumR += Pixel.R; SumG += Pixel.G; SumB += Pixel.B;
		const uint8 High = FMath::Max3(Pixel.R, Pixel.G, Pixel.B);
		const uint8 Low = FMath::Min3(Pixel.R, Pixel.G, Pixel.B);
		NonBlack += High > 10 ? 1 : 0;
		Chromatic += High - Low > 30 ? 1 : 0;
		NearWhite += Low > 200 ? 1 : 0;
	}
	const double Divisor = FMath::Max(1, Frame.Color.Num());
	Test.AddInfo(FString::Printf(
		TEXT("%s RGB: R[%u..%u mean=%.2f] G[%u..%u mean=%.2f] "
			"B[%u..%u mean=%.2f] nonblack=%d chromatic=%d nearwhite=%d"),
		Label, MinR, MaxR, SumR / Divisor, MinG, MaxG, SumG / Divisor,
		MinB, MaxB, SumB / Divisor, NonBlack, Chromatic, NearWhite));

	const FString Directory = FPaths::Combine(FPaths::ProjectSavedDir(),
		TEXT("Automation"), TEXT("URLab"), TEXT("NeutralProductionGPU"));
	IFileManager::Get().MakeDirectory(*Directory, true);
	const FString Path = FPaths::Combine(Directory,
		FString::Printf(TEXT("%s.png"), Label));
	TArray64<uint8> Png;
	FImageUtils::PNGCompressImageArray(Frame.Width, Frame.Height,
		TArrayView64<const FColor>(Frame.Color.GetData(), Frame.Color.Num()), Png);
	if (Png.Num() == 0 || !FFileHelper::SaveArrayToFile(Png, *Path))
	{
		Test.AddError(FString::Printf(TEXT("failed to write diagnostic capture %s"), *Path));
	}
	else
	{
		Test.AddInfo(FString::Printf(TEXT("saved lossless diagnostic capture: %s"), *Path));
	}
}

int32 CountChangedPixels(const TArray<FColor>& A, const TArray<FColor>& B)
{
	if (A.Num() != B.Num()) return 0;
	int32 Changed = 0;
	for (int32 Index = 0; Index < A.Num(); ++Index)
	{
		const int32 Difference = FMath::Abs(int32(A[Index].R) - int32(B[Index].R))
			+ FMath::Abs(int32(A[Index].G) - int32(B[Index].G))
			+ FMath::Abs(int32(A[Index].B) - int32(B[Index].B));
		if (Difference > 45) ++Changed;
	}
	return Changed;
}

bool HasIdentity(const FRenderCameraFrame& Frame, uint64 FrameId, double SourceTime)
{
	return Frame.Identity.SceneRevision == SceneRevision
		&& Frame.Identity.FrameId == FrameId
		&& Frame.Identity.ClockEpoch == 9
		&& Frame.Identity.bHasSourceTime
		&& FMath::IsNearlyEqual(Frame.Identity.SourceTime, SourceTime, 1e-9);
}
} // namespace URLabNeutralRenderWorldGpuTest

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabNeutralRenderWorldSceneContentGpuTest,
	"URLab.Render.NeutralProductionGPU.SceneContentAndMotion",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FURLabNeutralRenderWorldSceneContentGpuTest::RunTest(const FString&)
{
	using namespace URLabNeutralRenderWorldGpuTest;
	if (!FApp::CanEverRender())
	{
		AddInfo(TEXT("Skipped GPU pixel assertions: RHI rendering is unavailable; run with "
			"-RenderOffScreen and without -NullRHI"));
		return true;
	}
	UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
	if (!TestNotNull(TEXT("rendering editor world"), World)) return false;
	UURLabRenderWorldSubsystem* Coordinator =
		World->GetSubsystem<UURLabRenderWorldSubsystem>();
	if (!TestNotNull(TEXT("production render world coordinator"), Coordinator)) return false;

	FURLabRenderProducerRegistration Registration;
	URenderCameraComponent* Camera = nullptr;
	ON_SCOPE_EXIT
	{
		if (Camera && IsValid(Camera)) Camera->SetStreamingEnabled(false);
		if (Coordinator && Coordinator->IsRegistrationCurrent(Registration))
		{
			FString CleanupError;
			Coordinator->ReleaseProducer(Registration, CleanupError);
		}
	};

	FString Error;
	if (!TestTrue(TEXT("neutral producer registers"),
		Coordinator->RegisterProducer(Registration, Error)))
	{
		AddError(Error);
		return false;
	}
	FRenderScene Scene;
	FRenderSceneResources Resources;
	MakeScene(Scene, Resources);
	const FRenderFrame InitialFrame = MakeFrame(101, 1.0, -80.0, 0.0);
	FRenderSceneInstallOptions Options;
	Options.bForceRebuildAssets = true;
	FURLabRenderSceneCandidate Candidate;
	if (!TestTrue(TEXT("owned neutral scene stages through coordinator"),
		Coordinator->StageScene(Registration, TEXT(""), FRenderSceneInstallPayload(MoveTemp(Scene), MoveTemp(Resources), FRenderFrame(InitialFrame)), Options, Candidate, Error)))
	{
		AddError(Error);
		return false;
	}
	const FURLabRenderWorldCommitResult Commit =
		Coordinator->CommitCandidate(Registration, Candidate);
	if (!TestTrue(TEXT("owned neutral scene commits"), Commit.bCommitted))
	{
		AddError(Commit.Error);
		return false;
	}
	TestEqual(TEXT("committed scene revision"), Commit.SceneRevision,
		FString(SceneRevision));
	AURLabRenderScene* Active = Coordinator->ResolveActiveScene(Registration);
	if (!TestNotNull(TEXT("coordinator exposes production scene"), Active)) return false;
	TestEqual(TEXT("primitive and mesh build"), Active->NumBuiltGeoms(), 2);
	UMaterialInstanceDynamic* PrimitiveMid = Cast<UMaterialInstanceDynamic>(
		Active->GetGeomComponent(0) ? Active->GetGeomComponent(0)->GetMaterial(0) : nullptr);
	UMaterialInstanceDynamic* MeshMid = Cast<UMaterialInstanceDynamic>(
		Active->GetGeomComponent(1) ? Active->GetGeomComponent(1)->GetMaterial(0) : nullptr);
	FLinearColor PrimitiveColor = FLinearColor::Black;
	FLinearColor MeshColor = FLinearColor::Black;
	const bool bHasPrimitiveColor = PrimitiveMid
		&& PrimitiveMid->GetVectorParameterValue(
			FMaterialParameterInfo(TEXT("BaseColor")), PrimitiveColor);
	const bool bHasMeshColor = MeshMid
		&& MeshMid->GetVectorParameterValue(
			FMaterialParameterInfo(TEXT("BaseColor")), MeshColor);
	TestTrue(TEXT("primitive uses the production dynamic material"), bHasPrimitiveColor);
	TestTrue(TEXT("mesh uses the resolved production dynamic material"), bHasMeshColor);
	TestTrue(TEXT("primitive MID retains the red resource color"),
		bHasPrimitiveColor && PrimitiveColor.Equals(
			FLinearColor(1.0f, 0.01f, 0.01f, 1.0f), 0.001f));
	TestTrue(TEXT("mesh MID retains the green compiled material"),
		bHasMeshColor && MeshColor.Equals(
			FLinearColor(0.01f, 1.0f, 0.01f, 1.0f), 0.001f));
	if (PrimitiveMid && MeshMid)
	{
		const UMaterialInterface* PrimitiveParent = PrimitiveMid->Parent;
		const UMaterialInterface* MeshParent = MeshMid->Parent;
		AddInfo(FString::Printf(
			TEXT("render material readiness: primitive_mid=%s compiling=%s parent=%s "
				"parent_compiling=%s mesh_mid=%s compiling=%s parent=%s parent_compiling=%s"),
			*PrimitiveMid->GetPathName(), PrimitiveMid->IsCompiling() ? TEXT("true") : TEXT("false"),
			PrimitiveParent ? *PrimitiveParent->GetPathName() : TEXT("<null>"),
			PrimitiveParent && PrimitiveParent->IsCompiling() ? TEXT("true") : TEXT("false"),
			*MeshMid->GetPathName(), MeshMid->IsCompiling() ? TEXT("true") : TEXT("false"),
			MeshParent ? *MeshParent->GetPathName() : TEXT("<null>"),
			MeshParent && MeshParent->IsCompiling() ? TEXT("true") : TEXT("false")));
	}
	if (const UStaticMeshComponent* MeshComponent = Cast<UStaticMeshComponent>(
		Active->GetGeomComponent(1)))
	{
		const UStaticMesh* StaticMesh = MeshComponent->GetStaticMesh();
		AddInfo(FString::Printf(TEXT("owned mesh readiness: asset=%s compiling=%s"),
			StaticMesh ? *StaticMesh->GetPathName() : TEXT("<null>"),
			StaticMesh && StaticMesh->IsCompiling() ? TEXT("true") : TEXT("false")));
	}
	UMaterialInterface* Master = PrimitiveMid ? PrimitiveMid->Parent.Get() : nullptr;
	if (!TestNotNull(TEXT("production master material"), Master)) return false;
	const ERHIFeatureLevel::Type FeatureLevel = World->GetFeatureLevel();
	const EShaderPlatform ShaderPlatform = GetFeatureLevelShaderPlatform_Checked(FeatureLevel);
	const FMaterialResource* MasterResource = Master->GetMaterialResource(ShaderPlatform);
	const FMaterialFallbackProbe Staged = ProbeMaterialFallback(*PrimitiveMid, FeatureLevel);
	AddInfo(FString::Printf(
		TEXT("after production staging: gt_resource=%s gt_complete=%s rt_direct=%s "
			"rt_complete=%s rt_used_fallback=%s"),
		MasterResource ? TEXT("true") : TEXT("false"),
		MasterResource && MasterResource->IsGameThreadShaderMapComplete()
			? TEXT("true") : TEXT("false"),
		Staged.bHasDirectMaterial ? TEXT("true") : TEXT("false"),
		Staged.bDirectShaderMapComplete ? TEXT("true") : TEXT("false"),
		Staged.bUsedFallback ? TEXT("true") : TEXT("false")));
	TestTrue(TEXT("production staging prepares the active feature-level shader map"),
		MasterResource && MasterResource->IsGameThreadShaderMapComplete());
	TestTrue(TEXT("production staging leaves a complete direct render material"),
		Staged.bHasDirectMaterial && Staged.bDirectShaderMapComplete);
	TestFalse(TEXT("production staging does not use the default fallback material"),
		Staged.bUsedFallback);
	Camera = Active->GetCameraRig() ? Active->GetCameraRig()->GetCamera(0) : nullptr;
	if (!TestNotNull(TEXT("neutral scene camera"), Camera)) return false;
	Camera->SetStreamingEnabled(true);

	FRenderCameraFrame InitialCapture;
	if (!TestTrue(TEXT("initial RGB capture completes within bound"),
		Capture(*Camera, 101, InitialCapture))) return false;
	TestEqual(TEXT("initial capture width"), InitialCapture.Width, CaptureWidth);
	TestEqual(TEXT("initial capture height"), InitialCapture.Height, CaptureHeight);
	TestEqual(TEXT("initial capture pixel count"), InitialCapture.Color.Num(),
		CaptureWidth * CaptureHeight);
	TestTrue(TEXT("initial capture retains exact frame identity"),
		HasIdentity(InitialCapture, 101, 1.0));
	RecordCaptureDiagnostics(*this, TEXT("initial"), InitialCapture);
	const FColorMetrics InitialMetrics = Measure(InitialCapture.Color);
	AddInfo(FString::Printf(TEXT("initial pixels: red=%d green=%d green_centroid_x=%.2f"),
		InitialMetrics.RedPixels, InitialMetrics.GreenPixels,
		InitialMetrics.GreenCentroidX()));
	TestTrue(TEXT("emissive primitive produces a visible red region"),
		InitialMetrics.RedPixels > 150);
	TestTrue(TEXT("owned mesh material produces a visible green region"),
		InitialMetrics.GreenPixels > 150);

	const FRenderFrame ObjectMoved = MakeFrame(102, 2.0, -320.0, 0.0);
	if (!TestTrue(TEXT("coordinator applies object movement"), Coordinator->ApplyFrame(
		Registration, SceneRevision, ObjectMoved, Error)))
	{
		AddError(Error);
		return false;
	}
	FRenderCameraFrame ObjectMovedCapture;
	if (!TestTrue(TEXT("object-moved RGB capture completes within bound"),
		Capture(*Camera, 102, ObjectMovedCapture))) return false;
	TestTrue(TEXT("object-moved capture retains exact frame identity"),
		HasIdentity(ObjectMovedCapture, 102, 2.0));
	RecordCaptureDiagnostics(*this, TEXT("object_moved"), ObjectMovedCapture);
	const FColorMetrics ObjectMovedMetrics = Measure(ObjectMovedCapture.Color);
	AddInfo(FString::Printf(TEXT("object-moved pixels: red=%d green=%d green_centroid_x=%.2f"),
		ObjectMovedMetrics.RedPixels, ObjectMovedMetrics.GreenPixels,
		ObjectMovedMetrics.GreenCentroidX()));
	TestTrue(TEXT("moving the primitive out of view removes most red pixels"),
		ObjectMovedMetrics.RedPixels * 4 < InitialMetrics.RedPixels);
	TestTrue(TEXT("object movement visibly changes captured content"),
		CountChangedPixels(InitialCapture.Color, ObjectMovedCapture.Color) > 300);
	TestTrue(TEXT("stationary owned mesh remains visible after object movement"),
		ObjectMovedMetrics.GreenPixels > 150);

	const FRenderFrame CameraMoved = MakeFrame(103, 3.0, -320.0, 80.0);
	if (!TestTrue(TEXT("coordinator applies camera movement"), Coordinator->ApplyFrame(
		Registration, SceneRevision, CameraMoved, Error)))
	{
		AddError(Error);
		return false;
	}
	FRenderCameraFrame CameraMovedCapture;
	if (!TestTrue(TEXT("camera-moved RGB capture completes within bound"),
		Capture(*Camera, 103, CameraMovedCapture))) return false;
	TestTrue(TEXT("camera-moved capture retains exact frame identity"),
		HasIdentity(CameraMovedCapture, 103, 3.0));
	RecordCaptureDiagnostics(*this, TEXT("camera_moved"), CameraMovedCapture);
	const FColorMetrics CameraMovedMetrics = Measure(CameraMovedCapture.Color);
	AddInfo(FString::Printf(TEXT("camera-moved pixels: red=%d green=%d green_centroid_x=%.2f"),
		CameraMovedMetrics.RedPixels, CameraMovedMetrics.GreenPixels,
		CameraMovedMetrics.GreenCentroidX()));
	TestTrue(TEXT("owned mesh remains visible from moved camera"),
		CameraMovedMetrics.GreenPixels > 150);
	TestTrue(TEXT("camera translation moves the green region across the image"),
		FMath::Abs(CameraMovedMetrics.GreenCentroidX()
			- ObjectMovedMetrics.GreenCentroidX()) > 20.0);
	TestTrue(TEXT("camera movement visibly changes captured content"),
		CountChangedPixels(ObjectMovedCapture.Color, CameraMovedCapture.Color) > 300);
	TestEqual(TEXT("active scene records final frame id"),
		Active->GetAppliedFrame().Identity.FrameId, uint64(103));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabNeutralDynamicGpuTest,
	"URLab.Render.NeutralProductionGPU.DeformableAndDebugPixels",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FURLabNeutralDynamicGpuTest::RunTest(const FString&)
{
	using namespace URLabNeutralRenderWorldGpuTest;
	if (!FApp::CanEverRender()) return true;
	UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
	if (!TestNotNull(TEXT("GPU world"), World)) return false;
	UURLabRenderWorldSubsystem* Coordinator = World->GetSubsystem<UURLabRenderWorldSubsystem>();
	if (!TestNotNull(TEXT("render coordinator"), Coordinator)) return false;
	FURLabRenderProducerRegistration Registration;
	FString Error;
	if (!TestTrue(TEXT("register producer"), Coordinator->RegisterProducer(Registration, Error)))
	{ AddError(Error); return false; }
	ON_SCOPE_EXIT
	{
		if (Coordinator->IsRegistrationCurrent(Registration))
		{
			FString Ignored;
			Coordinator->ReleaseProducer(Registration, Ignored);
		}
	};
	FRenderScene Scene;
	Scene.SceneRevision = TEXT("neutral-dynamic-gpu");
	Scene.Environment.bSkyboxEnabled = true;
	Scene.Environment.SkyTop = FLinearColor(0.01f, 0.03f, 0.5f, 1.0f);
	Scene.Environment.SkyBottom = FLinearColor(0.01f, 0.03f, 0.5f, 1.0f);
	FRenderSceneResources Resources;
	Resources.SceneRevision = Scene.SceneRevision;
	FRenderMaterialResource& Material = Resources.Materials.AddDefaulted_GetRef();
	Material.Emission = 4.0f;
	Material.Roughness = 1.0f;
	FRenderSurfaceBinding& SurfaceBinding = Resources.Deformables.AddDefaulted_GetRef();
	SurfaceBinding.MaterialIndex = 0;
	SurfaceBinding.Rgba = FLinearColor(0.05f, 0.8f, 0.1f, 1.0f);
	FRenderDeformableDesc& Surface = Scene.Deformables.AddDefaulted_GetRef();
	Surface.Name = TEXT("external_cloth");
	Surface.VertexCount = 3;
	Surface.SurfaceIndex = 0;
	Surface.Triangles = {0, 1, 2, 0, 2, 1};
	FRenderCameraDesc& Camera = Scene.Cameras.AddDefaulted_GetRef();
	Camera.CanonicalName = TEXT("dynamic_camera");
	Camera.Optics.Resolution = FIntPoint(CaptureWidth, CaptureHeight);
	Camera.Optics.Projection = ERenderCameraProjection::Orthographic;
	Camera.Optics.OrthographicWidthCm = 400.0f;
	Camera.Optics.NearClipCm = 2.0f;
	Camera.Optics.FarClipCm = 1000.0f;
	Camera.Capture.Mode = ERenderCameraMode::Real;
	Camera.Capture.bManualCaptureOnly = true;
	FRenderFrame Frame;
	Frame.Identity.SceneRevision = Scene.SceneRevision;
	Frame.Identity.FrameId = 1;
	Frame.CameraSceneTransforms.Add(FTransform::Identity);
	FRenderFrame::FDeformable& Vertices = Frame.Deformables.AddDefaulted_GetRef();
	Vertices.Name = Surface.Name;
	Vertices.Positions = {FVector(300, -45, -45), FVector(300, 45, -45),
		FVector(300, 0, 45)};
	FRenderSceneInstallOptions Options;
	FURLabRenderSceneCandidate Candidate;
	if (!TestTrue(TEXT("stage dynamic GPU scene"), Coordinator->StageScene(Registration,
		TEXT(""), FRenderSceneInstallPayload(MoveTemp(Scene), MoveTemp(Resources),
			FRenderFrame(Frame)), Options, Candidate, Error)))
	{ AddError(Error); return false; }
	const FURLabRenderWorldCommitResult Commit = Coordinator->CommitCandidate(Registration, Candidate);
	if (!TestTrue(TEXT("commit dynamic GPU scene"), Commit.bCommitted))
	{ AddError(Commit.Error); return false; }
	AURLabRenderScene* Actor = Coordinator->ResolveActiveScene(Registration);
	if (!TestNotNull(TEXT("active scene"), Actor)) return false;
	URenderCameraComponent* CaptureCamera = Actor->GetCameraRig()
		? Actor->GetCameraRig()->GetCamera(0) : nullptr;
	if (!TestNotNull(TEXT("capture camera"), CaptureCamera)) return false;
	CaptureCamera->SetStreamingEnabled(true);
	FRenderCameraFrame First;
	if (!TestTrue(TEXT("capture initial cloth"), Capture(*CaptureCamera, 1, First)))
		return false;
	if (First.Color.Num() == CaptureWidth * CaptureHeight)
	{
		const FColor Corner = First.Color[0];
		TestTrue(TEXT("neutral sky colors the captured background"),
			Corner.B > Corner.R + 15 && Corner.B > Corner.G + 15);
	}
	Frame.Identity.FrameId = 2;
	Frame.Deformables[0].Positions[2].Y += 100.0;
	if (!TestTrue(TEXT("apply deformed frame"), Actor->ApplyFrame(Frame, Error)))
	{ AddError(Error); return false; }
	FRenderCameraFrame Second;
	if (!TestTrue(TEXT("capture changed cloth"), Capture(*CaptureCamera, 2, Second)))
		return false;
	TestTrue(TEXT("deformation changes RGB pixels"),
		CountChangedPixels(First.Color, Second.Color) > 100);
	Frame.Identity.FrameId = 3;
	FRenderFrame::FDebugMesh& Marker = Frame.DebugMeshes.AddDefaulted_GetRef();
	Marker.Name = TEXT("external_debug_marker");
	Marker.Color = FLinearColor::Red;
	Marker.Positions = {FVector(250, -30, -30), FVector(250, 30, -30),
		FVector(250, 0, 30)};
	Marker.Triangles = {0, 1, 2, 0, 2, 1};
	if (!TestTrue(TEXT("apply generic debug drawable"), Actor->ApplyFrame(Frame, Error)))
	{ AddError(Error); return false; }
	FRenderCameraFrame Third;
	if (!TestTrue(TEXT("capture debug drawable"), Capture(*CaptureCamera, 3, Third)))
		return false;
	RecordCaptureDiagnostics(*this, TEXT("dynamic_debug"), Third);
	const FColorMetrics WithDebug = Measure(Third.Color);
	const FColorMetrics WithoutDebug = Measure(Second.Color);
	AddInfo(FString::Printf(TEXT("dynamic debug pixels: red before=%d after=%d"),
		WithoutDebug.RedPixels, WithDebug.RedPixels));
	TestTrue(TEXT("generic debug mesh contributes red RGB pixels"),
		WithDebug.RedPixels > WithoutDebug.RedPixels + 25);
	return !HasAnyErrors();
}

#endif
