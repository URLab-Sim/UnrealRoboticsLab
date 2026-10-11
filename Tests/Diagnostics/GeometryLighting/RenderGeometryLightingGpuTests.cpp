// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Misc/AutomationTest.h"

#include "Components/PrimitiveComponent.h"
#include "Editor.h"
#include "Engine/World.h"
#include "HAL/FileManager.h"
#include "ImageUtils.h"
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
namespace URLabGeometryLightingGpuTest
{
constexpr int32 ImageSize = 192;
constexpr int32 MeshGeomIndex = 7;
constexpr int32 FlexIndex = 8;
const TCHAR* const Revision = TEXT("geometry-lighting-gpu-v1");

FRenderFrame MakeFrame(uint64 Id, int32 VisibleIndex, bool bLightFromCamera)
{
	FRenderFrame Frame;
	Frame.Identity.SceneRevision = Revision;
	Frame.Identity.FrameId = Id;
	Frame.BodySceneTransforms.Init(FTransform(FVector(300.0, 2000.0, 0.0)), 8);
	if (VisibleIndex >= 0 && VisibleIndex < 8)
	{
		Frame.BodySceneTransforms[VisibleIndex] = FTransform(FVector(300.0, 0.0, 0.0));
	}
	Frame.CameraSceneTransforms.Add(FTransform::Identity);
	Frame.LightSceneTransforms.Add(FTransform(
		FRotator(0.0, bLightFromCamera ? 0.0 : 180.0, 0.0), FVector::ZeroVector));
	FRenderFrame::FDeformable& Flex = Frame.Deformables.AddDefaulted_GetRef();
	Flex.Name = TEXT("front_facing_flex");
	const double Y = VisibleIndex == FlexIndex ? 0.0 : 2000.0;
	Flex.Positions = {
		FVector(300.0, Y - 30.0, -30.0), FVector(300.0, Y + 30.0, -30.0),
		FVector(300.0, Y + 30.0, 30.0), FVector(300.0, Y - 30.0, 30.0),
	};
	Flex.Normals.Init(-FVector::ForwardVector, 4);
	return Frame;
}

void MakeScene(FRenderScene& Scene, FRenderSceneResources& Resources)
{
	Scene.SceneRevision = Revision;
	Resources.SceneRevision = Revision;
	const ERenderGeomKind Kinds[] = {
		ERenderGeomKind::Plane, ERenderGeomKind::Box, ERenderGeomKind::Sphere,
		ERenderGeomKind::Capsule, ERenderGeomKind::Cylinder,
		ERenderGeomKind::Ellipsoid, ERenderGeomKind::Box, ERenderGeomKind::Mesh,
	};
	const TCHAR* Names[] = {
		TEXT("plane"), TEXT("box"), TEXT("sphere"), TEXT("capsule"),
		TEXT("cylinder"), TEXT("ellipsoid"), TEXT("shadow_box"), TEXT("regular_mesh"),
	};
	FRenderMaterialResource& Material = Resources.Materials.AddDefaulted_GetRef();
	Material.Emission = 0.0f;
	Material.Roughness = 1.0f;
	Material.Metallic = 0.0f;
	for (int32 Index = 0; Index < 8; ++Index)
	{
		Scene.Bodies.AddDefaulted_GetRef().Name = Names[Index];
		FRenderGeomDesc& Geom = Scene.Geoms.AddDefaulted_GetRef();
		Geom.BodyIndex = Index;
		Geom.Kind = Kinds[Index];
		Geom.Name = Names[Index];
		Geom.SizeCm = FVector(30.0, 30.0, 30.0);
		if (Index == 0)
		{
			Geom.SizeCm = FVector(120.0, 120.0, 0.0);
			Geom.BodyLocalTransform.SetRotation(FQuat::FindBetweenNormals(
				FVector::UpVector, -FVector::ForwardVector));
		}
		if (Index == 3 || Index == 4) Geom.SizeCm.Y = 35.0;
		if (Index == 5) Geom.SizeCm = FVector(30.0, 23.0, 36.0);
		if (Index == MeshGeomIndex) Geom.MeshResourceIndex = 0;
		FRenderSurfaceBinding& Binding = Resources.Geoms.AddDefaulted_GetRef();
		Binding.MaterialIndex = 0;
		Binding.Rgba = FLinearColor::White;
	}
	FRenderMeshResource& Mesh = Resources.Meshes.AddDefaulted_GetRef();
	Mesh.Positions = {
		FVector(0.0, -30.0, -30.0), FVector(0.0, 30.0, -30.0),
		FVector(0.0, 30.0, 30.0), FVector(0.0, -30.0, 30.0),
	};
	Mesh.Normals.Init(-FVector::ForwardVector, 4);
	Mesh.UVs = {
		FVector2D(0.0, 1.0), FVector2D(1.0, 1.0),
		FVector2D(1.0, 0.0), FVector2D(0.0, 0.0),
	};
	Mesh.Triangles = {0, 1, 2, 0, 2, 3};
	FRenderDeformableDesc& Flex = Scene.Deformables.AddDefaulted_GetRef();
	Flex.Name = TEXT("front_facing_flex");
	Flex.VertexCount = 4;
	Flex.SurfaceIndex = 0;
	Flex.Triangles = {0, 1, 2, 0, 2, 3};
	FRenderSurfaceBinding& FlexBinding = Resources.Deformables.AddDefaulted_GetRef();
	FlexBinding.MaterialIndex = 0;
	FlexBinding.Rgba = FLinearColor::White;
	FRenderLightDesc& Light = Scene.Lights.AddDefaulted_GetRef();
	Light.Name = TEXT("controlled_directional");
	Light.Kind = ERenderLightKind::Directional;
	Light.Intensity = 10000.0f;
	Light.Color = FLinearColor::White;
	Light.bCastShadows = true;
	Scene.Environment.AmbientIntensity = 0.0f;
	Scene.Environment.bHeadlightEnabled = false;
	Scene.Environment.bSkyboxEnabled = false;
	FRenderCameraDesc& Camera = Scene.Cameras.AddDefaulted_GetRef();
	Camera.CanonicalName = TEXT("facing_camera");
	Camera.Optics.Resolution = FIntPoint(ImageSize, ImageSize);
	Camera.Optics.Projection = ERenderCameraProjection::Orthographic;
	Camera.Optics.OrthographicWidthCm = 280.0f;
	Camera.Optics.NearClipCm = 2.0f;
	Camera.Optics.FarClipCm = 1000.0f;
	Camera.Capture.Mode = ERenderCameraMode::Real;
	Camera.Capture.bManualCaptureOnly = true;
	Camera.Capture.HistoryCapacity = 2;
}

bool Capture(URenderCameraComponent& Camera, uint64 Id, FRenderCameraFrame& Out)
{
	Camera.ClearHistory();
	FlushRenderingCommands();
	Camera.IssueSyncCapture();
	FlushRenderingCommands();
	Camera.WaitAndHarvestReadbacks(2.0);
	return Camera.GetFrame(Id, Out) && Out.Color.Num() == ImageSize * ImageSize;
}

double CenterLuminance(const FRenderCameraFrame& Frame)
{
	double Sum = 0.0;
	int32 Count = 0;
	for (int32 Y = ImageSize / 2 - 10; Y < ImageSize / 2 + 10; ++Y)
	{
		for (int32 X = ImageSize / 2 - 10; X < ImageSize / 2 + 10; ++X)
		{
			const FColor& Pixel = Frame.Color[Y * ImageSize + X];
			Sum += (Pixel.R + Pixel.G + Pixel.B) / 3.0;
			++Count;
		}
	}
	return Sum / Count;
}

int32 ChangedPixels(const FRenderCameraFrame& A, const FRenderCameraFrame& B)
{
	int32 Count = 0;
	for (int32 Index = 0; Index < A.Color.Num(); ++Index)
	{
		const FColor& Left = A.Color[Index];
		const FColor& Right = B.Color[Index];
		if (FMath::Abs(int32(Left.R) - int32(Right.R))
			+ FMath::Abs(int32(Left.G) - int32(Right.G))
			+ FMath::Abs(int32(Left.B) - int32(Right.B)) > 30) ++Count;
	}
	return Count;
}

void SaveDiagnostic(const TCHAR* Label, const FRenderCameraFrame& Frame)
{
	const FString Directory = FPaths::Combine(FPaths::ProjectSavedDir(),
		TEXT("Automation"), TEXT("URLab"), TEXT("GeometryLightingGPU"));
	IFileManager::Get().MakeDirectory(*Directory, true);
	TArray64<uint8> Png;
	FImageUtils::PNGCompressImageArray(Frame.Width, Frame.Height,
		TArrayView64<const FColor>(Frame.Color.GetData(), Frame.Color.Num()), Png);
	if (Png.Num()) FFileHelper::SaveArrayToFile(Png,
		*FPaths::Combine(Directory, FString(Label) + TEXT(".png")));
}
} // namespace URLabGeometryLightingGpuTest

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabGeometryLightingGpuTest,
	"URLab.Render.GeometryGPU.DirectionalFacingAndShadow",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FURLabGeometryLightingGpuTest::RunTest(const FString&)
{
	using namespace URLabGeometryLightingGpuTest;
	if (!FApp::CanEverRender())
	{
		AddInfo(TEXT("GPU assertions require an RHI; run with -RenderOffScreen without -NullRHI"));
		return true;
	}
	UWorld* World = GEditor ? GEditor->GetEditorWorldContext().World() : nullptr;
	if (!TestNotNull(TEXT("rendering editor world"), World)) return false;
	UURLabRenderWorldSubsystem* Coordinator = World->GetSubsystem<UURLabRenderWorldSubsystem>();
	if (!TestNotNull(TEXT("production coordinator"), Coordinator)) return false;
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
	FRenderSceneResources Resources;
	MakeScene(Scene, Resources);
	FRenderSceneInstallOptions Options;
	Options.bForceRebuildAssets = true;
	FURLabRenderSceneCandidate Candidate;
	if (!TestTrue(TEXT("stage nonemissive geometry"), Coordinator->StageScene(
		Registration, TEXT(""), FRenderSceneInstallPayload(MoveTemp(Scene),
			MoveTemp(Resources), MakeFrame(1, 0, true)), Options, Candidate, Error)))
	{ AddError(Error); return false; }
	const FURLabRenderWorldCommitResult Commit = Coordinator->CommitCandidate(Registration, Candidate);
	if (!TestTrue(TEXT("commit geometry scene"), Commit.bCommitted))
	{ AddError(Commit.Error); return false; }
	AURLabRenderScene* Active = Coordinator->ResolveActiveScene(Registration);
	if (!TestNotNull(TEXT("active geometry scene"), Active)) return false;
	URenderCameraComponent* Camera = Active->GetCameraRig()
		? Active->GetCameraRig()->GetCamera(0) : nullptr;
	if (!TestNotNull(TEXT("capture camera"), Camera)) return false;
	Camera->SetStreamingEnabled(true);
	if (Camera->CaptureComponent)
	{
		Camera->CaptureComponent->ShowFlags.SetEyeAdaptation(false);
		Camera->CaptureComponent->PostProcessSettings.bOverride_AutoExposureMethod = true;
		Camera->CaptureComponent->PostProcessSettings.AutoExposureMethod =
			EAutoExposureMethod::AEM_Manual;
		Camera->CaptureComponent->PostProcessSettings.bOverride_AutoExposureBias = true;
		Camera->CaptureComponent->PostProcessSettings.AutoExposureBias = 0.0f;
	}
	uint64 FrameId = 1;
	const TCHAR* Labels[] = {
		TEXT("plane"), TEXT("box"), TEXT("sphere"), TEXT("capsule"),
		TEXT("cylinder"), TEXT("ellipsoid"), TEXT("second box"),
		TEXT("regular mesh"), TEXT("deformable face"),
	};
	for (int32 Index = 0; Index <= FlexIndex; ++Index)
	{
		FRenderCameraFrame Front;
		if (Index != 0)
		{
			++FrameId;
			if (!TestTrue(TEXT("apply front-lit frame"), Active->ApplyFrame(
				MakeFrame(FrameId, Index, true), Error)))
			{ AddError(Error); return false; }
		}
		if (!TestTrue(TEXT("capture front-lit geometry"), Capture(*Camera, FrameId, Front)))
			return false;
		++FrameId;
		if (!TestTrue(TEXT("apply back-lit frame"), Active->ApplyFrame(
			MakeFrame(FrameId, Index, false), Error)))
		{ AddError(Error); return false; }
		FRenderCameraFrame Back;
		if (!TestTrue(TEXT("capture back-lit geometry"), Capture(*Camera, FrameId, Back)))
			return false;
		const double FrontValue = CenterLuminance(Front);
		const double BackValue = CenterLuminance(Back);
		AddInfo(FString::Printf(TEXT("%s facing light: %.1f; opposite light: %.1f"),
			Labels[Index], FrontValue, BackValue));
		UE_LOG(LogTemp, Display, TEXT("GeometryGPU %s facing light %.1f opposite %.1f"),
			Labels[Index], FrontValue, BackValue);
		if (Index == 0 || Index == MeshGeomIndex || Index == FlexIndex)
		{
			SaveDiagnostic(*FString::Printf(TEXT("%s_front"), Labels[Index]), Front);
			SaveDiagnostic(*FString::Printf(TEXT("%s_back"), Labels[Index]), Back);
		}
		TestTrue(FString::Printf(TEXT("%s front face responds to directional light"),
			Labels[Index]), FrontValue > BackValue + 12.0);
	}
	// Hold both images' geometry and light fixed. Only the box's cast-shadow flag
	// changes, so changed ground pixels demonstrate an actual projected shadow.
	FRenderFrame ShadowFrame = MakeFrame(++FrameId, -1, true);
	ShadowFrame.BodySceneTransforms[0] = FTransform(FVector(360.0, 0.0, 0.0));
	ShadowFrame.BodySceneTransforms[1] = FTransform(FVector(300.0, 0.0, 0.0));
	ShadowFrame.LightSceneTransforms[0] = FTransform(FRotator(0.0, 25.0, 0.0));
	if (!TestTrue(TEXT("apply shadow scene"), Active->ApplyFrame(ShadowFrame, Error)))
	{ AddError(Error); return false; }
	UPrimitiveComponent* Box = Active->GetGeomComponent(1);
	if (!TestNotNull(TEXT("shadow-casting box"), Box)) return false;
	Box->SetCastShadow(true);
	FRenderCameraFrame WithShadow;
	if (!TestTrue(TEXT("capture cast shadow"), Capture(*Camera, FrameId, WithShadow)))
		return false;
	Box->SetCastShadow(false);
	ShadowFrame.Identity.FrameId = ++FrameId;
	if (!TestTrue(TEXT("apply shadow-disabled frame"), Active->ApplyFrame(ShadowFrame, Error)))
	{ AddError(Error); return false; }
	FRenderCameraFrame WithoutShadow;
	if (!TestTrue(TEXT("capture without cast shadow"), Capture(*Camera, FrameId, WithoutShadow)))
		return false;
	const int32 ShadowPixels = ChangedPixels(WithShadow, WithoutShadow);
	AddInfo(FString::Printf(TEXT("ground pixels changed by box cast shadow: %d"), ShadowPixels));
	UE_LOG(LogTemp, Display, TEXT("GeometryGPU ground pixels changed by box shadow: %d"),
		ShadowPixels);
	SaveDiagnostic(TEXT("with_shadow"), WithShadow);
	SaveDiagnostic(TEXT("without_shadow"), WithoutShadow);
	TestTrue(TEXT("box casts a visible directional shadow onto plane"), ShadowPixels > 50);
	return !HasAnyErrors();
}
#endif
