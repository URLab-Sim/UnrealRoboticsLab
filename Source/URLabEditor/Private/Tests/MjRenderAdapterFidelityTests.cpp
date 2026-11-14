// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.

#include "Misc/AutomationTest.h"

#include "MuJoCo/Render/MjRenderKinematics.h"
#include "Utils/CoordinateConversion.h"

#if WITH_DEV_AUTOMATION_TESTS
namespace
{
const char* AdapterFidelityXml = R"(
<mujoco model="adapter_fidelity">
  <visual>
    <global fovy="60" offwidth="320" offheight="240"/>
    <map znear="0.02" zfar="40"/>
  </visual>
  <statistic extent="2"/>
  <asset>
    <material name="green" rgba="0 1 0 1"/>
    <mesh name="tet" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="1 1 1" rgb2="0 0 0"
      width="16" height="16"/>
    <texture name="checker" type="cube" builtin="checker" width="16" height="16"/>
  </asset>
  <worldbody>
    <geom name="override" type="box" size=".1 .1 .1" material="green" rgba="1 0 0 1"/>
    <geom name="inherit" type="box" size=".1 .1 .1" pos="1 0 0" material="green"/>
    <geom name="hidden" type="box" size=".1 .1 .1" pos="2 0 0" material="green" rgba="0 0 0 0"/>
    <geom name="sdf" type="sdf" mesh="tet" pos="3 0 0"/>
    <camera name="offset" pos="0 0 2" resolution="200 100" sensorsize="0.01 0.005"
      focal="0.01 0.01" principal="0.002 0.001"/>
    <body name="target" mocap="true" pos="0.5 0 1">
      <geom name="marker" type="sphere" size="0.05"/>
    </body>
    <body name="arm">
      <joint name="hinge" type="hinge" axis="0 0 1"/>
      <geom name="link" type="capsule" fromto="0 0 0 0.5 0 0" size="0.02"/>
      <camera name="tracker" mode="targetbody" target="target" pos="0.5 0 0.5"/>
    </body>
  </worldbody>
</mujoco>)";

bool PrepareAdapterFidelity(FMjRenderKinematicsCandidate& Out, FString& OutError)
{
	FMjModelSourcePayload Source;
	Source.Kind = EMjModelSourceKind::XmlWithAssets;
	Source.PrimaryBytes.Append(reinterpret_cast<const uint8*>(AdapterFidelityXml),
		FCStringAnsi::Strlen(AdapterFidelityXml));
	Source.Label = TEXT("adapter-fidelity");
	FMjRenderKinematicsOptions Options;
	Options.bIncludeUserCamera = true;
	return FMjRenderKinematicsCandidate::Prepare(MoveTemp(Source), TEXT("fidelity"),
		Options, Out, OutError);
}

int32 FindByName(const TArray<FRenderBodyDesc>& Bodies, const TCHAR* Name)
{
	return Bodies.IndexOfByPredicate([Name](const FRenderBodyDesc& Body) { return Body.Name == Name; });
}

int32 FindGeom(const TArray<FRenderGeomDesc>& Geoms, const TCHAR* Name)
{
	return Geoms.IndexOfByPredicate([Name](const FRenderGeomDesc& Geom) { return Geom.Name == Name; });
}

// GL cube lookup used by MuJoCo's skybox (direction -> face, column, row).
void GlCubeTexel(const FVector& D, int32 Size, int32& Face, int32& Col, int32& Row)
{
	const FVector A = D.GetAbs();
	double Sc, Tc, Ma;
	if (A.X >= A.Y && A.X >= A.Z)
	{
		Face = D.X > 0 ? 0 : 1; Ma = A.X;
		Sc = D.X > 0 ? -D.Z : D.Z; Tc = -D.Y;
	}
	else if (A.Y >= A.Z)
	{
		Face = D.Y > 0 ? 2 : 3; Ma = A.Y;
		Sc = D.X; Tc = D.Y > 0 ? D.Z : -D.Z;
	}
	else
	{
		Face = D.Z > 0 ? 4 : 5; Ma = A.Z;
		Sc = D.Z > 0 ? D.X : -D.X; Tc = -D.Y;
	}
	Col = FMath::Clamp(static_cast<int32>((Sc / Ma + 1.0) * 0.5 * Size), 0, Size - 1);
	Row = FMath::Clamp(static_cast<int32>((Tc / Ma + 1.0) * 0.5 * Size), 0, Size - 1);
}

// UE TextureCube face texel -> UE world direction (TextureCompressor convention).
FVector UnrealCubeDirection(int32 Face, int32 Col, int32 Row, int32 Size)
{
	const double X = (Col + 0.5) / Size * 2.0 - 1.0;
	const double Y = (Row + 0.5) / Size * 2.0 - 1.0;
	const double Z = 1.0;
	FVector Ret;
	switch (Face)
	{
		case 0: Ret = FVector(+Z, -Y, -X); break;
		case 1: Ret = FVector(-Z, -Y, +X); break;
		case 2: Ret = FVector(+X, +Z, +Y); break;
		case 3: Ret = FVector(+X, -Z, -Y); break;
		case 4: Ret = FVector(+X, -Y, +Z); break;
		default: Ret = FVector(-X, -Y, -Z); break;
	}
	return FVector(Ret.X, Ret.Z, Ret.Y);
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabMjAdapterFidelitySceneTest,
	"URLab.RenderKinematics.AdapterFidelity.Scene",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabMjAdapterFidelitySceneTest::RunTest(const FString&)
{
	FString Error;
	FMjRenderKinematicsCandidate Candidate;
	if (!TestTrue(TEXT("model with an SDF geom installs"), PrepareAdapterFidelity(Candidate, Error)))
	{
		AddError(Error);
		return false;
	}
	FRenderScene Scene;
	FRenderSceneResources Resources;
	FRenderFrame Initial;
	if (!TestTrue(TEXT("neutral scene transfers"),
		Candidate.TakeNeutralScene(Scene, Resources, Initial, Error))) return false;

	const int32 Override = FindGeom(Scene.Geoms, TEXT("override"));
	const int32 Inherit = FindGeom(Scene.Geoms, TEXT("inherit"));
	const int32 Hidden = FindGeom(Scene.Geoms, TEXT("hidden"));
	const int32 Sdf = FindGeom(Scene.Geoms, TEXT("sdf"));
	if (!TestTrue(TEXT("named geoms exist"), Override >= 0 && Inherit >= 0 && Hidden >= 0 && Sdf >= 0))
		return false;
	TestEqual(TEXT("authored geom rgba overrides the material colour"),
		Resources.Geoms[Override].Rgba, FLinearColor(1, 0, 0, 1));
	TestEqual(TEXT("default geom rgba inherits the material colour"),
		Resources.Geoms[Inherit].Rgba, FLinearColor(0, 1, 0, 1));
	TestEqual(TEXT("rgba alpha 0 hides a geom even with a material"),
		Resources.Geoms[Hidden].Rgba.A, 0.0f);
	TestEqual(TEXT("mesh-backed SDF geom renders as a mesh"),
		Scene.Geoms[Sdf].Kind, ERenderGeomKind::Mesh);
	TestTrue(TEXT("SDF geom references its mesh resource"),
		Resources.Meshes.IsValidIndex(Scene.Geoms[Sdf].MeshResourceIndex));

	const FRenderCameraDesc* Offset = Scene.Cameras.FindByPredicate(
		[](const FRenderCameraDesc& Camera) { return Camera.CanonicalName == TEXT("offset"); });
	if (TestNotNull(TEXT("intrinsic camera exists"), Offset))
	{
		TestEqual(TEXT("focal length is in pixels"), Offset->Optics.FocalPixelsX, 200.0f);
		TestEqual(TEXT("principal x is OpenCV, matching mujoco.Renderer"),
			Offset->Optics.PrincipalPixelsX, 60.0f);
		TestEqual(TEXT("principal y is OpenCV, matching mujoco.Renderer"),
			Offset->Optics.PrincipalPixelsY, 30.0f);
		TestEqual(TEXT("depth near defaults to znear * extent"), Offset->Capture.DepthNearCm, 4.0f);
		TestEqual(TEXT("depth far defaults to zfar * extent"), Offset->Capture.DepthFarCm, 8000.0f);
	}
	if (TestTrue(TEXT("user camera is installed"), Scene.UserCamera.IsSet()))
	{
		const FRenderCameraDesc& User = Scene.UserCamera.GetValue();
		TestEqual(TEXT("user camera fovy comes from visual/global"),
			User.Optics.VerticalFovDegrees, 60.0f);
		TestEqual(TEXT("user camera resolution comes from the offscreen buffer"),
			User.Optics.Resolution, FIntPoint(320, 240));
		TestEqual(TEXT("user camera near clip is znear * extent"), User.Optics.NearClipCm, 4.0f);
		TestEqual(TEXT("user camera far clip is zfar * extent"), User.Optics.FarClipCm, 8000.0f);
		TestEqual(TEXT("user camera depth follows its clip range"), User.Capture.DepthFarCm, 8000.0f);
	}

	TestTrue(TEXT("skybox is enabled"), Scene.Environment.bSkyboxEnabled);
	TestEqual(TEXT("skybox transfers six faces"),
		Scene.Environment.SkyboxFaceTextureIndices.Num(), 6);
	TestTrue(TEXT("dome top samples the zenith colour"), Scene.Environment.SkyTop.R > 0.9f);
	TestTrue(TEXT("dome bottom samples the nadir colour"), Scene.Environment.SkyBottom.R < 0.1f);
	TestEqual(TEXT("cube texture extracts a single square face"),
		FIntPoint(Resources.Textures[1].Width, Resources.Textures[1].Height), FIntPoint(16, 16));
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabMjAdapterSkyboxFaceMappingTest,
	"URLab.RenderKinematics.AdapterFidelity.SkyboxFaceMapping",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabMjAdapterSkyboxFaceMappingTest::RunTest(const FString&)
{
	constexpr int32 Size = 8;
	int32 Mismatches = 0;
	for (int32 Face = 0; Face < 6; ++Face)
		for (int32 Row = 0; Row < Size; ++Row)
			for (int32 Col = 0; Col < Size; ++Col)
			{
				const FVector Unreal = UnrealCubeDirection(Face, Col, Row, Size);
				const FVector MuJoCo(Unreal.X, -Unreal.Y, Unreal.Z);
				const FVector Gl(MuJoCo.X, MuJoCo.Z, -MuJoCo.Y);
				int32 GlFace, GlCol, GlRow;
				GlCubeTexel(Gl, Size, GlFace, GlCol, GlRow);
				if (GlFace != Face || GlCol != Col || GlRow != Row) ++Mismatches;
			}
	TestEqual(TEXT("each UE cube texel samples the same MuJoCo skybox texel"), Mismatches, 0);
	return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FURLabMjAdapterQposExtrasTest,
	"URLab.RenderKinematics.AdapterFidelity.QposMocapAndUserCamera",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FURLabMjAdapterQposExtrasTest::RunTest(const FString&)
{
	FString Error;
	FMjRenderKinematicsCandidate Candidate;
	if (!PrepareAdapterFidelity(Candidate, Error))
	{
		AddError(Error);
		return false;
	}
	FRenderScene Scene;
	FRenderSceneResources Resources;
	FRenderFrame Initial;
	if (!Candidate.TakeNeutralScene(Scene, Resources, Initial, Error)) return false;
	const int32 Target = FindByName(Scene.Bodies, TEXT("target"));
	const int32 Tracker = Scene.Cameras.IndexOfByPredicate(
		[](const FRenderCameraDesc& Camera) { return Camera.CanonicalName == TEXT("tracker"); });
	FMjRenderKinematics Kinematics;
	if (!TestTrue(TEXT("kinematics commits"), Kinematics.Commit(MoveTemp(Candidate), FString(), Error))
		|| !TestTrue(TEXT("mocap body and tracking camera exist"), Target > 0 && Tracker >= 0))
		return false;
	TestEqual(TEXT("mocap count comes from the model"), Kinematics.GetMocapCount(), 1);

	const TArray<double> Qpos = {0.3};
	FRenderFrame Default;
	if (!TestTrue(TEXT("qpos without mocap uses model defaults"), Kinematics.FrameFromQpos(
		TEXT("fidelity"), Qpos, 1, 0.0, false, 0, Default, Error))) return false;
	TestTrue(TEXT("default mocap pose is the model body pose"),
		Default.BodySceneTransforms[Target].GetTranslation().Equals(FVector(50, 0, 100), 1.e-3));

	const TArray<double> Mocap = {2, 2, 2, 1, 0, 0, 0};
	FMjRenderQposExtras Extras;
	Extras.Mocap = Mocap;
	const double Eye[3] = {1, 2, 3};
	const double Wxyz[4] = {1, 0, 0, 0};
	Extras.UserCameraSceneTransform = URLabCoordinateConversion::RightHandedOpticalPoseToUnreal(Eye, Wxyz);
	FRenderFrame Moved;
	if (!TestTrue(TEXT("qpos with mocap and user camera runs"), Kinematics.FrameFromQpos(
		TEXT("fidelity"), Qpos, 2, 0.0, false, 0, Moved, Error, Extras))) return false;
	TestTrue(TEXT("supplied mocap pose moves the mocap body"),
		Moved.BodySceneTransforms[Target].GetTranslation().Equals(FVector(200, -200, 200), 1.e-3));
	TestFalse(TEXT("tracking camera follows the mocap target (mj_camlight runs)"),
		Moved.CameraSceneTransforms[Tracker].GetRotation().Equals(
			Default.CameraSceneTransforms[Tracker].GetRotation(), 1.e-4));
	TestTrue(TEXT("user camera pose is applied"), Moved.UserCameraSceneTransform.IsSet()
		&& Moved.UserCameraSceneTransform->Equals(Extras.UserCameraSceneTransform.GetValue()));

	FRenderFrame Again;
	TestTrue(TEXT("mocap does not carry over to the next frame"), Kinematics.FrameFromQpos(
		TEXT("fidelity"), Qpos, 3, 0.0, false, 0, Again, Error));
	TestTrue(TEXT("next default frame restores the model mocap pose"),
		Again.BodySceneTransforms[Target].GetTranslation().Equals(FVector(50, 0, 100), 1.e-3));

	const TArray<double> Short = {2, 2, 2};
	FMjRenderQposExtras Bad;
	Bad.Mocap = Short;
	TestFalse(TEXT("wrong mocap width rejects"), Kinematics.FrameFromQpos(
		TEXT("fidelity"), Qpos, 4, 0.0, false, 0, Again, Error, Bad));
	return !HasAnyErrors();
}
#endif // WITH_DEV_AUTOMATION_TESTS
