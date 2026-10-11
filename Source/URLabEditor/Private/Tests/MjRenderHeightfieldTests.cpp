// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Misc/AutomationTest.h"
#include "MuJoCo/Core/MjOwnedModel.h"
#include "MuJoCo/Render/MjRenderResourceShim.h"
#include "MuJoCo/Render/MjRenderSceneAdapter.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRenderHeightfieldResourceTest,
	"URLab.RenderAdapter.HeightfieldClassicVisualMesh",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRenderHeightfieldResourceTest::RunTest(const FString&)
{
	const char* Xml = R"(<mujoco model="heightfield_render">
  <asset><hfield name="asymmetric" nrow="2" ncol="3"
    size="0.3 0.2 0.1 0.05" elevation="0 0.3 1 0.6 0.2 0.8"/></asset>
  <worldbody>
    <geom name="first" type="hfield" hfield="asymmetric" rgba="0.8 0.2 0.1 1"/>
    <geom name="second" type="hfield" hfield="asymmetric" pos="1 0 0"
      euler="0 0 30" rgba="0.1 0.6 0.8 1"/>
  </worldbody>
</mujoco>)";
	TArray<uint8> Bytes;
	Bytes.Append(reinterpret_cast<const uint8*>(Xml), FCStringAnsi::Strlen(Xml));
	FMjOwnedModelCandidate Model;
	FString Error;
	if (!TestTrue(TEXT("compile asymmetric heightfield"),
		FMjOwnedModelCandidate::FromBytes(EMjModelSourceKind::XmlWithAssets,
			Bytes, {}, Model, Error, TEXT("heightfield fixture"))))
	{ AddError(Error); return false; }
	FMjRenderSceneAdapter Adapter;
	FRenderScene Scene;
	FRenderSceneResources Resources;
	const bool bExtracted = Model.VisitPrepared([&](const mjModel_* Native,
		const mjData* Data)
	{
		return Adapter.Extract(Native, Data, TEXT("heightfield-a"),
			7, -1, false, nullptr, Scene, Resources, Error);
	});
	if (!TestTrue(TEXT("extract heightfield through neutral adapter"), bExtracted))
	{ AddError(Error); return false; }
	TestEqual(TEXT("two geom instances"), Scene.Geoms.Num(), 2);
	TestEqual(TEXT("one shared heightfield resource"), Resources.Meshes.Num(), 1);
	if (Scene.Geoms.Num() != 2 || Resources.Meshes.Num() != 1) return false;
	for (int32 GeomId = 0; GeomId < 2; ++GeomId)
	{
		TestEqual(FString::Printf(TEXT("heightfield %d uses neutral mesh kind"), GeomId),
			Scene.Geoms[GeomId].Kind, ERenderGeomKind::Mesh);
		TestEqual(FString::Printf(TEXT("heightfield %d shares mesh index"), GeomId),
			Scene.Geoms[GeomId].MeshResourceIndex, 0);
		TestEqual(FString::Printf(TEXT("heightfield %d keeps source name"), GeomId),
			Scene.Geoms[GeomId].MeshName, FString(TEXT("asymmetric")));
	}
	TestTrue(TEXT("instances retain distinct local poses"),
		!Scene.Geoms[0].BodyLocalTransform.Equals(Scene.Geoms[1].BodyLocalTransform));
	TestTrue(TEXT("instances retain distinct surface colours"),
		!Resources.Geoms[0].Rgba.Equals(Resources.Geoms[1].Rgba));
	const FRenderMeshResource& Mesh = Resources.Meshes[0];
	// Classic mjr_uploadHField: 4 top triangles, 12 skirt triangles, then
	// 2*numquads^2 bottom triangles (the fixture uses MuJoCo's default 4).
	TestEqual(TEXT("classic visual triangle vertices"), Mesh.Positions.Num(), 144);
	TestEqual(TEXT("normal count"), Mesh.Normals.Num(), Mesh.Positions.Num());
	TestEqual(TEXT("UV count"), Mesh.UVs.Num(), Mesh.Positions.Num());
	TestEqual(TEXT("triangle index count"), Mesh.Triangles.Num(), Mesh.Positions.Num());
	if (Mesh.Positions.Num() != 144) return false;
	// MuJoCo's compiler flips the XML elevation rows. Its visualizer sees row 0
	// as 0.6, 0.2, 0.8 and row 1 as 0, 0.3, 1 for this asymmetric fixture.
	TestTrue(TEXT("first strip corner is compiled row 1 col 0"),
		Mesh.Positions[0].Equals(FVector(-30.0, -20.0, 0.0), 1.e-4));
	TestTrue(TEXT("next strip corner is compiled row 0 col 0"),
		Mesh.Positions[1].Equals(FVector(-30.0, 20.0, 6.0), 1.e-4));
	TestTrue(TEXT("strip diagonal reaches compiled row 1 col 1"),
		Mesh.Positions[2].Equals(FVector(0.0, -20.0, 3.0), 1.e-4));
	TestTrue(TEXT("top planar UVs follow local grid axes"),
		Mesh.UVs[0].Equals(FVector2D(0.0, 0.0), 1.e-5)
		&& Mesh.UVs[1].Equals(FVector2D(0.0, 1.0), 1.e-5)
		&& Mesh.UVs[2].Equals(FVector2D(0.5, 0.0), 1.e-5));
	for (int32 Triangle = 0; Triangle < Mesh.Positions.Num() / 3; ++Triangle)
	{
		const int32 I = 3 * Triangle;
		TestEqual(FString::Printf(TEXT("triangle %d keeps classic corner order"), Triangle),
			Mesh.Triangles[I], I);
		const FVector Front = -FVector::CrossProduct(
			Mesh.Positions[I + 1] - Mesh.Positions[I],
			Mesh.Positions[I + 2] - Mesh.Positions[I]).GetSafeNormal();
		if (!Front.IsNearlyZero())
		{
			TestTrue(FString::Printf(TEXT("triangle %d UE front agrees with normal"), Triangle),
				FVector::DotProduct(Front, Mesh.Normals[I]) > 0.99);
		}
	}
	TestTrue(TEXT("left wall has outward normal"),
		Mesh.Normals[12].Equals(-FVector::ForwardVector, 1.e-5));
	TestTrue(TEXT("bottom has downward normal"),
		Mesh.Normals.Last().Equals(-FVector::UpVector, 1.e-5));
	TestTrue(TEXT("neutral resources validate"), Resources.IsValid(Error));
	TestTrue(TEXT("out-of-bounds compiled sample slice is rejected before read"),
		Model.VisitPrepared([&](const mjModel* Native, const mjData*)
		{
			mjModel Invalid = *Native;
			int32 BadAddress = static_cast<int32>(Native->nhfielddata);
			Invalid.hfield_adr = &BadAddress;
			FRenderSceneResources Ignored;
			FString Rejection;
			return !MjRenderResourceShim::Fill(Ignored, &Invalid,
				TEXT("invalid-hfield"), Rejection)
				&& Rejection.Contains(TEXT("sample slice"));
		}));

	// Native MuJoCo accepts positive dimensions with a one-sample axis. URLab's
	// neutral renderer reports those unsupported visual grids explicitly.
	const struct FDegenerateGrid
	{
		int32 Rows;
		int32 Cols;
		const TCHAR* Elevation;
	} DegenerateGrids[] = {
		{1, 1, TEXT("0")},
		{1, 3, TEXT("0 0.5 1")},
		{3, 1, TEXT("0 0.5 1")}
	};
	for (const FDegenerateGrid& Grid : DegenerateGrids)
	{
		const FString HfieldName = FString::Printf(TEXT("heightfield_%dx%d"), Grid.Rows, Grid.Cols);
		const FString DegenerateXml = FString::Printf(
			TEXT("<mujoco model='degenerate_heightfield'><asset><hfield name='%s' nrow='%d' ncol='%d' size='1 1 1 0.1' elevation='%s'/></asset><worldbody><geom name='terrain' type='hfield' hfield='%s'/></worldbody></mujoco>"),
			*HfieldName, Grid.Rows, Grid.Cols, Grid.Elevation, *HfieldName);
		TArray<uint8> DegenerateBytes;
		FTCHARToUTF8 DegenerateUtf8Xml(*DegenerateXml);
		DegenerateBytes.Append(reinterpret_cast<const uint8*>(DegenerateUtf8Xml.Get()), DegenerateUtf8Xml.Length());
		FMjOwnedModelCandidate DegenerateModel;
		FString DegenerateError;
		if (!TestTrue(FString::Printf(TEXT("native MuJoCo compiles %s"), *HfieldName),
			FMjOwnedModelCandidate::FromBytes(EMjModelSourceKind::XmlWithAssets,
				DegenerateBytes, {}, DegenerateModel, DegenerateError, HfieldName)))
		{
			AddError(DegenerateError);
			continue;
		}
		FRenderScene DegenerateScene;
		FRenderSceneResources DegenerateResources;
		const bool bDegenerateExtracted = DegenerateModel.VisitPrepared([&](const mjModel_* Native,
			const mjData* Data)
		{
			return Adapter.Extract(Native, Data, HfieldName,
				7, -1, false, nullptr, DegenerateScene, DegenerateResources, DegenerateError);
		});
		TestFalse(FString::Printf(TEXT("renderer rejects unsupported %s grid"), *HfieldName), bDegenerateExtracted);
		TestTrue(FString::Printf(TEXT("renderer error names %s and its dimensions"), *HfieldName),
			DegenerateError.Contains(HfieldName)
				&& DegenerateError.Contains(FString::Printf(TEXT("%dx%d"), Grid.Rows, Grid.Cols))
				&& DegenerateError.Contains(TEXT("at least 2 rows and 2 columns")));
	}
	return !HasAnyErrors();
}
#endif
