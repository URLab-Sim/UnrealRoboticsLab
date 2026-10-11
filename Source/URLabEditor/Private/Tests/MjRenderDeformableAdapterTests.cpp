// Copyright (c) 2026 Jonathan Embley-Riches. Licensed under the Apache License, Version 2.0.

#include "Misc/AutomationTest.h"
#include "MuJoCo/Core/MjOwnedModel.h"
#include "MuJoCo/Render/MjRenderSceneAdapter.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjRenderDeformableAdapterTest,
	"URLab.RenderAdapter.FlexSkinEnvironmentAndDebug",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjRenderDeformableAdapterTest::RunTest(const FString&)
{
	const char* Xml = R"(<mujoco model="render_surfaces">
  <worldbody>
    <body name="anchor"><geom type="sphere" size="0.02"/></body>
    <flexcomp name="cloth" type="grid" dim="2" count="2 2 1"
              spacing="0.1 0.1 0.1" pos="0 0 0.3" radius="0.01"/>
  </worldbody>
  <asset>
    <skin name="skin" vertex="0 0 0  1 0 0  0 1 0"
          face="0 1 2">
      <bone body="anchor" bindpos="0 0 0" bindquat="1 0 0 0"
            vertid="0 1 2" vertweight="1 1 1"/>
    </skin>
  </asset>
</mujoco>)";
	TArray<uint8> Bytes;
	Bytes.Append(reinterpret_cast<const uint8*>(Xml), FCStringAnsi::Strlen(Xml));
	FMjOwnedModelCandidate Model;
	FString Error;
	if (!TestTrue(TEXT("surface fixture compiles"), FMjOwnedModelCandidate::FromBytes(
		EMjModelSourceKind::XmlWithAssets, Bytes, {}, Model, Error, TEXT("surface fixture"))))
	{
		AddError(Error); return false;
	}
	FMjRenderSceneAdapter Adapter;
	FRenderScene Scene;
	FRenderSceneResources Resources;
	FRenderFrame Frame;
	const bool bInitial = Model.VisitPrepared([&](const mjModel* NativeModel,
		const mjData* NativeData)
	{
		return Adapter.Extract(NativeModel, NativeData, TEXT("surfaces-a"), 7, -1,
			false, nullptr, Scene, Resources, Error)
			&& Adapter.FrameFromData(NativeData, 1, Frame, Error);
	});
	if (!TestTrue(TEXT("adapter extracts surface topology and frame"), bInitial))
	{
		AddError(Error); return false;
	}
	TestEqual(TEXT("flex and skin surface count"), Scene.Deformables.Num(), 2);
	TestTrue(TEXT("flex topology has front and back faces"),
		Scene.Deformables[0].Triangles.Num() >= 6);
	// ProceduralMeshComponent uses the opposite of the conventional cross product
	// for its visible front face (see KismetProceduralMeshLibrary's TriNormal).
	// The Y reflection already reverses MuJoCo's face winding, so retaining the
	// source corner order makes UE's front face agree with the supplied normal.
	if (Scene.Deformables[0].Triangles.Num() >= 6
		&& Frame.Deformables[0].Positions.Num() >= 6
		&& Frame.Deformables[0].Normals.Num() >= 6)
	{
		const TArray<int32>& Triangles = Scene.Deformables[0].Triangles;
		const TArray<FVector>& Positions = Frame.Deformables[0].Positions;
		const TArray<FVector>& Normals = Frame.Deformables[0].Normals;
		for (int32 Face = 0; Face < 2; ++Face)
		{
			const int32 Base = 3 * Face;
			TestEqual(FString::Printf(TEXT("visual face %d keeps source corner order"), Face),
				Triangles[Base], Base);
			TestEqual(FString::Printf(TEXT("visual face %d keeps second corner"), Face),
				Triangles[Base + 1], Base + 1);
			TestEqual(FString::Printf(TEXT("visual face %d keeps third corner"), Face),
				Triangles[Base + 2], Base + 2);
			const FVector UeFrontNormal = -FVector::CrossProduct(
				Positions[Base + 1] - Positions[Base],
				Positions[Base + 2] - Positions[Base]).GetSafeNormal();
			const FVector SuppliedNormal = (Normals[Base] + Normals[Base + 1]
				+ Normals[Base + 2]).GetSafeNormal();
			TestTrue(FString::Printf(TEXT("visual face %d UE front agrees with MuJoCo normal"), Face),
				FVector::DotProduct(UeFrontNormal, SuppliedNormal) > 0.9);
		}
	}
	TestEqual(TEXT("skin triangle count"), Scene.Deformables[1].Triangles.Num(), 3);
	TestEqual(TEXT("frame has two surfaces"), Frame.Deformables.Num(), 2);
	TestEqual(TEXT("skin vertex count"), Frame.Deformables[1].Positions.Num(), 3);
	TestTrue(TEXT("MuJoCo visual face corners and normals are copied exactly"),
		Model.VisitPrepared([&](const mjModel* NativeModel, const mjData* NativeData)
		{
			mjvScene Visual{};
			mjv_makeScene(NativeModel, &Visual, 4096);
			mjvOption Option; mjv_defaultOption(&Option);
			mjvCamera Camera; mjv_defaultCamera(&Camera);
			mjv_updateScene(NativeModel, const_cast<mjData*>(NativeData), &Option,
				nullptr, &Camera, mjCAT_ALL, &Visual);
			const int32 Used = Visual.flexfaceused[0];
			bool bMatches = Frame.Deformables[0].Positions.Num() == 3 * Used
				&& Frame.Deformables[0].Normals.Num() == 3 * Used;
			const int32 First = 9 * Visual.flexfaceadr[0];
			for (int32 Vertex = 0; bMatches && Vertex < 3 * Used; ++Vertex)
			{
				const float* P = Visual.flexface + First + 3 * Vertex;
				const float* N = Visual.flexnormal + First + 3 * Vertex;
				bMatches = Frame.Deformables[0].Positions[Vertex].Equals(
					FVector(100 * P[0], -100 * P[1], 100 * P[2]), 1.e-4)
					&& Frame.Deformables[0].Normals[Vertex].Equals(
						FVector(N[0], -N[1], N[2]), 1.e-5);
			}
			mjv_freeScene(&Visual);
			return bMatches;
		}));
	TestTrue(TEXT("full neutral frame validates"), Frame.IsValid(Scene, Error));
	FRenderScene Patched = Scene;
	Patched.SceneRevision = TEXT("surfaces-b");
	Patched.Visualization.Features.Add(TEXT("com"));
	TestTrue(TEXT("patch syncs visualization to adapter"),
		Adapter.RebaseSceneRevision(TEXT("surfaces-a"), Patched, Error));
	FRenderFrame DebugFrame;
	TestTrue(TEXT("post-patch debug frame is produced"),
		Model.VisitPrepared([&](const mjModel*, const mjData* NativeData)
		{ return Adapter.FrameFromData(NativeData, 2, DebugFrame, Error); }));
	TestEqual(TEXT("post-patch revision is current"), DebugFrame.Identity.SceneRevision,
		FString(TEXT("surfaces-b")));
	TestTrue(TEXT("debug frame validates"), DebugFrame.IsValid(Patched, Error));
	return !HasAnyErrors();
}
#endif
