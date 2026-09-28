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

// Pure-logic tests for UMjLidarPointCloudViz: color ramps, palette lookups,
// normalization, stride math, and PLY text generation. No world or simulation
// required (same style as MjLidarTests.cpp).

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "MuJoCo/Components/Sensors/MjLidarPointCloudViz.h"

namespace
{
	// NOTE: prefixed Viz* because URLabEditor is a unity build - this file can
	// share a translation unit with MjLidarTests.cpp, whose anonymous-namespace
	// helpers must not collide with ours.
	bool VizNearlyEqualColor(FAutomationTestBase& Test, const FString& What, const FLinearColor& Actual, const FLinearColor& Expected, float Tolerance = 1e-4f)
	{
		const bool bEqual = FMath::IsNearlyEqual(Actual.R, Expected.R, Tolerance)
			&& FMath::IsNearlyEqual(Actual.G, Expected.G, Tolerance)
			&& FMath::IsNearlyEqual(Actual.B, Expected.B, Tolerance);
		return Test.TestTrue(FString::Printf(TEXT("%s (%s vs %s)"), *What, *Actual.ToString(), *Expected.ToString()), bEqual);
	}
} // namespace

// ============================================================================
// URLab.Lidar.Viz.RangeRamp_EndpointsAndClamp
//   The ramp's stops are assertable and out-of-range inputs clamp.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLidarVizRangeRampTest,
	"URLab.Lidar.Viz.RangeRamp_EndpointsAndClamp",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjLidarVizRangeRampTest::RunTest(const FString& Parameters)
{
	VizNearlyEqualColor(*this, TEXT("T=0 red"), UMjLidarPointCloudViz::RangeRampColor(0.0f), FLinearColor(1.0f, 0.0f, 0.0f));
	VizNearlyEqualColor(*this, TEXT("T=1/3 yellow"), UMjLidarPointCloudViz::RangeRampColor(1.0f / 3.0f), FLinearColor(1.0f, 1.0f, 0.0f));
	VizNearlyEqualColor(*this, TEXT("T=0.5 midpoint"), UMjLidarPointCloudViz::RangeRampColor(0.5f), FLinearColor(0.5f, 1.0f, 0.0f));
	VizNearlyEqualColor(*this, TEXT("T=2/3 green"), UMjLidarPointCloudViz::RangeRampColor(2.0f / 3.0f), FLinearColor(0.0f, 1.0f, 0.0f));
	VizNearlyEqualColor(*this, TEXT("T=1 blue"), UMjLidarPointCloudViz::RangeRampColor(1.0f), FLinearColor(0.0f, 0.0f, 1.0f));

	// Out-of-range values clamp to the endpoints.
	VizNearlyEqualColor(*this, TEXT("T=-0.5 clamps red"), UMjLidarPointCloudViz::RangeRampColor(-0.5f), FLinearColor(1.0f, 0.0f, 0.0f));
	VizNearlyEqualColor(*this, TEXT("T=1.5 clamps blue"), UMjLidarPointCloudViz::RangeRampColor(1.5f), FLinearColor(0.0f, 0.0f, 1.0f));
	return true;
}

// ============================================================================
// URLab.Lidar.Viz.ComputeColor_ByRange_Monotonic
//   ByRange maps the [RangeMinM..RangeMaxM] span monotonically onto the ramp.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLidarVizByRangeTest,
	"URLab.Lidar.Viz.ComputeColor_ByRange_Monotonic",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjLidarVizByRangeTest::RunTest(const FString& Parameters)
{
	FMjLidarVizColorContext Ctx; // RangeMinM = 0, RangeMaxM = 100 defaults
	Ctx.RangeMinM = 0.0f;
	Ctx.RangeMaxM = 100.0f;

	FLidarPoint Near;  Near.RangeM = 0.0f;
	FLidarPoint Mid;   Mid.RangeM = 50.0f;
	FLidarPoint Far;   Far.RangeM = 100.0f;

	VizNearlyEqualColor(*this, TEXT("range 0 -> red"),
		UMjLidarPointCloudViz::ComputePointColor(EMjLidarVizColorMode::ByRange, Near, Ctx), FLinearColor(1.0f, 0.0f, 0.0f));
	VizNearlyEqualColor(*this, TEXT("range 50 -> midpoint"),
		UMjLidarPointCloudViz::ComputePointColor(EMjLidarVizColorMode::ByRange, Mid, Ctx), FLinearColor(0.5f, 1.0f, 0.0f));
	VizNearlyEqualColor(*this, TEXT("range 100 -> blue"),
		UMjLidarPointCloudViz::ComputePointColor(EMjLidarVizColorMode::ByRange, Far, Ctx), FLinearColor(0.0f, 0.0f, 1.0f));

	// Monotonic across the span: red falls, blue rises with range.
	float PrevR = 2.0f;
	float PrevB = -1.0f;
	for (int32 i = 0; i <= 20; ++i)
	{
		FLidarPoint P;
		P.RangeM = 100.0f * static_cast<float>(i) / 20.0f;
		const FLinearColor C = UMjLidarPointCloudViz::ComputePointColor(EMjLidarVizColorMode::ByRange, P, Ctx);
		TestTrue(FString::Printf(TEXT("R non-increasing at %.1f m"), P.RangeM), C.R <= PrevR + 1e-6f);
		TestTrue(FString::Printf(TEXT("B non-decreasing at %.1f m"), P.RangeM), C.B >= PrevB - 1e-6f);
		PrevR = C.R;
		PrevB = C.B;
	}

	// Out-of-bounds ranges clamp to the endpoints.
	FLidarPoint Beyond; Beyond.RangeM = 250.0f;
	VizNearlyEqualColor(*this, TEXT("range 250 clamps blue"),
		UMjLidarPointCloudViz::ComputePointColor(EMjLidarVizColorMode::ByRange, Beyond, Ctx), FLinearColor(0.0f, 0.0f, 1.0f));
	FLidarPoint Behind; Behind.RangeM = -5.0f;
	VizNearlyEqualColor(*this, TEXT("range -5 clamps red"),
		UMjLidarPointCloudViz::ComputePointColor(EMjLidarVizColorMode::ByRange, Behind, Ctx), FLinearColor(1.0f, 0.0f, 0.0f));
	return true;
}

// ============================================================================
// URLab.Lidar.Viz.ComputeColor_ByHeight_Normalization
//   ByHeight uses the context's height bounds; flat fields hit the midpoint.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLidarVizByHeightTest,
	"URLab.Lidar.Viz.ComputeColor_ByHeight_Normalization",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjLidarVizByHeightTest::RunTest(const FString& Parameters)
{
	FMjLidarVizColorContext Ctx;
	Ctx.HeightMinCm = 0.0f;
	Ctx.HeightMaxCm = 100.0f;

	FLidarPoint Low;  Low.WorldPos = FVector(0.0f, 0.0f, 0.0f);
	FLidarPoint Mid;  Mid.WorldPos = FVector(0.0f, 0.0f, 50.0f);
	FLidarPoint High; High.WorldPos = FVector(0.0f, 0.0f, 100.0f);

	VizNearlyEqualColor(*this, TEXT("height 0 -> red"),
		UMjLidarPointCloudViz::ComputePointColor(EMjLidarVizColorMode::ByHeight, Low, Ctx), FLinearColor(1.0f, 0.0f, 0.0f));
	VizNearlyEqualColor(*this, TEXT("height 50 -> midpoint"),
		UMjLidarPointCloudViz::ComputePointColor(EMjLidarVizColorMode::ByHeight, Mid, Ctx), FLinearColor(0.5f, 1.0f, 0.0f));
	VizNearlyEqualColor(*this, TEXT("height 100 -> blue"),
		UMjLidarPointCloudViz::ComputePointColor(EMjLidarVizColorMode::ByHeight, High, Ctx), FLinearColor(0.0f, 0.0f, 1.0f));

	// Degenerate (flat) height range maps to the ramp midpoint.
	FMjLidarVizColorContext Flat;
	Flat.HeightMinCm = 7.0f;
	Flat.HeightMaxCm = 7.0f;
	FLidarPoint P;
	P.WorldPos = FVector(0.0f, 0.0f, 7.0f);
	VizNearlyEqualColor(*this, TEXT("flat range -> midpoint"),
		UMjLidarPointCloudViz::ComputePointColor(EMjLidarVizColorMode::ByHeight, P, Flat), FLinearColor(0.5f, 1.0f, 0.0f));
	return true;
}

// ============================================================================
// URLab.Lidar.Viz.ComputeColor_ByElevationRing_Dynamic
//   ByElevationRing colors each ring consistently across azimuths.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLidarVizByElevationTest,
	"URLab.Lidar.Viz.ComputeColor_ByElevationRing_Dynamic",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjLidarVizByElevationTest::RunTest(const FString& Parameters)
{
	FMjLidarVizColorContext Ctx;
	Ctx.ElevationMinDeg = -15.0f;
	Ctx.ElevationMaxDeg = 15.0f;

	FLidarPoint Bottom; Bottom.ElevationDeg = -15.0f;
	FLidarPoint Mid;    Mid.ElevationDeg = 0.0f;
	FLidarPoint Top;    Top.ElevationDeg = 15.0f;

	VizNearlyEqualColor(*this, TEXT("elevation -15 -> red"),
		UMjLidarPointCloudViz::ComputePointColor(EMjLidarVizColorMode::ByElevationRing, Bottom, Ctx), FLinearColor(1.0f, 0.0f, 0.0f));
	VizNearlyEqualColor(*this, TEXT("elevation 0 -> midpoint"),
		UMjLidarPointCloudViz::ComputePointColor(EMjLidarVizColorMode::ByElevationRing, Mid, Ctx), FLinearColor(0.5f, 1.0f, 0.0f));
	VizNearlyEqualColor(*this, TEXT("elevation 15 -> blue"),
		UMjLidarPointCloudViz::ComputePointColor(EMjLidarVizColorMode::ByElevationRing, Top, Ctx), FLinearColor(0.0f, 0.0f, 1.0f));

	// Same elevation (same ring) -> same color regardless of azimuth.
	FLidarPoint RingA; RingA.ElevationDeg = 5.0f; RingA.AzimuthDeg = 10.0f;
	FLidarPoint RingB; RingB.ElevationDeg = 5.0f; RingB.AzimuthDeg = 350.0f;
	const FLinearColor ColorA = UMjLidarPointCloudViz::ComputePointColor(EMjLidarVizColorMode::ByElevationRing, RingA, Ctx);
	const FLinearColor ColorB = UMjLidarPointCloudViz::ComputePointColor(EMjLidarVizColorMode::ByElevationRing, RingB, Ctx);
	VizNearlyEqualColor(*this, TEXT("same ring -> same color"), ColorA, ColorB);
	return true;
}

// ============================================================================
// URLab.Lidar.Viz.SurfaceIdColor_Deterministic
//   The palette is stable, 12-periodic, and gray for misses.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLidarVizSurfaceIdColorTest,
	"URLab.Lidar.Viz.SurfaceIdColor_Deterministic",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjLidarVizSurfaceIdColorTest::RunTest(const FString& Parameters)
{
	// Deterministic across calls, 12-periodic over ids.
	for (int32 Id = 0; Id < 24; ++Id)
	{
		TestTrue(FString::Printf(TEXT("id %d repeats every 12"), Id),
			UMjLidarPointCloudViz::SurfaceIdColor(Id) == UMjLidarPointCloudViz::SurfaceIdColor(Id + 12));
	}

	// Misses (negative ids) map to fixed gray.
	VizNearlyEqualColor(*this, TEXT("miss -1 -> gray"), UMjLidarPointCloudViz::SurfaceIdColor(-1), FLinearColor::Gray);
	VizNearlyEqualColor(*this, TEXT("miss -7 -> gray"), UMjLidarPointCloudViz::SurfaceIdColor(-7), FLinearColor::Gray);

	// Palette entries are distinct from the miss gray and from each other.
	for (int32 Id = 0; Id < 12; ++Id)
	{
		TestTrue(FString::Printf(TEXT("palette %d not gray"), Id), !(UMjLidarPointCloudViz::SurfaceIdColor(Id) == FLinearColor::Gray));
		for (int32 Other = Id + 1; Other < 12; ++Other)
		{
			TestTrue(FString::Printf(TEXT("palette %d != %d"), Id, Other),
				!(UMjLidarPointCloudViz::SurfaceIdColor(Id) == UMjLidarPointCloudViz::SurfaceIdColor(Other)));
		}
	}
	return true;
}

// ============================================================================
// URLab.Lidar.Viz.TargetIdColor_Deterministic
//   Target colors are stable per id and valid RGB.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLidarVizTargetIdColorTest,
	"URLab.Lidar.Viz.TargetIdColor_Deterministic",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjLidarVizTargetIdColorTest::RunTest(const FString& Parameters)
{
	TestTrue(TEXT("same id -> same color"),
		UMjLidarPointCloudViz::TargetIdColor(7) == UMjLidarPointCloudViz::TargetIdColor(7));
	TestTrue(TEXT("negative id stable"),
		UMjLidarPointCloudViz::TargetIdColor(-3) == UMjLidarPointCloudViz::TargetIdColor(-3));
	TestTrue(TEXT("different ids differ"),
		!(UMjLidarPointCloudViz::TargetIdColor(1) == UMjLidarPointCloudViz::TargetIdColor(2)));

	// Components stay within [0, 1].
	bool bInRange = true;
	for (int32 Id = 0; Id < 32; ++Id)
	{
		const FLinearColor C = UMjLidarPointCloudViz::TargetIdColor(Id);
		bInRange = bInRange
			&& C.R >= 0.0f && C.R <= 1.0f
			&& C.G >= 0.0f && C.G <= 1.0f
			&& C.B >= 0.0f && C.B <= 1.0f;
	}
	TestTrue(TEXT("all components in [0,1]"), bInRange);
	return true;
}

// ============================================================================
// URLab.Lidar.Viz.SubsampleStride_Bounds
//   Stride is 1 at or under budget, ceil(Total/Max) beyond it.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLidarVizSubsampleStrideTest,
	"URLab.Lidar.Viz.SubsampleStride_Bounds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjLidarVizSubsampleStrideTest::RunTest(const FString& Parameters)
{
	TestEqual(TEXT("empty input"), UMjLidarPointCloudViz::SubsampleStride(0, 100), 1);
	TestEqual(TEXT("under budget"), UMjLidarPointCloudViz::SubsampleStride(100, 200), 1);
	TestEqual(TEXT("exactly at budget"), UMjLidarPointCloudViz::SubsampleStride(200, 200), 1);
	TestEqual(TEXT("just over budget"), UMjLidarPointCloudViz::SubsampleStride(201, 200), 2);
	TestEqual(TEXT("default cloud vs default batched budget"), UMjLidarPointCloudViz::SubsampleStride(65536, 20000), 4);
	TestEqual(TEXT("tiny budget"), UMjLidarPointCloudViz::SubsampleStride(10, 3), 4);
	TestEqual(TEXT("zero budget guard"), UMjLidarPointCloudViz::SubsampleStride(5, 0), 5);
	return true;
}

// ============================================================================
// URLab.Lidar.Viz.BuildPly_HeaderAndFirstVertex
//   PLY header fields and metre-converted vertices are present.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLidarVizBuildPlyTest,
	"URLab.Lidar.Viz.BuildPly_HeaderAndFirstVertex",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjLidarVizBuildPlyTest::RunTest(const FString& Parameters)
{
	TArray<FVector> Positions;
	TArray<FLinearColor> Colors;
	Positions.Add(FVector(123.0f, 456.0f, 789.0f));
	Colors.Add(FLinearColor(1.0f, 0.0f, 0.0f));
	Positions.Add(FVector(-50.0f, 0.0f, 25.0f));
	Colors.Add(FLinearColor(0.0f, 1.0f, 0.0f));

	const FString Ply = UMjLidarPointCloudViz::BuildPlyText(Positions, Colors);

	TestTrue(TEXT("ply magic"), Ply.StartsWith(TEXT("ply\n")));
	TestTrue(TEXT("ascii format"), Ply.Contains(TEXT("format ascii 1.0\n")));
	TestTrue(TEXT("vertex count"), Ply.Contains(TEXT("element vertex 2\n")));
	TestTrue(TEXT("x property"), Ply.Contains(TEXT("property float x\n")));
	TestTrue(TEXT("rgb properties"),
		Ply.Contains(TEXT("property uchar red\n"))
		&& Ply.Contains(TEXT("property uchar green\n"))
		&& Ply.Contains(TEXT("property uchar blue\n")));
	TestTrue(TEXT("header ends"), Ply.Contains(TEXT("end_header\n")));

	// Vertices are centimetres converted to metres, then uchar RGB.
	TestTrue(TEXT("first vertex line"), Ply.Contains(TEXT("1.230000 4.560000 7.890000 255 0 0\n")));
	TestTrue(TEXT("second vertex line"), Ply.Contains(TEXT("-0.500000 0.000000 0.250000 0 255 0\n")));

	// Empty input yields a valid, zero-vertex header.
	const FString EmptyPly = UMjLidarPointCloudViz::BuildPlyText(TArray<FVector>(), TArray<FLinearColor>());
	TestTrue(TEXT("empty cloud vertex count"), EmptyPly.Contains(TEXT("element vertex 0\n")));
	return true;
}
