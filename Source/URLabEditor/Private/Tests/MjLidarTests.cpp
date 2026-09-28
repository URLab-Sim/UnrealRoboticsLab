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

// Pure-logic tests for ULidarComponent: beam grid expansion, spherical
// directions, point clustering, and noise sampling. No simulation or world
// required (see MjCameraTests.cpp for the session-based style).

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Components/LidarComponent.h"
#include "MuJoCo/Components/Sensors/MjLidarOusterOS1.h"
#include "UObject/Package.h"

namespace
{
	// NOTE: prefixed Lidar* because URLabEditor is a unity build - this file
	// can share a translation unit with MjLidarVizTests.cpp, whose anonymous-
	// namespace helpers must not collide with ours.
	bool LidarNearlyEqualF(FAutomationTestBase& Test, const TCHAR* What, float Actual, float Expected, float Tolerance = 1e-4f)
	{
		return Test.TestTrue(FString::Printf(TEXT("%s (%.5f vs %.5f)"), What, Actual, Expected),
			FMath::IsNearlyEqual(Actual, Expected, Tolerance));
	}
} // namespace

// ============================================================================
// URLab.Lidar.BeamAngles_FullWrapExclusive
//   A 360-degree azimuth span is periodic: endpoint-exclusive so 0 and 360
//   do not collide.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLidarBeamAnglesFullWrapTest,
	"URLab.Lidar.BeamAngles_FullWrapExclusive",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjLidarBeamAnglesFullWrapTest::RunTest(const FString& Parameters)
{
	TArray<float> Az, El;
	ULidarComponent::BuildBeamAngles(4, 0.0f, 360.0f, 1, 0.0f, 0.0f, Az, El);

	TestEqual(TEXT("azimuth beam count"), Az.Num(), 4);
	TestEqual(TEXT("elevation beam count"), El.Num(), 1);
	TestEqual(TEXT("az[0]"), Az[0], 0.0f);
	TestEqual(TEXT("az[1]"), Az[1], 90.0f);
	TestEqual(TEXT("az[2]"), Az[2], 180.0f);
	TestEqual(TEXT("az[3]"), Az[3], 270.0f);
	TestEqual(TEXT("el[0] midpoint of [0,0]"), El[0], 0.0f);
	return true;
}

// ============================================================================
// URLab.Lidar.BeamAngles_PartialInclusive
//   A partial azimuth span includes both endpoints.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLidarBeamAnglesPartialTest,
	"URLab.Lidar.BeamAngles_PartialInclusive",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjLidarBeamAnglesPartialTest::RunTest(const FString& Parameters)
{
	TArray<float> Az, El;
	ULidarComponent::BuildBeamAngles(4, -90.0f, 90.0f, 3, -10.0f, 20.0f, Az, El);

	TestEqual(TEXT("azimuth beam count"), Az.Num(), 4);
	TestEqual(TEXT("az[0] includes start"), Az[0], -90.0f);
	TestEqual(TEXT("az[1]"), Az[1], -30.0f);
	TestEqual(TEXT("az[2]"), Az[2], 30.0f);
	TestEqual(TEXT("az[3] includes end"), Az[3], 90.0f);

	TestEqual(TEXT("elevation beam count"), El.Num(), 3);
	TestEqual(TEXT("el[0] includes start"), El[0], -10.0f);
	TestEqual(TEXT("el[1] midpoint"), El[1], 5.0f);
	TestEqual(TEXT("el[2] includes end"), El[2], 20.0f);
	return true;
}

// ============================================================================
// URLab.Lidar.BeamAngles_SingleBeamMidpoint
//   A single beam sits at the midpoint of its (possibly open) span.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLidarBeamAnglesSingleTest,
	"URLab.Lidar.BeamAngles_SingleBeamMidpoint",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjLidarBeamAnglesSingleTest::RunTest(const FString& Parameters)
{
	TArray<float> Az, El;
	ULidarComponent::BuildBeamAngles(1, 10.0f, 20.0f, 1, -45.0f, 45.0f, Az, El);

	TestEqual(TEXT("single azimuth at midpoint"), Az[0], 15.0f);
	TestEqual(TEXT("single elevation at midpoint"), El[0], 0.0f);

	// Degenerate span (start == end) resolves to that value.
	ULidarComponent::BuildBeamAngles(1, 7.0f, 7.0f, 1, 0.0f, 0.0f, Az, El);
	TestEqual(TEXT("degenerate azimuth"), Az[0], 7.0f);
	return true;
}

// ============================================================================
// URLab.Lidar.SphericalDirection_Cardinals
//   SphericalDirection follows the documented sensor-frame convention.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLidarSphericalDirectionTest,
	"URLab.Lidar.SphericalDirection_Cardinals",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjLidarSphericalDirectionTest::RunTest(const FString& Parameters)
{
	const FVector X = ULidarComponent::SphericalDirection(0.0f, 0.0f);
	LidarNearlyEqualF(*this, TEXT("az 0/el 0 -> +X"), X.X, 1.0f);
	LidarNearlyEqualF(*this, TEXT("az 0/el 0 -> +X"), X.Y, 0.0f);
	LidarNearlyEqualF(*this, TEXT("az 0/el 0 -> +X"), X.Z, 0.0f);

	const FVector Y = ULidarComponent::SphericalDirection(90.0f, 0.0f);
	LidarNearlyEqualF(*this, TEXT("az 90 -> +Y"), Y.X, 0.0f);
	LidarNearlyEqualF(*this, TEXT("az 90 -> +Y"), Y.Y, 1.0f);
	LidarNearlyEqualF(*this, TEXT("az 90 -> +Y"), Y.Z, 0.0f);

	const FVector Z = ULidarComponent::SphericalDirection(0.0f, 90.0f);
	LidarNearlyEqualF(*this, TEXT("el 90 -> +Z"), Z.X, 0.0f);
	LidarNearlyEqualF(*this, TEXT("el 90 -> +Z"), Z.Y, 0.0f);
	LidarNearlyEqualF(*this, TEXT("el 90 -> +Z"), Z.Z, 1.0f);

	const FVector Up45 = ULidarComponent::SphericalDirection(0.0f, 45.0f);
	LidarNearlyEqualF(*this, TEXT("el 45 x"), Up45.X, FMath::Sqrt(0.5f));
	LidarNearlyEqualF(*this, TEXT("el 45 z"), Up45.Z, FMath::Sqrt(0.5f));

	const FVector Diag = ULidarComponent::SphericalDirection(45.0f, 0.0f);
	LidarNearlyEqualF(*this, TEXT("az 45 x"), Diag.X, FMath::Sqrt(0.5f));
	LidarNearlyEqualF(*this, TEXT("az 45 y"), Diag.Y, FMath::Sqrt(0.5f));

	LidarNearlyEqualF(*this, TEXT("unit length"), X.Size(), 1.0f);
	LidarNearlyEqualF(*this, TEXT("unit length"), Diag.Size(), 1.0f);
	return true;
}

// ============================================================================
// URLab.Lidar.Cluster_BasicTwoClusters
//   Near points cluster together; a far point forms its own cluster.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLidarClusterBasicTest,
	"URLab.Lidar.Cluster_BasicTwoClusters",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjLidarClusterBasicTest::RunTest(const FString& Parameters)
{
	TArray<FVector> Points;
	Points.Add(FVector(0.0f, 0.0f, 0.0f));
	Points.Add(FVector(10.0f, 0.0f, 0.0f));
	Points.Add(FVector(0.0f, 10.0f, 0.0f));
	Points.Add(FVector(1000.0f, 0.0f, 0.0f));

	TArray<int32> ClusterIds;
	TArray<int32> ClusterSizes;
	ULidarComponent::ClusterPoints(Points, 50.0f, ClusterIds, ClusterSizes);

	TestEqual(TEXT("cluster count"), ClusterSizes.Num(), 2);
	const bool bFirstThreeTogether = ClusterIds[0] == ClusterIds[1] && ClusterIds[1] == ClusterIds[2];
	TestTrue(TEXT("first three points share a cluster"), bFirstThreeTogether);
	TestTrue(TEXT("far point is separate"), ClusterIds[3] != ClusterIds[0]);

	// Sizes sum to the point count.
	int32 Total = 0;
	for (const int32 S : ClusterSizes)
	{
		Total += S;
	}
	TestEqual(TEXT("sizes sum"), Total, Points.Num());

	// Distance exactly at the threshold joins (<=).
	TArray<FVector> Pair;
	Pair.Add(FVector(0.0f, 0.0f, 0.0f));
	Pair.Add(FVector(50.0f, 0.0f, 0.0f));
	ULidarComponent::ClusterPoints(Pair, 50.0f, ClusterIds, ClusterSizes);
	TestEqual(TEXT("at-threshold pair cluster count"), ClusterSizes.Num(), 1);

	// Just beyond the threshold splits.
	Pair[1] = FVector(50.01f, 0.0f, 0.0f);
	ULidarComponent::ClusterPoints(Pair, 50.0f, ClusterIds, ClusterSizes);
	TestEqual(TEXT("beyond-threshold pair cluster count"), ClusterSizes.Num(), 2);
	return true;
}

// ============================================================================
// URLab.Lidar.Cluster_EmptyAndDegenerate
//   Empty input, single point, and zero threshold behave sanely.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLidarClusterEdgeTest,
	"URLab.Lidar.Cluster_EmptyAndDegenerate",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjLidarClusterEdgeTest::RunTest(const FString& Parameters)
{
	TArray<int32> ClusterIds;
	TArray<int32> ClusterSizes;

	ULidarComponent::ClusterPoints(TArray<FVector>(), 50.0f, ClusterIds, ClusterSizes);
	TestEqual(TEXT("empty -> no clusters"), ClusterSizes.Num(), 0);

	TArray<FVector> One;
	One.Add(FVector(1.0f, 2.0f, 3.0f));
	ULidarComponent::ClusterPoints(One, 50.0f, ClusterIds, ClusterSizes);
	TestEqual(TEXT("single point -> one cluster"), ClusterSizes.Num(), 1);
	TestEqual(TEXT("single point size"), ClusterSizes[0], 1);

	TArray<FVector> Far;
	Far.Add(FVector(0.0f, 0.0f, 0.0f));
	Far.Add(FVector(500.0f, 0.0f, 0.0f));
	ULidarComponent::ClusterPoints(Far, 0.0f, ClusterIds, ClusterSizes);
	TestEqual(TEXT("degenerate threshold -> two clusters"), ClusterSizes.Num(), 2);

	// Transitive chaining: A-B close, B-C close, A-C far -> one cluster.
	TArray<FVector> Chain;
	Chain.Add(FVector(0.0f, 0.0f, 0.0f));
	Chain.Add(FVector(10.0f, 0.0f, 0.0f));
	Chain.Add(FVector(20.0f, 0.0f, 0.0f));
	ULidarComponent::ClusterPoints(Chain, 15.0f, ClusterIds, ClusterSizes);
	TestEqual(TEXT("chained points -> one cluster"), ClusterSizes.Num(), 1);
	return true;
}

// ============================================================================
// URLab.Lidar.Gaussian_DeterministicAndBounded
//   Seeded stream is deterministic; samples look Gaussian-ish.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLidarGaussianTest,
	"URLab.Lidar.Gaussian_DeterministicAndBounded",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjLidarGaussianTest::RunTest(const FString& Parameters)
{
	// Zero stddev is a no-op.
	FRandomStream StreamA(42);
	TestEqual(TEXT("zero stddev"), ULidarComponent::SampleGaussian(StreamA, 0.0f), 0.0f);

	// Determinism: same seed, same sequence.
	FRandomStream StreamB(7);
	FRandomStream StreamC(7);
	bool bSame = true;
	for (int32 i = 0; i < 64; ++i)
	{
		if (ULidarComponent::SampleGaussian(StreamB, 2.0f) != ULidarComponent::SampleGaussian(StreamC, 2.0f))
		{
			bSame = false;
			break;
		}
	}
	TestTrue(TEXT("same seed reproduces the sequence"), bSame);

	// Statistical sanity: mean near zero, samples bounded.
	FRandomStream StreamD(123);
	double Sum = 0.0;
	float MaxAbs = 0.0f;
	constexpr int32 Num = 2000;
	for (int32 i = 0; i < Num; ++i)
	{
		const float V = ULidarComponent::SampleGaussian(StreamD, 1.0f);
		Sum += V;
		MaxAbs = FMath::Max(MaxAbs, FMath::Abs(V));
	}
	const double Mean = Sum / Num;
	TestTrue(FString::Printf(TEXT("mean %.4f near 0"), Mean), FMath::Abs(Mean) < 0.1);
	TestTrue(FString::Printf(TEXT("max |x| %.3f < 6 sigma"), MaxAbs), MaxAbs < 6.0f);
	return true;
}

// ============================================================================
// URLab.Lidar.OusterOS1_Preset
//   The device subclass only sets resolution, FOV, and rates.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjLidarOusterPresetTest,
	"URLab.Lidar.OusterOS1_Preset",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjLidarOusterPresetTest::RunTest(const FString& Parameters)
{
	UMjLidarOusterOS1* Sensor = NewObject<UMjLidarOusterOS1>(GetTransientPackage(), TEXT("TestOuster"));
	TestNotNull(TEXT("sensor created"), Sensor);

	TestEqual(TEXT("1024 azimuth beams"), Sensor->AzimuthBeams, 1024);
	TestEqual(TEXT("64 elevation beams"), Sensor->ElevationBeams, 64);
	TestEqual(TEXT("azimuth start"), Sensor->AzimuthFovStart, 0.0f);
	TestEqual(TEXT("azimuth end"), Sensor->AzimuthFovEnd, 360.0f);
	TestEqual(TEXT("elevation start (+22.5)"), Sensor->ElevationFovStart, 22.5f);
	TestEqual(TEXT("elevation end (-22.5)"), Sensor->ElevationFovEnd, -22.5f);
	TestEqual(TEXT("min range"), Sensor->MinRange, 0.1f);
	TestEqual(TEXT("max range"), Sensor->MaxRange, 120.0f);
	TestEqual(TEXT("scan rate"), Sensor->ScanFrequencyHz, 10.0f);
	return true;
}
