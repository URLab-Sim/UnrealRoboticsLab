// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// What writing and compiling an authored scene costs.
//
// Two numbers, because they answer different questions:
//
//   write     the spec to MJCF text. Not on the compile route at all any more;
//             it is what the bridge handshake and the render-farm upload are
//             handed, and the install still pays for it.
//   compile   building the authored scene and compiling the prepared model.
//
// The convention is one machine-readable
// line, averages over N runs, and no threshold asserted. A perf assertion on a
// shared machine fails for reasons that have nothing to do with the code, so
// this reports and the reader judges.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#if WITH_EDITOR

#include "MjTestHelpers.h"

#if URLAB_MJ_GEN

#include "HAL/PlatformTime.h"

#include "MuJoCo/Spec/MjSceneAssembly.h"
#include "MuJoCo/Spec/MjSceneSpec.h"

// The measurement has to reach the run's own log, and URLab's categories are
// not exported to this module, so the benchmark carries its own.
DEFINE_LOG_CATEGORY_STATIC(LogMjCompileBench, Display, All);

namespace MjCompileLatencyTests
{
/**
 * Bodies in the benchmark scene.
 *
 * Sized at a humanoid rather than at a toy: 28 bodies each carrying a geom and
 * a hinge is the shape of the robots people actually import, and the cost being
 * measured is per element.
 */
constexpr int32 BodyCount = 28;

/** Averaged over this many runs; the first is thrown away as a warm-up. */
constexpr int32 Runs = 5;

double MillisSince(double Start)
{
	return (FPlatformTime::Seconds() - Start) * 1000.0;
}

/**
 * Compile the authored participants directly, without creating a runtime.
 */
urlab::spec::FMjCompiledScene CompileSceneOffline(AAMjManager& Manager, AMjArticulation& Robot)
{
	FSceneAssembly Scene;
	Scene.SetSceneRoot(Manager.GetSceneSpec());
	FMjSceneParticipant AuthoredParticipant;
	AuthoredParticipant.Spec = Robot.GetSpec();
	AuthoredParticipant.Prefix = Robot.GetCompiledPrefix();
	Scene.Add(AuthoredParticipant.Spec, AuthoredParticipant.Prefix, AuthoredParticipant.MjPos, AuthoredParticipant.MjQuat);

	urlab::spec::FMjSceneSpecBuilder Builder;
	Builder.SetSceneRoot(Scene.GetSceneRoot());
	for (const FMjSceneParticipant& Participant : Scene.GetParticipants())
	{
		urlab::spec::FMjSceneSpecParticipant Placed;
		Placed.Spec = Participant.Spec;
		Placed.Prefix = Participant.Prefix;
		Placed.MjPos = Participant.MjPos;
		Placed.MjQuat = Participant.MjQuat;
		Builder.AddParticipant(Placed);
	}
	return Builder.Compile();
}

/** Hang `BodyCount` jointed, geom-bearing bodies off the session's world body. */
void BuildRobot(FMjUESession& Session)
{
	for (int32 i = 0; i < BodyCount; ++i)
	{
		UMjBody* NewBody = Session.Add<UMjBody>(Session.WorldBody,
			*FString::Printf(TEXT("Link%d"), i));
		if (NewBody == nullptr)
		{
			return;
		}
		if (UMjGeom* NewGeom = Session.Add<UMjGeom>(NewBody, *FString::Printf(TEXT("Shape%d"), i)))
		{
			NewGeom->SetSize({0.05});
		}
		Session.Add<UMjJoint>(NewBody, *FString::Printf(TEXT("Hinge%d"), i));
	}
}
} // namespace MjCompileLatencyTests

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjCompileLatency,
	"URLab.Perf.CompileLatency",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjCompileLatency::RunTest(const FString& Parameters)
{
	using namespace MjCompileLatencyTests;

	FMjUESession Session;
	if (!Session.Init([](FMjUESession& S) { BuildRobot(S); }))
	{
		AddError(Session.LastError);
		return false;
	}

	const mjModel* M = Session.Model();
	if (!TestNotNull(TEXT("compiled model"), M))
	{
		return false;
	}
	const int32 NBody = M->nbody;
	const int32 NJoint = M->njnt;
	const int32 NGeom = M->ngeom;

	double WriteMs = 0.0;
	double CompileMs = 0.0;
	int32 XmlBytes = 0;

	for (int32 Run = 0; Run < Runs + 1; ++Run)
	{
		const bool bWarmUp = (Run == 0);

		const double WriteStart = FPlatformTime::Seconds();
		TArray<FMjSpecDiagnostic> WriteErrors;
		const FString Xml = FSpecRef::OverActor(*Session.Robot).WriteMjcf(&WriteErrors);
		const double Write = MillisSince(WriteStart);
		if (WriteErrors.Num() > 0 || Xml.IsEmpty())
		{
			AddError(TEXT("the benchmark spec did not write"));
			return false;
		}

		const double CompileStart = FPlatformTime::Seconds();
		urlab::spec::FMjCompiledScene Compiled = CompileSceneOffline(*Session.Manager, *Session.Robot);
		const double Compile = MillisSince(CompileStart);
		if (!Compiled.IsValid())
		{
			AddError(TEXT("the benchmark spec did not compile"));
			return false;
		}

		if (!bWarmUp)
		{
			WriteMs += Write;
			CompileMs += Compile;
			XmlBytes = Xml.Len();
		}
	}

	WriteMs /= Runs;
	CompileMs /= Runs;

	// One line, greppable, in the same shape as ProtoSpec's harness so the two
	// halves of the pipeline can be read side by side. Logged rather than only
	// added to the test result, because a measurement nobody can find in the
	// run's own log has not been recorded.
	const FString Bench = FString::Printf(
		TEXT("BENCH scene=synthetic_robot n=%d nbody=%d njnt=%d ngeom=%d xml_bytes=%d ")
		TEXT("write_ms=%.3f compile_ms=%.3f"),
		Runs, NBody, NJoint, NGeom, XmlBytes, WriteMs, CompileMs);
	UE_LOG(LogMjCompileBench, Display, TEXT("%s"), *Bench);
	AddInfo(Bench);

	// Nothing here is a threshold. The assertions are that the measurement
	// happened and that it measured the pipeline rather than an early return:
	// both values are nonzero. The write is not on the compile route, so it
	// bounds neither of the other two.
	TestTrue(TEXT("the write was measured"), WriteMs > 0.0);
	TestTrue(TEXT("the compile was measured"), CompileMs > 0.0);
	return true;
}

#endif // URLAB_MJ_GEN

#endif // WITH_EDITOR
