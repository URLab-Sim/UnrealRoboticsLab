// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"
#include "Tests/MjTestHelpers.h"
#include "MuJoCo/Components/Bodies/MjWorldBody.h"
#include "MuJoCo/Components/Geometry/MjGeom.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "mujoco/mujoco.h"

// ============================================================================
// URLab.Import.Issue72Includes
//   Regression for https://github.com/URLab-Sim/UnrealRoboticsLab/issues/72.
//   Imports an MJCF split across <include> fragments (rooted in <mujocoinclude>),
//   including one pulled in from inside <worldbody> and two top-level fragments
//   that carry their own <worldbody>/<asset>/<compiler> — the gym-aloha shape.
//
//   Before the fix this crashed: a second "worldbody" SCS node was created for
//   the included scene's <worldbody> and renamed onto the existing one (fatal).
//   It also dropped <asset>/<compiler> declared inside <mujocoinclude> roots.
//
//   Reaching any assertion proves the import did not crash. We then check there
//   is exactly one worldbody node and that the mesh declared in the included
//   <asset> survived into the compiled model (nmesh == 1).
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjImportIssue72Includes,
	"URLab.Import.Issue72Includes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjImportIssue72Includes::RunTest(const FString& Parameters)
{
	const FString XmlPath = FPaths::Combine(FPaths::ProjectPluginsDir(),
		TEXT("UnrealRoboticsLab/Content/TestData/issue72_includes/parent.xml"));

	if (!FPaths::FileExists(XmlPath))
	{
		AddError(FString::Printf(TEXT("Fixture not found: %s"), *XmlPath));
		return false;
	}

	FMjXmlImportSession S;
	if (!S.InitFromFile(XmlPath))
	{
		AddError(FString::Printf(TEXT("InitFromFile failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	// Native MuJoCo (includes resolved) — sanity that the fragments loaded.
	TestEqual(TEXT("native body count"), (int32)S.NativeBodyCount, 5);
	TestEqual(TEXT("native joint count"), (int32)S.NativeJointCount, 2);
	TestEqual(TEXT("native geom count"), (int32)S.NativeGeomCount, 4);

	// Core crash fix: the included scene's <worldbody> and the model's own
	// <worldbody> must merge into a single worldbody node.
	const int32 WorldBodies = S.CountTemplates<UMjWorldBody>();
	TestEqual(TEXT("exactly one worldbody node"), WorldBodies, 1);

	// Structure traversal across both styles of include:
	//  - 'table' comes from a top-level <mujocoinclude> <worldbody>
	//  - 'arm'   comes from a <mujocoinclude> included INSIDE <worldbody>
	TestNotNull(TEXT("table body imported (top-level mujocoinclude)"), S.FindTemplate<UMjBody>(TEXT("table")));
	TestNotNull(TEXT("arm body imported (in-worldbody mujocoinclude)"), S.FindTemplate<UMjBody>(TEXT("arm")));

	// The mesh geom referencing the asset declared in the <mujocoinclude>.
	UMjGeom* TableGeom = S.FindTemplate<UMjGeom>(TEXT("table_geom"));
	TestNotNull(TEXT("table_geom imported"), TableGeom);
	if (TableGeom)
		TestEqual(TEXT("table_geom references 'tabletop' mesh"), TableGeom->mesh, FString(TEXT("tabletop")));

	// Compile through our pipeline. nmesh == 1 proves the <asset> mesh declared
	// inside the <mujocoinclude> was found (bug 2) and wired into the spec.
	if (!S.Compile())
	{
		AddError(FString::Printf(TEXT("Compile failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	mjModel* M = S.Model();
	TestNotNull(TEXT("compiled model valid"), M);
	if (M)
	{
		TestEqual(TEXT("compiled mesh count (asset from include)"), (int32)M->nmesh, 1);
		TestEqual(TEXT("compiled body count matches native"), (int32)M->nbody, (int32)S.NativeBodyCount);
		TestEqual(TEXT("compiled geom count matches native"), (int32)M->ngeom, (int32)S.NativeGeomCount);
		TestEqual(TEXT("compiled joint count matches native"), (int32)M->njnt, (int32)S.NativeJointCount);

		if (mjData* D = S.Data())
		{
			mj_step(M, D);
			mj_step(M, D);
		}
	}

	S.Cleanup();
	return true;
}

// ============================================================================
// URLab.Import.Issue72AlohaReal
//   Optional: import the actual gym-aloha file from issue #72 to confirm the fix
//   on the real model. Skipped unless URLAB_ALOHA_XML points at a copy of
//   bimanual_viperx_end_effector_transfer_cube.xml (raw, with includes) or its
//   flattened _ue.xml. Not committed because the arm meshes are large.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjImportIssue72AlohaReal,
	"URLab.Import.Issue72AlohaReal",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjImportIssue72AlohaReal::RunTest(const FString& Parameters)
{
	const FString XmlPath = FPlatformMisc::GetEnvironmentVariable(TEXT("URLAB_ALOHA_XML"));
	if (XmlPath.IsEmpty())
	{
		AddWarning(TEXT("Skipping: URLAB_ALOHA_XML env var not set"));
		return true;
	}
	if (!FPaths::FileExists(XmlPath))
	{
		AddWarning(FString::Printf(TEXT("Skipping: file not found at %s"), *XmlPath));
		return true;
	}

	FMjXmlImportSession S;
	if (!S.InitFromFile(XmlPath))
	{
		AddError(FString::Printf(TEXT("InitFromFile failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}

	AddInfo(FString::Printf(TEXT("Native: %d bodies, %d joints, %d geoms"),
		(int32)S.NativeBodyCount, (int32)S.NativeJointCount, (int32)S.NativeGeomCount));

	TestEqual(TEXT("exactly one worldbody node"), S.CountTemplates<UMjWorldBody>(), 1);
	TestTrue(TEXT("imported some bodies"), S.CountTemplates<UMjBody>() >= 1);

	if (!S.Compile())
	{
		AddError(FString::Printf(TEXT("Compile failed: %s"), *S.LastError));
		S.Cleanup();
		return false;
	}
	if (mjModel* M = S.Model())
	{
		AddInfo(FString::Printf(TEXT("Compiled: %d bodies, %d joints, %d geoms, %d meshes"),
			M->nbody, M->njnt, M->ngeom, M->nmesh));
		TestEqual(TEXT("compiled body count matches native"), (int32)M->nbody, (int32)S.NativeBodyCount);
		TestEqual(TEXT("compiled geom count matches native"), (int32)M->ngeom, (int32)S.NativeGeomCount);
	}

	S.Cleanup();
	return true;
}
