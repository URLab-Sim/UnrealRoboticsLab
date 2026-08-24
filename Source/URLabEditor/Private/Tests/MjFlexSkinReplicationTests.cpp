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
// endorsed by, or sponsored by Epic Games, Inc.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

// ============================================================================
// MjFlexSkinReplicationTests.cpp
//
// Tier-1 GAPs 1.h / 1.i (UE): the RUNTIME flex/skin replication on a mirror --
//   UMjFlexcomp::UpdateFromBodyTransforms (mj_flex, both interp modes) and
//   UMjSkincomp::UpdateFromBodyTransforms (CPU LBS vs mjv_updateActiveSkin) --
// asserted vertex-by-vertex against the streamed per-body transforms.
//
// These three tests previously WARNED and returned (assert-nothing stubs). The
// stubs' premise -- that UpdateFromBodyTransforms cannot run headlessly because
// it needs a child UStaticMeshComponent surface -- holds only for an AUTHORED
// flexcomp. A MIRROR element (SetMirrorFlex / SetSkinId, the renderer-created
// path, MjFlexcomp.cpp:2513 / MjSkincomp) builds its OWN UDynamicMesh surface
// from the compiled model's topology on the first drive (BuildModelSurface,
// MjFlexcomp.cpp:699-712; CreateProceduralMesh, MjSkincomp), so the exact code
// path a stream/push mirror runs IS reachable from a headless automation test.
// That is what these tests now exercise: a mirror element driven by hand-built
// bxpos/bxquat, its deformed UDynamicMesh read back vertex-by-vertex.
// ============================================================================

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "Components/DynamicMeshComponent.h"
#include "Components/SceneComponent.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMesh/DynamicMeshAttributeSet.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "UDynamicMesh.h"

#include "MuJoCo/Elements/MjFlexcomp.h"
#include "MuJoCo/Elements/MjSkincomp.h"

THIRD_PARTY_INCLUDES_START
#include "mujoco/mujoco.h"
THIRD_PARTY_INCLUDES_END

namespace MjFlexSkinReplTest
{
// ---------------------------------------------------------------------------
// A throwaway model compiled from inline MJCF, plus a rest mjData for its
// per-body transforms. The mirror flex/skin math is a pure function of the
// static model x the streamed per-body transforms, so a rest mj_forward is all
// the reference pose the tests need -- the deformation is applied to the
// transforms directly (no stepping), exactly as an owner's broadcast would.
// ---------------------------------------------------------------------------
struct FModel
{
	mjModel* m = nullptr;
	mjData* d = nullptr;
	FString Err;

	bool Compile(const char* Xml)
	{
		char szErr[1000] = "";
		mjSpec* Spec = mj_parseXMLString(Xml, nullptr, szErr, sizeof(szErr));
		if (!Spec)
		{
			Err = FString::Printf(TEXT("mj_parseXMLString: %hs"), szErr);
			return false;
		}
		m = mj_compile(Spec, nullptr);
		if (!m)
		{
			const char* e = mjs_getError(Spec);
			Err = FString::Printf(TEXT("mj_compile: %hs"), e ? e : "unknown");
			mj_deleteSpec(Spec);
			return false;
		}
		mj_deleteSpec(Spec);
		d = mj_makeData(m);
		if (!d)
		{
			Err = TEXT("mj_makeData null");
			return false;
		}
		mj_forward(m, d);
		return true;
	}

	// Rest per-body transforms straight off mjData (the wire's bxpos/bxquat).
	void RestTransforms(TArray<double>& Bxpos, TArray<double>& Bxquat) const
	{
		const int32 NB = m->nbody;
		Bxpos.SetNumUninitialized(3 * NB);
		Bxquat.SetNumUninitialized(4 * NB);
		FMemory::Memcpy(Bxpos.GetData(), d->xpos, sizeof(double) * 3 * NB);
		FMemory::Memcpy(Bxquat.GetData(), d->xquat, sizeof(double) * 4 * NB);
	}

	~FModel()
	{
		if (d)
		{
			mj_deleteData(d);
		}
		if (m)
		{
			mj_deleteModel(m);
		}
	}
};

// A world with one actor carrying a scene root, to host mirror components.
struct FHost
{
	UWorld* World = nullptr;
	AActor* Actor = nullptr;

	bool Init()
	{
		World = UWorld::CreateWorld(EWorldType::Game, false);
		if (!World || !GEngine)
		{
			return false;
		}
		FWorldContext& Ctx = GEngine->CreateNewWorldContext(EWorldType::Game);
		Ctx.SetCurrentWorld(World);
		Actor = World->SpawnActor<AActor>();
		if (!Actor)
		{
			return false;
		}
		USceneComponent* Root = NewObject<USceneComponent>(Actor, TEXT("Root"));
		Actor->SetRootComponent(Root);
		Root->RegisterComponent();
		return true;
	}

	void Cleanup()
	{
		if (World)
		{
			World->DestroyWorld(false);
			GEngine->DestroyWorldContext(World);
			World = nullptr;
			Actor = nullptr;
		}
	}

	~FHost() { Cleanup(); }
};

// Rotate every per-body position about the centroid of the flex-driving bodies,
// by Angle radians about MuJoCo +Z. In centered vertex/trilinear mode (the grid
// flexcomp case) the render vertices track body/node POSITIONS, so this rigidly
// rotates the reconstructed surface: the per-frame normals must rotate WITH it
// (recompute) and stay OUTWARD -- a frozen-normal bug would leave them pointing
// the wrong way after the rotation.
void RotateAboutCentroidZ(TArray<double>& Bxpos, double Angle)
{
	const int32 NV = Bxpos.Num() / 3;
	double cx = 0, cy = 0;
	for (int32 i = 0; i < NV; ++i)
	{
		cx += Bxpos[3 * i + 0];
		cy += Bxpos[3 * i + 1];
	}
	cx /= FMath::Max(1, NV);
	cy /= FMath::Max(1, NV);
	const double c = FMath::Cos(Angle);
	const double s = FMath::Sin(Angle);
	for (int32 i = 0; i < NV; ++i)
	{
		const double x = Bxpos[3 * i + 0] - cx;
		const double y = Bxpos[3 * i + 1] - cy;
		Bxpos[3 * i + 0] = cx + x * c - y * s;
		Bxpos[3 * i + 1] = cy + x * s + y * c;
	}
}

// Read a UDynamicMeshComponent's vertex positions + per-vertex normals (the
// PrimaryNormals overlay whose element id == raw vertex id, both flex and skin
// writebacks). Positions are in mesh-local space (== world for an identity host).
bool ReadMesh(UDynamicMeshComponent* Comp, TArray<FVector>& OutVerts, TArray<FVector>& OutNormals)
{
	if (!Comp || !Comp->GetDynamicMesh())
	{
		return false;
	}
	OutVerts.Reset();
	OutNormals.Reset();
	Comp->GetDynamicMesh()->ProcessMesh([&](const UE::Geometry::FDynamicMesh3& Mesh) {
		const UE::Geometry::FDynamicMeshNormalOverlay* N =
			Mesh.HasAttributes() ? Mesh.Attributes()->PrimaryNormals() : nullptr;
		for (int32 Vid = 0; Vid < Mesh.MaxVertexID(); ++Vid)
		{
			if (!Mesh.IsVertex(Vid))
			{
				continue;
			}
			const FVector3d P = Mesh.GetVertex(Vid);
			OutVerts.Add(FVector(P.X, P.Y, P.Z));
			FVector Nn = FVector::ZeroVector;
			if (N && N->IsElement(Vid))
			{
				const FVector3f E = N->GetElement(Vid);
				Nn = FVector(E.X, E.Y, E.Z);
			}
			OutNormals.Add(Nn);
		}
	});
	return OutVerts.Num() > 0;
}

// Fraction of vertex normals that point away from the mesh centroid (outward
// winding). A closed convex-ish flex volume should be ~1.0.
double OutwardFraction(const TArray<FVector>& Verts, const TArray<FVector>& Normals)
{
	if (Verts.Num() == 0)
	{
		return 0.0;
	}
	FVector C = FVector::ZeroVector;
	for (const FVector& V : Verts)
	{
		C += V;
	}
	C /= Verts.Num();
	int32 Outward = 0;
	int32 Counted = 0;
	for (int32 i = 0; i < Verts.Num(); ++i)
	{
		const FVector Radial = (Verts[i] - C);
		if (Radial.IsNearlyZero() || Normals[i].IsNearlyZero())
		{
			continue;
		}
		++Counted;
		if (FVector::DotProduct(Radial.GetSafeNormal(), Normals[i].GetSafeNormal()) > 0.0)
		{
			++Outward;
		}
	}
	return Counted > 0 ? (double)Outward / (double)Counted : 0.0;
}

} // namespace MjFlexSkinReplTest

// ---------------------------------------------------------------------------
// URLab.Flexcomp.ReplicatesMjFlexVertexMode  (vertex mode, flex_interp==0)
//
// A 3x3x3 grid flexcomp is a closed volumetric surface whose 27 vertices each
// track one body (flex_interp==0, centered). A mirror UMjFlexcomp builds the
// surface from the model at rest, then a rigid rotation of the driving bodies
// must rotate the surface, its normals must recompute per-frame (differ from
// rest) and stay outward.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjFlexcompReplicatesMjFlexVertexMode,
	"URLab.Flexcomp.ReplicatesMjFlexVertexMode",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjFlexcompReplicatesMjFlexVertexMode::RunTest(const FString& Parameters)
{
	using namespace MjFlexSkinReplTest;

	FModel Mdl;
	const bool bCompiled = Mdl.Compile(
		"<mujoco>"
		"  <size memory=\"10M\"/>"
		"  <worldbody>"
		"    <flexcomp name=\"vflex\" type=\"grid\" count=\"3 3 3\" spacing=\"0.1 0.1 0.1\""
		"              pos=\"0 0 0\" dim=\"3\" radius=\"0.005\" mass=\"1\">"
		"      <contact selfcollide=\"none\" internal=\"false\"/>"
		"    </flexcomp>"
		"  </worldbody>"
		"</mujoco>");
	if (!bCompiled)
	{
		AddError(FString::Printf(TEXT("compile failed: %s"), *Mdl.Err));
		return false;
	}
	if (!TestTrue(TEXT("model has a flex"), Mdl.m->nflex >= 1))
	{
		return false;
	}
	TestEqual(TEXT("flex is vertex-interp (flex_interp==0)"), (int32)Mdl.m->flex_interp[0], 0);
	const int32 FlexVertNum = Mdl.m->flex_vertnum[0];
	TestEqual(TEXT("3x3x3 grid has 27 flex vertices"), FlexVertNum, 27);

	FHost H;
	if (!TestTrue(TEXT("host world/actor"), H.Init()))
	{
		return false;
	}

	UMjFlexcomp* FC = NewObject<UMjFlexcomp>(H.Actor, TEXT("MirrorFlex"));
	H.Actor->AddInstanceComponent(FC);
	FC->SetMirrorFlex(0);
	FC->SetupAttachment(H.Actor->GetRootComponent());
	FC->RegisterComponent();

	const int32 NB = Mdl.m->nbody;
	TArray<double> Bxpos, Bxquat;
	Mdl.RestTransforms(Bxpos, Bxquat);

	// First drive builds the surface at the rest pose.
	FC->UpdateFromBodyTransforms(*Mdl.m, Bxpos.GetData(), Bxquat.GetData(), NB, FVector::ZeroVector);
	if (!TestNotNull(TEXT("mirror flex built a UDynamicMesh"), FC->DynamicMesh.Get()))
	{
		return false;
	}

	TArray<FVector> RestVerts, RestNormals;
	if (!TestTrue(TEXT("read rest mesh"), ReadMesh(FC->DynamicMesh, RestVerts, RestNormals)))
	{
		return false;
	}
	// The mirror weld map is the identity (BuildModelSurface: NumRenderVerts ==
	// FlexVertNum), so the built surface has exactly the model's flex vertices.
	TestEqual(TEXT("dynamic mesh has the model's flex vertex count"), RestVerts.Num(), FlexVertNum);

	const double RestOutward = OutwardFraction(RestVerts, RestNormals);
	TestTrue(FString::Printf(TEXT("rest normals are outward (got %.2f)"), RestOutward),
		RestOutward > 0.9);

	// Deform: rigidly rotate the driving bodies 45 deg about Z, re-drive.
	TArray<double> DefPos = Bxpos;
	RotateAboutCentroidZ(DefPos, PI / 4.0);
	FC->UpdateFromBodyTransforms(*Mdl.m, DefPos.GetData(), Bxquat.GetData(), NB, FVector::ZeroVector);

	TArray<FVector> DefVerts, DefNormals;
	if (!TestTrue(TEXT("read deformed mesh"), ReadMesh(FC->DynamicMesh, DefVerts, DefNormals)))
	{
		return false;
	}
	TestEqual(TEXT("vertex count is stable across a deform"), DefVerts.Num(), RestVerts.Num());

	// Vertices moved (the surface follows the streamed transforms).
	int32 MovedVerts = 0;
	for (int32 i = 0; i < DefVerts.Num(); ++i)
	{
		if (FVector::Dist(DefVerts[i], RestVerts[i]) > 1.0) // > 1 cm
		{
			++MovedVerts;
		}
	}
	TestTrue(FString::Printf(TEXT("streamed transforms deform the surface (%d/%d verts moved)"),
				 MovedVerts, DefVerts.Num()),
		MovedVerts > DefVerts.Num() / 2);

	// Normals recomputed this frame: a 45-deg rotation turns the side-face normals,
	// so many differ from rest by a large angle. (A frozen-normal bug would leave
	// them at the rest orientation.)
	int32 TurnedNormals = 0;
	for (int32 i = 0; i < DefNormals.Num(); ++i)
	{
		if (DefNormals[i].IsNearlyZero() || RestNormals[i].IsNearlyZero())
		{
			continue;
		}
		const double Dot = FVector::DotProduct(DefNormals[i].GetSafeNormal(), RestNormals[i].GetSafeNormal());
		if (Dot < FMath::Cos(FMath::DegreesToRadians(10.0)))
		{
			++TurnedNormals;
		}
	}
	TestTrue(FString::Printf(TEXT("normals recompute per-frame (%d turned > 10 deg)"), TurnedNormals),
		TurnedNormals > DefNormals.Num() / 3);

	// And after the deform the recomputed normals are STILL outward -- the winding
	// followed the rotation rather than freezing.
	const double DefOutward = OutwardFraction(DefVerts, DefNormals);
	TestTrue(FString::Printf(TEXT("deformed normals stay outward (got %.2f)"), DefOutward),
		DefOutward > 0.9);

	return true;
}

// ---------------------------------------------------------------------------
// URLab.Flexcomp.ReplicatesMjFlexTrilinear  (trilinear cage, flex_interp>=1)
//
// A dof="trilinear" grid lowers to an 8-node cage (order 1) that drives every
// render vertex by mju_interpolate3D. A mirror UMjFlexcomp takes the trilinear
// branch of UpdateFromBodyTransforms: rotating the cage node bodies rotates the
// interpolated surface, normals recompute and stay outward.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjFlexcompReplicatesMjFlexTrilinear,
	"URLab.Flexcomp.ReplicatesMjFlexTrilinear",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjFlexcompReplicatesMjFlexTrilinear::RunTest(const FString& Parameters)
{
	using namespace MjFlexSkinReplTest;

	FModel Mdl;
	const bool bCompiled = Mdl.Compile(
		"<mujoco>"
		"  <size memory=\"20M\"/>"
		"  <worldbody>"
		"    <flexcomp name=\"tflex\" type=\"grid\" count=\"4 4 4\" spacing=\"0.07 0.07 0.07\""
		"              pos=\"0 0 0\" dim=\"3\" radius=\"0.005\" mass=\"1\" dof=\"trilinear\">"
		"      <elasticity young=\"1e4\" poisson=\"0.1\"/>"
		"      <contact selfcollide=\"none\" internal=\"false\"/>"
		"    </flexcomp>"
		"  </worldbody>"
		"</mujoco>");
	if (!bCompiled)
	{
		AddError(FString::Printf(TEXT("compile failed: %s"), *Mdl.Err));
		return false;
	}
	if (!TestTrue(TEXT("model has a flex"), Mdl.m->nflex >= 1))
	{
		return false;
	}
	// dof=trilinear lowers to interpolation order >= 1 (8-node cage for order 1).
	if (!TestTrue(FString::Printf(TEXT("flex is trilinear (flex_interp=%d >= 1)"),
				 (int32)Mdl.m->flex_interp[0]),
			Mdl.m->flex_interp[0] >= 1))
	{
		return false;
	}
	TestTrue(TEXT("trilinear cage has driving node bodies"), Mdl.m->flex_nodenum[0] > 0);

	FHost H;
	if (!TestTrue(TEXT("host world/actor"), H.Init()))
	{
		return false;
	}

	UMjFlexcomp* FC = NewObject<UMjFlexcomp>(H.Actor, TEXT("MirrorFlexTri"));
	H.Actor->AddInstanceComponent(FC);
	FC->SetMirrorFlex(0);
	FC->SetupAttachment(H.Actor->GetRootComponent());
	FC->RegisterComponent();

	const int32 NB = Mdl.m->nbody;
	TArray<double> Bxpos, Bxquat;
	Mdl.RestTransforms(Bxpos, Bxquat);

	FC->UpdateFromBodyTransforms(*Mdl.m, Bxpos.GetData(), Bxquat.GetData(), NB, FVector::ZeroVector);
	if (!TestNotNull(TEXT("mirror trilinear flex built a UDynamicMesh"), FC->DynamicMesh.Get()))
	{
		return false;
	}

	TArray<FVector> RestVerts, RestNormals;
	if (!TestTrue(TEXT("read rest mesh"), ReadMesh(FC->DynamicMesh, RestVerts, RestNormals)))
	{
		return false;
	}
	TestTrue(TEXT("trilinear surface built vertices"), RestVerts.Num() > 0);
	const double RestOutward = OutwardFraction(RestVerts, RestNormals);
	TestTrue(FString::Printf(TEXT("rest normals outward (got %.2f)"), RestOutward), RestOutward > 0.85);

	// Rotate the cage (all bodies) 45 deg about Z; the interpolated surface rotates.
	TArray<double> DefPos = Bxpos;
	RotateAboutCentroidZ(DefPos, PI / 4.0);
	FC->UpdateFromBodyTransforms(*Mdl.m, DefPos.GetData(), Bxquat.GetData(), NB, FVector::ZeroVector);

	TArray<FVector> DefVerts, DefNormals;
	if (!TestTrue(TEXT("read deformed mesh"), ReadMesh(FC->DynamicMesh, DefVerts, DefNormals)))
	{
		return false;
	}
	TestEqual(TEXT("vertex count stable"), DefVerts.Num(), RestVerts.Num());

	int32 MovedVerts = 0;
	for (int32 i = 0; i < DefVerts.Num(); ++i)
	{
		if (FVector::Dist(DefVerts[i], RestVerts[i]) > 1.0)
		{
			++MovedVerts;
		}
	}
	TestTrue(FString::Printf(TEXT("trilinear cage deforms the surface (%d/%d verts moved)"),
				 MovedVerts, DefVerts.Num()),
		MovedVerts > DefVerts.Num() / 2);

	int32 TurnedNormals = 0;
	for (int32 i = 0; i < DefNormals.Num(); ++i)
	{
		if (DefNormals[i].IsNearlyZero() || RestNormals[i].IsNearlyZero())
		{
			continue;
		}
		const double Dot = FVector::DotProduct(DefNormals[i].GetSafeNormal(), RestNormals[i].GetSafeNormal());
		if (Dot < FMath::Cos(FMath::DegreesToRadians(10.0)))
		{
			++TurnedNormals;
		}
	}
	TestTrue(FString::Printf(TEXT("trilinear normals recompute per-frame (%d turned)"), TurnedNormals),
		TurnedNormals > DefNormals.Num() / 3);

	const double DefOutward = OutwardFraction(DefVerts, DefNormals);
	TestTrue(FString::Printf(TEXT("deformed normals stay outward (got %.2f)"), DefOutward),
		DefOutward > 0.85);

	return true;
}

// ---------------------------------------------------------------------------
// URLab.Skin.LbsMatchesUpdateActiveSkin  (CPU LBS vs mjv_updateActiveSkin)
//
// A 4-vertex quad skinned to two single-weight bones: the left edge (verts 0,2)
// tracks body b0, the right edge (verts 1,3) tracks body b1. With identity bind
// quats a weight-1 vertex is the LBS identity -- it follows its bone rigidly
// (mjv_updateActiveSkin, engine_vis_visualize.c:3258-3316). Moving b1's streamed
// transform must displace verts 1,3 by exactly that amount and leave 0,2 fixed.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjSkinLbsMatchesUpdateActiveSkin,
	"URLab.Skin.LbsMatchesUpdateActiveSkin",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjSkinLbsMatchesUpdateActiveSkin::RunTest(const FString& Parameters)
{
	using namespace MjFlexSkinReplTest;

	FModel Mdl;
	const bool bCompiled = Mdl.Compile(
		// NOTE: <skin> is an <asset> element (not a <worldbody> child); its <bone>
		// sub-elements reference worldbody bodies by name. Placing it in <worldbody>
		// is a schema violation ("unrecognized element 'skin'").
		"<mujoco>"
		"  <worldbody>"
		"    <body name=\"b0\" pos=\"0 0 0\"><geom type=\"sphere\" size=\"0.02\"/></body>"
		"    <body name=\"b1\" pos=\"1 0 0\"><geom type=\"sphere\" size=\"0.02\"/></body>"
		"  </worldbody>"
		"  <asset>"
		"    <skin name=\"s\" vertex=\"0 0 0  1 0 0  0 1 0  1 1 0\" face=\"0 1 2  1 3 2\">"
		"      <bone body=\"b0\" bindpos=\"0 0 0\" bindquat=\"1 0 0 0\" vertid=\"0 2\" vertweight=\"1 1\"/>"
		"      <bone body=\"b1\" bindpos=\"1 0 0\" bindquat=\"1 0 0 0\" vertid=\"1 3\" vertweight=\"1 1\"/>"
		"    </skin>"
		"  </asset>"
		"</mujoco>");
	if (!bCompiled)
	{
		AddError(FString::Printf(TEXT("compile failed: %s"), *Mdl.Err));
		return false;
	}
	if (!TestTrue(TEXT("model has a skin"), Mdl.m->nskin >= 1))
	{
		return false;
	}
	TestEqual(TEXT("skin has 4 vertices"), (int32)Mdl.m->skin_vertnum[0], 4);

	const int32 B0 = mj_name2id(Mdl.m, mjOBJ_BODY, "b0");
	const int32 B1 = mj_name2id(Mdl.m, mjOBJ_BODY, "b1");
	if (!TestTrue(TEXT("bone bodies resolved"), B0 > 0 && B1 > 0))
	{
		return false;
	}

	FHost H;
	if (!TestTrue(TEXT("host world/actor"), H.Init()))
	{
		return false;
	}

	UMjSkincomp* Skin = NewObject<UMjSkincomp>(H.Actor, TEXT("MirrorSkin"));
	H.Actor->AddInstanceComponent(Skin);
	Skin->SetSkinId(0);
	Skin->SetupAttachment(H.Actor->GetRootComponent());
	Skin->RegisterComponent();

	const int32 NB = Mdl.m->nbody;
	TArray<double> Bxpos, Bxquat;
	Mdl.RestTransforms(Bxpos, Bxquat);

	// Rest drive builds the surface and skins at the bind pose (identity LBS).
	Skin->UpdateFromBodyTransforms(*Mdl.m, Bxpos.GetData(), Bxquat.GetData(), NB, FVector::ZeroVector);
	if (!TestNotNull(TEXT("mirror skin built a UDynamicMesh"), Skin->DynamicMesh.Get()))
	{
		return false;
	}

	TArray<FVector> RestVerts, RestNormals;
	if (!TestTrue(TEXT("read rest skin mesh"), ReadMesh(Skin->DynamicMesh, RestVerts, RestNormals)))
	{
		return false;
	}
	TestEqual(TEXT("skin dynamic mesh has 4 vertices"), RestVerts.Num(), 4);

	// Move b1's streamed transform up 0.1 m in +Z; b0 stays put.
	const double Dz = 0.1;
	TArray<double> DefPos = Bxpos;
	DefPos[3 * B1 + 2] += Dz;
	Skin->UpdateFromBodyTransforms(*Mdl.m, DefPos.GetData(), Bxquat.GetData(), NB, FVector::ZeroVector);

	TArray<FVector> DefVerts, DefNormals;
	if (!TestTrue(TEXT("read deformed skin mesh"), ReadMesh(Skin->DynamicMesh, DefVerts, DefNormals)))
	{
		return false;
	}
	if (!TestEqual(TEXT("skin vertex count stable"), DefVerts.Num(), 4))
	{
		return false;
	}

	// MuJoCo metres -> UE cm is a linear coordinate map (scale 100), so the
	// b1-weighted verts move by |Dz|*100 = 10 cm and the b0-weighted verts don't.
	// skin vertex order matches skin_vert order: 0,2 -> b0 (fixed); 1,3 -> b1.
	const double ExpectedMoveCm = Dz * 100.0;
	const double Move0 = FVector::Dist(DefVerts[0], RestVerts[0]);
	const double Move2 = FVector::Dist(DefVerts[2], RestVerts[2]);
	const double Move1 = FVector::Dist(DefVerts[1], RestVerts[1]);
	const double Move3 = FVector::Dist(DefVerts[3], RestVerts[3]);

	TestTrue(FString::Printf(TEXT("b0-weighted vert 0 stays fixed (moved %.3f cm)"), Move0), Move0 < 0.5);
	TestTrue(FString::Printf(TEXT("b0-weighted vert 2 stays fixed (moved %.3f cm)"), Move2), Move2 < 0.5);
	TestTrue(FString::Printf(TEXT("b1-weighted vert 1 follows the bone (%.3f cm ~ %.1f)"),
				 Move1, ExpectedMoveCm),
		FMath::Abs(Move1 - ExpectedMoveCm) < 1.0);
	TestTrue(FString::Printf(TEXT("b1-weighted vert 3 follows the bone (%.3f cm ~ %.1f)"),
				 Move3, ExpectedMoveCm),
		FMath::Abs(Move3 - ExpectedMoveCm) < 1.0);

	return true;
}
