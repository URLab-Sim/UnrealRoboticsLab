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

// ============================================================================
// MjUrdfExportTests.cpp
//
// Tests for the mjModel -> URDF exporter (FUrdfExporter) and its matched ROS
// pieces: the qpos0 /joint_states shift and the latched robot_description
// publisher. The exporter is a direct port of the Python prototype validated
// against the Franka (docs/plan_ros_urdf_port_spec.md).
//
// The whole file compiles only when ROS 2 is linked (URLAB_WITH_ROS2), so the
// ROS-off build is unaffected. Most cases need no live ROS runtime -- string
// generation and forward kinematics run against a plain mjModel; only the
// robot_description wire case gates on an available ROS context.
// ============================================================================

#if defined(URLAB_WITH_ROS2) && URLAB_WITH_ROS2

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "HAL/FileManager.h"

#include "MjTestHelpers.h"
#include "Ros/UrdfExporter.h"
#include "Transport/RosContext.h"
#include "Transport/RosPublishTransport.h"
#include "State/MjStateTypes.h"
#include "State/MjCanonicalName.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"

#include "mujoco/mujoco.h"

#include <cmath>

namespace
{
// Two-link chain welded to the world: root "base" (no joint), "link1" on a
// limited hinge with a non-zero reference (exercises the qpos0 shift), "link2"
// on a limited slide carrying a mesh geom (exercises mesh emission).
const TCHAR* kChainXml = TEXT(R"XML(
<mujoco model="chain">
  <compiler angle="radian" autolimits="true"/>
  <asset>
    <mesh name="tet" vertex="0 0 0  0.1 0 0  0 0.1 0  0 0 0.1"/>
  </asset>
  <worldbody>
    <body name="base" pos="0 0 0.1">
      <geom name="g_base" type="box" size="0.05 0.05 0.05"/>
      <body name="link1" pos="0.2 0 0.03">
        <joint name="j1" type="hinge" axis="0 0 1" range="-1 2" ref="0.5"/>
        <geom name="g_box" type="box" size="0.03 0.03 0.03" pos="0.01 0 0"/>
        <body name="link2" pos="0.15 0 0">
          <joint name="j2" type="slide" axis="1 0 0" range="-0.1 0.2"/>
          <geom name="g_mesh" type="mesh" mesh="tet" pos="0.05 0 0.02"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
)XML");

// A jnt_pos anchor (non-zero) plus a two-joint wrist body, so the frame-shift
// and the multi-joint dummy-link chain are both exercised.
const TCHAR* kAnchorXml = TEXT(R"XML(
<mujoco model="anchor">
  <compiler angle="radian" autolimits="true"/>
  <worldbody>
    <body name="root" pos="0 0 0.2">
      <geom name="g_root" type="box" size="0.05 0.05 0.05"/>
      <body name="anchor" pos="0.1 0 0">
        <joint name="ja" type="hinge" axis="0 0 1" pos="0.05 0 0" range="-1 1"/>
        <geom name="g_anchor" type="box" size="0.02 0.02 0.02"/>
        <body name="wrist" pos="0.1 0 0">
          <joint name="jw1" type="hinge" axis="1 0 0" range="-1 1"/>
          <joint name="jw2" type="hinge" axis="0 1 0" range="-1 1"/>
          <geom name="g_wrist" type="box" size="0.02 0.02 0.02" pos="0.03 0 0"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
)XML");

// --- double quaternion (w,x,y,z) helpers for the FK self-test -------------

struct FQd { double w, x, y, z; };

FQd QMul(const FQd& A, const FQd& B)
{
	return {
		A.w * B.w - A.x * B.x - A.y * B.y - A.z * B.z,
		A.w * B.x + A.x * B.w + A.y * B.z - A.z * B.y,
		A.w * B.y - A.x * B.z + A.y * B.w + A.z * B.x,
		A.w * B.z + A.x * B.y - A.y * B.x + A.z * B.w};
}

FQd QConj(const FQd& Q) { return {Q.w, -Q.x, -Q.y, -Q.z}; }

FVector QRot(const FQd& Q, const FVector& V)
{
	const double Tx = 2.0 * (Q.y * V.Z - Q.z * V.Y);
	const double Ty = 2.0 * (Q.z * V.X - Q.x * V.Z);
	const double Tz = 2.0 * (Q.x * V.Y - Q.y * V.X);
	return FVector(
		V.X + Q.w * Tx + (Q.y * Tz - Q.z * Ty),
		V.Y + Q.w * Ty + (Q.z * Tx - Q.x * Tz),
		V.Z + Q.w * Tz + (Q.x * Ty - Q.y * Tx));
}

// URDF rpy (extrinsic XYZ) -> quaternion: R = Rz(yaw)*Ry(pitch)*Rx(roll).
FQd QFromRpy(const FVector& Rpy)
{
	const double R = Rpy.X * 0.5, P = Rpy.Y * 0.5, Y = Rpy.Z * 0.5;
	const FQd Qx{std::cos(R), std::sin(R), 0, 0};
	const FQd Qy{std::cos(P), 0, std::sin(P), 0};
	const FQd Qz{std::cos(Y), 0, 0, std::sin(Y)};
	return QMul(Qz, QMul(Qy, Qx));
}

struct FPose { FVector Pos = FVector::ZeroVector; FQd Rot{1, 0, 0, 0}; };

// Reproduce mjModel geom_xpos from the exported URDF at the zero pose (q=0 ==
// qpos0) and check every emitted geom lands within Tol metres of MuJoCo,
// rebased into the URDF root-link frame. This validates the joint origins, the
// axes, the qpos0 shift, the jnt_pos frame-shift and the dummy chain together.
bool CheckForwardKinematics(FAutomationTestBase& Test, const FUrdfModel& Model,
	mjModel* m, mjData* d, double Tol)
{
	mj_resetData(m, d);   // qpos <- qpos0
	mj_forward(m, d);

	// Root body of the model = the body with world as parent among the exported
	// links. Rebase everything into its frame.
	int RootBodyId = -1;
	for (int i = 1; i < m->nbody; ++i)
	{
		if (m->body_parentid[i] == 0) { RootBodyId = i; break; }
	}
	if (RootBodyId < 0)
	{
		Test.AddError(TEXT("no world-rooted body"));
		return false;
	}
	const FVector RootPos(d->xpos[3 * RootBodyId + 0], d->xpos[3 * RootBodyId + 1],
		d->xpos[3 * RootBodyId + 2]);
	const FQd RootQuat{d->xquat[4 * RootBodyId + 0], d->xquat[4 * RootBodyId + 1],
		d->xquat[4 * RootBodyId + 2], d->xquat[4 * RootBodyId + 3]};
	const FQd RootConj = QConj(RootQuat);

	// Link world poses in the root frame. Root links start at identity.
	TMap<FString, FPose> LinkWorld;
	for (const FUrdfLink& L : Model.Links)
		LinkWorld.Add(L.Name, FPose());

	// Joints are emitted in topological order, so a single pass resolves the tree.
	for (const FUrdfJoint& J : Model.Joints)
	{
		const FPose* Parent = LinkWorld.Find(J.Parent);
		if (!Parent)
		{
			Test.AddError(FString::Printf(TEXT("joint %s: unknown parent %s"), *J.Name, *J.Parent));
			return false;
		}
		FPose Child;
		Child.Pos = Parent->Pos + QRot(Parent->Rot, J.OriginPos);
		Child.Rot = QMul(Parent->Rot, QFromRpy(J.OriginRpy));
		LinkWorld.Add(J.Child, Child);
	}

	bool bOk = true;
	int Checked = 0;
	for (const FUrdfLink& L : Model.Links)
	{
		const FPose& LW = LinkWorld[L.Name];
		for (const FUrdfGeomFrame& G : L.Geoms)
		{
			const FVector UrdfWorld = LW.Pos + QRot(LW.Rot, G.LocalPos);
			const FVector MjWorld(d->geom_xpos[3 * G.MjGeomId + 0],
				d->geom_xpos[3 * G.MjGeomId + 1], d->geom_xpos[3 * G.MjGeomId + 2]);
			const FVector MjRebased = QRot(RootConj, MjWorld - RootPos);
			const double Err = (UrdfWorld - MjRebased).Size();
			++Checked;
			if (Err > Tol)
			{
				bOk = false;
				Test.AddError(FString::Printf(
					TEXT("geom %d (%s): FK error %.3g m > %.3g"), G.MjGeomId, *L.Name, Err, Tol));
			}
		}
	}
	Test.TestTrue(TEXT("FK checked at least one geom"), Checked > 0);
	return bOk && Checked > 0;
}

// Find a joint by its URDF name.
const FUrdfJoint* FindJoint(const FUrdfModel& Model, const TCHAR* Name)
{
	for (const FUrdfJoint& J : Model.Joints)
		if (J.Name == Name)
			return &J;
	return nullptr;
}

int CountJointType(const FUrdfModel& Model, const TCHAR* Type)
{
	int N = 0;
	for (const FUrdfJoint& J : Model.Joints)
		if (J.Type == Type)
			++N;
	return N;
}
}  // namespace

// ---------------------------------------------------------------------------
// 1. Counts / types / limits / mesh emission on the two-link chain.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjUrdfExportCounts,
	"URLab.Urdf.ExportCountsLimitsMeshes",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjUrdfExportCounts::RunTest(const FString& Parameters)
{
	FMjTestSession S;
	if (!S.CompileXml(kChainXml))
	{
		AddError(S.LastError);
		return false;
	}
	mjModel* m = S.m;

	const FString OutDir = FPaths::Combine(FPaths::ProjectSavedDir(),
		TEXT("URLab"), TEXT("Tests"), TEXT("urdf_chain"));
	FUrdfExportConfig Cfg;
	const FUrdfModel Model = FUrdfExporter::ExportToDir(m, TEXT("chain"), FString(), OutDir, Cfg);

	// Link count == non-world body count; joint count == joints exported.
	TestEqual(TEXT("link count == nbody-1"), Model.Links.Num(), static_cast<int32>(m->nbody) - 1);
	TestEqual(TEXT("two joints (root joint dropped)"), Model.Joints.Num(), 2);
	TestEqual(TEXT("one revolute"), CountJointType(Model, TEXT("revolute")), 1);
	TestEqual(TEXT("one prismatic"), CountJointType(Model, TEXT("prismatic")), 1);

	// Limits == jnt_range - qpos0.
	const FUrdfJoint* J1 = FindJoint(Model, TEXT("j1"));
	const FUrdfJoint* J2 = FindJoint(Model, TEXT("j2"));
	TestNotNull(TEXT("j1 present"), J1);
	TestNotNull(TEXT("j2 present"), J2);
	if (J1)
	{
		const int Jid = mj_name2id(m, mjOBJ_JOINT, "j1");
		const double Q0 = m->qpos0[m->jnt_qposadr[Jid]];
		TestEqual(TEXT("j1 is revolute"), J1->Type, FString(TEXT("revolute")));
		TestEqual(TEXT("j1 lower == range0 - qpos0"), J1->Lower, m->jnt_range[2 * Jid + 0] - Q0, 1e-9);
		TestEqual(TEXT("j1 upper == range1 - qpos0"), J1->Upper, m->jnt_range[2 * Jid + 1] - Q0, 1e-9);
		TestTrue(TEXT("j1 qpos0 non-zero (ref applied)"), FMath::Abs(Q0 - 0.5) < 1e-9);
	}
	if (J2)
	{
		const int Jid = mj_name2id(m, mjOBJ_JOINT, "j2");
		const double Q0 = m->qpos0[m->jnt_qposadr[Jid]];
		TestEqual(TEXT("j2 is prismatic"), J2->Type, FString(TEXT("prismatic")));
		TestEqual(TEXT("j2 lower == range0 - qpos0"), J2->Lower, m->jnt_range[2 * Jid + 0] - Q0, 1e-9);
		TestEqual(TEXT("j2 upper == range1 - qpos0"), J2->Upper, m->jnt_range[2 * Jid + 1] - Q0, 1e-9);
	}

	// Mesh emitted, referenced and on disk.
	TestEqual(TEXT("one mesh referenced"), Model.MeshIds.Num(), 1);
	TestTrue(TEXT("URDF references a file:// mesh"), Model.Xml.Contains(TEXT("file://")));
	TestTrue(TEXT("URDF references an .stl"), Model.Xml.Contains(TEXT(".stl")));
	if (Model.MeshIds.Num() == 1)
	{
		const FString Stl = FPaths::Combine(OutDir, TEXT("meshes"),
			FUrdfExporter::MeshBaseName(m, Model.MeshIds[0]) + TEXT(".stl"));
		TestTrue(TEXT("STL written to disk"), FPaths::FileExists(Stl));
		const int64 Size = IFileManager::Get().FileSize(*Stl);
		TestTrue(TEXT("STL is a valid binary header + triangles"), Size >= 84);
		TestTrue(TEXT("URDF names the emitted mesh"),
			Model.Xml.Contains(FUrdfExporter::MeshBaseName(m, Model.MeshIds[0]) + TEXT(".stl")));
	}
	TestTrue(TEXT("model.urdf written"),
		FPaths::FileExists(FPaths::Combine(OutDir, TEXT("model.urdf"))));

	return true;
}

// ---------------------------------------------------------------------------
// 2. Forward-kinematics round-trip: URDF at q=0 reproduces mjModel geom_xpos.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjUrdfExportFk,
	"URLab.Urdf.ForwardKinematics",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjUrdfExportFk::RunTest(const FString& Parameters)
{
	FMjTestSession S;
	if (!S.CompileXml(kChainXml))
	{
		AddError(S.LastError);
		return false;
	}
	const FString MeshDir = FPaths::Combine(FPaths::ProjectSavedDir(),
		TEXT("URLab"), TEXT("Tests"), TEXT("urdf_chain"), TEXT("meshes"));
	FUrdfExportConfig Cfg;
	const TArray<int32> Bodies = FUrdfExporter::BodyIdsForArt(S.m, FString());
	const FUrdfModel Model = FUrdfExporter::Build(S.m, TEXT("chain"), FString(), Bodies, MeshDir, Cfg);

	TestTrue(TEXT("chain FK matches geom_xpos"),
		CheckForwardKinematics(*this, Model, S.m, S.d, 1e-6));
	return true;
}

// ---------------------------------------------------------------------------
// 3. jnt_pos anchor frame-shift + multi-joint dummy chain: FK + dummy link.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjUrdfExportAnchorMultiJoint,
	"URLab.Urdf.AnchorAndMultiJoint",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjUrdfExportAnchorMultiJoint::RunTest(const FString& Parameters)
{
	FMjTestSession S;
	if (!S.CompileXml(kAnchorXml))
	{
		AddError(S.LastError);
		return false;
	}
	const FString MeshDir = FPaths::Combine(FPaths::ProjectSavedDir(),
		TEXT("URLab"), TEXT("Tests"), TEXT("urdf_anchor"), TEXT("meshes"));
	FUrdfExportConfig Cfg;
	const TArray<int32> Bodies = FUrdfExporter::BodyIdsForArt(S.m, FString());
	const FUrdfModel Model = FUrdfExporter::Build(S.m, TEXT("anchor"), FString(), Bodies, MeshDir, Cfg);

	// The two-joint wrist yields one dummy link and three joints total
	// (ja, jw1 on the dummy, jw2 on the real wrist link).
	bool bHasDummy = false;
	for (const FUrdfLink& L : Model.Links)
		if (L.Name == TEXT("wrist__j0"))
			bHasDummy = true;
	TestTrue(TEXT("dummy link inserted for two-joint body"), bHasDummy);
	TestEqual(TEXT("three joints (ja + jw1 + jw2)"), Model.Joints.Num(), 3);

	// The jnt_pos anchor is non-zero, so this exercises the frame-shift path.
	const int JaId = mj_name2id(S.m, mjOBJ_JOINT, "ja");
	TestTrue(TEXT("anchor joint has non-zero jnt_pos"),
		FMath::Abs(S.m->jnt_pos[3 * JaId + 0]) > 1e-9);

	TestTrue(TEXT("anchor+wrist FK matches geom_xpos"),
		CheckForwardKinematics(*this, Model, S.m, S.d, 1e-6));
	return true;
}

// ---------------------------------------------------------------------------
// 4. Reference match: the real Franka. Gated on the menagerie checkout being
//    present so CI without it stays green.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjUrdfExportPanda,
	"URLab.Urdf.PandaReference",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjUrdfExportPanda::RunTest(const FString& Parameters)
{
	const FString PandaXml = TEXT("C:/dev/menagerie/franka_emika_panda/panda.xml");
	if (!FPaths::FileExists(PandaXml))
	{
		AddInfo(TEXT("URLab.Urdf.PandaReference: menagerie panda.xml absent; skipping."));
		return true;
	}

	char Err[1000] = "";
	mjModel* m = mj_loadXML(TCHAR_TO_UTF8(*PandaXml), nullptr, Err, sizeof(Err));
	if (!m)
	{
		AddError(FString::Printf(TEXT("mj_loadXML failed: %hs"), Err));
		return false;
	}
	mjData* d = mj_makeData(m);

	const FString OutDir = FPaths::Combine(FPaths::ProjectSavedDir(),
		TEXT("URLab"), TEXT("Tests"), TEXT("urdf_panda"));
	FUrdfExportConfig Cfg;
	const FUrdfModel Model = FUrdfExporter::ExportToDir(m, TEXT("panda"), FString(), OutDir, Cfg);

	// The validated reference (docs/plan_ros_urdf_port_spec.md): 11 links, 10
	// joints (7 revolute, 2 prismatic, 1 fixed), 67 meshes.
	TestEqual(TEXT("11 links"), Model.Links.Num(), 11);
	TestEqual(TEXT("10 joints"), Model.Joints.Num(), 10);
	TestEqual(TEXT("7 revolute"), CountJointType(Model, TEXT("revolute")), 7);
	TestEqual(TEXT("2 prismatic"), CountJointType(Model, TEXT("prismatic")), 2);
	TestEqual(TEXT("1 fixed"), CountJointType(Model, TEXT("fixed")), 1);
	TestEqual(TEXT("67 meshes"), Model.MeshIds.Num(), 67);

	// Every 1-DOF joint limit is jnt_range - qpos0.
	bool bLimitsOk = true;
	for (const FUrdfJoint& J : Model.Joints)
	{
		if (!J.bHasLimit)
			continue;
		const int Jid = J.MjJointId;
		const double Q0 = m->qpos0[m->jnt_qposadr[Jid]];
		if (FMath::Abs(J.Lower - (m->jnt_range[2 * Jid + 0] - Q0)) > 1e-6
			|| FMath::Abs(J.Upper - (m->jnt_range[2 * Jid + 1] - Q0)) > 1e-6)
			bLimitsOk = false;
	}
	TestTrue(TEXT("all limits == jnt_range - qpos0"), bLimitsOk);

	// FK reproduces mjModel geom_xpos across the whole Franka.
	TestTrue(TEXT("panda FK matches geom_xpos"),
		CheckForwardKinematics(*this, Model, m, d, 1e-5));

	AddInfo(FString::Printf(TEXT("panda URDF written to %s (%d warnings)"),
		*FPaths::Combine(OutDir, TEXT("model.urdf")), Model.Warnings.Num()));

	mj_deleteData(d);
	mj_deleteModel(m);
	return true;
}

// ---------------------------------------------------------------------------
// 5. Matched qpos0 shift: FillJointState emits qpos - qpos0. Pure IR transform.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjUrdfJointStateShift,
	"URLab.Urdf.JointStateQpos0Shift",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjUrdfJointStateShift::RunTest(const FString& Parameters)
{
	FMjArticulationState Art;
	Art.Name = FName(TEXT("art"));

	// Hinge with a non-zero reference: JointState must report qpos - qpos0.
	FMjJointState Hinge;
	Hinge.Name = FName(TEXT("j1"));
	Hinge.Type = EMjJointType::Hinge;
	Hinge.QPos = {1.25};
	Hinge.QVel = {0.4};
	Hinge.RefPos = {0.5};
	Art.Joints.Add(Hinge);

	// Slide with no reference recorded: unshifted.
	FMjJointState Slide;
	Slide.Name = FName(TEXT("j2"));
	Slide.Type = EMjJointType::Slide;
	Slide.QPos = {0.30};
	Slide.QVel = {0.0};
	Art.Joints.Add(Slide);

	TArray<FString> Names;
	TArray<double> Positions;
	TArray<double> Velocities;
	UURLabRosPublishTransport::FillJointState(Art, Names, Positions, Velocities);

	TestEqual(TEXT("two positions"), Positions.Num(), 2);
	TestEqual(TEXT("hinge shifted by qpos0"), Positions[0], 0.75, 1e-9);
	TestEqual(TEXT("slide unshifted (no RefPos)"), Positions[1], 0.30, 1e-9);
	TestEqual(TEXT("velocity untouched"), Velocities[0], 0.4, 1e-9);
	return true;
}

// ---------------------------------------------------------------------------
// 6. Auto-export on compile + latched robot_description publish. The pure part
//    (export cached) always runs; the rcl publish gates on a live ROS context.
// ---------------------------------------------------------------------------
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjUrdfPublishRobotDescription,
	"URLab.Urdf.PublishRobotDescription",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMjUrdfPublishRobotDescription::RunTest(const FString& Parameters)
{
	FMjUESession S;
	if (!S.Init())
	{
		AddError(S.LastError);
		return false;
	}

	// RefreshStateCaches runs the auto-export and builds the collector cache.
	S.Manager->RefreshStateCaches();
	TestTrue(TEXT("at least one URDF cached after compile"),
		S.Manager->GetRobotDescriptions().Num() >= 1);

	FURLabRosContext::Get().Initialize();
	if (!FURLabRosContext::Get().IsAvailable())
	{
		UE_LOG(LogTemp, Display,
			TEXT("URLab.Urdf.PublishRobotDescription: no live ROS context; publish leg skipped."));
		S.Cleanup();
		return true;
	}

	UURLabRosPublishTransport* Ros = NewObject<UURLabRosPublishTransport>(S.Manager);
	TestTrue(TEXT("transport init"), Ros->TransportInit());

	mjModel* m = S.Manager->PhysicsEngine->GetModel();
	mjData* d = S.Manager->PhysicsEngine->GetData();
	const FMjStateSnapshot& Snap = S.Manager->GetStateCollector().Collect(m, d, 0);

	// Rebuilds the publisher set (JointState + latched robot_description via rcl)
	// and publishes once; must not tear the context down.
	Ros->PublishState(Snap);
	TestTrue(TEXT("art publisher built"), Ros->GetArtPublisherCountForTest() >= 1);
	TestTrue(TEXT("context still available after publish"),
		FURLabRosContext::Get().IsAvailable());

	Ros->TransportShutdown();
	S.Cleanup();
	return true;
}

#endif  // URLAB_WITH_ROS2
