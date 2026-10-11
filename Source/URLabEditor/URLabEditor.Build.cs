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

using UnrealBuildTool;
using System.IO;

public class URLabEditor : ModuleRules
{
	public URLabEditor(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"URLab",
			"URLabRos",
			"UnrealEd",
			"EditorSubsystem",
			"AssetTools",
			"AssetRegistry",
			"Blutility",
			"EditorScriptingUtilities",
			"PropertyEditor",
			"Slate",
			"SlateCore",
			"Projects",
			"LevelEditor",
			"Json",
			"JsonUtilities"
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"InputCore",
			"RenderCore",
			"RHI",
			// Flex/skin replication tests read back UDynamicMeshComponent surfaces
			// (UDynamicMesh::ProcessMesh / FDynamicMesh3), so the test module links
			// the geometry runtime directly.
			"GeometryFramework",
			"GeometryCore",
			"MeshDescription",
			"StaticMeshDescription",
			"ProceduralMeshComponent",
			"DesktopPlatform",
			"Kismet",
			"BlueprintGraph",
			"ToolMenus",
			// The Blueprint editor's component tree, whose context menu carries
			// the convex-decomposition entries (MjDecompositionMenu.cpp).
			"SubobjectEditor",
			// Import and generation diagnostics are routed to the editor's
			// Messages panel; the module owns the listing they land in.
			"MessageLog",
			// M_MuJoCo_Master is built from code (MjMasterMaterial.cpp) rather
			// than hand-authored, so the graph is reviewable and reproducible.
			"MaterialEditor"
		});

		// Editor import runs MuJoCo's fatal recursive dependency scanner in a
		// separate process. The helper is staged beside libmujoco, which URLab's
		// runtime module already stages in the same binary directory.
		string HelperName = Target.Platform == UnrealTargetPlatform.Win64
			? "urlab_mj_deps.exe" : "urlab_mj_deps";
		string Helper = Path.GetFullPath(Path.Combine(ModuleDirectory, "..", "..",
			"third_party", "install", "protospec", "bin", HelperName));
		if (File.Exists(Helper))
		{
			RuntimeDependencies.Add("$(BinaryOutputDir)/" + HelperName, Helper, StagedFileType.NonUFS);
		}
	}
}
