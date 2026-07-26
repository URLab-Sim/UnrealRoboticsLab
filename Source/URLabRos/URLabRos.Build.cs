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

using System;
using System.Collections.Generic;
using UnrealBuildTool;
using System.IO;

// URLabRos is the optional ROS 2 integration module. It depends on the core
// URLab module and contains every ROS-specific piece (the rcl C-ABI seam, the
// state publish transport, the control RPC transport, the ROS context, and the
// camera image sink). Core URLab has no dependency on it and no ROS references;
// non-ROS users simply do not enable this module. When ROS is not installed
// (URLAB_ROS2_ROOT unset) AddRos2 compiles the feature out and this module still
// builds green as a set of no-op stubs.
public class URLabRos : ModuleRules
{
	public URLabRos(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		// Mirror URLab: the ROS transports pull in AMjManager.h and the bridge
		// headers, which transitively reach msgpack (exceptions) and windows.h
		// (whose GetObject macro leaks across a unity TU into Chaos-using
		// neighbours). Disabling unity keeps every .cpp in its own TU.
		bEnableExceptions = true;
		bUseUnity = false;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"URLab"
		});

		// The ROS RPC transport includes MjTwistController.h (cmd_vel routing),
		// whose public header pulls in EnhancedInput's InputActionValue.h. URLab
		// depends on EnhancedInput privately, so it does not propagate; declare it
		// here for this module's own translation units.
		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"EnhancedInput",
			// The user-channel provider inflates a Struct channel's packed msgpack
			// map (via URLab's FURLabMsgpackUtil) and re-serialises it as JSON text
			// for its std_msgs/String topic.
			"Json"
		});

		if (Target.Platform == UnrealTargetPlatform.Linux)
		{
			PublicSystemLibraries.AddRange(new string[] { "pthread", "dl", "rt" });
		}

		AddRos2(Target);
	}

	private string ThirdPartyPath
	{
		get { return Path.Combine(PluginDirectory, "third_party", "install"); }
	}

	// Links the ROS 2 C API (rcl / rmw / rosidl_runtime_c + the message-package
	// typesupport/generator libs) so the in-process ROS publisher can fill rosidl
	// C structs and call rcl_publish. UBT compiles only the C ABI, so the large
	// version-coupled ROS set is NOT globbed: an explicit pinned link list plus a
	// pattern-scoped runtime DLL/so staging. The exact basenames and include
	// layout are recorded in docs/ros2_link_facts.md from an actual install; the
	// standalone harness (ros/urlab_ros_ws) validates the same C code against real
	// DDS.
	protected void AddRos2(ReadOnlyTargetRules Target)
	{
		// Resolve the install root: an explicit override, else the in-repo install
		// convention used for the other deps. On a Pixi/Conda (RoboStack) install
		// URLAB_ROS2_ROOT points at the environment's "Library" dir; a from-source
		// or apt install points at the prefix itself. Both hold include/lib/bin.
		string RosRoot = Environment.GetEnvironmentVariable("URLAB_ROS2_ROOT");
		if (string.IsNullOrEmpty(RosRoot))
		{
			RosRoot = Path.Combine(ThirdPartyPath, "ros2");
		}

		// Absent ROS: compile the feature out. Every ROS reference in the module
		// is fenced with #if URLAB_WITH_ROS2, so the build stays green with no ROS.
		if (string.IsNullOrEmpty(RosRoot) || !Directory.Exists(RosRoot))
		{
			PublicDefinitions.Add("URLAB_WITH_ROS2=0");
			Console.WriteLine("URLabRos: ROS 2 not found (set URLAB_ROS2_ROOT to enable) - building without ROS.");
			return;
		}

		string IncludeRoot = Path.Combine(RosRoot, "include");
		string LibDir = Path.Combine(RosRoot, "lib");
		string BinDir = Path.Combine(RosRoot, "bin");

		// ROS packages use a double-nested layout: include/<pkg>/<pkg>/... , so
		// each package directory is added (that is what lets #include <pkg/x.h>
		// resolve to include/<pkg>/<pkg>/x.h). rcl transitively pulls in packages
		// beyond the ones the module names directly (rcl_yaml_param_parser,
		// rcl_interfaces, ...), so the set is discovered rather than hand-listed:
		// a directory is a ROS package when it holds a same-named child. The flat
		// include root is deliberately NOT added - it also contains unrelated
		// dependency headers (hwloc, openssl, ...) that would shadow system
		// headers and break other translation units. These go on the system
		// include path so ROS headers' warnings (C4668 on __STDC_VERSION__) do
		// not trip UE's warnings-as-errors.
		if (Directory.Exists(IncludeRoot))
		{
			foreach (string PkgDir in Directory.GetDirectories(IncludeRoot))
			{
				string Pkg = Path.GetFileName(PkgDir);
				if (Directory.Exists(Path.Combine(PkgDir, Pkg)))
				{
					PublicSystemIncludePaths.Add(PkgDir);
				}
			}
		}

		// Link only the C ABI the module references directly. The rest of the ROS
		// graph is loaded by the DDS/rmw runtime, so it is staged, not linked.
		string[] LinkNames =
		{
			"rcl", "rcutils", "rmw", "rosidl_runtime_c",
			"builtin_interfaces__rosidl_generator_c", "builtin_interfaces__rosidl_typesupport_c",
			"std_msgs__rosidl_generator_c", "std_msgs__rosidl_typesupport_c",
			"std_srvs__rosidl_generator_c", "std_srvs__rosidl_typesupport_c",
			"geometry_msgs__rosidl_generator_c", "geometry_msgs__rosidl_typesupport_c",
			"sensor_msgs__rosidl_generator_c", "sensor_msgs__rosidl_typesupport_c",
			"tf2_msgs__rosidl_generator_c", "tf2_msgs__rosidl_typesupport_c",
			"nav_msgs__rosidl_generator_c", "nav_msgs__rosidl_typesupport_c",
			"rosgraph_msgs__rosidl_generator_c", "rosgraph_msgs__rosidl_typesupport_c"
		};

		if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			foreach (string Name in LinkNames)
			{
				string LibFile = Path.Combine(LibDir, Name + ".lib");
				if (File.Exists(LibFile))
				{
					PublicAdditionalLibraries.Add(LibFile);
				}
				else
				{
					Console.WriteLine("URLabRos: ROS 2 lib not found (skipped): {0}", LibFile);
				}
			}

			// Stage the ROS/DDS runtime DLL cluster from the ROS bin dir. Scoped by
			// pattern to the ROS + DDS families (not the whole environment), with
			// prefixes so the version-suffixed DDS names (fastdds-3.6, fastcdr-2.3,
			// foonathan_memory-0.7.4) resolve without hard-coding the suffix.
			string[] DllPatterns =
			{
				"rcl*.dll", "rmw*.dll", "rcutils.dll", "rcpputils.dll",
				"ament_index_cpp.dll", "rosidl_*.dll", "*__rosidl_*.dll",
				"fastdds*.dll", "fastcdr*.dll", "foonathan_memory*.dll",
				"tinyxml2.dll", "spdlog.dll", "dds_security*.dll",
				"libssl*.dll", "libcrypto*.dll"
			};
			StageRosRuntime(BinDir, DllPatterns, true);
		}
		else if (Target.Platform == UnrealTargetPlatform.Linux)
		{
			foreach (string Name in LinkNames)
			{
				// Link the unversioned .so symlink so the SONAME is recorded, not
				// an absolute versioned path.
				string SoFile = Path.Combine(LibDir, "lib" + Name + ".so");
				if (File.Exists(SoFile))
				{
					PublicAdditionalLibraries.Add(SoFile);
				}
				else
				{
					Console.WriteLine("URLabRos: ROS 2 lib not found (skipped): {0}", SoFile);
				}
			}

			// Stage the shared-object cluster under $ORIGIN.
			string[] SoPatterns =
			{
				"librcl*.so*", "librmw*.so*", "librcutils.so*", "librcpputils.so*",
				"libament_index_cpp.so*", "librosidl_*.so*", "*__rosidl_*.so*",
				"libfastdds*.so*", "libfastcdr*.so*", "libfoonathan_memory*.so*",
				"libtinyxml2.so*", "libspdlog.so*"
			};
			StageRosRuntime(LibDir, SoPatterns, false);
		}

		PublicDefinitions.Add("URLAB_WITH_ROS2=1");
	}

	// Stages every file under Dir matching any of Patterns next to the plugin
	// binary. On Win64 the staged DLLs are also delay-loaded, mirroring
	// AddThirdPartyLibrary in URLab.Build.cs. De-duplicates so overlapping
	// patterns stage once.
	private void StageRosRuntime(string Dir, string[] Patterns, bool bDelayLoad)
	{
		if (!Directory.Exists(Dir))
		{
			Console.WriteLine("URLabRos: ROS 2 runtime dir not found (no staging): {0}", Dir);
			return;
		}
		HashSet<string> Seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
		foreach (string Pattern in Patterns)
		{
			foreach (string FilePath in Directory.GetFiles(Dir, Pattern, SearchOption.TopDirectoryOnly))
			{
				string Name = Path.GetFileName(FilePath);
				if (!Seen.Add(Name))
				{
					continue;
				}
				RuntimeDependencies.Add("$(BinaryOutputDir)/" + Name, FilePath, StagedFileType.NonUFS);
				if (bDelayLoad)
				{
					PublicDelayLoadDLLs.Add(Name);
				}
			}
		}
	}
}
