// Copyright Epic Games, Inc. All Rights Reserved.


using System;
using UnrealBuildTool;
using System.IO;
using System.Text.RegularExpressions;

public class URLab : ModuleRules
{
	public URLab(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		//bUseRTTI = true;  // Required for type info
		//bEnableExceptions = true;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"InputCore",
			"PhysicsCore",
			"XmlParser",
			"HTTP",
			"Json",
			"JsonUtilities",
			"Slate",
			"SlateCore",
			"Projects",
			"ApplicationCore",
			"RenderCore",
			"RHI",
			"UMG",
			"AssetRegistry",
			"ProceduralMeshComponent",
			"GeometryFramework",
			"GeometryCore"
		});

		// Editor-only dependencies for DecomposeMesh and other #if WITH_EDITOR code
		if (Target.bBuildEditor)
		{
			PrivateDependencyModuleNames.AddRange(new string[]
			{
				"UnrealEd",
				"AssetTools"
			});
		}

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"CinematicCamera",
			"ImageWrapper",
			"EnhancedInput",
			"Chaos",
			"Landscape",
			"Eigen",
			"DesktopPlatform"
		});

		DynamicallyLoadedModuleNames.AddRange(new string[]
		{
			// Add dynamically loaded modules here
		});

		if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			PublicSystemLibraries.AddRange(new string[]
			{
				"kernel32.lib",
				"user32.lib"
			});
		}
		else if (Target.Platform == UnrealTargetPlatform.Linux)
		{
			PublicSystemLibraries.AddRange(new string[]
			{
				"pthread", "dl", "rt"
			});
		}

		if (Target.Platform == UnrealTargetPlatform.Linux)
		{
			// Ensure Unreal recognizes MuJoCo's export macros
		}


		if (Target.Platform == UnrealTargetPlatform.Linux)
		{
			PublicDefinitions.Add("_WIN32=0");
			PublicDefinitions.Add("USE_DECLSPEC=1");
			PublicDefinitions.Add("__linux__=1");
			PublicDefinitions.Add("__unix__=1");
		}


		AddMuj(Target);
		AddCoACD(Target);
		AddZeroMQ(Target);
	}

	private string ThirdPartyPath
	{
		get { return Path.Combine(PluginDirectory, "third_party", "install"); }
	}

	private void AddThirdPartyLibrary(string LibraryName, ReadOnlyTargetRules Target)
	{
		string FullPath = Path.Combine(ThirdPartyPath, LibraryName);
		
		// Add include directory if it exists
		string IncludePath = Path.Combine(FullPath, "include");
		if (Directory.Exists(IncludePath))
		{
			PublicIncludePaths.Add(IncludePath);
		}

		if (Target.Platform == UnrealTargetPlatform.Win64)
		{
			// Link all libraries in the lib directory
			string LibPath = Path.Combine(FullPath, "lib");
			if (Directory.Exists(LibPath))
			{
				string[] LibFiles = Directory.GetFiles(LibPath, "*.lib", SearchOption.AllDirectories);
				foreach (string LibFile in LibFiles)
				{
					PublicAdditionalLibraries.Add(LibFile);
				}
			}

			// Stage DLLs next to executable for packaged builds
			// Skip MSVC runtime DLLs (handled by redistributable installer)
			string BinPath = Path.Combine(FullPath, "bin");
			if (Directory.Exists(BinPath))
			{
				string[] DllFiles = Directory.GetFiles(BinPath, "*.dll", SearchOption.AllDirectories);
				foreach (string DllFile in DllFiles)
				{
					string DllName = Path.GetFileName(DllFile);
					if (DllName.StartsWith("vcruntime") || DllName.StartsWith("msvcp") || DllName.StartsWith("concrt"))
						continue;
					RuntimeDependencies.Add("$(BinaryOutputDir)/" + DllName, DllFile, StagedFileType.NonUFS);
					PublicDelayLoadDLLs.Add(DllName);
				}
			}
		}
		else if (Target.Platform == UnrealTargetPlatform.Linux)
		{
			// Link all static libraries and shared objects for Linux
			string LibPath = Path.Combine(FullPath, "lib");
			if (Directory.Exists(LibPath))
			{
				foreach (string LibFile in Directory.GetFiles(LibPath, "*.a", SearchOption.AllDirectories))
				{
					PublicAdditionalLibraries.Add(LibFile);
				}
				foreach (string LibFile in Directory.GetFiles(LibPath, "*.so", SearchOption.AllDirectories))
				{
					PublicAdditionalLibraries.Add(LibFile);
					RuntimeDependencies.Add(LibFile);
					PublicDelayLoadDLLs.Add(Path.GetFileName(LibFile));
				}
			}

			// Add binaries (often plugin/shared objects) for Linux
			string BinPath = Path.Combine(FullPath, "bin");
			if (Directory.Exists(BinPath))
			{
				foreach (string BinFile in Directory.GetFiles(BinPath, "*.so", SearchOption.AllDirectories))
				{
					RuntimeDependencies.Add(BinFile);
					PublicDelayLoadDLLs.Add(Path.GetFileName(BinFile));
					PublicAdditionalLibraries.Add(BinFile);
				}
			}
		}
	}

	// Pinned MuJoCo version. Bump in lockstep with $PinnedCommit in
	// third_party/MuJoCo/build.ps1 / build.sh and the matching constant
	// in src/include/mujoco/mujoco.h's mjVERSION_HEADER.
	private const int ExpectedMjVersionHeader = 3007000;  // MuJoCo 3.7.0

	protected void AddMuj(ReadOnlyTargetRules Target)
	{
		VerifyMuJoCoInstall();
		AddThirdPartyLibrary("MuJoCo", Target);
	}

	// Reads the installed mujoco.h and verifies its mjVERSION_HEADER matches
	// what URLab was bumped to. Catches the common "I git-pulled URLab but
	// forgot to re-run third_party/MuJoCo/build.ps1" failure mode before any
	// code compiles, with a message that says exactly what to do.
	private void VerifyMuJoCoInstall()
	{
		string MjHeader = Path.Combine(ThirdPartyPath, "MuJoCo", "include", "mujoco", "mujoco.h");
		if (!File.Exists(MjHeader))
		{
			throw new BuildException(
				"MuJoCo headers missing at '{0}'. Run third_party/MuJoCo/build.ps1 (Windows) " +
				"or third_party/MuJoCo/build.sh (Linux/macOS) to build the dependency, then retry.",
				MjHeader);
		}

		string Text = File.ReadAllText(MjHeader);
		Match M = Regex.Match(Text, @"#define\s+mjVERSION_HEADER\s+(\d+)");
		if (!M.Success)
		{
			throw new BuildException(
				"Could not parse mjVERSION_HEADER from '{0}'. The header may be corrupt — " +
				"wipe third_party/install/MuJoCo/ and re-run third_party/MuJoCo/build.ps1.",
				MjHeader);
		}

		int InstalledVersion = int.Parse(M.Groups[1].Value);
		if (InstalledVersion != ExpectedMjVersionHeader)
		{
			throw new BuildException(
				"MuJoCo install at '{0}' is at version {1}; URLab expects {2}. " +
				"Wipe third_party/install/MuJoCo/ and third_party/MuJoCo/src/build/, " +
				"then re-run third_party/MuJoCo/build.ps1 (Windows) or build.sh (Linux/macOS).",
				Path.Combine(ThirdPartyPath, "MuJoCo"), InstalledVersion, ExpectedMjVersionHeader);
		}
	}

	protected void AddCoACD(ReadOnlyTargetRules Target)
	{
		AddThirdPartyLibrary("CoACD", Target);
		PublicDefinitions.Add("COACD_EXPORTS=1");
	}

	protected void AddZeroMQ(ReadOnlyTargetRules Target)
	{
		AddThirdPartyLibrary("libzmq", Target);
	}
}
