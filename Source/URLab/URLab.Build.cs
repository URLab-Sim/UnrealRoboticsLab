// Copyright Epic Games, Inc. All Rights Reserved.


using System;
using UnrealBuildTool;
using System.IO;

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

	protected void AddMuj(ReadOnlyTargetRules Target)
	{
		AddThirdPartyLibrary("MuJoCo", Target);
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
