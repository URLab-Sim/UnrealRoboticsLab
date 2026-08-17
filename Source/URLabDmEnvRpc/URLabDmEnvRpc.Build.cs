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

using System;
using System.IO;
using UnrealBuildTool;

public class URLabDmEnvRpc : ModuleRules
{
	public URLabDmEnvRpc(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.NoPCHs;

		bEnableExceptions = true;
		bUseUnity = false;
		UndefinedIdentifierWarningLevel = WarningLevel.Off;
		ShadowVariableWarningLevel = WarningLevel.Off;
		UnsafeTypeCastWarningLevel = WarningLevel.Off;

		PublicDefinitions.Add("PROTOBUF_ENABLE_DEBUG_LOGGING_MAY_LEAK_PII=0");

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"URLab"
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"Json"
		});

		PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "Public"));
		PrivateIncludePaths.Add(Path.Combine(ModuleDirectory, "Private"));
		PrivateIncludePaths.Add(Path.Combine(ModuleDirectory, "Private", "GenProto"));

		string ThirdPartyPath = Path.GetFullPath(Path.Combine(PluginDirectory, "third_party", "install"));
		string GrpcRoot = Path.Combine(ThirdPartyPath, "grpc");
		string GrpcInclude = Path.Combine(GrpcRoot, "include");
		string GrpcLib = Path.Combine(GrpcRoot, "lib");

		if (Directory.Exists(GrpcInclude))
		{
			PublicIncludePaths.Add(GrpcInclude);
		}

		if (Directory.Exists(GrpcLib))
		{
			foreach (string LibFile in Directory.GetFiles(GrpcLib, "*.a"))
			{
				PublicAdditionalLibraries.Add(LibFile);
			}
		}

		if (Target.Platform == UnrealTargetPlatform.Linux)
		{
			PublicSystemLibraries.AddRange(new string[] { "pthread", "dl", "rt", "z" });
		}
	}
}
