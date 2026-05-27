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

#include "URLab.h"
#include "HAL/FileManager.h"
#include "Misc/Paths.h"
#include "Misc/MessageDialog.h"
#include "Utils/URLabLogging.h"
#include "Interfaces/IPluginManager.h"

#if WITH_EDITOR
#include "AssetToolsModule.h"
#include "IAssetTools.h"
#endif

#define LOCTEXT_NAMESPACE "FURLabModule"
void FURLabModule::StartupModule()
{
    FString PluginDir = IPluginManager::Get().FindPlugin("UnrealRoboticsLab")->GetBaseDir();
    FString InstallDir = FPaths::Combine(PluginDir, TEXT("third_party/install"));

    // Resolve LibraryPattern (literal filename or shell-style wildcard) to
    // an absolute path inside SearchDir, or empty if no file matches.
    auto ResolveLibraryFile = [](const FString& SearchDir, const FString& LibraryPattern) -> FString {
        if (!LibraryPattern.Contains(TEXT("*"))) {
            const FString Candidate = FPaths::Combine(SearchDir, LibraryPattern);
            return FPaths::FileExists(Candidate) ? Candidate : FString();
        }
        TArray<FString> Matches;
        IFileManager::Get().FindFiles(Matches, *FPaths::Combine(SearchDir, LibraryPattern),
                                      /*Files=*/true, /*Directories=*/false);
        return Matches.Num() > 0 ? FPaths::Combine(SearchDir, Matches[0]) : FString();
    };

    // Load a shared library and log success/failure. LibraryPattern may
    // contain `*` so we can match toolset-versioned filenames (libzmq bakes
    // the MSVC toolset version into its DLL name; building with a different
    // toolset than the URLab hardcode used to surface as a delay-load
    // 0xc06d007e on first ZMQ call). BinSubDir is the per-package
    // subdirectory under third_party/install/<SubDir>/ that holds the
    // loadable artifact: "bin" on Windows (DLLs), "lib" on Linux (.so).
    auto LoadDependencyDLL = [&](const FString& LibraryPattern, const FString& SubDir, const FString& BinSubDir) {
        // Try plugin third-party path first (editor / development).
        FString DLLPath = ResolveLibraryFile(FPaths::Combine(InstallDir, SubDir, BinSubDir), LibraryPattern);
        if (DLLPath.IsEmpty()) {
            // Packaged build: shared libs staged next to the executable.
            DLLPath = ResolveLibraryFile(FPlatformProcess::GetModulesDirectory(), LibraryPattern);
        }
        if (DLLPath.IsEmpty()) {
            UE_LOG(LogURLab, Error,
                TEXT("%s not found in third_party/install/%s/%s or the modules directory. "
                     "Run third_party/build_all.ps1 (Windows) or build_all.sh (Linux/macOS) "
                     "from the plugin root, then rebuild the editor."),
                *LibraryPattern, *SubDir, *BinSubDir);
            return false;
        }
        void* Handle = FPlatformProcess::GetDllHandle(*DLLPath);
        if (Handle) {
            UE_LOG(LogURLab, Log, TEXT("Loaded %s from %s"), *FPaths::GetCleanFilename(DLLPath), *DLLPath);
            return true;
        }
        UE_LOG(LogURLab, Error, TEXT("Failed to load %s (GetLastError surfaces in delay-load fault otherwise)"), *DLLPath);
        return false;
    };

    bool bAllDepsLoaded = true;
#if PLATFORM_WINDOWS
    // Load MuJoCo. Since 3.7.0 the obj/stl decoders are compiled into
    // mujoco.dll itself (changelog item 9); loading the standalone
    // obj_decoder.dll / stl_decoder.dll would cause a plugin-registration
    // collision and crash during module init.
    bAllDepsLoaded &= LoadDependencyDLL(TEXT("mujoco.dll"), TEXT("MuJoCo"), TEXT("bin"));
    // libzmq's CMake bakes <toolset>-<threading>-<abi> into the output
    // name (e.g. libzmq-v143-mt-4_3_6.dll for VS 2022, libzmq-v144-*
    // for newer toolsets). Glob so URLab works regardless of which VS
    // version the user built third_party with.
    bAllDepsLoaded &= LoadDependencyDLL(TEXT("libzmq-*-mt-*.dll"), TEXT("libzmq"), TEXT("bin"));
    bAllDepsLoaded &= LoadDependencyDLL(TEXT("lib_coacd.dll"), TEXT("CoACD"), TEXT("bin"));
#elif PLATFORM_LINUX
    // Linux .so layout: third_party/install/<pkg>/lib/. Glob the SONAME
    // suffix so a MuJoCo bump (or any minor version bump) doesn't require
    // tracking the literal version in the loader.
    bAllDepsLoaded &= LoadDependencyDLL(TEXT("libmujoco.so*"), TEXT("MuJoCo"), TEXT("lib"));
    bAllDepsLoaded &= LoadDependencyDLL(TEXT("libzmq.so*"), TEXT("libzmq"), TEXT("lib"));
    bAllDepsLoaded &= LoadDependencyDLL(TEXT("lib_coacd.so*"), TEXT("CoACD"), TEXT("lib"));
#endif

    if (!bAllDepsLoaded) {
        // Surface the failure now instead of letting the first ZMQ / MuJoCo
        // call fault with delay-load's generic 0xc06d007e.
        const FText Title = LOCTEXT("URLabDllLoadFailedTitle", "URLab: third-party DLL load failed");
        const FText Body = LOCTEXT("URLabDllLoadFailedBody",
            "URLab could not load one or more required third-party libraries "
            "(MuJoCo / libzmq / CoACD).\n\n"
            "Run third_party/build_all.ps1 (Windows) or build_all.sh (Linux/macOS) "
            "from the plugin root, then rebuild the editor.\n\n"
            "See the URLab log for the missing file name(s).");
        FMessageDialog::Open(EAppMsgType::Ok, Body, Title);
    }

    // Some CoACD dependencies like TBB or runtimes might be in CoACD/bin
    // They should be loaded automatically if in search path, but we can verify here if needed.

#if WITH_EDITOR
    // Register custom asset category
    IAssetTools& AssetTools = FModuleManager::LoadModuleChecked<FAssetToolsModule>("AssetTools").Get();
    MuJoCoAssetCategoryBit = AssetTools.RegisterAdvancedAssetCategory(FName(TEXT("MuJoCo")), LOCTEXT("MujocoAssetCategory", "MuJoCo"));
    UE_LOG(LogURLab, Log, TEXT("Registered MuJoCo Asset Category with bitmask: %u"), MuJoCoAssetCategoryBit);
#endif
}

void FURLabModule::ShutdownModule()
{
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FURLabModule, URLab)
