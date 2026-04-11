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

#include "MjPythonHelper.h"
#include "URLabEditorLogging.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/PlatformProcess.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/MessageDialog.h"
#include "Misc/ScopedSlowTask.h"
#include "DesktopPlatformModule.h"
#include "Framework/Application/SlateApplication.h"

FString FMjPythonHelper::GetLocalIniPath()
{
    TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin("UnrealRoboticsLab");
    if (!Plugin.IsValid())
    {
        UE_LOG(LogURLabEditor, Warning, TEXT("[Python] Could not find UnrealRoboticsLab plugin. Using project config dir."));
        return FPaths::ConvertRelativePathToFull(FPaths::ProjectPluginsDir() / TEXT("UnrealRoboticsLab") / TEXT("Config") / TEXT("LocalUnrealRoboticsLab.ini"));
    }
    FString FullPath = FPaths::ConvertRelativePathToFull(Plugin->GetBaseDir() / TEXT("Config") / TEXT("LocalUnrealRoboticsLab.ini"));
    UE_LOG(LogURLabEditor, Verbose, TEXT("[Python] Local INI path: %s"), *FullPath);
    return FullPath;
}

FString FMjPythonHelper::GetUEBundledPythonPath()
{
    FString PythonDir = FPaths::EngineDir() / TEXT("Binaries/ThirdParty/Python3");
#if PLATFORM_WINDOWS
    FString Path = PythonDir / TEXT("Win64/python.exe");
#elif PLATFORM_LINUX
    FString Path = PythonDir / TEXT("Linux/bin/python3");
#elif PLATFORM_MAC
    FString Path = PythonDir / TEXT("Mac/bin/python3");
#else
    FString Path;
#endif
    return FPaths::ConvertRelativePathToFull(Path);
}

FString FMjPythonHelper::GetStoredPythonOverride()
{
    FString IniPath = GetLocalIniPath();
    if (!FPaths::FileExists(IniPath)) return FString();

    FString FileContent;
    if (!FFileHelper::LoadFileToString(FileContent, *IniPath)) return FString();

    // Parse simple INI: look for PythonPath= line
    TArray<FString> Lines;
    FileContent.ParseIntoArrayLines(Lines);
    for (const FString& Line : Lines)
    {
        FString Trimmed = Line.TrimStartAndEnd();
        if (Trimmed.StartsWith(TEXT("PythonPath=")))
        {
            return Trimmed.Mid(11).TrimStartAndEnd();
        }
    }
    return FString();
}

void FMjPythonHelper::StorePythonOverride(const FString& PythonPath)
{
    FString IniPath = GetLocalIniPath();
    FString AbsPythonPath = FPaths::ConvertRelativePathToFull(PythonPath);

    // Write directly to file — GConfig can silently fail for paths outside the project
    FString IniContent = FString::Printf(TEXT("[PythonSettings]\nPythonPath=%s\n"), *AbsPythonPath);
    if (FFileHelper::SaveStringToFile(IniContent, *IniPath))
    {
        UE_LOG(LogURLabEditor, Log, TEXT("[Python] Stored Python path '%s' to config: %s"), *AbsPythonPath, *IniPath);
    }
    else
    {
        UE_LOG(LogURLabEditor, Warning, TEXT("[Python] Failed to write config file: %s"), *IniPath);
    }
}

FString FMjPythonHelper::ResolvePythonPath()
{
    // 1. Check user override
    FString Override = GetStoredPythonOverride();
    if (!Override.IsEmpty() && FPaths::FileExists(Override))
    {
        return Override;
    }

    // 2. Fall back to UE's bundled Python
    FString Bundled = GetUEBundledPythonPath();
    if (FPaths::FileExists(Bundled))
    {
        return Bundled;
    }

    // 3. Try system PATH as last resort
    return TEXT("python");
}

bool FMjPythonHelper::ValidatePythonBinary(const FString& PythonPath)
{
    int32 ReturnCode = -1;
    FString StdOut, StdErr;
    FPlatformProcess::ExecProcess(*PythonPath, TEXT("--version"), &ReturnCode, &StdOut, &StdErr);
    if (ReturnCode == 0)
    {
        FString Version = StdOut.IsEmpty() ? StdErr : StdOut; // Python 2 prints to stderr
        UE_LOG(LogURLabEditor, Log, TEXT("[Python] Validated: %s -> %s"), *PythonPath, *Version.TrimStartAndEnd());
        return true;
    }
    return false;
}

bool FMjPythonHelper::CheckPythonPackages(const FString& PythonPath)
{
    int32 ReturnCode = -1;
    FString StdOut, StdErr;
    FPlatformProcess::ExecProcess(*PythonPath, TEXT("-c \"import trimesh; import numpy; import scipy\""), &ReturnCode, &StdOut, &StdErr);
    return (ReturnCode == 0);
}

bool FMjPythonHelper::InstallPythonPackages(const FString& PythonPath, FString& OutLog)
{
    // First check if pip is available
    {
        int32 PipCheck = -1;
        FString PipOut, PipErr;
        FPlatformProcess::ExecProcess(*PythonPath, TEXT("-m pip --version"), &PipCheck, &PipOut, &PipErr);
        if (PipCheck != 0)
        {
            OutLog = FString::Printf(
                TEXT("pip is not available in this Python environment.\n\n")
                TEXT("Please install pip first, or choose a different Python interpreter.\n")
                TEXT("You can also install packages manually:\n")
                TEXT("  %s -m ensurepip\n")
                TEXT("  %s -m pip install trimesh numpy scipy"),
                *PythonPath, *PythonPath);
            UE_LOG(LogURLabEditor, Warning, TEXT("[Python] pip not available: %s"), *PipErr);
            return false;
        }
    }

    int32 ReturnCode = -1;
    FString StdOut, StdErr;
    UE_LOG(LogURLabEditor, Log, TEXT("[Python] Installing packages: %s -m pip install trimesh numpy scipy"), *PythonPath);
    FPlatformProcess::ExecProcess(*PythonPath, TEXT("-m pip install trimesh numpy scipy"), &ReturnCode, &StdOut, &StdErr);
    OutLog = StdOut + TEXT("\n") + StdErr;
    if (ReturnCode == 0)
    {
        UE_LOG(LogURLabEditor, Log, TEXT("[Python] Package install succeeded."));
        return true;
    }
    UE_LOG(LogURLabEditor, Warning, TEXT("[Python] Package install failed (code %d): %s"), ReturnCode, *OutLog);
    return false;
}

FString FMjPythonHelper::EnsurePythonReady()
{
    FString PythonPath = ResolvePythonPath();
    bool bIsUEBundled = (PythonPath == GetUEBundledPythonPath());

    // Validate the resolved Python
    if (!ValidatePythonBinary(PythonPath))
    {
        // Python not found at all — prompt for path
        FText Title = FText::FromString(TEXT("Python Interpreter Not Found"));
        FText Message = FText::FromString(FString::Printf(
            TEXT("URLab uses Python to preprocess mesh files during MJCF import.\n\n")
            TEXT("This is needed because Unreal Engine does not natively support all mesh formats ")
            TEXT("used by MuJoCo (e.g. STL, OBJ with certain features). The preprocessing step ")
            TEXT("converts meshes to a compatible format and resolves asset conflicts.\n\n")
            TEXT("Could not find a valid Python interpreter at:\n%s\n\n")
            TEXT("Would you like to browse for your Python interpreter?\n\n")
            TEXT("Click 'No' to skip mesh preprocessing (import will use raw XML — some meshes may not display correctly)."),
            *PythonPath));

        EAppReturnType::Type Result = FMessageDialog::Open(EAppMsgType::YesNo, Message, Title);
        if (Result == EAppReturnType::No)
        {
            return FString(); // User chose to skip
        }

        // Open file picker
        TArray<FString> OutFiles;
        IDesktopPlatform* DesktopPlatform = FDesktopPlatformModule::Get();
        if (DesktopPlatform)
        {
            DesktopPlatform->OpenFileDialog(
                FSlateApplication::Get().FindBestParentWindowHandleForDialogs(nullptr),
                TEXT("Select Python Interpreter"),
                TEXT(""),
                TEXT(""),
#if PLATFORM_WINDOWS
                TEXT("Python Executable (*.exe)|*.exe"),
#else
                TEXT("All Files (*)|*"),
#endif
                0, OutFiles);
        }

        if (OutFiles.Num() == 0)
        {
            return FString(); // Cancelled
        }

        PythonPath = OutFiles[0];
        if (!ValidatePythonBinary(PythonPath))
        {
            FMessageDialog::Open(EAppMsgType::Ok,
                FText::FromString(FString::Printf(TEXT("'%s' is not a valid Python interpreter."), *PythonPath)),
                FText::FromString(TEXT("Invalid Python")));
            return FString();
        }

        StorePythonOverride(PythonPath);
        bIsUEBundled = false;
    }

    // Check for required packages
    if (!CheckPythonPackages(PythonPath))
    {
        FString EnvLabel = bIsUEBundled
            ? TEXT("Unreal Engine's bundled Python")
            : FString::Printf(TEXT("your selected Python at:\n%s"), *PythonPath);

        FText Title = FText::FromString(TEXT("Python Packages Required"));
        FText Message = FText::FromString(FString::Printf(
            TEXT("URLab needs the 'trimesh', 'numpy', and 'scipy' Python packages to preprocess mesh files.\n\n")
            TEXT("Unreal Engine does not natively support all mesh formats used by MuJoCo, ")
            TEXT("so these packages are used to convert and prepare meshes for import.\n\n")
            TEXT("These will be installed to %s.\n\n")
            TEXT("Install now?\n\n")
            TEXT("Note: The editor will be unresponsive during installation. ")
            TEXT("This may take a minute.\n\n")
            TEXT("Click 'Yes' to install, 'No' to skip mesh preprocessing, ")
            TEXT("or 'Cancel' to choose a different Python interpreter."),
            *EnvLabel));

        EAppReturnType::Type Result = FMessageDialog::Open(EAppMsgType::YesNoCancel, Message, Title);

        if (Result == EAppReturnType::No)
        {
            return FString(); // Skip
        }
        else if (Result == EAppReturnType::Cancel)
        {
            // User wants a different Python — open file picker
            TArray<FString> OutFiles;
            IDesktopPlatform* DesktopPlatform = FDesktopPlatformModule::Get();
            if (DesktopPlatform)
            {
                DesktopPlatform->OpenFileDialog(
                    FSlateApplication::Get().FindBestParentWindowHandleForDialogs(nullptr),
                    TEXT("Select Python Interpreter"),
                    TEXT(""),
                    TEXT(""),
#if PLATFORM_WINDOWS
                    TEXT("Python Executable (*.exe)|*.exe"),
#else
                    TEXT("All Files (*)|*"),
#endif
                    0, OutFiles);
            }

            if (OutFiles.Num() == 0) return FString();

            PythonPath = OutFiles[0];
            if (!ValidatePythonBinary(PythonPath))
            {
                FMessageDialog::Open(EAppMsgType::Ok,
                    FText::FromString(FString::Printf(TEXT("'%s' is not a valid Python interpreter."), *PythonPath)),
                    FText::FromString(TEXT("Invalid Python")));
                return FString();
            }
            StorePythonOverride(PythonPath);

            // Re-check packages with new Python
            if (CheckPythonPackages(PythonPath))
            {
                UE_LOG(LogURLabEditor, Log, TEXT("[Python] Packages already available in selected Python."));
                return PythonPath;
            }

            // Ask to install for the new Python
            FText InstallMsg = FText::FromString(FString::Printf(
                TEXT("Install 'trimesh', 'numpy', and 'scipy' to:\n%s?\n\n")
                TEXT("The editor will be unresponsive during installation.\n\n")
                TEXT("Click 'No' to skip mesh preprocessing."), *PythonPath));
            if (FMessageDialog::Open(EAppMsgType::YesNo, InstallMsg, Title) == EAppReturnType::No)
            {
                return FString();
            }
        }

        // Install packages
        FString InstallLog;
        if (!InstallPythonPackages(PythonPath, InstallLog))
        {
            FMessageDialog::Open(EAppMsgType::Ok,
                FText::FromString(FString::Printf(
                    TEXT("Failed to install packages. You can install them manually by running:\n\n")
                    TEXT("%s -m pip install trimesh numpy\n\n")
                    TEXT("Error log:\n%s"),
                    *PythonPath, *InstallLog)),
                FText::FromString(TEXT("Package Install Failed")));
            return FString();
        }

        // Verify install worked
        if (!CheckPythonPackages(PythonPath))
        {
            FMessageDialog::Open(EAppMsgType::Ok,
                FText::FromString(TEXT("Packages were installed but still cannot be imported. Check your Python environment.")),
                FText::FromString(TEXT("Package Verification Failed")));
            return FString();
        }
    }

    // Always save the resolved path so the user can see which Python is in use
    StorePythonOverride(PythonPath);

    UE_LOG(LogURLabEditor, Log, TEXT("[Python] Ready: %s"), *PythonPath);
    return PythonPath;
}
