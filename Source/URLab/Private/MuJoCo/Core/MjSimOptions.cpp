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
#include "MuJoCo/Core/MjSimOptions.h"
#include "Utils/URLabLogging.h"
#include "Misc/MessageDialog.h"
#include "HAL/PlatformProcess.h"
#include "Misc/App.h"

#if WITH_EDITOR
    #include "Dialog/SCustomDialog.h"
    #include "Widgets/Input/SHyperlink.h"
    #include "Widgets/Input/SMultiLineEditableTextBox.h"
    #include "Widgets/SBoxPanel.h"
#endif

#include <mujoco/mujoco.h>

void FMuJoCoOptions::ApplyToSpec(mjSpec* Spec) const
{
    if (!Spec) return;

    if (bOverride_MemoryMB)
    {
        Spec->memory = static_cast<mjtSize>(MemoryMB) * 1024 * 1024;
    }

    Spec->option.timestep = Timestep;

    // Gravity: UE cm/s² → MuJoCo m/s², negate Y for handedness
    Spec->option.gravity[0] =  Gravity.X / 100.0;
    Spec->option.gravity[1] = -Gravity.Y / 100.0;
    Spec->option.gravity[2] =  Gravity.Z / 100.0;

    Spec->option.wind[0] =  Wind.X / 100.0;
    Spec->option.wind[1] = -Wind.Y / 100.0;
    Spec->option.wind[2] =  Wind.Z / 100.0;

    Spec->option.magnetic[0] =  Magnetic.X;
    Spec->option.magnetic[1] = -Magnetic.Y;
    Spec->option.magnetic[2] =  Magnetic.Z;

    Spec->option.density = Density;
    Spec->option.viscosity = Viscosity;
    Spec->option.impratio = Impratio;
    Spec->option.tolerance = Tolerance;
    Spec->option.iterations = Iterations;
    Spec->option.ls_iterations = LsIterations;

    if (bOverride_Integrator) Spec->option.integrator = (int)Integrator;
    if (bOverride_Cone)       Spec->option.cone = (int)Cone;
    if (bOverride_Solver)     Spec->option.solver = (int)Solver;

    if (bOverride_NoslipIterations) Spec->option.noslip_iterations = NoslipIterations;
    if (bOverride_NoslipTolerance)  Spec->option.noslip_tolerance = NoslipTolerance;
    if (bOverride_CCD_Iterations)   Spec->option.ccd_iterations = CCD_Iterations;
    if (bOverride_CCD_Tolerance)    Spec->option.ccd_tolerance = CCD_Tolerance;

    // MultiCCD
    constexpr int MJ_ENBL_MULTICCD = 1 << 4;
    if (bEnableMultiCCD)
        Spec->option.enableflags |= MJ_ENBL_MULTICCD;
    else
        Spec->option.enableflags &= ~MJ_ENBL_MULTICCD;

    // Sleep
    constexpr int MJ_ENBL_SLEEP = 1 << 5;
    if (bEnableSleep)
    {
        Spec->option.enableflags |= MJ_ENBL_SLEEP;
        Spec->option.sleep_tolerance = SleepTolerance;
    }
}

void FMuJoCoOptions::ApplyOverridesToModel(mjModel* Model) const
{
    if (!Model) return;

    if (bOverride_Timestep) Model->opt.timestep = Timestep;

    if (bOverride_Gravity)
    {
        Model->opt.gravity[0] =  Gravity.X / 100.0;
        Model->opt.gravity[1] = -Gravity.Y / 100.0;
        Model->opt.gravity[2] =  Gravity.Z / 100.0;
    }

    if (bOverride_Wind)
    {
        Model->opt.wind[0] =  Wind.X / 100.0;
        Model->opt.wind[1] = -Wind.Y / 100.0;
        Model->opt.wind[2] =  Wind.Z / 100.0;
    }

    if (bOverride_Magnetic)
    {
        Model->opt.magnetic[0] =  Magnetic.X;
        Model->opt.magnetic[1] = -Magnetic.Y;
        Model->opt.magnetic[2] =  Magnetic.Z;
    }

    if (bOverride_Density)      Model->opt.density = Density;
    if (bOverride_Viscosity)    Model->opt.viscosity = Viscosity;
    if (bOverride_Impratio)     Model->opt.impratio = Impratio;
    if (bOverride_Tolerance)    Model->opt.tolerance = Tolerance;
    if (bOverride_Iterations)   Model->opt.iterations = Iterations;
    if (bOverride_LsIterations) Model->opt.ls_iterations = LsIterations;

    if (bOverride_Integrator) Model->opt.integrator = (int)Integrator;
    if (bOverride_Cone)       Model->opt.cone = (int)Cone;
    if (bOverride_Solver)     Model->opt.solver = (int)Solver;

    if (bOverride_NoslipIterations) Model->opt.noslip_iterations = NoslipIterations;
    if (bOverride_NoslipTolerance)  Model->opt.noslip_tolerance = NoslipTolerance;
    if (bOverride_CCD_Iterations)   Model->opt.ccd_iterations = CCD_Iterations;
    if (bOverride_CCD_Tolerance)    Model->opt.ccd_tolerance = CCD_Tolerance;

    // MultiCCD
    constexpr int MJ_ENBL_MULTICCD = 1 << 4;
    if (bEnableMultiCCD)
        Model->opt.enableflags |= MJ_ENBL_MULTICCD;
    else
        Model->opt.enableflags &= ~MJ_ENBL_MULTICCD;

    // Sleep
    constexpr int MJ_ENBL_SLEEP = 1 << 5;
    if (bEnableSleep)
    {
        Model->opt.enableflags |= MJ_ENBL_SLEEP;
        Model->opt.sleep_tolerance = SleepTolerance;
    }
    else
    {
        Model->opt.enableflags &= ~MJ_ENBL_SLEEP;
    }
}

namespace
{
    template <typename TValue,
              typename FHasOverride,
              typename FGetValue,
              typename FApply,
              typename FEquals,
              typename FFormat>
    void ResolveField(const TArray<const FMuJoCoOptions*>& Options,
                      const TArray<int32>& Priorities,
                      const TArray<FString>& Names,
                      mjSpec* Spec,
                      const TCHAR* FieldName,
                      FHasOverride HasOverride,
                      FGetValue    GetValue,
                      FApply       Apply,
                      FEquals      Equals,
                      FFormat      Format,
                      TArray<FString>& OutDialogLines)
    {
        struct FEntry
        {
            int32   Index;
            int32   Priority;
            TValue  Value;
            FString Name;
        };

        TArray<FEntry> Contributors;
        for (int32 i = 0; i < Options.Num(); ++i)
        {
            if (!Options[i]) continue;
            if (!HasOverride(Options[i])) continue;
            Contributors.Add(FEntry{ i, Priorities[i], GetValue(Options[i]), Names[i] });
        }

        if (Contributors.Num() == 0) return;

        if (Contributors.Num() == 1)
        {
            Apply(Spec, Contributors[0].Value);
            return;
        }

        // Multi-contributor. First check if all values agree - silent in that case.
        bool bAllAgree = true;
        const TValue& Reference = Contributors[0].Value;
        for (int32 i = 1; i < Contributors.Num(); ++i)
        {
            if (!Equals(Contributors[i].Value, Reference))
            {
                bAllAgree = false;
                break;
            }
        }
        if (bAllAgree)
        {
            Apply(Spec, Reference);
            return;
        }

        // Real disagreement: sort by (priority desc, index asc).
        Contributors.Sort([](const FEntry& A, const FEntry& B) {
            if (A.Priority != B.Priority) return A.Priority > B.Priority;
            return A.Index < B.Index;
        });

        const FEntry& Winner = Contributors[0];
        Apply(Spec, Winner.Value);

        // Build contributor summary string.
        FString Summary;
        for (int32 i = 0; i < Contributors.Num(); ++i)
        {
            if (i > 0) Summary += TEXT(", ");
            Summary += FString::Printf(TEXT("%s(pri=%d, val=%s)"),
                *Contributors[i].Name,
                Contributors[i].Priority,
                *Format(Contributors[i].Value));
        }

        UE_LOG(LogURLab, Warning,
            TEXT("[SimOptions] '%s' conflict across %d articulations: %s. Applying %s's value %s."),
            FieldName, Contributors.Num(), *Summary,
            *Winner.Name, *Format(Winner.Value));

        // Detect priority tie with the winner.
        bool bTie = false;
        for (int32 i = 1; i < Contributors.Num(); ++i)
        {
            if (Contributors[i].Priority == Winner.Priority)
            {
                bTie = true;
                break;
            }
        }
        if (bTie)
        {
            FString TiedNames;
            for (const FEntry& E : Contributors)
            {
                if (E.Priority != Winner.Priority) continue;
                if (!TiedNames.IsEmpty()) TiedNames += TEXT(", ");
                TiedNames += E.Name;
            }
            UE_LOG(LogURLab, Warning,
                TEXT("[SimOptions] '%s' ambiguous priority tie between %s (all pri=%d). "
                     "Fell back to actor iteration order; applied %s's value. "
                     "Set AMjArticulation.SimOptionsPriority to disambiguate."),
                FieldName, *TiedNames, Winner.Priority, *Winner.Name);
        }

        OutDialogLines.Add(FString::Printf(
            TEXT("%s (%d articulations disagreed%s)\n"
                 "     APPLIED: %s value %s (priority %d)"),
            FieldName,
            Contributors.Num(),
            bTie ? TEXT(", priority tie") : TEXT(""),
            *Winner.Name,
            *Format(Winner.Value),
            Winner.Priority));
    }
}

void FMuJoCoOptions::Resolve(const TArray<const FMuJoCoOptions*>& Options,
                             const TArray<int32>& Priorities,
                             const TArray<FString>& ContributorNames,
                             mjSpec* TargetSpec)
{
    if (!TargetSpec) return;
    if (Options.Num() == 0) return;

    checkf(Options.Num() == Priorities.Num() && Options.Num() == ContributorNames.Num(),
        TEXT("FMuJoCoOptions::Resolve: parallel arrays must have equal length (%d/%d/%d)"),
        Options.Num(), Priorities.Num(), ContributorNames.Num());

    TArray<FString> DialogLines;

    // --- Timing ---
    ResolveField<float>(Options, Priorities, ContributorNames, TargetSpec, TEXT("Timestep"),
        [](const FMuJoCoOptions* O){ return O->bOverride_Timestep; },
        [](const FMuJoCoOptions* O){ return O->Timestep; },
        [](mjSpec* S, float V){ S->option.timestep = V; },
        [](float A, float B){ return FMath::IsNearlyEqual(A, B, 1e-12f); },
        [](float V){ return FString::Printf(TEXT("%.6g"), V); },
        DialogLines);

    ResolveField<int32>(Options, Priorities, ContributorNames, TargetSpec, TEXT("MemoryMB"),
        [](const FMuJoCoOptions* O){ return O->bOverride_MemoryMB; },
        [](const FMuJoCoOptions* O){ return O->MemoryMB; },
        [](mjSpec* S, int32 V){ S->memory = static_cast<mjtSize>(V) * 1024 * 1024; },
        [](int32 A, int32 B){ return A == B; },
        [](int32 V){ return FString::Printf(TEXT("%d"), V); },
        DialogLines);

    // --- Physics Environment ---
    // Gravity / Wind / Magnetic do the UE -> MJ coordinate conversion inside Apply.
    ResolveField<FVector>(Options, Priorities, ContributorNames, TargetSpec, TEXT("Gravity"),
        [](const FMuJoCoOptions* O){ return O->bOverride_Gravity; },
        [](const FMuJoCoOptions* O){ return O->Gravity; },
        [](mjSpec* S, FVector V){
            S->option.gravity[0] =  V.X / 100.0;
            S->option.gravity[1] = -V.Y / 100.0;
            S->option.gravity[2] =  V.Z / 100.0;
        },
        [](FVector A, FVector B){ return A.Equals(B, 1e-6); },
        [](FVector V){ return V.ToString(); },
        DialogLines);

    ResolveField<FVector>(Options, Priorities, ContributorNames, TargetSpec, TEXT("Wind"),
        [](const FMuJoCoOptions* O){ return O->bOverride_Wind; },
        [](const FMuJoCoOptions* O){ return O->Wind; },
        [](mjSpec* S, FVector V){
            S->option.wind[0] =  V.X / 100.0;
            S->option.wind[1] = -V.Y / 100.0;
            S->option.wind[2] =  V.Z / 100.0;
        },
        [](FVector A, FVector B){ return A.Equals(B, 1e-6); },
        [](FVector V){ return V.ToString(); },
        DialogLines);

    ResolveField<FVector>(Options, Priorities, ContributorNames, TargetSpec, TEXT("Magnetic"),
        [](const FMuJoCoOptions* O){ return O->bOverride_Magnetic; },
        [](const FMuJoCoOptions* O){ return O->Magnetic; },
        [](mjSpec* S, FVector V){
            S->option.magnetic[0] =  V.X;
            S->option.magnetic[1] = -V.Y;
            S->option.magnetic[2] =  V.Z;
        },
        [](FVector A, FVector B){ return A.Equals(B, 1e-6); },
        [](FVector V){ return V.ToString(); },
        DialogLines);

    ResolveField<float>(Options, Priorities, ContributorNames, TargetSpec, TEXT("Density"),
        [](const FMuJoCoOptions* O){ return O->bOverride_Density; },
        [](const FMuJoCoOptions* O){ return O->Density; },
        [](mjSpec* S, float V){ S->option.density = V; },
        [](float A, float B){ return FMath::IsNearlyEqual(A, B, 1e-12f); },
        [](float V){ return FString::Printf(TEXT("%.6g"), V); },
        DialogLines);

    ResolveField<float>(Options, Priorities, ContributorNames, TargetSpec, TEXT("Viscosity"),
        [](const FMuJoCoOptions* O){ return O->bOverride_Viscosity; },
        [](const FMuJoCoOptions* O){ return O->Viscosity; },
        [](mjSpec* S, float V){ S->option.viscosity = V; },
        [](float A, float B){ return FMath::IsNearlyEqual(A, B, 1e-12f); },
        [](float V){ return FString::Printf(TEXT("%.6g"), V); },
        DialogLines);

    // --- Solver ---
    ResolveField<float>(Options, Priorities, ContributorNames, TargetSpec, TEXT("Impratio"),
        [](const FMuJoCoOptions* O){ return O->bOverride_Impratio; },
        [](const FMuJoCoOptions* O){ return O->Impratio; },
        [](mjSpec* S, float V){ S->option.impratio = V; },
        [](float A, float B){ return FMath::IsNearlyEqual(A, B, 1e-12f); },
        [](float V){ return FString::Printf(TEXT("%.6g"), V); },
        DialogLines);

    ResolveField<float>(Options, Priorities, ContributorNames, TargetSpec, TEXT("Tolerance"),
        [](const FMuJoCoOptions* O){ return O->bOverride_Tolerance; },
        [](const FMuJoCoOptions* O){ return O->Tolerance; },
        [](mjSpec* S, float V){ S->option.tolerance = V; },
        [](float A, float B){ return FMath::IsNearlyEqual(A, B, 1e-12f); },
        [](float V){ return FString::Printf(TEXT("%.6g"), V); },
        DialogLines);

    ResolveField<int32>(Options, Priorities, ContributorNames, TargetSpec, TEXT("Iterations"),
        [](const FMuJoCoOptions* O){ return O->bOverride_Iterations; },
        [](const FMuJoCoOptions* O){ return (int32)O->Iterations; },
        [](mjSpec* S, int32 V){ S->option.iterations = V; },
        [](int32 A, int32 B){ return A == B; },
        [](int32 V){ return FString::Printf(TEXT("%d"), V); },
        DialogLines);

    ResolveField<int32>(Options, Priorities, ContributorNames, TargetSpec, TEXT("LsIterations"),
        [](const FMuJoCoOptions* O){ return O->bOverride_LsIterations; },
        [](const FMuJoCoOptions* O){ return (int32)O->LsIterations; },
        [](mjSpec* S, int32 V){ S->option.ls_iterations = V; },
        [](int32 A, int32 B){ return A == B; },
        [](int32 V){ return FString::Printf(TEXT("%d"), V); },
        DialogLines);

    ResolveField<int32>(Options, Priorities, ContributorNames, TargetSpec, TEXT("Integrator"),
        [](const FMuJoCoOptions* O){ return O->bOverride_Integrator; },
        [](const FMuJoCoOptions* O){ return (int32)O->Integrator; },
        [](mjSpec* S, int32 V){ S->option.integrator = V; },
        [](int32 A, int32 B){ return A == B; },
        [](int32 V){ return FString::Printf(TEXT("%d"), V); },
        DialogLines);

    ResolveField<int32>(Options, Priorities, ContributorNames, TargetSpec, TEXT("Cone"),
        [](const FMuJoCoOptions* O){ return O->bOverride_Cone; },
        [](const FMuJoCoOptions* O){ return (int32)O->Cone; },
        [](mjSpec* S, int32 V){ S->option.cone = V; },
        [](int32 A, int32 B){ return A == B; },
        [](int32 V){ return FString::Printf(TEXT("%d"), V); },
        DialogLines);

    ResolveField<int32>(Options, Priorities, ContributorNames, TargetSpec, TEXT("Solver"),
        [](const FMuJoCoOptions* O){ return O->bOverride_Solver; },
        [](const FMuJoCoOptions* O){ return (int32)O->Solver; },
        [](mjSpec* S, int32 V){ S->option.solver = V; },
        [](int32 A, int32 B){ return A == B; },
        [](int32 V){ return FString::Printf(TEXT("%d"), V); },
        DialogLines);

    ResolveField<int32>(Options, Priorities, ContributorNames, TargetSpec, TEXT("NoslipIterations"),
        [](const FMuJoCoOptions* O){ return O->bOverride_NoslipIterations; },
        [](const FMuJoCoOptions* O){ return (int32)O->NoslipIterations; },
        [](mjSpec* S, int32 V){ S->option.noslip_iterations = V; },
        [](int32 A, int32 B){ return A == B; },
        [](int32 V){ return FString::Printf(TEXT("%d"), V); },
        DialogLines);

    ResolveField<float>(Options, Priorities, ContributorNames, TargetSpec, TEXT("NoslipTolerance"),
        [](const FMuJoCoOptions* O){ return O->bOverride_NoslipTolerance; },
        [](const FMuJoCoOptions* O){ return O->NoslipTolerance; },
        [](mjSpec* S, float V){ S->option.noslip_tolerance = V; },
        [](float A, float B){ return FMath::IsNearlyEqual(A, B, 1e-12f); },
        [](float V){ return FString::Printf(TEXT("%.6g"), V); },
        DialogLines);

    ResolveField<int32>(Options, Priorities, ContributorNames, TargetSpec, TEXT("CCD_Iterations"),
        [](const FMuJoCoOptions* O){ return O->bOverride_CCD_Iterations; },
        [](const FMuJoCoOptions* O){ return (int32)O->CCD_Iterations; },
        [](mjSpec* S, int32 V){ S->option.ccd_iterations = V; },
        [](int32 A, int32 B){ return A == B; },
        [](int32 V){ return FString::Printf(TEXT("%d"), V); },
        DialogLines);

    ResolveField<float>(Options, Priorities, ContributorNames, TargetSpec, TEXT("CCD_Tolerance"),
        [](const FMuJoCoOptions* O){ return O->bOverride_CCD_Tolerance; },
        [](const FMuJoCoOptions* O){ return O->CCD_Tolerance; },
        [](mjSpec* S, float V){ S->option.ccd_tolerance = V; },
        [](float A, float B){ return FMath::IsNearlyEqual(A, B, 1e-12f); },
        [](float V){ return FString::Printf(TEXT("%.6g"), V); },
        DialogLines);

    // bEnableMultiCCD, bEnableSleep, SleepTolerance have no bOverride_ flag;
    // Manager-level scene toggles only, not per-articulation.

    if (DialogLines.Num() > 0)
    {
        UE_LOG(LogURLab, Warning,
            TEXT("[SimOptions] %d field(s) had cross-articulation conflicts. "
                 "See https://urlab-sim.github.io/UnrealRoboticsLab/guides/sim_options_priority/ "
                 "for resolution rules."),
            DialogLines.Num());
    }

#if WITH_EDITOR
    if (DialogLines.Num() > 0 && !FApp::IsUnattended())
    {
        static const TCHAR* GuideURL =
            TEXT("https://urlab-sim.github.io/UnrealRoboticsLab/guides/sim_options_priority/");

        FString Msg;
        Msg += FString::Printf(
            TEXT("SimOptions conflict detected between %d articulations.\n\n"),
            Options.Num());
        Msg += TEXT("MuJoCo stores one value per simulation-option field "
                    "(timestep, gravity, solver, ...) globally. Two or more "
                    "articulations in this scene set the same field to "
                    "different values. Each conflict below was resolved by "
                    "SimOptionsPriority; you can change the outcome without "
                    "editing MJCF.\n\nConflicts:\n");
        for (const FString& Line : DialogLines)
        {
            Msg += TEXT("  * ");
            Msg += Line;
            Msg += TEXT("\n\n");
        }
        Msg += TEXT("What to do:\n");
        Msg += TEXT("  * Accept the resolution: no action needed if the "
                    "winning value is correct for your scene.\n");
        Msg += TEXT("  * Pick a different winner: change SimOptionsPriority "
                    "on the articulations in the Details panel.\n");
        Msg += TEXT("  * Apply a full override via the MjManager: set the "
                    "field on AAMjManager -> Options. Manager values are a "
                    "full override applied post-compile and ignore the "
                    "articulation resolution entirely.\n");
        Msg += TEXT("\nFull per-contributor breakdown (every articulation's "
                    "value and priority) is in the Output Log under "
                    "LogURLab.\n\n"
                    "Press Dismiss to continue simulation.");

        TSharedRef<SCustomDialog> Dialog = SNew(SCustomDialog)
            .Title(NSLOCTEXT("URLab", "SimOptionsConflictTitle", "SimOptions Conflict"))
            .Buttons({ SCustomDialog::FButton(NSLOCTEXT("URLab", "Dismiss", "Dismiss")) })
            .Content()
            [
                SNew(SVerticalBox)
                + SVerticalBox::Slot().AutoHeight().Padding(4.0f)
                [
                    SNew(SMultiLineEditableTextBox)
                        .Text(FText::FromString(Msg))
                        .IsReadOnly(true)
                        .AutoWrapText(true)
                        .AlwaysShowScrollbars(false)
                        .Padding(FMargin(6.0f))
                ]
                + SVerticalBox::Slot().AutoHeight().Padding(8.0f, 6.0f)
                [
                    SNew(SHyperlink)
                        .Text(NSLOCTEXT("URLab", "OpenGuide", "Open SimOptions Priority guide in browser"))
                        .OnNavigate_Lambda([]()
                        {
                            FPlatformProcess::LaunchURL(GuideURL, nullptr, nullptr);
                        })
                ]
            ];

        Dialog->ShowModal();
    }
#endif
}
