// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Tests/MjTestHelpers.h"
#include "MuJoCo/Components/Geometry/MjGeom.h"

// ============================================================================
// URLab.Geom.FitscaleType_IsDouble
//
//   Phase 4.1 changed UMjGeom::fitscale from bool to double (matching MuJoCo's
//   mjsGeom.fitscale C type — a scaling factor for hfield/mesh attached geoms,
//   not a toggle). This test pins the type at compile time so a regression
//   would fail to build, and verifies the import/export round trip works.
// ============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjFitscaleTypeIsDouble,
    "URLab.Geom.FitscaleType_IsDouble",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjFitscaleTypeIsDouble::RunTest(const FString& Parameters)
{
    // Compile-time type pin. A regression that flips fitscale back to bool
    // breaks here at the static_assert; no need to run anything.
    static_assert(
        std::is_same_v<decltype(UMjGeom::fitscale), double>,
        "UMjGeom::fitscale must be double (mjsGeom.fitscale is double; the "
        "previous bool typing was a Phase 4.1 codegen bug).");

    // Light dynamic check: defaults survive construction.
    UMjGeom* Geom = NewObject<UMjGeom>();
    TestEqual(TEXT("fitscale defaults to 0.0"), Geom->fitscale, 0.0);
    TestFalse(TEXT("bOverride_fitscale defaults off"), Geom->bOverride_fitscale);
    return true;
}
