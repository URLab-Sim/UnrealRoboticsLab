// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Writing an authored <option> onto MuJoCo's option block.
//
// <option> is an ordinary spec element: its fields hold MJCF's own values
// in MJCF's own units, and only the ones the spec actually sets are
// written, so anything left unset keeps whatever the compiler decided. That is
// the whole reason presence is modelled rather than mirrored with a toggle.
//
// MuJoCo keeps the option block twice -- once on the spec, once on every model
// compiled from it -- and the live set_sim_options path edits a compiled model
// without recompiling. Both destinations are the same struct, so both are the
// same function.

#include "CoreMinimal.h"

#include "mujoco/mujoco.h"

class UMjFlag;
class UMjOption;

/**
 * Apply an authored `<option>` and its `<flag>` child to an option block.
 *
 * Only the two flag bits URLab surfaces in its own UI and RPC are mapped:
 * `sleep` and `multiccd`. The rest reach the compiler as MJCF text once the
 * spec is the compile input, and hand-copying MuJoCo's flag table here in
 * the meantime would only create a second place for it to be wrong.
 */
URLAB_API void MjApplyOption(const UMjOption* Option, const UMjFlag* Flags, mjOption& Out);

/** As above, onto a spec the compiler has not run over yet. */
URLAB_API void MjApplyOptionToSpec(const UMjOption* Option, const UMjFlag* Flags, mjSpec* Spec);

/** As above, onto a compiled model, which is what the live RPC edits. */
URLAB_API void MjApplyOptionToModel(const UMjOption* Option, const UMjFlag* Flags, mjModel* Model);
