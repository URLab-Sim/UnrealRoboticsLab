// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// MuJoCo's authored-field flags, declared here because MuJoCo does not.
//
// A spec's three model-level blocks -- compiler, option, visual -- carry a bit
// per field saying the document wrote it. MuJoCo's attach conflict resolver
// reads those bits: with them it can tell "the child authored 0.002" from "the
// child never said", which are the same value and different intents when 0.002
// is also MuJoCo's default. Without them the resolver compares against the
// defaults instead, and that one case slips through under every policy,
// including error.
//
// Both functions are EXPORTED from the MuJoCo library and ABSENT from its
// installed headers: they live in `src/user/user_api.h` (`mjs_isAuthored` at
// :444, `mjs_setAuthored` at :447) and nothing under `include/mujoco` mentions
// them. So URLab declares them, and the cost of declaring them is stated
// plainly: a signature change upstream would link, or fail to link, rather than
// being caught by the compiler comparing against a header.
//
// What stands in for that header is a test. `URLab.MuJoCo.AttachPolicy.*`
// exercises both symbols end to end -- set the flag on a built spec, attach
// under each conflict policy, assert the resolution, and clear the flag to
// assert the conflict disappears -- so drift surfaces in the suite rather than
// in a user's session. The MuJoCo update checklist names it.
//
// This is not part of URLab's supported surface. It sits in Public only because
// the generated spec write and the test that guards it are in different
// modules, and both have to see one declaration rather than two.

struct mjSpec_;
typedef struct mjSpec_ mjSpec;

extern "C"
{
/** Nonzero when `field_ptr`, a field of the block `elem_ptr` owns, was authored. */
int mjs_isAuthored(const void* elem_ptr, const void* field_ptr);

/** Record whether `field_ptr` was authored. Unknown field pointers are ignored. */
void mjs_setAuthored(const void* elem_ptr, const void* field_ptr, int authored);
}
