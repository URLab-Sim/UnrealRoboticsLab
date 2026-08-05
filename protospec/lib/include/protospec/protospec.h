// ProtoSpec public API — everything, in one include.
//
// Pulls the whole supported surface: the object model, reflection, MJCF IO, the
// MuJoCo compile bridge, validation, and the ergonomic SDK (builders /
// traversal / refs / classes / attach + save). A consumer of this header links
// the full library (protospec, protospec_io, protospec_validate,
// protospec_bridge, protospec_sdk + protospec_sdk_io) and MuJoCo.
//
// Prefer the focused headers when you do not need all of it -- e.g. a pure tree
// tool includes only <protospec/model.h> + <protospec/sdk.h> and links neither
// the bridge nor MuJoCo.
#ifndef PROTOSPEC_PUBLIC_PROTOSPEC_H
#define PROTOSPEC_PUBLIC_PROTOSPEC_H

#include "protospec/model.h"
#include "protospec/reflect.h"
#include "protospec/io.h"
#include "protospec/compile.h"
#include "protospec/validate.h"

// The SDK umbrella + save surface live under the SDK's own protospec/ include
// root (protospec/lib/sdk/protospec/); reached here by relative path so this all-in-one
// header needs nothing beyond the physical repo layout.
#include "../../sdk/protospec/sdk.h"   // ps::sdk builders / traversal / refs / classes / attach
#include "../../sdk/protospec/save.h"  // ps::sdk::{Save, SaveAs, ExternalizeAssets}

#endif  // PROTOSPEC_PUBLIC_PROTOSPEC_H
