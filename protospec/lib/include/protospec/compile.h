// ProtoSpec public API — compile a Model to MuJoCo.
//
// The `ps::mjcf` surface: Compile turns a const `ps::mjcf::Model` into a
// Compiled { mjModel*, Binding, CompileReport }; Recompile migrates live
// simulation state across a structural edit (DR-11). CompileOptions carries the
// in-memory VfsAsset list, the model base dir, and the auto-naming policy.
// Binding maps every tree element to the id MuJoCo assigned (name-based, stable
// under discardvisual/fusestatic) and offers typed reverse lookup + address
// sugar (QposAdr / DofAdr / ActId / ...).
//
// These headers forward-declare mjModel/mjData and never include <mujoco/...>,
// so a consumer can hold and query a Compiled/Binding without the engine on its
// own include path; linking protospec_bridge (and mujoco) is enough.
//
// The compile bridge lives directly in `ps::mjcf` (there is no `bridge`
// sub-namespace): a consumer writes `ps::mjcf::Compile`, `ps::mjcf::Binding`,
// etc. `protospec/lib/compile` is the implementation directory, not a namespace
// a consumer names.
#ifndef PROTOSPEC_PUBLIC_COMPILE_H
#define PROTOSPEC_PUBLIC_COMPILE_H

#include "protospec/model.h"

#include "../../compile/bridge.h"  // Compile, Recompile, Compiled, CompileOptions, VfsAsset, CompileToXml
#include "../../compile/binding.h"  // Binding (also pulled by compile.h; named for clarity)

#endif  // PROTOSPEC_PUBLIC_COMPILE_H
