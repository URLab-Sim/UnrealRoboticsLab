# ProtoSpec

ProtoSpec is an IDL-driven redesign of the MJCF authoring layer. One schema
file describes the model format; a generator emits the C++ object model,
serialization and reflection tables from it. ProtoSpec owns the FILE boundary
only — reading MJCF into a document and writing a document back out.
Everything past that boundary is mjSpec and MuJoCo's own compiler, so
correctness is defined as byte-exact agreement with stock MuJoCo over its own
corpus, enforced by a round-trip differential.

## How it works

```
src/xml/mjcf.schema         MuJoCo's own grammar: every element, field, type,
        │                   default, union and reference relationship
        ▼
protospec_gen/              the generator (pure Python, no deps)
        │   emit.py         → lib/generated/   C++ types, XML binding tables,
        │                     reflection, keywords, defaults   (11 files)
        ▼
lib/                        handwritten library around the generated core
```

Generated code is **checked in** and byte-gated: `uv run python -m
protospec_gen.emit --check` fails if `lib/generated/` disagrees with the
schema by a single byte, so drift between schema and code cannot exist.

The object model is deliberately plain: generated structs of owned values,
every optional field presence-tracked (`std::optional`), references stored by
name with typed wrappers. No hidden compiler state, no pointers into a graph
— a `Model` is a value you can copy, diff, and serialize.

The layers on top:

- **`lib/io/`** — MJCF reader/writer (vendored tinyxml2), table-driven by the
  generated XML binding, with handwritten quirk handlers for the format's
  irregular corners. MuJoCo-free.
- **`lib/core/`** — canonicalization resolvers (orientation and inertia
  spellings fold into canonical quat/diaginertia at parse end). MuJoCo-free.
- **`lib/sdk/`** — the emission-profile seam and the default-class query
  (below). Its authoring verbs are a test fixture and live under `lib/test/`.

## The SDK

`lib/test/protospec/sdk.h` is a pure tree library over the generated types —
it is written once against the reflection/visit hooks and never needs
regenerating when the schema grows. It is a test fixture: the host that ships
ProtoSpec brings its own document profile and its own authoring UI, so only
the profile seam and the default-class query below are shipped surface.

- `builders.h` — typed `Add*` verbs that insert into the right child list
  (`AddBody`, `AddPrimitive`, `AddFreeJoint`, `AddMaterial`, …).
- `traversal.h` — `World`, `Find<T>`, `ForEachOfType<T>`, path-to-element.
- `parents.h` — `ParentMap`, the upward index (shipped: the default-class
  query needs it).
- `refs.h` — typed reference handling: `SetRef`, `Resolve`, `FindReferrers`,
  `Rename` (referrer-safe), `DeleteRecursive`.
- `classes.h` — defaults-class queries (`Effective`, and the allocation-free
  per-field `EffectiveField` / `EffectiveRef`). Shipped surface: an editor
  resolves an inherited value while a document is being edited.
- `class_edits.h` — the mutating class transforms (`FlattenDefaults`,
  `ExtractClass`).
- `edits.h` — `Duplicate`, `Reparent`.

### Emission profiles

The object model above is one *profile*. The SDK and the MJCF reader/writer are
generic over a profile tag `P` (`profile.h`) carrying five policies — `Doc`,
`Str`, `Tree`, `Ref`, `Ident` — so a host with its own storage (presence-wrapped
engine properties, a component hierarchy, its own string type) runs the same
algorithms on its own objects. `plain_profile.h` is the reference profile and
the default for every verb, so a call site that never mentions a profile reads
exactly as it always did.

A complete load → edit → save round trip (`lib/test/test_public_api.cc` runs
exactly this):

```cpp
#include "protospec/sdk.h"
namespace mj  = ps::mjcf;
namespace sdk = ps::sdk;

auto parsed = ps::mjcf::io::ParseMjcfString(xml, "hello.xml");
mj::Model& model = *parsed.model;

mj::Body& box = sdk::AddBody(sdk::World(model), "box");
box.pos = std::array<double, 3>{0, 0, 1};
sdk::AddFreeJoint(box, "box_free");
mj::Geom& g = sdk::AddPrimitive(box, mj::GeomType::box, "box_geom");

mj::Material& mat = sdk::AddMaterial(model, "grid_mat");
sdk::SetRef(g.material, mat);            // typed, name-backed reference

sdk::Save(model, "hello.xml");           // canonical MJCF back to disk
```

## Correctness

The round-trip differential is the permanent net: parse a corpus model, write
it back, `mj_loadXML` the result, and diff that `mjModel` field-by-field
(every sizes int, name table, and pointer array) against a stock `mj_loadXML`
of the original file. `lib/harness/model_diff_lib.cc` is the comparison core;
`mj_model_diff` is its CLI and `tests/test_differential.py` drives the sweep.

The claim this suite enforces: **byte-exact vs the enclosing MuJoCo checkout**
over MuJoCo's own model corpus (last verified against main at 3.11.0,
2026-07-22).

### Running the corpus net

One entry point, same behaviour on both platforms. It builds `ps_roundtrip` and
`mj_model_diff` against the staged MuJoCo, runs the differential over the
corpus, and exits non-zero on anything but the recorded allowed failures:

```powershell
# Windows
./corpus_net.ps1                                    # defaults below
./corpus_net.ps1 -MujocoRoot <dir> -Corpus <dir> -BuildType Release
```

```sh
# Linux
./corpus_net.sh                                     # defaults below
./corpus_net.sh <mujoco-root> <corpus-dir> <build-type>
```

Defaults: MuJoCo at `../third_party/install/MuJoCo`, corpus at
`../third_party/MuJoCo/src`, `Release`. Exit 0 the net holds, 1 it does not,
2 the harness could not run. The verdict is `tools/corpus_net.py`, shared by
both scripts: every failure must be one of the four recorded allowed failures
(named, with their diagnoses, at the top of that file) *and* at least
`_PARITY_FLOOR_IDENTICAL` models must have round-tripped identically, so a run
that silently skipped its subject fails rather than passing empty.

## Building and testing

Everything runs from this `protospec/` directory. Python tooling uses
[uv](https://docs.astral.sh/uv/); the C++ library is a standalone CMake
project with a MuJoCo-free core.

```sh
# Generated code matches the schema, byte for byte.
uv run python -m protospec_gen.emit --check

# C++ core (object model, io, SDK) + unit tests. No MuJoCo needed.
cmake -S lib -B lib/build && cmake --build lib/build -j && ctest --test-dir lib/build

# Python suite: schema, generator, extractors, differentials.
uv run pytest
```

Tests that need MuJoCo *source* (the corpus study under `tools/`) default to
the enclosing checkout and honor `PROTOSPEC_MUJOCO_SRC` as an override. Tests
that need *prebuilt* binaries (the round-trip differential runs `ps_roundtrip`
and `mj_model_diff`) skip when those have not been built, so a plain
`uv run pytest` stays green everywhere.
