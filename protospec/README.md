# ProtoSpec authoring generator and native helpers

This directory reads the pinned MuJoCo MJCF schema and generates Unreal element classes, typed accessors, field descriptors, defaults, enum keywords, and dispatch tables. The generated `UMjNodeComponent` tree is URLab's persistent authoring document. URLab reads and writes MJCF directly through those generated descriptors, then uses MuJoCo's native XML parser and compiler for the executable model.

`protospec_gen.frontend` parses MuJoCo's upstream schema with MuJoCo's parser and applies the small UE authoring overlay. `protospec_gen.emit_ue` emits `Source/URLab/{Public,Private}/MuJoCo/Gen` and the generated coverage test. Ordinary MJCF attributes, including upstream aliases, remain raw generated fields. Spatial spellings are normalized after import using the native MuJoCo orientation resolver and URLab's spatial adapter.

The retained C++ helper libraries have narrow roles:

- `protospec_core`: native-backed orientation resolution.
- `protospec_mjcf`: XML `<include>` expansion and source diagnostics.
- `tinyxml2`: XML parsing and writing used by the direct UE gateway.
- `protospec_harness`: native `mjModel` comparison for URLab editor parity tests.
- `urlab_mj_deps`: staged editor-only subprocess that calls native `mju_getXMLDependencies` and returns framed recursive dependency paths. A fatal native scanner error exits this process, not the editor.

There is no standalone persistent C++ MJCF object model, generic authoring SDK, or second reader/writer. MuJoCo remains the authority for physics semantics and compilation.

Regenerate and check the UE tree from this directory:

```sh
uv run python -m protospec_gen.emit_ue --out ..
uv run python -m protospec_gen.emit_ue --out .. --check
uv run pytest -q tests/test_emit_ue.py tests/test_frontend.py tests/test_mjdefaults.py tests/test_xmlref.py tests/test_corpus_coverage.py
```

`build.sh` and `build.ps1` build and stage the four retained C++ archives, required headers and the dependency-helper executable under URLab's `third_party/install/protospec`. The helper links the same staged MuJoCo as URLab; Linux's UE-libc++ build uses the matched native build cache to link its small standalone process. The URLab differential compile and import tests exercise the generated UE document → XML → native parser → native compiler path.

Editor import captures source XML, includes, nested-model dependencies and file resources on the generated root component. Unchanged files retain their captured bytes. The portable bundle writer emits current generated UE XML and writes the retained resource tree; changed imported mesh descriptions and texture source pixels are exported through URLab's existing OBJ/PNG converters into new bundle resources. It neither canonicalizes shortcuts through a new `mjspec` document nor interprets MuJoCo defaults itself. Native MuJoCo remains the compiler authority. Native-ignored unknown attributes on the pinned `frame`/`replicate` aliases are generated as a bounded reader policy: import warns and drops them, while ordinary elements stay strict. A changed upstream alias set stops generation for policy review.

Focused editor automation filters are `URLab.MuJoCo.DirectUeReader.NativeAliasIgnoredAttributes`, `URLab.MuJoCo.DirectUeReader.OrdinaryUnknownAttribute`, `URLab.MuJoCo.SourceBundle.NativeDependencyIsolation`, and `URLab.Export.EditedMeshAndTexturePortableBundle`. The last test mutates actual imported vertex/pixel data, hides its original source directory before bundle construction, then compiles the portable XML. Run with the project's Unreal editor commandlet, for example `-ExecCmds="Automation RunTests URLab.Export.EditedMeshAndTexturePortableBundle" -Unattended -NullRHI -NoSound -TestExit="Automation Test Queue Empty"`. The current full import-factory corpus uses `URLab.Gen.Profile.FullCorpusPortableEditor` over the native-valid inventory; results are pending in `audit_docs/implementation_runs/full_corpus_20260927`.

## Refreshing after a MuJoCo source bump

The selected source checkout, installed headers, and native library must come
from the same commit. `refresh_mj_defaults.py` checks the install's
`INSTALLED_SHA.txt`, selected headers, and the compiled native version before
writing a snapshot. Generation rejects a snapshot from a different source
revision, and fails if any probed field is neither defaulted nor explicitly
skipped.

On Linux, when MuJoCo was built with Unreal's Clang/libc++ toolchain, make a
standalone probe cache from that build's CMake cache. This links Unreal's
static C++ runtime into the temporary probe and uses the host glibc; it does
not change the staged MuJoCo library:

```sh
uv run python tools/make_probe_cmake_cache.py \
  --source ../third_party/MuJoCo/src \
  --mujoco ../third_party/install/MuJoCo \
  --source-cache ../third_party/MuJoCo/src/build/CMakeCache.txt \
  --out /tmp/mujoco-probe-toolchain.txt
uv run python tools/refresh_mj_defaults.py \
  --source ../third_party/MuJoCo/src \
  --mujoco ../third_party/install/MuJoCo \
  --cmake-cache /tmp/mujoco-probe-toolchain.txt \
  --out protospec_gen/mujoco_defaults.json
uv run python -m protospec_gen.emit_ue --out ..
uv run python -m protospec_gen.emit_ue --out .. --check
```

For an upstream candidate, one command can resolve a requested SHA (or resolve
`HEAD` once), prepare an isolated source checkout and native install, then run
the rehearsal, ProtoSpec tests, and documentation-anchor drift check. The
source and install may be supplied again to rerun the checks cheaply:

```sh
uv run python tools/prepare_upgrade.py \
  --target HEAD \
  --workdir /tmp/urlab-mujoco-candidate

# Reuse an already checked out and built candidate:
uv run python tools/prepare_upgrade.py \
  --target <40-character-sha> \
  --source /tmp/mujoco-candidate \
  --install /tmp/mujoco-candidate-install \
  --workdir /tmp/urlab-mujoco-candidate-check
```

The command requires Git, CMake, Ninja, a C++20 compiler, `uv`, and pytest. It
records each stage and its log under the work directory. Candidate tests are
reported separately from exact-count/baseline tests, which run against the
committed MuJoCo pin. The report also lists new schema attributes and retired
classification/documentation entries. For an existing UE
Clang/libc++ install, it derives the host-runnable probe cache from the matching
source `build/CMakeCache.txt`; `--probe-cache` may specify that cache explicitly.
All source, build, install, probe, and generated files stay in the supplied
scratch area. No production pin, staged library, generated UE file, or committed
defaults snapshot is changed.

This checks default probing, schema and UE generation, candidate corpus tests,
pin regressions, and documentation-anchor drift. Old corpus XML that the
candidate native library rejects remains a visible migration blocker even when
generation succeeds. A new schema can also stop at an unsupported authoring
policy or native API change; the report names the failing stage and log. Passing
it is evidence for this candidate's defaults and generator, not a full Unreal
runtime upgrade. Updating the runtime still needs the coordinated
gitlink/install/profile change, Unreal build, and focused editor/model
validation.

The refresh derives default probes from upstream `spec`/`field=` bindings and
public `mjs_default*` functions; it does not depend on the retired spec-write
planner. New ordinary fields and enum keywords flow through generated UE
descriptors. A previously bound native default becoming unbound, an ambiguous
native member spelling, or a new ordinary field with no public native member
stops refresh with the field name. Custom XML/compiler aliases are recorded as
explicit `unbound` skips when no public member exists. The remaining manual
authoring policy is the angle-unit overlay where upstream lacks a unit facet;
new asset formats, reference semantics outside upstream `ref<>` and standard
name/type pairs, or native public-API changes still need focused review.
