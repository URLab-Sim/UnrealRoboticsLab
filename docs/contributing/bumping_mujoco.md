# Bumping MuJoCo

How to move URLab onto a newer MuJoCo. The codegen is built so common bumps are nearly zero-touch: re-run one script and ship. This page covers the routine path and the rarer bumps where MuJoCo adds something the rules do not yet know about.

URLab currently pins MuJoCo to upstream `main` (header version `3010000`, that is 3.10.0-dev), not a tagged release. The submodule lives at `third_party/MuJoCo/src/`. For the codegen internals this builds on, read [Codegen](codegen.md) first.

## TL;DR

```bash
# 1. Move the submodule pointer to the new commit (upstream main, or a tag)
cd third_party/MuJoCo/src && git fetch origin && git checkout <commit-or-tag>

# 2. Rebuild the install (Windows: .ps1, Linux/macOS: .sh)
cd .. && .\build.ps1 -NoSubmoduleSync       # or ./build.sh on Linux/macOS

# 3. Refresh all three snapshots + run the C++ codegen
cd ../../ && python Scripts/codegen/regen_all.py

# 4. Read any diagnostics the codegen printed (stderr). Apply rule edits
#    if it asked for any (see "Reading diagnostics"). Re-run step 3.

# 5. Close the editor, rebuild + test
"$UE/Engine/Binaries/DotNET/UnrealBuildTool/UnrealBuildTool.exe" \
    url_projEditor Win64 Development "-Project=$URLPROJ/url_proj.uproject"
"$UE/Engine/Binaries/Win64/UnrealEditor-Cmd.exe" "$URLPROJ/url_proj.uproject" \
    -ExecCmds="Automation RunTests URLab" -Unattended -NullRHI -NoSound \
    -NoSplash -stdout -log -TestExit="Automation Test Queue Empty"
```

If step 3 printed no diagnostics and step 5 ran green, you are done.

!!! note "Build script"
    The canonical builders are `third_party/build_all.{ps1,sh}` (all three deps) and the per-dep `third_party/MuJoCo/build.{ps1,sh}`. Use `-NoSubmoduleSync` (or `--no-submodule-sync`) when you have already checked out the new commit yourself, so the build does not snap the submodule back. The drift-check behaviour around these scripts is in [Building from Source](building.md).

## The pipeline

`Scripts/codegen/regen_all.py` runs four steps in order and writes all snapshots under `Scripts/codegen/snapshots/`:

1. **`build_mjxmacro_snapshot.py`** parses `mjxmacro.h` and writes `mjxmacro_snapshot.json` (the `mjModel` / `mjData` array layouts URLab needs for `MjBind.h` views).
2. **`build_mjcf_schema_snapshot.py`** parses `third_party/MuJoCo/src/src/xml/xml_native_reader.cc` and writes `mjcf_schema_snapshot.json` (every MJCF element with its attributes and child structure). It auto-detects the MuJoCo version from `mjVERSION_HEADER`.
3. **`build_introspect_snapshot.py`** does a libclang clang-AST scrape of `mjspec.h` and `mjmodel.h` and writes `introspect_snapshot.json` (every `mjsX` struct's fields, the `mjt*` enums, and every `mjs_setTo*` signature). This supersedes the retired `build_mjspec_snapshot.py`. It needs `clang.cindex` and a loadable libclang; if libclang is missing the step fails and `regen_all.py` falls back to the committed snapshot, silently skipping any header change in the bump. Run regen in an environment that has libclang (see [Environment](#environment)).
4. **`generate_ue_components.py`** reads all three snapshots plus `codegen_rules.json` and emits the per-component `.h` / `.cpp` files between the `CODEGEN_*_START` / `CODEGEN_*_END` markers, then clang-formats the output. Hand-edits outside the markers are preserved.

Step 4 prints drift diagnostics to stderr whenever the snapshots surface something the rules do not cover.

## Reading diagnostics

After each codegen run, watch for a block that starts with:

```
--- codegen diagnostics (N) ---
[diagnostic] schema has top-level element 'foo' but no category in codegen_rules.json ...
```

Each line names the exact rule path to edit.

### `schema has top-level element 'X' but no category`

MuJoCo added a new top-level MJCF element. Pick one:

- **Add a category** in `codegen_rules.json` under `categories.X` with `base_class_name`, `base_class_header`, `mjs_struct`, and `schema_common_block: "X.attrs"`. Codegen emits `UMjX` next run.
- **Mark unmodeled** in `intentionally_unmodeled_elements` with a one-line reason.
- **Treat as a container** by listing it in the `container_keys` set inside `_emit_drift_diagnostics`.

### `schema actuator/sensor subtype 'X' has no entry in categories.actuator/sensor.subtypes`

MuJoCo added an actuator or sensor type. Add a subtype entry and URLab emits `UMjXActuator` or `UMjXSensor` next run, including the common base attrs. If the type has an `mjs_setToX` preset function, also add a `subtype_setto` entry pointing at the C function name; codegen marshals the signature for you.

```json
"actuator": {
  "subtypes": [
    { "key": "newactuator", "enum_value": "NewActuator",
      "class_name": "UMjNewActuator", "header": "MjNewActuator.h" }
  ],
  "subtype_setto": {
    "newactuator": { "call": "mjs_setToNewActuator" }
  }
}
```

### `mjs_setToX param 'Y' is not in the schema attrs and has no param_renames or setto_param_defaults entry`

MuJoCo added a parameter to an existing `mjs_setTo*` function. Decide:

- **It maps to an MJCF attr** the user should control: add it to the per-subtype extras so codegen emits a UPROPERTY and passes the value through.
- **It is internal**: pin a sentinel in `setto_param_defaults[fn_name][param]` and codegen hard-codes it.
- **It was renamed**: add a `param_renames` entry in the subtype's setto rule.

### `attr 'X' (used by Y) falls back to default_type ('float')`

A schema attr's UE type defaulted to `float`. If it is really an int / array / bool / string, add it to `type_mappings`:

```json
"type_mappings": { "newcount": "TArray<int32>" }
```

## Rare cases the diagnostics do not catch

### An `mjsX` field was renamed

Most renames are handled by the auto-resolver, but a brand-new one surfaces as a compile error (`error C2039: 'oldname' is not a member of 'mjsX'`). Add the rename to the element's `attr_to_mjs_field` block:

```json
"camera": { "attr_to_mjs_field": { "target": "targetbody" } }
```

### A new enum-valued attr

If MuJoCo adds an attr whose XML values are a fixed enum, use `xml_enum_attrs` so codegen emits the UE enum bridge, the XML parse, and the mjs write together:

```json
"camera": {
  "xml_enum_attrs": {
    "projection": {
      "ue_property": "Projection",
      "ue_enum_type": "EMjCameraProjection",
      "mjs_field": "proj",
      "mjs_cast": "mjtProjection",
      "value_map": {
        "perspective":  ["Perspective",  "mjPROJ_PERSPECTIVE"],
        "orthographic": ["Orthographic", "mjPROJ_ORTHOGRAPHIC"]
      }
    }
  }
}
```

The UE enum itself is hand-declared in the component header; codegen owns the bridge but not the enum decl.

### A packed `data[]`-style attr

Some structs pack values into an array. URLab handles this with `mjs_data_packed_attrs`:

```json
"equality": {
  "mjs_data_packed_attrs": {
    "anchor": {
      "slot_range": [0, 3],
      "condition": "(EqualityType == EMjEqualityType::Connect) || (EqualityType == EMjEqualityType::Weld)",
      "export_op": "cm_to_m"
    }
  }
}
```

### The spec C-API changed shape

If `mjs_addX` / `mjs_setString` / `mjs_attach` gained new required args, codegen needs a one-shot Python edit in `generate_ue_components.py`. The relevant helpers are `_emit_setto_call` and the `_emit_X_export` / `_emit_X_import` family. This is uncommon; most bumps only add fields and attrs, which the emitters handle automatically.

## When not to use codegen for a new element

Some elements are not a pure attribute-to-struct mapping:

- **`flexcomp`** uses `xml_passthrough_emission: true` because at spec time it builds a standalone MJCF fragment and re-parses it via `mj_parseXMLString` + `mjs_attach`. Codegen still owns the top-level UPROPERTYs and emits `BuildSchemaAttrsXml()`; hand-rolling is limited to the wrapper and sub-element handling in `MjFlexcomp.cpp`.
- Sub-elements (`<contact>`, `<edge>`, `<elasticity>`, `<pin>`) under `<flexcomp>` are not modelled by codegen; new sub-element attrs need manual handling in the host `.cpp`.

If a bump grows `flexcomp` a new sub-element, that is the case to widen codegen for first; the rest of URLab's components are already covered.

## Reference

| File | Owned by | Touch on a bump? |
|---|---|---|
| `Scripts/codegen/snapshots/introspect_snapshot.json` | `build_introspect_snapshot.py` (libclang) | regen rewrites it; commit the result |
| `Scripts/codegen/snapshots/mjxmacro_snapshot.json` | `build_mjxmacro_snapshot.py` | same |
| `Scripts/codegen/snapshots/mjcf_schema_snapshot.json` | `build_mjcf_schema_snapshot.py` | same |
| `Scripts/codegen/codegen_rules.json` | hand-written | only when diagnostics ask |
| `Scripts/codegen/generate_ue_components.py` | hand-written | only when the MuJoCo C-API shape changes |
| `Source/URLab/.../*.{h,cpp}` between the `CODEGEN_*` markers | regen | do not hand-edit |
| outside the markers | hand-written | preserved across regen |
| `third_party/MuJoCo/src/` | submodule pointer | `git checkout <commit>`, then rebuild |
| `URLab.Build.cs` `SkipThirdPartyDriftChecks` | hand-written | leave `false` in commits |

## Environment

`regen_all.py`'s introspect step imports `clang.cindex` and loads libclang. Run regen and the codegen tests in an environment that has libclang; without it the introspect step fails silently and a header change is quietly skipped. Override the library path with `$LIBCLANG_LIBRARY_FILE` or `--libclang` if auto-detection fails.

## Checklist after a bump

- [ ] `python Scripts/codegen/regen_all.py` prints no diagnostics.
- [ ] `python -m pytest Scripts/codegen/tests/` is green.
- [ ] `UnrealBuildTool` compiles `url_projEditor Win64 Development` without new warnings.
- [ ] `Automation RunTests URLab` is all-green.
- [ ] A representative imported model still loads and simulates in the editor.
- [ ] `git diff --stat` shows the three snapshot JSONs, the submodule pointer, optionally a handful of codegen-emitted files, and only the rule edits the diagnostics asked for.

## Related

- [Codegen](codegen.md): the snapshot and generator details.
- [Building from Source](building.md): dependency drift checks and the build gate.
