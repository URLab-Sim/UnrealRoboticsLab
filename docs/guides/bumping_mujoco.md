# Bumping MuJoCo

This guide describes how to move URLab onto a new MuJoCo release. The codegen
is designed so common cases are zero-touch: in most version bumps you re-run
one script and ship. The rest of this doc covers the rare bumps where MuJoCo
adds something the rules don't yet know about.

## TL;DR

```bash
# 1. Move the submodule pointer to the new tag
cd third_party/MuJoCo/src && git fetch --tags && git checkout 3.9.0

# 2. Rebuild the install (Windows: .ps1, Linux/macOS: .sh)
cd ../ && ./build.sh                 # or .\build.ps1 -NoSubmoduleSync

# 3. Refresh all three snapshots + run the C++ codegen
cd ../../ && python Scripts/codegen/regen_all.py

# 4. Read any diagnostics the codegen printed (stderr). Apply rule edits
#    if it asked for any (see "Reading diagnostics" below). Re-run step 3.

# 5. Close the editor, rebuild + test
"$UE/Engine/Binaries/DotNET/UnrealBuildTool/UnrealBuildTool.exe" \
    url_projEditor Win64 Development "-Project=$URLPROJ/url_proj.uproject"
"$UE/Engine/Binaries/Win64/UnrealEditor-Cmd.exe" "$URLPROJ/url_proj.uproject" \
    -ExecCmds="Automation RunTests URLab" -Unattended -NullRHI -NoSound \
    -NoSplash -stdout -log -TestExit="Automation Test Queue Empty"
```

If step 3 printed no diagnostics and step 5 ran green, you are done.

## The pipeline

`Scripts/codegen/regen_all.py` runs four scripts in order:

1. **`build_mjspec_snapshot.py`** parses `third_party/install/MuJoCo/include/mujoco/mjspec.h`
   and writes `Scripts/mjspec_snapshot.json` — every `mjsX` struct's field
   list plus every `mjs_setTo*` function signature.

2. **`build_mjxmacro_snapshot.py`** parses `mjxmacro.h` and writes
   `Scripts/mjxmacro_snapshot.json` — the `mjModel` / `mjData` array
   layouts (outer dim + per-row stride) URLab needs for `MjBind.h` views.

3. **`build_mjcf_schema_snapshot.py`** parses
   `third_party/MuJoCo/src/src/xml/xml_native_reader.cc` and writes
   `Scripts/mjcf_schema_snapshot.json` — every MJCF element with its
   attribute list and child structure. Auto-detects the MuJoCo version
   from `mjVERSION_HEADER`.

4. **`generate_ue_components.py`** reads all three snapshots plus
   `codegen_rules.json` and emits `Source/URLab/{Public,Private}/MuJoCo/Components/**/*.{h,cpp}`
   between `// --- CODEGEN_*_START ---` / `// --- CODEGEN_*_END ---`
   markers. Hand-edits OUTSIDE the markers are preserved across regens.

Step 4 also prints **drift diagnostics** to stderr when the snapshots
surface something the rules don't cover.

## Reading diagnostics

After every codegen run, watch for a block that starts with:

```
--- codegen diagnostics (N) ---
[diagnostic] schema has top-level element 'foo' but no category in codegen_rules.json ...
```

Each line names the exact rule path to edit. The categories of diagnostic:

### `schema has top-level element 'X' but no category`

MuJoCo added a new top-level MJCF element. Pick one:
- **Add a category** in `codegen_rules.json` → `categories.X` with a
  `base_class_name`, `base_class_header`, `mjs_struct`, and
  `schema_common_block: "X.attrs"`. Codegen will emit `UMjX` next run.
- **Mark unmodeled** in `intentionally_unmodeled_elements` with a
  one-line reason (e.g. UE has its own primitives for that concept).
- **Treat as a container** by listing it in the `container_keys` set
  inside `_emit_drift_diagnostics`.

### `schema actuator/sensor subtype 'X' has no entry in categories.actuator/sensor.subtypes`

MuJoCo added a new actuator or sensor type. Add a subtype entry —
URLab will emit `UMjXActuator` or `UMjXSensor` next run, including the
common base attrs. If the type has a `mjs_setToX` preset function, also
add a `subtype_setto` entry pointing at the C function name; codegen
will marshal the C signature for you. Example:

```json
"actuator": {
  "subtypes": [
    ...,
    { "key": "newactuator", "enum_value": "NewActuator",
      "class_name": "UMjNewActuator", "header": "MjNewActuator.h" }
  ],
  "subtype_setto": {
    ...,
    "newactuator": { "call": "mjs_setToNewActuator" }
  }
}
```

### `mjs_setToX param 'Y' is not in the schema attrs and has no param_renames or setto_param_defaults entry`

MuJoCo added a parameter to an existing `mjs_setTo*` function. Decide:
- **It maps to an MJCF attr** that URLab should expose: add it to the
  per-subtype `actuator_types[subtype]` extras (the schema already has
  it from the auto-extract). Codegen will emit a UPROPERTY and pass
  the user value through to the setto call.
- **It's an internal param** the user doesn't control: pin a sentinel
  in `setto_param_defaults[fn_name][param]` (e.g. `"-1.0"` for doubles,
  `"0"` for ints, or another meaningful default). Codegen will hard-code
  the sentinel in the call site.
- **The param was renamed in MuJoCo**: add a `param_renames` entry in
  the subtype's setto rule.

### `attr 'X' (used by Y) falls back to default_type ('float')`

A schema attr's UE type defaulted to `float`. If the attr is actually
an int / TArray / bool / FString, add it to `type_mappings`:

```json
"type_mappings": {
  ...,
  "newcount": "TArray<int32>"
}
```

## Rare cases the diagnostics don't catch

### A `mjsX` struct field was renamed

The auto-resolver in `_resolve_mjs_field` covers most renames (e.g.
`focal` → `focal_length` via the `+ "_length"` heuristic? No — that's
handled explicitly), but a brand-new rename surfaces as a compile error
(`error C2039: 'oldname' is not a member of 'mjsX'`). Fix by adding the
rename to the element's `attr_to_mjs_field` block:

```json
"camera": {
  "attr_to_mjs_field": { "target": "targetbody" }
}
```

### A new enum-valued attr

If MuJoCo adds an attr whose XML values are a fixed enum (`"perspective"`,
`"orthographic"`, ...), use `xml_enum_attrs` so the codegen emits the
UE-side enum, the XML parse, and the mjs write together:

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

The UE enum (`EMjCameraProjection`) is hand-declared in the component
header — codegen owns the bridge but not the enum decl itself, so a UE
enum bump is a one-time `.h` edit per attr.

### A `mjsEquality.data[]`-style packed attr

Some MuJoCo structs pack values into an array. URLab handles this via
`mjs_data_packed_attrs` rules:

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

### MuJoCo changed the spec API enough that the codegen-emitted call
### sites don't compile

If `mjs_addX` / `mjs_setString` / `mjs_attach` got new required args, the
codegen needs a one-shot Python edit in `generate_ue_components.py`. The
relevant helpers:

- `_emit_setto_call` — generates `mjs_setToX(...)` per actuator subtype.
- The various `_emit_X_export` / `_emit_X_import` helpers in
  `generate_ue_components.py` for canonicalisations, enums, and packed
  data.

These are uncommon — most MuJoCo versions only add fields/attrs, which
the existing emitters handle automatically.

## When NOT to use the codegen for a new element

Some elements need hand-rolling because they're not a pure
attribute-to-struct mapping:

- **`flexcomp`** uses `xml_passthrough_emission: true` because at spec
  time it builds a standalone MJCF fragment and re-parses it via
  `mj_parseXMLString + mjs_attach` (MuJoCo expands the flexcomp macro
  in its XML parser, not the spec compiler). The codegen still owns all
  the top-level UPROPERTYs and emits a `BuildSchemaAttrsXml()` helper —
  hand-rolling is limited to the surrounding `<flexcomp>...</flexcomp>`
  wrapper and the sub-element parsing/writing. See
  `MjFlexcomp.cpp::BuildFlexcompXml`.
- Sub-elements like `<contact>` / `<edge>` / `<elasticity>` / `<pin>`
  under `<flexcomp>` aren't modelled by the codegen yet (there's no
  per-sub-element UPROPERTY-group support). New sub-element attrs need
  manual handling in the host element's `.cpp`.

If you bump MuJoCo and `flexcomp` grows a new sub-element with rich
content, that's the case to widen the codegen for first — the rest of
URLab's components are already covered.

## Reference

| File | Owned by | Touch on a bump? |
|------|---------|------------------|
| `Scripts/mjspec_snapshot.json`           | `build_mjspec_snapshot.py`        | regen_all.py rewrites it; commit the result |
| `Scripts/mjxmacro_snapshot.json`         | `build_mjxmacro_snapshot.py`      | same |
| `Scripts/mjcf_schema_snapshot.json`      | `build_mjcf_schema_snapshot.py`   | same |
| `Scripts/codegen/codegen_rules.json`     | hand-written                      | only when diagnostics ask for it |
| `Scripts/codegen/generate_ue_components.py` | hand-written                   | only when MuJoCo changes the C-API shape |
| `Source/URLab/.../*.{h,cpp}` between `// --- CODEGEN_*_START ---` / `// --- CODEGEN_*_END ---` | regen_all.py | DO NOT hand-edit |
| Outside the markers | hand-written | preserved across regen |
| `third_party/MuJoCo/src/`                | submodule pointer                 | `git checkout <newtag>` in submodule, then re-run `build.sh`/`.ps1` |
| `Source/URLab/URLab.Build.cs` `SkipThirdPartyDriftChecks` | hand-written | leave `false` in commits; flip to `true` only locally if you need to build before committing the submodule pointer |

## Smoke-test checklist after a bump

- [ ] `python Scripts/codegen/regen_all.py` prints no diagnostics.
- [ ] `micromamba run -n mj python -m pytest Scripts/codegen/tests/` is green.
- [ ] `UnrealBuildTool` compiles `url_projEditor Win64 Development` without warnings new to this branch.
- [ ] Editor automation: `Automation RunTests URLab` is all-green.
- [ ] Open a representative imported model in the editor and verify it
      still loads + simulates.
- [ ] `git diff --stat` for the bump: expect changes in the three snapshot
      JSONs, the submodule pointer, optionally a handful of codegen-emitted
      `.h`/`.cpp` files, and only any rule edits the diagnostics asked for.
