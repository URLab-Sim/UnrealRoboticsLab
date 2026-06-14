# Codegen

URLab generates most of its MuJoCo component wrappers from the MuJoCo headers and schema. Read this before editing anything under `Source/URLab/Public/MuJoCo/Components/`, `Source/URLab/Private/MuJoCo/Components/`, or `Source/URLab/Public/MuJoCo/Generated/`, so a regen does not silently revert your work.

The goal of the pipeline is that a routine MuJoCo bump is one command: rebuild the snapshots, run the generator, ship. Hand-written rules cover only the URLab-side decisions that cannot be derived from MuJoCo's own metadata. For the end-to-end bump procedure see [Bumping MuJoCo](bumping_mujoco.md).

## The three snapshots

The codegen reads three JSON snapshots, all under `Scripts/codegen/snapshots/`. Each is rebuilt from the installed MuJoCo headers and submodule source.

| Snapshot | Built by | Captures |
|---|---|---|
| `mjxmacro_snapshot.json` | `build_mjxmacro_snapshot.py` | `mjModel` / `mjData` array layouts (outer dim plus per-row stride) from `mjxmacro.h`, used for `MjBind.h` views and the `MJOPTION` / `MJSTATISTIC` / `MJVISUAL` field tables |
| `mjcf_schema_snapshot.json` | `build_mjcf_schema_snapshot.py` | the MJCF schema (every element with its attribute list and child structure) from `xml_native_reader.cc`, plus the per-sensor objtype / reftype scrape. Auto-detects the MuJoCo version from `mjVERSION_HEADER` |
| `introspect_snapshot.json` | `build_introspect_snapshot.py` | a clang-AST scrape of `mjspec.h` / `mjmodel.h` / `mujoco.h`: `mjsX` struct fields, `mjt*` enum decls with doc comments, struct decls, `mjs_setTo*` signatures, `#define` constants |

!!! note "The introspect snapshot supersedes the old mjspec snapshot"
    There is no `build_mjspec_snapshot.py`. It is retired. The clang-AST introspect snapshot now supplies the `mjsX` struct fields, `mjt*` enums, and `mjs_setTo*` signatures that the old regex-based script used to scrape. The introspect step needs `clang.cindex` and a loadable libclang (see [Environment](#environment)).

Run `python Scripts/codegen/regen_all.py` to rebuild all three snapshots and then run the generator. Pass `--skip-codegen` to rebuild only the snapshots.

## The generator

`Scripts/codegen/generate_ue_components.py` reads:

1. The three snapshots above.
2. `Scripts/codegen/codegen_rules.json`, which holds URLab-side decisions: which fields to expose, how to type-map them, which categories use which layout.

It emits:

- `.h` and `.cpp` for every category subclass (single-uclass, multi-uclass, and no-subclass layouts).
- Marker-region content for each `CODEGEN_*_START` / `CODEGEN_*_END` pair in hand-written files.
- Full-file banner-mode artifacts under `Source/URLab/Public/MuJoCo/Generated/` (for example `MjOptionGenerated.h`).

The generator runs **clang-format** over its output, so emitted files match a clang-format pass across the tree. clang-format is a hard requirement: set `$CLANG_FORMAT`, put `clang-format` on PATH, or let it find VS 2022's bundled LLVM. Use clang-format 19.x to match what the tree was committed with; an unformatted emit would trip the `--check` drift gate.

CLI flags:

| Flag | Effect |
|---|---|
| `--check` | Exit non-zero if a regen would change anything. Used by the build gate. |
| `--dry-run` | Print diffs, write nothing. |
| `--strict` | Exit code 2 if any drift diagnostic fires. |
| `--require-introspect` | Fail if the introspect snapshot is missing rather than running on a stale one. |

## The build gate

`Scripts/build_and_test.ps1` runs the generator with `--check --strict --require-introspect` before invoking UBT. If the committed `Source/` has drifted from what the generator would emit, or any drift diagnostic fires, the build fails before wasting compile time. Re-run the generator without `--check`, commit the output, then re-run the script.

## Marker pairs in hand-written files

Most `Source/URLab/Private/MuJoCo/Components/*.cpp` files carry two marker pairs:

```cpp
// --- CODEGEN_IMPORT_START ---
// (codegen-emitted XML -> UPROPERTY assignments)
// --- CODEGEN_IMPORT_END ---

// --- CODEGEN_EXPORT_START ---
// (codegen-emitted UPROPERTY -> mjs spec field writes)
// --- CODEGEN_EXPORT_END ---
```

Inside the markers is codegen-owned; outside is hand-written and preserved across regens. Do not move logic across the boundary in one commit. Split it into a "prepare hand side" commit and a "regen" commit so each diff is reviewable.

`MjSensor.cpp` has extra markers:

```cpp
// --- CODEGEN_SENSOR_TYPE_SWITCH_START ---  (case bodies in ExportTo)
// --- CODEGEN_SENSOR_TAG_TO_TYPE_START ---  (TagToType TMap entries)
```

## Editing rules

`codegen_rules.json` holds URLab decisions only. Anything derivable from a snapshot must not be hand-listed. Common sections:

- `type_mappings`: attr name to UE type, for example `"size": "TArray<float>"`.
- `element_rules.<elem>`: per-element exclude / rename / meta config.
- `element_rules.<elem>.xml_enum_attrs`: XML-string to UE-enum mapping.
- `canonicalizations`: how multi-attr clusters collapse into one UPROPERTY.
- `categories.<cat>.subtypes`: the UE classes per category.
- `synthetic_categories`: whole-file banner-mode targets (mjOption, mjVisual, and so on).
- `generated_enums`: banner-mode `.h` files holding multiple UENUMs.
- `intentionally_unmodeled_elements` / `intentionally_unmodeled_mjs_fields`: silence the drift diagnostic for items handled elsewhere.

When MuJoCo adds a field, enum value, or sensor type, the drift diagnostic surfaces it. Run with `--strict` to make those fatal.

## Common mistakes

1. **Edit between `CODEGEN` markers and forget the rule.** The next regen reverts your change. Fix the rule, then regen.
2. **A new `mjsX` field after a bump.** The drift diagnostic surfaces it. Add a `canonicalization`, `xml_enum_attrs`, or `attr_to_mjs_field` entry, or list it in `intentionally_unmodeled_mjs_fields` with a one-line reason.
3. **Hand-write a `bOverride_X` pair for an `xml_enum_attr`.** It conflicts with the codegen-emitted declaration when `emit_property_decl: true`. Pick one source of truth.
4. **Forget to regen before a PR.** The `--check` gate fails the build. Re-run the generator and commit the output.

## Environment

The introspect step imports `clang.cindex` and loads libclang. Run regen and the codegen tests in an environment that has libclang. If libclang is missing, the introspect step fails and `regen_all.py` falls back to the committed snapshot, so a header change in a bump is silently skipped. `--require-introspect` (used by the build gate) turns that silent fallback into a hard error. Override the library path with `$LIBCLANG_LIBRARY_FILE` or `--libclang` if auto-detection fails.

## Related

- [Bumping MuJoCo](bumping_mujoco.md): the full version-bump procedure, with the diagnostic-to-rule mapping.
- [Building from Source](building.md): the third-party drift checks and the build gate.
- [MJCF Support](../concepts/mjcf_support.md): the coverage this pipeline produces.
