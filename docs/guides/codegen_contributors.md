# Codegen contributor guide

How to change URLab's MuJoCo wrappers without breaking the codegen
pipeline. Read this first when you're about to edit anything under
`Source/URLab/Public/MuJoCo/Components/` or `Source/URLab/Public/MuJoCo/Generated/`.

## The three snapshots

| Snapshot                                | Built by                                         | What it captures                                                  |
|-----------------------------------------|--------------------------------------------------|-------------------------------------------------------------------|
| `Scripts/mjcf_schema_snapshot.json`     | `Scripts/codegen/build_mjcf_schema_snapshot.py`  | MJCF schema (`MJCF[]` table) + per-sensor objtype/reftype scrape  |
| `Scripts/mjspec_snapshot.json`          | `Scripts/codegen/build_mjspec_snapshot.py`       | `mjsX` struct fields, `mjs_setTo*` signatures, `mjt*` enum values |
| `Scripts/mjxmacro_snapshot.json`        | `Scripts/codegen/build_mjxmacro_snapshot.py`     | `MJOPTION_FIELDS`, `MJSTATISTIC_FIELDS`, `MJVISUAL_*_FIELDS`, mjModel/mjData pointer tables |
| `Scripts/introspect_snapshot.json`      | `Scripts/codegen/build_introspect_snapshot.py`   | clang-AST scrape of mjspec.h / mjmodel.h / mujoco.h: function sigs, enum decls with doc comments, struct decls, `#define` constants |

Run `python Scripts/codegen/regen_all.py` to refresh all of them after
a MuJoCo bump.

## The generator

`Scripts/codegen/generate_ue_components.py` reads:

1. The four snapshots above
2. `Scripts/codegen/codegen_rules.json` (URLab-side decisions: which fields to expose, how to type-map them, which categories use which layout)

…and emits:

- `.h` + `.cpp` for every category subclass (single-uclass / multi-uclass / no-subclasses layouts)
- Marker-region content for each CODEGEN_*_START/END pair in hand-written files
- Full-file banner-mode artifacts under `Source/URLab/Public/MuJoCo/Generated/`

CLI flags:

| Flag        | Effect                                                                     |
|-------------|----------------------------------------------------------------------------|
| `--check`   | Non-zero exit if regen would change anything. Used by the build script gate. |
| `--dry-run` | Print diffs, write nothing                                                 |
| `--strict`  | Exit code 2 if any drift diagnostic fires                                  |

## The build gate

`Scripts/build_and_test.ps1` runs `--check` before invoking UBT. If
codegen output drifted from the committed source, the build fails with
exit code 4 before wasting compile time. Re-run the generator without
`--check`, commit the output, then re-run the script.

Bypass for known-incompatible local edits:

```
$env:URLAB_SKIP_CODEGEN_DRIFT = '1'  # NOT YET WIRED — manually skip by editing the script
```

## Editing rules

`codegen_rules.json` is **URLab decisions only** — anything derivable
from a snapshot should NOT be hand-listed. Common rule sections:

- `type_mappings`: attr name → UE type (`"size": "TArray<float>"`)
- `element_rules.<elem>`:  per-element exclude/rename/meta config
- `element_rules.<elem>.xml_enum_attrs`: XML-string → UE-enum mapping
- `canonicalizations`: how multi-attr clusters collapse into UPROPERTYs
- `categories.<cat>.subtypes`: list of UE classes per category
- `synthetic_categories`: whole-file banner-mode targets (mjOption, mjVisual, etc.)
- `generated_enums`: banner-mode .h files holding multiple UENUMs
- `intentionally_unmodeled_elements` / `intentionally_unmodeled_mjs_fields`: silence the drift diagnostic for known-handled-elsewhere items

When MuJoCo adds a new field / enum value / sensor type, the drift
diagnostic should surface it. Run `--strict` to make those fatal in CI.

## Marker pairs in hand-written files

Most `Source/URLab/Private/MuJoCo/Components/*.cpp` files have these
two pairs:

```
// --- CODEGEN_IMPORT_START ---
// (codegen-emitted XML → UPROPERTY assignments)
// --- CODEGEN_IMPORT_END ---

// --- CODEGEN_EXPORT_START ---
// (codegen-emitted UPROPERTY → mjs spec field writes)
// --- CODEGEN_EXPORT_END ---
```

Inside the markers is codegen-owned. Outside is hand-written. Do not
move logic across the boundary in a single commit — split into a
"prepare hand-side" commit and a "regen" commit so the diff is
reviewable.

Special markers for `MjSensor.cpp`:

```
// --- CODEGEN_SENSOR_TYPE_SWITCH_START ---  (case bodies in ExportTo)
// --- CODEGEN_SENSOR_TAG_TO_TYPE_START ---  (TagToType TMap entries)
```

## Plan v8 references

- `docs/plan_codegen_introspect.md` — full plan (v9 status at top)
- `docs/plan_codegen_deviations.md` — D1–D10 deviation log
- `Source/URLab/Public/MuJoCo/Generated/README.md` — per-file index of generated artifacts

## Common mistakes

1. **Edit between CODEGEN markers and forget to update the rule** → next regen reverts your change. Fix: edit the rule + run regen.
2. **Add a field to `mjsX` upstream after a MuJoCo bump** → drift diagnostic surfaces it. Either add to a `canonicalization`, `xml_enum_attrs`, `attr_to_mjs_field`, etc., or add to `intentionally_unmodeled_mjs_fields` with a one-line reason.
3. **Hand-write a `bOverride_X` pair for an xml_enum_attr** → conflicts with the codegen-emitted decl when `emit_property_decl: true` is set. Pick one source of truth.
4. **Forget to run regen before opening a PR** → build gate fails with exit 4. CI will catch it.
