# Regenerating MuJoCo authoring code

URLab's MuJoCo component classes and field descriptors are generated from the pinned `third_party/MuJoCo/src/src/xml/mjcf.schema`. The generated files under `Source/URLab/*/MuJoCo/Gen/` are checked in. Do not edit them directly.

| Path | Role |
|---|---|
| `protospec/protospec_gen/frontend.py` | parses MuJoCo's schema with MuJoCo's own parser and applies authoring overlays |
| `protospec/protospec_gen/overlay.py`, `overlay_ue.py` | small schema and UE storage exceptions |
| `protospec/protospec_gen/emit_ue.py` | emits UE classes, typed accessors, defaults, keywords, child dispatch and field descriptors |
| `Source/URLab/Private/MuJoCo/Spec/MjUeXml*.cpp` | directly imports and exports the generated UE document |
| `Source/URLab/Private/MuJoCo/Spec/MjSpecParse.cpp` | passes emitted MJCF to MuJoCo's native XML parser and compiler |

From `protospec/`, run:

```bash
uv run python -m protospec_gen.emit_ue --out ..
uv run python -m protospec_gen.emit_ue --out .. --check
uv run pytest -q
```

The wrappers `Scripts/regen_ue_profile.ps1` and `Scripts/regen_ue_profile.sh` provide the same regeneration and `--check` operation from the plugin root. The check compares generated output byte for byte without writing. A MuJoCo version bump may change the generated component tree; review the schema and overlay exceptions before committing the result.

`protospec/build.ps1` or `build.sh` stages only the native-backed orientation helper, XML include expander, tinyxml2, and native model comparison helper needed by URLab. The plain C++ AST, generic profile SDK, generic reader/writer, and direct `mjSpec` write backend have been retired. A new ordinary schema attribute should flow through generated storage and the direct XML gateway without per-element handwritten code; MuJoCo's parser remains responsible for compiler semantics.
