# Regenerating MuJoCoSpec code

URLab's `MuJoCoSpec` enums, structs, and `mjSpec` marshalers are generated from the pinned MuJoCo 4 submodule (`third_party/MuJoCo/src`) using `Tools/SpecCodegen/generate_ue_spec.py` and `Tools/SpecCodegen/manifest.py`. The generated files are checked in — do not edit them by hand.

| Path | Role |
|---|---|
| `Tools/SpecCodegen/generate_ue_spec.py` | Reads MuJoCo 4 `introspect` + `mjcf.schema` and emits UE headers/sources |
| `Tools/SpecCodegen/manifest.py` | Configures element/struct/enum mappings for codegen |
| `Source/URLab/Public/MuJoCoSpec/Generated/MJSpecEnums.h` | Generated `UENUM` declarations |
| `Source/URLab/Public/MuJoCoSpec/Generated/MJSpecStructs.h` | Generated `USTRUCT` declarations (`FMJ*Spec`) |
| `Source/URLab/Public/MuJoCoSpec/Generated/MJSpecMarshal.h` | Generated marshaling declarations |
| `Source/URLab/Private/MuJoCoSpec/Generated/MJSpecMarshal.cpp` | Generated `FMJ*Spec` <-> `mjs*` marshaling implementations |

From the plugin root, run:

```bash
python3 Tools/SpecCodegen/generate_ue_spec.py
```

To verify there is no drift between the pinned MuJoCo submodule and the checked-in generated files (this gate also runs automatically in `Scripts/build_and_test_linux.sh`):

```bash
python3 Tools/SpecCodegen/generate_ue_spec.py --check
```
