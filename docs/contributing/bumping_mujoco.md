# Bumping MuJoCo

How to move URLab onto a newer MuJoCo release.

The engine is a submodule at `third_party/MuJoCo/src` (currently **MuJoCo 4.0.2**, commit `7252110fb`, `mjVERSION_HEADER 4000002`). Everything URLab derives from it — generated `MuJoCoSpec` structs/enums/marshalers, native compiler behavior, and compiled-model parity — is pinned to that commit.

## Procedure

### 1. Read the upstream changelog

Review MuJoCo's `doc/changelog.rst` between the current pin and the target commit for breaking C API (`mjSpec` / `mjs_*`) changes, `mjcf.schema` additions or removals, and CMake/library packaging changes.

### 2. Move the submodule and stage the gitlink

```bash
cd third_party/MuJoCo/src
git fetch origin && git checkout <target>
cd ../../..
git add third_party/MuJoCo/src
```

Stage the gitlink before building:

- `third_party/MuJoCo/build.ps1` / `build.sh` syncs the submodule against the index.
- `URLab.Build.cs` compares `git ls-tree HEAD` against the checked-in submodule pin and fails the build if the submodule has drifted.

### 3. Rebuild the third-party install

```bash
cd third_party && ./build_all.sh   # or .\build_all.ps1 on Windows
```

Verify the installed header version:

```bash
grep '#define mjVERSION_HEADER' third_party/install/MuJoCo/include/mujoco/mujoco.h
```

### 4. Regenerate `MuJoCoSpec` codegen and check for drift

```bash
python3 Tools/SpecCodegen/generate_ue_spec.py
python3 Tools/SpecCodegen/generate_ue_spec.py --check
```

Review any changes to `Tools/SpecCodegen/manifest.py` and `Source/URLab/{Public,Private}/MuJoCoSpec/Generated/` alongside `Source/URLab/{Public,Private}/MuJoCoSpec/` and `Source/URLabEditor/Private/MuJoCoSpec/`.

### 5. Build and run tests

Run `Scripts/build_and_test_linux.sh` (which runs `generate_ue_spec.py --check`, builds the plugin, and runs the Unreal automation test suite) or `Scripts/build_and_test.ps1` on Windows.
