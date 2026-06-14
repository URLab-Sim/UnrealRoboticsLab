# Troubleshooting

Common failures and how to fix them, grouped by symptom. Each entry
gives the cause and the exact fix. For full setup steps see
[Installation](installation.md); for building from source see
[Building from Source](contributing/building.md).

## Build error: submodule drift

**Symptom.** The UE build fails with a message like:

```
MuJoCo submodule drift: URLab expects SHA 72cb2b2... but
third_party/MuJoCo/src/ is at aef0589...
```

**Cause.** You pulled URLab, but the submodule under
`third_party/<dep>/src/` still points at the old commit. This is the
**Layer A** check.

**Fix.** Sync the submodules to the pinned SHAs and rebuild the named
dep:

```powershell
git submodule update --init --recursive --force
cd third_party; .\MuJoCo\build.ps1; cd ..
```

The same applies to CoACD and libzmq; the message names the specific
dep. `--force` is required because the CoACD build overlays a custom
`CMakeLists.txt` onto its working tree, which makes a plain
`git submodule update` refuse to check out a new SHA.

## Build error: install is stale

**Symptom.** The UE build fails with a message like:

```
MuJoCo install is stale: built from SHA 0000... but
third_party/MuJoCo/src/ is now at 72cb2b2...
```

**Cause.** You updated the submodule source but did not rebuild the
dep, so `third_party/install/<dep>/` is out of sync with `src/`. This is
the **Layer B** check.

**Fix.** Rebuild the named dep:

```powershell
cd third_party; .\MuJoCo\build.ps1; cd ..
```

## Build error: MSVC stack overflow (0xC00000FD)

**Symptom.** `build_all.ps1` fails with exit code `-1073741571` while
building MuJoCo.

**Cause.** The compiler ran out of internal stack while processing
MuJoCo's sensor templates.

**Fix.** Update Visual Studio to **VS 2022 (17.10+)** or **VS 2025**
(the MuJoCo CI reference). As a workaround you can force a larger stack
with `-DCMAKE_CXX_FLAGS="/F10000000"` on the CMake configure line.

## Project prompts to rebuild modules on open

**Symptom.** Opening the `.uproject` prompts that modules are out of
date or missing and offers to rebuild ("compile from source").

**Cause.** The project binaries have not been built (or are stale)
relative to the current source.

**Fix.** Build the project first, then reopen:

- Windows: run `Scripts/build_and_test.ps1`.
- Linux: run `Scripts/build_and_test_linux.sh`.
- Or build from an IDE: Visual Studio or JetBrains Rider (Rider
  recommended).

After the build succeeds, reopen the project.

## Dashboard not appearing

**Symptom.** The Simulate dashboard widget does not show in PIE.

**Cause / fix.** The widget is context-sensitive. Check all three:

- An `MjManager` actor is present in the level.
- `bAutoCreateSimulateWidget` is enabled in the `MjManager` settings.
- **Show Plugin Content** is on in the Content Browser settings, so the
  UI assets are visible to the engine (see [Installation](installation.md)).

## Robot is static

**Symptom.** The robot does not move when you send control.

**Cause / fix.** Check, in order:

- **Control source.** If the `MjManager` or `MjArticulation` control
  source is set to **UI**, ZMQ writes are ignored, and vice versa. Set
  it to match where your control comes from (UI sliders vs ZMQ client).
- **Paused.** Ensure the `MjManager` is not paused.
- **Static.** Ensure the robot is not set to `Static` in its component
  settings.

## Older UE versions: assets will not load

**Symptom.** UI widgets, materials, or input mappings are missing on a
UE version older than 5.7.

**Cause.** The bundled `.uasset` files were serialized in UE 5.7 and are
not backwards compatible. The C++ plugin code compiles and the core
simulation runs, but the dashboard UI and some editor features are
unavailable.

**Fix.** Use UE 5.7, the only tested and supported version. If you must
stay on an older version and have 5.7 installed alongside it, you can
recreate each asset by opening it in 5.7, copying its graph nodes
(Ctrl+A, Ctrl+C), and pasting them into a new asset of the same type in
your older version.
