# Building from Source

The deep build reference for contributors: third-party dependency drift checks, keeping local submodule edits, and the Linux toolchain internals. If you only want a working first build, follow [Installation](../installation.md) first; this page picks up where that leaves off.

URLab vendors three native dependencies (MuJoCo, CoACD, libzmq) as git submodules under `third_party/<dep>/src/`, each pinned to an exact commit. `third_party/build_all.{ps1,sh}` syncs each submodule to its pinned SHA, wipes and rebuilds `third_party/install/<dep>/`, and records the source SHA in `INSTALLED_SHA.txt`. URLab's `Build.cs` reads those files to detect drift on the next UE build.

## After a `git pull`

A plain `git pull` updates URLab's code and its submodule pointers, but it does not check out new submodule sources or rebuild them. When a pinned SHA moves, run the full sequence:

```powershell
git pull
git submodule update --init --recursive --force  # sync src/ to the new pinned SHA
cd third_party; .\build_all.ps1; cd ..           # rebuild any dep whose SHA moved
```

!!! note "Why `--force`"
    `third_party/CoACD/build.ps1` overlays a custom `CMakeLists.txt` onto the CoACD working tree on every build. A plain `git submodule update` then refuses to check out a new SHA ("Your local changes would be overwritten"). The overlay is reapplied by the next build, so discarding it with `--force` is safe. If you are intentionally editing submodule source, see [Iterating on a dependency](#iterating-on-a-dependency) instead.

## Drift checks

`URLab.Build.cs` cross-checks the submodules on every UE build and fails fast with the exact command to run. There are two layers.

### Layer A: submodule drift

The submodule under `src/` points at a different commit than URLab expects:

```
MuJoCo submodule drift: URLab expects SHA 72cb2b2... but third_party/MuJoCo/src/ is at aef0589...
```

Fix by syncing the submodule, then rebuilding that dep:

```powershell
git submodule update --init --recursive --force
cd third_party; .\MuJoCo\build.ps1; cd ..
```

### Layer B: install is stale

The source was synced but the dep was not rebuilt, so `install/<dep>/` no longer matches `src/`:

```
MuJoCo install is stale: built from SHA 0000... but third_party/MuJoCo/src/ is now at 72cb2b2...
```

Fix by rebuilding:

```powershell
cd third_party; .\MuJoCo\build.ps1; cd ..
```

The message always names the specific dependency. The same two layers apply to CoACD and libzmq.

## Iterating on a dependency

When you are patching a submodule's source, you do not want every build to snap your edits back to the pinned SHA. Pass the opt-out flag so the build skips the submodule sync but still rebuilds the install from your working tree:

```powershell
# Skip all three submodule syncs for this run
.\third_party\build_all.ps1 -NoSubmoduleSync

# Or one dep
.\third_party\MuJoCo\build.ps1 -NoSubmoduleSync
```

On Linux and macOS the equivalent flag is `--no-submodule-sync`.

While iterating, your submodule HEAD no longer matches URLab's pointer and the install SHA no longer matches your tree, so both drift layers will fire on the next UE build. To suppress the checks for long-running local work, flip the constant at the top of `Source/URLab/URLab.Build.cs`:

```csharp
private static readonly bool SkipThirdPartyDriftChecks = true;
```

!!! warning
    Flip it back to `false` before committing. Left `true`, your teammates lose the drift safety net (and the most common new-checkout failure, a missing `build_all` run, stops being caught).

## Build troubleshooting

### MSVC stack overflow (`0xC00000FD`)

If `build_all.ps1` exits with code `-1073741571`, MSVC ran out of internal stack while compiling MuJoCo's sensor templates.

- Fix: update to the latest VS 2022 (17.10+) or VS 2025.
- Workaround: force a larger stack with `cmake -B build ... -DCMAKE_CXX_FLAGS="/F10000000"`.

### Plugin fails to load (`0xc06d007e`)

A missing DLL almost always means `build_all` was never run, so `third_party/install/<dep>/bin/` is empty. Run the after-pull sequence above. The drift checks normally catch this, but they self-disable on non-git checkouts, so a downloaded zip will not warn you.

## Linux build internals

URLab builds on Linux against UE 5.7 (the only supported version — avoid 5.8 due to Vulkan driver regressions), but the flow is more involved than on Windows because UE's bundled clang and libc++ require the native deps to be ABI-compatible. The [Installation](../installation.md) page covers the one-time walkthrough; this section documents the internals you may need when debugging a per-dep failure.

### Why `--engine`

UE on Linux uses its own bundled clang (currently 20.1.x under `$UE_ROOT/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/...`) and links against its bundled libc++. If the deps are built with the system gcc and libstdc++ (which is what `build_all.sh` does without `--engine`), the resulting `.so` files have a different C++ ABI than the plugin link expects, and you get a wall of `std::*` undefined-symbol errors at link time.

```bash
cd "$URLAB_ROOT/third_party"
./build_all.sh --engine "$UE_ROOT"
```

`--engine` globs `Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v*_clang-*/` (version-sorted, so future UE toolchain bumps are picked up automatically) and exports `CC / CXX / AR / RANLIB / CFLAGS / CXXFLAGS / LDFLAGS` for each per-dep `build.sh`.

### Toolchain flags

These are the flags `build_all.sh --engine` applies internally. You need them only when replicating a build by hand or debugging a per-dep failure.

- `-Wno-unknown-warning-option` lets clang ignore `-Werror=stringop-overflow` (a GCC-only flag) that TBB, CoACD's transitive dependency, tries to use.
- `-Wno-missing-template-arg-list-after-template-kw` keeps clang 20 from rejecting OpenVDB's `OpT::template eval(...)` syntax.
- `-Qunused-arguments` quiets MuJoCo's `-Werror -Wunused-command-line-argument` noise from `-stdlib=libc++` on compile-only steps.
- `BUILD_STATIC=OFF` is forced for libzmq on Linux; the static archive's `mailbox_safe.cpp` pulls `pthread_cond_clockwait`, which UE's link sysroot cannot resolve. The `.so` works fine.

### Building one dep manually (the env-var sandwich)

To rebuild a single dep with the same flags `build_all.sh --engine` applies (for example when iterating on MuJoCo), select the toolchain and export the sandwich yourself:

```bash
UE_TC=$(ls -d "$UE_ROOT/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64"/v*_clang-*/x86_64-unknown-linux-gnu | sort -V | tail -1)

CC="$UE_TC/bin/clang" \
CXX="$UE_TC/bin/clang++" \
AR="$UE_TC/bin/llvm-ar" \
RANLIB="$UE_TC/bin/llvm-ranlib" \
CFLAGS="-fPIC -Qunused-arguments -Wno-unknown-warning-option" \
CXXFLAGS="-stdlib=libc++ -nostdinc++ -isystem $UE_TC/include/c++/v1 -fPIC -Qunused-arguments -Wno-unknown-warning-option -Wno-missing-template-arg-list-after-template-kw" \
LDFLAGS="-stdlib=libc++ -fuse-ld=lld -L$UE_TC/lib64 -Wl,-rpath,$UE_TC/lib64" \
bash third_party/MuJoCo/build.sh
```

### Runtime `.so` staging

UE on Linux does not auto-stage `RuntimeDependencies` for editor builds, and UBT's auto-computed RPATH for a plugin symlinked outside the host project can resolve incorrectly. URLab works around this by symlinking the native `.so` files into the plugin's `Binaries/Linux/`, so the loader resolves them through `${ORIGIN}` (which UBT does set correctly).

`Scripts/setup_runtime_linux.sh` does the symlinking. It is idempotent and warn-skips when `Binaries/Linux/` does not exist yet (a fresh checkout before the plugin `.so` is built). You normally do not call it directly: both `build_all.sh` (via each per-dep `build.sh`) and `Scripts/build_and_test_linux.sh` invoke it after their build steps. Run it by hand only if you changed `third_party/install/<pkg>/lib/` outside those scripts and need to re-sync:

```bash
"$URLAB_ROOT/Scripts/setup_runtime_linux.sh"
```

For packaged (non-editor) builds, `RuntimeDependencies.Add(...)` in `URLab.Build.cs` stages the libs through `BuildCookRun` and UBT's `${ORIGIN}` RPATH resolves them, so no manual step is needed.

## Related

- [Installation](../installation.md): basic first-build flow on Windows and Linux.
- [Codegen](codegen.md): regenerating MuJoCo wrappers, and the build-time drift gate.
- [Bumping MuJoCo](bumping_mujoco.md): moving to a new MuJoCo version end to end.
