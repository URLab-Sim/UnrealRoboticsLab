# Linux Setup

URLab supports Linux against Unreal Engine 5.7+. The plugin builds and the full automation suite (`URLab.*`) passes 177/177 with a working editor — but the build flow is more involved than on Windows because UE's bundled clang + libc++ require the third-party dependencies to be ABI-compatible.

This page is a **one-time setup** walkthrough. Jump to [Day-to-day workflow](#day-to-day-workflow) once you have a working build; the [Troubleshooting / Advanced](#troubleshooting--advanced) section covers debugging tips and the build flags applied internally by `build_all.sh --engine`.

If you hit anything not covered here please open an issue.

## Prerequisites

### Unreal Engine 5

You have two options. The binary is dramatically faster to set up; the source build is useful when you want to debug into the engine itself.

#### Option A — precompiled binary (recommended for most users)

Epic ships a precompiled UE5 binary for Linux — no source build, no GitHub access, no `Setup.sh`.

1. Sign in to your Epic Games account at <https://www.unrealengine.com/linux> (top of the page).
2. Download **Linux Unreal Engine** — currently a single `.zip` named like `Linux_Unreal_Engine_5.7.x.zip`, ~25 GB compressed / ~43 GB extracted.
3. Extract anywhere. The rest of this guide refers to the extract root as `$UE_ROOT` (e.g. `/home/<user>/UnrealEngine`).
4. Confirm the editor binary exists and is executable:

   ```bash
   ls -la $UE_ROOT/Engine/Binaries/Linux/UnrealEditor
   ```

#### Option B — source build (useful for engine-level debugging)

Lets you set breakpoints in engine code, step through UBT, and patch UE itself. Costs a multi-hour clone + compile and ~150 GB disk; rarely needed for plugin work but invaluable when you do need it.

1. [Link your Epic Games account to GitHub](https://www.unrealengine.com/en-US/ue-on-github) so you can access <https://github.com/EpicGames/UnrealEngine>.
2. Clone the `5.7` branch (or `release` for whatever Epic considers stable):

   ```bash
   git clone -b 5.7 https://github.com/EpicGames/UnrealEngine.git
   cd UnrealEngine
   ```

3. Run the bundled setup + project-files scripts:

   ```bash
   ./Setup.sh                 # downloads ~30 GB of binary deps via Git LFS
   ./GenerateProjectFiles.sh  # generates Makefile / IDE project files
   make UnrealEditor          # ~hours on first build, parallel-safe
   ```

4. `$UE_ROOT` is your `UnrealEngine/` clone root for the rest of this doc.

#### General notes (apply to both options)

- **Disk-space rule of thumb:** 70 GB free for option A initial setup is comfortable (~43 GB UE binaries + ~10–20 GB shader / DDC cache + a few GB for project + plugin). Option B needs ~150 GB.
- **Display:** the editor needs a Wayland or X11 session. On a headless server, common options are NICE DCV, X2Go, or a TigerVNC `:1` display (`export DISPLAY=:1` before launching).
- **System libs:** UE bundles most of what it needs (libc++, ICU, etc.). On a fresh Ubuntu 22.04 you may still need `apt install libsdl2-2.0-0 libvulkan1` if the editor can't open a window — that's the symptom you'd see after launch with a missing-library message in the log.

### Other prerequisites

- **CMake 3.24+** — Ubuntu 22.04 ships 3.22, which is below CoACD's minimum. Easiest fix:

  ```bash
  pip install --user "cmake>=3.24,<4"
  export PATH="$HOME/.local/bin:$PATH"
  ```

- **A host UE5 C++ project** with `UnrealRoboticsLab` cloned into its `Plugins/` (or symlinked there). The host project must be C++; if your project is Blueprints-only, add a dummy C++ class in the editor first (Tools → New C++ Class).

The plugin's third-party submodules (`MuJoCo`, `CoACD`, `libzmq`) must be initialised:

```bash
cd UnrealRoboticsLab
git submodule update --init --recursive
```

## Why a special build flow

UE on Linux uses its own bundled clang (currently 20.1.x at `$UE_ROOT/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/...`) and links against its bundled libc++. If you build the third-party libs with the system gcc + libstdc++ (which is what `third_party/build_all.sh` does *without* `--engine`), the resulting `.so` files have a different C++ ABI than the UE plugin link expects, and you'll get a wall of `std::*` undefined-symbol errors at link time.

`build_all.sh --engine $UE_ROOT` points the third-party CMake builds at UE's clang + libc++. Once the libs are built that way, the plugin links and runs against them cleanly.

## One-time setup

```bash
UE_ROOT=/path/to/UnrealEngine
URLAB_ROOT=$UE_ROOT/../URLabTest/Plugins/UnrealRoboticsLab    # adjust to your layout
```

### 1. Build third-party libs against UE's toolchain

```bash
cd "$URLAB_ROOT/third_party"
./build_all.sh --engine "$UE_ROOT"
```

Expected on success: `third_party/install/{MuJoCo,CoACD,libzmq}/` populated with headers + `.so` files.

`--engine` makes the script glob `Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v*_clang-*/` (version-sorted, so future UE bumps from v26 → v27 are picked up automatically), then export `CC / CXX / AR / RANLIB / CFLAGS / CXXFLAGS / LDFLAGS` for each per-dep `build.sh`.

### 2. Generate UE project files and build the editor target

```bash
cd "$UE_ROOT"
Engine/Build/BatchFiles/Linux/GenerateProjectFiles.sh -project=/path/to/HostProject.uproject -game -engine
Engine/Build/BatchFiles/Linux/Build.sh HostProjectEditor Linux Development -Project=/path/to/HostProject.uproject
```

The third-party `.so` files (libmujoco, lib_coacd, libzmq) get symlinked into the plugin's `Binaries/Linux/` automatically by each per-dep `build.sh` and by `Scripts/build_and_test_linux.sh` — see [How runtime staging works](#how-runtime-staging-works) in Troubleshooting / Advanced if you need to do it manually.

### 3. Launch the editor

```bash
DISPLAY=:1 "$UE_ROOT/Engine/Binaries/Linux/UnrealEditor" /path/to/HostProject.uproject
```

`URLab` should appear under loaded plugins, the **MuJoCo** asset category should register, and the MJCF importer should respond when you drag an `.xml` into the Content Browser.

## Day-to-day workflow

After the one-time setup, normal iteration is just:

```bash
git pull
./Scripts/build_and_test_linux.sh \
    --engine "$UE_ROOT" \
    --project /path/to/HostProject.uproject
```

That builds the editor target (incremental), re-stages the runtime symlinks if a third-party dep was rebuilt, runs the `URLab.*` automation suite, and emits the build+test summary block to paste into a PR. Expected: `177 / 177 passed`.

Close the editor before running — the test harness needs the project lock free.

If you only changed plugin C++ and want to skip the test pass, the editor `Build.sh` step from one-time setup #2 is the inner loop.

## Troubleshooting / Advanced

### How runtime staging works

UE on Linux doesn't auto-stage `RuntimeDependencies` for editor builds, and UBT's auto-computed RPATH for plugins symlinked outside the host project can resolve incorrectly. URLab works around this by symlinking the third-party `.so` files into the plugin's `Binaries/Linux/` so the loader resolves them via `${ORIGIN}` (which UBT does add correctly).

The helper `Scripts/setup_runtime_linux.sh` does the symlinking. It's idempotent and warn-skips when `Binaries/Linux/` doesn't exist yet (first-time fresh checkout, before the plugin .so has been built). You **don't normally call it directly** — both `build_all.sh` (via each per-dep `build.sh`) and `Scripts/build_and_test_linux.sh` invoke it after their respective build steps.

You'd run it manually only if you fiddled with `third_party/install/<pkg>/lib/` outside of those scripts and need to re-sync the symlinks:

```bash
"$URLAB_ROOT/Scripts/setup_runtime_linux.sh"
```

For packaged (non-editor) builds, `RuntimeDependencies.Add("$(BinaryOutputDir)/...", LibFile, NonUFS)` from `URLab.Build.cs` stages the libs through `BuildCookRun`, and UBT's `${ORIGIN}` RPATH resolves them — no manual step needed.

### Build flags applied internally by `build_all.sh --engine`

Reference, in case you need to debug a per-dep failure or replicate the build manually:

- `-Wno-unknown-warning-option` lets clang ignore `-Werror=stringop-overflow` (a GCC-only flag) that TBB (CoACD's transitive dep) tries to use.
- `-Wno-missing-template-arg-list-after-template-kw` keeps clang 20 from rejecting OpenVDB's `OpT::template eval(...)` syntax.
- `-Qunused-arguments` quiets MuJoCo's `-Werror -Wunused-command-line-argument` noise from `-stdlib=libc++` on compile-only steps.
- `BUILD_STATIC=OFF` is applied automatically for libzmq on Linux (in `third_party/libzmq/build.sh`) — the static archive's `mailbox_safe.cpp` pulls `pthread_cond_clockwait` which UE's link sysroot can't resolve, and the `.so` works fine.

### Building one dep manually (env-var sandwich)

If you want to rebuild only one dep with the same flags `build_all.sh --engine` would apply (e.g. iterating on MuJoCo locally), the equivalent invocation is:

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

### Known caveats

- **Plugin must be located inside (or symlinked inside) the host project's `Plugins/` directory.** UBT's auto-RPATH calculation makes assumptions about the relative position of the plugin and the engine; we work around them by staging libs into `${ORIGIN}`, but the plugin still needs to be findable to UBT through the host project's tree.
- **PIE editor first launch compiles Vulkan SM5 shaders + a derived-data cache** (~10–20 GB). Plan for it; subsequent launches are fast.
- **The `urlab_bridge` Python package** (separate repo) hasn't been smoke-tested against this Linux flow yet. Open an issue if you hit problems.
