# Linux Setup

URLab supports Linux against Unreal Engine 5.7+ binary distribution. The plugin builds and the full automation suite (`URLab.*`) passes 177/177 with a working editor — but the build flow is more involved than on Windows because UE's bundled clang + libc++ require the third-party dependencies to be ABI-compatible.

If you hit anything not covered here please open an issue.

## Prerequisites

### Unreal Engine 5

Epic ships a precompiled UE5 binary for Linux — no source build, no GitHub access, no Setup.sh. It's the path of least friction; the source-build path is also supported by Epic but takes much longer (~hours of compile, ~150 GB disk) and is not required.

1. Sign in to your Epic Games account at <https://www.unrealengine.com/linux> (top of the page).
2. Download **Linux Unreal Engine** — currently a single `.zip` named like `Linux_Unreal_Engine_5.7.x.zip`, ~25 GB compressed / ~43 GB extracted.
3. Extract anywhere. The rest of this guide refers to the extract root as `$UE_ROOT` (e.g. `/home/<user>/UnrealEngine`).
4. Confirm the editor binary exists and is executable:

   ```bash
   ls -la $UE_ROOT/Engine/Binaries/Linux/UnrealEditor
   ```

**Disk-space rule of thumb:** 70 GB free during initial setup is comfortable: ~43 GB UE binaries, ~10–20 GB shader / DDC cache after first launch, and a few GB for a host project + the URLab plugin.

**Display:** the editor needs a Wayland or X11 session. On a headless server, common options are NICE DCV, X2Go, or a TigerVNC `:1` display (export `DISPLAY=:1` before launching).

**System libs:** UE bundles most of what it needs (libc++, ICU, etc.). On a fresh Ubuntu 22.04 you may still need `apt install libsdl2-2.0-0 libvulkan1` if the editor can't open a window — that's the symptom you'd see after launch with a missing-library message in the log.

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

UE on Linux uses its own bundled clang (currently 20.1.x at `$UE_ROOT/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/...`) and links against its bundled libc++. If you build the third-party libs with the system gcc + libstdc++ (which is what `third_party/build_all.sh` does by default), the resulting `.so` files have a different C++ ABI than the UE plugin link expects, and you'll get a wall of `std::*` undefined-symbol errors at link time.

Solution: point the third-party CMake builds at UE's clang and libc++ via env vars. Once the libs are built that way, the plugin links and runs against them cleanly.

## Step-by-step

```bash
UE_ROOT=/path/to/UnrealEngine
URLAB_ROOT=$UE_ROOT/../URLabTest/Plugins/UnrealRoboticsLab    # adjust to your layout
```

### 1. Build third-party libs against UE's toolchain

```bash
cd "$URLAB_ROOT/third_party"
./build_all.sh --engine "$UE_ROOT"
```

`--engine` is what makes the build use UE's bundled clang + libc++ rather than the host's gcc + libstdc++. Skip it and the resulting `.so` files have a different C++ ABI than UE's plugin link expects, which surfaces as either a wall of `std::*` undefined-symbol errors at link time, or — worse, with some toolchain combinations — system gcc emits LTO bitcode objects rather than real `.so` files which fail to load with a cryptic "file too short" at editor startup.

The script globs `Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v*_clang-*/` so future UE version bumps (v26 → v27) don't break it. It then exports the CC / CXX / AR / RANLIB pointing at UE's tools and the matching CFLAGS / CXXFLAGS / LDFLAGS internally; no env var sandwich on the user's side.

Expected on success: `third_party/install/{MuJoCo,CoACD,libzmq}/` populated with headers + `.so` files.

Build flags applied internally (for reference, in case you need to debug a per-dep failure):
- `-Wno-unknown-warning-option` lets clang ignore `-Werror=stringop-overflow` (a GCC-only flag) that TBB (CoACD's transitive dep) tries to use.
- `-Wno-missing-template-arg-list-after-template-kw` keeps clang 20 from rejecting OpenVDB's `OpT::template eval(...)` syntax.
- `-Qunused-arguments` quiets MuJoCo's `-Werror -Wunused-command-line-argument` noise from `-stdlib=libc++` on compile-only steps.
- `BUILD_STATIC=OFF` is applied automatically for libzmq on Linux (in `third_party/libzmq/build.sh`) — the static archive's `mailbox_safe.cpp` pulls `pthread_cond_clockwait` which UE's link sysroot can't resolve, and the `.so` works fine.

### 2. Generate UE project files and build the editor target

```bash
cd "$UE_ROOT"
Engine/Build/BatchFiles/Linux/GenerateProjectFiles.sh -project=/path/to/HostProject.uproject -game -engine
Engine/Build/BatchFiles/Linux/Build.sh HostProjectEditor Linux Development -Project=/path/to/HostProject.uproject
```

### 3. Stage runtime libs

UE on Linux doesn't auto-stage `RuntimeDependencies` for editor builds, and UBT's auto-computed RPATH for plugins symlinked outside the host project can resolve incorrectly. URLab ships a helper that symlinks the third-party `.so` files into the plugin's `Binaries/Linux/` so the loader finds them via `${ORIGIN}` (which UBT does add correctly):

```bash
"$URLAB_ROOT/Scripts/setup_runtime_linux.sh"
```

This is idempotent. The `Scripts/build_and_test_linux.sh` runner invokes it automatically after the build.

### 4. Launch the editor

```bash
DISPLAY=:1 "$UE_ROOT/Engine/Binaries/Linux/UnrealEditor" /path/to/HostProject.uproject
```

`URLab` should appear under loaded plugins, the **MuJoCo** asset category should register, and the MJCF importer should respond when you drag an `.xml` into the Content Browser.

## Running the automation suite

```bash
cd "$URLAB_ROOT"
./Scripts/build_and_test_linux.sh \
    --engine "$UE_ROOT" \
    --project /path/to/HostProject.uproject
```

Produces the same summary block as `build_and_test.{sh,ps1}` for pasting into a PR. Expected: `177 / 177 passed`.

Close the editor before running — the test harness needs the project lock free.

## Known caveats

- **Plugin must be located inside (or symlinked inside) the host project's `Plugins/` directory.** UBT's auto-RPATH calculation makes assumptions about the relative position of the plugin and the engine; we work around them by staging libs into `${ORIGIN}`, but the plugin still needs to be findable to UBT through the host project's tree.
- **PIE editor first launch compiles Vulkan SM5 shaders + a derived-data cache** (~10–20 GB). Plan for it; subsequent launches are fast.
- **The `urlab_bridge` Python package** (separate repo) hasn't been smoke-tested against this Linux flow yet. Open an issue if you hit problems.
