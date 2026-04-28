# Linux Setup

URLab supports Linux against Unreal Engine 5.7+ binary distribution. The plugin builds and the full automation suite (`URLab.*`) passes 177/177 with a working editor — but the build flow is more involved than on Windows because UE's bundled clang + libc++ require the third-party dependencies to be ABI-compatible.

If you hit anything not covered here please open an issue.

## Prerequisites

- **UE 5.7+** Linux binary distribution from <https://www.unrealengine.com/linux>. Extract anywhere; the rest of this guide refers to the extract root as `$UE_ROOT` (e.g. `/home/<user>/UnrealEngine`).
- **CMake 3.24+** — Ubuntu 22.04 ships 3.22, which is below CoACD's minimum. Easiest fix:
  ```bash
  pip install --user "cmake>=3.24,<4"
  export PATH="$HOME/.local/bin:$PATH"
  ```
- **A host UE5 C++ project** with `UnrealRoboticsLab` cloned into its `Plugins/` (or symlinked there).

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
UE_TC=$UE_ROOT/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64/v26_clang-20.1.8-rockylinux8/x86_64-unknown-linux-gnu
```

### 1. Build third-party libs against UE's toolchain

```bash
cd "$URLAB_ROOT/third_party"

CC="$UE_TC/bin/clang" \
CXX="$UE_TC/bin/clang++" \
AR="$UE_TC/bin/llvm-ar" \
RANLIB="$UE_TC/bin/llvm-ranlib" \
CFLAGS="-fPIC -Qunused-arguments -Wno-unknown-warning-option" \
CXXFLAGS="-stdlib=libc++ -nostdinc++ -isystem $UE_TC/include/c++/v1 -fPIC -Qunused-arguments -Wno-unknown-warning-option -Wno-missing-template-arg-list-after-template-kw" \
LDFLAGS="-stdlib=libc++ -fuse-ld=lld -L$UE_TC/lib64 -Wl,-rpath,$UE_TC/lib64" \
bash ./build_all.sh
```

Expected on success: `third_party/install/{MuJoCo,CoACD,libzmq}/` populated with headers + `.so` files.

Notes:
- `-Wno-unknown-warning-option` lets clang ignore `-Werror=stringop-overflow` (a GCC-only flag) that TBB (CoACD's transitive dep) tries to use.
- `-Wno-missing-template-arg-list-after-template-kw` keeps clang 20 from rejecting OpenVDB's `OpT::template eval(...)` syntax.
- `-Qunused-arguments` quiets MuJoCo's `-Werror -Wunused-command-line-argument` noise from `-stdlib=libc++` on compile-only steps.
- `BUILD_STATIC=OFF` is now applied automatically for libzmq on Linux (in `third_party/libzmq/build.sh`) — the static archive's `mailbox_safe.cpp` pulls `pthread_cond_clockwait` which UE's link sysroot can't resolve, and the `.so` works fine.

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
