# ROS 2 workspace setup and validation (M1)

This document is the complete, self-contained instruction set for installing
ROS 2, building the standalone `urlab_rcl_test` harness, and validating the
in-process rcl publish/subscribe path that the UnrealRoboticsLab ROS integration
is built on. You do not need to read any design document to follow it.

**Pinned distribution: ROS 2 Lyrical Luth** (codename `lyrical`, released
2026-05-22, an LTS supported to 2031). Everything below assumes Lyrical.

**Why this matters most:** in-process ROS 2 on **Windows** has no public
reference implementation. Proving that a Windows process can link the rcl C API
and exchange messages over DDS with a second process is the single biggest
de-risking step for the whole ROS effort. Lyrical is pinned specifically because
the ROS project has made Windows a first-class target (Windows became Tier 1 in
Kilted, 2025, which also moved the default Windows install to **Pixi/Conda** via
prefix.dev + RoboStack; Lyrical continues that with improved Windows 11 support).
The Windows path below is therefore the primary one; do it first. Linux follows
as the secondary / CI path.

The workspace at `ros/urlab_ros_ws/` is a plain CMake project (not colcon). It
consumes the ROS 2 installation you provide; it never builds ROS itself and never
installs anything.

---

## Part 1 — Windows (primary, via Pixi/Conda)

The default and recommended Windows install for current ROS 2 is **Pixi**
(prefix.dev) with the **RoboStack** conda channels. There is no system-wide
install and no `setup.bat` to source: ROS lives inside a Pixi project's `.pixi/`
environment, and you activate it with `pixi shell`.

### 1.1 Prerequisites

1. **Visual Studio 2022** with the **"Desktop development with C++"** workload
   (MSVC toolchain + Windows SDK). RoboStack's ROS 2 binaries are built with
   MSVC, and so is Unreal Engine, so both sides of the eventual link use the same
   compiler and C runtime — there is no libc++/libstdc++ ABI concern on Windows
   (contrast Linux, Part 2.4). You compile `UrlabRclCore.cpp` and `test_main.cpp`
   with this toolchain.
2. **Enable Developer mode** (Settings -> System -> For developers -> Developer
   mode). Pixi/Conda uses symlinks; Developer mode lets them be created without
   admin rights. This is the official RoboStack recommendation for Windows.

### 1.2 Install Pixi

In PowerShell (verified against the official Pixi install docs):

```powershell
powershell -ExecutionPolicy Bypass -c "irm -useb https://pixi.sh/install.ps1 | iex"
```

This downloads Pixi and adds it to your `PATH`. Open a new terminal afterwards so
`pixi` is available, and confirm with `pixi --version`.

### 1.3 Create the ROS 2 Lyrical environment

IMPORTANT (Windows): create the Pixi env at a SHORT, SPACE-FREE path such as
`C:\dev\urlab_ros2_env`. Do NOT put it inside the UE project tree — that path
contains a space (`Unreal Projects`) and is deeply nested, and ROS 2's Windows
launchers (`ros2` and other Python console scripts) fail with
`failed to create process.` when their interpreter path contains a space or
exceeds the Windows path limit. The ROS env is an install, not source; it does
not belong in the repo. (The harness project in `ros/urlab_ros_ws/` stays in the
repo; you activate this env from its `C:\dev` location, or via `URLAB_ROS2_SETUP`,
when building the harness.)

Create a Pixi project that pulls ROS 2 Lyrical from the RoboStack channel. Use
the FULL prefix.dev channel URL, not the bare `robostack-lyrical` name: the
newer RoboStack channels (kilted, lyrical) are hosted only on prefix.dev, and a
bare `robostack-lyrical` resolves against `conda.anaconda.org` (where lyrical is
not published) and fails with a 404 on `noarch/repodata.json`. Older distros
(humble, jazzy) happen to work with the bare name because they are mirrored on
anaconda.org; lyrical is not.

```powershell
mkdir C:\dev -Force
cd C:\dev
pixi init urlab_ros2_env -c https://prefix.dev/robostack-lyrical -c conda-forge
cd urlab_ros2_env
pixi add ros-lyrical-desktop
pixi shell
ros2 --help          # sanity check: must succeed, not "failed to create process."
```

If you already ran `pixi init` with the bare `robostack-lyrical` name, edit the
`channels` line in the generated `pixi.toml` to
`["https://prefix.dev/robostack-lyrical", "conda-forge"]` and re-run
`pixi add ros-lyrical-desktop`.

`ros-lyrical-desktop` includes the rcl C API, the message packages this workspace
needs (`sensor_msgs`, `geometry_msgs`, `tf2_msgs`, `std_msgs`, `rosgraph_msgs`),
the CMake config packages, and the `ros2` CLI used for verification. If you want a
smaller footprint, `ros-lyrical-ros-base` is a lighter alternative that still
provides rcl and the `ros2` CLI.

This installs ROS 2 into `urlab_ros2_env\.pixi\`. Nothing is installed
system-wide.

### 1.4 Activate the environment

From the `urlab_ros2_env` directory:

```powershell
pixi shell
```

`pixi shell` activates the environment for the current shell: it puts the ROS
libraries, headers, `cmake`, and the `ros2` CLI on `PATH` and sets the ROS
environment variables (`AMENT_PREFIX_PATH`, `CMAKE_PREFIX_PATH`, `CONDA_PREFIX`,
...). Confirm ROS is live:

```powershell
ros2 --help
```

Run this `pixi shell` from a **"Developer PowerShell for VS 2022"** (or otherwise
ensure the MSVC `cl` compiler is on `PATH`) so CMake finds the C++ compiler when
it configures the workspace. You can alternatively run one-off commands without a
persistent shell via `pixi run <command>`.

### 1.5 Build and test

With the environment active (from 1.4), build and run the harness:

```powershell
cd <plugin>\ros\urlab_ros_ws
.\build_and_test.ps1
```

The script detects the active ROS environment (via `AMENT_PREFIX_PATH`),
configures and builds the harness (Release), runs the selftest, then runs a
cross-process `ros2 topic echo` check. It writes nothing outside
`ros\urlab_ros_ws\build\`.

If you prefer not to use `pixi shell`, point the script at an activation script
instead:

```powershell
.\build_and_test.ps1 -RosSetup 'C:\path\to\ros_or_conda_activate.bat'
# or: $env:URLAB_ROS2_SETUP = 'C:\path\to\activate.bat'
```

### 1.6 Verify expected output

On success you will see, in order:

- `distro: lyrical` printed at startup.
- `selftest: OK (ctrl + twist loopback verified)` — the harness published a
  `Float64MultiArray` and a `Twist` to the core's own subscriptions and the
  callbacks fired with the exact values.
- `reinit: OK` and `urlab_rcl_test: PASS`.
- The cross-process block, where a second process reads one JointState sample:

  ```
  >>> Cross-process verify (ros2 topic echo)...
  header:
    stamp: ...
    frame_id: ''
  name:
  - joint_a
  - joint_b
  - joint_c
  position:
  - 1.0
  - 2.0
  - 3.0
  velocity:
  - 0.1
  - 0.2
  - 0.3
  effort:
  - 10.0
  - 20.0
  - 30.0
  ...
  >>> Cross-process verify OK: JointState names received.
  === urlab_rcl_test: ALL CHECKS PASSED (Windows) ===
  ```

The joint names `joint_a/joint_b/joint_c` and positions `1.0/2.0/3.0` are the
deterministic values the harness publishes; seeing them proves real inter-process
DDS works in-process on Windows. **This is the key gate — if it passes, the
in-process design is validated on the hard platform.**

You can also run the echo manually while `urlab_rcl_test --publish 600` runs in
another activated shell:

```powershell
ros2 topic echo --once /urlab_test/joint_states
ros2 topic echo --once /urlab_test/imu
ros2 topic echo --once /clock
ros2 topic list
```

---

## Part 2 — Linux (secondary / CI)

Linux can use apt (system install) or the same Pixi/Conda flow as Windows. The
apt path is the default the script expects.

### 2.1 Install ROS 2 Lyrical (apt)

```bash
sudo apt update && sudo apt install -y ros-lyrical-ros-base ros-dev-tools
```

This installs Lyrical into `/opt/ros/lyrical`.

(Alternatively, use Pixi exactly as in Part 1.2-1.4 but with `curl -fsSL
https://pixi.sh/install.sh | sh` to install Pixi.)

### 2.2 Source / activate the environment

`build_and_test.sh` picks up ROS in this order: `URLAB_ROS2_SETUP` if set, else
the apt default `/opt/ros/lyrical/setup.bash`, else an already-active environment
(e.g. inside `pixi shell`). To override the apt default:

```bash
export URLAB_ROS2_SETUP=/opt/ros/lyrical/setup.bash
```

### 2.3 Build and test

```bash
cd <plugin>/ros/urlab_ros_ws
./build_and_test.sh
```

Success ends with `=== urlab_rcl_test: ALL CHECKS PASSED (Linux) ===` and the
same selftest / `ros2 topic echo` output shape as Windows (Part 1.6).

### 2.4 clang + libc++ toolchain smoke (Linux only, deferred)

```bash
./build_and_test.sh --libcxx
```

This rebuilds `UrlabRclCore.cpp` with **clang + libc++** and links it against the
libstdc++-built rcl cluster, pre-validating the exact standard-library mix that
an Unreal Engine Linux build produces (UE uses clang + bundled libc++, while
stock ROS is gcc + libstdc++). It is a **Linux-only** concern and is **deferred**
until the Linux leg is worked on: it does **not** apply on Windows, where MSVC is
used on both sides and there is no stdlib ABI mix to validate. Skip it entirely
for the initial Windows run.

---

## Part 3 — Record the link facts

The Unreal build wiring (a later phase) needs the exact library names, include
directories, and runtime dependency cluster that your build actually used. After
a green run, capture them into `docs/ros2_link_facts.md` (a template with labeled
blanks lives there already).

On a Pixi/Conda install the ROS files live under the environment prefix, not a
system path. With the env active, find the prefix:

- Windows (PowerShell): `echo $env:CONDA_PREFIX` — libraries are under
  `%CONDA_PREFIX%\Library\bin` (DLLs), `%CONDA_PREFIX%\Library\lib` (`.lib`),
  headers under `%CONDA_PREFIX%\Library\include`.
- Linux (bash): `echo $CONDA_PREFIX` — `.so` under `$CONDA_PREFIX/lib`, headers
  under `$CONDA_PREFIX/include`. (apt install: `/opt/ros/lyrical/{lib,include}`.)

### Windows

- **Link library basenames** (the `.lib` files CMake linked): inspect the CMake
  link line:

  ```powershell
  cmake --build build --config Release --verbose 2>&1 | Select-String '\.lib'
  ```

  Record the `rcl`, `rcutils`, `rmw`, `rmw_implementation`/`rmw_fastrtps_cpp`,
  `rosidl_runtime_c`, `rosidl_typesupport_c`, and the per-message-package
  `*__rosidl_typesupport_c` / `*__rosidl_generator_c` basenames as found.

- **Runtime DLL cluster** (must be staged next to a UE build): list the DLLs the
  executable actually depends on:

  ```powershell
  dumpbin /dependents build\Release\urlab_rcl_test.exe
  ```

  Record every ROS/DDS DLL it names (`rcl.dll`, `rmw*.dll`,
  `rmw_fastrtps_cpp.dll`, `fastrtps.dll`/`fastdds.dll`, `fastcdr.dll`,
  `rosidl_*`, the message-package DLLs, and their transitive deps). Repeat
  `dumpbin /dependents` on those DLLs to catch the transitive set. Note their
  directory (typically `%CONDA_PREFIX%\Library\bin`).

- **Include directories**: from the CMake configure output or by inspecting
  `%CONDA_PREFIX%\Library\include` — note whether headers are flat or nested per
  package (`include/<pkg>/<pkg>/msg/...`).

### Linux

```bash
ldd build/urlab_rcl_test
```

Record the `librcl.so`, `librmw*.so`, `librosidl_runtime_c.so`,
`libfastrtps.so`/`libfastdds.so`, `libfastcdr.so`, and message-package `.so`
basenames (the unversioned symlink names) and their directory, plus the include
layout under `$CONDA_PREFIX/include` (Pixi) or `/opt/ros/lyrical/include` (apt).

Paste all of this into `docs/ros2_link_facts.md`. Those user-observed facts — not
any agent guess — are what the Unreal build wiring consumes.

---

## Validation gates (what "M1 passed" means)

- **A2 (Windows pub/sub):** `build_and_test.ps1` fully green on Windows. This is
  the de-risk gate for the whole in-process design; do it first.
- **A1 (Linux pub/sub):** `build_and_test.sh` fully green on Linux.
- **A3 (toolchain smoke):** `build_and_test.sh --libcxx` green on Linux
  (deferred; Linux-only).
- **Facts recorded:** `docs/ros2_link_facts.md` filled in, including the
  rcl/rosidl API assumptions confirmed against the actual Lyrical install.

Once these pass and the facts are recorded, the `UrlabRclCore.h` contract is
frozen (additive changes only) and the Unreal-side wiring can begin.

---

## Sources

Install commands above were verified on 2026-07-26 against:

- [ROS 2 Lyrical Luth release notes](https://docs.ros.org/en/lyrical/Releases/Release-Lyrical-Luth.html)
- [ROS 2 Kilted Kaiju release blog (Windows Tier 1 + Pixi/Conda default)](https://www.openrobotics.org/blog/2025/5/23/ros-2-kilted-kaiju-released)
- [RoboStack getting started](https://robostack.github.io/GettingStarted.html)
- [Pixi ROS 2 tutorial](https://pixi.prefix.dev/latest/tutorials/ros2/)
- [Pixi installation](https://pixi.prefix.dev/latest/installation/)
