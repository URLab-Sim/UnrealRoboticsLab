# ROS 2 Integration

URLab publishes robot state and scene geometry as standard ROS 2 messages so
downstream tools — rviz, MoveIt, nav2, your own nodes — consume them without any
shim. It also accepts `cmd_vel` (`geometry_msgs/Twist`) and can route external ROS
control through MuJoCo actuators.

ROS 2 is **optional and modular**. The core plugin has no ROS dependency; nothing
breaks when ROS is absent. The ROS pieces live in a separate `URLabRos` module that
compiles to a no-op when ROS 2 is not installed.

The integration works on **Ubuntu 22.04 / 24.04** (apt or Pixi/Conda) and
**Windows 11** (Pixi/Conda via RoboStack). This guide covers Ubuntu; for Windows
setup see [ROS Workspace Setup](../ros_workspace_setup.md).

---

## 1. Install ROS 2

**Pinned distribution: ROS 2 Lyrical Luth** (LTS, supported to 2031).

=== "Ubuntu 24.04 (apt)"

    ```bash
    sudo apt update
    sudo apt install -y ros-lyrical-ros-base ros-dev-tools
    ```

    Installs into `/opt/ros/lyrical`. `ros-lyrical-ros-base` is enough; the
    `ros-dev-tools` package gives you `ros2` CLI for verification.

=== "Ubuntu 22.04 / 24.04 (Pixi)"

    Pixi is useful when you want an isolated ROS install that doesn't touch the
    system, or when you're on 22.04 (apt doesn't ship Lyrical for 22.04).

    ```bash
    curl -fsSL https://pixi.sh/install.sh | sh
    # restart your shell, then:
    pixi init urlab_ros2_env -c https://prefix.dev/robostack-lyrical -c conda-forge
    cd urlab_ros2_env
    pixi add ros-lyrical-desktop
    ```

    Activate with `pixi shell` from the `urlab_ros2_env` directory whenever you
    build or launch the editor.

---

## 2. Build URLab with ROS enabled

### 2.1 Prerequisites

Build the plugin's native dependencies first (MuJoCo, CoACD, libzmq). See
[Installation](../installation.md) for the full flow. From the plugin root:

```bash
cd third_party
./build_all.sh --engine "$UE_ROOT"
```

### 2.2 Set the ROS root

The `URLabRos` module's build logic probes the `URLAB_ROS2_ROOT` environment
variable. Point it at your ROS install prefix (the directory that holds `include/`,
`lib/`, and `bin/` subdirectories):

=== "apt"

    ```bash
    export URLAB_ROS2_ROOT=/opt/ros/lyrical
    ```

=== "Pixi"

    ```bash
    export URLAB_ROS2_ROOT=$CONDA_PREFIX
    ```

If `URLAB_ROS2_ROOT` is unset or points at a missing directory, the module builds
without ROS (`URLAB_WITH_ROS2=0`) and prints a notice:

```
URLabRos: ROS 2 not found (set URLAB_ROS2_ROOT to enable) - building without ROS.
```

### 2.3 Build

```bash
./Scripts/build_and_test_linux.sh \
    --engine "$UE_ROOT" \
    --project /path/to/YourProject.uproject
```

UBT links the minimal C-ABI set (`librcl.so`, `librosidl_runtime_c.so`, the
message-package `.so` files) and stages the transitive DDS cluster next to the
plugin binary.

### 2.4 Runtime library path

UE does not auto-stage `RuntimeDependencies` for editor builds on Linux, so the
ROS `.so` files must be reachable by the dynamic linker. Two options:

**Option A — `LD_LIBRARY_PATH` (quickest for development):**

```bash
export LD_LIBRARY_PATH="$URLAB_ROS2_ROOT/lib:$LD_LIBRARY_PATH"
./UnrealEditor YourProject.uproject
```

**Option B — symlink into the plugin's `Binaries/Linux/`:**

```bash
./Scripts/setup_runtime_linux.sh
```

This symlinks the ROS `.so` cluster under `Binaries/Linux/` so UBT's `$ORIGIN`
RPATH resolves them without any env var. Idempotent; re-run after a build.

---

## 3. Verify it works

### 3.1 Check module load

Launch the editor and look for the ROS context log line in the output:

```
LogURLabRos: ROS 2 context up (distro lyrical, node 'urlab').
```

If you see this, the module loaded and connected to DDS. A warning instead means
the DLLs were found but `rcl_init` failed (usually a DDS configuration issue).

If you see no `LogURLabRos` lines at all, the module didn't load — check that the
ROS `.so` files are on `LD_LIBRARY_PATH` and that `URLAB_ROS2_ROOT` was set during
the build (re-run `Build.sh` after setting it).

### 3.2 Start PIE and check topics

With the editor open, enter Play-In-Editor (PIE) with a level that contains an
`MjManager` and at least one robot articulation. In another terminal (with ROS 2
sourced), run:

```bash
ros2 topic list
```

You should see the standard ROS topics:

| Topic | Message type | Notes |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | Sim time, published at sim rate |
| `/tf` | `tf2_msgs/TFMessage` | Per-body transforms, 50 Hz |
| `/tf_static` | `tf2_msgs/TFMessage` | Static transforms, latched |
| `/<actor>/joint_states` | `sensor_msgs/JointState` | Per-articulation, 50 Hz |
| `/<actor>/pose` | `geometry_msgs/PoseStamped` | Articulation root pose |
| `/<actor>/odometry` | `nav_msgs/Odometry` | Root odometry (if free joint), 50 Hz |
| `/<actor>/imu` | `sensor_msgs/Imu` | Per-IMU-sensor, 100 Hz |
| `/<actor>/sensors` | `sensor_msgs/JointState` | Sensor readouts |
| `/<actor>/robot_description` | `std_msgs/String` | URDF, latched |
| `/<actor>/planning_scene` | `moveit_msgs/PlanningScene` | Includes world geometry, 10 Hz |
| `/<cam>/image` | `sensor_msgs/Image` | One per streaming camera |
| `/<cam>/camera_info` | `sensor_msgs/CameraInfo` | One per streaming camera |
| `/urlab/obstacle_cloud` | `sensor_msgs/PointCloud2` | Sampled world geometry, 10 Hz |
| `/map` | `nav_msgs/OccupancyGrid` | 2D occupancy raster, latched |
| `/octomap_binary` | `octomap_msgs/Octomap` | Volumetric occupancy, 5 Hz |
| `/urlab/cmd_vel` | `geometry_msgs/Twist` | Twist control input |

### 3.3 Visualise in rviz

```bash
ros2 run rviz2 rviz2
```

Add a `RobotModel` display, set the description topic to
`/<actor>/robot_description`, and add a `TF` display. The robot appears in rviz
with live joint state, driven by MuJoCo physics running inside Unreal.

---

## 4. MoveIt planning

To use URLab with MoveIt:

1. Launch `urlab_moveit` from the `ros/urlab_moveit/` package:
   ```bash
   ros2 launch urlab_moveit franka.launch.py
   ```
   This brings up `move_group` with the URDF from `/robot_description`, the SRDF
   auto-generated by `generate_srdf.py`, and the planning scene fed from
   `/planning_scene`.

2. Use the MoveIt RViz plugin or the Python MoveIt API (`moveit_commander`) to
   plan and execute trajectories. The bridge publishes joint trajectories to
   URLab's control path.

---

## 5. How it fits together

```
┌─────────────────────────────────────────────────────┐
│  Unreal Editor (PIE)                                 │
│  ┌──────────┐  ┌──────────────┐  ┌───────────────┐  │
│  │  MuJoCo   │  │ FMjState-    │  │ IMjRosOutput- │  │
│  │  thread   ├─►│ Collector    ├─►│ Provider      │  │
│  │ (physics) │  │ (typed IR)   │  │ (per topic)   │  │
│  └──────────┘  └──────────────┘  └───────┬───────┘  │
│                                          │ rcl C ABI │
│  ┌───────────────────────────────────────┴──────────┐│
│  │  UrlabRclCore (extern "C", no rclcpp)            ││
│  │  Fills rosidl C structs → rcl_publish            ││
│  └──────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────┘
                          │ DDS (FastDDS)
                          ▼
   ┌──────────────────────────────────────────┐
   │  ROS 2 ecosystem (rviz, MoveIt, nav2, …) │
   └──────────────────────────────────────────┘
```

State flows from MuJoCo through a transport-neutral typed IR
(`FMjStateCollector`). Each ROS provider reads the IR and publishes one topic. The
whole ROS side lives in `Source/URLabRos/` and links only the rcl C API — no
`rclcpp`, no `ament`, no colcon.

The bridge (ZMQ + shared memory) is always available; ROS is an additional,
independent transport that publishes the same state in ROS-native formats.

---

## 6. Architecture notes

**No rclcpp.** The plugin links the rcl C API directly through a thin
`extern "C"` seam (`Source/URLabRos/Private/Ros/UrlabRclCore.h`). This avoids
the rclcpp dependency (and its `libstdc++` ABI mismatch with UE's bundled
`libc++`) and keeps link times small.

**Self-registering providers.** Every ROS topic is published by a class
that implements `IMjRosOutputProvider` and registers itself with the
`REGISTER_MJ_ROS_OUTPUT_PROVIDER` macro. The `RosPublishTransport` discovers them
at module load and drives their `Build`/`Publish` lifecycle each step. To add a
new topic, write a provider and register it — no plumbing changes needed.

**Graceful degradation.** When `URLAB_WITH_ROS2=0` (no ROS at build time) or when
`FURLabRosContext::Initialize()` fails at runtime (no DDS, bad config), every ROS
code path is fenced and degrades to a no-op. The simulation, bridge, and dashboard
continue normally.

---

## 7. Troubleshooting

**"ROS 2 not found" during build.**
`URLAB_ROS2_ROOT` is unset or points at a missing directory. Export it and
re-build. If the ROS install uses a different layout (e.g. `include/` is nested
under a `ros2/` prefix), point `URLAB_ROS2_ROOT` at the directory that directly
contains `include/`, `lib/`, and `bin/`.

**Editor fails to load with "lib<name>.so: cannot open shared object file".**
The ROS `.so` cluster is not on the linker's search path. Use
`LD_LIBRARY_PATH` or run `Scripts/setup_runtime_linux.sh` to symlink them.

**Module loads but no topics appear.**
The ROS context failed to initialise. Look for this warning in the editor log:
```
LogURLabRos: Warning: ROS 2 unavailable: rcl context init failed (…)
```
This usually means DDS discovery cannot start — check that no firewall is blocking
UDP multicast and that `$ROS_DOMAIN_ID` is consistent across terminals.

**`ros2 topic list` shows topics but rviz sees no TF.**
Wait a few seconds after PIE starts. The TF provider throttles to 50 Hz and needs
at least one sim step before the first message goes out. Also check that
`/tf_static` appears — some tools need it before they render anything.

**Wrong message types or missing fields.**
URLab is pinned to ROS 2 Lyrical. If you sourced a different distro (Humble,
Jazzy, Kilted), message definitions may differ. Run `ros2 topic info <topic>` to
check the type.
