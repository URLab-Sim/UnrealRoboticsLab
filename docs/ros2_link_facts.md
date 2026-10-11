# ROS 2 link facts (recorded during M1)

Recorded from a green `urlab_rcl_test` build on Windows against the pinned
Lyrical install (see `docs/ros_workspace_setup.md`). These are user-observed
facts from the actual install, not guesses; the Unreal build wiring (`AddRos2`
in `Source/URLab/URLab.Build.cs`) consumes this file and it is authoritative
over any provisional list elsewhere.

The install is Pixi/Conda (prefix.dev + RoboStack), so the files live under the
environment prefix, not a system path:

```
C:\dev\urlab_ros2_env\.pixi\envs\default\Library
```

Point `URLAB_ROS2_ROOT` at that `Library` directory (it holds `include\`,
`lib\`, `bin\`).

---

## Distro / version

| Field | Value |
|---|---|
| Distro | Lyrical Luth (codename `lyrical`) |
| Package versions (from cmake configure) | rcl 10.4.4, rmw_fastrtps_cpp 9.4.8, std_msgs/geometry_msgs/sensor_msgs 5.9.2, tf2_msgs 0.45.7, rosgraph_msgs 2.4.5, rosidl_generator_c 5.2.1 |
| Install method | Pixi / RoboStack (`https://prefix.dev/robostack-lyrical`) |
| RMW implementation in use | `rmw_fastrtps_cpp` (default; selected automatically at configure) |

---

## Windows (primary)

### Link libraries — the minimal set UBT must link

UBT compiles only the C ABI (`UrlabRclCore.cpp`), so the link-time set is far
smaller than the full transitive list CMake pulls in. On Windows an import `.lib`
only requires the symbols our object files actually reference; every downstream
DLL dependency (rmw_implementation, the FastDDS cluster, the fastrtps
typesupports) is resolved by the loader at runtime, not at link. The minimal
correct set, all confirmed present under `Library\lib`:

| Purpose | As-found `.lib` |
|---|---|
| rcl | `rcl.lib` |
| rcutils | `rcutils.lib` |
| rmw | `rmw.lib` |
| rosidl runtime | `rosidl_runtime_c.lib` |
| builtin_interfaces (gen + ts) | `builtin_interfaces__rosidl_generator_c.lib`, `builtin_interfaces__rosidl_typesupport_c.lib` |
| std_msgs (gen + ts) | `std_msgs__rosidl_generator_c.lib`, `std_msgs__rosidl_typesupport_c.lib` |
| geometry_msgs (gen + ts) | `geometry_msgs__rosidl_generator_c.lib`, `geometry_msgs__rosidl_typesupport_c.lib` |
| sensor_msgs (gen + ts) | `sensor_msgs__rosidl_generator_c.lib`, `sensor_msgs__rosidl_typesupport_c.lib` |
| tf2_msgs (gen + ts) | `tf2_msgs__rosidl_generator_c.lib`, `tf2_msgs__rosidl_typesupport_c.lib` |
| rosgraph_msgs (gen + ts) | `rosgraph_msgs__rosidl_generator_c.lib`, `rosgraph_msgs__rosidl_typesupport_c.lib` |

`AddRos2` pins exactly this list.

For reference, the FULL set the standalone CMake link line pulled in transitively
(via ament imported targets, not needed for the UBT link) additionally included:
`rmw_implementation`, `rosidl_typesupport_c`, `rosidl_dynamic_typesupport`,
`rosidl_buffer`, `rcl_yaml_param_parser`, `rcl_logging_interface`,
`fastcdr-2.3`, and the `*__rosidl_typesupport_fastrtps_c/cpp`,
`*__rosidl_typesupport_introspection_c/cpp`, `*__rosidl_typesupport_cpp` and
`*__rosidl_generator_py` variants for `rcl_interfaces`, `service_msgs`,
`type_description_interfaces`, `action_msgs`, `unique_identifier_msgs`. These are
runtime DLLs, staged (below), not linked.

### Runtime DLL cluster (must be on PATH / staged)

Direct dependencies of `urlab_rcl_test.exe` (from `dumpbin /dependents`), ROS
only:

```
rcl.dll  rcutils.dll  rmw.dll  rosidl_runtime_c.dll
sensor_msgs__rosidl_typesupport_c.dll   sensor_msgs__rosidl_generator_c.dll
geometry_msgs__rosidl_typesupport_c.dll geometry_msgs__rosidl_generator_c.dll
std_msgs__rosidl_typesupport_c.dll      std_msgs__rosidl_generator_c.dll
tf2_msgs__rosidl_typesupport_c.dll      tf2_msgs__rosidl_generator_c.dll
rosgraph_msgs__rosidl_typesupport_c.dll rosgraph_msgs__rosidl_generator_c.dll
```

Transitive cluster loaded at runtime by `rmw` -> `rmw_implementation` ->
`rmw_fastrtps_cpp` (from `dumpbin /dependents` on `rmw_implementation.dll` and
the FastDDS chain in `Library\bin`):

```
rmw_implementation.dll  rmw_fastrtps_cpp.dll  rmw_fastrtps_shared_cpp.dll
rmw_dds_common.dll  rmw_dds_common__rosidl_*.dll
rcpputils.dll  ament_index_cpp.dll
rosidl_typesupport_c.dll  rosidl_typesupport_cpp.dll
rosidl_typesupport_fastrtps_c.dll  rosidl_typesupport_fastrtps_cpp.dll
rosidl_typesupport_introspection_c.dll  rosidl_typesupport_introspection_cpp.dll
rosidl_dynamic_typesupport.dll  rosidl_dynamic_typesupport_fastrtps.dll
rcl_logging_interface.dll  rcl_logging_spdlog.dll  spdlog.dll
rcl_yaml_param_parser.dll
fastdds-3.6.dll  fastcdr-2.3.dll  foonathan_memory-0.7.4.dll  tinyxml2.dll
libssl-3-x64.dll  libcrypto-3-x64.dll  dds_security_crypto.dll
<pkg>__rosidl_typesupport_fastrtps_c.dll / _cpp.dll and
<pkg>__rosidl_typesupport_introspection_c.dll / _cpp.dll for
builtin_interfaces, std_msgs, geometry_msgs, sensor_msgs, tf2_msgs,
rosgraph_msgs, rcl_interfaces, service_msgs, type_description_interfaces,
action_msgs, unique_identifier_msgs
```

Note the version-suffixed DDS basenames: `fastdds-3.6`, `fastcdr-2.3`,
`foonathan_memory-0.7.4`. `AddRos2` stages these with prefix patterns so the
suffix does not have to be hard-coded.

### Include layout

| Field | Value |
|---|---|
| Include root | `%URLAB_ROS2_ROOT%\include` (= `...\Library\include`) |
| Layout | per-package nested: `include\<pkg>\<pkg>\msg\<snake>.h` (e.g. `include\sensor_msgs\sensor_msgs\msg\joint_state.h`) |
| Consequence | each package needs its own `-I include\<pkg>` entry; `#include <sensor_msgs/msg/joint_state.h>` resolves under `include\sensor_msgs`. Confirmed for `rcl`, `rmw`, `rosidl_runtime_c`, `rosidl_typesupport_interface`, `builtin_interfaces`. |
| Lib dir | `%URLAB_ROS2_ROOT%\lib` (`.lib`) |
| Bin dir (DLLs) | `%URLAB_ROS2_ROOT%\bin` |

---

## Linux (secondary)

Not yet run on this machine (Windows is the primary M1 platform). Fill from
`ldd build/urlab_rcl_test` after a green `build_and_test.sh`. The `AddRos2`
Linux branch mirrors the Windows pin: link the unversioned `lib<name>.so`
symlinks for the same minimal set and stage the `*.so*` cluster under
`$ORIGIN`, exactly as `AddThirdPartyLibrary` does for libzmq. Expected
basenames: `librcl.so`, `librcutils.so`, `librmw.so`, `librosidl_runtime_c.so`,
`lib<pkg>__rosidl_generator_c.so`, `lib<pkg>__rosidl_typesupport_c.so`, plus the
runtime `librmw_fastrtps_cpp.so`, `libfastdds.so`, `libfastcdr.so` cluster.

---

## rcl / rosidl API assumptions — all confirmed on Lyrical

`UrlabRclCore.cpp` compiled and linked clean against the actual Lyrical install
and the selftest + cross-process `ros2 topic echo` passed, which confirms the
API assumptions the core was written against:

| # | Assumption | Status |
|---|---|---|
| 1 | Header paths (`rcl/rcl.h`, `rmw/qos_profiles.h`, `rosidl_runtime_c/*`, `<pkg>/msg/<snake>.h`) | OK (compiled clean; nested include layout above) |
| 2 | `rcl_init_options_set_domain_id(rcl_init_options_t*, size_t)`; `RCL_DEFAULT_DOMAIN_ID` | OK |
| 3 | `ROSIDL_GET_MSG_TYPE_SUPPORT(pkg, msg, Type)` resolves once `<pkg>__rosidl_typesupport_c` is linked | OK |
| 4 | `rcl_wait_set_init` seven-count signature | OK |
| 5 | `wait_set.subscriptions[i]` index-aligned with add order | OK (selftest ctrl + twist callbacks fired) |
| 6 | `rcl_take(...)` signature; `rmw_get_zero_initialized_message_info()` | OK |
| 7 | Loaned-message API present (`rcl_publisher_can_loan_messages`, borrow/publish/return) | OK (compiled/linked) |
| 8 | `rcl_get_error_string()` -> `rcutils_error_string_t{.str}`; `rcl_reset_error()` | OK |
| 9 | rosidl C struct field names as used across JointState/Imu/Image/TF/Twist/Clock/Time/Float64MultiArray | OK (`ros2 topic echo` showed correct JointState `name/position/velocity/effort`) |
| 10 | Sequence + string helpers (`rosidl_runtime_c__String__assign`, `__Sequence__init`, `<type>__init/fini`) | OK |
| 11 | QoS symbols (`rmw_qos_profile_default`, transient-local / reliable / keep-last) | OK |
| 12 | `/tf_static` transient-local latch delivers to late joiners | OK (design choice; publisher created without error) |
| 13 | Plain-CMake consumption via namespaced imported targets against an active Lyrical env | OK (`find_package(rcl)` etc. resolved; see `ros/urlab_ros_ws/CMakeLists.txt`) |
| 14 | Env activation via `pixi run` (non-interactive) / `AMENT_PREFIX_PATH` detection | OK (built via `pixi run --manifest-path C:\dev\urlab_ros2_env\pixi.toml`) |

### M1 result

`ros/urlab_ros_ws/build_and_test.ps1` ran fully green on Windows:

```
distro: lyrical
selftest: OK (ctrl + twist loopback verified)
reinit: OK
urlab_rcl_test: PASS
>>> Cross-process verify (ros2 topic echo)...
name:
- joint_a
- joint_b
- joint_c
position: [1.0, 2.0, 3.0]  velocity: [0.1, 0.2, 0.3]  effort: [10.0, 20.0, 30.0]
>>> Cross-process verify OK: JointState names received.
=== urlab_rcl_test: ALL CHECKS PASSED (Windows) ===
```

The cross-process echo proves real inter-process DDS works in-process on
Windows — the key de-risk gate for the whole ROS design.
