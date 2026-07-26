# ROS 2 link facts (fill during M1)

Template. The user fills these blanks from an actual green `urlab_rcl_test` build
(see `docs/ros_workspace_setup.md`, Part 3). The Unreal build wiring consumes
this file; it is authoritative over any provisional list guessed elsewhere.

Fill the **Windows** table first (Windows is the primary M1 platform); Linux is
secondary. The default Windows install is Pixi/Conda (prefix.dev + RoboStack), so
the library/DLL names come from your Pixi environment prefix
(`%CONDA_PREFIX%\Library\{bin,lib,include}`), not a system path — record the
names exactly as found there.

---

## Distro / version

| Field | Value |
|---|---|
| Distro | Lyrical Luth (codename `lyrical`) |
| Exact patch version (`ros2 doctor` / release tag) | `____` |
| Install method (Pixi/RoboStack vs apt) | `____` |
| RMW implementation in use (`echo $RMW_IMPLEMENTATION`, default) | `____` |

---

## Windows (primary)

### Link libraries (`.lib` basenames CMake actually linked)

| Purpose | Provisional name | As-found `.lib` |
|---|---|---|
| rcl | `rcl` | `____` |
| rcutils | `rcutils` | `____` |
| rmw | `rmw` | `____` |
| rmw implementation | `rmw_implementation` / `rmw_fastrtps_cpp` | `____` |
| rosidl runtime | `rosidl_runtime_c` | `____` |
| rosidl typesupport | `rosidl_typesupport_c` | `____` |
| builtin_interfaces (ts + gen) | `builtin_interfaces__rosidl_typesupport_c`, `builtin_interfaces__rosidl_generator_c` | `____` |
| std_msgs (ts + gen) | `std_msgs__rosidl_typesupport_c`, `std_msgs__rosidl_generator_c` | `____` |
| geometry_msgs (ts + gen) | `geometry_msgs__rosidl_typesupport_c`, `geometry_msgs__rosidl_generator_c` | `____` |
| sensor_msgs (ts + gen) | `sensor_msgs__rosidl_typesupport_c`, `sensor_msgs__rosidl_generator_c` | `____` |
| tf2_msgs (ts + gen) | `tf2_msgs__rosidl_typesupport_c`, `tf2_msgs__rosidl_generator_c` | `____` |
| rosgraph_msgs (ts + gen) | `rosgraph_msgs__rosidl_typesupport_c`, `rosgraph_msgs__rosidl_generator_c` | `____` |
| Any others the link line showed | | `____` |

### Runtime DLL cluster (from `dumpbin /dependents`, transitively)

Paste the ROS/DDS DLLs `urlab_rcl_test.exe` and its dependencies pull in:

```
____
```

(Expected to include: `rcl.dll`, `rcutils.dll`, `rmw.dll`, `rmw_fastrtps_cpp.dll`,
`fastrtps.dll`/`fastdds.dll`, `fastcdr.dll`, `rosidl_runtime_c.dll`,
`rosidl_typesupport_c.dll`, the message-package DLLs, plus their deps.)

### Include layout

| Field | Value |
|---|---|
| Include root | `%CONDA_PREFIX%\Library\include` (Pixi) — or `____` |
| Layout | flat / per-package nested (`include/<pkg>/<pkg>/msg/...`) — circle one: `____` |
| Lib dir | `%CONDA_PREFIX%\Library\lib` (Pixi) — or `____` |
| Bin dir (DLLs) | `%CONDA_PREFIX%\Library\bin` (Pixi) — or `____` |

---

## Linux (secondary)

### Link libraries (`.so` unversioned symlink basenames)

| Purpose | Provisional name | As-found `.so` |
|---|---|---|
| rcl | `librcl.so` | `____` |
| rcutils | `librcutils.so` | `____` |
| rmw | `librmw.so` | `____` |
| rmw implementation | `librmw_implementation.so` | `____` |
| rosidl runtime | `librosidl_runtime_c.so` | `____` |
| rosidl typesupport | `librosidl_typesupport_c.so` | `____` |
| per-package ts + gen | `lib<pkg>__rosidl_typesupport_c.so`, `lib<pkg>__rosidl_generator_c.so` | `____` |

### Runtime cluster (from `ldd build/urlab_rcl_test`)

```
____
```

(Expected: `librcl.so`, `librmw*.so`, `librosidl_runtime_c.so`,
`libfastrtps.so`/`libfastdds.so`, `libfastcdr.so`, message-package `.so`s.)

### Include layout

| Field | Value |
|---|---|
| Include root | `$CONDA_PREFIX/include` (Pixi) or `/opt/ros/lyrical/include` (apt) — or `____` |
| Layout | flat / per-package nested — circle one: `____` |
| Lib dir | `$CONDA_PREFIX/lib` (Pixi) or `/opt/ros/lyrical/lib` (apt) — or `____` |

---

## rcl / rosidl API assumptions to confirm (ROS 2 Lyrical)

`UrlabRclCore.cpp` was written against the ROS 2 C API (`rcl` / `rmw` /
`rosidl_runtime_c`) without a ROS tree present in the repo. The rcl C API is
stable across distros, so these should hold on Lyrical, but each item below is a
best-knowledge assumption; confirm each against your actual Lyrical install
during the M1 build (a clean compile confirms most of them automatically). Mark
`OK` or record the correction.

| # | Assumption | Status |
|---|---|---|
| 1 | Header paths: `rcl/rcl.h`, `rcl/error_handling.h`, `rcl/subscription.h`, `rcl/wait.h`, `rmw/qos_profiles.h`, `rosidl_runtime_c/message_type_support_struct.h`, `rosidl_runtime_c/string_functions.h`, `rosidl_runtime_c/primitives_sequence_functions.h`, and message headers `<pkg>/msg/<snake_case>.h`. | `____` |
| 2 | `rcl_init_options_set_domain_id(rcl_init_options_t*, size_t)` exists and takes a `size_t`; `RCL_DEFAULT_DOMAIN_ID` selects the `ROS_DOMAIN_ID` env behaviour. | `____` |
| 3 | `ROSIDL_GET_MSG_TYPE_SUPPORT(pkg, msg, Type)` is available after including the message header and resolves once `<pkg>__rosidl_typesupport_c` is linked. | `____` |
| 4 | `rcl_wait_set_init` signature is `(ws, n_subscriptions, n_guard_conditions, n_timers, n_clients, n_services, n_events, context*, allocator)` — seven count arguments in that order. | `____` |
| 5 | After adding subscriptions in order, `wait_set.subscriptions[i]` is index-aligned with the i-th `rcl_wait_set_add_subscription` call (non-null when ready). | `____` |
| 6 | `rcl_take(subscription*, void* msg, rmw_message_info_t*, rmw_subscription_allocation_t*)` signature; `rmw_get_zero_initialized_message_info()` exists. | `____` |
| 7 | Loaned-message API present in rcl: `rcl_publisher_can_loan_messages`, `rcl_borrow_loaned_message(pub*, type_support*, void**)`, `rcl_publish_loaned_message`, `rcl_return_loaned_message_from_publisher`. | `____` |
| 8 | Error string: `rcl_get_error_string()` returns `rcutils_error_string_t` with a `.str` member; `rcl_reset_error()` clears it. | `____` |
| 9 | rosidl C struct field names as used — `sensor_msgs__msg__JointState{header, name, position, velocity, effort}`; `sensor_msgs__msg__Imu{header, orientation(x/y/z/w), orientation_covariance[9], angular_velocity, angular_velocity_covariance[9], linear_acceleration, linear_acceleration_covariance[9]}`; `sensor_msgs__msg__Image{header, height, width, encoding, is_bigendian, step, data}`; `tf2_msgs__msg__TFMessage{transforms}`; `geometry_msgs__msg__TransformStamped{header, child_frame_id, transform{translation, rotation}}`; `geometry_msgs__msg__TwistStamped{header, twist{linear, angular}}`; `rosgraph_msgs__msg__Clock{clock}`; `builtin_interfaces__msg__Time{sec:int32, nanosec:uint32}`; `std_msgs__msg__Float64MultiArray{layout, data}`; `geometry_msgs__msg__Twist{linear, angular}`. | `____` |
| 10 | Sequence helpers: `rosidl_runtime_c__String__Sequence__init`, `rosidl_runtime_c__String__assign`, `rosidl_runtime_c__double__Sequence__init`, `rosidl_runtime_c__uint8__Sequence__init/fini`; message `<type>__init` / `<type>__fini`. | `____` |
| 11 | QoS symbols: `rmw_qos_profile_default`, `RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL`, `RMW_QOS_POLICY_RELIABILITY_RELIABLE`, `RMW_QOS_POLICY_HISTORY_KEEP_LAST`. | `____` |
| 12 | `/tf_static` QoS choice (transient-local, keep-last depth 1) delivers latched static transforms to late joiners as intended. Design choice, not an API fact. | `____` |
| 13 | Plain-CMake consumption via `find_package(ament_cmake REQUIRED)` + `ament_target_dependencies(...)` works against an active Lyrical env (Pixi/Conda or apt), wiring includes + libs regardless of layout. | `____` |
| 14 | Environment activation: Windows via `pixi shell` (Pixi/Conda, default), Linux via `pixi shell` or `source /opt/ros/lyrical/setup.bash` (apt); the build scripts detect an active env via `AMENT_PREFIX_PATH`, overridable with `URLAB_ROS2_SETUP`. | `____` |
