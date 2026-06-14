# API Reference

`urlab_client` is the typed Python client for URLab. It opens a ZMQ
session against a running editor (or PIE / standalone) and exposes the
simulation, scene, and runtime through typed objects.

This page is the reference. For the guided flow, read
[Quickstart](quickstart.md) first. For the wire ops underneath each
method, see [Protocol Reference](../reference/protocol.md).

## Connection

```python
from urlab_client import URLabClient

client = URLabClient("tcp://localhost", step_mode="direct")
client.discover()
...
client.close()
```

`URLabClient` is a context manager; `with URLabClient(...) as client:`
closes the transport on exit. On the same host, prefer
`transport="shm"` (see [Transports](#transports)).

### `URLabClient(...)`

```python
URLabClient(
    address: str = "tcp://localhost",
    *,
    step_mode: str | StepMode = "auto",
    step_port: int = 5559,
    state_port: int = 5555,
    ctrl_port: int = 5556,
    info_port: int = 5557,
    mujoco_version_check: bool = True,
    local_model: bool = True,
    rcv_timeout_ms: int = 5000,
    auto_promote_step_mode: bool = True,
    transport: str | Transport = "zmq",
    shm_dir: str | None = None,
)
```

- `address` - TCP/IPC base address; ports below are appended.
- `step_mode` - `"auto"` (default), `"direct"`, `"puppet"`, `"live"`,
  or a `StepMode` member.
- `step_port` (5559) - REQ/REP RPC.
- `state_port` (5555) - PUB/SUB state stream (used in `live`).
- `transport` - `"zmq"` (default) or `"shm"`.
- `local_model` - load the handshake's compiled model into a local
  `mujoco.MjModel`. Required for `puppet` and client-side MPC / IK.
- `mujoco_version_check` - raise `URLabVersionMismatch` when server and
  client MuJoCo builds differ.
- `auto_promote_step_mode` - when `True`, `discover()` follows up with a
  `set_mode` RPC if the constructor named `direct` or `puppet`.
- `rcv_timeout_ms` - default RPC recv timeout; individual ops override.

### `client.discover(observations="standard")`

Sends the handshake, loads the model, and builds the articulation /
entity / camera wrappers. Safe to call multiple times.

`observations` selects how much per-step data the server serialises:

- `"minimal"` - qpos, qvel only.
- `"standard"` - qpos, qvel, ctrl, act, sensors. Default.
- `"full"` - adds per-body `xpos` / `xquat` and actuator forces.

After `client.sim.start()` returns `READY`, the embedded handshake is
auto-absorbed; call `discover()` again only to change `observations`.

### `client.close()`

Stops streaming threads, reverts the auto-promoted step mode, closes
the transport. Idempotent.

## Transports

### ZMQ (default)

```python
client = URLabClient("tcp://localhost", transport="zmq")
```

TCP REQ/REP on 5559 for RPCs, plus PUB/SUB on 5555 (state) and 5558+
(per camera). Cross-host capable; the only path that works between
machines.

### SHM (same host)

```python
client = URLabClient("tcp://localhost", transport="shm")
```

Shared-memory ring buffers with kernel-event signalling. Lower jitter,
especially on camera streams. The `shm_session_dir` field in the
handshake tells the bridge which directory under
`<ProjectSavedDir>/URLabShm/` carries the rings.

The handshake itself and any oversize RPC fall back to the ZMQ REQ
socket transparently, then resume SHM for the next op.

## Session state

Read-only attributes set by `discover()`:

| Attribute | Type | Notes |
|---|---|---|
| `session_id` | `str` | RPC session GUID |
| `urlab_version` | `str` | server plugin version |
| `mujoco_version` | `str` | server-side MuJoCo build string |
| `manager_present` | `bool` | `False` pre-PIE, `True` once PIE is live |
| `model` | `mujoco.MjModel \| None` | local model (when `local_model=True`) |
| `data` | `mujoco.MjData \| None` | local sim state (mirrors server each step) |
| `articulations` | `Dict[str, URLabArticulation]` | keyed by prefix |
| `articulations_by_id` | `Dict[str, URLabArticulation]` | subset with non-empty `actor_id` |
| `entities` | `Dict[str, URLabEntity]` | every dynamic body (articulations included) |
| `global_cameras` | `Dict[str, URLabCameraView]` | scene-level cameras |
| `recording` / `replay` | API objects | see below |
| `step_count` | `int` | steps since `discover()` |
| `sim_time` | `float` | server sim clock |

Time-aligned clocks (populated each step): `sim_time_sec`,
`sim_time_nsec`, `wall_time_sec`, `wall_time_nsec`, `recv_wall_time_ns`.

!!! note "Locking"
    State read from non-step threads must hold `client._data_lock` (a
    reentrant lock). The `live`-mode SUB callback writes through it.

## Stepping the sim

```python
reply = client.step(n_steps=1, *, include_cameras=False, observations="standard")
reply = client.reset(keyframe_name=None, seed=None, per_articulation_qpos=None)
```

| Mode | What advances physics | Use |
|---|---|---|
| `direct` | UE steps `n_steps` times per RPC against your buffered ctrl. | Deterministic rollouts, data collection. |
| `puppet` | Client calls `mj_step` locally, pushes qpos/qvel to UE for rendering. | MJX, MPC, custom integrators. |
| `live` | UE physics runs autonomously; the RPC stamps ctrl and reads state. | Low-latency loops, real-time playback. |
| `auto` | Defaults to `live`. | When in doubt. |

In `puppet`, `n_steps=0` skips the local `mj_step` and just pushes
whatever is in `client.data`.

### Reset

```python
client.reset()                                       # default
client.reset(keyframe_name="home")                   # to a keyframe
client.reset(seed=42)                                # seeded reset
client.reset(per_articulation_qpos={"robot": {...}}) # per-joint override
```

Returns the same payload shape as `step()` and mirrors qpos/qvel back
into `client.data`. The keyframe name is resolved server-side and
applied with MuJoCo's keyframe reset. The seed is recorded URLab-side
for reproducibility bookkeeping (see
[Running Policies](policies.md#seeding)).

## Namespaces

The client groups RPCs by topic.

!!! note "`manager_present` invariant"
    `client.scene.*`, `client.outliner.*`, `client.debug.*`, and
    `client.viewport.*` ops are valid editor-time.
    `client.runtime.*` requires PIE to be live (`manager_present == True`).
    `client.sim.start()` is the bridge.

### `client.scene` - editor-time authoring

```python
client.scene.create_level(name, *, force_overwrite=False) -> None
client.scene.load_level(name_or_path) -> None
client.scene.save_level() -> None
client.scene.import_xml(xml_path, *, force_reimport=False) -> URLabBlueprint
client.scene.spawn_actor(blueprint, actor_id, *, location=..., rotation_quat=..., rotation_euler=..., scale=...) -> URLabSpawnHandle
client.scene.spawn_light(kind="directional", actor_id="", *, location=..., rotation_euler=..., intensity=5000.0, color=...) -> URLabLightHandle
client.scene.destroy_actor(target, *, by_name=False) -> None
client.scene.set_actor_transform(target, *, by_name=False, location=None, rotation_quat=None, rotation_euler=None) -> None
client.scene.destroy_asset(asset_path) -> bool
client.scene.current_level() -> str
client.scene.ensure_manager() -> bool
client.scene.snapshot() -> SceneSnapshot
client.scene.duplicate_actor(target, new_actor_id, *, by_name=False, location=None) -> URLabSpawnHandle
client.scene.actor_hierarchy(target, *, by_name=False) -> ActorHierarchyNode
client.scene.spawn_grid(blueprint, base_actor_id, count_x, count_y, *, spacing=(1,1,0), origin=(0,0,0), rotation_quat=None, rotation_euler=None, scale=(1,1,1)) -> Dict[str, URLabSpawnHandle]
client.scene.apply_scene(level_name, assets, *, save=True) -> Dict[str, URLabSpawnHandle]
```

`import_xml` returns a [`URLabBlueprint`](#urlabblueprint) that plugs
straight into `spawn_actor(blueprint=...)` (or pass a raw class-path
string). `import_xml(force_reimport=True)` destroys the existing
generated Blueprint first.

`apply_scene` loads or creates the level, imports each unique XML,
spawns one actor per [`URLabAsset`](#urlabasset), and saves. It is
idempotent per `actor_id`: a repeat call updates the existing actor's
transform in place. `spawn_grid` is the batched form; cells go at
`origin + (i*spacing.x, j*spacing.y, 0)`, capped at 1024.

`snapshot()` returns a heavier `SceneSnapshot` than
`outliner.list_actors()`, carrying per-actor articulation metadata.

`target` arguments default to `actor_id` lookup; pass `by_name=True`
to look up by UE actor name.

### `client.sim` - PIE lifecycle

```python
client.sim.start(level_path=None, *, timeout_s=30.0, raise_on_failure=True) -> PIEStartResult
client.sim.stop() -> None
client.sim.status() -> PIEStatus
```

`start()` raises `URLabPIEError` on failure by default; pass
`raise_on_failure=False` to receive a [`PIEStartResult`](#piestartresult)
in any state. On `READY`, `result.handshake_payload` is already
absorbed. With no `level_path` and a healthy session, `start()`
short-circuits to `READY` without restarting PIE.

### `client.runtime` - live physics mutators

```python
client.runtime.set_paused(paused: bool) -> bool
client.runtime.set_sim_speed(percent: float) -> float          # 100 = realtime
client.runtime.set_control_source(source, *, articulation=None) -> None  # "zmq" | "ui"
client.runtime.set_twist(articulation, *, linear=(0,0,0), angular=(0,0,0)) -> None
client.runtime.set_qpos(target, qpos, *, by_name=False) -> None
client.runtime.set_mocap_pose(body, *, pos=None, quat=None) -> MocapPose
client.runtime.read_mocap_pose(body) -> MocapPose
client.runtime.get_contacts(*, body1=None, body2=None, geom1=None, geom2=None, max_contacts=64) -> ContactsResult
client.runtime.list_keyframes() -> List[KeyframeInfo]
client.runtime.set_sim_options(**opts) -> SimOptions           # timestep, gravity, ...
client.runtime.set_mode(mode) -> StepMode
client.runtime.forward() -> dict                               # mj_forward, no integration
```

!!! warning "Gains are not on runtime"
    There is no `client.runtime.set_gains(...)`. PD gains are set
    through the articulation's controller:
    `art.controller.set_gains(...)`. See [Controllers](#controllers).

`set_qpos` mirrors the write back into `client.data.qpos`.
`set_sim_options` returns the resulting [`SimOptions`](#simoptions) and
mirrors them into `client.model.opt`. `set_mocap_pose` /
`read_mocap_pose` operate on the compiled MJ body name and error
`not_mocap_body` if the body is not flagged `mocap="true"`.
`get_contacts` snapshots active MuJoCo contacts, filters AND-match
exact names, and caps at `max_contacts` with a `truncated` flag.
`forward()` runs `mj_forward` (kinematics + dynamics, no integration)
so you can read consistent derived state after writing qpos/qvel.

### `client.outliner` - world introspection

```python
client.outliner.list_actors() -> List[ActorInfo]
client.outliner.list_blueprints() -> List[BlueprintInfo]
client.outliner.find_actors(*, class_filter=None, tag=None, name_prefix=None, in_pie=False) -> List[ActorInfo]
client.outliner.get_actor_bounds(target, *, by_name=False, components_only=False) -> ActorBounds
client.outliner.select_actor(target, *, by_name=False) -> None
client.outliner.add_quick_convert(target, *, by_name=False, static=False, complex_mesh=False, coacd_threshold=0.05, driven_by_unreal=False, friction=(1, 1, 1)) -> None
client.outliner.remove_quick_convert(target, *, by_name=False) -> None
```

`find_actors()` AND-matches across set parameters. `get_actor_bounds()`
returns an AABB in MJ metres. `add_quick_convert` turns a static-mesh
actor into a MuJoCo collidable.

### `client.debug` - debug visualisation

```python
client.debug.draw_marker(location, color, *, ttl=0.0, label=None, tag=None) -> None
client.debug.draw_line(from_, to, color, *, ttl=0.0, thickness=1.0, tag=None) -> None
client.debug.draw_box(center, half_extents, color, *, rotation_quat=None, ttl=0.0, tag=None) -> None
client.debug.draw_arrow(from_, to, color, *, ttl=0.0, thickness=1.0, arrow_size=None, tag=None) -> None
client.debug.draw_axes(location, *, rotation_quat=None, rotation_euler=None, scale=0.2, ttl=0.0, tag=None) -> None
client.debug.clear_markers(*, tag=None) -> None
client.debug.set_overlay_text(text, *, anchor="top_left") -> None
```

Works in editor and PIE (PIE world wins if running). Positions are MJ
metres; colors are `[r, g, b]` floats in `[0, 1]`. `ttl` is seconds:
`0` = single frame, positive = expires after that many seconds, `-1` =
persistent until `clear_markers()`. Tagged clearing is not honoured;
the plugin always does a full flush.

### `client.viewport` - perspective viewport control

```python
client.viewport.set_camera(location, *, rotation_quat=None, rotation_euler=None, fov=None) -> CameraPose
client.viewport.get_camera() -> CameraPose
client.viewport.frame_actor(target, *, by_name=False) -> CameraPose
client.viewport.set_mode(mode) -> str
client.viewport.track_actor(target, *, by_name=False, offset=None, smoothing=0.0) -> str
client.viewport.untrack() -> bool
```

Editor-only; the PIE play window has its own camera. `set_mode` accepts
`"lit"`, `"unlit"`, `"wireframe"`, `"collision"`, `"reflections"`.
`track_actor` lerps the camera toward `actor + rotation * offset`
(default 2 m behind, 1 m above); `smoothing` in `[0, 1)`.

### `client.recording`

```python
client.recording.start(name=None, max_duration_s=None) -> RecordingHandle
client.recording.stop() -> RecordingSummary
client.recording.save(path=None) -> Path
client.recording.clear() -> None
```

`save()` returns an absolute `pathlib.Path` to a `.json` file. A bare
filename resolves under `<Project>/Saved/URLab/Replays/`; an absolute
path passes through. Properties: `is_active`, `frame_count`,
`sim_duration`, `last_saved_path`.

### `client.replay`

```python
client.replay.load(path) -> ReplaySession
client.replay.list_sessions() -> List[str]
client.replay.set_active(name) -> None
client.replay.start() -> ReplayStatus
client.replay.stop() -> None
client.replay.play(path_or_name, *, loop=False) -> ReplaySession
```

Replay requires `direct` or `puppet`. Property:
`active_session: Optional[str]`.

## Articulations and entities

Every dynamic body lives in `client.entities`. Articulations (anything
imported from MJCF as a multi-jointed robot) also live in
`client.articulations`, keyed by **prefix** (the MJCF model name).

`URLabArticulation` subclasses `URLabEntity`. `URLabEntity` carries
root-link state and the external-wrench buffer; `URLabArticulation`
adds the per-kind handle dicts and per-DoF state.

### Entities

A scene often contains dynamic objects that are not articulations:
boxes the robot pushes, free-jointed pallets, kinematic heightfields.
They stream alongside articulation state so a policy sees the whole
world per step, and they surface as bare `URLabEntity` instances.

```python
client.discover()

pallet = client.entities["pallet"]   # URLabEntity
pallet.root_pos_w                    # (3,) world xpos
pallet.root_quat_w                   # (4,) MuJoCo wxyz
pallet.has_free_base                 # True for free-jointed bodies
pallet.free_joint                    # "pallet_free" or None
pallet.qpos_offset                   # global qpos start (None if no free joint)
pallet.qvel_offset
```

Two kinds are bucketed at handshake:

| Kind | `root_pos_w` / `root_quat_w` | `qpos_offset` / `qvel_offset` |
|---|---|---|
| Free-jointed (single free root) | always (from local mirror) | int into `client.data` (7 / 6 wide) |
| Kinematic dynamic (no free joint) | always | `None` |

`root_pos_w` / `root_quat_w` always populate: the bridge writes the
wire-shipped `xpos` / `xquat` into the local model mirror every step.
To read raw `qpos` / `qvel` for free-jointed bodies, slice
`client.data` with the offsets:

```python
import numpy as np
qpos = np.asarray(client.data.qpos[pallet.qpos_offset : pallet.qpos_offset + 7])
```

### Common state (both classes)

| Attribute | Shape | Frame |
|---|---|---|
| `root_pos_w` | `(3,)` | world |
| `root_quat_w` | `(4,)` | MuJoCo `(w, x, y, z)` |
| `root_quat_xyzw` | `(4,)` | SciPy/ROS `(x, y, z, w)` |
| `root_lin_vel_w` | `(3,)` | world |
| `root_ang_vel_w` | `(3,)` | world (articulation only) |
| `root_ang_vel_b` | `(3,)` | body (articulation only, MuJoCo native) |
| `root_lin_vel_b` | `(3,)` | body (derived) |
| `projected_gravity_b` | `(3,)` | body (derived) |

`apply_xfrc(body=..., force=..., torque=...)` (articulation) or
`apply_xfrc(force=..., torque=...)` (entity) buffers a 6-DOF wrench on
a named body. The buffer clears after the next `step()`.

!!! note
    `apply_xfrc()` is inert in `puppet` mode; the bridge overwrites
    `d->xfrc_applied` from the local model each step.

### Articulation surface

Per-kind dicts, each keyed by short unprefixed name:

```python
art.actuators: Dict[str, Actuator]
art.joints:    Dict[str, Joint]
art.sensors:   Dict[str, Sensor]
art.bodies:    Dict[str, Body]
art.cameras:   Dict[str, URLabCameraView]
```

Identity:

```python
art.prefix          # MJCF model name, e.g. "robot"
art.actor_id        # bridge stable id; "" if not assigned
art.body_id         # root body's global MuJoCo id
art.has_free_base   # True if root joint is mjJNT_FREE
art.free_joint      # name of the free joint (or None)
art.control_mode    # ControlMode.UE_CONTROLLER | ControlMode.RAW
art.controller      # URLabController | None  (None when RAW)
```

Flat per-step arrays:

```python
art.ctrl_array          # (n_actuators,)  user setpoint buffer
art.last_applied_ctrl   # (n_actuators,)  read-only echo of UE-applied ctrl
art.act_array           # (n_actuators,)  activation state for stateful actuators
art.qpos_array          # dense per-articulation qpos
art.qvel_array          # dense per-articulation qvel
art.twist_linear        # (3,)  ROS-Twist linear
art.twist_angular       # (3,)  ROS-Twist angular
art.dof_qpos            # joint qpos excluding free-base root
art.dof_qvel            # joint qvel excluding free-base root
```

Getters / setters:

```python
art.set_ctrl({"shoulder": 0.5})          # partial; missing keys keep last value
art.get_ctrl() -> Dict[str, float]
art.get_qpos() -> Dict[str, float]        # first component per joint
art.get_sensors() -> Dict[str, np.ndarray]
```

Name resolution (handles original-XML names renamed by UE):

```python
art.resolve_joint("shoulder_joint")     # -> "shoulder" or None
art.resolve_actuator("shoulder")        # -> live key or None
```

## Per-kind handles

### `Actuator`

```python
a.name        # str
a.id          # global MuJoCo actuator id
a.type        # ActuatorType | None  (authored kind from handshake)
a.joint       # unprefixed joint name (if joint-driven)
a.ctrlrange   # (lo, hi) | None
a.forcerange  # (lo, hi) | None
a.kp          # float | None  (gainprm[0] if present)
a.kv          # float | None  (gainprm[1] if present)
a.force       # float | None  (per-step at observation level "full")

a.set_control(v: float)
a.value -> float
```

### `Joint`

```python
j.name              # str
j.id                # global MuJoCo joint id
j.jnt_type          # int (0=free, 1=ball, 2=slide, 3=hinge)
j.qpos_offset       # global offset in client.data.qpos
j.qpos_local_offset # local offset within art.qpos_array
j.qvel_offset / j.qvel_local_offset
j.range             # (lo, hi) | None
```

### `Sensor`

```python
s.name         # str
s.id           # global MuJoCo sensor id
s.dim          # output dimension
s.latest       # np.ndarray | None  (per-step)
```

### `Body`

```python
b.name    # str
b.id      # global MuJoCo body id
b.xpos    # (3,) | None  (per-step)
b.xquat   # (4,) | None  (per-step at observation level "full")
```

## Controllers

`art.controller` is `None` when the articulation shipped no controller
block (a raw-passthrough articulation). Otherwise it is a
`URLabController`, or its `URLabPDController` subclass for PD-tuned
articulations.

Control mode is per-articulation:

| Mode | Behaviour |
|---|---|
| `ue_controller` | Step ctrl is staged to each actuator's network value; UE's controller runs every physics sub-step against the fixed setpoint. The policy emits setpoints (position targets for PD), UE owns signal generation. |
| `raw` | Step ctrl goes straight to `d->ctrl`; UE's controller is bypassed. The policy emits raw actuator signals. |

The default follows what is wired in UE. Override before `step()`:

```python
art.control_mode = "raw"   # or "ue_controller"
```

In `ue_controller`, `n_steps` decimates: the inner loop sees fresh
`qpos` / `qvel` every sub-step against the same setpoint. In `raw`,
ctrl is held constant across the sub-steps.

### `URLabController`

```python
ctrl.articulation_prefix   # str
ctrl.kind                  # ControllerKind | str ("pd", ...)
ctrl.params                # live param dict
ctrl.schema                # validation schema

ctrl.configure(**kwargs) -> Dict[str, Any]   # validate + push
ctrl.refresh() -> Dict[str, Any]              # re-pull from server
```

`configure` validates against the schema and returns the
server-acknowledged params. A rejected payload comes back as
`controller_schema_violation`.

### `URLabPDController` (subclass)

Partial-patch: joints not mentioned keep their current value.
`set_defaults` rebinds every joint to the new default.

```python
ctrl.kp / ctrl.kv / ctrl.torque_limit    # live Mapping[str, float] views

ctrl.set_gains(kp=None, kv=None, torque_limit=None) -> dict
ctrl.set_defaults(kp=None, kv=None, torque_limit=None) -> dict
```

```python
art.controller.set_gains(
    kp={"waist": 300.0, "shoulder": 280.0},
    kv={"waist": 20.0,  "shoulder": 18.0},
    torque_limit={"waist": 35.0, "shoulder": 45.0},
)
art.controller.set_defaults(kp=100, kv=5, torque_limit=200)
```

Setters are safe in any step mode. Under `raw`, params are tracked
server-side but have no immediate effect until you flip back to
`ue_controller`.

## Cameras

A camera is a `URLabCameraView`. Articulation-local cameras live under
`art.cameras`; scene-level under `client.global_cameras`.

```python
cam.name           # str
cam.mode           # CameraMode (real / depth / semantic / instance)
cam.resolution     # (W, H)
cam.fovy           # vertical FOV in degrees
cam.dtype          # uint8 (real / segm) or float32 (depth)
cam.latest_frame   # np.ndarray | None
cam.sim_time       # float | None
```

Frames populate on demand via `client.step(include_cameras=...)`, or
continuously from per-camera SUB threads in `live` mode.
`include_cameras` accepts `True` / `False`, or a mapping such as
`{"head": "sync"}` (block for a fresh frame) or `{"head": "latest"}`
(ship the cached frame).

| Mode | Shape | Channels |
|---|---|---|
| `real` | `(H, W, 4)` | RGBA |
| `depth` | `(H, W)` | float32 metres |
| `semantic` / `instance` | `(H, W, 4)` | BGRA per-class / per-instance tint |

## Scene composition helpers

```python
from urlab_client import URLabAsset, URLabBlueprint, URLabSpawnHandle, URLabLightHandle
```

### `URLabAsset`

```python
@dataclass
class URLabAsset:
    actor_id: str
    xml: str                                         # absolute MJCF path
    location: Sequence[float] = (0, 0, 0)
    rotation_quat: Optional[Sequence[float]] = None  # (x, y, z, w)
    rotation_euler: Optional[Sequence[float]] = None # (roll, pitch, yaw) deg
    scale: Sequence[float] = (1, 1, 1)
```

### `URLabBlueprint`

Returned by `import_xml`. Pass into `spawn_actor(blueprint=...)`.

```python
@dataclass
class URLabBlueprint:
    class_path: str       # "/Game/MJ/Robot.Robot_C"
    short_name: str       # MJCF model name
    imported_now: bool    # True if this call triggered a fresh import
```

### `URLabSpawnHandle`

```python
@dataclass
class URLabSpawnHandle:
    actor_id: str
    actor_name: str
    actor_path: str
    blueprint_class_path: str
    location: tuple = (0, 0, 0)
    rotation_quat: tuple = (0, 0, 0, 1)
    was_existing: bool = False        # True if an existing actor was updated
    requires_pie_restart: bool = False

    def runtime(self, client) -> URLabArticulation | None: ...
```

## Typed return objects

```python
from urlab_client import (
    PIEStartResult, PIEStatus, PIEState,
    SimOptions, ActorInfo, BlueprintInfo,
    RecordingHandle, RecordingSummary,
    ReplaySession, ReplayStatus,
)
```

### `PIEStartResult`

```python
@dataclass
class PIEStartResult:
    state: PIEState                     # READY | COMPILE_FAILED | TIMEOUT
    compile_error: str = ""
    handshake_payload: dict | None = None

    @property
    def is_ready(self) -> bool: ...     # state == PIEState.READY
```

`PIEState` is `READY`, `COMPILE_FAILED`, `TIMEOUT`, `OFF`, `COMPILING`.

### `SimOptions`

Typed mirror of the `mjOption` fields URLab exposes. Every field is
`Optional`; only fields the server echoed for the call are non-`None`.
Values reflect the server's live `opt` after the patch.

```python
@dataclass
class SimOptions:
    timestep: float | None = None                          # seconds
    gravity: tuple[float, float, float] | None = None      # m/s^2
    wind: tuple[float, float, float] | None = None
    magnetic: tuple[float, float, float] | None = None     # gauss
    density: float | None = None
    viscosity: float | None = None
    impratio: float | None = None
    tolerance: float | None = None
    iterations: int | None = None
    ls_iterations: int | None = None
    integrator: str | None = None        # euler | rk4 | implicit | implicitfast
    cone: str | None = None              # pyramidal | elliptic
    solver: str | None = None            # pgs | cg | newton
    noslip_iterations: int | None = None
    noslip_tolerance: float | None = None
    ccd_iterations: int | None = None
    ccd_tolerance: float | None = None
    enable_multiccd: bool | None = None
    enable_sleep: bool | None = None
    sleep_tolerance: float | None = None
```

### `ActorInfo`

```python
@dataclass
class ActorInfo:
    name: str
    actor_class: str
    actor_id: str                               # "" if none
    is_articulation: bool
    has_quick_convert: bool
    location: tuple[float, float, float]
    rotation_quat: tuple[float, float, float, float]
    is_static_mesh_actor: bool = False
    is_light: bool = False
    # Quick-Convert fields when has_quick_convert is True:
    static: bool | None = None
    complex_mesh: bool | None = None
    coacd_threshold: float | None = None
    driven_by_unreal: bool | None = None
    friction: tuple[float, float, float] | None = None
```

### `KeyframeInfo`

Element of `client.runtime.list_keyframes()`. Arrays are in compiled
order: `qpos[nq]`, `qvel[nv]`, `ctrl[nu]`, `mocap_pos[3*nmocap]`,
`mocap_quat[4*nmocap]`.

```python
@dataclass
class KeyframeInfo:
    name: str
    time: float
    qpos: list[float]
    qvel: list[float]
    ctrl: list[float]
    mocap_pos: list[float]
    mocap_quat: list[float]
```

### `Contact` / `ContactsResult`

`force` is the 6-vector `[fx, fy, fz, tx, ty, tz]` in the contact
frame; `dist` is negative when penetrating.

```python
@dataclass
class Contact:
    geom1: str; geom2: str; body1: str; body2: str
    pos: tuple[float, float, float]
    normal: tuple[float, float, float]
    dist: float
    force: tuple[float, float, float, float, float, float]

@dataclass
class ContactsResult:
    n_contacts: int
    truncated: bool
    contacts: list[Contact]
```

### `MocapPose`

`pos` in MJ metres; `quat` is wxyz (MuJoCo native).

```python
@dataclass
class MocapPose:
    body: str
    pos: tuple[float, float, float]
    quat: tuple[float, float, float, float]
```

### `RecordingSummary` / `ReplayStatus`

```python
@dataclass
class RecordingSummary:
    frame_count: int
    sim_duration_s: float

@dataclass
class ReplayStatus:
    active_session: str
    total_frames: int = 0
```

## Enums

All enums in `urlab_client.enums` are `str`-mixin enums; they
serialise as the wire string.

- `StepMode` - `AUTO`, `DIRECT`, `PUPPET`, `LIVE`.
- `ControlMode` - `UE_CONTROLLER`, `RAW`.
- `ActuatorType` - `MOTOR`, `POSITION`, `VELOCITY`, `INT_VELOCITY`,
  `DAMPER`, `CYLINDER`, `MUSCLE`, `ADHESION`, `DC_MOTOR`.
- `ControllerKind` - `PD`, `PASSTHROUGH`.
- `CameraMode` - `REAL`, `DEPTH`, `SEMANTIC`, `INSTANCE`.
- `CameraTiming` - `SYNC`, `LATEST`.
- `ObservationLevel` - `MINIMAL`, `STANDARD`, `FULL`.

## Errors

```python
from urlab_client import URLabRPCError, URLabVersionMismatch, URLabPIEError
```

- `URLabRPCError(code, message, *, op=None)` - the server returned an
  `error` reply. `code` is short and stable (for example
  `not_in_editor`, `unknown_op`, `unknown_articulation`,
  `missing_field`). See [Protocol Reference](../reference/protocol.md#errors)
  for the full set.
- `URLabVersionMismatch` - raised by `discover()` when client and
  server MuJoCo versions differ. Pass `mujoco_version_check=False` to
  downgrade to a warning.
- `URLabPIEError(*, code, message, state)` - raised by
  `client.sim.start()` when the editor failed to enter `Playing`. A
  subclass of `URLabRPCError`. `.state` is a `PIEState`;
  `.message` carries the compile log when relevant.

## See also

- [Quickstart](quickstart.md) - guided walkthrough.
- [Running Policies](policies.md) - pretrained policies and the gym env.
- [Protocol Reference](../reference/protocol.md) - wire ops and errors.
